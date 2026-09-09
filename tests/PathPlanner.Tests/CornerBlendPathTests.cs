using PathPlanner.Geometry;
using PathPlanner.Parsing;

namespace PathPlanner.Tests;

public sealed class CornerBlendPathTests
{
    [Fact]
    public void Create_RoundsCornerWithinLinearToleranceAndPreservesEndpoints()
    {
        var program = Parse(
            """
            X[0] Z[0]
            X[1] Z[0]
            X[1] Z[1]
            """);
        var constraints = LinearConstraints("X", "Z");
        var path = CornerBlendPath.Create(program, constraints, 0.05, 0);

        Assert.Equal([0.0, 0.0], path.Evaluate(0));
        Assert.Equal([1.0, 1.0], path.Evaluate(path.ProcessLength));

        var maximumDeviation = 0.0;
        for (var sample = 0; sample <= 500; sample++)
        {
            var point = path.Evaluate(path.ProcessLength * sample / 500);
            maximumDeviation = Math.Max(
                maximumDeviation,
                DistanceToOriginalCorner(point[0], point[1]));
        }

        Assert.InRange(maximumDeviation, 0.001, 0.0501);
    }

    [Fact]
    public void Create_UsesUnwrappedCAsExactMasterCoordinate()
    {
        var program = Parse(
            """
            X[0] C[30]
            X[1] C[45]
            X[0] C[90]
            """);
        var constraints = new[]
        {
            new AxisConstraints("X", AxisKind.Linear, 100, 1000, 10000),
            new AxisConstraints("C", AxisKind.Rotary, 1000, 10000, 100000)
        };
        var path = CornerBlendPath.Create(program, constraints, 0.1, 0.1);

        Assert.True(path.IsRotaryDriven);
        Assert.Equal(60, path.ProcessLength, 10);
        Assert.Equal(30, path.Evaluate(0)[1], 10);
        Assert.Equal(60, path.Evaluate(30)[1], 10);
        Assert.Equal(90, path.Evaluate(60)[1], 10);
    }

    private static CncProgram Parse(string text) =>
        new CncProgramParser().Parse(new StringReader(text));

    private static AxisConstraints[] LinearConstraints(params string[] names) =>
        names.Select(name =>
            new AxisConstraints(name, AxisKind.Linear, 100, 1000, 10000)).ToArray();

    private static double DistanceToOriginalCorner(double x, double z)
    {
        var firstSegment = Math.Sqrt(
            Math.Pow(Math.Clamp(x, 0, 1) - x, 2) + Math.Pow(z, 2));
        var secondSegment = Math.Sqrt(
            Math.Pow(1 - x, 2) + Math.Pow(Math.Clamp(z, 0, 1) - z, 2));
        return Math.Min(firstSegment, secondSegment);
    }
}
