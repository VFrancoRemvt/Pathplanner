using PathPlanner.Geometry;
using PathPlanner.Parsing;
using PathPlanner.Planning;

namespace PathPlanner.Tests;

public sealed class ConstrainedTrajectoryPlannerTests
{
    [Fact]
    public void Plan_StartsAndEndsAtRestProfileEndpoints()
    {
        var program = new CncProgramParser().Parse(
            new StringReader("X[0]\nX[100]"));
        AxisConstraints[] constraints =
        [
            new("X", AxisKind.Linear, 20, 100, 10_000)
        ];
        var path = CornerBlendPath.Create(program, constraints, 0, 0);

        var plan = ConstrainedTrajectoryPlanner.Plan(
            path,
            program.AxisNames,
            constraints,
            samplingRateHz: 100,
            requestedProcessSpeed: 10);

        var last = plan.NumberOfRows - 1;
        Assert.Equal(0, plan.Samples[0, 0]);
        Assert.Equal(100, plan.Samples[last, 0], 12);

        var firstIncrement = plan.Samples[1, 0] - plan.Samples[0, 0];
        var middle = plan.NumberOfRows / 2;
        var cruiseIncrement = plan.Samples[middle + 1, 0] - plan.Samples[middle, 0];
        var lastIncrement = plan.Samples[last, 0] - plan.Samples[last - 1, 0];

        Assert.True(firstIncrement < cruiseIncrement / 10);
        Assert.True(lastIncrement < cruiseIncrement / 10);
        Assert.True(plan.AxisSummaries[0].MaximumVelocity <= 20);
        Assert.True(plan.AxisSummaries[0].MaximumAcceleration <= 100);
        Assert.True(plan.AxisSummaries[0].MaximumJerk <= 10_000);
    }
}
