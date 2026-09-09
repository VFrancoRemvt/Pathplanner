using System.Globalization;

namespace PathPlanner.Tests;

public sealed class TrajectoryConverterTests
{
    [Fact]
    public async Task ConvertAsync_WritesCurrentTraStHeaderAndAxisUnits()
    {
        const string input =
            """
            X[0] Z[0]
            X[5] Z[0]
            X[10] Z[1]
            """;
        var output = new StringWriter(CultureInfo.InvariantCulture);
        var options = new TrajectoryConversionOptions
        {
            SamplingRateHz = 50,
            RequestedFeedMmPerMinute = 60,
            LinearToleranceMm = 0.01,
            RotaryToleranceDegrees = 0,
            ProfileName = "Prueba XZ",
            AxisConstraints =
            [
                new("X", AxisKind.Linear, 10, 100, 10_000),
                new("Z", AxisKind.Linear, 10, 100, 10_000)
            ]
        };

        var report = await new TrajectoryConverter().ConvertAsync(
            new StringReader(input),
            output,
            options);
        var csv = output.ToString();

        Assert.StartsWith("# Trajectory Stream Data - Version 1.0", csv);
        Assert.Contains("# Profile: Prueba XZ", csv, StringComparison.Ordinal);
        Assert.Contains("# Sampling rate: 50Hz", csv, StringComparison.Ordinal);
        Assert.Contains("# Column 1: Position X [mm]", csv, StringComparison.Ordinal);
        Assert.Contains("# Column 2: Position Z [mm]", csv, StringComparison.Ordinal);
        Assert.Equal(report.NumberOfRows / 50.0, report.DurationSeconds, 12);
        Assert.Equal(ProcessSpeedKind.MillimetresPerMinute, report.SpeedKind);
    }

    [Fact]
    public async Task ConvertAsync_ReducesRpmAndReportsLimitingAxis()
    {
        const string input =
            """
            X[0] C[0]
            X[0.1] C[180]
            X[0] C[360]
            """;
        var output = new StringWriter(CultureInfo.InvariantCulture);
        var options = new TrajectoryConversionOptions
        {
            SamplingRateHz = 20,
            RequestedRpm = 60,
            LinearToleranceMm = 0.01,
            RotaryToleranceDegrees = 0.1,
            ProfileName = "Límite C",
            AxisConstraints =
            [
                new("X", AxisKind.Linear, 1_000, 1e9, 1e12),
                new("C", AxisKind.Rotary, 30, 1e9, 1e12)
            ]
        };

        var report = await new TrajectoryConverter().ConvertAsync(
            new StringReader(input),
            output,
            options);

        Assert.True(report.WasReduced);
        Assert.InRange(report.AppliedProcessSpeed, 0, 5.001);
        Assert.NotNull(report.LimitingConstraint);
        Assert.Equal("C", report.LimitingConstraint.AxisName);
        Assert.Equal(
            KinematicQuantity.Velocity,
            report.LimitingConstraint.Quantity);
        Assert.All(
            report.AxisSummaries,
            summary => Assert.True(double.IsFinite(summary.MaximumJerk)));
    }

    [Fact]
    public async Task ConvertAsync_RejectsMissingAxisLimits()
    {
        var options = new TrajectoryConversionOptions
        {
            SamplingRateHz = 100,
            RequestedFeedMmPerMinute = 100,
            LinearToleranceMm = 0,
            RotaryToleranceDegrees = 0,
            AxisConstraints =
            [
                new("X", AxisKind.Linear, 1, 1, 1)
            ]
        };

        var exception = await Assert.ThrowsAsync<ArgumentException>(
            () => new TrajectoryConverter().ConvertAsync(
                new StringReader("X[0] Z[0]\nX[1] Z[1]"),
                new StringWriter(),
                options));

        Assert.Contains("Faltan límites", exception.Message, StringComparison.Ordinal);
    }
}
