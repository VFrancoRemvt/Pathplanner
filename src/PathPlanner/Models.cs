using System.Collections.ObjectModel;

namespace PathPlanner;

public enum AxisKind
{
    Linear,
    Rotary
}

public enum KinematicQuantity
{
    Velocity,
    Acceleration,
    Jerk
}

public enum ProcessSpeedKind
{
    RevolutionsPerMinute,
    MillimetresPerMinute
}

public sealed record AxisConstraints(
    string AxisName,
    AxisKind Kind,
    double MaxVelocity,
    double MaxAcceleration,
    double MaxJerk);

public sealed record TrajectoryConversionOptions
{
    public required double SamplingRateHz { get; init; }

    public double? RequestedRpm { get; init; }

    public double? RequestedFeedMmPerMinute { get; init; }

    public required double LinearToleranceMm { get; init; }

    public required double RotaryToleranceDegrees { get; init; }

    public required IReadOnlyList<AxisConstraints> AxisConstraints { get; init; }

    public string ProfileName { get; init; } = "PathPlanner trajectory";
}

public sealed record ConstraintDiagnostic(
    string AxisName,
    KinematicQuantity Quantity,
    double ObservedValue,
    double ConfiguredLimit,
    double Utilization);

public sealed record AxisKinematicSummary(
    string AxisName,
    double MaximumVelocity,
    double MaximumAcceleration,
    double MaximumJerk);

public sealed record ConversionReport(
    int NumberOfRows,
    double DurationSeconds,
    ProcessSpeedKind SpeedKind,
    double RequestedProcessSpeed,
    double AppliedProcessSpeed,
    bool WasReduced,
    ConstraintDiagnostic? LimitingConstraint,
    IReadOnlyList<AxisKinematicSummary> AxisSummaries);

public sealed class CncProgram
{
    private readonly double[,] _positions;
    private readonly ReadOnlyCollection<string> _axisNames;

    internal CncProgram(IReadOnlyList<string> axisNames, double[,] positions)
    {
        _axisNames = Array.AsReadOnly(axisNames.ToArray());
        _positions = positions;
    }

    public IReadOnlyList<string> AxisNames => _axisNames;

    public int PointCount => _positions.GetLength(0);

    public int AxisCount => _positions.GetLength(1);

    public double GetPosition(int pointIndex, int axisIndex) => _positions[pointIndex, axisIndex];

    public int GetAxisIndex(string axisName)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(axisName);

        for (var index = 0; index < _axisNames.Count; index++)
        {
            if (string.Equals(_axisNames[index], axisName, StringComparison.OrdinalIgnoreCase))
            {
                return index;
            }
        }

        return -1;
    }

    internal double[] GetPoint(int pointIndex)
    {
        var point = new double[AxisCount];
        for (var axis = 0; axis < AxisCount; axis++)
        {
            point[axis] = _positions[pointIndex, axis];
        }

        return point;
    }
}

public sealed class CncParseException : FormatException
{
    public CncParseException(int lineNumber, string reason)
        : base($"Línea {lineNumber}: {reason}")
    {
        LineNumber = lineNumber;
    }

    public int LineNumber { get; }
}

public sealed class TrajectoryPlanningException : InvalidOperationException
{
    public TrajectoryPlanningException(string message)
        : base(message)
    {
    }
}
