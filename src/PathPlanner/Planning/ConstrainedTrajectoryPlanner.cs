using PathPlanner.Geometry;

namespace PathPlanner.Planning;

internal static class ConstrainedTrajectoryPlanner
{
    private const int RampSearchIterations = 40;
    private const int SpeedSearchIterations = 48;
    private const double RelativeTolerance = 1e-6;

    internal static PlannedTrajectory Plan(
        CornerBlendPath path,
        IReadOnlyList<string> axisNames,
        IReadOnlyList<AxisConstraints> constraints,
        double samplingRateHz,
        double requestedProcessSpeed)
    {
        var period = 1.0 / samplingRateHz;
        if (TryPlanAtSpeed(
                path,
                axisNames,
                constraints,
                period,
                requestedProcessSpeed,
                out var requestedPlan,
                out _))
        {
            return requestedPlan! with { WasDynamicallyReduced = false };
        }

        var high = requestedProcessSpeed;
        var low = requestedProcessSpeed;
        PlannedTrajectory? best = null;
        KinematicCheck? lastFailedCheck = null;

        for (var reduction = 0; reduction < SpeedSearchIterations; reduction++)
        {
            low /= 2;
            if (TryPlanAtSpeed(
                    path,
                    axisNames,
                    constraints,
                    period,
                    low,
                    out best,
                    out var check))
            {
                break;
            }

            lastFailedCheck = check;
        }

        if (best is null)
        {
            var detail = lastFailedCheck?.Worst is null
                ? string.Empty
                : $" Última restricción: {lastFailedCheck.Worst.AxisName} " +
                  $"{lastFailedCheck.Worst.Quantity}.";
            throw new TrajectoryPlanningException(
                "No se encontró una velocidad positiva que cumpla los límites." + detail);
        }

        for (var iteration = 0; iteration < SpeedSearchIterations; iteration++)
        {
            var middle = (low + high) / 2;
            if (TryPlanAtSpeed(
                    path,
                    axisNames,
                    constraints,
                    period,
                    middle,
                    out var candidate,
                    out _))
            {
                low = middle;
                best = candidate;
            }
            else
            {
                high = middle;
            }

            if ((high - low) / Math.Max(low, double.Epsilon) <= RelativeTolerance)
            {
                break;
            }
        }

        return best! with { WasDynamicallyReduced = true };
    }

    private static bool TryPlanAtSpeed(
        CornerBlendPath path,
        IReadOnlyList<string> axisNames,
        IReadOnlyList<AxisConstraints> constraints,
        double period,
        double peakSpeed,
        out PlannedTrajectory? plan,
        out KinematicCheck? failedCheck)
    {
        plan = null;
        failedCheck = null;

        var maximumRampDuration = path.ProcessLength / peakSpeed;
        var minimumRampDuration = 4 * period;
        if (maximumRampDuration < minimumRampDuration)
        {
            return false;
        }

        var highRamp = Math.Floor(maximumRampDuration / period) * period;
        if (highRamp < minimumRampDuration)
        {
            return false;
        }

        var highCandidate = BuildCandidate(path, period, peakSpeed, highRamp);
        var highCheck = CheckKinematics(
            highCandidate.Samples,
            period,
            axisNames,
            constraints);
        if (!highCheck.IsFeasible)
        {
            failedCheck = highCheck;
            return false;
        }

        var lowCandidate = BuildCandidate(path, period, peakSpeed, minimumRampDuration);
        var lowCheck = CheckKinematics(
            lowCandidate.Samples,
            period,
            axisNames,
            constraints);
        if (lowCheck.IsFeasible)
        {
            plan = CreatePlan(lowCandidate, lowCheck, period);
            return true;
        }

        var lowRamp = minimumRampDuration;
        var selectedCandidate = highCandidate;
        var selectedCheck = highCheck;

        for (var iteration = 0; iteration < RampSearchIterations; iteration++)
        {
            var middleRamp = Math.Ceiling(
                ((lowRamp + highRamp) / 2) / period) * period;
            if (middleRamp >= highRamp || middleRamp <= lowRamp)
            {
                break;
            }

            var middleCandidate = BuildCandidate(
                path,
                period,
                peakSpeed,
                middleRamp);
            var middleCheck = CheckKinematics(
                middleCandidate.Samples,
                period,
                axisNames,
                constraints);

            if (middleCheck.IsFeasible)
            {
                highRamp = middleRamp;
                selectedCandidate = middleCandidate;
                selectedCheck = middleCheck;
            }
            else
            {
                lowRamp = middleRamp;
                failedCheck = middleCheck;
            }
        }

        plan = CreatePlan(selectedCandidate, selectedCheck, period);
        return true;
    }

    private static CandidateTrajectory BuildCandidate(
        CornerBlendPath path,
        double period,
        double peakSpeed,
        double rampDuration)
    {
        var rampDistance = peakSpeed * rampDuration;
        var cruiseDistance = Math.Max(0, path.ProcessLength - rampDistance);
        var cruiseDuration = cruiseDistance / peakSpeed;
        var continuousDuration = (2 * rampDuration) + cruiseDuration;
        var intervalCount = Math.Max(
            2,
            (int)Math.Ceiling(continuousDuration / period));
        var quantizedDuration = intervalCount * period;
        var timeScale = quantizedDuration / continuousDuration;
        var rows = intervalCount + 1;
        var samples = new double[rows, path.AxisCount];

        for (var row = 0; row < rows; row++)
        {
            var realTime = row * period;
            var profileTime = Math.Min(
                continuousDuration,
                realTime / timeScale);
            var processCoordinate = EvaluateProcessCoordinate(
                profileTime,
                path.ProcessLength,
                peakSpeed,
                rampDuration,
                cruiseDuration);
            var positions = path.Evaluate(processCoordinate);
            for (var axis = 0; axis < path.AxisCount; axis++)
            {
                samples[row, axis] = positions[axis];
            }
        }

        return new CandidateTrajectory(
            samples,
            peakSpeed / timeScale);
    }

    private static double EvaluateProcessCoordinate(
        double time,
        double length,
        double peakSpeed,
        double rampDuration,
        double cruiseDuration)
    {
        if (time <= rampDuration)
        {
            var normalized = time / rampDuration;
            return peakSpeed * rampDuration * IntegratedSmootherStep(normalized);
        }

        if (time <= rampDuration + cruiseDuration)
        {
            return (peakSpeed * rampDuration / 2) +
                (peakSpeed * (time - rampDuration));
        }

        var totalDuration = (2 * rampDuration) + cruiseDuration;
        var remainingRampTime = Math.Max(0, totalDuration - time);
        var normalizedRemaining = remainingRampTime / rampDuration;
        return length -
            (peakSpeed * rampDuration * IntegratedSmootherStep(normalizedRemaining));
    }

    private static double IntegratedSmootherStep(double value)
    {
        var clamped = Math.Clamp(value, 0, 1);
        var squared = clamped * clamped;
        var fourth = squared * squared;
        var fifth = fourth * clamped;
        var sixth = fifth * clamped;
        return (2.5 * fourth) - (3 * fifth) + sixth;
    }

    private static KinematicCheck CheckKinematics(
        double[,] samples,
        double period,
        IReadOnlyList<string> axisNames,
        IReadOnlyList<AxisConstraints> constraints)
    {
        var rows = samples.GetLength(0);
        var axes = samples.GetLength(1);
        var summaries = new AxisKinematicSummary[axes];
        ConstraintDiagnostic? worst = null;

        for (var axis = 0; axis < axes; axis++)
        {
            var velocity = new double[rows + 1];
            var acceleration = new double[rows + 2];
            var maximumVelocity = 0.0;
            var maximumAcceleration = 0.0;
            var maximumJerk = 0.0;

            for (var row = 1; row < rows; row++)
            {
                velocity[row] =
                    (samples[row, axis] - samples[row - 1, axis]) / period;
                maximumVelocity = Math.Max(maximumVelocity, Math.Abs(velocity[row]));
            }

            velocity[rows] = 0;
            for (var row = 1; row <= rows; row++)
            {
                acceleration[row] =
                    (velocity[row] - velocity[row - 1]) / period;
                maximumAcceleration = Math.Max(
                    maximumAcceleration,
                    Math.Abs(acceleration[row]));
            }

            acceleration[rows + 1] = 0;
            for (var row = 1; row <= rows + 1; row++)
            {
                var jerk = (acceleration[row] - acceleration[row - 1]) / period;
                maximumJerk = Math.Max(maximumJerk, Math.Abs(jerk));
            }

            summaries[axis] = new AxisKinematicSummary(
                axisNames[axis],
                maximumVelocity,
                maximumAcceleration,
                maximumJerk);

            var limit = constraints[axis];
            worst = SelectWorst(
                worst,
                axisNames[axis],
                KinematicQuantity.Velocity,
                maximumVelocity,
                limit.MaxVelocity);
            worst = SelectWorst(
                worst,
                axisNames[axis],
                KinematicQuantity.Acceleration,
                maximumAcceleration,
                limit.MaxAcceleration);
            worst = SelectWorst(
                worst,
                axisNames[axis],
                KinematicQuantity.Jerk,
                maximumJerk,
                limit.MaxJerk);
        }

        return new KinematicCheck(
            worst is null || worst.Utilization <= 1 + 1e-9,
            worst,
            summaries);
    }

    private static ConstraintDiagnostic SelectWorst(
        ConstraintDiagnostic? current,
        string axisName,
        KinematicQuantity quantity,
        double observed,
        double limit)
    {
        var candidate = new ConstraintDiagnostic(
            axisName,
            quantity,
            observed,
            limit,
            observed / limit);
        return current is null || candidate.Utilization > current.Utilization
            ? candidate
            : current;
    }

    private static PlannedTrajectory CreatePlan(
        CandidateTrajectory candidate,
        KinematicCheck check,
        double period) =>
        new(
            candidate.Samples,
            period,
            candidate.AppliedProcessSpeed,
            check.Worst,
            check.Summaries);

    private sealed record CandidateTrajectory(
        double[,] Samples,
        double AppliedProcessSpeed);

    private sealed record KinematicCheck(
        bool IsFeasible,
        ConstraintDiagnostic? Worst,
        IReadOnlyList<AxisKinematicSummary> Summaries);
}

internal sealed record PlannedTrajectory(
    double[,] Samples,
    double PeriodSeconds,
    double AppliedProcessSpeed,
    ConstraintDiagnostic? LimitingConstraint,
    IReadOnlyList<AxisKinematicSummary> AxisSummaries,
    bool WasDynamicallyReduced = false)
{
    internal int NumberOfRows => Samples.GetLength(0);

    internal double DurationSeconds => NumberOfRows / (1 / PeriodSeconds);
}
