namespace PathPlanner.Geometry;

internal sealed class CornerBlendPath
{
    private const int ToleranceSamples = 64;
    private const int ArcSamplesPerSegment = 16;

    private readonly double[][] _points;
    private readonly double[] _sourceCoordinates;
    private readonly Blend?[] _blends;
    private readonly AxisKind[] _axisKinds;
    private readonly int _cAxisIndex;
    private readonly double[]? _arcCoordinates;
    private readonly double[]? _arcSourceCoordinates;

    private CornerBlendPath(
        double[][] points,
        double[] sourceCoordinates,
        Blend?[] blends,
        AxisKind[] axisKinds,
        int cAxisIndex,
        double[]? arcCoordinates,
        double[]? arcSourceCoordinates)
    {
        _points = points;
        _sourceCoordinates = sourceCoordinates;
        _blends = blends;
        _axisKinds = axisKinds;
        _cAxisIndex = cAxisIndex;
        _arcCoordinates = arcCoordinates;
        _arcSourceCoordinates = arcSourceCoordinates;
        ProcessLength = cAxisIndex >= 0
            ? sourceCoordinates[^1] - sourceCoordinates[0]
            : arcCoordinates is null
                ? sourceCoordinates[^1] - sourceCoordinates[0]
                : arcCoordinates[^1];
    }

    internal int AxisCount => _points[0].Length;

    internal bool IsRotaryDriven => _cAxisIndex >= 0;

    internal double ProcessLength { get; }

    internal static CornerBlendPath Create(
        CncProgram program,
        IReadOnlyList<AxisConstraints> constraints,
        double linearTolerance,
        double rotaryTolerance)
    {
        var byName = constraints.ToDictionary(
            constraint => constraint.AxisName,
            StringComparer.OrdinalIgnoreCase);
        var kinds = program.AxisNames.Select(name => byName[name].Kind).ToArray();
        var points = Enumerable.Range(0, program.PointCount)
            .Select(program.GetPoint)
            .ToArray();
        var cAxis = program.GetAxisIndex("C");
        var source = cAxis >= 0
            ? BuildRotaryCoordinates(points, cAxis)
            : BuildLinearCoordinates(points, kinds);

        var emptyBlends = new Blend?[points.Length];
        var initial = new CornerBlendPath(
            points,
            source,
            emptyBlends,
            kinds,
            cAxis,
            null,
            null);
        var blends = initial.CreateBlends(linearTolerance, rotaryTolerance);
        var smoothed = new CornerBlendPath(
            points,
            source,
            blends,
            kinds,
            cAxis,
            null,
            null);

        if (cAxis >= 0)
        {
            return smoothed;
        }

        var (arcCoordinates, arcSourceCoordinates) = smoothed.BuildArcLengthLookup();
        return new CornerBlendPath(
            points,
            source,
            blends,
            kinds,
            cAxis,
            arcCoordinates,
            arcSourceCoordinates);
    }

    internal double[] Evaluate(double processCoordinate)
    {
        if (processCoordinate < -1e-12 || processCoordinate > ProcessLength + 1e-12)
        {
            throw new ArgumentOutOfRangeException(nameof(processCoordinate));
        }

        var clamped = Math.Clamp(processCoordinate, 0, ProcessLength);
        if (IsRotaryDriven)
        {
            return EvaluateSource(_sourceCoordinates[0] + clamped);
        }

        var source = InterpolateLookup(
            _arcCoordinates!,
            _arcSourceCoordinates!,
            clamped);
        return EvaluateSource(source);
    }

    private static double[] BuildRotaryCoordinates(double[][] points, int cAxis)
    {
        var result = points.Select(point => point[cAxis]).ToArray();
        for (var index = 1; index < result.Length; index++)
        {
            if (result[index] <= result[index - 1])
            {
                throw new TrajectoryPlanningException(
                    $"El eje C debe ser estrictamente creciente; puntos {index} y {index + 1}.");
            }
        }

        return result;
    }

    private static double[] BuildLinearCoordinates(
        IReadOnlyList<double[]> points,
        IReadOnlyList<AxisKind> kinds)
    {
        var result = new double[points.Count];
        for (var point = 1; point < points.Count; point++)
        {
            var squaredDistance = 0.0;
            for (var axis = 0; axis < kinds.Count; axis++)
            {
                if (kinds[axis] == AxisKind.Linear)
                {
                    var delta = points[point][axis] - points[point - 1][axis];
                    squaredDistance += delta * delta;
                }
            }

            var distance = Math.Sqrt(squaredDistance);
            if (distance <= 0)
            {
                throw new TrajectoryPlanningException(
                    $"Los puntos lineales {point} y {point + 1} son coincidentes.");
            }

            result[point] = result[point - 1] + distance;
        }

        return result;
    }

    private Blend?[] CreateBlends(double linearTolerance, double rotaryTolerance)
    {
        var result = new Blend?[_points.Length];
        for (var knot = 1; knot < _points.Length - 1; knot++)
        {
            var leftSpan = _sourceCoordinates[knot] - _sourceCoordinates[knot - 1];
            var rightSpan = _sourceCoordinates[knot + 1] - _sourceCoordinates[knot];
            var maximumHalfWindow = 0.45 * Math.Min(leftSpan, rightSpan);
            if (maximumHalfWindow <= 0)
            {
                continue;
            }

            var maximumBlend = CreateBlend(knot, maximumHalfWindow);
            if (IsWithinTolerance(maximumBlend, linearTolerance, rotaryTolerance))
            {
                result[knot] = maximumBlend;
                continue;
            }

            var low = 0.0;
            var high = maximumHalfWindow;
            for (var iteration = 0; iteration < 48; iteration++)
            {
                var middle = (low + high) / 2;
                var candidate = CreateBlend(knot, middle);
                if (IsWithinTolerance(candidate, linearTolerance, rotaryTolerance))
                {
                    low = middle;
                }
                else
                {
                    high = middle;
                }
            }

            if (low > maximumHalfWindow * 1e-9)
            {
                result[knot] = CreateBlend(knot, low);
            }
        }

        return result;
    }

    private Blend CreateBlend(int knot, double halfWindow)
    {
        var start = _sourceCoordinates[knot] - halfWindow;
        var end = _sourceCoordinates[knot] + halfWindow;
        return new Blend(
            knot,
            start,
            end,
            EvaluateLinear(start),
            EvaluateLinear(end),
            GetSegmentSlope(knot - 1),
            GetSegmentSlope(knot));
    }

    private bool IsWithinTolerance(
        Blend blend,
        double linearTolerance,
        double rotaryTolerance)
    {
        for (var sample = 0; sample <= ToleranceSamples; sample++)
        {
            var coordinate = blend.Start +
                ((blend.End - blend.Start) * sample / ToleranceSamples);
            var candidate = EvaluateBlend(blend, coordinate);
            var reference = EvaluateLinear(coordinate);
            var linearSquared = 0.0;

            for (var axis = 0; axis < AxisCount; axis++)
            {
                var deviation = candidate[axis] - reference[axis];
                if (_axisKinds[axis] == AxisKind.Linear)
                {
                    linearSquared += deviation * deviation;
                }
                else if (Math.Abs(deviation) > rotaryTolerance + 1e-12)
                {
                    return false;
                }
            }

            if (Math.Sqrt(linearSquared) > linearTolerance + 1e-12)
            {
                return false;
            }
        }

        return true;
    }

    private (double[] ArcCoordinates, double[] SourceCoordinates) BuildArcLengthLookup()
    {
        var arc = new List<double> { 0 };
        var source = new List<double> { _sourceCoordinates[0] };
        var previous = EvaluateSource(_sourceCoordinates[0]);
        var total = 0.0;

        for (var segment = 0; segment < _sourceCoordinates.Length - 1; segment++)
        {
            var start = _sourceCoordinates[segment];
            var end = _sourceCoordinates[segment + 1];
            for (var sample = 1; sample <= ArcSamplesPerSegment; sample++)
            {
                var coordinate = start +
                    ((end - start) * sample / ArcSamplesPerSegment);
                var current = EvaluateSource(coordinate);
                total += LinearDistance(previous, current);
                arc.Add(total);
                source.Add(coordinate);
                previous = current;
            }
        }

        if (total <= 0)
        {
            throw new TrajectoryPlanningException("La trayectoria suavizada no tiene longitud.");
        }

        return (arc.ToArray(), source.ToArray());
    }

    private double LinearDistance(IReadOnlyList<double> left, IReadOnlyList<double> right)
    {
        var squared = 0.0;
        for (var axis = 0; axis < AxisCount; axis++)
        {
            if (_axisKinds[axis] == AxisKind.Linear)
            {
                var delta = right[axis] - left[axis];
                squared += delta * delta;
            }
        }

        return Math.Sqrt(squared);
    }

    private double[] EvaluateSource(double coordinate)
    {
        var segment = FindSegment(_sourceCoordinates, coordinate);
        if (segment > 0 &&
            _blends[segment] is { } leftBlend &&
            coordinate >= leftBlend.Start &&
            coordinate <= leftBlend.End)
        {
            return EvaluateBlend(leftBlend, coordinate);
        }

        var nextKnot = segment + 1;
        if (nextKnot < _blends.Length &&
            _blends[nextKnot] is { } rightBlend &&
            coordinate >= rightBlend.Start &&
            coordinate <= rightBlend.End)
        {
            return EvaluateBlend(rightBlend, coordinate);
        }

        return EvaluateLinear(coordinate);
    }

    private double[] EvaluateLinear(double coordinate)
    {
        var segment = FindSegment(_sourceCoordinates, coordinate);
        var start = _sourceCoordinates[segment];
        var end = _sourceCoordinates[segment + 1];
        var fraction = end == start ? 0 : (coordinate - start) / (end - start);
        var result = new double[AxisCount];

        for (var axis = 0; axis < AxisCount; axis++)
        {
            result[axis] = _points[segment][axis] +
                ((_points[segment + 1][axis] - _points[segment][axis]) * fraction);
        }

        return result;
    }

    private double[] EvaluateBlend(Blend blend, double coordinate)
    {
        var length = blend.End - blend.Start;
        var u = Math.Clamp((coordinate - blend.Start) / length, 0, 1);
        var u2 = u * u;
        var u3 = u2 * u;
        var u4 = u3 * u;
        var u5 = u4 * u;
        var h00 = 1 - (10 * u3) + (15 * u4) - (6 * u5);
        var h10 = u - (6 * u3) + (8 * u4) - (3 * u5);
        var h01 = (10 * u3) - (15 * u4) + (6 * u5);
        var h11 = (-4 * u3) + (7 * u4) - (3 * u5);
        var result = new double[AxisCount];

        for (var axis = 0; axis < AxisCount; axis++)
        {
            result[axis] =
                (h00 * blend.StartPoint[axis]) +
                (h10 * length * blend.StartSlope[axis]) +
                (h01 * blend.EndPoint[axis]) +
                (h11 * length * blend.EndSlope[axis]);
        }

        return result;
    }

    private double[] GetSegmentSlope(int segment)
    {
        var span = _sourceCoordinates[segment + 1] - _sourceCoordinates[segment];
        var result = new double[AxisCount];
        for (var axis = 0; axis < AxisCount; axis++)
        {
            result[axis] =
                (_points[segment + 1][axis] - _points[segment][axis]) / span;
        }

        return result;
    }

    private static int FindSegment(IReadOnlyList<double> coordinates, double value)
    {
        if (value <= coordinates[0])
        {
            return 0;
        }

        if (value >= coordinates[^1])
        {
            return coordinates.Count - 2;
        }

        var low = 0;
        var high = coordinates.Count - 1;
        while (high - low > 1)
        {
            var middle = (low + high) / 2;
            if (coordinates[middle] <= value)
            {
                low = middle;
            }
            else
            {
                high = middle;
            }
        }

        return low;
    }

    private static double InterpolateLookup(
        IReadOnlyList<double> keys,
        IReadOnlyList<double> values,
        double key)
    {
        var segment = FindSegment(keys, key);
        var fraction = (key - keys[segment]) / (keys[segment + 1] - keys[segment]);
        return values[segment] +
            ((values[segment + 1] - values[segment]) * fraction);
    }

    private sealed record Blend(
        int Knot,
        double Start,
        double End,
        double[] StartPoint,
        double[] EndPoint,
        double[] StartSlope,
        double[] EndSlope);
}
