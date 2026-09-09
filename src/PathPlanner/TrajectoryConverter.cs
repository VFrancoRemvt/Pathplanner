using System.Text;
using PathPlanner.Geometry;
using PathPlanner.Parsing;
using PathPlanner.Planning;
using PathPlanner.TraSt;

namespace PathPlanner;

public sealed class TrajectoryConverter
{
    private readonly ITrajectoryWriter _writer;

    public TrajectoryConverter(ITrajectoryWriter? writer = null)
    {
        _writer = writer ?? new TraStCsvWriter();
    }

    public async Task<ConversionReport> ConvertFileAsync(
        string inputPath,
        string outputPath,
        TrajectoryConversionOptions options,
        CancellationToken cancellationToken = default)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(inputPath);
        ArgumentException.ThrowIfNullOrWhiteSpace(outputPath);

        var parser = new CncProgramParser();
        var program = parser.ParseFile(inputPath);
        await using var stream = new FileStream(
            outputPath,
            FileMode.Create,
            FileAccess.Write,
            FileShare.None,
            64 * 1024,
            useAsync: true);
        await using var writer = new StreamWriter(
            stream,
            new UTF8Encoding(encoderShouldEmitUTF8Identifier: false));
        return await ConvertAsync(
            program,
            writer,
            options,
            cancellationToken);
    }

    public async Task<ConversionReport> ConvertAsync(
        TextReader input,
        TextWriter output,
        TrajectoryConversionOptions options,
        CancellationToken cancellationToken = default)
    {
        var program = new CncProgramParser().Parse(input);
        return await ConvertAsync(
            program,
            output,
            options,
            cancellationToken);
    }

    public async Task<ConversionReport> ConvertAsync(
        CncProgram program,
        TextWriter output,
        TrajectoryConversionOptions options,
        CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(program);
        ArgumentNullException.ThrowIfNull(output);
        ArgumentNullException.ThrowIfNull(options);

        var constraints = ValidateAndOrderConstraints(program, options);
        var isRotaryDriven = program.GetAxisIndex("C") >= 0;
        var requestedDisplaySpeed = ValidateProcessSpeed(options, isRotaryDriven);
        var requestedInternalSpeed = isRotaryDriven
            ? requestedDisplaySpeed * 6
            : requestedDisplaySpeed / 60;

        var path = CornerBlendPath.Create(
            program,
            constraints,
            options.LinearToleranceMm,
            options.RotaryToleranceDegrees);
        var planned = ConstrainedTrajectoryPlanner.Plan(
            path,
            program.AxisNames,
            constraints,
            options.SamplingRateHz,
            requestedInternalSpeed);

        await _writer.WriteAsync(
            output,
            options.ProfileName,
            program.AxisNames,
            constraints,
            options.SamplingRateHz,
            planned.Samples,
            cancellationToken);

        var appliedDisplaySpeed = isRotaryDriven
            ? planned.AppliedProcessSpeed / 6
            : planned.AppliedProcessSpeed * 60;
        var wasReduced =
            appliedDisplaySpeed < requestedDisplaySpeed * (1 - 1e-6);

        return new ConversionReport(
            planned.NumberOfRows,
            planned.DurationSeconds,
            isRotaryDriven
                ? ProcessSpeedKind.RevolutionsPerMinute
                : ProcessSpeedKind.MillimetresPerMinute,
            requestedDisplaySpeed,
            appliedDisplaySpeed,
            wasReduced,
            wasReduced ? planned.LimitingConstraint : null,
            planned.AxisSummaries);
    }

    private static AxisConstraints[] ValidateAndOrderConstraints(
        CncProgram program,
        TrajectoryConversionOptions options)
    {
        if (!double.IsFinite(options.SamplingRateHz) || options.SamplingRateHz <= 0)
        {
            throw new ArgumentOutOfRangeException(
                nameof(options),
                "La frecuencia de muestreo debe ser positiva y finita.");
        }

        if (!double.IsFinite(options.LinearToleranceMm) ||
            options.LinearToleranceMm < 0 ||
            !double.IsFinite(options.RotaryToleranceDegrees) ||
            options.RotaryToleranceDegrees < 0)
        {
            throw new ArgumentOutOfRangeException(
                nameof(options),
                "Las tolerancias deben ser finitas y no negativas.");
        }

        if (string.IsNullOrWhiteSpace(options.ProfileName) ||
            options.ProfileName.Contains('\r', StringComparison.Ordinal) ||
            options.ProfileName.Contains('\n', StringComparison.Ordinal))
        {
            throw new ArgumentException(
                "El nombre de perfil no puede estar vacío ni contener saltos de línea.",
                nameof(options));
        }

        ArgumentNullException.ThrowIfNull(options.AxisConstraints);
        var duplicate = options.AxisConstraints
            .GroupBy(item => item.AxisName, StringComparer.OrdinalIgnoreCase)
            .FirstOrDefault(group => group.Count() > 1);
        if (duplicate is not null)
        {
            throw new ArgumentException(
                $"Los límites del eje {duplicate.Key} están duplicados.",
                nameof(options));
        }

        var byName = options.AxisConstraints.ToDictionary(
            item => item.AxisName,
            StringComparer.OrdinalIgnoreCase);
        var ordered = new AxisConstraints[program.AxisCount];
        for (var axis = 0; axis < program.AxisCount; axis++)
        {
            var name = program.AxisNames[axis];
            if (!byName.TryGetValue(name, out var constraint))
            {
                throw new ArgumentException(
                    $"Faltan límites para el eje {name}.",
                    nameof(options));
            }

            ValidateConstraint(constraint, options);
            ordered[axis] = constraint;
        }

        var unexpected = byName.Keys.FirstOrDefault(
            name => program.GetAxisIndex(name) < 0);
        if (unexpected is not null)
        {
            throw new ArgumentException(
                $"Se proporcionaron límites para el eje inexistente {unexpected}.",
                nameof(options));
        }

        var cIndex = program.GetAxisIndex("C");
        if (cIndex >= 0 && ordered[cIndex].Kind != AxisKind.Rotary)
        {
            throw new ArgumentException(
                "El eje C debe declararse como rotativo.",
                nameof(options));
        }

        if (cIndex < 0 && ordered.All(item => item.Kind != AxisKind.Linear))
        {
            throw new ArgumentException(
                "Una trayectoria sin C requiere al menos un eje lineal.",
                nameof(options));
        }

        return ordered;
    }

    private static void ValidateConstraint(
        AxisConstraints constraint,
        TrajectoryConversionOptions options)
    {
        if (string.IsNullOrWhiteSpace(constraint.AxisName) ||
            !double.IsFinite(constraint.MaxVelocity) ||
            !double.IsFinite(constraint.MaxAcceleration) ||
            !double.IsFinite(constraint.MaxJerk) ||
            constraint.MaxVelocity <= 0 ||
            constraint.MaxAcceleration <= 0 ||
            constraint.MaxJerk <= 0)
        {
            throw new ArgumentException(
                $"Los límites del eje {constraint.AxisName} deben ser positivos y finitos.",
                nameof(options));
        }
    }

    private static double ValidateProcessSpeed(
        TrajectoryConversionOptions options,
        bool isRotaryDriven)
    {
        var selected = isRotaryDriven
            ? options.RequestedRpm
            : options.RequestedFeedMmPerMinute;
        var unneeded = isRotaryDriven
            ? options.RequestedFeedMmPerMinute
            : options.RequestedRpm;
        var description = isRotaryDriven ? "rpm" : "avance";

        if (selected is null ||
            !double.IsFinite(selected.Value) ||
            selected.Value <= 0)
        {
            throw new ArgumentException(
                $"Debe indicarse un valor de {description} positivo y finito.",
                nameof(options));
        }

        if (unneeded is not null)
        {
            throw new ArgumentException(
                isRotaryDriven
                    ? "Una trayectoria con C no acepta avance lineal."
                    : "Una trayectoria sin C no acepta rpm.",
                nameof(options));
        }

        return selected.Value;
    }
}
