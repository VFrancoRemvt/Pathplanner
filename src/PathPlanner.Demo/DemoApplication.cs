using System.Globalization;
using PathPlanner.Parsing;

namespace PathPlanner.Demo;

public static class DemoApplication
{
    public const int SuccessExitCode = 0;
    public const int InvalidInputExitCode = 2;
    public const int ConversionFailureExitCode = 3;

    public static async Task<int> RunAsync(
        string[] args,
        TextReader input,
        TextWriter output,
        TextWriter error,
        CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(args);
        ArgumentNullException.ThrowIfNull(input);
        ArgumentNullException.ThrowIfNull(output);
        ArgumentNullException.ThrowIfNull(error);

        try
        {
            if (args.Any(argument =>
                    string.Equals(argument, "--help", StringComparison.OrdinalIgnoreCase) ||
                    string.Equals(argument, "-h", StringComparison.OrdinalIgnoreCase)))
            {
                await output.WriteLineAsync(HelpText);
                return SuccessExitCode;
            }

            DemoSettings settings;
            CncProgram program;
            var parser = new CncProgramParser();

            if (args.Length == 0)
            {
                var inputPath = await PromptExistingFileAsync(input, output);
                program = parser.ParseFile(inputPath);
                settings = await PromptSettingsAsync(inputPath, program, input, output);
                await PrintSettingsAsync(settings, output);
                var confirmed = await PromptAsync(
                    input,
                    output,
                    "¿Generar el archivo? [s/N]: ");
                if (!string.Equals(confirmed, "s", StringComparison.OrdinalIgnoreCase) &&
                    !string.Equals(confirmed, "sí", StringComparison.OrdinalIgnoreCase) &&
                    !string.Equals(confirmed, "si", StringComparison.OrdinalIgnoreCase))
                {
                    await output.WriteLineAsync("Operación cancelada.");
                    return SuccessExitCode;
                }
            }
            else
            {
                settings = ParseArguments(args);
                program = parser.ParseFile(settings.InputPath);
                ValidateAxes(program, settings.Constraints);
            }

            var options = new TrajectoryConversionOptions
            {
                SamplingRateHz = settings.SamplingRateHz,
                RequestedRpm = settings.Rpm,
                RequestedFeedMmPerMinute = settings.Feed,
                LinearToleranceMm = settings.LinearToleranceMm,
                RotaryToleranceDegrees = settings.RotaryToleranceDegrees,
                AxisConstraints = settings.Constraints,
                ProfileName = settings.ProfileName
            };

            var report = await new TrajectoryConverter().ConvertFileAsync(
                settings.InputPath,
                settings.OutputPath,
                options,
                cancellationToken);
            await PrintReportAsync(settings.OutputPath, report, output);
            return SuccessExitCode;
        }
        catch (Exception exception) when (
            exception is ArgumentException or
            CncParseException or
            FileNotFoundException)
        {
            await error.WriteLineAsync($"Entrada inválida: {exception.Message}");
            await error.WriteLineAsync("Use --help para consultar la sintaxis.");
            return InvalidInputExitCode;
        }
        catch (Exception exception) when (
            exception is TrajectoryPlanningException or
            IOException or
            UnauthorizedAccessException)
        {
            await error.WriteLineAsync($"No se pudo generar el CSV: {exception.Message}");
            return ConversionFailureExitCode;
        }
    }

    private static DemoSettings ParseArguments(IReadOnlyList<string> args)
    {
        string? inputPath = null;
        string? outputPath = null;
        string? profile = null;
        double? samplingRate = null;
        double? rpm = null;
        double? feed = null;
        double? linearTolerance = null;
        double? rotaryTolerance = null;
        var constraints = new List<AxisConstraints>();

        for (var index = 0; index < args.Count; index++)
        {
            var option = args[index];
            if (!option.StartsWith("--", StringComparison.Ordinal))
            {
                throw new ArgumentException($"Argumento inesperado: {option}.");
            }

            if (index + 1 >= args.Count)
            {
                throw new ArgumentException($"Falta el valor de {option}.");
            }

            var value = args[++index];
            switch (option.ToLowerInvariant())
            {
                case "--input":
                    inputPath = value;
                    break;
                case "--output":
                    outputPath = value;
                    break;
                case "--profile":
                    profile = value;
                    break;
                case "--sampling-rate":
                    samplingRate = ParsePositive(value, option);
                    break;
                case "--rpm":
                    rpm = ParsePositive(value, option);
                    break;
                case "--feed":
                    feed = ParsePositive(value, option);
                    break;
                case "--linear-tolerance":
                    linearTolerance = ParseNonNegative(value, option);
                    break;
                case "--rotary-tolerance":
                    rotaryTolerance = ParseNonNegative(value, option);
                    break;
                case "--axis":
                    constraints.Add(ParseAxisConstraint(value));
                    break;
                default:
                    throw new ArgumentException($"Opción desconocida: {option}.");
            }
        }

        if (string.IsNullOrWhiteSpace(inputPath) ||
            string.IsNullOrWhiteSpace(outputPath) ||
            string.IsNullOrWhiteSpace(profile) ||
            samplingRate is null ||
            linearTolerance is null ||
            rotaryTolerance is null ||
            constraints.Count == 0)
        {
            throw new ArgumentException(
                "Faltan opciones obligatorias: --input, --output, --profile, " +
                "--sampling-rate, tolerancias y al menos un --axis.");
        }

        if ((rpm is null) == (feed is null))
        {
            throw new ArgumentException(
                "Indique exactamente una velocidad: --rpm o --feed.");
        }

        return new DemoSettings(
            inputPath,
            outputPath,
            profile,
            samplingRate.Value,
            rpm,
            feed,
            linearTolerance.Value,
            rotaryTolerance.Value,
            constraints);
    }

    private static AxisConstraints ParseAxisConstraint(string value)
    {
        var parts = value.Split(':', StringSplitOptions.TrimEntries);
        if (parts.Length != 5 || string.IsNullOrWhiteSpace(parts[0]))
        {
            throw new ArgumentException(
                $"Límite de eje inválido \"{value}\". Use EJE:linear|rotary:V:A:J.");
        }

        var kind = parts[1].ToLowerInvariant() switch
        {
            "linear" => AxisKind.Linear,
            "rotary" => AxisKind.Rotary,
            _ => throw new ArgumentException(
                $"Tipo de eje inválido \"{parts[1]}\".")
        };
        return new AxisConstraints(
            parts[0],
            kind,
            ParsePositive(parts[2], $"velocidad de {parts[0]}"),
            ParsePositive(parts[3], $"aceleración de {parts[0]}"),
            ParsePositive(parts[4], $"jerk de {parts[0]}"));
    }

    private static async Task<DemoSettings> PromptSettingsAsync(
        string inputPath,
        CncProgram program,
        TextReader input,
        TextWriter output)
    {
        var outputPath = await PromptRequiredAsync(
            input,
            output,
            "Archivo CSV de salida: ");
        var profile = await PromptRequiredAsync(
            input,
            output,
            "Nombre del perfil: ");
        var samplingRate = await PromptPositiveAsync(
            input,
            output,
            "Frecuencia de muestreo [Hz]: ");
        var hasC = program.GetAxisIndex("C") >= 0;
        var processSpeed = await PromptPositiveAsync(
            input,
            output,
            hasC ? "Velocidad solicitada [rpm]: " : "Avance solicitado [mm/min]: ");
        var linearTolerance = await PromptNonNegativeAsync(
            input,
            output,
            "Tolerancia lineal global [mm]: ");
        var rotaryTolerance = await PromptNonNegativeAsync(
            input,
            output,
            "Tolerancia rotativa global [deg]: ");

        var constraints = new List<AxisConstraints>();
        foreach (var axisName in program.AxisNames)
        {
            var defaultKind = string.Equals(
                axisName,
                "C",
                StringComparison.OrdinalIgnoreCase)
                ? AxisKind.Rotary
                : AxisKind.Linear;
            var kindText = await PromptAsync(
                input,
                output,
                $"Eje {axisName}, tipo [L/R, predeterminado " +
                $"{(defaultKind == AxisKind.Linear ? "L" : "R")}]: ");
            var kind = string.IsNullOrWhiteSpace(kindText)
                ? defaultKind
                : kindText.StartsWith("R", StringComparison.OrdinalIgnoreCase)
                    ? AxisKind.Rotary
                    : kindText.StartsWith("L", StringComparison.OrdinalIgnoreCase)
                        ? AxisKind.Linear
                        : throw new ArgumentException(
                            $"Tipo inválido para el eje {axisName}: {kindText}.");
            var unit = kind == AxisKind.Linear ? "mm" : "deg";
            var maximumVelocity = await PromptPositiveAsync(
                input,
                output,
                $"  Velocidad máxima [{unit}/s]: ");
            var maximumAcceleration = await PromptPositiveAsync(
                input,
                output,
                $"  Aceleración máxima [{unit}/s²]: ");
            var maximumJerk = await PromptPositiveAsync(
                input,
                output,
                $"  Jerk máximo [{unit}/s³]: ");
            constraints.Add(new AxisConstraints(
                axisName,
                kind,
                maximumVelocity,
                maximumAcceleration,
                maximumJerk));
        }

        return new DemoSettings(
            inputPath,
            outputPath,
            profile,
            samplingRate,
            hasC ? processSpeed : null,
            hasC ? null : processSpeed,
            linearTolerance,
            rotaryTolerance,
            constraints);
    }

    private static async Task<string> PromptExistingFileAsync(
        TextReader input,
        TextWriter output)
    {
        var path = await PromptRequiredAsync(
            input,
            output,
            "Archivo TXT de entrada: ");
        if (!File.Exists(path))
        {
            throw new FileNotFoundException("No existe el archivo de entrada.", path);
        }

        return path;
    }

    private static async Task<string> PromptRequiredAsync(
        TextReader input,
        TextWriter output,
        string prompt)
    {
        var value = await PromptAsync(input, output, prompt);
        if (string.IsNullOrWhiteSpace(value))
        {
            throw new ArgumentException($"Se requiere un valor para \"{prompt.Trim()}\".");
        }

        return value.Trim();
    }

    private static async Task<double> PromptPositiveAsync(
        TextReader input,
        TextWriter output,
        string prompt) =>
        ParsePositive(
            await PromptRequiredAsync(input, output, prompt),
            prompt.Trim());

    private static async Task<double> PromptNonNegativeAsync(
        TextReader input,
        TextWriter output,
        string prompt) =>
        ParseNonNegative(
            await PromptRequiredAsync(input, output, prompt),
            prompt.Trim());

    private static async Task<string> PromptAsync(
        TextReader input,
        TextWriter output,
        string prompt)
    {
        await output.WriteAsync(prompt);
        var value = await input.ReadLineAsync();
        if (value is null)
        {
            throw new ArgumentException("La entrada interactiva terminó antes de tiempo.");
        }

        return value.Trim();
    }

    private static double ParsePositive(string value, string name)
    {
        var result = ParseNumber(value, name);
        if (result <= 0)
        {
            throw new ArgumentException($"{name} debe ser mayor que cero.");
        }

        return result;
    }

    private static double ParseNonNegative(string value, string name)
    {
        var result = ParseNumber(value, name);
        if (result < 0)
        {
            throw new ArgumentException($"{name} no puede ser negativo.");
        }

        return result;
    }

    private static double ParseNumber(string value, string name)
    {
        if (!double.TryParse(
                value,
                NumberStyles.Float,
                CultureInfo.InvariantCulture,
                out var result) ||
            !double.IsFinite(result))
        {
            throw new ArgumentException($"{name} debe ser un número finito.");
        }

        return result;
    }

    private static void ValidateAxes(
        CncProgram program,
        IReadOnlyCollection<AxisConstraints> constraints)
    {
        var supplied = new HashSet<string>(
            constraints.Select(item => item.AxisName),
            StringComparer.OrdinalIgnoreCase);
        var expected = new HashSet<string>(
            program.AxisNames,
            StringComparer.OrdinalIgnoreCase);
        if (!supplied.SetEquals(expected) || supplied.Count != constraints.Count)
        {
            throw new ArgumentException(
                "Las opciones --axis deben coincidir exactamente con los ejes del TXT.");
        }
    }

    private static async Task PrintSettingsAsync(
        DemoSettings settings,
        TextWriter output)
    {
        await output.WriteLineAsync();
        await output.WriteLineAsync("Resumen:");
        await output.WriteLineAsync($"  Entrada: {Path.GetFullPath(settings.InputPath)}");
        await output.WriteLineAsync($"  Salida: {Path.GetFullPath(settings.OutputPath)}");
        await output.WriteLineAsync($"  Perfil: {settings.ProfileName}");
        await output.WriteLineAsync($"  Muestreo: {settings.SamplingRateHz:G17} Hz");
        await output.WriteLineAsync(
            settings.Rpm is not null
                ? $"  Velocidad: {settings.Rpm:G17} rpm"
                : $"  Avance: {settings.Feed:G17} mm/min");
    }

    private static async Task PrintReportAsync(
        string outputPath,
        ConversionReport report,
        TextWriter output)
    {
        var file = new FileInfo(outputPath);
        var unit = report.SpeedKind == ProcessSpeedKind.RevolutionsPerMinute
            ? "rpm"
            : "mm/min";
        await output.WriteLineAsync("CSV generado correctamente.");
        await output.WriteLineAsync($"  Ruta: {file.FullName}");
        await output.WriteLineAsync($"  Tamaño: {file.Length} bytes");
        await output.WriteLineAsync($"  Filas: {report.NumberOfRows}");
        await output.WriteLineAsync($"  Duración: {report.DurationSeconds:G17} s");
        await output.WriteLineAsync(
            $"  Velocidad solicitada: {report.RequestedProcessSpeed:G17} {unit}");
        await output.WriteLineAsync(
            $"  Velocidad aplicada: {report.AppliedProcessSpeed:G17} {unit}");

        if (report.WasReduced && report.LimitingConstraint is { } limiting)
        {
            await output.WriteLineAsync(
                $"  Recálculo: {limiting.AxisName} {limiting.Quantity}, " +
                $"{limiting.ObservedValue:G17} / {limiting.ConfiguredLimit:G17}.");
        }
    }

    private const string HelpText =
        """
        PathPlanner.Demo - generador de CSV para Triamec TraSt

        Uso interactivo:
          dotnet run --project src/PathPlanner.Demo

        Uso por argumentos:
          PathPlanner.Demo --input ruta.txt --output ruta.csv --profile NOMBRE
            --sampling-rate HZ (--rpm RPM | --feed MM_MIN)
            --linear-tolerance MM --rotary-tolerance DEG
            --axis EJE:linear|rotary:VELOCIDAD:ACELERACION:JERK [--axis ...]

        Códigos de salida: 0 correcto, 2 entrada inválida, 3 fallo de conversión.
        """;

    private sealed record DemoSettings(
        string InputPath,
        string OutputPath,
        string ProfileName,
        double SamplingRateHz,
        double? Rpm,
        double? Feed,
        double LinearToleranceMm,
        double RotaryToleranceDegrees,
        IReadOnlyList<AxisConstraints> Constraints);
}
