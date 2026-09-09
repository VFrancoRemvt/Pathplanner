using System.Globalization;
using System.Text.RegularExpressions;

namespace PathPlanner.Parsing;

public sealed partial class CncProgramParser
{
    public CncProgram ParseFile(string path)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(path);
        using var reader = File.OpenText(path);
        return Parse(reader);
    }

    public CncProgram Parse(TextReader reader)
    {
        ArgumentNullException.ThrowIfNull(reader);

        var axisNames = new List<string>();
        var rows = new List<double[]>();
        string? line;
        var lineNumber = 0;

        while ((line = reader.ReadLine()) is not null)
        {
            lineNumber++;
            if (string.IsNullOrWhiteSpace(line))
            {
                throw new CncParseException(lineNumber, "la línea está vacía.");
            }

            var values = ParseLine(line, lineNumber);
            if (rows.Count == 0)
            {
                axisNames.AddRange(values.Select(item => item.AxisName));
            }
            else
            {
                ValidateAxisSet(axisNames, values, lineNumber);
            }

            var byName = values.ToDictionary(
                item => item.AxisName,
                item => item.Value,
                StringComparer.OrdinalIgnoreCase);
            rows.Add(axisNames.Select(axis => byName[axis]).ToArray());
        }

        if (rows.Count < 2)
        {
            throw new CncParseException(Math.Max(1, lineNumber), "se requieren al menos dos puntos.");
        }

        var positions = new double[rows.Count, axisNames.Count];
        for (var row = 0; row < rows.Count; row++)
        {
            for (var axis = 0; axis < axisNames.Count; axis++)
            {
                positions[row, axis] = rows[row][axis];
            }
        }

        return new CncProgram(axisNames, positions);
    }

    private static List<ParsedValue> ParseLine(string line, int lineNumber)
    {
        var result = new List<ParsedValue>();
        var position = 0;

        while (position < line.Length)
        {
            if (string.IsNullOrWhiteSpace(line[position..]))
            {
                break;
            }

            var match = AxisValueRegex().Match(line, position);
            if (!match.Success || match.Index != position)
            {
                var residual = line[position..].Trim();
                throw new CncParseException(
                    lineNumber,
                    $"sintaxis no reconocida cerca de \"{residual}\".");
            }

            var axisName = match.Groups["axis"].Value;
            if (result.Any(item => string.Equals(
                    item.AxisName,
                    axisName,
                    StringComparison.OrdinalIgnoreCase)))
            {
                throw new CncParseException(lineNumber, $"el eje {axisName} está duplicado.");
            }

            var valueText = match.Groups["value"].Value;
            if (!double.TryParse(
                    valueText,
                    NumberStyles.Float,
                    CultureInfo.InvariantCulture,
                    out var value) ||
                !double.IsFinite(value))
            {
                throw new CncParseException(
                    lineNumber,
                    $"el valor \"{valueText}\" del eje {axisName} no es finito.");
            }

            result.Add(new ParsedValue(axisName, value));
            position = match.Index + match.Length;
        }

        if (result.Count == 0)
        {
            throw new CncParseException(lineNumber, "no contiene posiciones de ejes.");
        }

        return result;
    }

    private static void ValidateAxisSet(
        IReadOnlyCollection<string> expected,
        IReadOnlyCollection<ParsedValue> actual,
        int lineNumber)
    {
        var actualNames = new HashSet<string>(
            actual.Select(item => item.AxisName),
            StringComparer.OrdinalIgnoreCase);
        var missing = expected.Where(axis => !actualNames.Contains(axis)).ToArray();
        var unexpected = actualNames.Where(axis => !expected.Contains(
            axis,
            StringComparer.OrdinalIgnoreCase)).ToArray();

        if (missing.Length == 0 && unexpected.Length == 0)
        {
            return;
        }

        var details = new List<string>();
        if (missing.Length > 0)
        {
            details.Add($"faltan ejes: {string.Join(", ", missing)}");
        }

        if (unexpected.Length > 0)
        {
            details.Add($"sobran ejes: {string.Join(", ", unexpected)}");
        }

        throw new CncParseException(lineNumber, string.Join("; ", details) + ".");
    }

    [GeneratedRegex(
        @"\G\s*(?<axis>[A-Za-z]+)\s*\[\s*(?<value>[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)\s*\]",
        RegexOptions.CultureInvariant)]
    private static partial Regex AxisValueRegex();

    private sealed record ParsedValue(string AxisName, double Value);
}
