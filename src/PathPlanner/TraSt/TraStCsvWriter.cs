using System.Globalization;
using System.Text;

namespace PathPlanner.TraSt;

public interface ITrajectoryWriter
{
    Task WriteAsync(
        TextWriter writer,
        string profileName,
        IReadOnlyList<string> axisNames,
        IReadOnlyList<AxisConstraints> axisConstraints,
        double samplingRateHz,
        double[,] positions,
        CancellationToken cancellationToken = default);
}

public sealed class TraStCsvWriter : ITrajectoryWriter
{
    public async Task WriteAsync(
        TextWriter writer,
        string profileName,
        IReadOnlyList<string> axisNames,
        IReadOnlyList<AxisConstraints> axisConstraints,
        double samplingRateHz,
        double[,] positions,
        CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(writer);
        ArgumentNullException.ThrowIfNull(axisNames);
        ArgumentNullException.ThrowIfNull(axisConstraints);
        ArgumentNullException.ThrowIfNull(positions);

        var rows = positions.GetLength(0);
        var axes = positions.GetLength(1);
        if (axisNames.Count != axes || axisConstraints.Count != axes)
        {
            throw new ArgumentException(
                "Los nombres, límites y columnas de posiciones deben coincidir.");
        }

        var duration = rows / samplingRateHz;
        await writer.WriteLineAsync(
            "# Trajectory Stream Data - Version 1.0".AsMemory(),
            cancellationToken);
        await writer.WriteLineAsync(
            $"# Profile: {profileName}".AsMemory(),
            cancellationToken);
        await writer.WriteLineAsync(
            $"# Sampling rate: {samplingRateHz.ToString("G17", CultureInfo.InvariantCulture)}Hz".AsMemory(),
            cancellationToken);
        await writer.WriteLineAsync(
            $"# Move duration: {duration.ToString("G17", CultureInfo.InvariantCulture)}s".AsMemory(),
            cancellationToken);
        await writer.WriteLineAsync(
            $"# Number of rows: {rows.ToString(CultureInfo.InvariantCulture)}".AsMemory(),
            cancellationToken);

        for (var axis = 0; axis < axes; axis++)
        {
            var unit = axisConstraints[axis].Kind == AxisKind.Linear ? "mm" : "deg";
            await writer.WriteLineAsync(
                $"# Column {axis + 1}: Position {axisNames[axis]} [{unit}]".AsMemory(),
                cancellationToken);
        }

        var line = new StringBuilder(axes * 24);
        for (var row = 0; row < rows; row++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            line.Clear();
            for (var axis = 0; axis < axes; axis++)
            {
                if (axis > 0)
                {
                    line.Append(',');
                }

                line.Append(positions[row, axis].ToString(
                    "G17",
                    CultureInfo.InvariantCulture));
            }

            await writer.WriteLineAsync(line.ToString().AsMemory(), cancellationToken);
        }
    }
}
