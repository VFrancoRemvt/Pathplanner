using PathPlanner.Demo;

namespace PathPlanner.Tests;

public sealed class DemoApplicationTests
{
    [Fact]
    public async Task RunAsync_HelpReturnsSuccess()
    {
        var output = new StringWriter();

        var exitCode = await DemoApplication.RunAsync(
            ["--help"],
            new StringReader(string.Empty),
            output,
            new StringWriter());

        Assert.Equal(DemoApplication.SuccessExitCode, exitCode);
        Assert.Contains("--sampling-rate", output.ToString(), StringComparison.Ordinal);
    }

    [Fact]
    public async Task RunAsync_ArgumentsGenerateCsv()
    {
        var directory = CreateTemporaryDirectory();
        try
        {
            var inputPath = Path.Combine(directory, "input.txt");
            var outputPath = Path.Combine(directory, "output.csv");
            await File.WriteAllTextAsync(
                inputPath,
                "X[0]Z[0]\nX[1]Z[0]\nX[2]Z[0.2]");
            var standardOutput = new StringWriter();
            var error = new StringWriter();

            var exitCode = await DemoApplication.RunAsync(
                [
                    "--input", inputPath,
                    "--output", outputPath,
                    "--profile", "Demo",
                    "--sampling-rate", "20",
                    "--feed", "60",
                    "--linear-tolerance", "0.01",
                    "--rotary-tolerance", "0",
                    "--axis", "X:linear:100:1000:100000",
                    "--axis", "Z:linear:100:1000:100000"
                ],
                new StringReader(string.Empty),
                standardOutput,
                error);

            Assert.Equal(DemoApplication.SuccessExitCode, exitCode);
            Assert.True(File.Exists(outputPath));
            Assert.Contains(
                "CSV generado correctamente",
                standardOutput.ToString(),
                StringComparison.Ordinal);
            Assert.Equal(string.Empty, error.ToString());
        }
        finally
        {
            Directory.Delete(directory, recursive: true);
        }
    }

    [Fact]
    public async Task RunAsync_InvalidArgumentsReturnDocumentedCode()
    {
        var error = new StringWriter();

        var exitCode = await DemoApplication.RunAsync(
            ["--unknown", "value"],
            new StringReader(string.Empty),
            new StringWriter(),
            error);

        Assert.Equal(DemoApplication.InvalidInputExitCode, exitCode);
        Assert.Contains("Entrada inválida", error.ToString(), StringComparison.Ordinal);
    }

    private static string CreateTemporaryDirectory()
    {
        var path = Path.Combine(
            Path.GetTempPath(),
            "PathPlannerTests",
            Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(path);
        return path;
    }
}
