using PathPlanner.Parsing;

namespace PathPlanner.Tests;

public sealed class CncProgramParserTests
{
    [Fact]
    public void Parse_PreservesFirstRowOrderAndReordersLaterRows()
    {
        const string text =
            """
            X[1.5] Z[-2] C[0]
            C[15] X[1.25] Z[-2.5]
            """;

        var program = new CncProgramParser().Parse(new StringReader(text));

        Assert.Equal(["X", "Z", "C"], program.AxisNames);
        Assert.Equal(2, program.PointCount);
        Assert.Equal(15, program.GetPosition(1, 2));
        Assert.Equal(1.25, program.GetPosition(1, 0));
    }

    [Theory]
    [InlineData("X[1] X[2]\nX[3] X[4]", "duplicado")]
    [InlineData("X[1] Z[2]\nX[3]", "faltan ejes")]
    [InlineData("X[1] Z[2]\nX[3] Z[4] W[5]", "sobran ejes")]
    [InlineData("X[1] basura\nX[2]", "sintaxis")]
    [InlineData("X[1]", "al menos dos")]
    public void Parse_RejectsInvalidInputWithLineContext(
        string text,
        string expectedReason)
    {
        var exception = Assert.Throws<CncParseException>(
            () => new CncProgramParser().Parse(new StringReader(text)));

        Assert.Contains("Línea", exception.Message, StringComparison.Ordinal);
        Assert.Contains(expectedReason, exception.Message, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void Parse_UsesInvariantDecimalAndAcceptsWhitespace()
    {
        const string text = " X [ 1.25e-2 ] Z[-3.] \nX[.5] Z[4]\t";

        var program = new CncProgramParser().Parse(new StringReader(text));

        Assert.Equal(0.0125, program.GetPosition(0, 0), 12);
        Assert.Equal(-3, program.GetPosition(0, 1));
        Assert.Equal(0.5, program.GetPosition(1, 0));
    }
}
