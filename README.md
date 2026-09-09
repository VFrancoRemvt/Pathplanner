# PathPlanner

Biblioteca C# y aplicación de demostración para convertir trayectorias de torno
en tablas CSV compatibles con el ejemplo
[Triamec TraSt-CSV](https://github.com/Triamec/TraSt-CSV).

El proyecto no conecta con TAM ni controla hardware. Calcula la trayectoria,
comprueba sus límites y genera un archivo que se puede revisar y enviar a
Triamec.

## Proyectos

- `src/PathPlanner`: biblioteca .NET 8 integrable.
- `src/PathPlanner.Demo`: consola interactiva y automatizable.
- `tests/PathPlanner.Tests`: pruebas de parsing, geometría, dinámica, CSV y demo.

## Formato de entrada

Cada línea representa un punto y contiene todas las posiciones:

```text
X[-5.784000]Z[-0.0396756659]W[-0.0736754871]C[0.00000]
X[-5.780950]Z[-0.0396756659]W[-0.0638048435]C[15.00000]
```

Los nombres de eje pueden cambiar de orden entre líneas. La primera línea
determina el orden de columnas del CSV. Los números usan punto decimal y
cultura invariante.

- Ejes lineales: milímetros.
- Ejes rotativos: grados.
- `C` debe ser estrictamente creciente y se conserva desenrollado.

## Cálculo de la trayectoria

Para un programa con `C`, este eje es la coordenada maestra:

```text
velocidad C [deg/s] = rpm × 6
```

Sin `C`, la coordenada maestra es la longitud geométrica suavizada y:

```text
velocidad lineal [mm/s] = avance [mm/min] / 60
```

Las esquinas se sustituyen por blends quínticos tangentes, limitados por una
tolerancia global lineal y otra rotativa. Los puntos inicial y final no cambian.

El perfil temporal parte y termina en reposo, con transiciones quínticas y
crucero cuando el recorrido lo permite. La biblioteca muestrea las posiciones a
la frecuencia indicada y calcula por eje:

- velocidad en mm/s o deg/s;
- aceleración en mm/s² o deg/s²;
- jerk en mm/s³ o deg/s³.

Si la velocidad solicitada supera algún límite, se busca la mayor rpm o avance
factible y se vuelve a generar la trayectoria. El informe identifica el eje,
la magnitud, el valor observado y el límite que condiciona el resultado.

## Uso de la biblioteca

```csharp
using PathPlanner;

var options = new TrajectoryConversionOptions
{
    SamplingRateHz = 10_000,
    RequestedRpm = 1_000,
    LinearToleranceMm = 0.001,
    RotaryToleranceDegrees = 0.01,
    ProfileName = "Desbaste 4 ejes",
    AxisConstraints =
    [
        new("X", AxisKind.Linear, 50, 500, 10_000),
        new("Z", AxisKind.Linear, 50, 500, 10_000),
        new("W", AxisKind.Linear, 20, 200, 5_000),
        new("C", AxisKind.Rotary, 12_000, 50_000, 1_000_000)
    ]
};

var report = await new TrajectoryConverter().ConvertFileAsync(
    "programa.txt",
    "trayectoria.csv",
    options);
```

Para una trayectoria sin `C`, use `RequestedFeedMmPerMinute` en lugar de
`RequestedRpm`.

También se puede trabajar con `TextReader`, `TextWriter` o un `CncProgram`
previamente analizado. `ConversionReport` contiene filas, duración, velocidad
solicitada y aplicada, diagnóstico limitante y máximos de cada eje.

## Aplicación de demostración

Modo interactivo:

```bash
dotnet run --project src/PathPlanner.Demo
```

Modo automatizado para X/Z:

```bash
dotnet run --project src/PathPlanner.Demo -- \
  --input programa.txt \
  --output trayectoria.csv \
  --profile "Predesbaste XZ" \
  --sampling-rate 10000 \
  --feed 120 \
  --linear-tolerance 0.001 \
  --rotary-tolerance 0 \
  --axis X:linear:50:500:10000 \
  --axis Z:linear:50:500:10000
```

Formato de cada `--axis`:

```text
EJE:linear|rotary:VELOCIDAD_MAX:ACELERACION_MAX:JERK_MAX
```

Use `--rpm` para programas con `C` y `--feed` para programas sin `C`.
`--help` muestra la ayuda. Códigos de salida:

- `0`: correcto o cancelación interactiva;
- `2`: entrada o argumentos inválidos;
- `3`: fallo al planificar o escribir.

## CSV generado

La salida reproduce la cabecera vigente del ejemplo de Triamec:

```text
# Trajectory Stream Data - Version 1.0
# Profile: ...
# Sampling rate: ...Hz
# Move duration: ...s
# Number of rows: ...
# Column 1: Position X [mm]
```

Los datos se escriben secuencialmente, con punto decimal y precisión `double`
de ida y vuelta. La duración de cabecera sigue la convención del ejemplo:
filas divididas por frecuencia.

Triamec indica que su formato CSV aún está sujeto a cambios. Por ello la salida
está aislada tras `ITrajectoryWriter`; un cambio futuro no necesita modificar
el parser ni el planificador.

## Compilación y pruebas

Requiere .NET 8:

```bash
dotnet restore PathPlanner.sln
dotnet build PathPlanner.sln --configuration Release
dotnet test PathPlanner.sln --configuration Release
```

## Consideraciones de seguridad

Un CSV válido no sustituye la validación de la máquina real. Antes de ejecutar
una tabla se deben confirmar escalas, sentidos, homing, límites de recorrido,
frecuencia admitida, configuración de unidades y comportamiento de parada con
Triamec. La demo deliberadamente no habilita ejes ni envía movimiento.
