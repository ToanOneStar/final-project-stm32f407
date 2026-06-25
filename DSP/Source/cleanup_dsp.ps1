$base = Split-Path -Parent $MyInvocation.MyCommand.Path

$removeDirs = @(
    "BasicMathFunctions",
    "BayesFunctions",
    "CommonTables",
    "ComplexMathFunctions",
    "DistanceFunctions",
    "FastMathFunctions",
    "FilteringFunctions",
    "InterpolationFunctions",
    "QuaternionMathFunctions",
    "SVMFunctions",
    "StatisticsFunctions",
    "SupportFunctions"
)

foreach ($d in $removeDirs) {
    $path = Join-Path $base $d
    if (Test-Path $path) {
        Remove-Item -Recurse -Force $path
        Write-Host "Removed dir: $d"
    }
}

$removeFiles = @(
    "ControllerFunctions\ControllerFunctions.c",
    "ControllerFunctions\arm_pid_init_q15.c",
    "ControllerFunctions\arm_pid_init_q31.c",
    "ControllerFunctions\arm_pid_reset_q15.c",
    "ControllerFunctions\arm_pid_reset_q31.c",
    "ControllerFunctions\arm_sin_cos_f32.c",
    "ControllerFunctions\arm_sin_cos_q31.c"
)

foreach ($f in $removeFiles) {
    $path = Join-Path $base $f
    if (Test-Path $path) {
        Remove-Item -Force $path
        Write-Host "Removed file: $f"
    }
}

Write-Host ""
Write-Host "=== Con lai trong DSP/Source ==="
Get-ChildItem -Path $base -Recurse -Filter "*.c" | Select-Object -ExpandProperty Name
