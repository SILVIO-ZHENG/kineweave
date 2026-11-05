param(
    [string]$Robot = 'g2_24',
    [int]$Port = 8765,
    [switch]$NoBrowser
)
$ErrorActionPreference = 'Stop'
$env:PYTHONUTF8 = '1'
# The convenience launcher always uses simulation; use documented explicit commands for hardware.
Push-Location -LiteralPath $PSScriptRoot
try {
    if (-not (Test-Path -LiteralPath '.venv\Scripts\python.exe')) {
        throw 'Run Setup.ps1 first'
    }
    $teleopLaunchArgs = @('-m', 'teleop_v2', 'serve', '--driver', 'sim', '--robot', $Robot, '--port', "$Port")
    if (-not $NoBrowser) { $teleopLaunchArgs += '--open' }
    & '.\.venv\Scripts\python.exe' @teleopLaunchArgs
} finally { Pop-Location }
