param([switch]$Development)
$ErrorActionPreference = 'Stop'
# Use UTF-8 consistently in this process and its children.
$env:PYTHONUTF8 = '1'
Push-Location -LiteralPath $PSScriptRoot
try {
    if (-not (Test-Path -LiteralPath '.venv\Scripts\python.exe')) {
        py -3.12 -m venv .venv
        if ($LASTEXITCODE -ne 0) { throw 'Python 3.12 environment creation failed' }
    }
    $teleopExtras = if ($Development) { '.[camera,dev]' } else { '.[camera]' }
    & '.\.venv\Scripts\python.exe' -m pip install -e $teleopExtras
    if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed' }
    Write-Host 'KineWeave is ready. Run Start.ps1 or Start.cmd.'
} finally { Pop-Location }
