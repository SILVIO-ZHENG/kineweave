@echo off
setlocal
cd /d "%~dp0"
set PYTHONUTF8=1
if not exist ".venv\Scripts\python.exe" (
  echo Run Setup.ps1 first.
  pause
  exit /b 1
)
set "TELEOP_ROBOT=%~1"
if "%TELEOP_ROBOT%"=="" set "TELEOP_ROBOT=g2_24"
".venv\Scripts\python.exe" -m teleop_v2 serve --driver sim --robot "%TELEOP_ROBOT%" --open
