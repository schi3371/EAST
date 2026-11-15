@echo off
setlocal DisableDelayedExpansion
set "EAST_ROOT=%~1"
if not defined EAST_ROOT set "EAST_ROOT=%~dp0"
pushd "%EAST_ROOT%" || goto :failed
if not exist ".venv\Scripts\python.exe" goto :failed
if not exist "Ortho-Sim.py" goto :failed
".venv\Scripts\python.exe" -u Ortho-Sim.py
set "EAST_EXIT=%ERRORLEVEL%"
popd
if "%EAST_EXIT%"=="0" exit /b 0
echo EAST exited with status %EAST_EXIT%. Keep the console output for troubleshooting.
pause
exit /b %EAST_EXIT%

:failed
echo EAST setup is missing or the installation folder moved. Run Install-EAST.cmd again.
pause
exit /b 1
