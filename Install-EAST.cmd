@echo off
setlocal DisableDelayedExpansion
set "EAST_ROOT=%~1"
if not defined EAST_ROOT set "EAST_ROOT=%~dp0"
pushd "%EAST_ROOT%" || goto :failed
if not exist "Ortho-Sim.py" goto :wrongfolder
if not exist "requirements-windows.txt" goto :wrongfolder
if not exist "tester_config.json" goto :wrongfolder

py -3.10 -c "import struct; assert struct.calcsize('P') == 8; import tkinter" || goto :pythonmissing
if not exist ".venv\Scripts\python.exe" (
    py -3.10 -m venv .venv || goto :failed
)
".venv\Scripts\python.exe" -c "import struct, sys; assert sys.version_info[:2] == (3, 10) and struct.calcsize('P') == 8; import tkinter" || goto :badvenv
".venv\Scripts\python.exe" -m pip install -r requirements-windows.txt || goto :failed
".venv\Scripts\python.exe" -m pip check || goto :failed
".venv\Scripts\python.exe" Ortho-Sim.py --check-environment || goto :failed
if not exist "EAST Logs" mkdir "EAST Logs" || goto :failed
> "EAST Logs\.east-install-write-check.tmp" echo EAST write check
if errorlevel 1 goto :failed
if not exist "EAST Logs\.east-install-write-check.tmp" goto :failed
del "EAST Logs\.east-install-write-check.tmp"

set "EAST_INSTALL_ROOT=%CD%"
set "EAST_LAUNCH_SCRIPT=%~dp0Launch-EAST.cmd"
if not exist "%EAST_LAUNCH_SCRIPT%" goto :failed
powershell.exe -NoProfile -Command "$ErrorActionPreference='Stop'; $desktop=[Environment]::GetFolderPath('DesktopDirectory'); if (!$desktop) {throw 'Desktop folder unavailable'}; $link=(New-Object -ComObject WScript.Shell).CreateShortcut((Join-Path $desktop 'EAST.lnk')); $link.TargetPath=$env:ComSpec; $link.Arguments='/d /c '+[char]34+[char]34+$env:EAST_LAUNCH_SCRIPT+[char]34+' '+[char]34+$env:EAST_INSTALL_ROOT+[char]34+[char]34; $link.WorkingDirectory=$env:EAST_INSTALL_ROOT; $link.WindowStyle=7; $link.Description='Launch the EAST AFO stiffness tester'; $icon=Join-Path $env:EAST_INSTALL_ROOT 'images\icon.ico'; if (Test-Path -LiteralPath $icon) {$link.IconLocation=$icon}; $link.Save()" || goto :failed
echo.
echo Setup complete. Double-click EAST on this Windows account's desktop.
popd
pause
exit /b 0

:wrongfolder
echo Put both EAST setup scripts next to Ortho-Sim.py in the complete release folder.
goto :failed
:pythonmissing
echo Setup requires 64-bit Python 3.10 with Tkinter and the Windows py launcher.
goto :failed
:badvenv
echo This folder already has an incompatible .venv. Close EAST, rename that environment, and rerun setup.
:failed
echo Setup stopped. See the message above before using the desktop shortcut.
pause
exit /b 1
