# EAST execution bench checklist — 1.8.0

**Every physical check below is NOT TESTED.** Run on the Windows laboratory
computer. Keep original experimental files. Supply observations/logs before
calling the freezing problem resolved. Close EAST normally before pulling;
if shutdown is blocked, isolate motor power and use the acknowledged exit path.

## Pull, verify and restart (PowerShell, inside the Windows EAST checkout)

```powershell
git status --short --branch
git pull --ff-only origin main
.\.venv\Scripts\python.exe -m unittest discover -s tests -q
.\.venv\Scripts\python.exe Ortho-Sim.py --check-environment
.\.venv\Scripts\python.exe Ortho-Sim.py
```

Use the existing configured Python environment. These changes add no third-party
dependencies. Do not overwrite local edits if pull reports a conflict.

## 1. Stationary timing — NOT TESTED

Close the GUI and other ODrive/Phidget programs before each standalone diagnostic.
Fixture stationary, motor not enabled by another application. The diagnostic never
arms, moves, homes, reboots, clears errors or saves controller configuration.
Sensor interfaces are opened only with the explicit real-hardware option.

```powershell
$eastDiagDir = Join-Path 'Diagnostics' (Get-Date -Format 'yyyyMMdd-HHmmss')
New-Item -ItemType Directory -Path $eastDiagDir
.\.venv\Scripts\python.exe analysis\diagnose_east_timing.py --real-hardware --duration 30 --interval-ms 8 --capture-limit-ms 150 --output "$eastDiagDir\motor-150.json"
.\.venv\Scripts\python.exe analysis\diagnose_east_timing.py --real-hardware --duration 30 --interval-ms 8 --capture-limit-ms 150 --with-sensor --sensor-recording "$eastDiagDir\bridge.csv" --rotary-recording "$eastDiagDir\rotary.csv" --output "$eastDiagDir\motor-sensors-150.json"
```

Repeat identical stationary conditions in the GUI with Terminal visible and Plot
visible; wait 20 s then press Ctrl+Shift+D. Keep each reported JSON. Compare queue
wait, lock wait, individual property brackets, total duration, p95/p99/max, achieved
rates and gaps. Record screen scaling, Python version, storage location and whether
sensor recording was active. A requested 8 ms interval is not an achieved 125 Hz.
The GUI and standalone scheduling differ; do not compare rates without noting that.

Only as a separately labelled stationary evaluation, repeat the standalone command
with `--capture-limit-ms 200` and a new output name. This changes the diagnostic
comparison only, never `tester_config.json` or the normal GUI's 150/250 ms limits.
Do not treat a 200 ms result as validated for all speeds. A low-rate or out-of-range
rotary diagnostic retains raw events; normal test startup still blocks those states.

## 2. Connection failure and stationary recovery — NOT TESTED

- Attempt Connect with ODrive absent; confirm Tk, editable fields and Quit respond.
- Connect all three interfaces. Save stationary timings and note idle evidence.
- While stationary, exercise USB disconnect/reconnect; wait for stop/motion workers
  to retire, then use green Connect. Confirm fresh monitor recovery or a useful
  error identifying the still-executing worker. No test may restart automatically.
- A lost object can be replaced only after the old transport exits. Confirm the
  connection generation changes and old calibration positioning is suspended.
- Confirm unproven continuity/reset requires physical zero recovery and matching
  empty-machine tare. Do not use equal raw motor turns as reset proof. Do not
  enable continuity, phase recovery or watchdog shortcuts.
- Bench-confirm the enclosure's actual nRST/reset behaviour separately under
  stationary, physically safe conditions; never bypass its hardware protection.

## 3. Stop, Quit and isolation exit — NOT TESTED

- Confirm Stop/Escape remains responsive during connection/preparation/faults.
- Confirm Quit locks new motion immediately and displays progress.
- At the laptop's normal scaling and 125%/150% if available, verify the blocked
  shutdown dialog shows both acknowledgements and the exit action without scrolling.
- Invoke isolation exit only after physically engaging E-stop and isolating motor
  supply. Confirm the next launch reports dirty/incomplete state; no clean reference
  is restored. Preserve logs and note any buffered-data loss.

## 4. Short slow empty-machine motion and Stop — NOT TESTED

Establish physical 90° with the square, acquire matching empty tare, confirm accepted
rotary readiness and clear fixture. Use a short bounded ROM and slow speed within
the existing constant-speed-span validation. Stop early. Confirm independent motor,
force and rotary records; idle observation and cleanup outcomes must remain separate.
Check that movement remains blocked if freshness/recovery is incomplete.

## 5. Complete 1°/s and 5°/s runs — NOT TESTED

Once prior steps succeed, run empty-machine ±10°, 100°/s², initially one cycle at
1°/s, then the intended repeats, and then 5°/s. Verify initialisation, sweeps, return
at 2°/s and 5°/s², observed idle, recorder completion and final outcome metadata.
Inspect actual gaps/rates and the separate rotary stream. Six sweeps within a run
are not six independent reliability trials. Preserve failed runs too.

## 6. Higher-speed progression — NOT TESTED

Progress only after relevant slow-run/Stop/recovery checks succeed. Stay within
existing angle/speed/acceleration limits. Recheck feedback and measurement ages,
transport response, raw sample gaps and finalisation at each condition. Offline
passes and slow-run success do not validate 10–20°/s or instantaneous acceleration.
