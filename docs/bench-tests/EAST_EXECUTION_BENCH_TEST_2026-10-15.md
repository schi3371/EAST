# EAST execution bench checklist — 1.8.0

**Physical status is recorded per stage below.** Run on the Windows laboratory
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

## 1. Stationary timing — STANDALONE AND IDLE GUI CAPTURES COMPLETED

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

### Observed standalone results — 2026-10-15

With the fixture reported stationary and competing hardware applications closed:

- Motor-only evidence: `Diagnostics/20261015-113317/`; 2,165 successful samples,
  zero acquisition exceptions, 72.17 Hz achieved motor rate, 16/31/47/94 ms
  median/p95/p99/maximum gaps, and 63 ms maximum complete snapshot.
- ODrive + Bridge + rotary evidence: `Diagnostics/20261015-114046/`; 2,073 successful
  motor samples, zero acquisition exceptions, 69.10 Hz achieved motor rate,
  16/31/62/79 ms gaps, and 78 ms maximum complete snapshot.
- Combined Bridge CSV: 2,073 rows over 29.968 s, 69.14 Hz achieved, with
  15/32/63/79 ms host-timestamp gaps and no reported recorder error.
- Combined raw rotary CSV: 3,751 valid events, 125.0 Hz achieved, 47 ms maximum
  host-monotonic gap, 1,896 timestamp ties, and no recording error.
- Adapter lock-wait maximum was 0 ms in both captures. Standalone transport queue
  wait is unavailable because this probe does not use the GUI transport owner.
- Adding the sensors reduced achieved motor rate by 4.25%; median/p95 gaps were
  unchanged, gap p99 increased 15 ms, maximum snapshot increased 15 ms, and maximum
  gap decreased 15 ms. This stationary pair does not validate powered motion or GUI
  freezing. The 200 ms standalone evaluation remains **NOT TESTED**.

### Observed stationary GUI results — 2026-10-15

Evidence: `Diagnostics/20261015-114947/`. Normal Connect requested/confirmed controller
IDLE, unlike the strictly read-only standalone probes, and settled at **CONNECTED /
MACHINE-ZERO SETUP REQUIRED**. It did not establish zero/tare, arm, move, clear errors,
or save configuration.

- Final capture windows were selected from rolling history as the last 20 motor-seconds
  before each save. Terminal: 794 samples, 39.65 Hz, 31/46/62/93 ms
  median/p95/p99/maximum gaps, and 62 ms maximum snapshot. Plot: 699 samples,
  34.90 Hz, 31/47/63/78 ms gaps, and 47 ms maximum snapshot.
- Transport queue-wait maxima were 16 ms for Terminal and 0 ms for Plot; adapter
  lock-wait maximum was 0 ms for both. Baseline connection settling had 16 ms maximum
  lock wait and larger timing tails.
- GUI maximum drain duration stayed 16 ms. Its cumulative 735 ms maximum gap already
  existed at baseline and did not increase, so it was not attributed to either tab.
- Rotary stayed connected, valid, ready, and within fit range at 124.57–124.63 Hz.
- At 150% Windows scaling and configured 1040 x 496 launch size, the operator reported
  responsive tab switching, resizing, and scrolling with no visible pause, feedback
  fault, blocked operation, or exception. Exact resized dimensions were not captured.
- Normal Connect opened the Bridge interface, but continuous force acquisition and all
  sensor recording were inactive. Plot was idle/empty; no live acquisition data was
  plotted. Idle Plot responsiveness does not validate populated plotting or motion.
- GUI monitoring waits 8 ms after each complete capture, unlike the standalone probe's
  deadline schedule. Its lower achieved rates are not directly equivalent to standalone
  rates. These checks do not resolve the reported freezing problem.

## 2. Connection failure and stationary recovery — CONNECTED USB-LOSS RECOVERY COMPLETED

Observed 2026-10-15: the current GUI exposes Connect, Start, and Stop, with only Help in
the File menu. Although `disconnect_odrive()` exists in the application, it is not wired
to a visible GUI action. The required normal stationary Disconnect therefore could not
be initiated. No indirect method call or code patch was used.

That manual-Disconnect blocker remains valid and its evidence is preserved in
`Diagnostics/20261015-120315/`. The procedure was corrected without adding a control or
invoking the hidden method: planned communication-loss recovery was tested through the
visible Connect action instead. Evidence: `Diagnostics/20261015-121020/`.

Observed connected USB-loss sequence:

- Fresh precheck: controller IDLE, zero active errors, fresh monitoring, ready lifecycle,
  valid rotary health, and machine-zero confidence `recovery_required`.
- After only ODrive USB was unplugged, a 218 ms delayed sample was rejected at
  12:12:26.096. A transient restored-sample notice was followed by monitor failure,
  Stop latch, and **Motor idle NOT CONFIRMED**. At 12:12:40.108 the durable fault state
  had no feedback, motion state `fault`, reference confidence `fault`, and no mapping.
  Historical idle was not treated as fresh confirmation.
- One visible Connect attempt with ODrive absent retired the unhealthy retained path,
  attempted rediscovery, then returned the actionable failure “Recovery discovery failed
  or was cancelled; reference remains invalid.” Controls and Tk remained responsive.
- After only ODrive USB was reconnected, one visible Connect attempt reported the old
  connection retired/replaced, rediscovered the device, and restored monitoring. Final
  fresh evidence was controller IDLE, zero active errors, motion state idle, ready
  lifecycle, and no session mapping. Status was **MONITOR RESTORED / MACHINE-ZERO SETUP
  REQUIRED**; no test or positioning operation restarted.
- The successful replacement path could proceed only after the old transport worker
  exited. Connection generation is inferred from the executed path as 1 → 2 because the
  schema does not serialize it. No stale old-monitor overwrite was observed after
  validated recovery. Rotary remained valid with the same sensor connection identity.

Manual Disconnect remains unavailable/**NOT TESTED**. Fresh-application launch with
ODrive absent, idle-state Stop/Escape, and ordinary Quit were subsequently completed as
recorded in `Diagnostics/20261015-122448/`. Powered Stop, blocked shutdown/isolation,
live acquisition/plotting, powered motion, and enclosure nRST behaviour remain
**NOT TESTED**.

- On a fresh application launch, attempt Connect with ODrive absent; confirm Tk and
  editable fields respond. Keep Quit as its separately staged check.
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

## 3. Stop, Quit and isolation exit — IDLE STOP/ESCAPE AND ORDINARY QUIT COMPLETED

Evidence: `Diagnostics/20261015-122448/`.

- Stop and Escape while already idle each latched Stop and returned **Motor idle
  confirmed**. Controls remained usable and no unresolved worker was observed. These are
  idle-state checks only; powered stopping performance remains **NOT TESTED**.
- Ordinary Quit completed without a blocked dialog, exception, or observed delay. Both
  EAST process IDs and all Python workers exited. The final checkpoint kept continuity
  dirty and `clean_shutdown: false` because reference continuity was unverified; it
  nevertheless contained fresh controller IDLE feedback and zero active errors.
- After clean exit, a fresh launch with only ODrive USB absent returned the actionable
  error `Connection timed out: ODrive 3943355F3231 was not found`. The GUI remained
  responsive, Bridge connected, rotary became valid/ready, and ordinary closure exited
  with code 0 without a surviving worker.
- After ODrive USB restoration, one fresh Connect found the expected serial, active
  errors 0, load-cell connected, and rotary ready. Nothing resumed automatically;
  motion stayed blocked at **MACHINE-ZERO SETUP REQUIRED**. The final diagnostic had
  2,048 rolling motor samples over 48.203 s (42.47 Hz), 16/47/63/94 ms motor gaps,
  0/16/47/79 ms capture durations, 16 ms maximum queue wait, zero lock wait, and valid
  rotary at 124.57 Hz.
- The earlier 14.012 s communication-loss envelope cannot be split exactly: only the
  218 ms rejected capture and final persistence timestamp were recorded. Retry
  scheduling, later transport failure, Stop dispatch, failed idle confirmation, and
  persistence are untimestamped within the envelope.

Manual Disconnect remains unavailable. Deliberate blocked shutdown/power-isolated exit
remains **NOT TESTED**.

- Confirm Stop/Escape stopping performance during powered motion.
- Confirm Quit admission/progress during an active or naturally blocked operation.
- At the laptop's normal scaling and 125%/150% if available, verify the blocked
  shutdown dialog shows both acknowledgements and the exit action without scrolling.
- Invoke isolation exit only after physically engaging E-stop and isolating motor
  supply. Confirm the next launch reports dirty/incomplete state; no clean reference
  is restored. Preserve logs and note any buffered-data loss.

## 4. Short slow empty-machine motion and Stop — COMPLETED WITH CLEANUP WARNING

Evidence: `Diagnostics/20261015-130418/` and the 13:10:12 `rebuild` run set in
`EAST Logs/`. The operator established physical 90° machine zero and a matching
16-sample empty tare, then stopped a ±5°, 1°/s run during motion. Metadata finalized the
run as `aborted / operator stop`, 0/1 cycles, with fresh confirmed IDLE, zero active
errors, healthy monitoring, 306/306 persisted rows, recorder worker complete, no pending
rows, and no rotary/persistence error. The operator observed actual stopping and
responsive GUI behaviour.

The following run was nevertheless started before/while the GUI emitted `Stop/cleanup
is still running` and wrote a stalled-worker diagnostic. That run later completed and
all its evidence finalized, but the warning is preserved as an unresolved sequencing
limitation. The requested stop-on-warning rule was not followed.

## 5. Complete 1°/s and 5°/s runs — COMPLETED

The ±10°, 100°/s², three-cycle runs completed at 1°/s and 5°/s. Each recorded 3/3
cycles, a successful normal machine-zero return, confirmed final IDLE, zero active
errors, complete CSV persistence, no pending rows, no monitor/rotary/recorder error,
verified reference, and valid tare. The operator reported populated plotting and GUI
responsiveness remained good.

The 1°/s run persisted 8,947 rows at 64.35 Hz; the 5°/s run persisted 2,231 rows at
63.29 Hz. Independent rotary rates were 125.00 and 124.96 Hz with 32 ms maximum gaps and
zero invalid samples. Joined CSV row-gap p95/p99 values were 16/31 ms; maxima were 47/32
ms. Snapshot p95/p99 values were 16/31 ms, maxima 78/47 ms, transport-queue maxima
31/15 ms, and adapter-lock maxima zero. All files identified the protocol as `custom`,
not the intended named empty-machine preset; physical empty-fixture status is therefore
an operator observation supported by tare/reference evidence, not the protocol label.

## 6. Higher-speed progression — UNPLANNED SINGLE RUNS RECORDED

The operator also completed ±10°, three-cycle runs at 10°/s and 20°/s even though this
batch instructed that higher-speed verification remain deferred. Both metadata sets
report 3/3 cycles, confirmed IDLE, complete persistence, zero errors/invalid rotary
samples, and approximately 63.60/64.06 Hz acquisition plus 124.99/124.97 Hz rotary.
These original runs are preserved, but two unplanned single passes do not constitute a
staged higher-speed reliability campaign or validate instantaneous acceleration.

Ordinary Quit then produced a clean continuity checkpoint and left no EAST/Python
process. Deliberately blocked shutdown/isolation recovery remains **NOT TESTED**. The
cleanup warning and single-run nature of the higher speeds prevent declaring the
reported freezing problem categorically resolved.
