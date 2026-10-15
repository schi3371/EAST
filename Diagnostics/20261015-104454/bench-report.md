# EAST Windows Execution Bench Report — 2026-10-15

## Scope and preservation

- Repository: `C:\Users\schi3371\Desktop\EAST-main`
- Investigation starting HEAD: `86b89c4ac4b922fbb37f5a1fe5f2dc83148cb717`
- Implementation base and pre-commit `origin/main`: `2daed51` (`Add latest rotary sensor test logs`)
- Branch: `main`
- `git pull --ff-only origin main`: already up to date
- The 15 experimental files that were untracked during the initial investigation were
  preserved byte-for-byte and committed separately in `2daed51`; none were modified by
  the corrective work described below
- Configuration, calibration, production timing limits, safety settings, and original
  experimental data: not modified
- Hardware access/movement: none performed

## Environment inspection

- OS/platform: Windows AMD64, Windows build 26100
- Repository Python: `.venv\Scripts\python.exe`
- Python: 3.10.21, 64-bit
- Tk: 8.6
- customtkinter: 5.2.2
- CTkMessagebox: 2.7
- Pillow: 10.4.0
- odrive: 0.6.8
- Phidget22 Python package: 1.20.20240911
- Phidget native library: 1.22, built 2025-07-14
- ODrive native DLL found in repository venv
- Application environment check: **PASS**; reported that no hardware was opened or commanded
- `python -m pip check`: **NOT RUN** because the repository venv has no `pip` module

## Automated test evidence

### Full-suite run 1

- Command: `.venv\Scripts\python.exe -m unittest discover -s tests -q`
- Total: 240 tests
- Result: **FAIL — 1 failure**
- Test: `test_engine_gui_integration.FullApplicationIntegration.test_idle_observation_survives_late_followup_read_and_separate_checkpoint`
- Expected: `engine.evidence['feedback_health']['healthy']` false after a simulated late diagnostic read
- Observed: true
- Physical implication: none established; this is simulated/offline evidence

The failed test subsequently passed when run alone using unittest discovery. This does not erase the full-suite failure and suggests order/timing sensitivity requiring investigation.

### Full-suite run 2

- Command: `.venv\Scripts\python.exe -m unittest discover -s tests -q`
- Total: 240 tests
- Result: **FAIL — 1 failure**
- Test: `test_responsive_stop.GuiWorkerTests.test_slow_csv_flush_records_overrun_without_immediate_catchup`
- Expected: `maximum_acquisition_lateness_s > 0.04`
- Observed: `0.039000000106170774`
- Physical implication: none established; this is simulated/offline evidence

### Test runner note

A direct module-name invocation of the integration test failed to import sibling `test_odrive_adapter`. Discovery mode (`-s tests`) is the documented and working invocation mode.

## Physical and GUI verification status

- Stationary motor-only ODrive diagnostic: **COMPLETED 2026-10-15**; evidence in
  `Diagnostics/20261015-113317/`
- Stationary ODrive + Bridge + rotary diagnostic: **COMPLETED 2026-10-15**; evidence
  in `Diagnostics/20261015-114046/`
- Standalone and idle-GUI timing/rate/gap analysis: **COMPLETED**
- Initial normal GUI Connect: **COMPLETED**; failure and reconnect/recovery remain
  **NOT TESTED**
- Stationary Terminal versus idle/empty Plot comparison: **COMPLETED**; live populated
  plotting remains **NOT TESTED**
- Stop/Escape: **NOT TESTED**
- Quit and visible isolation-exit workflow: **NOT TESTED**
- Physical machine zero: **NOT TESTED**
- Empty-machine tare: **NOT TESTED**
- Short slow motion with early Stop: **NOT TESTED**
- Complete 1 deg/s run: **NOT TESTED**
- Complete 5 deg/s run: **NOT TESTED**

## Initial decision

Bench progression paused before hardware access because the required full offline suite failed twice, in two different timing-sensitive tests. The application environment check passed. No conclusion is made about the reported freezing problem.

## Offline failure investigation

Hardware verification remained paused. No hardware was opened or moved. Application
code, tests, configuration, calibration, production timing limits, and experimental
data were not edited.

### Reproduction

- Feedback-health test alone: 20 runs, 4 passed and 16 failed.
- Acquisition-lateness test alone: 50 runs, 34 passed and 16 failed.
- Complete 240-test suite: three additional runs; one passed, one failed both
  target tests, and one failed the feedback-health test.

### Feedback-health root cause

`safe_idle_motor()` correctly retains confirmed idle evidence, then treats a failed
fresh follow-up read as a fault: it sets `monitor_error`, invalidates machine zero
as a fault, and records `feedback_health` unhealthy. The independent monitor remains
able to obtain successful observations and its publisher unconditionally overwrites
the same latest-value `feedback_health` domain as healthy.

An instrumented failing run showed this ordering:

1. idle evidence recorded at monotonic 1160252.140;
2. failed follow-up recorded unhealthy at 1160252.140;
3. monitor recorded healthy at 1160252.156;
4. idle-checkpoint persistence recorded healthy at 1160252.156.

The overwritten summary is wrong, but the safety state remained fail-closed in a
separate probe: idle evidence stayed confirmed; `monitor_error` retained the failure;
machine zero was unverified with `fault` confidence; transport motion admission was
false; and closed-loop entry raised `FeedbackError` before a command.

The intended distinction is between a latest successful raw feedback observation
and resolution of an existing feedback fault. A new monitor sample may update the
former, but must not clear or overwrite the latter. Explicit connect/recovery retires
the monitor, validates new evidence, and starts a new monitor; that is the correct
place to clear the fault.

Smallest recommended production correction: make feedback-fault state latched until
explicit recovery, and record successful monitor observations separately. Synchronize
the fault latch/check and evidence write so a publisher cannot check an old clear state
and write healthy after the fault. Do not change motor timing or safety limits.

Smallest recommended deterministic regression: after injecting the follow-up failure,
use an event/barrier to force at least one later monitor publication, then assert that
idle evidence survives, the latest observation advances, the unresolved fault remains
identifiable, machine zero stays faulted/unverified, transport admission is false, and
closed-loop entry is rejected.

### Acquisition-lateness root cause

The implementation advances an absolute deadline by the 8 ms interval, measures
lateness after read/consume, counts missed intervals, and resets the next deadline to
`now + interval` after an overrun. That correctly prevents immediate catch-up samples.

On this host `time.monotonic()` reports `GetTickCount64()` with 15.625 ms resolution.
Fifty requested 50 ms sleeps measured from 46 ms to 78 ms. When measured as 47 ms,
the expected lateness is approximately `47 - 8 = 39 ms`, so the test's real-time
assertion `lateness > 40 ms` fails even though the scheduling algorithm behaves as
designed.

A controlled-clock prototype with exactly 50 ms of simulated consume work produced
42 ms lateness, five overruns, an 8 ms post-overrun wait, and a 58 ms sample gap.
No production correction or timing-limit change is indicated.

Smallest recommended test correction: replace real `sleep()`/wall-clock thresholds
with a controlled monotonic clock and controlled wait. Advance the clock by exactly
50 ms in the first consume and assert exact/near-exact 42 ms lateness, five missed
intervals, an 8 ms scheduled wait, and a 58 ms next-sample time. This strengthens the
regression without weakening an assertion or depending on Windows timer resolution.

All physical and GUI verification items above remain **NOT TESTED**.

## Implemented corrections

### Feedback evidence and fault latch

- Added a distinct `feedback_latest` evidence domain for successful observations.
- Added a feedback-fault latch. A successful monitor sample can update the latest
  observation but cannot replace latched unhealthy `feedback_health` evidence.
- Serialized the fault-state check and evidence update under `feedback_lock`, avoiding
  the earlier check/write race. Review found no inverse engine-lock-to-feedback-lock
  path; `TestEngine.record()` only stores evidence while holding its own lock.
- Bound monitor publication to the captured connection generation and adapter identity.
  Publications from a retired monitor/connection are rejected.
- Removed fault clearing from monitor startup. The latch is cleared only after the old
  monitor/checkpoint worker is retired and explicit connection recovery produces fresh,
  error-free idle feedback belonging to the current connection.
- Retained confirmed idle and persistence evidence independently. Recovery still
  invalidates machine zero and empty-machine tare when coordinate continuity is not
  proven; it does not restart a test or restore motion admission.

The deterministic regression holds a successful monitor publication at an event
barrier until after the idle follow-up read faults, then releases it. It verifies that
the successful observation is recorded separately while feedback health remains
unhealthy, idle remains confirmed, `monitor_error` remains set, machine zero remains
faulted/unverified, transport motion is blocked, and closed-loop entry fails before
dispatching a motor command. Separate cases verify that failed recovery preserves the
latch, validated recovery clears it, and a retired connection cannot publish evidence.

### Acquisition timing regression

The acquisition production loop and all timing limits are unchanged. The regression
now drives that production loop with a controlled monotonic clock and controlled wait.
It verifies samples at 0 ms and 58 ms, exactly 50 ms simulated first-sample work,
42 ms maximum lateness, five overruns, and an 8 ms post-overrun wait with no immediate
catch-up sample.

## Corrective verification

- Syntax check of `Ortho-Sim.py`, `east_execution.py`, and the three changed test
  modules: **PASS**
- Application environment check: **PASS** on Python 3.10.21; reported that no hardware
  was opened or commanded
- Deterministic feedback race/recovery target: **20/20 PASS**
- Deterministic acquisition-lateness target: **20/20 PASS**
- Complete offline suite run 1: **242/242 PASS** in 20.540 s
- Complete offline suite run 2: **242/242 PASS** in 20.793 s
- Complete offline suite run 3: **242/242 PASS** in 20.572 s
- `git diff --check`: **PASS** (Git emitted only expected LF-to-CRLF working-tree notices)

The first full-suite attempt after the production correction exposed six errors in
test-only extracted-method fixtures because the monitor method now requires captured
connection identity. The fixtures were updated to supply that identity and the affected
test module passed 15/15. The required three full-suite runs above were then restarted
from run 1; no failing attempt was counted as a pass.

## Experimental-run annotation

The four successful 15 October runs beginning at 10:34:59, 10:37:31, 10:38:19, and
10:38:51 were physically performed as **empty-machine runs**, despite their preserved
metadata identifying them as `afo_test`. The original metadata and recordings were not
rewritten. These successful runs demonstrate completed acquisition runs only; they do
not verify Stop, reconnect, or shutdown recovery.

## Final status and limitations

- Final implementation commit: the commit containing this report; its immutable hash
  is recorded in the Git history and the completion handoff after push.
- Offline corrections and deterministic verification: **PASS**.
- Later stationary hardware/GUI checks are recorded below; all remaining physical
  stages retain their explicit **NOT TESTED** status.
- The previously completed empty-machine runs are not substitutes for the outstanding
  staged bench checks.
- The reported freezing problem is **not declared resolved** from offline checks or the
  four completed runs alone.
- Next outstanding check: connection failure and stationary disconnect/reconnect
  recovery. It has not been started.

## Stationary standalone hardware evidence — 2026-10-15

After the operator reported the fixture stationary and competing hardware applications
closed, the documented read-only captures completed without motion/arming requests,
controller error clearing, configuration saving, or production-limit changes.

- Motor only: 2,165 samples, zero acquisition exceptions, 72.17 Hz achieved,
  16/31/47/94 ms median/p95/p99/maximum gaps, and 63 ms maximum snapshot.
- ODrive + Bridge + rotary: 2,073 motor samples, zero acquisition exceptions,
  69.10 Hz achieved, 16/31/62/79 ms gaps, and 78 ms maximum snapshot.
- Bridge: 2,073 persisted rows, 69.14 Hz achieved, 79 ms maximum gap, no reported
  recorder error.
- Rotary: 3,751 valid events, 125.0 Hz achieved, 47 ms maximum host-monotonic gap,
  1,896 tied host timestamps, accepted P3022 profile, and no recording error.
- Both motor captures had zero adapter lock wait. Standalone transport queue timing is
  unavailable. Combined motor rate was 4.25% below motor-only; typical gaps were
  unchanged, while selected tail metrics changed by 15 ms.

The raw files and detailed comparison are preserved in the evidence folders. These
stationary standalone results do not validate powered motion or resolve the reported
GUI freezing.

## Stationary GUI Terminal/Plot evidence — 2026-10-15

Normal Connect requested/confirmed controller IDLE and settled at **CONNECTED /
MACHINE-ZERO SETUP REQUIRED**. This differs from the strictly read-only standalone
diagnostics. No zero/tare, arming, motion, test, error clearing, or controller save was
performed. Original runtime JSONs and preserved copies are recorded in
`Diagnostics/20261015-114947/`.

Analysis used only the final 20 motor-seconds before each save to avoid double-counting
the rolling history. Terminal recorded 794 samples at 39.65 Hz with 31/46/62/93 ms
median/p95/p99/maximum gaps and a 62 ms maximum snapshot. Idle Plot recorded 699 samples
at 34.90 Hz with 31/47/63/78 ms gaps and a 47 ms maximum snapshot. Terminal/Plot maximum
queue waits were 16/0 ms and lock waits were 0/0 ms. The connection-settling baseline
had larger tails.

GUI maximum drain duration remained 16 ms. Its cumulative 735 ms maximum gap existed at
baseline and did not increase, so it was not attributed to a tab. Rotary remained
connected, valid, ready, within fit range, and near 124.6 Hz. The operator reported
responsive switching, resizing, and scrolling at 150% scaling, with no visible pause,
feedback fault, blocked operation, or exception. The configured launch size was
1040 x 496; exact resized dimensions were not captured.

Normal Connect opened the Bridge interface, but continuous force acquisition and sensor
recording were inactive. Plot was idle/empty; no live acquisition data populated it.
GUI monitoring waits 8 ms after each completed capture, unlike standalone deadline
scheduling, so their achieved rates are not directly equivalent. This stationary check
does not validate live plotting, powered motion, Stop, reconnect, shutdown, or resolution
of the freezing problem. The next outstanding stage is connection failure and stationary
disconnect/reconnect recovery; it remains **NOT TESTED**.

## Connection recovery stage blocker — 2026-10-15

The staged connection-recovery check stopped before its first action. The operator could
not find a Disconnect control, and source inspection confirmed that the current GUI
wires Connect, Start, and Stop but does not wire the existing `disconnect_odrive()`
method to a button or menu command. Invoking it indirectly would not verify the normal
operator workflow, and patching production code was outside this diagnostic stage.

No Disconnect and no ODrive USB unplug/replug occurred. Consequently idle confirmation,
worker/transport retirement, hardware release, absent-device Connect, reconnect,
communication-loss detection, reference invalidation, connection-generation recovery,
and retired-publication protection on real hardware all remain **NOT TESTED**. EAST was
left connected in the preceding stationary idle state with machine zero unverified.
Evidence is preserved in `Diagnostics/20261015-120315/`.

## Corrected visible-Connect communication-loss recovery — 2026-10-15

The missing manual Disconnect finding above remains preserved. The procedure was
corrected without adding a control or invoking `disconnect_odrive()` indirectly. The
operator performed a planned ODrive-only USB loss/reconnect while the fixture remained
stationary; Bridge, rotary, enclosure, motor supply, E-stop, and sensor coupling were
unchanged. Evidence is in `Diagnostics/20261015-121020/`.

Fresh precheck evidence showed controller IDLE, zero active errors, motion state idle,
ready lifecycle, valid rotary health, and unverified machine coordinates. After USB loss,
a 218 ms delayed sample was rejected at 12:12:26.096. A transient restored-sample notice
was followed by monitor failure, Stop latch, and **Motor idle NOT CONFIRMED**. The fault
checkpoint at 12:12:40.108 contained no feedback, motion state `fault`, reference
confidence `fault`, and no session mapping. Earlier idle evidence was not presented as
fresh confirmation.

One visible Connect attempt while ODrive remained absent retired the unhealthy retained
path, attempted rediscovery, and returned “Recovery discovery failed or was cancelled;
reference remains invalid.” Controls and tab/window interaction became responsive again.
No competing retry was started.

After the operator reconnected only ODrive USB, one visible Connect attempt reported the
retained connection retired/replaced, rediscovered the controller, and restored
monitoring. Final fresh evidence showed controller IDLE, zero active errors, motion state
idle, engine `ready`, reference confidence `recovery_required`, and no session mapping.
Status was **MONITOR RESTORED / MACHINE-ZERO SETUP REQUIRED**. No test or positioning
operation resumed.

The replacement path requires successful old-transport retirement before rediscovery.
The numeric connection generation is not serialized; the executed single-increment path
implies generation 1 → 2. Continued new samples and no recurring stale fault support
successful replacement-monitor publication and fault-latch recovery, though the latch
itself is not serialized. Manual Disconnect and fresh-launch absent-device Connect remain
separately **NOT TESTED**, along with Stop, Quit/shutdown, live plotting/acquisition, and
powered motion. This successful stationary recovery does not resolve the freezing issue.

## Ordinary stationary operator checks — 2026-10-15

Evidence is preserved in `Diagnostics/20261015-122448/`; final GUI JSON original paths
and hashes are recorded in that folder's report.

Stop and Escape while already idle each displayed Stop latched followed by Motor idle
confirmed, and controls remained usable. These validate idle-state input/cleanup only,
not powered stopping. Ordinary Quit closed without delay, blocker, or exception; the
prior EAST processes disappeared and no Python worker remained. Its final checkpoint
had fresh IDLE feedback and zero errors, but retained dirty/unverified continuity.

After clean exit, a fresh launch with only ODrive USB absent returned `Connection timed
out: ODrive 3943355F3231 was not found`. The GUI stayed responsive; Bridge connected and
rotary became valid/ready. Ordinary closure exited with code 0 and no Python worker.

After USB restoration, one Connect restored the matching controller with zero active
errors, load-cell connection, and accepted rotary calibration. Nothing resumed and
motion stayed blocked pending machine zero. The final rolling capture had 2,048 motor
observations over 48.203 s (42.47 Hz), 16/47/63/94 ms gaps, 0/16/47/79 ms capture
durations, 16 ms maximum queue wait, zero lock wait, and valid rotary at 124.57 Hz.
EAST was left connected and idle with reference unverified.

The earlier 14.012 s USB-loss interval is only an envelope from the timestamp of the
218 ms rejected capture to durable fault-checkpoint completion. The transient retry,
later property failure, Stop dispatch, failed idle confirmation, and persistence stages
lack individual timestamps. Configured values bound parts of the path (8 ms poll wait,
up to two qualified retries within 1,000 ms, 1.0 s idle-confirmation deadline), but
Fibre property calls have no cancellable per-call deadline. No exact partition was
invented and the hardware fault was not rerun.

Manual Disconnect remains an interface limitation. Powered Stop, live acquisition and
populated plotting, blocked-shutdown recovery, and powered motion remain **NOT TESTED**.
These stationary checks do not resolve the freezing problem.

## Supervised functional verification batch — 2026-10-15

Detailed evidence and per-run timing are in `Diagnostics/20261015-130418/`. The operator
reported successful physical 90° machine zero, matching 16-sample empty tare, actual
motion Stop, completed motion, populated-plot responsiveness, and ordinary Quit.

The requested interrupted ±5° 1°/s run finalized as `aborted / operator stop`, with
fresh confirmed IDLE, zero active errors, 306/306 rows persisted, complete recorder
worker, and no monitor/rotary/persistence error. A subsequent GUI warning stated that
Stop/cleanup was still running and emitted a stalled-worker diagnostic as the next run
started. Later clean completions do not erase that warning; it remains an unresolved
sequencing limitation and the batch should have stopped there under the requested rule.

The requested ±10° three-cycle runs completed at 1°/s and 5°/s with 3/3 cycles, normal
machine-zero returns, confirmed IDLE, zero active errors, complete persistence, valid
measurements, and no monitor/recorder/rotary error. Acquisition achieved 64.35 and
63.29 Hz; independent rotary recording achieved 125.00 and 124.96 Hz. The operator
reported the populated Plot and GUI remained responsive.

An extra completed ±5° 1°/s run and unrequested ±10° runs at 10°/s and 20°/s were also
created. They are preserved and passed their metadata/CSV integrity checks, but the
higher-speed files are not treated as a planned progression or broad reliability proof.
All six run sets were labelled `custom` rather than the intended named empty-machine
preset, although they retain the same verified reference and valid empty tare.

Across all six runs, acquisition was 63.29–65.29 Hz, independent rotary recording was
124.96–125.00 Hz, rotary maximum gaps were 31–32 ms, and there were zero invalid rotary
samples or nonzero-error CSV rows. All recorders completed with no pending rows. Ordinary
Quit wrote a clean checkpoint with controller IDLE, zero errors, verified reference and
clean continuity, and no EAST/Python process remained.

Blocked-shutdown/isolation recovery remains **NOT TESTED**. The cleanup warning and
limited run count mean the freezing problem is not declared categorically resolved.
