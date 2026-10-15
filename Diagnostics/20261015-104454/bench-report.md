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

- Stationary motor-only ODrive diagnostic: **NOT TESTED**
- Stationary ODrive + Bridge + rotary diagnostic: **NOT TESTED**
- Timing/rate/gap analysis: **NOT TESTED**
- GUI Connect/recovery: **NOT TESTED**
- Terminal versus Plot comparison: **NOT TESTED**
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
- Hardware and GUI checks listed above: **NOT TESTED** in this corrective session.
- The previously completed empty-machine runs are not substitutes for the outstanding
  staged bench checks.
- The reported freezing problem is **not declared resolved** from offline checks or the
  four completed runs alone.
- Next outstanding check: stationary, read-only, motor-only ODrive diagnostics, only
  after the physically present operator confirms the fixture is stationary and all
  other hardware applications are closed. No hardware step was started here.
