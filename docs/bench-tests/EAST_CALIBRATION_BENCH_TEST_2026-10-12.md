# EAST Calibration Workflow Bench Test — 2026-10-12

## Scope and controls

- Host: Windows lab computer
- Repository: `C:\Users\schi3371\Desktop\EAST-main`
- Tested commit at start: `592034a` (`Add calibration-only positioning and reconnect workflow`)
- Software version: `1.6.0-calibration-positioning`
- Accepted motor conversion: `2.055 AFO deg/ODrive turn` — unchanged
- Required setup: empty machine, no AFO installed, enclosure and E-stop protections active
- Movement rule: no hardware movement without the operator's explicit go-ahead immediately before that movement
- Acceptance rule: code inspection and automated tests are supporting evidence only; they do not pass a Windows-layout or hardware-behaviour check

## Baseline software evidence

- `git status -sb`: `main` matched `origin/main`; working tree clean before this report was created.
- Relevant test command: `.venv\Scripts\python.exe -m unittest discover -s tests -p "test_calibration_motion.py"`
  - Result: 10 tests passed.
- Relevant test command: `.venv\Scripts\python.exe -m unittest discover -s tests -p "test_rotary_calibration.py"`
  - Result: 35 tests passed.
- Relevant test command: `.venv\Scripts\python.exe -m unittest discover -s tests -p "test_gui_contract.py"`
  - Result: 14 tests passed.
- Note: an initial direct module invocation exposed the suite's sibling-import assumption; discovery mode completed successfully. This is not hardware evidence.

## Bench checks

### 1. Windows layout and controls

Expected/procedure:

- Launch `Ortho-Sim.py` at the lab computer's normal display scaling.
- Inspect the complete main window and Rotary Sensor Calibration window.
- Confirm controls, instructions, status messages, and capture fields are readable and accessible.
- Resize and scroll both applicable views; switch repeatedly between Session Terminal and Plot.
- Close the calibration window and confirm the main application remains usable.

Observed behaviour: Not yet observed.

Status: **NOT TESTED**

Evidence/logs: Pending screenshots or operator observations. No hardware movement is required.

### 2. Sensor connection and stationary capture

Expected/procedure:

- With the empty fixture stationary, connect the P3022 on HUB0007 port 0 in the intended input mode.
- Record actual hub serial, parent/channel SKU, input mode, configured sample interval, and live readings.
- Capture stationary Sensor + motor and Sensor only points as applicable, without requiring normal machine zero or tare.
- Compare saved raw samples and point summary with displayed readings and check separation of independent physical angle from commanded/positioning angle.

Observed behaviour: Not yet observed.

Status: **NOT TESTED**

Evidence/logs: Pending `EAST Logs/Rotary Calibration/rotary_<timestamp>/` and Session Terminal messages.

### 3. Calibration-only positioning

Expected/procedure:

- Independently measure the stationary fixture, enter the current positioning angle, acknowledge the empty/clear fixture, and enable calibration-only positioning.
- After separate explicit approval for each movement, test small negative and positive degree steps, then small raw-turn steps.
- Verify direction, requested versus actual raw-turn displacement, displayed estimate, settling, timeout, and travel-limit rejection.
- Confirm normal strain Start remains blocked without normal machine zero and tare.

Observed behaviour: Not yet observed.

Status: **NOT TESTED**

Evidence/logs: Pending Session Terminal and `calibration_motion_events.json`.

### 4. Practical measurement and reconnect workflow

Expected/procedure:

- After explicit approval, move to a small test position and wait for confirmed idle.
- Open the enclosure only when motion is stopped, measure independently, and confirm movement disables while points remain.
- Close the enclosure, reconnect, confirm positioning remains disabled, and explicitly re-establish or confirm the positioning reference based on whether the fixture moved.
- Resume calibration capture/motion without normal machine zero or tare.
- Verify reconnect/reset evidence and separation of incompatible raw encoder coordinate frames.

Observed behaviour: Not yet observed.

Status: **NOT TESTED**

Evidence/logs: Pending Session Terminal, runtime-state audit, point files, and motion-event file.

### 5. Stop and interruption behaviour

Expected/procedure:

- Under the lab-approved procedure and with explicit approval for each initiating movement, check calibration Stop, enclosure interlock, and physical E-stop.
- Verify motion stops, no automatic restart occurs, and recovery requires appropriate operator action.
- Check calibration-window close during an approved small movement and reconnect handling while movement is active.
- Do not unplug USB during motion or introduce unapproved faults.

Observed behaviour: Not yet observed.

Status: **NOT TESTED**

Evidence/logs: Pending Session Terminal and controller/runtime-state evidence.

### 6. Saved data and usability

Expected/procedure:

- Inspect `raw_samples.csv`, `points.csv`, `points.json`, `session_metadata.json`, and `calibration_motion_events.json`.
- Verify timestamps, readings, raw motor turns, frame IDs, machine-zero status, positioning-reference provenance, and point retention.
- Confirm approximate positioning references are not represented as independently measured angles.
- Record clipped controls, confusing instructions, unexpected reference loss, capture blocks, or missing metadata.

Observed behaviour: Not yet observed.

Status: **NOT TESTED**

Evidence/logs: Pending generated calibration-session directory.

## Faults and deviations

- None observed yet.

## Current conclusion

The implementation and relevant offline tests have been inspected. No Windows layout or physical hardware behaviour has yet been marked as verified. Bench testing must proceed one check at a time with operator observations and saved evidence.
