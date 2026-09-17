# EAST Neutral Recovery and Bench Test Checklist

Use this checklist for the first non-clinical bench verification of version `1.2.1-safety-hardening`. This software has not been validated on hardware by this code change.

## Before launch

- [ ] Use the latest identified commit/build and record its commit hash or executable version.
- [ ] Confirm `tester_config.json` still contains the accepted `2.055` degree/turn conversion and approved provisional limits.
- [ ] Confirm automatic session continuity, phase recovery, measured-angle recovery, and watchdog are `false` unless separately verified.
- [ ] Close every other EAST GUI and motion diagnostic.
- [ ] Confirm the physical E-stop and door/interlock stop the ODrive independently of software.
- [ ] Remove the AFO for the initial empty-machine check; clear the full +/-15 degree envelope.

## Connect and inspect

- [ ] Launch `Ortho-Sim.py` or the identified `EAST.exe`.
- [ ] Press Connect and confirm the configured ODrive serial is reported.
- [ ] Confirm Connect accepts the axis only with relative setpoints, circular setpoints disabled, and the expected mapper scale; do not bypass a coordinate-mode rejection.
- [ ] Confirm no errors are automatically cleared and no configuration is saved.
- [ ] If any ODrive error is reported, stop and resolve it outside the test workflow.
- [ ] Confirm the GUI initially blocks Start and Manual Mode when reference recovery is required.
- [ ] Optionally run `python "Testing Scripts/inspect_reference.py"` with the GUI closed; save its JSON output.

## Physical neutral recovery

- [ ] Enter Operator ID and Fixture ID before setting neutral.
- [ ] Open `Verify / Recover` and read the displayed reason.
- [ ] Confirm the fixture is clear, the E-stop is accessible, and the mechanism is continuously observed.
- [ ] If movement is required, use only the 0.25 degree recovery jog. Confirm each jog idles after settling and total recovery path length cannot exceed 5 degrees, including reversals.
- [ ] Independently establish the mounted fixture/AFO at physical 90 degrees.
- [ ] Select `Set Current Physical Position as 90 deg Neutral` and confirm that this action itself causes no movement.
- [ ] Confirm the GUI changes to `VERIFIED` and displays a session-turn value. It does not need to be zero.

## Low-risk motion checks

- [ ] Start with the empty machine and a small step at the configured 1-5 degree/s manual/return speed.
- [ ] Check left/right direction against the GUI labels.
- [ ] Check manual travel clamps at +/-15 degrees from the verified neutral, not from connection position.
- [ ] Test `Return to Verified 90 deg Neutral`; confirm settle, idle, and no visible drift.
- [ ] During a low-speed move, press Stop. Confirm immediate idle and no automatic neutral return.
- [ ] Repeat the stop check with Escape and, where controlled, the physical E-stop/interlock.
- [ ] Restart the application/controller and confirm normal movement is blocked until recovery unless a separately validated continuity method has been enabled.

## Speed verification run

- [ ] Use a non-clinical dummy specimen and an independent angle/time reference.
- [ ] Record command speed, acceleration, ROM, cycles, operator, AFO/dummy ID, fixture ID, calibration ID, build/commit, date/time, and reference method.
- [ ] Run a conservative speed first, then repeat across the intended 0.1-20 degree/s range.
- [ ] Confirm the CSV command and ODrive trajectory fields change with each GUI speed.
- [ ] Confirm successful tests return to verified neutral, settle, enter confirmed idle, and remain at neutral during post-idle observation.
- [ ] Confirm Stop/fault runs are marked aborted/error and do not return automatically.

## Stop immediately if

- [ ] Direction is unexpected, speed appears excessive, or physical angle approaches a fixture limit.
- [ ] The displayed reference becomes recovery-required/faulted or feedback becomes stale/non-finite.
- [ ] ODrive idle is unconfirmed, an active/disarm error appears, or communications are intermittent.
- [ ] The fixture shifts, the load cell saturates, load changes unexpectedly, or independent angle disagrees materially with the GUI.
- [ ] CSV/metadata writing reports an error.

## Save and report

- [ ] Save the matched CSV and metadata JSON without separating or renaming only one file.
- [ ] Save the session terminal screenshot, reference status screenshot, independent measurement file/video, and any ODrive error output.
- [ ] Report whether neutral was nonzero in session turns, whether setting neutral moved the mechanism, stop-to-idle behavior, return/settle/idle behavior, and any drift.
- [ ] Report commanded versus measured speed by direction/cycle and percentage error using `analysis/verify_speed.py` plus the independent measurement.
- [ ] Record every deviation before formal AFO testing.
