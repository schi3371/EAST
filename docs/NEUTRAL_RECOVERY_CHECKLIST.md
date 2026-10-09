# EAST Machine-Zero, Tare, and Bench Test Checklist

Use this checklist for the first non-clinical bench verification of version `1.4.0-protocol-presets`. This software has not been validated on hardware by this code change.

## Before launch

- [ ] Use the latest identified commit/build and record its commit hash or executable version.
- [ ] Confirm `tester_config.json` still contains the accepted `2.055` degree/turn conversion and approved provisional limits.
- [ ] Confirm automatic session continuity is `false` and `odrive_uptime_units` is `null` until `system_stats.uptime` is bench-verified. Confirm phase recovery, measured-angle recovery, and watchdog are also disabled.
- [ ] Close every other EAST GUI and motion diagnostic.
- [ ] Confirm the physical E-stop and door/interlock stop the ODrive independently of software.
- [ ] Remove the AFO for the initial empty-machine check; clear the full +/-30 degree envelope.

## Connect and inspect

- [ ] Launch `Ortho-Sim.py` or the identified `EAST.exe`.
- [ ] Press Connect and confirm the configured ODrive serial is reported.
- [ ] Confirm Connect accepts the axis only with relative setpoints, circular setpoints disabled, and the expected mapper scale; do not bypass a coordinate-mode rejection.
- [ ] Confirm no errors are automatically cleared and no configuration is saved.
- [ ] If any ODrive error is reported, stop and resolve it outside the test workflow.
- [ ] Confirm the GUI initially blocks Start and Manual Mode when machine-zero setup is required, while leaving the setup button available.
- [ ] Optionally run `python "Testing Scripts/inspect_reference.py"` with the GUI closed; save its JSON output.

## Machine-zero setup

- [ ] Remove the AFO and removable loads. Enter Operator ID and Fixture ID before setting machine zero.
- [ ] Open `Set Machine Zero - Fixture at 90 deg` and read the displayed reason.
- [ ] Confirm the fixture is clear, the E-stop is accessible, and the mechanism is continuously observed.
- [ ] Use the supplied square to align the moving fixture at physical 90 degrees to the fixed machine reference.
- [ ] If movement is required, use only the 0.25 degree `Jog Left`/`Jog Right` controls. Confirm each jog has a bounded per-move timeout, idles after settling, and total setup path length cannot exceed 5 degrees including reversals. Confirm the dialog does not expire merely because 120 seconds pass.
- [ ] Select `Set Machine Zero - Fixture at 90 deg` and confirm that the action itself causes no position movement.
- [ ] Confirm the GUI changes to `Machine zero: VERIFIED` and displays a session-turn value. That ODrive value does not need to be zero; the corresponding software machine angle is 0 degrees.

## Empty-machine tare

- [ ] Keep the AFO and removable loads off the fixture and ensure nothing touches/preloads the load cell.
- [ ] Enter Calibration ID and select `Tare Empty Machine`; acknowledge the unloaded condition.
- [ ] Confirm `Empty-machine tare: VALID` displays an offset and timestamp.
- [ ] Before mounting the AFO, load `Standard Empty-Machine Baseline`, confirm the displayed five motion values, and retain the unloaded movement curve for matched baseline subtraction.
- [ ] Mount the AFO only after the tare is complete. Do not tare again after mounting it.
- [ ] Confirm a temporary Fixture ID or Calibration ID mismatch retains the tare, displays `IDENTITY MISMATCH`, blocks Start, and automatically returns to `VALID` after correction without invalidating machine zero.
- [ ] Confirm `Reset Test Fields` clears only file/AFO/motion fields and returns Protocol to Custom while preserving operator/fixture/calibration, machine zero, connection, and tare.
- [ ] Confirm `Clear Session Tare` requires confirmation, clears only the session tare, and preserves machine zero, connection, identifiers, and test fields.
- [ ] Confirm any later machine-zero setup jog or re-establishment invalidates the tare.
- [ ] Confirm Start detects and rejects a different connected Phidget serial/channel.

## Low-risk motion checks

- [ ] Start with the empty machine and a small step at the configured 1-5 degree/s manual/return speed.
- [ ] Check left/right direction against the GUI labels.
- [ ] Check manual travel clamps at +/-30 degrees from verified machine zero, not from connection position.
- [ ] Test `Return to Machine Zero - 90 deg` without enabling Manual Mode; confirm settle, idle, and no visible drift.
- [ ] During a low-speed move, press Stop. Confirm immediate idle and no automatic machine-zero return.
- [ ] Repeat the stop check with Escape and, where controlled, the physical E-stop/interlock.
- [ ] Perform a clean manual disconnect and clean app close. Confirm a clean checkpoint is written only after confirmed idle and fresh error-free feedback.
- [ ] Restart/reconnect and confirm normal movement remains blocked because automatic continuity is intentionally disabled pending ODrive uptime-unit verification.

## Speed verification run

- [ ] Use a non-clinical dummy specimen and an independent angle/time reference.
- [ ] Confirm Custom initially has empty motion fields. Confirm each standard preset loads 3 cycles, +/-10 degrees, 5 degrees/s, and 100 degrees/s2 without replacing session identifiers.
- [ ] Record command speed, acceleration, ROM, cycles, operator, AFO/dummy ID, fixture ID, calibration ID, build/commit, date/time, and reference method.
- [ ] Run a conservative speed first, then repeat across the intended 0.1-20 degree/s range.
- [ ] Confirm the CSV command and ODrive trajectory fields change with each GUI speed.
- [ ] Confirm successful tests return to machine zero, settle, enter confirmed idle, and remain at zero during post-idle observation.
- [ ] Confirm Start does not perform another tare and that mounted-AFO preload remains visible in raw force/torque.
- [ ] Confirm Stop/fault runs are marked aborted/error and do not return automatically.

## Stop immediately if

- [ ] Direction is unexpected, speed appears excessive, or physical angle approaches a fixture limit.
- [ ] Machine zero becomes setup-required/faulted or feedback becomes stale/non-finite.
- [ ] ODrive idle is unconfirmed, an active/disarm error appears, or communications are intermittent.
- [ ] The fixture shifts, the load cell saturates, load changes unexpectedly, or independent angle disagrees materially with the GUI.
- [ ] CSV/metadata writing reports an error.

## Save and report

- [ ] Save the matched CSV and metadata JSON without separating or renaming only one file.
- [ ] Save the session terminal screenshot, machine-zero and tare status screenshot, independent measurement file/video, and any ODrive error output.
- [ ] Report the machine-zero session-turn value, whether setting zero moved the mechanism, tare offset/repeatability, stop-to-idle behavior, return/settle/idle behavior, and drift.
- [ ] Report commanded versus measured speed by direction/cycle and percentage error using `analysis/verify_speed.py` plus the independent measurement.
- [ ] Record every deviation before formal AFO testing.
