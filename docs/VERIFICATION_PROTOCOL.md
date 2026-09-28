# EAST Software and Speed Verification Protocol

## Status and scope

Version 1.4.0-protocol-presets defines the GUI speed field as commanded mounted-AFO angular speed in degrees per second. The software converts it to ODrive turns per second using the accepted conversion in `tester_config.json`:

`ODrive turns/s = commanded AFO deg/s / afo_degrees_per_odrive_turn`

The accepted value is 2.055 AFO degrees per ODrive turn. It is implemented consistently for commands, ODrive-derived angles, and ODrive-derived velocities.

The current provisional command limits are 0.1-20 deg/s for speed, 0.1-100 deg/s^2 for acceleration, and 0-15 deg magnitude at each angle limit. Acceleration is operator-editable during verification and is not a validated final protocol value. The software requires at least 5 deg of calculated constant-speed travel across the full commanded ROM:

`calculated constant-speed span = total ROM - commanded speed^2 / commanded acceleration`

This check predicts the commanded trapezoidal profile only. It does not measure physical acceleration, speed, or ROM.

The maximum motion timeout is 60 seconds and the existing 5-second timeout margin remains enabled. This supports quasi-static testing at 0.5 deg/s over a -10 deg to +10 deg sweep: the calculated trapezoidal move timeout is 45.005 seconds (reported as approximately 45.01 seconds). A movement whose calculated duration plus margin exceeds 60 seconds is still rejected before motion begins; the timeout is not disabled or bypassed.

## Mandatory pre-run checks

- Confirm the physical E-stop resets the ODrive and is reachable throughout the run.
- Confirm the enclosure/door interlock works and the movement envelope is clear.
- Confirm the ODrive serial, Phidget channel/serial, fixture ID, AFO ID, and calibration ID.
- Review the provisional speed, acceleration, angle, cycle, and manual-travel limits in `tester_config.json`.
- Confirm the load-cell calibration direction, coefficient, calibration certificate, tare stability, lever arm, and geometry polynomial.
- Confirm ODrive reports no active errors before Start.
- Confirm only one EAST GUI/diagnostic process is running; the application must hold the runtime hardware lock.
- With the AFO removed, use the supplied square and bounded setup jog to align the fixture at physical 90 degrees. Confirm the GUI reports `Machine zero: VERIFIED`; this physical position is software machine angle 0 degrees.
- Confirm `Set Machine Zero - Fixture at 90 deg` records the encoder state without commanding position movement.
- At verified machine zero and before mounting the AFO, select `Tare Empty Machine` and acknowledge that the machine is unloaded. Confirm the displayed tare is valid for the current fixture, calibration, Phidget serial, and channel.
- Before mounting the AFO, load `Standard Empty-Machine Baseline` and run the unloaded fixture through the intended ROM/cycles. Confirm the result metadata reports `test_type = empty_machine_baseline`; do not identify baselines from file names.
- Mount the AFO after the empty-machine tare. Do not tare again: Start must use the stored offset so initial AFO preload is preserved.
- Treat an AFO's zero-torque neutral angle as a later analysis result, not as machine zero.
- Do not enable automatic continuity, phase recovery, measured-angle recovery, or the watchdog until the related hardware assumptions and units are independently verified and documented.
- Use a non-clinical dummy specimen for initial verification.

## Speed verification design

1. Attach an independent angular reference to the mounted AFO or rocker. A calibrated encoder is preferred; video/ImageJ tracking is acceptable if its frame timing, spatial scale, camera alignment, and angle reference are checked.
2. Test at least three commanded speeds spanning the intended protocol range. Use at least five cycles per speed and a fixed angle range large enough to contain a constant-speed region.
3. Record operator, AFO/dummy ID, fixture ID, calibration ID, test configuration, environmental notes, and independent instrument identifiers.
4. Preserve the generated CSV and matching metadata JSON without renaming one independently of the other.
5. Calculate internal ODrive-derived speed from `ODrive-Derived AFO Angle (deg)` versus `Elapsed Time (s)`. Use a linear fit within each `moving_to_max` and `moving_to_min` phase.
6. Exclude acceleration and deceleration. The supplied analysis excludes the first and last 20% of each angular excursion by default.
7. From the video/ImageJ angle-time trace, calculate independently measured speed in the constant-speed region and independently measured minimum/maximum angle at the movement reversals.
8. Compare both internal ODrive-derived and independently measured speed with the command. Compare independently measured angular limits and total ROM with the commanded range. Report direction-specific mean, standard deviation, and percentage error.

## Tare and moving baseline

The empty-machine tare is a stationary unloaded load-cell offset measured at machine zero. The empty-machine baseline is a complete unloaded torque-angle movement that captures angle- and direction-dependent machine response. They are not interchangeable.

For later correction, use `AFO torque = loaded-machine torque - matched empty-machine torque`. Match baseline and loaded runs by Fixture ID, Calibration ID, machine-zero reference identity/generation, ROM, speed, acceleration, movement direction, and applicable session information. EAST records the matching fields but does not automatically perform the subtraction.

The CSV `ODrive-Derived AFO Angle (deg)` and `ODrive-Derived AFO Velocity (deg/s)` columns both use the same accepted 2.055 degree/turn conversion as the command. Angle is relative to verified machine zero. Command-versus-CSV analysis checks trajectory execution within the ODrive-derived coordinate system. The independent angular reference provides end-to-end verification of physical speed and ROM rather than re-validating the accepted conversion. The analysis tool continues to accept the legacy `Raw AFO Angle (deg)` heading for older files.

Run the supplied analysis with:

```text
python analysis/verify_speed.py "EAST Logs/<run>_strain_data.csv" --output speed_results.csv
```

## Acceptance criteria

Acceptance limits must be approved in the thesis protocol before formal testing. Do not infer an acceptance threshold from the software defaults. At minimum, assess mean error, repeatability across cycles, directional asymmetry, and whether a constant-speed region exists at each commanded speed.

## Stop conditions

Stop and set the system idle if motion is in the wrong direction, an angle approaches the physical fixture limit, speed appears excessive, load changes unexpectedly, the fixture moves, the load cell saturates, sampling becomes intermittent, the GUI reports an ODrive error/timeout, or the independent angle trace disagrees materially with the application.

## Output traceability

Each run produces:

- A CSV with wall-clock and monotonic elapsed time, command values, test type, preset name/version/modified state, motion phase and direction, machine-zero-relative raw/averaged angle, raw/averaged load/torque, raw ODrive position/velocity, converted AFO velocity, raw voltage ratio, stored tare offset, and ODrive error state.
- A JSON sidecar with test parameters, operator/AFO/fixture/calibration identifiers, complete preset provenance, empty-machine tare metadata, baseline-matching metadata, calibration and geometry constants, ODrive configuration snapshot, software version, Git commit, start/end time, run outcome, samples, and completed cycles.
- The JSON sidecar also contains the initial and final machine-zero record, verification method, session mapping, reference confidence/reason, and runtime-state directory.

For every successful run, verify that the session log reports machine-zero settling, confirmed ODrive idle, and a completed post-idle observation. A run must not be classified as completed if CSV/metadata writing, machine-zero return, idle confirmation, or post-idle observation fails.

Automatic mapping restoration after clean app restart/manual disconnect remains disabled until the units and reset behavior of the EAST ODrive's `system_stats.uptime` field are verified on hardware. The continuity code and tests must not be treated as bench validation.

Formal AFO stiffness testing must not begin while the load-cell calibration, torque geometry, safety limits, or acceptance criteria remain unverified.
