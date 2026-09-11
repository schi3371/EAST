# EAST Software and Speed Verification Protocol

## Status and scope

Version 1.1.0 defines the GUI speed field as commanded mounted-AFO angular speed in degrees per second. The software converts it to ODrive turns per second using the accepted conversion in `tester_config.json`:

`ODrive turns/s = commanded AFO deg/s / afo_degrees_per_odrive_turn`

The accepted value is 2.055 AFO degrees per ODrive turn. It is implemented consistently for commands, ODrive-derived angles, and ODrive-derived velocities.

The current provisional command limits are 0.1-20 deg/s for speed, 0.1-100 deg/s^2 for acceleration, and 0-12 deg magnitude at each angle limit. Acceleration is operator-editable during verification and is not a validated final protocol value. The software requires at least 5 deg of calculated constant-speed travel across the full commanded ROM:

`calculated constant-speed span = total ROM - commanded speed^2 / commanded acceleration`

This check predicts the commanded trapezoidal profile only. It does not measure physical acceleration, speed, or ROM.

## Mandatory pre-run checks

- Confirm the physical E-stop resets the ODrive and is reachable throughout the run.
- Confirm the enclosure/door interlock works and the movement envelope is clear.
- Confirm the ODrive serial, Phidget channel/serial, fixture ID, AFO ID, and calibration ID.
- Review the provisional speed, acceleration, angle, cycle, and manual-travel limits in `tester_config.json`.
- Confirm the load-cell calibration direction, coefficient, calibration certificate, tare stability, lever arm, and geometry polynomial.
- Confirm ODrive reports no active errors before Start.
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

The CSV `ODrive-Derived AFO Angle (deg)` and `ODrive-Derived AFO Velocity (deg/s)` columns both use the same accepted 2.055 degree/turn conversion as the command. Command-versus-CSV analysis checks trajectory execution within the ODrive-derived coordinate system. The independent angular reference provides end-to-end verification of physical speed and ROM rather than re-validating the accepted conversion. The analysis tool continues to accept the legacy `Raw AFO Angle (deg)` heading for older files.

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

- A CSV with wall-clock and monotonic elapsed time, command values, motion phase, raw and averaged angle/load/torque, raw ODrive position/velocity, converted AFO velocity, raw voltage ratio, tare offset, and ODrive error state.
- A JSON sidecar with test parameters, operator/AFO/fixture/calibration identifiers, calibration and geometry constants, ODrive configuration snapshot, software version, Git commit, start/end time, run outcome, samples, and completed cycles.

Formal AFO stiffness testing must not begin while the load-cell calibration, torque geometry, safety limits, or acceptance criteria remain unverified.
