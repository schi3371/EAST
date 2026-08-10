# EAST Software and Speed Verification Protocol

## Status and scope

Version 1.1.0 defines the GUI speed field as commanded mounted-AFO angular speed in degrees per second. The software converts it to ODrive turns per second using the single provisional calibration in `tester_config.json`:

`ODrive turns/s = commanded AFO deg/s / afo_degrees_per_odrive_turn`

The configured value is currently 2.055 AFO degrees per ODrive turn. This definition is implemented consistently, but the physical conversion is not considered validated until it is checked against an independent angle reference.

## Mandatory pre-run checks

- Confirm the physical E-stop resets the ODrive and is reachable throughout the run.
- Confirm the enclosure/door interlock works and the movement envelope is clear.
- Confirm the ODrive serial, Phidget channel/serial, fixture ID, AFO ID, and calibration ID.
- Review the provisional speed, acceleration, angle, cycle, and manual-travel limits in `tester_config.json`.
- Confirm the load-cell calibration direction, coefficient, calibration certificate, tare stability, lever arm, and geometry polynomial.
- Confirm ODrive reports no active errors before Start.
- Use a non-clinical dummy specimen for initial verification.

## Speed verification design

1. Attach an independent angular reference to the mounted AFO or rocker. A calibrated encoder is preferred; video tracking is acceptable if its timing and angle scale are independently checked.
2. Test at least three commanded speeds spanning the intended protocol range. Use at least five cycles per speed and a fixed angle range large enough to contain a constant-speed region.
3. Record operator, AFO/dummy ID, fixture ID, calibration ID, test configuration, environmental notes, and independent instrument identifiers.
4. Preserve the generated CSV and matching metadata JSON without renaming one independently of the other.
5. Calculate speed from `Raw AFO Angle (deg)` versus `Elapsed Time (s)`. Use a linear fit within each `moving_to_max` and `moving_to_min` phase.
6. Exclude acceleration and deceleration. The supplied analysis excludes the first and last 20% of each angular excursion by default.
7. Compare software-derived speed with both the command and the independent reference. Report direction-specific mean, standard deviation, and percentage error.

The CSV `Raw AFO Angle (deg)` and `Measured AFO Velocity (deg/s)` columns both use the same configured 2.055 degree/turn conversion as the command. Command-versus-CSV analysis therefore checks trajectory execution only under the assumed conversion; it cannot independently validate that conversion or prove the physical AFO speed. The independent angular reference is required for that claim.

Run the supplied analysis with:

```text
python analysis/verify_speed.py "OrthoSim Logs/<run>_strain_data.csv" --output speed_results.csv
```

## Acceptance criteria

Acceptance limits must be approved in the thesis protocol before formal testing. Do not infer an acceptance threshold from the software defaults. At minimum, assess mean error, repeatability across cycles, directional asymmetry, and whether a constant-speed region exists at each commanded speed.

## Stop conditions

Stop and set the system idle if motion is in the wrong direction, an angle approaches the physical fixture limit, speed appears excessive, load changes unexpectedly, the fixture moves, the load cell saturates, sampling becomes intermittent, the GUI reports an ODrive error/timeout, or the independent angle trace disagrees materially with the application.

## Output traceability

Each run produces:

- A CSV with wall-clock and monotonic elapsed time, command values, motion phase, raw and averaged angle/load/torque, raw ODrive position/velocity, converted AFO velocity, raw voltage ratio, tare offset, and ODrive error state.
- A JSON sidecar with test parameters, operator/AFO/fixture/calibration identifiers, calibration and geometry constants, ODrive configuration snapshot, software version, Git commit, start/end time, run outcome, samples, and completed cycles.

Formal AFO stiffness testing must not begin while the motion conversion, load-cell calibration, torque geometry, safety limits, or acceptance criteria remain unverified.
