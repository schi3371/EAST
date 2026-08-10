# Changelog

## 1.1.0 - 2026-08-10

- Defined GUI speed as commanded AFO angular speed in degrees per second.
- Replaced conflicting motion factors with one provisional, configurable degree/turn conversion.
- Applied commanded speed to the ODrive trapezoidal trajectory velocity field.
- Added bounded validation for cycles, speed, acceleration, and angle magnitudes.
- Added interruptible motion timeouts, ODrive error checks, immediate idle stop, and guaranteed idle cleanup.
- Added bounded manual movement and an Escape-key software stop.
- Removed duplicate acquisition threads and duplicate-sample suppression.
- Added monotonic elapsed time, motion phase, commands, raw sensor values, physical units, and ODrive state to CSV output.
- Added operator, AFO, fixture, and calibration identifiers plus per-run JSON metadata.
- Added unique filenames, software/Git traceability, run outcome, and completion summaries.
- Replaced unsafe legacy testing scripts with guarded command-line diagnostics.
- Added offline conversion/calculation tests and a speed-verification analysis tool.
- Documented provisional constants and the required physical verification protocol.
