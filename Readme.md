# EAST AFO Stiffness Tester

EAST controls the motorised benchtop tester and records sagittal-plane AFO angle, load, and calculated torque. The application interfaces with an ODrive S1 and a PhidgetBridge voltage-ratio input.

This repository is research software. The 2.055 AFO-degrees-per-ODrive-turn mechanical conversion is accepted. Load calibration, torque geometry, operating limits, and protocol acceptance criteria remain provisional until experimentally verified. Do not use the system for formal AFO testing until those remaining checks are complete.

## Hardware assumptions

- ODrive S1 serial number `3943355F3231`, axis 0
- PhidgetBridge channel 1; no Phidget serial is currently specified
- ODrive custom D6374 150 Kv motor
- EG Series 50:1 planetary gearbox
- DACell UU-K50 load cell
- Physical E-stop/door reed switch pulls ODrive nRST low

All software constants and limits are in `tester_config.json`. Review that file before operating the system.

## Speed definition

The GUI speed field is a commanded mounted-AFO angular speed in degrees per second. With `INPUT_MODE_TRAP_TRAJ`, the application converts this value to turns per second and writes it to:

```text
axis0.trap_traj.config.vel_limit
```

The conversion uses the configured `afo_degrees_per_odrive_turn` value. `axis0.controller.config.vel_limit` is set higher as a safety cap; it is not used as the trajectory-speed command.

The accepted conversion is 2.055 AFO degrees per ODrive turn. This value is used consistently for commanded motion, ODrive-derived angle, and ODrive-derived velocity.

The CSV angle and converted velocity use this same accepted value. Comparing them with the command checks controller execution; an independent angular reference remains useful for end-to-end verification of physical AFO speed and ROM.

The current commanded-speed range is 0.1-20 deg/s. Acceleration remains an editable commanded parameter in the range 0.1-100 deg/s^2; no acceleration value is yet validated as a final protocol setting. A test is blocked unless the full commanded ROM provides at least 5 deg of calculated constant-speed travel:

```text
calculated constant-speed span = total ROM - speed^2 / acceleration
```

This is a command-profile feasibility calculation, not an independent measurement of physical motion.

## Running the GUI

1. Install the Windows ODrive and Phidget drivers.
2. Create a Python environment and install `requirements.txt`.
3. Review `tester_config.json`, especially serial/channel values and provisional limits.
4. Run `python Ortho-Sim.py`.
5. Enter the full commanded test configuration, including speed, acceleration, angle limits, cycles, operator, AFO ID, fixture ID, and calibration ID.
6. Connect the ODrive. The application uses the configured fixed neutral target of `0.0` ODrive relative turns as 90 degrees.
7. Complete physical clearance and E-stop checks, then press Start and confirm the run summary. The test cannot start unless the fixture is at the fixed neutral reference.

After a successful test, the motor automatically returns to `0.0` turns. `Return to 90 deg (0 turns)` provides the same movement in manual mode after operator confirmation. `Reset Form` stops motion and clears the form; it does not move the mechanism. Because the ODrive exposes a relative rather than absolute position, the physical 90 degree alignment must still be verified whenever the encoder reference or mechanical setup can change.

The Stop button and Escape key request an immediate software stop and set the axis idle. They are not substitutes for the physical E-stop.

## Outputs

Each run creates a uniquely named pair in `EAST Logs`:

- `*_strain_data.csv`: raw samples, filtered values, elapsed time, motion phase, commanded speed/acceleration/range/cycles, sensor values, converted units, and ODrive errors.
- `*_metadata.json`: operator/specimen/fixture/calibration identifiers, all conversion and calibration constants, active ODrive trajectory settings, software version/Git revision, timestamps, outcome, and completion counts.

Raw sensor and ODrive columns are preserved for reprocessing. Columns labelled `ODrive-Derived` use the accepted motion conversion but are not independent measurements. Moving-average columns are derived outputs and should not replace unfiltered data in verification analyses.

## Verification

Run offline tests from the repository root:

```text
python -m unittest discover -s tests -v
```

Analyse a completed speed-verification CSV with:

```text
python analysis/verify_speed.py "EAST Logs/<run>_strain_data.csv" --output speed_results.csv
```

See `docs/VERIFICATION_PROTOCOL.md` for the pre-run checks, experimental design, stop conditions, and output requirements.

## Diagnostic scripts

Scripts under `Testing Scripts` are guarded bench diagnostics. They do nothing when imported and refuse to open or move hardware without `--confirm-hardware`. Use `--help` to see required parameters. The GUI remains the authoritative application for recorded strain tests.

For Mac-only GUI layout checks, use the preview script. It does not import ODrive, Phidget, PyQtGraph, pywinstyles, or the main hardware-control GUI:

```text
python "Testing Scripts/gui-layout-preview.py"
```

Install its lightweight dependency with:

```text
python -m pip install -r requirements-gui-preview.txt
```

## Project structure

- `Ortho-Sim.py`: GUI and coordinated hardware workflow
- `east_core.py`: hardware-independent validation, conversions, load/torque calculations, and metadata helpers
- `tester_config.json`: hardware assumptions, calibration values, limits, and sampling settings
- `analysis/verify_speed.py`: angle-time speed verification
- `Testing Scripts/gui-layout-preview.py`: Mac-safe GUI layout preview with no hardware imports
- `tests/`: dependency-free offline tests
- `docs/VERIFICATION_PROTOCOL.md`: laboratory verification procedure
- `Odrive Backup Config/`: stored ODrive hardware configuration backup

## Building the executable

`auto_exe_builder.py` builds the Windows executable with PyInstaller and includes the images and tester configuration. Build from the repository root so the expected paths resolve correctly. The generated executable is named `EAST.exe`.

The GUI header will display lab branding if these files are present:

- `images/epic_lab_logo.png`
- `images/university_of_sydney_logo.png`

## Change history

See `CHANGELOG.md`.
