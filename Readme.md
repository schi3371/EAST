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
6. Connect the ODrive. On first launch or whenever controller-session continuity cannot be proven, the GUI shows `RECOVERY REQUIRED` and blocks normal motion.
7. Enter Operator ID and Fixture ID, select `Verify / Recover`, use only the bounded slow jog if required, physically align the mounted fixture/AFO at 90 degrees, acknowledge the checks, and select `Set Current Physical Position as 90 deg Neutral`. Setting neutral records state and does not move the motor.
8. Complete physical clearance and E-stop checks, then press Start and confirm the run summary. The test cannot start unless the verified neutral is fresh and the mechanism is at neutral.

The application never assumes that `0.0` relative turns is physical neutral. A successful test returns to the verified session neutral using the dedicated return profile, confirms position and low velocity over a dwell, requests and confirms idle, and observes the neutral position after idle. `Return to Verified 90 deg Neutral` provides a deliberate manual return after confirmation. `Reset Form` and `Stop` stop motion and clear/abort as applicable; neither initiates a return.

Stop, Escape, acquisition faults, feedback faults, and communication faults request idle immediately and never initiate automatic movement. An unconfirmed idle request is reported as a fault. These controls are not substitutes for the physical E-stop.

The persistent reference record, runtime checkpoint, audit events, and process lock are stored outside the repository and frozen executable. Defaults are `%LOCALAPPDATA%\EAST` on Windows, `~/Library/Application Support/EAST` on macOS, and `$XDG_STATE_HOME/east` on Linux. `EAST_STATE_DIR` provides an explicit override for testing or managed deployment. Automatic cross-session continuity, phase recovery, measured-angle recovery, and the ODrive watchdog are implemented but disabled until their hardware assumptions are experimentally verified.

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

Motion diagnostics also require `--allow-unreferenced-diagnostic`, acquire the same exclusive hardware lock as the GUI, mark their output as unreferenced, and invalidate normal-test reference trust. Physical neutral recovery in the GUI is mandatory afterwards.

Inspect persisted reference state and read-only ODrive capabilities with no hardware writes:

```text
python "Testing Scripts/inspect_reference.py"
```

Exercise the inspection path without importing hardware drivers:

```text
python "Testing Scripts/inspect_reference.py" --mock
```

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
- `east_reference.py`: persistent reference state, atomic checkpoints, process lock, motion ownership, and settle validation
- `east_odrive.py`: serialized ODrive reads/writes and read-only capability inspection
- `tester_config.json`: hardware assumptions, calibration values, limits, and sampling settings
- `analysis/verify_speed.py`: angle-time speed verification
- `Testing Scripts/gui-layout-preview.py`: Mac-safe GUI layout preview with no hardware imports
- `Testing Scripts/inspect_reference.py`: read-only live/mock reference and capability inspection
- `tests/`: hardware-independent offline safety, conversion, logging, and analysis tests
- `docs/VERIFICATION_PROTOCOL.md`: laboratory verification procedure
- `docs/NEUTRAL_REFERENCE_DESIGN.md`: reference state, persistence, motion lifecycle, and disabled-feature assumptions
- `docs/NEUTRAL_RECOVERY_CHECKLIST.md`: first bench recovery and speed-test checklist
- `Odrive Backup Config/`: stored ODrive hardware configuration backup

## Building the executable

`auto_exe_builder.py` builds the Windows executable with PyInstaller and includes the images and tester configuration. Build from the repository root so the expected paths resolve correctly. The generated executable is named `EAST.exe`.

The GUI header will display lab branding if these files are present:

- `images/epic_lab_logo.png`
- `images/university_of_sydney_logo.png`

## Change history

See `CHANGELOG.md`.
