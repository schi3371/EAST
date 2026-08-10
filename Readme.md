# OrthoSim - EAST AFO Stiffness Tester

OrthoSim controls the EAST motorised benchtop tester and records sagittal-plane AFO angle, load, and calculated torque. The application interfaces with an ODrive S1 and a PhidgetBridge voltage-ratio input.

This repository is research software. Version 1.1.0 improves safety and traceability, but the configured mechanical conversion, load calibration, torque geometry, operating limits, and protocol acceptance criteria remain provisional until experimentally verified. Do not use the system for formal AFO testing until those checks are complete.

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

The current conversion is 2.055 AFO degrees per ODrive turn and is explicitly provisional. The implementation is dimensionally consistent, but experimental comparison with an independent angular reference is still required.

The CSV angle and converted velocity use this same value. Comparing them with the command checks controller execution conditional on the assumed conversion; it does not independently prove physical AFO angular speed.

## Running the GUI

1. Install the Windows ODrive and Phidget drivers.
2. Create a Python environment and install `requirements.txt`.
3. Review `tester_config.json`, especially serial/channel values and provisional limits.
4. Run `python Ortho-Sim.py`.
5. Enter the full test configuration, including operator, AFO ID, fixture ID, and calibration ID.
6. Connect the ODrive. Connection leaves the axis idle in test mode.
7. Complete physical clearance and E-stop checks, then press Start and confirm the run summary.

The Stop button and Escape key request an immediate software stop and set the axis idle. They are not substitutes for the physical E-stop.

## Outputs

Each run creates a uniquely named pair in `OrthoSim Logs`:

- `*_strain_data.csv`: raw samples, filtered values, elapsed time, motion phase, command values, sensor values, converted units, and ODrive errors.
- `*_metadata.json`: operator/specimen/fixture/calibration identifiers, all conversion and calibration constants, active ODrive trajectory settings, software version/Git revision, timestamps, outcome, and completion counts.

Raw columns are preserved for reprocessing. Moving-average columns are derived outputs and should not replace raw data in verification analyses.

## Verification

Run offline tests from the repository root:

```text
python -m unittest discover -s tests -v
```

Analyse a completed speed-verification CSV with:

```text
python analysis/verify_speed.py "OrthoSim Logs/<run>_strain_data.csv" --output speed_results.csv
```

See `docs/VERIFICATION_PROTOCOL.md` for the pre-run checks, experimental design, stop conditions, and output requirements.

## Diagnostic scripts

Scripts under `Testing Scripts` are guarded bench diagnostics. They do nothing when imported and refuse to open or move hardware without `--confirm-hardware`. Use `--help` to see required parameters. The GUI remains the authoritative application for recorded strain tests.

## Project structure

- `Ortho-Sim.py`: GUI and coordinated hardware workflow
- `east_core.py`: hardware-independent validation, conversions, load/torque calculations, and metadata helpers
- `tester_config.json`: hardware assumptions, calibration values, limits, and sampling settings
- `analysis/verify_speed.py`: angle-time speed verification
- `tests/`: dependency-free offline tests
- `docs/VERIFICATION_PROTOCOL.md`: laboratory verification procedure
- `Odrive Backup Config/`: stored ODrive hardware configuration backup

## Building the executable

`auto_exe_builder.py` builds the Windows executable with PyInstaller and includes the images and tester configuration. Build from the repository root so the expected paths resolve correctly.

## Change history

See `CHANGELOG.md`.
