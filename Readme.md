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

The configured motion-timeout ceiling is 60 seconds with a 5-second safety margin. This permits a 0.5 deg/s quasi-static endpoint-to-endpoint sweep over -10 deg to +10 deg, which requires approximately 45.01 seconds including the margin. Commands calculated to require more than 60 seconds remain blocked before motion starts.

## Running the GUI

1. On Windows install the ODrive and Phidget drivers. On macOS follow [Mac setup](docs/MAC_SETUP.md), including the Phidget driver extension if your Bridge requires it.
2. Create a Python environment. On Windows install `requirements-windows.txt` (which preserves the existing `requirements.txt` environment). On macOS install `requirements-macos.txt`; see [Mac setup](docs/MAC_SETUP.md).
3. Review `tester_config.json`, especially serial/channel values and provisional limits.
4. Run `python Ortho-Sim.py`.
5. Enter the full commanded test configuration, or select `Standard AFO Test` / `Standard Empty-Machine Baseline` and press `Load Preset`. Selecting a dropdown item alone does not change any fields. The provisional standard values are versioned in `tester_config.json`; loaded motion values remain editable and modifications are recorded.
6. Connect the ODrive. On first launch or whenever controller-session continuity cannot be proven, the GUI shows `Machine zero: SETUP REQUIRED` and blocks normal motion.
7. With the AFO removed, enter Operator ID and Fixture ID and open `Set Machine Zero - Fixture at 90 deg`. Use the supplied square and only the bounded `Jog Left`/`Jog Right` controls to align the moving fixture at physical 90 degrees to the fixed machine reference. Acknowledge the checks and select `Set Machine Zero - Fixture at 90 deg`. This records the current encoder position as machine angle 0 degrees and causes no position movement.
8. Enter Calibration ID and select `Tare Empty Machine` while the AFO and removable loads are still absent. The tare is valid only for the current application measurement session, fixture/calibration identity, Phidget device, and channel.
9. Before mounting an AFO, load `Standard Empty-Machine Baseline` (or use a validated custom baseline protocol) and run the unloaded fixture through the intended ROM/cycles. EAST assigns the reserved result AFO identifier `EMPTY_MACHINE_BASELINE` and records `test_type = empty_machine_baseline`.
10. Mount the AFO without taring again. Complete physical clearance and E-stop checks, then press Start and confirm the run summary. The test cannot start unless machine zero is verified, the fixture is at machine zero, and a matching empty-machine tare exists.

Machine zero is a control reference: the fixture is physically at 90 degrees and the software angle is 0 degrees. It is not the mounted AFO's zero-torque neutral angle. Any AFO neutral-angle estimate must be derived later from the recorded torque-angle data.

The application never assumes that `0.0` ODrive relative turns is machine zero. A successful test returns to the saved session machine zero using the dedicated return profile, confirms position and low velocity over a dwell, requests and confirms idle, and observes the zero position after idle. `Return to Machine Zero - 90 deg` provides a deliberate return after confirmation and does not require Manual Mode. `Reset Test Fields` clears only file name, AFO ID, motion inputs, and preset selection; it preserves the connection, operator/fixture/calibration IDs, machine zero, and valid tare. `Clear Session Tare` is the separate confirmed action that discards only the stored tare. `Stop` idles and aborts as applicable; none of these actions initiates a return.

A stationary empty-machine tare and a moving empty-machine baseline are different measurements. Tare records the unloaded load-cell offset at machine zero. The baseline records angle- and direction-dependent unloaded machine response over the full movement. Later corrected torque must use `AFO torque = loaded-machine torque - matched empty-machine torque`, matching fixture, calibration, machine-zero reference identity/generation, ROM, speed, acceleration, direction, and session information. EAST records those fields but does not automatically subtract a baseline.

Stop, Escape, acquisition faults, feedback faults, and communication faults request idle immediately and never initiate automatic movement. An unconfirmed idle request is reported as a fault. These controls are not substitutes for the physical E-stop.

The application accepts motion only when the connected axis explicitly reports relative, non-circular setpoints and the configured position/velocity mapper scale. The final ownership check and each target write are serialized with Stop. Confirmed motor idle does not release the motion owner; controls remain blocked until the owning worker has finished data flush and cleanup.

The persistent machine-zero record, runtime checkpoint, audit events, and process lock are stored outside the repository and frozen executable. Defaults are `%LOCALAPPDATA%\EAST` on Windows, `~/Library/Application Support/EAST` on macOS, and `$XDG_STATE_HOME/east` on Linux. `EAST_STATE_DIR` provides an explicit override for testing or managed deployment. Invalid JSON or structurally invalid state is preserved with a `.corrupt-*` suffix and cannot enable motion. Clean shutdown/disconnect continuity logic is implemented, but automatic restoration remains disabled until the units and behavior of ODrive `system_stats.uptime` are verified on the lab hardware. Phase recovery, measured-angle recovery, and the ODrive watchdog also remain disabled.

## macOS hardware GUI

macOS and Windows use the same `Ortho-Sim.py`, motion controls and configuration.
The Mac dependency file avoids Windows-only packages and includes a newer Phidget
native library; the established ODrive 0.6.8 API is unchanged. Windows dependencies
remain unchanged. Run `python Ortho-Sim.py --check-environment` to load runtime
libraries without discovering, opening or commanding hardware.

See [Mac setup and hardware check](docs/MAC_SETUP.md). Successful library and GUI
checks do not validate USB attachment, timing, stopping or physical movement.

## Session terminal and live plot

The right-hand panel contains **Session Terminal** and **Plot** tabs. Settings
remain in the left-hand panel. EAST opens on Session Terminal; starting a test
selects Plot. Switching tabs preserves terminal history and plot samples,
and both continue receiving updates throughout a run. The Plot tab displays
torque against ODrive-derived angle and resizes with the main window.

## Outputs

Each run creates a uniquely named pair in `EAST Logs`:

- `*_strain_data.csv`: raw samples, filtered values, elapsed time, motion phase and direction, machine-zero-relative angle, commanded speed/acceleration/range/cycles, test type and preset provenance, raw voltage ratio, stored tare, force/torque, raw ODrive position/velocity, converted units, and ODrive errors.
- `*_metadata.json`: operator/specimen/fixture/calibration identifiers, preset key/name/version/load time/original and actual values, the empty-machine tare record, baseline-matching fields, machine-zero record/mapping, all conversion and calibration constants, active ODrive trajectory settings, software version/Git revision, timestamps, outcome, and completion counts.

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

Motion diagnostics also require `--allow-unreferenced-diagnostic`, acquire the same exclusive hardware lock as the GUI, mark their output as unreferenced, and invalidate normal-test reference trust. Machine-zero setup in the GUI is mandatory afterwards.

Inspect persisted reference state and read-only ODrive capabilities with no hardware writes:

```text
python "Testing Scripts/inspect_reference.py"
```

Exercise the inspection path without importing hardware drivers:

```text
python "Testing Scripts/inspect_reference.py" --mock
```

For Mac-only GUI layout checks, use the preview script. It does not import ODrive, Phidget, pywinstyles, or the main hardware-control GUI:

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


### Manual calibration using motor turns

Manual Mode now has a **Degrees / Motor turns** step selector and a live display
of raw ODrive session position, displacement from verified machine zero, and
motor-derived angle. Readouts use freshness-checked feedback; disconnected or
stale data is shown as unavailable. Changing units clears the step entry.

For angle-only calibration, the PhidgetBridge and empty-machine tare are not
required for manual movement. Connect ODrive, establish physical 90 degrees
using the existing bounded machine-zero setup jogs, then enable Manual Mode.
Choose **Motor turns**, enter `1`, and click Left or Right once. The target is
current motor position plus/minus exactly one ODrive turn, independent of the
2.055 conversion for calculating the commanded step.

The existing conversion still defines conservative degree-equivalent step and
travel bounds, motor speed/acceleration and motion timeouts. Motor-turn steps
retain the existing 0.01-10 degree-equivalent step range (approximately
0.00486618-4.86618 turns at 2.055). A step that would exceed the existing overall
travel limits is rejected rather than silently shortened. Continuous Mode is
disabled while motor-turn steps are selected; the existing degree continuous
mode is unchanged. Verified zero, ownership, feedback and stop checks remain.

After each step the terminal reports requested units, initial position, settled
position and measured displacement in turns. Use the measured displacement,
rather than assuming every requested step was achieved exactly, when pairing
with independent physical-angle readings. Motor-derived degrees are estimates,
not sensor measurements. ODrive session position is not a physical absolute
angle and can change reference after a controller restart.

Align/check the square with motion disabled; powered movements retain the
existing enclosure interlock and E-stop. This update does not bypass those
protections, change the 2.055 conversion, or integrate sensor feedback into
motion control. The standalone rotary-sensor logger remains available separately. The GUI now
includes the independent stationary calibration workflow described below.


### Independent rotary-sensor calibration

EAST 1.5.0 adds **Rotary Sensor Calibration** in Manual Mode. Connect HUB0007
by its serial number and use the default **Voltage ratio** mode for the
hub-powered P3022. Position the empty fixture with the existing Manual Mode
controls, measure its actual angle independently, and capture stationary holds.
The motor command/display supplies no calibration reference. Holds record sensor
readings and raw ODrive turns; movement is blocked during capture. Cancelled and
failed holds are retained with an exclusion reason.

Calibration and Validation points remain separate. **Fit + Save Calibration
Report** produces a provisional local sensor fit, validation errors and an
independent degrees-per-motor-turn fit. No fit is applied automatically to
control, zero, tare or strain-test data. The accepted 2.055 and existing movement
protections remain in use. Record reference-instrument graduation spacing;
coarse protractor readings limit the conclusions.

See [the full calibration procedure](docs/ROTARY_SENSOR_CALIBRATION.md) for
setup, three-repeat collection, separate validation, voltage/ratio interpretation
and the automatically saved CSV/JSON files. Close other applications using the
rotary hub port before connecting. No extra Python dependencies are required
beyond the existing GUI/Phidget packages.
