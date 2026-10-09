# P3022 quick communication test

This standalone test runs on macOS and Windows. It reads the existing
PandAuto P3022-V1-CW360 through a Phidgets HUB0007, port 0, using VoltageInput
and `setIsHubPortDevice(True)`. It does not import EAST, open ODrive, open the
load-cell Bridge, command any movement, change a tare, or establish machine zero.

## Before connecting

Disconnect USB power before wiring. Based on YOUR sensor's printed label:

| P3022 wire | Function | Phidget cable wire |
| --- | --- | --- |
| White | VCC / +5 V | Red |
| Blue | Vout / signal | White |
| Black | GND | Black |

Verify against the label and actual plug positions. Insulate each joint and
check for shorts before plugging into USB. For this check, keep motor power
disconnected. Rotate only a freely accessible, mechanically uncoupled sensor
shaft; do not force the powered-down machine/gearbox or bypass its interlock.
A connected hub is not proof that the sensor itself is connected correctly.

## macOS: already prepared on this Mac

Open this folder in VS Code. Its `.venv` is an isolated Python environment
containing the Phidgets package and its native Mac library. In VS Code select
this folder's `.venv/bin/python` interpreter, or use the terminal:

```sh
.venv/bin/python p3022_quick_test.py --list
.venv/bin/python p3022_quick_test.py
```

The default test runs for 60 seconds. For a longer test:

```sh
.venv/bin/python p3022_quick_test.py --duration 0
```

Stop with Ctrl+C. If you click VS Code's Run Python File button, choose
**Run Python File in Terminal** so you can see the readings.

## Windows: setup on the other computer

Copy this folder, excluding `.venv` and `results`. A Mac environment cannot
be reused on Windows. In a VS Code terminal within the copied folder:

```powershell
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe p3022_quick_test.py --list
.\.venv\Scripts\python.exe p3022_quick_test.py
```

If native-library loading or discovery fails, install/open the official
Phidget Control Panel for that OS and check that HUB0007 appears. Current pip
packages include the native library on supported Mac/Windows platforms;
installing a Python package into a different interpreter will not help the
interpreter selected in VS Code.

Close any Control Panel **Voltage Input test window** before running the
script, because two programs cannot own the same channel simultaneously.
The Control Panel's main discovery window can remain open.

## What to look for

1. `--list` should show HUB0007 and its serial number. Discovery does not
   configure or open the sensor input.
2. The live test should show voltage and an **estimated shaft angle**.
3. If the shaft can safely be rotated independently, voltage should change
   smoothly and return near the original value when returned to that position.
4. Keep the shaft stationary, record the voltage, stop the script, unplug/replug
   the hub, and rerun. Compare the voltage at that unchanged physical position.
   The script does not zero itself at startup.
5. A voltage near 0 V, 5 V, or a fluctuating voltage alone does not prove correct
   operation: an endpoint or a disconnected/floating input can look similar.
   Check repeatable response to known shaft movement before drawing conclusions.

CSV files go into `results/` beside the script. Timestamps are **host callback
arrival times**, not sensor hardware timestamps; this is a communication test,
not validated speed acquisition. Original nominal 0-5 V angle fields are retained for compatibility. Their flags
are NOT sensor-health flags. Separate estimated-angle fields are populated for
finite readings inside the hub's 0-5.3 V electrical range, including readings above
5 V. Raw voltages are never clipped or silently discarded.

The displayed estimate is **360 x voltage / assumed full-scale voltage**.
The default full-scale assumption is 5 V, matching the original nominal test.
Select another explicit assumption with `--full-scale-voltage`. For exploratory
estimates using the operator-reported 5.19 V value:

```sh
.venv/bin/python p3022_quick_test.py --serial 750256 --full-scale-voltage 5.19
```

On Windows use `.\.venv\Scripts\python.exe` instead of `.venv/bin/python`.
**5.19 V is an assumed endpoint, not a calibration or a live VCC measurement.**
The output has not yet been confirmed to follow its supply exactly. USB supply
variation, endpoint offsets, coupling and nonlinearity remain unverified.

Angles above 360 degrees are explicitly marked as linear extrapolations above
assumed full scale. They are not clipped or wrapped to zero; a reading above the
assumed endpoint does not establish a completed revolution. Non-finite readings
or readings outside the hub's 0-5.3 V input range remain saved but have no estimate.
The final observed-voltage summary includes **all finite readings**, including
above 5 V and outside the electrical range.

New CSV columns:

- `estimated_shaft_angle_deg`: uncalibrated linear estimate, not an EAST fixture angle.
- `angle_estimate_full_scale_voltage_v`: explicit assumed scaling endpoint.
- `angle_estimate_basis`: assumption provenance; never marked calibrated.
- `electrical_range_valid`: finite and within the hub input specification.
- `angle_estimate_status`: uncalibrated, extrapolated, outside electrical range, or non-finite.

The legacy `nominal_shaft_angle_deg`, `nominal_relative_angle_deg` and
`nominal_range_valid` retain the original 5 V convention. The optional relative
console display is labelled **legacy nominal** and remains blank above 5 V.
Use the new estimate column when reviewing the provisional absolute angle.

## Add estimates to existing recordings

The backfill tool appends the new columns while preserving every original column,
row, timestamp, voltage and old validity flag. It validates all files first.
An explicit full-scale assumption is required; it is not recorded as a measured
historical supply. Close a running logger before updating its CSVs.

Preview only:

```sh
.venv/bin/python backfill_p3022_estimates.py results --full-scale-voltage 5.19
```

Apply after preview:

```sh
.venv/bin/python backfill_p3022_estimates.py results --full-scale-voltage 5.19 --apply
```

Before replacing any file, the tool saves byte-for-byte originals beneath
`results/originals_before_angle_estimates/<timestamp>/`. A migration manifest
records checksums, row counts, selected assumption and completed files. Each CSV
replacement is atomic. Files with estimate columns are refused, preventing an
accidental repeat from replacing provenance.

Recordings from different USB setups must not be treated as having a measured
5.19 V supply merely because the exploratory estimate uses that assumption.
Keep backups and raw measurements for later calibrated reanalysis.

Focused verification (no hardware):

```sh
.venv/bin/python -m unittest discover -s . -p 'test_p3022_estimates.py' -v
```

For an optional relative display, manually record a reference voltage and supply
that SAME value after a restart, for example:

```sh
.venv/bin/python p3022_quick_test.py --reference-voltage 2.5000
```

The relative display wraps into -180 to +180 degrees, is single-turn only,
and does not store or change the EAST machine-zero reference. Replace 2.5000
with your measured value; do not assume that 2.5 V is physical machine zero.
Use `--direction -1` if you need to reverse the display convention.

If multiple HUB0007 devices are connected, select one with `--serial NUMBER`.
The script stops on detach, missing fresh readings, or reported Phidget errors;
it never substitutes simulated data for a failed hardware connection.

## Documentation

- [HUB0007 specifications and channel modes](https://www.phidgets.com/?prodid=1290)
- [Phidgets Python setup](https://www.phidgets.com/docs/Language_-_Python)
- [macOS setup](https://www.phidgets.com/docs/OS_-_macOS)
- [P3022 manufacturer datasheet](https://www.pandauto.com/static/upload/file/20210820/1629473353113298.pdf)

`--demo --duration 2` checks display and CSV writing using explicitly labelled
simulated data, without any Phidgets hardware or package being required.
