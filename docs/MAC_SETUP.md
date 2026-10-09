# EAST on macOS

Use the same `Ortho-Sim.py` as on Windows. The separate GUI preview remains a
hardware-free layout tool. This setup changes no controller firmware, conversion,
calibration, travel limit, timeout, reference rule or enclosure interlock.

## Install and launch

Use Python 3.11 with working Tk support (the python.org macOS installer includes
Tk). Native Apple Silicon Python is supported by the pinned ODrive library;
Rosetta is not required. From the repository directory:

```sh
python3.11 -m venv .venv
.venv/bin/python -m pip install -r requirements-macos.txt
.venv/bin/python Ortho-Sim.py --check-environment
.venv/bin/python Ortho-Sim.py
```

If `.venv` already exists with suitable Python/Tk, reuse it and start with the
dependency installation command. Select `.venv/bin/python` as the VS Code
interpreter. Do not install the legacy Windows `requirements.txt` on the Mac.
Windows users can continue using that file or `requirements-windows.txt`.

`--check-environment` imports Python libraries and loads native libraries only.
It does not construct the GUI, discover USB devices, acquire the EAST hardware
lock, create reference state, connect to hardware or issue motor commands.

The Mac pins ODrive 0.6.8, matching the application API, and Phidget22
1.26.20260821, matching the standalone rotary-sensor diagnostic. The newer Phidget
package includes its native macOS library. Legacy HID Phidgets additionally need
the Phidget Control Panel/driver extension; approve that through the official
installer when required. The Standalone Control Panel is already installed on
the development Mac, but physical Bridge attachment remains to be checked.

Official references:
- [ODrive Python package](https://docs.odriverobotics.com/v/latest/guides/python-package.html)
- [Phidget22 Python package and macOS notes](https://pypi.org/project/phidget22/)
- [Phidgets macOS installation](https://www.phidgets.com/docs/OS_-_macOS)

## First hardware check

1. Close other EAST, odrivetool and diagnostic sessions. Close any Control Panel
   channel windows using the load cell or rotary sensor.
2. Connect the ODrive using the existing USB isolator and a data-capable cable.
   Keep the fixture clear and the enclosure closed for powered movement.
3. Launch EAST and press Connect ODrive. Check that the configured serial is
   recognised and fresh raw motor turns appear. Connection requests idle;
   it does not automatically command a position move.
4. Establish machine zero at physical 90 degrees using the bounded setup jogs
   and square. The Mac has its own reference state; do not copy a Windows
   session reference into it.
5. Check a small bounded Manual Mode step, its settled displacement, Stop/Escape,
   return to machine zero, and disconnect/reconnect. Verify actual behaviour
   before using one-turn calibration steps or recorded runs.
6. For load-cell testing, connect the PhidgetBridge, check the configured channel,
   then follow the existing empty-machine tare/baseline workflow. Manual Mode
   does not require the Bridge/tare; a strain test does.
7. Check normal Quit while idle, then restart. Automatic reference-continuity
   restoration remains disabled. Log hardware test outcomes and any failures.

The P3022 is available as a separate measurement diagnostic and through the
[GUI stationary calibration window](ROTARY_SENSOR_CALIBRATION.md). It has not
been added to EAST motor control or the strain-test angle columns.

## Troubleshooting

- Missing module: use the repository `.venv/bin/python` and reinstall the Mac
  requirements in that environment.
- Phidget framework/library error: check the installed Phidget22 version. The
  older Windows pin expects `/Library/Frameworks/Phidget22.framework`; use the
  Mac requirement, rather than modifying load-cell calibration or wiring.
- No ODrive found: check motor-controller power, enclosure/interlock state,
  data cable/isolator, serial identity and competing applications first. Do not
  reset configuration or update firmware to troubleshoot the Mac connection.
- GUI/Tk error or crash: record Python/Tk/macOS versions and the error. A
  rendered window alone does not verify normal shutdown or motor operation.

## Development verification — 9 October 2026

Verified on Apple Silicon, macOS 26.3, Python 3.11.2 and Tk 8.6:

- Runtime check loaded ODrive 0.6.8 and the native Phidget22 1.26 library.
- Native VoltageRatioInput creation and close succeeded without opening a device.
- The actual hardware GUI, plot and About dialog rendered and idle shutdown
  completed successfully (exit code 0). Device discovery/creation was mocked to
  raise on any unexpected startup hardware access; neither was called.
- 106 EAST tests and 8 standalone rotary-sensor tests passed; compilation and
  diff checks passed.

The rotary window also rendered and completed mocked capture/export, cancellation
and normal idle shutdown without opening devices. It uses a modeless transient
child window to avoid macOS automatic window tab grouping.

The ODrive USB connection, PhidgetBridge attachment, physical motion, acquisition
and stopping behaviour have not yet been tested on this Mac. Windows hardware
operation has not been retested; its existing dependency versions are preserved.
