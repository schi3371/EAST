# Set up EAST on a Windows computer

This section is for the person preparing the control computer. Once setup is complete, operators launch EAST by double-clicking the **EAST** desktop shortcut. The shortcut starts the application and leaves hardware connection to the operator.

## Prerequisites

- A lab-managed 64-bit Windows 10 or 11 computer with USB data ports, a local user-writable installation folder, and access to the approved EAST release and Python package index during setup.
- 64-bit CPython 3.10 with Tkinter and the Windows Python launcher. Confirm that `py -3.10 --version` reports Python 3.10. This version is the proposed deployment target for the existing pinned Windows requirements.
- The Phidget22 Windows driver and Control Panel. With the hardware in the normal safe setup state, confirm the PhidgetBridge and rotary-sensor interface appear in the Control Panel. Close device test windows before starting EAST.
- The ODrive S1 native USB interface. Check Windows Device Manager for the ODrive native interface and WinUSB. Use the lab-approved driver setup and retain the controller's existing firmware and configuration.
- Access to the lab's calibration records, release configuration and research-data storage.

## Install once and create the shortcut

1. Get the complete approved EAST release from the laboratory's shared location. Record the release tag or commit. Extract it into a stable local folder, for example `C:\EAST\Ortho-Sim` if your account can write there. Keep the source modules, both requirements files, `tester_config.json`, `calibration`, and `images` together. Do not copy the old laptop's virtual environment or machine-zero state.
2. Open the repository folder. **Install-EAST.cmd** and **Launch-EAST.cmd** are included next to **Ortho-Sim.py**. Verify the approved config and calibration files match the installed ODrive, load-cell channel, fixture and rotary sensor. Record any lab-approved overrides.
3. Double-click **Install-EAST.cmd** under the Windows account that will operate EAST. It creates a local `.venv`, installs the pinned `requirements-windows.txt` packages, runs `pip check` and EAST's environment check, and creates **EAST** on that account's Desktop after the checks pass. Retain the scripts in the installation folder.
4. Double-click **EAST** on the Desktop and confirm the expected application build opens and the window shows **DISCONNECTED** before you select **Connect**. A minimized console accompanies the application and can be restored from the taskbar for startup diagnostics.
5. If the folder is moved or the account changes, rerun **Install-EAST.cmd** from the installation folder. If you replace the release, preserve its predecessor, configuration records and run data until the replacement passes the checks below.

## First use on a replacement computer

1. Start EAST with the fixture unloaded and follow the SOP's enclosure, power-on, connection and empty-fixture zero checks. Verify the expected controller, valid rotary readings and load-cell readings before admitting motion.
2. Establish a fresh verified zero and empty-machine tare. Run the approved unloaded acceptance check in **docs/VERIFICATION_PROTOCOL.md**. Confirm normal Stop and cleanup behaviour under the lab's verification procedure before testing a specimen.
3. Confirm the configured output folder can be written under the operating account. The supplied configuration uses **EAST Logs** within the release folder. Verify the resulting strain CSV, rotary CSV and metadata JSON, then back them up to the nominated shared storage.
4. Record the verified release, Python and driver versions, configuration and calibration identifiers, device identities, acceptance run and shortcut location in the lab handover record.

## If setup or startup fails

- If the installer rejects Python or an existing environment, check `py -3.10 --version`. Rename an incompatible `.venv` after closing EAST, then rerun setup. Keep the pinned dependency versions with the release.
- If the shortcut opens and immediately exits, open Command Prompt in the release folder and run `Launch-EAST.cmd` to see the error. For an installation check, run `.venv\Scripts\python.exe Ortho-Sim.py --check-environment`.
- If the GUI opens but devices are absent, check the USB data cables, Device Manager, Phidget Control Panel and configured device identities. Close other EAST instances, ODrive tools and Phidget channel test windows before retrying.
- EAST stores local runtime state under `%LOCALAPPDATA%\EAST` by default. Use the SOP's fresh zero and tare procedure on the new computer.
