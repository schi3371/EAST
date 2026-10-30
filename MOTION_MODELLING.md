# Motion Modelling in the EAST GUI

Open `Ortho-Sim.py` normally. In the manual-controls panel, choose **Motion
Modelling** (below Rotary Sensor Calibration). This separate window shares the
existing connection and normal strain-test execution; it does not launch the old
standalone runner or create another motor controller.

## Preparation and operation

1. Keep the fixture empty, physically clear and initially at 90 degrees. Use the
   main window's connection, **Home / Return to Zero** and unloaded tare.
   Operator, fixture ID (e.g. `0`) and calibration ID must match that preparation.
   The tool reads those identities; zero is established by the official rotary
   homing workflow described in [Rotary Homing](ROTARY_HOMING.md). Turn Manual Mode off. Close Calibration
   and Machine Zero setup windows before starting.
2. Open **Motion Modelling**. Defaults: nominal +/-10 degrees, 100 degrees/s²,
   three numbered cycles per run, slow speed 0.5 degrees/s, operating speed
   5 degrees/s. These are editable and pass the normal parameter validator.
3. Choose **Run One Slow Test** for S1, or **Run Six-Test Sequence** for
   S1/M1/S2/M2/S3/M3. Review the confirmation and press Run. Each run uses the
   usual initial positive endpoint, negative/positive cycles, settling, conservative
   return to machine zero and confirmed idle. Startup/return phases are recorded
   and labelled separately, as in normal tests.
4. Watch run/cycle/phase, elapsed time, raw motor turns, calibrated rotary angle
   and the mirrored **Session Terminal**. The normal main-window plot remains
   available. **Stop**, Escape or the main Stop cancels the remaining sequence
   through the existing Stop service. Closing the child window during a batch
   requests Stop and waits for cleanup/save before allowing closure.

Main test inputs and competing motion controls stay disabled during a sequence,
including between runs. Reference, tare and rotary readiness checks run again for
**every** trial. A fault, Stop, cleanup warning, partial recording, missing cycles,
metadata failure or unconfirmed idle prevents automatic advancement. A manifest
save failure also prevents subsequent admission. No automatic retry or resume is
implemented; investigate the terminal, recover normally and start a new batch.

## Saved evidence

In the configured normal output directory (default `EAST Logs`):

- A uniquely named `MM_<batch>_<run>_..._strain_data.csv`, containing the existing
  complete load-cell/force/torque, motor-position/timing and joined rotary columns.
- A full delivered-event `_rotary_samples.csv` and normal metadata JSON per run.
  Metadata keeps the usual reference, unloaded tare, controller/configuration,
  commanded parameters, calibration, phase definitions, provenance and recorder
  statistics. An additional `motion_modelling` object identifies batch and run.
- `motion_modelling_<timestamp>.json`: frozen plan, identities/reference snapshot,
  linked run files and outcomes, updated atomically before movement and between
  runs. An interrupted application may leave its last saved batch outcome as
  `running`; inspect the linked per-run metadata before analysis.

Raw motor and rotary host timestamps/sequence fields remain available for
nonlinear modelling; channels are not claimed hardware-synchronous. The usual
ODrive-derived torque geometry remains unchanged. This window does not implement
rotary feedback control, an automatic zero, a nonlinear fit, or loaded-AFO testing.

## Verification

`python -m unittest discover -s tests -q` runs only offline fake-device tests.
New coverage exercises six-run recording through the actual GUI services, parent
Start exclusion, completion/idle gates, Stop during motion and before admission,
initialization and disk failures, terminal mirroring and window close. The long
fake-service batch tests allow desktop scheduler jitter in their **test-only**
feedback timing configuration; production limits and motion modules are unchanged.
Hardware execution, timing/accuracy and Windows display checks remain bench work.
