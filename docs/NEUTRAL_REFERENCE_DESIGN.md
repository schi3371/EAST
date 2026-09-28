# EAST Machine-Zero Reference and Recovery Design

## Safety model

EAST distinguishes four different facts that must not be conflated:

1. **Machine zero** is the empty moving fixture mechanically aligned at physical 90 degrees to the fixed machine reference. EAST defines this position as machine angle 0 degrees.
2. The **physical reference record** states how and when machine zero was established, by whom, on which fixture and controller/configuration.
3. The **session mapping** states which ODrive `pos_rel` value represents machine zero in the current controller power session.
4. The **AFO zero-torque neutral angle** is a specimen property that may be estimated later from recorded torque-angle data. It is not used as the machine control zero.

The runtime checkpoint separately records the latest feedback, reference confidence, motion state, owner, and clean/unclean shutdown evidence.

The physical record can persist across launches. The session mapping is not automatically trusted after a launch unless an explicitly enabled method proves that the relative encoder frame is continuous or resolves it from confirmed encoder phase. Both methods are disabled by default. Therefore the current default after a new application/controller session is `SETUP REQUIRED`.

## Runtime state

The state directory is `%LOCALAPPDATA%\EAST` on Windows, `~/Library/Application Support/EAST` on macOS, or `$XDG_STATE_HOME/east` on Linux. `EAST_STATE_DIR` overrides it for test/deployment purposes.

- `neutral_reference.json`: legacy internal filename for the durable machine-zero record and hardware/configuration fingerprint.
- `runtime_checkpoint.json`: generation-ordered current session/motion evidence.
- `reference_events.jsonl`: append-only reference/recovery/invalidation audit events.
- `hardware.lock`: OS-held exclusive process lock.

JSON replacement uses a same-directory temporary file, flush, file `fsync`, atomic replace, and directory `fsync` where supported. A corrupt state file is renamed with a `.corrupt-*` suffix instead of being overwritten. A failed persistence operation blocks or invalidates verification before hardware can be armed.

## Reference states

- `UNKNOWN`: no machine-zero record exists.
- `RECOVERY_REQUIRED`: a physical record may exist, but no defensible current-session mapping exists. The GUI displays `Machine zero: SETUP REQUIRED`.
- `VERIFIED`: the machine-zero record and current-session mapping are both available and match the connected hardware fingerprint.
- `FAULT`: feedback, persistence, configuration identity, or another safety condition failed.

Start, ordinary manual movement, and return to machine zero require `VERIFIED`. The setup dialog is the only unreferenced movement path in the GUI. It uses 0.25 degree steps, 1 degree/s, 5 degree/s2, a 5 degree cumulative path-length budget, a separate +/-5 degree displacement envelope, one safety acknowledgement per open setup session, settle validation, a per-motion timeout, and confirmed idle after each step. There is no overall 120-second setup-window expiry. Reversing direction consumes more of the path budget; it does not restore it.

`Set Machine Zero - Fixture at 90 deg` records the current encoder position only and performs no position motion. It requires a connected ODrive, fresh finite feedback, no active errors, low velocity, confirmed idle, Operator ID, Fixture ID, safety acknowledgement, and explicit confirmation that the supplied square shows the moving fixture at physical 90 degrees.

`Return to Machine Zero - 90 deg` targets the saved encoder position. It is available when the reference is verified and the machine is idle, without requiring Manual Mode. If Start finds the machine away from zero, it instructs the operator to return to the existing zero and never silently redefines it.

## Empty-machine tare

`Tare Empty Machine` is a separate deliberate action performed at verified machine zero before an AFO or other removable load is mounted. The operator must acknowledge that the AFO/load is removed and enter Operator ID, Fixture ID, and Calibration ID. EAST samples the configured number of voltage-ratio readings and stores the offset, timestamp, operator, fixture, calibration, connected Phidget serial, channel, and sample count.

Start never tares. It requires the stored tare to match the current fixture, calibration, Phidget device, and channel, and records the tare in every metadata sidecar and its offset in every CSV row. This preserves the mounted AFO's preload. A typed Fixture ID or Calibration ID mismatch is non-destructive: EAST retains the tare, displays `IDENTITY MISMATCH`, and blocks Start until the text is corrected. Actual integrity events such as a connected-device mismatch or machine-zero setup jog/re-establishment can invalidate the tare. `Reset Test Fields` preserves it; only confirmed `Clear Session Tare`, replacement by another tare, or an integrity event clears it.

After taring and before mounting an AFO, the operator runs the unloaded fixture using the `Standard Empty-Machine Baseline` preset or an approved custom equivalent. EAST tags this by `test_type = empty_machine_baseline` and uses `EMPTY_MACHINE_BASELINE` internally; it does not infer baseline status from the file name. This full movement is distinct from the stationary tare and measures angle- and direction-dependent machine response.

Later analysis uses `AFO torque = loaded-machine torque - matched empty-machine torque`. A valid pair must match Fixture ID, Calibration ID, machine-zero reference ID/generation, minimum/maximum angle and total ROM, speed, acceleration, movement direction, and applicable session/tare mapping. These dimensions are preserved in the JSON and CSV. Acquisition does not automatically perform the subtraction.

The CSV preserves the inputs needed for later AFO neutral-angle analysis: machine-zero-relative angle, raw voltage ratio, tare offset, force, torque, movement direction, cycle/phase, machine-zero metadata, tare metadata, and raw ODrive position/velocity. EAST does not currently calculate or claim a zero-torque AFO neutral angle automatically.

## Motion lifecycle

Every GUI motion has one owner token. Idle confirmation and worker completion are separate: physical idle can be confirmed while the owner remains reserved, and only that owner can release the motion slot after cleanup. Connection, new motion, and control re-enablement remain blocked until release. Before closed-loop enable, EAST:

1. Confirms the required reference state.
2. Confirms finite/fresh feedback and no active error.
3. Persists an `arming` checkpoint.
4. Confirms `absolute_setpoints == false`, `circular_setpoints == false`, the expected position/velocity mapper scale, and required feedback/setpoint fields.
5. Loads the current position as the controller setpoint.
6. Rechecks cancellation and requests closed loop.

Before each target write, it persists a `moving` checkpoint. The final ownership check and target write share one command gate with Stop, so no queued target can be written after Stop is latched. Target arrival requires finite/fresh feedback, bounded snapshot acquisition time, no active errors, the expected closed-loop state, an unchanged disarm-reason baseline, position tolerance, low velocity, and a continuous settle dwell.

Successful strain tests use a dedicated 2 degree/s, 5 degree/s2 machine-zero return. `completed` is assigned only after zero-position settle, confirmed idle, 500 ms post-idle observation, acquisition-thread exit, and final CSV flush. A late Stop or acquisition/flush error cannot be upgraded to completed. Stop, Escape (including setup and plot windows), acquisition/logging fault, ODrive fault, watchdog feed fault, and communication/monitor fault request idle immediately and never initiate return motion.

Machine zero remains valid through test-setting edits, operator/AFO changes, plot actions, test completion, and ordinary referenced movement. It is invalidated only when controller power/reset or continuity loss cannot be excluded, an unreferenced setup adjustment is made, hardware/configuration identity changes, state is missing/corrupt, or ODrive feedback/persistence fails.

## Optional features disabled by default

- **Controller-session continuity:** clean application close and manual disconnect save a clean checkpoint only after confirmed idle and fresh error-free idle feedback. Restoration requires explicitly confirmed ODrive uptime units, matching clean-shutdown/wall-time/uptime evidence, the same reference ID/generation and conversion, and full serial/axis/firmware/configuration identity. This implementation exists but remains disabled because `system_stats.uptime` units have not been verified on the EAST ODrive.
- **Encoder phase recovery:** requires confirmed phase source, units, period, controller-turns-per-period, sign, uncertainty, matching full hardware/configuration identity, and exactly one candidate inside the safety range.
- **Measured-angle recovery:** requires an existing physical reference, the confirmed phase model, matching fixture/hardware identity, bounded measurement uncertainty, and exactly one candidate. It restores a session mapping without replacing the physical reference record.
- **ODrive watchdog:** remains disabled. Configuration rejects enabling it until health-coupled feeding has been explicitly verified on the bench.

No setup/recovery path clears ODrive errors, writes/saves encoder configuration, seeks current/load, performs blind homing, or moves automatically after reference restoration.

## Bench evidence still required

- Confirm ODrive firmware/API fields and whether `system_stats.uptime` units/behavior are suitable for continuity proof.
- Confirm the AMT212B/RS485 raw phase source, period, wrapping, direction sign, uncertainty, and relationship to controller turns before enabling phase recovery.
- Confirm idle request/confirmation and optional watchdog behavior with the physical E-stop and door interlock.
- Confirm setup jog and all normal movement directions, physical travel bounds, settle thresholds, and post-idle drift.
- Confirm empty-machine tare repeatability, Phidget serial reporting, and that mounting an AFO after tare preserves its preload in the recorded force/torque.
- Confirm persistent state location is writable on the lab account and across executable updates.
- Complete commanded-versus-independent speed and ROM verification before formal AFO testing.
