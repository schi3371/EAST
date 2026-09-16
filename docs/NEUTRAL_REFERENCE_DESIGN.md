# EAST Neutral Reference and Recovery Design

## Safety model

EAST distinguishes three different facts that must not be conflated:

1. The **physical reference record** states how and when the mounted fixture/AFO was established at physical 90 degrees, by whom, on which fixture and controller/configuration.
2. The **session mapping** states which ODrive `pos_rel` value represents that physical neutral in the current controller session.
3. The **runtime checkpoint** records the latest feedback, reference confidence, motion state, owner, and clean/unclean shutdown evidence.

The physical record can persist across launches. The session mapping is not automatically trusted after a launch unless an explicitly enabled method proves that the relative encoder frame is continuous or resolves it from confirmed encoder phase. Both methods are disabled by default. Therefore the normal default after a new application/controller session is `RECOVERY REQUIRED`.

## Runtime state

The state directory is `%LOCALAPPDATA%\EAST` on Windows, `~/Library/Application Support/EAST` on macOS, or `$XDG_STATE_HOME/east` on Linux. `EAST_STATE_DIR` overrides it for test/deployment purposes.

- `neutral_reference.json`: durable physical reference and hardware/configuration fingerprint.
- `runtime_checkpoint.json`: generation-ordered current session/motion evidence.
- `reference_events.jsonl`: append-only reference/recovery/invalidation audit events.
- `hardware.lock`: OS-held exclusive process lock.

JSON replacement uses a same-directory temporary file, flush, file `fsync`, atomic replace, and directory `fsync` where supported. A corrupt state file is renamed with a `.corrupt-*` suffix instead of being overwritten. A failed persistence operation blocks or invalidates verification before hardware can be armed.

## Reference states

- `UNKNOWN`: no physical reference exists.
- `RECOVERY_REQUIRED`: a physical record may exist, but no defensible current-session mapping exists.
- `VERIFIED`: physical reference and current-session mapping are both available and match the connected hardware fingerprint.
- `FAULT`: feedback, persistence, configuration identity, or another safety condition failed.

Start, ordinary manual movement, and automatic neutral return require `VERIFIED`. The recovery dialog is the only unreferenced movement path in the GUI. It uses 0.25 degree steps, 1 degree/s, 5 degree/s2, a +/-5 degree cumulative bound, a 120 second session timeout, a safety acknowledgement, settle validation, and confirmed idle after each step.

`Set Current Physical Position as 90 deg Neutral` performs no motion. It requires fresh finite feedback, low velocity, confirmed idle, Operator ID, Fixture ID, and an explicit physical-alignment confirmation.

## Motion lifecycle

Every GUI motion has one owner and cancellation generation. Before closed-loop enable, EAST:

1. Confirms the required reference state.
2. Confirms finite/fresh feedback and no active error.
3. Persists an `arming` checkpoint.
4. Loads the current position as the controller setpoint.
5. Rechecks cancellation and requests closed loop.

Before each target write, it persists a `moving` checkpoint and rechecks ownership. Target arrival requires finite/fresh feedback, no active errors, position tolerance, low velocity, and a continuous settle dwell.

Successful strain tests use a dedicated 2 degree/s, 5 degree/s2 neutral return. `completed` is assigned only after neutral settle, confirmed idle, and 500 ms post-idle neutral observation. Stop, Escape, acquisition/logging fault, ODrive fault, watchdog feed fault, and communication/monitor fault request idle immediately and never initiate return motion.

## Optional features disabled by default

- **Controller-session continuity:** requires explicitly confirmed ODrive uptime units and matching clean-shutdown/wall-time/uptime evidence.
- **Encoder phase recovery:** requires confirmed phase source, units, period, controller-turns-per-period, sign, uncertainty, matching hardware/configuration, and exactly one candidate inside the safety range.
- **Measured-angle recovery:** requires the confirmed phase model plus bounded measurement uncertainty.
- **ODrive watchdog:** lifecycle and mocks are implemented, but hardware timing/behavior must be verified before enabling.

No recovery path clears ODrive errors, writes/saves encoder configuration, seeks current/load, performs blind homing, or moves automatically after reference restoration.

## Bench evidence still required

- Confirm ODrive firmware/API fields and whether `system_stats.uptime` units/behavior are suitable for continuity proof.
- Confirm the AMT212B/RS485 raw phase source, period, wrapping, direction sign, uncertainty, and relationship to controller turns before enabling phase recovery.
- Confirm idle request/confirmation and optional watchdog behavior with the physical E-stop and door interlock.
- Confirm recovery jog and all normal movement directions, physical travel bounds, settle thresholds, and post-idle drift.
- Confirm persistent state location is writable on the lab account and across executable updates.
- Complete commanded-versus-independent speed and ROM verification before formal AFO testing.
