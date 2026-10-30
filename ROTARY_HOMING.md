# Rotary machine zero — EAST v1.9.0

The main GUI now uses the accepted rotary angle calibration as the zero reference.
**Home / Return to Zero** replaces the manual square-alignment/jog popup and the
provisional child window. The stopping band is fixed at **±0.10°**. It requires
0.5 seconds of stationary sensor observations inside the band, confirmed motor
idle, then another 0.5-second observation inside the band. This is a homing
acceptance criterion, not a specification of absolute angular accuracy.

## Normal operation

1. Close EAST before `git pull --ff-only`. Start `Ortho-Sim.py` and confirm version
   `1.9.0-rotary-machine-zero`. Enter Operator ID, Fixture ID and load calibration ID.
2. Connect. The live rotary angle is now in the main zero panel. Select **Empty
   fixture** or **AFO fitted**, confirm clear mechanism/unchanged sensor mounting/
   accessible E-stop, then choose **Home / Return to Zero**. Enclosure protection
   remains in use. Homing is allowed with an AFO fitted, as explicitly requested.
3. Homing reads the current sensor angle and makes bounded position corrections.
   It automatically saves the corresponding motor reference after the complete
   sensor, idle and recording checks pass. There is no manual zero-setting step.
4. For the first setup of an application session, home the **empty** fixture and
   capture unloaded tare before mounting an AFO. Homing with an AFO fitted does
   not measure or replace unloaded tare. A motor-feedback recovery alone now
   preserves an existing independent load-cell tare; changed Bridge identity or
   mismatched fixture/calibration IDs still invalidate or block its use.
5. Before a new test, the software checks the current rotary angle, current motor
   reference, calibration identity and sensor connection. If zero is not ready,
   Start prepares it using automatic homing (the main clearance checkbox must be
   confirmed). Only successful preparation can proceed to the normal test
   confirmation/start. Test settings are frozen across this preparation.
6. Motion-modelling batches still require home and unloaded tare before admission;
   each trial rechecks zero. An interrupted batch is not silently resumed.

## After a stale-feedback stop

Wait for normal recording/Stop cleanup. Confirm the main zero-panel checkbox and
choose **Home / Return to Zero**. This action refreshes/revalidates unhealthy
ODrive monitoring and then homes automatically; it does not require the removed
manual setup or a separate zero-restoration popup. It waits briefly for fresh
rotary samples after reconnection. If fresh controller/sensor feedback cannot be
obtained, it reports the reason and does not command movement.

Stale feedback still stops motion. Reading a current rotary angle cannot prove
controller health, idle state or motor-encoder continuity. The new workflow
removes manual reference recovery, not these checks. No interrupted experiment
restarts automatically. A user-requested new test may prepare zero and then start.
Stop during preparation cancels that continuation.

## What is controlled and recorded

ODrive retains its existing inner position control/TRAP_TRAJ execution. The outer
homing policy uses measured rotary angle, with the existing recovery speed of
1°/s and acceleration of 5°/s². Per-correction limit is 0.5 nominal degrees;
cumulative limit 20 degrees; maximum 80 corrections and 120 seconds. The existing
±15° travel bounds, controller protections, ownership and feedback limits remain.
A blocked native transport call still requires the existing Stop/isolation path.
Wrong direction, lack of progress, noisy/invalid readings, changed connection or
calibration, post-idle drift, and failed recording/persistence prevent acceptance.

Only the homing workflow is changed: normal test trajectories and torque geometry
retain their existing motor conversion. Tests now require a rotary-derived zero
mapping and fresh sensor agreement at initialization. The method, sensor profile,
residual measured angle and tolerance are stored with test metadata.

`EAST Logs/rotary_home_<timestamp>.json` and a companion `_rotary_samples.csv`
record fixture/AFO condition, calibration/connection identity, policy, corrections,
observations, recording outcome and the saved reference ID. The durable reference
also records the rotary homing method and calibration/connection identity. Older
`provisional_rotary_home_*` files remain historical trial evidence.

Offline tests cover recovery with an AFO declared fitted, retained tare, automatic
pre-test homing, cancelled preparation, zero persistence, recording failures and
normal-test regressions. Updated loaded/empty hardware behavior and repeated
zero accuracy still require bench verification; no physical motor was operated
while implementing v1.9.0.
