# Provisional rotary homing — v1.8.2

The original **Machine Zero Setup** and **Return to Machine Zero** remain in the
GUI. A separate **Provisional Rotary Homing** button appears below Motion
Modelling. It approaches the zero of the accepted rotary calibration and records
the corresponding candidate motor position. It never writes a machine-zero
reference, changes the sensor calibration, restores a lost reference, or starts
an interrupted experiment again.

## First bench trial

1. Stop/close EAST before updating: `git pull --ff-only`. Relaunch `Ortho-Sim.py`
   and check the displayed version is `1.8.2-provisional-rotary-homing`.
2. Keep the fixture empty. Connect/recover ODrive until monitoring is healthy.
   The fixture may be displaced and the saved zero may be invalid. No tare is
   needed. Confirm the accepted rotary calibration/mounting still applies.
3. Close Rotary Sensor Calibration, Machine Zero Setup and any active motion
   experiment. Open **Provisional Rotary Homing**. Its live label displays the
   accepted measured angle; a negative angle should correct toward positive.
4. Leave the provisional stopping band at **±0.10 degrees** for the first trial.
   This is a sensor-reading criterion, not an absolute accuracy specification.
   The editable band is restricted to ±0.05 through ±0.5 degrees. The accepted
   calibration fit RMSE is about 0.064 degrees; ±0.01-degree physical accuracy
   has not been established.
5. Confirm the empty fixture/unchanged mounting/clear mechanism checkbox. Keep
   enclosure protection and the physical E-stop in use. Choose **Run Provisional
   Homing**, read the movement summary, and confirm. Observe the mechanism and
   Session Terminal; Stop/Escape uses the normal Stop service.
6. Wait for the result and file-saving completion. A successful result requires
   all sampled sensor readings to remain inside the band for 0.5 seconds, with
   stationary motor feedback, and another 0.5-second observation after confirmed
   motor idle. Check physical alignment independently before approving the method.
7. If a normal test is needed afterwards, use the original Machine Zero Setup
   after physical confirmation, then capture unloaded tare as usual. This trial
   itself does not make an invalid zero valid.

If fresh feedback cannot be restored, homing stays blocked. If another stale
feedback fault occurs, save the terminal and new evidence files rather than
repeatedly restarting the experiment. The intermittent transport/timing problem
has not been claimed resolved.

## Execution and limits

The outer policy is **measure → bounded position correction → settle → measure**.
ODrive's existing inner position control / TRAP_TRAJ path, ownership, cancellation,
identity checks and feedback limits remain in use. It is not a high-frequency PID
loop over USB. The existing recovery profile is 1 degree/s and 5 degrees/s².

Each correction is at most 0.5 nominal degrees, with smaller corrections near
zero. Limits are 80 corrections, 20 nominal degrees cumulative travel and a
120-second overall deadline (a blocked native transport call remains subject to
the existing Stop/isolation limitations). Both the measured rotary angle and the
estimated motor-based travel are bounded by the configured ±15-degree range.
Feedback must be fresh; changing sensor connection, calibration, operator,
fixture or motion conversion blocks continuation. Wrong-direction movement,
insufficient progress, noisy zero, stale/invalid sensor data, unexpected motor
state, recording failures and post-idle drift prevent success.

The method reuses the existing unreferenced calibration-positioning reference
only for bounded movements. It does not claim a newly verified physical zero.
Operator cancellation preserves the prior reference unless another actual fault
has invalidated it; ordinary fault handling remains in effect.

## Evidence and validation

`EAST Logs/provisional_rotary_home_<timestamp>.json` records identities, the full
sensor calibration, requested policy/control settings, zero snapshots before and
after, attempted corrections, fresh sample observations, outcome and candidate
motor zero on success. A companion `_rotary_samples.csv` records the raw rotary
events. Progress is atomically saved before each correction. An interrupted
process may leave a preparing/running result; that is not a successful trial.
Recording/Stop workers retain motion ownership while cleanup is unresolved.

Offline tests exercise convergence from both sides, correction/travel/time
limits, bad tolerances, noise, wrong direction and no progress. Actual application
services with fake devices test unreferenced homing, preserved verified zero,
Stop, sensor loss, post-idle drift, and a failed evidence write. Hardware motion,
accuracy, coupling, Windows display and zero repeatability still require the lab
trial. No physical motor was operated during development.

A shared settle-check race was also corrected: feedback is now validated against
the clock read **after** retrieving the snapshot, avoiding false negative ages
when the monitor publishes during retrieval. This does not relax freshness or
coherence limits, or explain away the earlier long feedback acquisitions.
