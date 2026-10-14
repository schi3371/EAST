# EAST 1.8.0 supervised execution

This is an offline implementation. **Windows physical behaviour: NOT TESTED.**
It does not establish that the October freezing or USB faults are resolved.
The main GUI layout and operator steps remain the same.

## Ownership and dispatch

`east_execution.TestEngine` has one bounded background lifecycle queue. Connect,
Start preparation, machine-zero saving, measured-reference recovery, tare,
calibration file preparation/finalisation and calibration fitting submit work.
Operator fields are copied on the GUI thread before submission. Results/status
travel back through plain events; Tk changes happen on the GUI thread.

`TransportOwner` is the sole ODrive transport thread for a connected controller.
`ODriveAdapter` routes reads, trajectory configuration, enable, target writes,
idle requests and watchdog operations through it. Requests carry a connection
identity and cancellation epoch. Motion ownership/cancellation is rechecked
at dispatch; queued motion is rejected immediately by Stop. Idle has priority
0, motion 5, reads 10. A request already executing retains its owner until the
Fibre call returns. No timer can interrupt arbitrary Fibre property access.
The queue holds at most 64 pending operations.

Motion scripts retain the existing trajectory/settle sequence and dedicated
owned workers. Acquisition and the monitor are headless services. These workers
never touch ODrive directly outside the adapter. The existing motion coordinator
still prevents conflicting modes; admission is nonblocking when a prior command
holds its dispatch gate. Stop bypasses the lifecycle worker, latches cancellation
immediately and uses its existing supervised idle/fault-escalation worker.

Lifecycle states are disconnected, connecting, ready, moving, stopping, fault
and closing. Lifecycle readiness alone does not authorise motion. Movement also
requires the existing zero/tare/measurement/ownership checks. Engine activity,
shutdown, an unhealthy/dead monitor, stale feedback and cancellation block motion.

## Timing and recording

Position time means host completion of the position remote read, rather than
completion of the later velocity/error/state/disarm/uptime bundle. Position and
velocity each retain acquisition start/end brackets. These are host brackets,
not synchronised controller measurement timestamps. Motor state/error and disarm
checks remain live; uptime is omitted from powered monitoring but still read
at idle, setup and recovery. The optional phase path remains disabled by config.

Production budgets are unchanged: **150 ms complete capture; 250 ms cached
position freshness; 100 ms rotary freshness**. Capture duration includes dispatcher
waiting, I/O lock waiting and all requested property reads. Position freshness
uses the position timestamp. Idle confirmation reads current state and active
errors independently of position/uptime. Its wall/monotonic observation times
and connection identity are retained even if subsequent measurement or disk
operations fail. That observation alone cannot enable a later movement.

An 8 ms wait after a GUI monitor capture produces a period of capture time plus
8 ms. Headless diagnostics schedule on 8 ms deadlines and report actual rates;
compare property durations and gaps, rather than assuming the two schedules have
identical rates. No catch-up measurements are invented. `FeedbackMonitor` reports
achieved rate and maximum gap; `AcquisitionLoop` reports overruns/lateness.

`CSVRecorder` owns strain/calibration CSV writing and fsync. Acquisition only
submits ordered immutable rows. The normal limit is 2,000 queued rows plus the
active flush batch; calibration uses 8,192. Overflow and disk failure are explicit
errors. A batch with uncertain durability remains available for a separate
recovery file, never appended automatically into the original data. Recording
finish has a 3 s deadline; a surviving writer retains ownership/cleanup gating.

The required checkpoint writer also has a 3 s deadline, including queue admission.
Timing out does not dispose of its worker or allow motor enable. The durable
arming checkpoint must complete before any closed-loop request. Routine strain
fsync runs off acquisition. Rotary callbacks retain the existing bounded event
queues, original timestamps, ties, sequence identities and calibration checks;
rotary formatting/writing stays on its independent recording worker.

Existing strain/rotary column names and normal filename behaviour remain intact.
Schema 4 appends `MOTOR_TIMING_COLUMNS`: position/velocity brackets, position age,
whole-capture start/duration, dispatcher wait and I/O lock wait. Empty optional
fields mean unavailable, not zero. CSV row time remains the host load-loop/logging
time; the streams are not hardware-synchronised. Torque still uses ODrive angle;
the torque–angle display still uses the accepted rotary angle.

## Recovery and shutdown

Connect reuses a healthy stationary session as before. For a stopped/unhealthy
monitor it locks motion, retires the old monitor/checkpoint writer, obtains fresh
idle/measurement/identity evidence, and starts a new monitor. A dead failed writer
can be retired; its old error cannot permanently prevent a fresh writer.

If the old Fibre object has been lost, rediscovery happens only after its old
transport thread exits and all queued requests are retired. A replacement gets a
new connection generation. No request for the old connection can reach it.
Controller resets, changed frames and unproven continuity invalidate the reference
and tare. Raw turns are never relabelled as physical 90°. Automatic restoration
remains disabled: uptime/reset behaviour still lacks bench validation. Calibration
positioning is suspended by Connect and requires a fresh operator acknowledgement.
Recovery never restarts a test.

Quit immediately closes movement admission. Tk polls cleanup progress; it does
not join hardware, recording or persistence workers. After three seconds of
blocked cleanup, or an unconfirmed idle result, a modeless shutdown dialog exposes
**Exit After Motor Power Is Isolated** directly. Both physical E-stop and motor
supply-isolation acknowledgements are required. Text resizes; the action sits in
its own fixed dialog row outside scrolling settings. Actual Windows display
scaling remains NOT TESTED.

Isolation acknowledgement is not controller evidence. The process marks the run
incomplete, invalidates reference continuity, attempts dirty outcome/buffer saving
and exits with failure status. A four-second timer bounds that best-effort exit;
blocked disk/transport may leave unsaved data. The startup dirty marker is written
before hardware use, so a blocked exit writer cannot make the next launch appear
clean. Clean shutdown clears it only after required cleanup/transport retirement.

A calibration writer that outlives its deadline retains its passive capture guard;
the GUI retires that guard once the writer exits. It does not mark the point good.
Failed test preparation before paths/metadata exist restores editable fields while
keeping unsafe movement blocked. There is no automatic motor-zero restoration,
watchdog enabling, sensor feedback control, conversion change or baseline subtraction.

## Stationary GUI observations

`Ctrl+Shift+D` saves a diagnostic JSON in EAST's runtime state directory (the
terminal reports its path). Select Terminal or Plot, leave the same stationary
hardware/settings for at least 20 s, then save once for each tab. JSON contains a
bounded recent motor timing history, GUI drain/gap maxima, rotary measurement
summary and lifecycle state. Saving happens off Tk. GUI maxima are cumulative
within the launch; restart for independent matched comparisons. This observes GUI
and sensor-acquisition overhead. It does not create a stationary force CSV or
claim to reproduce a moving test. Use the separate headless sensor-recording mode
for stationary disk/acquisition comparisons. Never run competing ODrive programs.
