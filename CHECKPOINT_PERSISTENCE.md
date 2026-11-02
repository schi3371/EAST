# Checkpoint persistence backlog — v1.12.1

A recorded test stopped with a full checkpoint queue, a last completed disk write of 0.406 seconds, and a required checkpoint durability timeout. This is persistence failure reported by the feedback-monitor pipeline; it is not evidence of an ODrive communication or hardware fault. The underlying Windows storage delay is not identified by the terminal excerpt.

Periodic runtime checkpoints are submitted every 0.5 seconds. Previously every snapshot entered the same bounded queue. During slow disk writes, obsolete intermediate snapshots accumulated ahead of required movement-boundary checkpoints.

Pending consecutive periodic snapshots now coalesce only when their state and extra context/owner match. The newest replaces the older pending snapshot. In-flight writes are never interrupted. Required durable checkpoints, audit events, changes of state/context and fault records are never replaced or overtaken. Raw strain/rotary data are unchanged; runtime snapshots are not the experimental data stream.

Required checkpoint acknowledgements retain their three-second deadline. A separate no-progress check stops continued periodic submission after three seconds without persistence progress. Real write errors and nonreplaceable queue overflow still stop the sequence. Persistence errors from the monitor's checkpoint submission now identify their source as checkpoint persistence rather than a generic feedback-monitor failure. This does not relax ODrive capture/freshness limits or recorder requirements.

After updating with EAST closed, restart the application and complete normal recovery/zero checks. Reselect the intended motion method: restarting restores the configured default, not the previous session's selector choice. Use a new batch; do not resume or label a partial run completed. Preserve the failed files for diagnosis. A further persistence stop needs investigation of storage latency and runtime-state diagnostics; do not bypass its checkpoint acknowledgement requirement.
