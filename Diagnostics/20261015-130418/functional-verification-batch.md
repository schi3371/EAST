# EAST supervised functional verification batch — 2026-10-15

## Pre-motion automated evidence

At 13:04:18.420 AEDT, the existing GUI session's runtime checkpoint reported controller
state 1 (IDLE), active errors 0, no motion owner, a 32 ms feedback capture, zero adapter
lock wait, zero transport queue wait, and no session mapping. Reference confidence was
`fault` with reason `Previous power-isolated exit: physical machine-zero recovery
required`. This is the expected fail-closed prerequisite for the normal physical
machine-zero workflow; it is not treated as verified zero.

The production configuration remains unchanged: ±15° maximum travel, 0.1–20°/s speed,
0.1–100°/s² acceleration, minimum 5° constant-speed span, 8 ms requested acquisition,
150 ms motor capture limit, and accepted rotary profile
`calibration/rotary_P3022_20261014.json`.

## Planned single-batch settings

Use the existing **Standard Empty-Machine Baseline** protocol so all three recordings
are identified as empty-machine evidence. Preserve the same operator, fixture, and
calibration identity fields throughout.

1. Interrupted Stop run: minimum 5°, maximum 5°, speed 1°/s, acceleration 100°/s²,
   one cycle. Stop during the long commanded sweep, not during setup or after motion.
2. Completed slow run: minimum 10°, maximum 10°, speed 1°/s, acceleration 100°/s²,
   three cycles.
3. Completed corresponding run: minimum 10°, maximum 10°, speed 5°/s,
   acceleration 100°/s², three cycles.

All settings are within existing configured limits. No timing, configuration,
calibration, or application source change is authorized.

## Operator observations and automated file review

The operator reported that physical machine zero, empty-machine tare, actual Stop,
completed motion, populated plotting, GUI responsiveness, and ordinary Quit all worked
well. The terminal transcript records machine zero established at 13:07:12 AEDT and a
16-sample tare at 13:07:16. The same reference ID/generation and tare identity were
retained across every run.

Six run sets were produced, not the three planned sets. All 18 original metadata,
strain-CSV, and rotary-CSV files are preserved under `EAST Logs/`:

| Start | Settings | Outcome | Rows / achieved acquisition | Rotary recording |
|---|---|---|---|---|
| 13:10:12 | ±5°, 1°/s, 1 cycle; operator Stop | aborted as expected, 0/1 | 306; 65.29 Hz | 581 events; 124.97 Hz |
| 13:10:36 | ±5°, 1°/s, 1 cycle | completed 1/1 | 1,911; 63.40 Hz | 3,764; 124.98 Hz |
| 13:11:31 | ±10°, 1°/s, 3 cycles | completed 3/3 | 8,947; 64.35 Hz | 17,374; 125.00 Hz |
| 13:14:04 | ±10°, 5°/s, 3 cycles | completed 3/3 | 2,231; 63.29 Hz | 4,404; 124.96 Hz |
| 13:16:06 | ±10°, 10°/s, 3 cycles | completed 3/3 | 1,440; 63.60 Hz | 2,827; 124.99 Hz |
| 13:16:38 | ±10°, 20°/s, 3 cycles | completed 3/3 | 1,076; 64.06 Hz | 2,100; 124.97 Hz |

For every run, metadata reports confirmed final idle, zero final active errors, healthy
feedback, no monitor error, no recorder or rotary error, zero pending CSV rows, recorder
worker complete, verified machine zero, and valid tare. Raw CSV review found zero rows
with nonzero axis errors and zero invalid rotary joins. The independent rotary files had
31–32 ms maximum gaps. The joined acquisition stream achieved 63.29–65.29 Hz despite
the configured 8 ms request; p95/p99 row gaps were 16/31 ms and maxima were 32 ms except
47 ms on the long 1°/s run. Rotary events are recorded independently near 125 Hz, while
only the newest event is joined to each slower acquisition row.

Motor snapshot capture p95 was 16 ms for every run, p99 was 31 ms, maximum was 31–78 ms,
maximum transport queue wait was 15–47 ms, and adapter-lock wait was zero. Maximum
ODrive position age in the CSVs was 63–140 ms, below the configured 250 ms freshness
limit. Maximum rotary age in joined rows was 16–32 ms, below its configured 100 ms
limit. Metadata lateness maxima were 22–23 ms. The large `acquisition_overruns` counts
therefore describe deadline misses relative to the requested 8 ms schedule; they are not
equivalent to missing/persistence-failed rows.

### Stop/cleanup warning

The requested Stop run correctly finalized `aborted` with error `operator stop`, fresh
confirmed IDLE at 13:10:17.887, 306/306 rows persisted, recorder completion, no pending
rows, and no monitoring/persistence/rotary error. The operator observed actual stopping
and a responsive GUI.

However, after the next run was started the GUI emitted `Stop/cleanup is still running;
idle or data finalization may be unconfirmed` and wrote a stalled-worker diagnostic at
13:10:36.514. Its stack captured the new run's feedback monitor, motion initialization,
acquisition, and checkpoint threads. The second run subsequently completed 1/1 with
confirmed idle and complete persistence, and all later runs also finalized cleanly, but
the warning is retained as an unresolved sequencing/diagnostic limitation. The batch
should have stopped at that warning under the requested rule; later passes do not erase
it.

Original external diagnostic artifacts remain at:

- `C:\Users\schi3371\AppData\Local\EAST\stalled_workers.log`, SHA-256
  `83B4557D6B117B80F963F3217DA93B3EE2EC4DF1827CA5FCC06E0A831A769B26`
- `C:\Users\schi3371\AppData\Local\EAST\stalled_workers.timing.json`, SHA-256
  `D693A7E1349851114D3975A5A976978558E289F8AAEA93A542A83EC53AFA8C10`

### Deviations and finish

All six recordings identify themselves as `custom`, not
`standard_empty_machine_baseline`, although the operator reports an empty fixture and
the metadata retains the empty tare/reference/matching dimensions. The extra completed
±5° run and the 10°/s and 20°/s runs were outside the requested three-run batch. They are
preserved and reported, but the two higher-speed single runs are not treated as a planned
progression or broad reliability validation.

Ordinary Quit subsequently closed normally. Automated verification found no remaining
Python/EAST process. The 13:20:41.185 checkpoint records clean shutdown, clean continuity,
controller IDLE, zero active errors, verified machine zero, no motion owner, and a 15 ms
final capture. Deliberately blocked shutdown/isolation recovery remains NOT TESTED.
