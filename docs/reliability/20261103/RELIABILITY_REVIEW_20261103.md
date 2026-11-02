# EAST reliability and coordinate review — 3 November 2026

Release candidate: **1.17.0-reliability-audit**, comparison preset **3.0**. Review baseline: source 8203501, logs through f6035a2. This is a software error-path review supported by recorded evidence and simulated tests, not hardware certification or a claim that all possible failures have been eliminated.

## Latest Run 9: both attempts saved

Condition: current cubic, nominal ±15°, 5°/s, three cycles, empty fixture. Both attempts have complete raw CSV files for the three numbered cycles and final error metadata. Neither passed its final unassisted rotary zero check (±0.1°).

| Attempt / local start on 2 Nov | Strain rows | Raw rotary rows | Completed cycles | Final observed rotary error |
|---|---:|---:|---:|---:|
| d31f6a1b / 20:42:37 | 3,194 | 6,313 | 3/3 | −0.78685° |
| 0cc1152e / 20:45:05 | 3,891 | 6,283 | 3/3 | −0.87585° |

These are retained three-cycle acquisitions with failed zero returns, not fully passed tests. Preserve both; use one predeclared primary attempt and the other as an additional within-session attempt when analysing repeatability. Do not silently choose whichever looks better. They are not independent remount trials. Run 9 need not be repeated merely because the terminal said error. The next uncollected condition is **Run 10: wider candidate, nominal ±15°, 5°/s, three cycles**. Start a new comparison batch at 10 after updating and confirming normal zero/tare readiness; the new protocol is separately identified in metadata.

The separate homing after the first Run 9 failed on “Motor is not stationary during rotary zero observation.” Its record includes a correction near zero followed by an immediate post-idle stationarity rejection. This supports correcting the observation procedure; it does not establish that sensor calibration or the cubic alone caused the return error.

## Coordinate finding

Physical zero is the rotary angle at fixture 90°. ODrive turns are session-local motor coordinates. A model defines displacement conversion; it does not redefine physical zero. Measuring a 2.055-driven trajectory with the rotary sensor is intentional and necessary to assess conversion error.

Homing already uses small nominal 2.055-based correction steps with rotary observations; it does **not** home using the current cubic. The older Original comparison used a saved neutral motor position. Current/Wider cubic comparisons start anchored at measured zero and refine their increasing-branch origin once at the initial positive endpoint, then freeze it for numbered cycles. These are different operational alignment procedures and must be described as such.

In the two Run 9 attempts, the cubic origin shifted by −0.21453 and −0.33907 motor turns respectively during that initial alignment. Therefore, a final zero error is a result of the operational model/alignment/trajectory/reversal/idle sequence; it is not a clean estimate of polynomial coefficient error alone. The logs do not isolate backlash, compliance, or idle release as the sole cause.

For protocol 3.0, every open-loop comparison run explicitly prepares measured zero before dispatch. An in-band fixture undergoes passive stationary verification; an out-of-band fixture receives the existing official homing transaction within the confirmed empty-fixture plan. The old/new references and evidence file are recorded. Original now uses `target = starting motor + (desired angle − measured starting angle) / 2.055`; its slope and motor-speed conversion remain fixed. Cubic coefficients and the initial positive-endpoint alignment remain unchanged. This removes a small starting-offset convention mismatch without silently replacing the cubic algorithms under test.

Do not pool protocol 2.1 and 3.0 as identical repeated trials. Earlier data remain valuable. For a future pure coefficient comparison, define a separately named common alignment procedure without endpoint refinement, rather than relabelling these operational comparisons.

## Reviewed fault paths and changes

| Boundary / scenario | Finding and response | Evidence / checks |
|---|---|---|
| Homing after motor idle | A single transient velocity reading previously rejected zero immediately. Reset the observation window and wait for a full fresh stationary dwell within the existing overall deadline. Persistent movement, position drift, stale feedback and insufficient independent samples still reject. | Field homing record; transient/persistent-motion, drift and frozen-sample tests. |
| Physical zero vs selected model | Readiness depended partly on motor-derived angle/global model selection. Physical zero now uses fresh rotary angle for every model, while retaining motor health and verified-reference requirements. | Original/rotary offset integration test. |
| Between comparison runs | Add an owned, persisted zero-preparation transaction per run; record reference changes before any successor dispatch. No manual Check Zero needed between healthy runs. | Passive and active preparation, cancellation and complete comparison tests. |
| Zero-return qualification | All strain tests qualify a stationary rotary observation window after idle. Motor-target drift is recorded separately during tests. A failed unassisted return remains failed even if separate recovery succeeds. | Original and cubic return/recovery tests; raw rows preserved. |
| GUI sensor event queue | Raw sensor samples were also queued for a GUI consumer that ignored them; an unserviced GUI could fill that queue. Samples now go to archival subscribers and the live snapshot, with GUI status/progress/error events retained. No archival dropping introduced. | 5,000 callbacks with GUI undrained; every raw sample delivered; archival overflow still faults. |
| Worker cleanup / next-run admission | Explicitly check both CSV and raw-rotary recorder workers. Wait for homing lifecycle cleanup before resuming an ordinary test. | Live-recorder admission and handoff tests. |
| Stop vs data fault | Use typed motion cancellation to distinguish operator Stop from actual sensor/controller/recording faults. Terminal wording no longer controls cancellation. | Stop during preparation, main-window Stop, terminal-warning and transport tests. |
| Batch scheduler/UI exception | Stop, retain ownership until workers finish, preserve run links, and attempt a final error manifest. Report persistence failure rather than leaving an apparently healthy running batch. | Injected scheduler/UI failure test. |
| Timing diagnostics | Keep connection-wide statistics, add frozen run-local statistics, bound rejected-bundle detail history while retaining total counts, and preserve rotary failure evidence. | Consecutive-window isolation/freeze test; late-read tests. |
| Render load | Enforce intended plot point cap; show zero preparation/recovery as current status. | Regression suite / reviewed source. |

The initial AST inventory contained **526 raise/stop/invalidate/finalise sites across 24 modules**. This is a navigation/coverage inventory, not 526 independent defects. The review followed admission, transport/ownership, sensor delivery, persistence, motion completion, zero qualification, cleanup, recovery and successor-dispatch boundaries. Existing calibration, ordinary/manual motion, loaded protocols, comparison and feedback-pilot tests remain part of regression coverage.

## Limits deliberately retained

Production rotary age limit remains 100 ms; ODrive capture limit 150 ms; ODrive age limit 250 ms; zero band ±0.1° and dwell 0.5 s. Passing a noise display near zero does not override freshness or stationarity. Comparison-only late-bundle retry retains its prior strict policy: discard the late bundle, allow one replacement only while the last good observation remains fresh, and stop on repeated delay, expiry or changed controller evidence.

Open-loop comparison recordings may still mark linear sensor extrapolation as unchecked outside the calibrated evidence range. The nominal ±15° command is not represented as a physical machine limit. Homing, ordinary control and the limited feedback pilot retain their existing separate policies. This release does not expand the calibration, extrapolation policy or pilot envelope.

Stop, disconnected/stale sensor, controller faults/reset, recording overflow, required durability failure, persistent settling failure, inconsistent model direction, and exhausted homing/motion deadlines remain genuine failure cases. A successful sensor reading is not proof that the motor transport or disk is healthy.

## Unresolved questions and next lab evidence

Windows scheduling, USB/Fibre timing, sleep/power management, and physical coupling can still cause faults. A 156 ms bundle proves a timing deadline was missed; it does not by itself identify a hardware throughput limit. Queue wait, lock wait, property timing, sensor gap and run-local statistics should be compared in subsequent logs before changing deadlines or controller architecture. Keep the lab computer awake during the sequence. Software tests cannot demonstrate loaded-machine safety or validate physical tracking accuracy.

A persistence/transport queue delay can occur between a health check and command dispatch. Ownership and cancellation are rechecked at dispatch, with independent monitoring retained, but these host-side checks do not provide hard real-time guarantees. This review does not claim to remove that limitation or add a controller-side watchdog.

Next: retain both Run 9 acquisitions, continue from Run 10 using the reviewed release, and examine first successful/error logs for stationary-window resets, per-run preparation evidence, measured tracking and timing. Do not refit from partial/error-labelled trajectories indiscriminately. Loaded-AFO reliability, independent remounts, material/design comparisons and broader feedback commissioning remain separate experiments.

## Verification

Final test totals and release source hashes are recorded in `verification.json` and `source_sha256.json` beside this report. Tests simulate hardware and failure injection; a successful Windows bench run remains required.

380 cases ran in discovery (370.8 s); the subsequently added statistics-isolation case also passed. Discovery exposed three test issues: an obsolete terminal-text cancellation expectation, a stale-sensor fixture missing the newly recorded metadata, and a scheduler-sensitive fake sensor in the multi-run recovery test. After test-only corrections, the affected groups passed (11, 28 and 6 cases respectively); the new statistics test passed independently. Production source was unchanged after discovery. A full clean discovery was not repeated after these fixture corrections; exact invocation results are retained in verification.json.
