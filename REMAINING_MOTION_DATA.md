# Remaining Motion Data (Empty Machine) — v1.13.0

Close EAST, update the repository and restart. Prepare the empty fixture using the normal connection, rotary homing and valid unloaded tare workflow. Remove the AFO and removable loads. Verify clearance over measured +/-15 degrees plus the 0.1-degree endpoint tolerance before selecting this suite.

Choose **Remaining Motion Data (Empty Machine)** in Protocol, press **Load Preset**, then **Start**. The suite window opens. Check the operator/fixture/calibration identifiers and the prefilled setup ID, press **Run Remaining Data Suite**, and review the single full-plan confirmation. No configuration edits, per-run imports or further Start presses are needed. Filename prefix may be blank; this suite supplies MotionSuite automatically. The loaded fixed settings are three cycles, main ROM +/-10 degrees and acceleration 100 degrees/s². The wider and hold stages have their own frozen parameters; reload the preset if the fixed main values were changed.

## Automatic stages

| Runs | Stage | Settings |
|---|---|---|
| 1–3 | Updated +/-10-degree repeat set 2 | 0.5, 5, 20 nominal degrees/s, three cycles each |
| 4–6 | Updated +/-10-degree repeat set 3 | Same conditions, three cycles each |
| 7–9 | Rotary-supervised wide identification | Measured targets +/-15 degrees, Original motor conversion, 0.5/5/20 nominal bulk speeds, three cycles each |
| 10 | Stability and stationary direction checks | 30 seconds at zero, then targets -10, -5, 0, +5, +10 degrees approached from both directions, two-second holds, three repeats |

There are 27 numbered sweep cycles and 30 stationary target holds. The first two repeat sets complement the already completed Updated batch; they are repeated runs in this prepared setup, not independent rehoming/remount trials. Allow roughly 25–35 minutes, depending on correction count, communications and storage performance. Ordinary return, settling, idle confirmation and recording finalization occur between runs. No extra inter-run rest is silently introduced.

Methods are selected per run by the frozen suite plan, regardless of the main Motion method menu. That menu and competing controls remain locked until the suite completes or finishes fault cleanup. The suite never changes unloaded tare or writes a replacement machine-zero reference between runs.

## How the wide-range acquisition works

The Original conversion's nominal +/-15-degree command is not assumed to produce measured +/-15 degrees. Each wide leg starts with a conservative motor movement corresponding to 80% of the live measured angular error, followed by slow, bounded rotary-feedback corrections. The primary speed applies to the bulk leg; endpoint corrections use the existing conservative 1 degree/s, 5 degrees/s² profile, with individual nominal corrections no greater than 0.5 degrees. The final observation uses the existing half-second fresh-sample dwell and +/-0.1-degree envelope, targeting a more central mean.

This provides raw motor-position/rotary-angle data throughout the wider travel, including settled endpoints. Endpoint corrections are identified in motion-phase labels and metadata. Exclude them from constant-speed sweep comparisons. These endpoints are feedback-corrected acquisition data and must not be reported as independent validation of the cubic's feed-forward endpoint accuracy. No model coefficients are refitted or deployed automatically.

The bulk fraction is a conservative planning margin for this experimentally observed mechanism, not a certified travel guarantee. Fresh sensor measurements, controller state, ownership, Stop and recorded data remain supervised. Readings beyond +/-15 degrees plus the 0.1-degree tolerance fault the acquisition. Mechanical clearance still needs operator confirmation. No loaded AFO belongs in this suite.

The 0.5-degree/s full traversal is split into bounded moves, so the existing global 60-second per-move timeout remains unchanged. The fine positioning retains the official correction-count, cumulative-travel and timeout guards. Interior directional hold targets allow approach from both sides without commanding travel beyond the outer +/-15-degree envelope; outer endpoints are approached from inside the range.

## Homing edge-noise correction

Previously homing rejected a mean already within +/-0.1 degrees if a noise sample crossed the band edge. It now makes bounded corrections toward a mean within +/-0.04 degrees while still requiring every observed sample to lie within the original +/-0.1-degree band. The inner mean target leaves headroom for noise and post-idle movement; it is a centring strategy, not a claim of 0.04-degree absolute accuracy.

An observed span greater than 0.16 degrees is rejected rather than chased. Centred observations that still cross the band, insufficient fresh samples, wrong movement direction, lack of progress, stale feedback and failure after idle still prevent zero acceptance. The original post-idle +/-0.1-degree check and evidence persistence remain required. Homing with an AFO fitted remains available outside this empty-machine suite under the normal workflow.

## Records, interruption and remaining validation

Each of ten runs saves its normal strain CSV, raw rotary CSV and metadata JSON. A uniquely named `motion_suite_<batch-id>.json` records the frozen stages, method flags, repeat numbers, identifiers, file links and outcomes. Per-run `rotary_targeted_acquisition` evidence records bulk/fine commands, final error and measured observations, hold statistics and timestamps. The 30-second window supports stability/noise analysis; it is not an independent sensor calibration.

Stop, Escape, window closure, sensor/controller fault, persistence failure or unresolved cleanup cancels all remaining runs. Completed and partial files remain separate. There is no automatic retry or interrupted-batch resume. Correct the cause and begin a new suite if appropriate; do not relabel previous partial files as complete.

This suite covers the remaining empty-machine characterisation under one setup. Independent setup-level validation, loaded-AFO comparisons, remount reliability and final validation of any revised wider model still require separate experiments. Software was verified with fake devices; laboratory operation has not been established by those tests.

## Offline verification

The 331-test regression suite passed. After UI wording and the additional measured-angle guard in the bulk feedback provider were finalized, all seven remaining-protocol tests passed, including the ten-run recording workflow and a guard check independent of the load-cell callback. Dwell durations and mechanical response are simulated in the full-suite fake-device test; these results do not establish laboratory accuracy or operating clearance.
