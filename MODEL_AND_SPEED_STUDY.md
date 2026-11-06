# EAST controller and speed decision study

The decision is the fastest measured test condition that gives acceptable loaded-AFO stiffness repeatability, tracking, and usable torque-angle data. The selected speed is qualified for the studied specimen and setup. The walking waveform can be a separate development study.

## A. Common-trajectory empty-fixture screen

With the fixture empty, verify rotary zero and capture unloaded tare. Turn on **Engineering controls**, load **Streamed Model Screen — Seven Methods (Empty)**, and press **Start**. Run the default range **1–7** first and review before running **8–14** and **15–21**. These are three different orders of the same seven methods.

| First block | Method |
|---|---|
| R1 | Original 2.055 |
| R2 | Refit linear |
| R3 | Shared cubic |
| R4 | Current direction-dependent cubic |
| R5 | Wider direction-dependent cubic |
| R6 | Direction-dependent quintic |
| R7 | Wider direction-dependent cubic with rotary feedback |

One numbered cycle per run, ±5°, 0.5°/s requested trajectory, 0.5°/s² ramp. The motor trajectory ceiling is distinct from the requested ramp. The three new fits use the frozen coefficients from the 2 November wider mapping report. The increasing branch is anchored at measured zero and the origin is held for all legs. The same streaming path and guard envelope apply to all seven arms. The earlier two-run pilot used its own alignment procedure and is commissioning evidence.

Keep the run metadata, joined strain CSV, raw rotary CSV and batch manifest together. Use the recorded desired and measured angle samples to score RMS, bias and worst error by sweep direction, reversal/endpoint performance, update gaps and rate limits. Record actual speed and coverage through the planned stiffness window, unassisted return, verification time and failures. Compare one score per run across blocks. A separate automatic rotary homing action follows a fully persisted run with a model zero miss. Preserve the unassisted outcome and the separate homing evidence. Review and resolve any actual drive or acquisition fault before another test.

## B. Qualify finalist controllers at the intended SOP condition

Shortlist the strongest feedforward fit and the strongest feedback configuration. If the feedforward winner uses a different fit, compare that fit with feedback off and on under the same trajectory before choosing the feedback finalist. Then verify the finalist methods on the empty fixture at the intended range and speed. For streamed control, commission speed at ±5° (1 then 2°/s), acceleration at a proven speed, and range at a proven speed before attempting ±10° at 5°/s. Each escalation needs its own reviewed command limits and tracking evidence. The seven-method preset is fixed to the initial condition. Higher-speed and loaded feedback require later controller commissioning.

For a workflow comparison, include the current native endpoint-driven SOP path at the proposed range and speed. At the selected condition, compare finalists using matched empty traces and the same mounted AFO. Two five-cycle runs per method on each of two independent mounts give eight loaded runs. Alternate order and use cycles 2–4 for each run-level stiffness estimate. Select a controller for the speed study based on loaded tracking, usable data, repeat scatter, zero procedure and time.

## C. Find the fastest reliable loaded speed

Screen 5°/s first, 20°/s if it gives a usable measured window, and 0.5°/s as the slow comparator. Use the existing standard empty and AFO presets for individual native runs. Set the candidate speeds explicitly. Hold motion method, range, acceleration, fixture, branch, angle window, conditioning and rest common. Save matched empty traces for each speed. If comparing different acceleration profiles, repeat one overlapping speed under both profiles.

For each speed that passes screening, use one AFO, six independent remounts, two runs per speed per mount, five recorded cycles per run. Use cycles 2–4 for the run-level slope. Within each mount, execute the speeds in the order shown, then repeat in reverse order with the same rest. For fewer speeds, remove unused conditions from the order and keep starting orders balanced.

| Mount | First pass, °/s |
|---|---|
| M01 | 0.5 → 5 → 20 |
| M02 | 5 → 20 → 0.5 |
| M03 | 20 → 0.5 → 5 |
| M04 | 20 → 5 → 0.5 |
| M05 | 0.5 → 20 → 5 |
| M06 | 5 → 0.5 → 20 |

Name files by mount, speed and repeat, e.g. `REL_M03_5dps_R2`. Three qualified speeds give 36 loaded runs and 180 recorded cycles. Two speeds give 24 runs and 120 cycles. Retain exclusions and timestamps.

Before scoring, agree on the largest acceptable repeat difference and remount variability in stiffness units, based on the stiffness difference the study needs to resolve. For each speed report:
- paired same-mount run differences and repeat scatter
- variability of the six mount means, with individual mount results shown
- within-mount stiffness differences versus 5°/s, kept separate from repeatability
- measured speed and sample coverage within the stiffness window, excluded/faulted runs, and median elapsed time per usable test

Select the fastest **measured** speed that meets the agreed limits and yields complete, usable runs. If 20°/s falls short and 5°/s qualifies, use 5°/s. If faster conditions fail, evaluate 0.5°/s against the same criteria. If stiffness shifts systematically with speed, document the selected rate as part of the measurement definition.

## Combined feedback commissioning sequence

Load **Feedback Commissioning Sequence — All Stages (Empty)** with Engineering controls enabled, then press Start. Choose **Start at stage** and **Stop after stage**. Leave the end blank to select through stage 11. Each stage runs Wider feedforward followed by Wider with rotary feedback, with one cycle per run. The two runs share the desired trajectory, fixed zero anchor, streaming schedule and motor ceilings.

Automatic progression through the selected stage range is the default. **Pause for review after each stage** is optional. If selected before starting, press **Continue to next stage** after reviewing a pair. The confirmation lists all selected motion conditions. Stop and genuine drive, sensor and recording failures halt the sequence. To resume after cleanup, create a new batch at the required stage. Existing recordings remain separate. The older individual stage definitions remain compatible with saved data and are hidden from the preset menu.

| Stage | ROM | Reference speed, °/s | Reference acceleration, °/s² |
|---|---|---:|---:|
| 01 | ±5° | 1 | 1 |
| 02 | ±5° | 2 | 2 |
| 03 | ±10° | 0.5 | 0.5 |
| 04 | ±10° | 2 | 2 |
| 05 | ±10° | 5 | 5 |
| 06 | ±10° | 5 | 20 |
| 07 | ±10° | 10 | 20 |
| 08 | ±10° | 10 | 50 |
| 09 | ±10° | 20 | 100 |
| 10 | ±15° | 5 | 20 |
| 11 | ±15° | 20 | 100 |

Stages 01/02 and 04/05 change speed and acceleration together to commission reachable profiles. Stages 05/06 and 07/08 isolate acceleration at the same speed and ROM. Stage 03 widens the range at the original slow rate. Stages 06/10 and 09/11 isolate ROM. The table defines initial screening conditions, followed by repetitions of usable conditions for repeatability.

Confirm physical clearance through the maximum selected range and review actual travel, tracking, update gaps and command limiting. One batch manifest records the selected stage range, all skipped runs, review approvals and separate data files. A full selection contains 11 stages and 22 runs.

The planned reference ramp is distinct from the motor trajectory ceilings. The motor speed ceiling is 1.5 times requested fixture speed, capped at the routine maximum converted through 2.055. Acceleration uses 1.5 times reference acceleration with a floor of 5 and cap at the routine ceiling. Target slew uses that motor rate with the existing capped time interval. These caps can prevent the requested fixture rate from being reached. Metadata retains planned reference, applied motor limits, measured motion, rate limiting and timestamps so infeasible settings can be identified. No claim of rate achievement follows from the preset label.

The commissioning tracking threshold is reference travel over the configured motor feedback stale interval plus 0.25° reserve, with a minimum 1°. Exceeding it is recorded per sample and does not stop the commissioning run: dynamic lag is an experimental outcome. Actual measured travel is still guarded at selected ROM plus 1° and remains subject to the accepted rotary-calibration range. This remains a provisional fault limit. The original slow pilot and seven-method screen retain their tracking stops. Evaluate accuracy using the recorded tracking errors and threshold flags. Sensor freshness, controller faults, Stop, durable recording and post-idle zero checks still apply.

Leg deadlines use the planned ramp duration plus six seconds, within the existing global motion timeout. This accommodates a ±10° traversal at 0.5°/s. The original slow pilot and seven-method screen retain their original settings. Higher stages require bench validation. All commissioning presets are empty-fixture only. Loaded feedback commissioning follows review of the empty-fixture evidence.


## Automatic recovery after a model zero-return miss

For automated empty-fixture protocols, a fully saved measurement run that misses its unassisted rotary zero is followed by official rotary homing. There is no separate 0.25° eligibility cutoff. Homing retains its own motion, sensor-validity, progress and timeout checks. After zero and the updated reference are verified and saved, the next measurement run starts automatically. Completed sweeps are never replayed.

The run's unassisted return remains recorded as failed, with its measured offset and range. The batch labels its acquisition as recovered after the separate homing action and records the before/after references and evidence file. At the final run, automatic homing still completes before the batch finishes. Stop, drive faults, stale/invalid sensing, incomplete recording or failed homing halt the sequence. Loaded protocols retain their existing behaviour.

## Routine sensor-qualified return to zero

Routine single tests and loaded AFO protocols retain motion ownership through final zero qualification. After the unassisted return is observed in idle, any residual offset triggers the official rotary centring policy within that same run. The fixture is then put in idle and observed again; a post-idle miss triggers another correction attempt within the shared travel, correction-count and time budgets. Completion requires a fresh stationary rotary window inside ±0.10°, confirmed idle and healthy recording/feedback. Stop cancels the operation; no failed measurement sweep is restarted.

`zero_return_check` preserves the original unassisted outcome. `automatic_zero_return` records correction steps, powered centring attempts and the final post-idle result. Correction/observation phases are separate from stiffness sweeps. Tare, model coefficients and the machine-zero reference are not changed. Empty-fixture engineering comparisons retain their separately saved unassisted outcome and existing batch homing path so historical controller comparisons remain interpretable. This software workflow still requires bench confirmation with a fitted AFO before SOP approval.

## Full automated model × speed comparison (empty fixture)

Select **Full Model × Speed Comparison — 0.5 to 50 dps (Empty)** and run the sequence. It has 147 runs: seven methods × seven reference speeds (0.5, 5, 10, 20, 30, 40, 50°/s) × three independently recorded one-cycle repeats, all at ±15° and reference acceleration 100°/s². The methods are streamed Original 2.055, refitted linear, shared cubic, current directional cubic, wider directional cubic, directional quintic, and wider cubic with proportional rotary feedback. Model order rotates across the three repeat blocks at each speed; speed groups ascend. These are repeated empty-fixture acquisitions, not remounting reliability trials.

All arms use the same 40 ms requested update period, desired ramp and motor limits for a given speed. The original conversion is streamed through this common pipeline; it is not the old endpoint-only implementation. Full-range inverses use ±8 motor turns relative to the frozen model origin; both frozen branches are checked for endpoint coverage and positive slope before the plan is admitted. This extends the model evaluation range; it does not establish accuracy at new angles, loads or speeds.

An explicit engineering profile permits requested speed through 50°/s, acceleration no higher than the existing 100°/s², and a 90-second leg budget (the 0.5°/s full traversal takes just over 60 seconds). Ordinary AFO/manual validation retains its existing 20°/s ceiling and 60-second budget. No motor current or power limit is raised. Higher commanded speed is commissioning, not demonstrated machine capability. The full ±15° traversal at 50°/s with acceleration 100°/s² has only 5° of planned constant-speed travel; initial positioning is triangular and does not reach 50°/s. Evaluate measured speed and tracking through the actual analysis window.

Tracking-threshold exceedances are recorded without stopping. Fully saved model zero misses automatically home before the next run. In this preset only, saved model-domain or leg-deadline failures may also home and skip to the next condition if motor idle, healthy sensing and complete recording cleanup are confirmed. Failed/partial attempts remain failed, have their own metadata, and are not replayed or counted as complete cycles. The manifest distinguishes `failed_run_recovered_skipped` from `acquired_zero_recovered` and the batch reports `completed_with_failed_runs` when necessary. Stop, travel guard, drive faults, stale/invalid sensor data, recording failures and unsuccessful homing halt continuation.

Use Start at run / Stop after run to resume a new batch. Speed groups contain 21 runs each: 0.5°/s R1–21; 5 R22–42; 10 R43–63; 20 R64–84; 30 R85–105; 40 R106–126; 50 R127–147. No pause between conditions is required. Allow approximately 90–120 minutes including settling and zero corrections; hardware behaviour determines the actual duration. Keep the laptop awake for the entire run.
