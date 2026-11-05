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

Keep the run metadata, joined strain CSV, raw rotary CSV and batch manifest together. Use the recorded desired and measured angle samples to score RMS, bias and worst error by sweep direction, reversal/endpoint performance, update gaps and rate limits. Record actual speed and coverage through the planned stiffness window, unassisted return, verification time and failures. Compare one score per run across blocks. A bounded separate homing action may follow a fully persisted run with a small zero miss. Preserve the unassisted outcome and the separate homing evidence. Review and resolve any actual drive or acquisition fault before another test.

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

## Individually selected feedback commissioning stages

After completing the first model screen, the **Feedback Stage** presets commission the wider cubic with feedback off (R1) and on (R2). Both use one cycle and the same desired angle-time trajectory, fixed zero anchor, streaming schedule, proportional gain policy and motor ceilings. Each preset runs only its two conditions. Select the next stage explicitly after reviewing the preceding measurements.

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

Use the Protocol menu with Engineering controls enabled. Load the chosen **Feedback Stage**, press Start and run R1–R2. Confirm physical clearance for the selected range and review actual travel, tracking, update gaps and limiting before another stage. No stage starts another preset.

The planned reference ramp is distinct from the motor trajectory ceilings. The motor speed ceiling is 1.5 times requested fixture speed, capped at the routine maximum converted through 2.055. Acceleration uses 1.5 times reference acceleration with a floor of 5 and cap at the routine ceiling. Target slew uses that motor rate with the existing capped time interval. These caps can prevent the requested fixture rate from being reached. Metadata retains planned reference, applied motor limits, measured motion, rate limiting and timestamps so infeasible settings can be identified. No claim of rate achievement follows from the preset label.

The initial commissioning tracking guard allows the reference travel over the configured motor feedback stale interval plus 0.25° reserve, with a minimum 1°. Actual measured travel is guarded at selected ROM plus 1° and remains subject to the accepted rotary-calibration range. These are provisional fault limits. Evaluate accuracy using the recorded tracking errors, separately from the guard. Sensor freshness, controller faults, Stop, durable recording and post-idle zero checks still apply.

Leg deadlines use the planned ramp duration plus six seconds, within the existing global motion timeout. This accommodates a ±10° traversal at 0.5°/s. The original slow pilot and seven-method screen retain their original settings. Higher stages require bench validation. All commissioning presets are empty-fixture only. Loaded feedback commissioning follows review of the empty-fixture evidence.
