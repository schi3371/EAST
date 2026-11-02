# Three-method comparison and separate rotary-feedback pilot — v1.15.0

## Main comparison

Empty fixture only. Connect, allow automatic zero verification, and capture unloaded tare before loading the preset. Select **Model Comparison — Original / Current / Wider (Empty)**, Load Preset, Start, then Run Model Comparison. Default starting run is 1. Review clearance and the complete plan.

22 runs: Original 2.055 commissioning at +/-5 and +/-10, then wider-cubic commissioning at +/-5 and +/-10 (one cycle each, 5 degrees/s). Eighteen main runs: three methods x three speeds x two runs, each +/-15 nominal degrees and three cycles. Block 1: 0.5, 5, 20 degrees/s, Original/Current/Wider at each. Block 2 reverses speeds and methods. Total 58 numbered cycles. Allow 60–90 minutes including normal preparation/settling. This reverses order but is not a fully randomised/counterbalanced experiment.

Original ROM is NOMINAL under 2.055; the actual measured travel may differ. All comparison methods stop on rotary readings beyond +/-15.1 degrees. Confirm clearance, inspect commissioning results and use Stop if movement is unexpected. No feedback endpoint correction masks the main comparison. Slow full-range moves are split into <=30-second bounded legs; intermediate stops must be excluded from constant-speed analysis. The existing global timeout remains unchanged. A fault or zero-return miss halts the sequence. Restart only the interrupted run using a NEW batch's starting-run field; prior files remain separate.

The wider candidate is the frozen cubic in east_wide_candidate.py, fitted from batch 408453cc. The fifth-order fit is not deployed. Both cubics retain their operational initial increasing-branch origin alignment, fixed for numbered cycles. Normal GUI model selection is unchanged. Per-run model variant, coefficients/model ID, motor coordinates and raw rotary/strain data are saved. This is one empty-fixture setup, not independent remount or loaded-AFO reliability.

## Separate pilot

After reviewing the main comparison and confirming healthy operation, load **Rotary Feedback Pilot — Empty +/-5 at 0.5 dps**. Run from 1. Two runs at +/-5, one cycle each: (1) streamed wider-cubic feedforward reference; (2) identical streaming with proportional rotary feedback. Allow about 3–5 minutes. This does not automatically escalate range or speed.

Both runs use a smooth desired trajectory with maximum angular rate 0.5 degrees/s and 0.5 degrees/s² acceleration. The existing ODrive motor loop remains active. The outer loop requests updates every 0.1 seconds (not a real-time guarantee), proportional gain 0.3 for feedback and zero for reference. Requested correction is clamped to +/-0.1 angle-equivalent degrees using the local model slope. Motor-target slew is bounded at 0.75/2.055 turns/s; each slew interval is capped at 0.2 seconds. ODrive trajectory limits are also applied. There is no integral term. Rate limiting is present in both reference and feedback runs.

The pilot faults on invalid/stale sensor or controller feedback, cancellation/recording failure, measured travel beyond +/-6 degrees, tracking error beyond 1 degree, invalid model derivative/input, or a leg taking more than 30 seconds. These are provisional pilot guards, not validated clinical acceptance criteria. They do not replace physical clearance or the E-stop. An unstable/poorly tracking pilot must not be automatically retried.

Normal CSVs retain raw rotary/motor/torque data. Metadata rotary_feedback_pilot.samples retains desired and measured angle, error, measured motor turns, feedforward target, requested/applied correction, rate-limit flag, command target, sensor sequence/timestamp, feedback timestamp, phase/cycle and command issuance time. Metadata is finalised on orderly completion/fault/Stop; sudden process/power loss may leave that desired-command trace incomplete even when raw CSV data survive. Final motor settling is not itself an angular accuracy pass; normal post-idle zero qualification remains required.

Compare tracking and speed using identical commanded trajectories, and inspect reversals/overshoot, sampling/update intervals, correction saturation and command limiting. Selecting a starting run of 2 deliberately skips the reference and is not a paired comparison. These are commissioning data for a new controller, not proof of improved performance. Keep the main open-loop comparison and feedback-pilot datasets separate.
