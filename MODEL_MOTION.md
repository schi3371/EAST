# Experimental model-driven test motion — v1.11.0

Normal GUI tests and three-speed AFO/baseline batches now use the pooled direction-specific cubic from the 2 November motion analysis. This is experimental feed-forward positioning, not prospective validation or continuous rotary closed-loop tracking. No hardware was operated during implementation.

## Operator use

1. Update EAST with the application closed. Connect, home using the rotary sensor, and capture the normal unloaded tare before mounting an AFO.
2. Load the desired preset. With model mode enabled, preset loading limits each side of ROM to 9 degrees. Original preset definitions remain unchanged; reduced ROM is correctly marked modified. Manually entered ranges over 9 degrees are rejected before motion.
3. Begin with an empty-machine single-speed slow run. Inspect rotary endpoints, reversals, return to zero and measured speed before loaded or faster runs. Completing a run does not validate its accuracy.
4. The three-speed batch uses this implementation. Its 20 degrees/s condition is outside the original 0.5/5 nominal degrees/s training conditions and is labelled accordingly in segment metadata. Loaded operation and reversals are also unvalidated.

The main protocol status and confirmations identify the experimental mode. Existing Stop, sensor freshness, ownership, tare, recording and batch cancellation checks remain. A return outside the existing +/-0.1 degree rotary-zero band during post-idle observation marks the run unsuccessful and prevents the next batch run. Resolve the cause and use Home / Return to Zero; no automatic retry occurs.

## Mapping and anchoring

For fitted coordinate x in motor turns and planned direction d (+1 increasing, -1 decreasing):

    angle = sum((b[i] + d*c[i]) * x**i for i in range(4))

Coefficients and provenance are embedded in `east_motion_model.py` and model-enabled run metadata. Bisection inverts the monotonic branches. No extrapolation beyond the fitted +/-4.38 motor turns is allowed. Targets are limited to +/-9 degrees so both branches remain inside the fitted range.

At run start, fresh stationary motor feedback and a rotary reading within +/-0.1 degree establish a per-run origin using the initial increasing branch. This is an explicit experimental alignment assumption: the original fit has direction offsets, and stationary/reversal branch behaviour has not been independently validated. After the initial unnumbered increasing movement settles, the origin is refined once using that known branch and the fresh rotary/motor observation, then stays fixed for every numbered sweep and return. This is coordinate alignment, not a coefficient refit. Both initial and refined anchors are saved. The initial positioning endpoint is therefore outside the frozen-origin evaluation dataset. Each segment selects its branch from the destination relative to the fresh measured angle. Inconsistent motor direction or a starting coordinate outside the fitted range blocks movement.

The decreasing branch's final zero target need not equal the motor count originally recorded by homing. The verified reference and unloaded tare are retained. In model mode, start readiness uses fresh rotary zero instead of requiring that opposite-direction zero have that original motor count. Reference verification, controller/fixture identity and feedback health remain required. The original rotary calibration hash is required; a changed sensor calibration blocks model operation pending review/refit. Manual movement and homing retain legacy conversions and controls.

## Speed and acceleration

For each segment, motor-turn distance divided by measured angular distance supplies a secant conversion. Requested nominal speed and acceleration are converted with that ratio for the normal ODrive trapezoidal trajectory. This adjusts average spatial conversion; it does not guarantee constant instantaneous angular speed or acceleration. Timeouts use planned measured angular distance. Motor velocity and acceleration are capped at the previous global ceilings converted with the legacy factor. Any cap is recorded per segment. Evaluate actual speed from rotary recordings.

## Records and scope

Per-run `experimental_motion_model` metadata includes model ID, coefficients, fit range, anchor, fixed origin, experimental status, and each segment's measured start angle, target, direction, motor target, conversion ratio and trajectory settings. Existing raw CSV columns are preserved. The motor-derived angle column remains the legacy 2.055-based channel; it is not corrected or rotary-measured angle.

The Motion Modelling identification tool deliberately retains legacy motion and records model disabled, allowing comparable identification data. Manual steps, jogging, homing, calibration and the standalone mapping runner also remain legacy.

## Rollback

Close EAST, set `motion.experimental_model.enabled` to `false` in `tester_config.json`, and restart. Reload the preset to restore its original ROM. This restores normal-test legacy targets, trajectory conversion and start-readiness semantics. Logs remain intact. Editing configuration content intentionally invalidates the standalone CLI reviewed-source guard until reviewed.

The pre-model version is commit `b802d05`. Reverting the model implementation commit provides a repository-wide rollback without deleting recordings.

## Validation remaining

Collect prospective empty-fixture data and evaluate endpoint bias, reversals, actual speed and timing sensitivity. Per-run anchoring, including the initial-position origin refinement, is operational calibration and must be reported; errors after anchoring do not establish transfer without anchoring. Agreement with the calibrated rotary sensor does not independently establish absolute physical-angle accuracy. Implementation does not complete thesis validation.

## Offline verification record

Verification used Python 3.11 with fake devices. The 313-test suite passed before the final motor-ceiling guard. With that guard and its additional test, all 10 model tests passed; the 314-test full run had three failures in existing homing/batch tests (including stale simulated sensor readings). Those three cases passed when rerun individually. Production freshness limits were not relaxed. These intermittent regression-test failures remain a test-stability limitation. Python 3.14's repeated-import test harness also encountered a native compiler crash; Python 3.11 was used for release verification. No physical bench validation has been performed.
