# Selecting the test motion method — v1.12.0

The main Test Parameters panel now has a **Motion method** menu:

- **Original (2.055 degrees/turn)** uses the original fixed conversion for normal test targets and trajectory settings.
- **Updated cubic (experimental)** uses the direction-specific inverse cubic and rotary alignment described in MODEL_MOTION.md.

Select the method while idle, then load the desired preset and start normally. This is a session selection: it does not rewrite tester_config.json. On restart, the configured default applies. No Git revert or configuration edit is needed to switch within a session.

The selection applies to normal single-speed tests and both three-speed protocols. It is locked during movement, initialization, active batches (including between runs), homing and unresolved cleanup. Changing selection does not clear the unloaded tare or command movement. Normal reference and rotary-zero checks still apply; switching back to the original method may require Home / Return to Zero because it expects the original motor-zero count.

Motion Modelling identification remains explicitly Original regardless of the main selector, to preserve comparable identification data. Every normal run records its actual motion_method and model enabled state. The batch freezes the selected configuration for all three runs.

## Range and extrapolation

The model-specific +/-9-degree clamp has been removed. Loading a preset preserves its original +/-10-degree definition. Requested ranges through the existing +/-15-degree machine setting are accepted. This setting is a configured envelope, not proof of physical clearance or demonstrated accuracy at the boundary.

The cubic coefficients have not been refitted. The fitted coordinate range remains +/-4.38 motor turns; movements beyond it are extrapolated. Each segment records an extrapolated flag, the model's fitted range and coefficients remain in metadata, and confirmations identify extrapolation as permitted. The inverse solver uses a wider monotonic numerical bracket (+/-8 motor turns), which covers both model branches through requested +/-15 degrees without selecting a turning-point root. The numerical bracket is not an experimentally validated range.

Fresh rotary measurements, accepted calibration identity, ownership, Stop, recorder checks, original global motor-rate ceilings and final rotary-zero verification remain enforced. Model positioning remains experimental feed-forward; instantaneous angular speed is not guaranteed constant. No laboratory hardware was operated during implementation.

Original nominal +/-15-degree commands can produce a different measured range. Do not treat the removal of the model fit clamp as a rotary-supervised wide-range identification protocol; that guided acquisition workflow has not yet been implemented.

The existing motion-timeout limits also remain. For example, a full -15 to +15 degree sweep at 0.5 degrees/s needs more than the current 60-second motion-timeout ceiling once the margin is included, so that combination is rejected. The range change does not silently relax this existing timeout.

Offline verification: the full 319-test suite had one error in a simulated batch-window fixture; the other 318 tests passed. After correcting that fixture, all 15 motion-method tests passed on the final rerun. Hardware operation was not tested.
