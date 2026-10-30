"""Provisional sensor homing policy. No hardware or GUI dependencies."""
from dataclasses import dataclass, asdict
import math
import time


@dataclass(frozen=True)
class HomingPolicy:
    tolerance_deg: float = 0.10
    dwell_s: float = 0.5
    maximum_step_deg: float = 0.5
    maximum_travel_deg: float = 20.0
    maximum_corrections: int = 80
    timeout_s: float = 120.0

    def validate(self):
        if not math.isfinite(self.tolerance_deg) or not 0.05 <= self.tolerance_deg <= 0.5:
            raise ValueError('Provisional stopping band must be between ±0.05 and ±0.5 degrees')
        return self


def home(policy, observe, move, check, clock=time.monotonic):
    """Each observation covers fresh samples and stationary motor evidence.

    move uses the app's existing position/trajectory execution. check enforces
    cancellation, feedback and session identity. This never establishes zero.
    """
    policy.validate()
    started, travel, corrections, stalled = clock(), 0.0, [], 0

    def guard():
        check()
        if clock()-started > policy.timeout_s:
            raise RuntimeError('Provisional homing timed out')

    def sample():
        guard()
        value = observe(policy.dwell_s)
        guard()
        if not all(math.isfinite(value[k]) for k in ('mean_deg', 'min_deg', 'max_deg')):
            raise RuntimeError('Non-finite rotary observation')
        if value.get('samples', 0) < 5:
            raise RuntimeError('Insufficient independent rotary samples')
        return value

    before = sample()
    for index in range(policy.maximum_corrections+1):
        guard()
        if (before['min_deg'] >= -policy.tolerance_deg
                and before['max_deg'] <= policy.tolerance_deg):
            return dict(outcome='sensor_zero_reached', policy=asdict(policy),
                        final_observation=before, corrections=corrections,
                        nominal_travel_deg=travel, zero_reference_written=False)
        if index == policy.maximum_corrections:
            raise RuntimeError('Maximum homing corrections reached')
        error = before['mean_deg']
        # Do not chase a mean near zero when noise crosses the stopping band.
        if abs(error) <= policy.tolerance_deg:
            raise RuntimeError('Rotary noise crosses the stopping band; homing not accepted')
        magnitude = min(policy.maximum_step_deg, max(0.02, abs(error)*0.5))
        delta = -math.copysign(magnitude, error)
        if travel+abs(delta) > policy.maximum_travel_deg:
            raise RuntimeError('Maximum cumulative homing travel reached')
        guard()
        move(delta)
        travel += abs(delta)
        after = sample()
        corrections.append(dict(before_deg=error, nominal_step_deg=delta,
                                after_deg=after['mean_deg']))
        improvement = abs(error)-abs(after['mean_deg'])
        if improvement < -0.03:
            raise RuntimeError('Measured angle moved away from zero; check direction/coupling')
        stalled = stalled+1 if improvement < 0.005 else 0
        if stalled >= 3:
            raise RuntimeError('Insufficient rotary progress; check coupling or motion')
        before = after
