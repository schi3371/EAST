"""Official rotary sensor homing policy. No hardware or GUI dependencies."""
from dataclasses import dataclass, asdict
import math
import time


ZERO_TOLERANCE_DEG = 0.10
ZERO_DWELL_S = 0.5


@dataclass(frozen=True)
class HomingPolicy:
    tolerance_deg: float = ZERO_TOLERANCE_DEG
    dwell_s: float = ZERO_DWELL_S
    maximum_step_deg: float = 0.5
    maximum_travel_deg: float = 20.0
    maximum_corrections: int = 80
    timeout_s: float = 120.0

    def validate(self):
        if not math.isfinite(self.tolerance_deg) or self.tolerance_deg != ZERO_TOLERANCE_DEG:
            raise ValueError('The approved homing acceptance band is fixed at ±0.10 degrees')
        if (self.dwell_s != ZERO_DWELL_S or not 0 < self.maximum_step_deg <= 0.5
                or not 0 < self.maximum_travel_deg <= 20 or not 0 < self.timeout_s <= 120
                or not isinstance(self.maximum_corrections, int) or not 1 <= self.maximum_corrections <= 80):
            raise ValueError('Homing policy exceeds the approved movement/settling limits')
        return self


def home(policy, observe, move, check, clock=time.monotonic, *, centre_mean_deg=0.04):
    """Each observation covers fresh samples and stationary motor evidence.

    move uses the app's existing position/trajectory execution. check enforces
    cancellation, feedback and session identity. The app persists zero only after recording and post-idle checks pass.
    """
    policy.validate()
    if not math.isfinite(centre_mean_deg) or not 0 < centre_mean_deg <= policy.tolerance_deg:
        raise ValueError('Centring mean must lie inside the fixed acceptance band')
    def accepted(value):
        return (value['min_deg'] >= -policy.tolerance_deg
                and value['max_deg'] <= policy.tolerance_deg
                and abs(value['mean_deg']) <= centre_mean_deg)
    started, travel, corrections, stalled = clock(), 0.0, [], 0
    stalled_travel = 0.0

    def guard():
        check()
        if clock()-started > policy.timeout_s:
            raise RuntimeError('Rotary homing timed out')

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
        if accepted(before):
            return dict(outcome='sensor_zero_reached', policy=asdict(policy),
                        final_observation=before, corrections=corrections,
                        centring_mean_limit_deg=centre_mean_deg,
                        nominal_travel_deg=travel, zero_reference_written=False)
        if index == policy.maximum_corrections:
            raise RuntimeError('Maximum homing corrections reached')
        error = before['mean_deg']
        # A narrow window near an edge can be centred; excessive noise cannot.
        if before['max_deg']-before['min_deg'] > 0.16:
            raise RuntimeError('Rotary noise span exceeds centring allowance; homing not accepted')
        if abs(error) < 0.01:
            raise RuntimeError('Rotary noise crosses the stopping band despite centred mean; homing not accepted')
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
        if accepted(after):
            before = after
            continue
        improvement = abs(error)-abs(after['mean_deg'])
        if improvement < -0.03:
            raise RuntimeError('Measured angle moved away from zero; check direction/coupling')
        # Tiny corrections can remain within the observed sensor/mechanical
        # response plateau. Require bounded commanded travel as well as three
        # unsuccessful observations before diagnosing absent motion.
        if improvement < 0.005:
            stalled += 1
            stalled_travel += abs(delta)
        else:
            stalled, stalled_travel = 0, 0.0
        if stalled >= 3 and stalled_travel >= 0.15:
            raise RuntimeError('Insufficient rotary progress; check coupling or motion')
        before = after
