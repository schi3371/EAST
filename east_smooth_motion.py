"""Offline candidate for filtered position streaming. This module does no I/O."""
from dataclasses import dataclass
import math

PRESET = 'smooth_rotary_pair'
PERIOD_S = .05
BANDWIDTH = 20.
HALF_RANGE_DEG = 10.
PLATEAU_HALF_RANGE_DEG = 8.
STUDY_SPEEDS = (.5, 5., 10., 20., 30., 40., 50.)


def plateau_acceleration(speed):
    """Peak acceleration for the cubic speed ramp over the available 2 degrees."""
    return .75 * speed**2 / (HALF_RANGE_DEG - PLATEAU_HALF_RANGE_DEG)


def runnable_study_conditions(motion):
    """Select conditions that meet the full requested plateau within current limits."""
    conditions = []
    for speed in STUDY_SPEEDS:
        acceleration = max(motion["minimum_acceleration_deg_s2"], plateau_acceleration(speed))
        if (motion["minimum_speed_deg_s"] <= speed <= motion["maximum_speed_deg_s"]
                and acceleration <= motion["maximum_acceleration_deg_s2"]):
            conditions.append((speed, acceleration))
    return conditions


def clamp(value, lower, upper):
    return min(upper, max(lower, value))


def smoothstep(u):
    u = clamp(u, 0., 1.)
    return u**3 * (10. + u * (-15. + 6. * u))


@dataclass(frozen=True)
class Sweep:
    start: float
    target: float
    peak: float
    ramp: float
    cruise: float

    @classmethod
    def plan(cls, start, target, speed, acceleration):
        if (not all(math.isfinite(v) for v in (start, target, speed, acceleration))
                or max(abs(start), abs(target)) > 15 or start == target
                or speed <= 0 or acceleration <= 0):
            raise ValueError("Invalid candidate sweep")
        distance = abs(target - start)
        # The cubic speed ramp has maximum acceleration 1.5 * peak / ramp.
        peak = min(speed, math.sqrt(distance * acceleration / 1.5))
        ramp = 1.5 * peak / acceleration
        cruise = max(0., distance / peak - ramp)
        return cls(start, target, peak, ramp, cruise)

    @property
    def duration(self):
        return 2 * self.ramp + self.cruise

    @property
    def direction(self):
        return 1 if self.target > self.start else -1

    def at(self, t):
        if not math.isfinite(t):
            raise ValueError("Invalid trajectory time")
        if t <= 0:
            return self.start, 0.
        if t >= self.duration:
            return self.target, 0.
        if t < self.ramp:
            u = t / self.ramp
            travel = self.peak * self.ramp * u**3 * (1 - .5 * u)
            speed = self.peak * u**2 * (3 - 2 * u)
        elif t < self.ramp + self.cruise:
            travel = self.peak * (t - .5 * self.ramp)
            speed = self.peak
        else:
            u = (self.duration - t) / self.ramp
            remaining = self.peak * self.ramp * u**3 * (1 - .5 * u)
            speed = self.peak * u**2 * (3 - 2 * u)
            return self.target - self.direction * remaining, self.direction * speed
        return self.start + self.direction * travel, self.direction * speed


class FilteredPositionSweep:
    """Generate requests for a future POS_FILTER adapter.

    Feedback is optional and deliberately slow. Both the reference and the
    measurement use the same leg clock. Trim is zero at the start and endpoint.
    """

    GAIN = .15
    DEAD_BAND_DEG = .05
    TRIM_CAP_DEG = .10
    TRIM_RATE_DEG_S = .20
    FILTER_TAU_S = .25
    MAX_SAMPLE_AGE_S = .10

    def __init__(self, sweep, model, start_motor_turns, *, feedback=False):
        if model.variant != "candidate" or not math.isfinite(start_motor_turns):
            raise ValueError("Expected frozen wider cubic and a finite start")
        self.sweep = sweep
        self.model = model
        self.feedback = feedback
        self.start_motor_turns = start_motor_turns
        self.start_offset = start_motor_turns - model.target(sweep.start, sweep.direction)
        self.previous_time = None
        self.previous_sample = None
        self.filtered_error = None
        self.trim = 0.

    def command(self, elapsed, *, measured=None, sample_time=None):
        if not math.isfinite(elapsed) or elapsed < 0:
            raise ValueError("Invalid command time")
        if self.previous_time is None and elapsed != 0:
            raise ValueError("First command must start at the held position")
        if self.previous_time is not None and elapsed <= self.previous_time:
            raise ValueError("Command time must increase")
        dt = 0. if self.previous_time is None else elapsed - self.previous_time
        desired, velocity = self.sweep.at(elapsed)
        direction = self.sweep.direction
        model_target = self.model.target(desired, direction)
        filtered_error, trim = self.filtered_error, self.trim
        error = None
        if self.feedback:
            if (measured is None or sample_time is None or not math.isfinite(measured)
                    or not math.isfinite(sample_time) or not 0 <= elapsed - sample_time <= self.MAX_SAMPLE_AGE_S
                    or (self.previous_sample is not None and sample_time <= self.previous_sample)):
                raise ValueError("Rotary sample is stale, invalid, or out of order")
            error = self.sweep.at(sample_time)[0] - measured
            if filtered_error is None:
                filtered_error = error
            else:
                weight = -math.expm1(-(sample_time - self.previous_sample) / self.FILTER_TAU_S)
                filtered_error += weight * (error - filtered_error)
            outside_dead_band = math.copysign(max(0., abs(filtered_error) - self.DEAD_BAND_DEG), filtered_error)
            # Restrict trim to the tested travel, with room to slew back to zero.
            cap = min(self.TRIM_CAP_DEG, self.TRIM_RATE_DEG_S * max(0., self.sweep.duration - elapsed))
            # Use a headroom envelope that itself changes at no more than the
            # trim slew rate, even when the fixture is moving much faster.
            ratio = min(1., self.TRIM_RATE_DEG_S / self.sweep.peak)
            lower = -min(cap, ratio * (15. + desired))
            upper = min(cap, ratio * (15. - desired))
            requested = clamp(self.GAIN * outside_dead_band, lower, upper)
            trim = clamp(requested, trim - self.TRIM_RATE_DEG_S * dt, trim + self.TRIM_RATE_DEG_S * dt)

        # The directional inverse changes at reversal. Blend the new leg from
        # the held motor position over the acceleration ramp.
        blend_distance = min(2., abs(self.sweep.target - self.sweep.start))
        u = min(1., abs(desired - self.sweep.start) / blend_distance)
        offset = self.start_offset * (1. - smoothstep(u))
        offset_velocity = -self.start_offset * 30. * u**2 * (1. - u)**2 * abs(velocity) / blend_distance
        slope = self.model.slope(model_target, direction)
        nominal = model_target + offset
        if elapsed >= self.sweep.duration:
            trim = 0.
        # The headroom envelope above is invariant under the slew limiter.
        # Clip the angle only to absorb roundoff at the model boundaries.
        corrected = clamp(desired + trim, -15., 15.)
        target = self.start_motor_turns if elapsed == 0 else self.model.target(corrected, direction) + offset
        self.previous_time = elapsed
        self.previous_sample = sample_time
        self.filtered_error, self.trim = filtered_error, trim
        return dict(time_s=elapsed, angle_deg=desired, angular_speed_deg_s=velocity,
                    timestamp_aligned_error_deg=error, filtered_error_deg=filtered_error,
                    rotary_trim_deg=trim, nominal_motor_turns=nominal,
                    nominal_motor_speed_turns_s=velocity / slope + offset_velocity,
                    target_motor_turns=target)
