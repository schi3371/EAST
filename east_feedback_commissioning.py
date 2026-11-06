"""Explicit, individually selected empty-fixture feedback pairs."""
from dataclasses import dataclass

PREFIX = "feedback_stage_"
SEQUENCE_PRESET = "feedback_commissioning_sequence"


@dataclass(frozen=True)
class Condition:
    half_range_deg: float
    speed_deg_s: float
    acceleration_deg_s2: float


CONDITIONS = {
    "01": Condition(5., 1., 1.),
    "02": Condition(5., 2., 2.),
    "03": Condition(10., .5, .5),
    "04": Condition(10., 2., 2.),
    "05": Condition(10., 5., 5.),
    "06": Condition(10., 5., 20.),
    "07": Condition(10., 10., 20.),
    "08": Condition(10., 10., 50.),
    "09": Condition(10., 20., 100.),
    "10": Condition(15., 5., 20.),
    "11": Condition(15., 20., 100.),
}


def condition_for(key):
    if not key.startswith(PREFIX):
        return None
    suffix = key[len(PREFIX):]
    if suffix not in CONDITIONS:
        raise ValueError("Unknown feedback commissioning condition")
    return CONDITIONS[suffix]


def policy(config, key):
    condition = condition_for(key)
    if condition is None:
        return None
    motion = config["motion"]
    if (condition.speed_deg_s > motion["maximum_speed_deg_s"]
            or condition.acceleration_deg_s2 > motion["maximum_acceleration_deg_s2"]
            or condition.half_range_deg > motion["maximum_afo_angle_deg"]):
        raise ValueError("Feedback condition exceeds configured motion ceilings")
    # Motor-space headroom stays within the routine configured ceilings.
    motor_speed = min(1.5 * condition.speed_deg_s, motion["maximum_speed_deg_s"])
    motor_acceleration = min(max(5., 1.5 * condition.acceleration_deg_s2),
                             motion["maximum_acceleration_deg_s2"])
    factor = motion["afo_degrees_per_odrive_turn"]
    # Provisional tracking allowance: reference travel during the existing maximum
    # motor-feedback age, plus 0.25° reserve. During commissioning this is a
    # recorded comparison threshold, not a stop: lag is an experimental outcome.
    age_s = config["reference"]["feedback_stale_after_ms"] / 1000
    return dict(half_range_deg=condition.half_range_deg, speed_deg_s=condition.speed_deg_s,
                ramp_acceleration_deg_s2=condition.acceleration_deg_s2,
                motor_speed_nominal_deg_s=motor_speed,
                motor_acceleration_nominal_deg_s2=motor_acceleration,
                max_command_rate_turns_s=motor_speed/factor,
                tracking_guard_deg=max(1., condition.speed_deg_s*age_s+.25),
                travel_guard_deg=condition.half_range_deg+1.,
                tracking_error_action="record",
                tracking_guard_basis="reference travel over motor stale limit plus 0.25° reserve, minimum 1°",
                fixed_origin=True)
