"""Frozen prospective model screen. No hardware or GUI dependencies."""
from dataclasses import dataclass
import math

from east_motion_model import B as CURRENT_B, C as CURRENT_C, MODEL_ID as CURRENT_ID
from east_wide_candidate import B as WIDE_B, C as WIDE_C, MODEL_ID as WIDE_ID

SCREEN_PRESET = "streamed_model_screen"
# Offline candidate coefficients, frozen before the prospective screen.
LINEAR = (0.0038337377179473746, 2.162006754050806)
SHARED_CUBIC = (-0.12723766341844275, 2.3584913949213813, 0.005303580678974427, -0.0057405206270091675)
QUINTIC = (-0.08197434739733728, 2.355163257919607, -0.002203552617468106, -0.0052794892494939655, 0.00015645829684124372, -5.5958663367775965e-06, -0.1512475832852065, -0.0016824555313649668, -0.0004921542755910412, 0.00019172727550545687, 1.6802156451625648e-05, -3.417108724733485e-06)

MODELS = {
    "original": ("original-2.055", (0.0, 2.055), ()),
    "linear": ("wide-refit-linear-20261102", LINEAR, ()),
    "shared_cubic": ("wide-shared-cubic-20261102", SHARED_CUBIC, ()),
    "current": (CURRENT_ID, CURRENT_B, CURRENT_C),
    "candidate": (WIDE_ID, WIDE_B, WIDE_C),
    "quintic": ("wide-direction-quintic-20261102", QUINTIC[:6], QUINTIC[6:]),
}
# Each block contains every method, with the two wider arms labelled separately.
BLOCKS = (
    ("original", "linear", "shared_cubic", "current", "candidate", "quintic", "feedback"),
    ("candidate", "current", "quintic", "feedback", "shared_cubic", "original", "linear"),
    ("quintic", "feedback", "linear", "candidate", "original", "shared_cubic", "current"),
)
MOTOR_BRACKET = 3.5


def _coefficients(variant, direction):
    _, base, directional = MODELS[variant]
    if direction not in (-1, 1):
        raise ValueError("Invalid model direction")
    return tuple(b + direction * c for b, c in zip(base, directional)) if directional else base


def _inverse(variant, angle, direction):
    coefficients = _coefficients(variant, direction)
    lo, hi = -MOTOR_BRACKET, MOTOR_BRACKET
    if not math.isfinite(angle) or not _evaluate(coefficients, lo) <= angle <= _evaluate(coefficients, hi):
        raise ValueError("Streamed target outside frozen inverse bracket")
    for _ in range(45):
        mid = (lo + hi) / 2
        if _evaluate(coefficients, mid) < angle:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def _evaluate(coefficients, position):
    result = 0.0
    for coefficient in reversed(coefficients):
        result = result * position + coefficient
    return result


@dataclass(frozen=True)
class StreamScreenModel:
    variant: str
    origin_turns: float
    anchor_motor_turns: float
    anchor_rotary_angle_deg: float

    @classmethod
    def anchored(cls, variant, motor, angle):
        if not math.isfinite(motor) or not math.isfinite(angle) or abs(angle) > .1:
            raise ValueError("Streamed model anchor requires rotary zero")
        return cls(variant, motor - _inverse(variant, angle, 1), motor, angle)

    def target(self, angle, direction):
        if abs(angle) > 5:
            raise ValueError("Common-trajectory screen restricted to ±5°")
        return self.origin_turns + _inverse(self.variant, angle, direction)

    def slope(self, motor, direction):
        position = motor - self.origin_turns
        if not math.isfinite(position) or abs(position) > MOTOR_BRACKET:
            raise ValueError("Measured position outside streamed model bracket")
        coefficients = _coefficients(self.variant, direction)
        return _evaluate(tuple(i * b for i, b in enumerate(coefficients) if i), position)

    def metadata(self):
        model_id, base, directional = MODELS[self.variant]
        return dict(model_id=model_id, model_variant=self.variant, b=list(base), c=list(directional),
                    origin_turns=self.origin_turns, anchor_motor_turns=self.anchor_motor_turns,
                    anchor_rotary_angle_deg=self.anchor_rotary_angle_deg,
                    zero_alignment="increasing branch anchored to measured rotary zero; origin fixed for all legs",
                    inverse_bracket_turns=[-MOTOR_BRACKET, MOTOR_BRACKET])
