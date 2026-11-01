"""Experimental direction-cubic feed-forward. No hardware calls. Extrapolation is explicitly recorded.

Coefficients describe established empty-fixture sweeps, not reversal dynamics.
A fresh rotary/motor anchor translates the old fit origin into each new run.
"""
from dataclasses import dataclass
import math

MODEL_ID = 'direction-cubic-20261102-v1'
CALIBRATION_SHA256 = 'bf6af002589b5e9f1da180437bcfff5eff212c7788578fa9a6ba0dc1a3b461f9'
# Identical accepted profile with Windows CRLF or repository LF line endings.
CALIBRATION_HASHES = (CALIBRATION_SHA256, 'a8c561f2fd75e4e9c2db123c76c9bae8168eaae559d077d2578126daae927201')
B = (-0.1427128197, 2.3654951387, 0.0010129331, -0.0058765584)
C = (-0.1461277652, -0.0002424609, -0.0006208927, 0.0000061889)
X_LIMIT = 4.38
TARGET_LIMIT_DEG = 15.0
INVERSE_LIMIT_TURNS = 8.0  # Monotonic numerical bracket; covers both branches through +/-15 degrees.
ORIGINAL_METHOD = "Original (2.055 degrees/turn)"
UPDATED_METHOD = "Updated cubic (experimental)"


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError('Motion model inputs must be finite')
    return value


def curve(x, direction):
    x = finite(x)
    if direction not in (-1, 1):
        raise ValueError('Model direction must be +1 or -1')
    if abs(x) > INVERSE_LIMIT_TURNS + 1e-10:
        raise ValueError("Model coordinate exceeds the monotonic inverse bracket")
    return sum((b + direction*c)*x**i for i, (b,c) in enumerate(zip(B,C)))


def inverse(angle, direction):
    angle = finite(angle)
    lo, hi = -INVERSE_LIMIT_TURNS, INVERSE_LIMIT_TURNS
    if not curve(lo,direction) <= angle <= curve(hi,direction):
        raise ValueError('Target angle exceeds the monotonic inverse bracket')
    for _ in range(60):
        mid = (lo+hi)/2
        if curve(mid,direction) < angle: lo = mid
        else: hi = mid
    return (lo+hi)/2


def enabled(config):
    settings = config['motion'].get('experimental_model', {})
    if not isinstance(settings, dict) or type(settings.get('enabled', False)) is not bool:
        raise ValueError('motion.experimental_model.enabled must be true or false')
    if settings.get('model_id', MODEL_ID) != MODEL_ID:
        raise ValueError('Unknown experimental motion model ID')
    return settings.get('enabled', False)


def validate_range(minimum, maximum):
    for value in (minimum, maximum):
        if not 0 < finite(value) <= TARGET_LIMIT_DEG:
            raise ValueError('Target exceeds the existing +/-15 degree machine envelope')


def provenance():
    return dict(model_id=MODEL_ID, rotary_calibration_sha256=CALIBRATION_SHA256, b=list(B), c=list(C), fitted_motor_range_turns=[-X_LIMIT,X_LIMIT],
                target_limit_deg=TARGET_LIMIT_DEG, extrapolation_allowed=True, inverse_bracket_turns=[-INVERSE_LIMIT_TURNS,INVERSE_LIMIT_TURNS], fit_nominal_speeds_deg_s=[0.5,5.0],
                status='experimental_not_prospectively_validated',
                control='inverse-cubic feed-forward targets, not continuous rotary feedback',
                zero_alignment='provisional zero anchor, refined once after initial increasing positioning; fixed during numbered cycles',
                speed_method='secant average conversion per segment; instantaneous angular speed is not constant',
                reversals='branch selected from planned destination direction; transient behaviour unvalidated')


@dataclass(frozen=True)
class RunModel:
    origin_turns: float
    anchor_turns: float
    anchor_angle_deg: float

    @classmethod
    def anchored(cls, motor_turns, rotary_angle_deg):
        motor, angle = finite(motor_turns), finite(rotary_angle_deg)
        if abs(angle) > 0.1:
            raise ValueError('Experimental model must be anchored at rotary zero within +/-0.1 degree')
        return cls(motor-inverse(angle,1), motor, angle)

    @classmethod
    def observed(cls, motor_turns, rotary_angle_deg, direction):
        motor, angle = finite(motor_turns), finite(rotary_angle_deg)
        return cls(motor-inverse(angle,direction), motor, angle)

    def target(self, angle, direction):
        angle = finite(angle)
        if abs(angle) > TARGET_LIMIT_DEG:
            raise ValueError('Target exceeds the existing +/-15 degree machine envelope')
        return self.origin_turns + inverse(angle,direction)

    def metadata(self):
        return dict(**provenance(),origin_turns=self.origin_turns,anchor_motor_turns=self.anchor_turns,
                    anchor_rotary_angle_deg=self.anchor_angle_deg,segments=[])
