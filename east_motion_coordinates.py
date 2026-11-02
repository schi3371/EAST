"""Explicit measured-pose anchor for the fixed Original conversion."""
from dataclasses import dataclass
import math
@dataclass(frozen=True)
class LinearPoseAnchor:
    motor_turns: float
    rotary_angle_deg: float
    degrees_per_turn: float
    def __post_init__(self):
        if (not all(math.isfinite(v) for v in (self.motor_turns,self.rotary_angle_deg,self.degrees_per_turn))
                or self.degrees_per_turn<=0 or abs(self.rotary_angle_deg)>.1):
            raise ValueError('Original run anchor requires finite stationary rotary zero')
    def target(self,angle):
        if not math.isfinite(angle):raise ValueError('Non-finite Original target')
        return self.motor_turns+(angle-self.rotary_angle_deg)/self.degrees_per_turn
    def metadata(self):
        return dict(motor_turns=self.motor_turns,rotary_angle_deg=self.rotary_angle_deg,
            degrees_per_turn=self.degrees_per_turn,zero_target_turns=self.target(0.),
            convention='target = starting motor + (desired angle - measured starting angle) / fixed conversion',
            coefficients_refitted=False)
