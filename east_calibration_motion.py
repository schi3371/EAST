"""Session-local, operator-established positioning for empty-fixture calibration.

This reference never establishes machine zero or authorises a strain test.
Its angle is an approximate positioning value, not a calibration observation.
"""
from dataclasses import dataclass, field
import math
import threading
from typing import Optional

from east_rotary_calibration import finite_number


@dataclass
class CalibrationPositioningReference:
    connection: object
    fixture_id: str
    operator_id: str
    zero_turns: float
    factor: float
    maximum_angle: float
    last_position_turns: float
    last_uptime: Optional[float]
    active: bool = True
    _lock: object = field(default_factory=threading.RLock, init=False, repr=False)

    @classmethod
    def establish(cls, app, angle, *, acknowledged):
        if not acknowledged:
            raise ValueError("Confirm empty fixture and current calibration positioning angle")
        if app.startup_block_reason or app.odrive_adapter is None or app.strain_test_active:
            raise ValueError("Connect ODrive and finish any test before calibration positioning")
        if app.motion_coordinator.owner is not None or app.feedback_retry_pending:
            raise ValueError("Wait for motion/capture and feedback retry to finish")
        if app.monitor_error:
            raise ValueError("Reconnect ODrive after the feedback monitor fault")
        operator, fixture = app.operator_input.get().strip(), app.fixture_id_input.get().strip()
        if not operator or not fixture:
            raise ValueError("Enter Operator ID and Fixture ID")
        feedback = app.get_feedback()
        cls.stationary(app, feedback)
        angle = finite_number(angle, "Current positioning angle")
        factor = app.system_config["motion"]["afo_degrees_per_odrive_turn"]
        maximum = app.system_config["motion"]["maximum_afo_angle_deg"]
        if abs(angle) > maximum:
            raise ValueError(f"Positioning angle must be within ±{maximum:g} degrees")
        return cls(app.odrive_adapter, fixture, operator,
                   feedback.position_turns - angle / factor, factor, maximum,
                   feedback.position_turns, feedback.system_uptime)

    @staticmethod
    def stationary(app, feedback):
        factor = app.system_config["motion"]["afo_degrees_per_odrive_turn"]
        limit = app.system_config["reference"]["stationary_velocity_limit_deg_s"] / factor
        if feedback.active_errors or feedback.current_state != 1 or abs(feedback.velocity_turns_s) > limit:
            raise ValueError("ODrive must report error-free, stationary idle")

    def check(self, app, *, stationary=False):
        # Both the GUI and motion worker check this reference. Serialize the
        # feedback read/uptime comparison to avoid mistaking an older concurrent
        # observation for a controller reset.
        with self._lock:
            return self._check(app, stationary=stationary)

    def _check(self, app, *, stationary=False):
        if not self.active or app.odrive_adapter is not self.connection:
            self.active = False
            raise ValueError("Reconnect, then confirm the current calibration position")
        context = (app.operator_context if threading.current_thread() is not threading.main_thread()
                   else {"fixture": app.fixture_id_input.get().strip(), "operator": app.operator_input.get().strip()})
        if (context["fixture"] != self.fixture_id
                or context["operator"] != self.operator_id
                or app.system_config["motion"]["afo_degrees_per_odrive_turn"] != self.factor
                or app.system_config["motion"]["maximum_afo_angle_deg"] != self.maximum_angle):
            self.active = False
            raise ValueError("Calibration positioning identity/configuration changed")
        if app.feedback_retry_pending:
            raise ValueError("Wait for fresh feedback")
        feedback = app.get_feedback()
        if feedback.active_errors or app.monitor_error:
            self.active = False
            raise ValueError("ODrive fault: reconnect and confirm the calibration position")
        uptime = feedback.system_uptime
        if self.last_uptime is not None and uptime is not None and uptime < self.last_uptime:
            self.active = False
            raise ValueError("Controller reset: reconnect and confirm the calibration position")
        if stationary:
            self.stationary(app, feedback)
        if uptime is not None:
            self.last_uptime = uptime
        return feedback

    def angle(self, position):
        return (position - self.zero_turns) * self.factor

    def target(self, position, direction, step):
        if direction not in (-1, 1) or not math.isfinite(step) or step <= 0:
            raise ValueError("Invalid calibration step")
        target = position + direction * step
        if abs(self.angle(target)) > self.maximum_angle + 1e-9:
            raise ValueError("Full calibration step exceeds estimated travel limits; no movement commanded")
        return target

    def suspend(self):
        self.active = False
