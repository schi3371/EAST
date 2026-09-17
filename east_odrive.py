"""Serialized ODrive access used by the EAST GUI and inspection utility."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Optional

from east_reference import FeedbackError, FeedbackSnapshot, HardwareFingerprint, utc_now_iso


class UnsupportedCoordinateModeError(RuntimeError):
    """Raised when the connected axis cannot safely use EAST relative setpoints."""


def _read_path(root: Any, path: str, default: Any = None) -> Any:
    value = root
    try:
        for part in path.split("."):
            value = getattr(value, part)
        return value
    except Exception:
        return default


def _finite_float(value: Any, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise FeedbackError(f"{label} is not numeric") from exc
    if not math.isfinite(number):
        raise FeedbackError(f"{label} is not finite")
    return number


@dataclass(frozen=True)
class IdleResult:
    requested: bool
    confirmed: bool
    active_errors: Optional[int]
    current_state: Optional[int]
    detail: str


class ODriveAdapter:
    """Thread-safe adapter; all fibre traffic passes through one re-entrant lock."""

    def __init__(self, device: Any, axis_number: int, idle_state: int, closed_loop_state: int):
        self.device = device
        self.axis_number = int(axis_number)
        self.axis = getattr(device, f"axis{self.axis_number}")
        self.idle_state = int(idle_state)
        self.closed_loop_state = int(closed_loop_state)
        self.io_lock = threading.RLock()

    def snapshot(
        self,
        include_phase: bool = False,
        max_capture_duration_s: Optional[float] = None,
    ) -> FeedbackSnapshot:
        capture_started = time.monotonic()
        with self.io_lock:
            position = _finite_float(self.axis.pos_vel_mapper.pos_rel, "ODrive position")
            velocity = _finite_float(self.axis.pos_vel_mapper.vel, "ODrive velocity")
            active_errors = int(self.axis.active_errors)
            current_state = int(self.axis.current_state)
            disarm_reason = int(_read_path(self.axis, "disarm_reason", 0) or 0)
            uptime_value = _read_path(self.device, "system_stats.uptime")
            uptime = None if uptime_value is None else _finite_float(uptime_value, "ODrive uptime")
            raw_phase = None
            if include_phase:
                value = _read_path(self.device, "rs485_encoder_group0.raw")
                if value is not None:
                    raw_phase = _finite_float(value, "RS485 encoder raw phase")
        capture_finished = time.monotonic()
        snapshot = FeedbackSnapshot(
            captured_monotonic_s=capture_finished,
            captured_at=utc_now_iso(),
            position_turns=position,
            velocity_turns_s=velocity,
            active_errors=active_errors,
            current_state=current_state,
            disarm_reason=disarm_reason,
            system_uptime=uptime,
            raw_phase=raw_phase,
            capture_started_monotonic_s=capture_started,
            capture_duration_s=capture_finished - capture_started,
        )
        snapshot.validate(max_capture_duration_s=max_capture_duration_s)
        return snapshot

    def validate_motion_capabilities(
        self,
        expected_mapper_scale: float,
        scale_tolerance: float = 1e-9,
    ) -> Dict[str, Any]:
        """Fail closed unless the live axis matches EAST's relative-frame model."""
        missing = object()
        with self.io_lock:
            absolute = _read_path(
                self.axis, "controller.config.absolute_setpoints", missing
            )
            circular = _read_path(
                self.axis, "controller.config.circular_setpoints", missing
            )
            mapper_scale = _read_path(
                self.axis, "pos_vel_mapper.config.scale", missing
            )
            required = {
                "pos_vel_mapper.pos_rel": _read_path(
                    self.axis, "pos_vel_mapper.pos_rel", missing
                ),
                "pos_vel_mapper.vel": _read_path(
                    self.axis, "pos_vel_mapper.vel", missing
                ),
                "controller.input_pos": _read_path(
                    self.axis, "controller.input_pos", missing
                ),
                "requested_state": _read_path(self.axis, "requested_state", missing),
                "current_state": _read_path(self.axis, "current_state", missing),
                "active_errors": _read_path(self.axis, "active_errors", missing),
            }
        absent = [name for name, value in required.items() if value is missing]
        if absent:
            raise UnsupportedCoordinateModeError(
                "ODrive is missing required motion capabilities: " + ", ".join(absent)
            )
        if absolute is not False:
            raise UnsupportedCoordinateModeError(
                "controller.config.absolute_setpoints must be explicitly false"
            )
        if circular is not False:
            raise UnsupportedCoordinateModeError(
                "controller.config.circular_setpoints must be explicitly false"
            )
        scale = _finite_float(mapper_scale, "position/velocity mapper scale")
        expected = _finite_float(expected_mapper_scale, "required mapper scale")
        tolerance = abs(_finite_float(scale_tolerance, "mapper scale tolerance"))
        if not math.isclose(scale, expected, rel_tol=0.0, abs_tol=tolerance):
            raise UnsupportedCoordinateModeError(
                f"position/velocity mapper scale {scale:g} does not match "
                f"required {expected:g}"
            )
        _finite_float(required["pos_vel_mapper.pos_rel"], "ODrive position")
        _finite_float(required["pos_vel_mapper.vel"], "ODrive velocity")
        return {
            "absolute_setpoints": absolute,
            "circular_setpoints": circular,
            "pos_vel_mapper_scale": scale,
            "required_pos_vel_mapper_scale": expected,
        }

    def fingerprint(self) -> HardwareFingerprint:
        """Read identity and frame-relevant configuration without changing hardware."""
        with self.io_lock:
            serial = _read_path(self.device, "serial_number", "unknown")
            firmware = ".".join(
                str(_read_path(self.device, name, "?"))
                for name in ("fw_version_major", "fw_version_minor", "fw_version_revision")
            )
            details = {
                "absolute_setpoints": _read_path(self.axis, "controller.config.absolute_setpoints"),
                "circular_setpoints": _read_path(self.axis, "controller.config.circular_setpoints"),
                "circular_setpoint_range": _read_path(
                    self.axis, "controller.config.circular_setpoint_range"
                ),
                "pos_vel_mapper_scale": _read_path(self.axis, "pos_vel_mapper.config.scale"),
                "pos_vel_mapper_offset_valid": _read_path(
                    self.axis, "pos_vel_mapper.config.offset_valid"
                ),
                "commutation_mapper_scale": _read_path(
                    self.axis, "commutation_mapper.config.scale"
                ),
                "commutation_mapper_offset_valid": _read_path(
                    self.axis, "commutation_mapper.config.offset_valid"
                ),
                "load_encoder": _read_path(self.axis, "config.load_encoder"),
                "commutation_encoder": _read_path(self.axis, "config.commutation_encoder"),
                "motor_type": _read_path(self.axis, "config.motor.motor_type"),
                "rs485_encoder_protocol_mode": _read_path(
                    self.device, "config.rs485_encoder_group0.mode",
                    _read_path(self.device, "rs485_encoder_group0.config.mode"),
                ),
            }
        return HardwareFingerprint.build(serial, self.axis_number, firmware, details)

    def read_only_report(self, include_phase: bool = True) -> Dict[str, Any]:
        snapshot = self.snapshot(include_phase=include_phase)
        fingerprint = self.fingerprint()
        with self.io_lock:
            pos_abs = _read_path(self.axis, "pos_vel_mapper.pos_abs")
            report = {
                "snapshot": snapshot.to_dict(),
                "fingerprint": fingerprint.to_dict(),
                "capabilities": {
                    "pos_abs_available": pos_abs is not None,
                    "pos_abs": pos_abs,
                    "raw_phase_available": snapshot.raw_phase is not None,
                    "watchdog_feed_available": callable(_read_path(self.axis, "watchdog_feed")),
                },
                "controller": {
                    "input_mode": _read_path(self.axis, "controller.config.input_mode"),
                    "control_mode": _read_path(self.axis, "controller.config.control_mode"),
                    "velocity_limit_turns_s": _read_path(
                        self.axis, "controller.config.vel_limit"
                    ),
                },
                "trajectory": {
                    "velocity_limit_turns_s": _read_path(
                        self.axis, "trap_traj.config.vel_limit"
                    ),
                    "acceleration_limit_turns_s2": _read_path(
                        self.axis, "trap_traj.config.accel_limit"
                    ),
                    "deceleration_limit_turns_s2": _read_path(
                        self.axis, "trap_traj.config.decel_limit"
                    ),
                },
                "health": {
                    "pos_vel_mapper_status": _read_path(
                        self.axis, "pos_vel_mapper.status"
                    ),
                    "pos_vel_mapper_active_errors": _read_path(
                        self.axis, "pos_vel_mapper.active_errors"
                    ),
                    "rs485_encoder_status": _read_path(
                        self.device, "rs485_encoder_group0.status"
                    ),
                    "rs485_encoder_active_errors": _read_path(
                        self.device, "rs485_encoder_group0.active_errors"
                    ),
                },
                "watchdog": {
                    "enabled": _read_path(self.axis, "config.enable_watchdog"),
                    "timeout_s": _read_path(self.axis, "config.watchdog_timeout"),
                },
            }
        return report

    def configure_trajectory(
        self,
        velocity_turns_s: float,
        acceleration_turns_s2: float,
        controller_velocity_limit_turns_s: float,
        control_mode: int,
        input_mode: int,
    ) -> None:
        velocity = _finite_float(velocity_turns_s, "trajectory velocity")
        acceleration = _finite_float(acceleration_turns_s2, "trajectory acceleration")
        controller_limit = _finite_float(
            controller_velocity_limit_turns_s, "controller velocity limit"
        )
        if velocity <= 0 or acceleration <= 0 or controller_limit < velocity:
            raise ValueError("Trajectory limits must be positive and internally consistent")
        with self.io_lock:
            self.axis.controller.config.control_mode = int(control_mode)
            self.axis.controller.config.input_mode = int(input_mode)
            self.axis.trap_traj.config.vel_limit = velocity
            self.axis.trap_traj.config.accel_limit = acceleration
            self.axis.trap_traj.config.decel_limit = acceleration
            self.axis.controller.config.vel_limit = controller_limit

    def command_position(self, target_turns: float) -> None:
        target = _finite_float(target_turns, "target position")
        with self.io_lock:
            self.axis.controller.input_pos = target

    def enter_closed_loop_holding_current(
        self,
        cancelled=lambda: False,
        timeout_s: float = 2.0,
        expected_mapper_scale: float = 1.0,
        mapper_scale_tolerance: float = 1e-9,
    ) -> FeedbackSnapshot:
        self.request_closed_loop_holding_current(
            cancelled=cancelled,
            expected_mapper_scale=expected_mapper_scale,
            mapper_scale_tolerance=mapper_scale_tolerance,
        )
        return self.wait_for_state(self.closed_loop_state, cancelled, timeout_s)

    def request_closed_loop_holding_current(
        self,
        cancelled=lambda: False,
        expected_mapper_scale: float = 1.0,
        mapper_scale_tolerance: float = 1e-9,
    ) -> None:
        """Load the live position and issue one closed-loop state request."""
        self.validate_motion_capabilities(expected_mapper_scale, mapper_scale_tolerance)
        if cancelled():
            raise RuntimeError("Motion was cancelled before arming")
        with self.io_lock:
            current = _finite_float(self.axis.pos_vel_mapper.pos_rel, "ODrive position")
            # Setpoint is loaded before arming so closed loop cannot jump to a stale target.
            self.axis.controller.input_pos = current
            if cancelled():
                raise RuntimeError("Motion was cancelled before closed-loop request")
            self.axis.requested_state = self.closed_loop_state

    def wait_for_state(
        self,
        expected_state: int,
        cancelled=lambda: False,
        timeout_s: float = 2.0,
        progress=None,
    ) -> FeedbackSnapshot:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if cancelled():
                raise RuntimeError("Motion was cancelled while entering closed loop")
            snapshot = self.snapshot()
            if snapshot.active_errors:
                raise RuntimeError(
                    f"ODrive active errors entering closed loop: {snapshot.active_errors}"
                )
            if progress is not None:
                progress()
            if snapshot.current_state == int(expected_state):
                return snapshot
            time.sleep(0.02)
        raise TimeoutError(
            f"ODrive did not enter state {int(expected_state)} within {timeout_s:g} seconds"
        )

    def request_idle(self, timeout_s: float = 1.0) -> IdleResult:
        try:
            with self.io_lock:
                self.axis.requested_state = self.idle_state
        except Exception as exc:
            return IdleResult(False, False, None, None, f"idle request failed: {exc}")
        deadline = time.monotonic() + timeout_s
        last: Optional[FeedbackSnapshot] = None
        try:
            while time.monotonic() < deadline:
                last = self.snapshot()
                if last.current_state == self.idle_state:
                    return IdleResult(
                        True,
                        True,
                        last.active_errors,
                        last.current_state,
                        "idle confirmed",
                    )
                time.sleep(0.02)
        except Exception as exc:
            return IdleResult(True, False, None, None, f"idle confirmation failed: {exc}")
        return IdleResult(
            True,
            False,
            last.active_errors if last else None,
            last.current_state if last else None,
            "idle request sent but idle was not confirmed",
        )

    def configure_watchdog(self, enabled: bool, timeout_s: float) -> None:
        """Configure the optional watchdog; callers must own its feed lifecycle."""
        timeout = _finite_float(timeout_s, "watchdog timeout")
        if timeout <= 0:
            raise ValueError("Watchdog timeout must be positive")
        with self.io_lock:
            self.axis.config.watchdog_timeout = timeout
            self.axis.config.enable_watchdog = bool(enabled)

    def feed_watchdog(self) -> None:
        with self.io_lock:
            self.axis.watchdog_feed()
