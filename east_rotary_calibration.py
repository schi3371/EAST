"""Offline local rotary calibration. No hardware/GUI imports or control writes."""
import csv
import math
import statistics
from pathlib import Path


INPUT_MODES = ("Voltage ratio", "Voltage")
SAMPLE_FIELDS = (
    "point_id", "role", "reference_angle_deg", "repeat", "approach",
    "host_timestamp_iso", "host_monotonic_s", "raw_reading", "input_mode", "unit",
    "sensor_valid", "motor_position_turns", "motor_velocity_turns_s",
    "motor_feedback_valid", "hub_serial", "measured_supply_v",
    "capture_mode", "motor_reference_id", "motor_feedback_status",
)
POINT_FIELDS = (
    "point_id", "role", "reference_angle_deg", "repeat", "approach", "input_mode",
    "unit", "samples", "raw_mean", "raw_sd", "raw_min", "raw_max",
    "motor_position_mean_turns", "motor_span_turns", "valid", "reason",
    "capture_mode", "motor_reference_id",
)


def finite_number(value, name):
    try:
        number = float(value)
    except (ValueError, TypeError) as exc:
        raise ValueError(f"{name} must be numeric") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be finite")
    return number


def reading_valid(value, mode):
    if mode not in INPUT_MODES:
        return False
    return math.isfinite(value) and 0 <= value <= (1.0 if mode == "Voltage ratio" else 5.3)


def summarise_hold(samples, metadata, *, failure=None):
    """Keep all samples; failed holds remain exported and excluded from fitting."""
    mode = metadata["input_mode"]
    raw = [float(s["raw_reading"]) for s in samples if math.isfinite(float(s["raw_reading"]))]
    motor = [float(s["motor_position_turns"]) for s in samples
             if s.get("motor_feedback_valid") and s.get("motor_position_turns") is not None]
    reason = failure
    if not reason and len(samples) < 10:
        reason = "Fewer than 10 samples"
    if not reason and any(not s["sensor_valid"] for s in samples):
        reason = "Out-of-range/non-finite sensor reading"
    if not reason and metadata.get("capture_mode", "Sensor + motor") != "Sensor only" and any(not s["motor_feedback_valid"] for s in samples):
        reason = "Motor feedback unavailable or movement detected"
    if not reason and raw and max(raw) - min(raw) > (0.5 if mode == "Voltage ratio" else 2.65):
        reason = "Possible sensor rollover within hold; arithmetic mean is unsuitable"
    return {
        **metadata, "samples": len(samples),
        "raw_mean": statistics.mean(raw) if raw else None,
        "raw_sd": statistics.stdev(raw) if len(raw) > 1 else 0.0 if raw else None,
        "raw_min": min(raw) if raw else None, "raw_max": max(raw) if raw else None,
        "motor_position_mean_turns": statistics.mean(motor) if motor else None,
        "motor_span_turns": max(motor) - min(motor) if motor else None,
        "valid": not bool(reason), "reason": reason or "",
    }


def linear_fit(x, y):
    if len(x) != len(y) or len(x) < 3:
        raise ValueError("At least three paired observations are required")
    if any(not math.isfinite(v) for v in x + y):
        raise ValueError("Fit values must be finite")
    xm, ym = statistics.mean(x), statistics.mean(y)
    sxx = sum((v - xm) ** 2 for v in x)
    if sxx <= 1e-18:
        raise ValueError("Readings have insufficient variation to fit a scale")
    slope = sum((a - xm) * (b - ym) for a, b in zip(x, y)) / sxx
    if abs(slope) < 1e-12:
        raise ValueError("No measurable angular response")
    intercept = ym - slope * xm
    residuals = [b - (slope * a + intercept) for a, b in zip(x, y)]
    syy = sum((v - ym) ** 2 for v in y)
    return {
        "slope": slope, "intercept": intercept,
        "rmse_deg": math.sqrt(statistics.mean(v * v for v in residuals)),
        "max_abs_residual_deg": max(abs(v) for v in residuals),
        "r_squared": 1 - sum(v * v for v in residuals) / syy if syy else None,
        "count": len(x),
    }


def fit_calibration(points):
    training = [p for p in points if p["valid"] and p["role"] == "Calibration"]
    if len(training) < 3:
        raise ValueError("Capture at least three valid calibration points")
    modes = {p["input_mode"] for p in points}
    if len(modes) != 1:
        raise ValueError("Do not mix voltage and ratio modes in one calibration")
    angles = [finite_number(p["reference_angle_deg"], "Reference angle") for p in training]
    if len(set(angles)) < 3 or not any(abs(a) < 1e-9 for a in angles) or not min(angles) < 0 < max(angles):
        raise ValueError("Include physical zero and distinct independently measured angles on both sides")
    readings = [finite_number(p["raw_mean"], "Sensor mean") for p in training]
    mode = next(iter(modes))
    if max(readings) - min(readings) > (0.5 if mode == "Voltage ratio" else 2.65):
        raise ValueError("Possible rollover across points. A local straight-line fit is unsuitable")
    model = linear_fit(readings, angles)
    model.update({
        "input_mode": mode, "unit": "V/V" if mode == "Voltage ratio" else "V",
        "reference_angle_range_deg": [min(angles), max(angles)],
        "raw_reading_range": [min(readings), max(readings)],
        "fitted_zero_reading": -model["intercept"] / model["slope"],
        "status": "provisional_local_fit_not_automatically_accepted",
        "method": "OLS: independently measured fixture angle = slope * raw reading + intercept",
        "motor_control_changed": False,
    })
    motor_points = [p for p in training if p.get("capture_mode", "Sensor + motor") != "Sensor only"
                    and p.get("motor_position_mean_turns") is not None]
    model["motor_conversion_point_count"] = len(motor_points)
    if len(motor_points) >= 3:
        try:
            model["motor_conversion_fit"] = linear_fit(
                [float(p["motor_position_mean_turns"]) for p in motor_points],
                [float(p["reference_angle_deg"]) for p in motor_points],
            )
        except ValueError as exc:
            model["motor_conversion_fit_error"] = str(exc)
    validation = []
    for p in points:
        if p["valid"] and p["role"] == "Validation":
            predicted = model["slope"] * p["raw_mean"] + model["intercept"]
            validation.append({
                "point_id": p["point_id"], "reference_angle_deg": p["reference_angle_deg"],
                "predicted_angle_deg": predicted, "error_deg": predicted - p["reference_angle_deg"],
                "within_calibrated_range": min(angles) <= p["reference_angle_deg"] <= max(angles)
                    and min(readings) <= p["raw_mean"] <= max(readings),
            })
    model["validation"] = validation
    model["validation_count"] = len(validation)
    if validation:
        model["validation_rmse_deg"] = math.sqrt(statistics.mean(p["error_deg"] ** 2 for p in validation))
        model["validation_max_abs_error_deg"] = max(abs(p["error_deg"]) for p in validation)
    return model


def write_table(path, fields, rows):
    """Rewrite session summaries; raw readings are appended/flushed separately."""
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    temporary.replace(path)


class StationaryCaptureGuard:
    """Reserve existing motion ownership without commanding/arming the motor."""
    def __init__(self, app):
        self.app = app
        self.token = None
        if app.startup_block_reason or app.odrive_adapter is None or app.strain_test_active:
            raise ValueError("Connect ODrive and finish any active run before capturing")
        mapping = app.reference_manager.require_verified()
        if app.fixture_id_input.get().strip() != app.reference_manager.record.fixture_id:
            raise ValueError("Fixture ID must match the verified machine zero")
        self.reference_id = mapping.reference_id
        self.token = app.motion_coordinator.acquire("rotary-static-capture")
        try:
            self.origin = app.get_feedback().position_turns
            self.check()
        except Exception:
            self.close()
            raise

    def check(self):
        self.app.motion_coordinator.assert_active(self.token)
        mapping = self.app.reference_manager.require_verified()
        if mapping.reference_id != self.reference_id or self.app.odrive_adapter is None:
            raise ValueError("Machine-zero reference or ODrive connection changed during capture")
        feedback = self.app.get_feedback()
        conversion = self.app.system_config["motion"]["afo_degrees_per_odrive_turn"]
        velocity_limit = self.app.system_config["reference"]["stationary_velocity_limit_deg_s"] / conversion
        position_limit = self.app.system_config["motion"]["position_tolerance_deg"] / conversion
        if (feedback.active_errors or feedback.current_state != 1
                or abs(feedback.velocity_turns_s) > velocity_limit
                or abs(feedback.position_turns - self.origin) > position_limit):
            raise ValueError("Motor must remain idle and stationary throughout the hold")
        return feedback

    def close(self):
        if self.token is not None:
            self.app.motion_coordinator.release(self.token)
            self.token = None


class SensorOnlyCaptureGuard:
    """Reserve passive observation; never verify zero, clear faults or arm a motor."""
    def __init__(self, app):
        self.app, self.token, self.origin = app, None, None
        if app.strain_test_active:
            raise ValueError("Finish the active test before sensor-only capture")
        self.token = app.motion_coordinator.reserve_observation("rotary-sensor-only-capture")
        try:
            self.check()
        except Exception:
            self.close()
            raise

    def check(self):
        if self.app.motion_coordinator.active_token != self.token:
            raise ValueError("Sensor-only capture reservation was cancelled")
        # Missing/stale ODrive feedback is permitted for passive sensor capture.
        # Any available turns are observations only, never a motor calibration.
        try:
            feedback = self.app.get_feedback()
        except Exception:
            return None
        if feedback is None:
            return None
        conversion = self.app.system_config["motion"]["afo_degrees_per_odrive_turn"]
        velocity_limit = self.app.system_config["reference"]["stationary_velocity_limit_deg_s"] / conversion
        position_limit = self.app.system_config["motion"]["position_tolerance_deg"] / conversion
        if feedback.current_state != 1 or abs(feedback.velocity_turns_s) > velocity_limit:
            raise ValueError("Stop motor movement before sensor-only capture")
        if self.origin is None:
            self.origin = finite_number(feedback.position_turns, "Observed motor position")
        if abs(feedback.position_turns - self.origin) > position_limit:
            raise ValueError("Movement detected during sensor-only hold")
        return feedback

    def close(self):
        if self.token is not None:
            self.app.motion_coordinator.release(self.token)
            self.token = None
