"""Hardware-independent configuration, conversion, and logging helpers for EAST."""

from __future__ import annotations

import json
import math
import os
import re
import subprocess
import uuid
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, Optional, Tuple


APP_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_PATH = APP_DIR / "tester_config.json"

CSV_COLUMNS = [
    "Timestamp ISO 8601",
    "Elapsed Time (s)",
    "Sample Index",
    "Cycle",
    "Motion Phase",
    "Movement Direction",
    "Commanded AFO Speed (deg/s)",
    "Commanded AFO Acceleration (deg/s^2)",
    "Commanded Minimum Angle (deg)",
    "Commanded Maximum Angle (deg)",
    "Commanded Cycles",
    "File Name Prefix",
    "Operator ID",
    "AFO ID",
    "Fixture ID",
    "Calibration ID",
    "Test Type",
    "Preset Name",
    "Preset Version",
    "Preset Modified",
    "Commanded ODrive Velocity (turns/s)",
    "Nominal Move Distance (deg)",
    "Expected Constant-Speed Span (deg)",
    "Raw ODrive Position (turns)",
    "ODrive-Derived AFO Angle (deg)",
    "Moving Avg ODrive-Derived AFO Angle (deg)",
    "Raw Voltage Ratio (V/V)",
    "Tare Offset (V/V)",
    "Raw Mass (kg)",
    "Raw Weight (g)",
    "Moving Avg Weight (g)",
    "Raw Force (N)",
    "Raw Torque (Nm)",
    "Moving Avg Torque (Nm)",
    "Raw ODrive Velocity (turns/s)",
    "ODrive-Derived AFO Velocity (deg/s)",
    "Axis Active Errors",
]


@dataclass(frozen=True)
class TestParameters:
    """A validated snapshot of all operator-entered values for one test run."""

    file_prefix: str
    operator: str
    afo_id: str
    fixture_id: str
    calibration_id: str
    test_type: str
    cycles: int
    commanded_afo_speed_deg_s: float
    commanded_afo_acceleration_deg_s2: float
    min_angle_deg: float
    max_angle_deg: float


@dataclass(frozen=True)
class EmptyMachineTare:
    """A load-cell tare captured with the fixture at machine zero and no AFO fitted."""

    offset_v_per_v: float
    captured_at: str
    operator_id: str
    fixture_id: str
    calibration_id: str
    phidget_serial_number: Optional[int]
    phidget_channel: int
    sample_count: int

    def __post_init__(self) -> None:
        if not math.isfinite(float(self.offset_v_per_v)):
            raise ValueError("Empty-machine tare offset must be finite")
        for value, label in (
            (self.captured_at, "capture time"),
            (self.operator_id, "operator ID"),
            (self.fixture_id, "fixture ID"),
            (self.calibration_id, "calibration ID"),
        ):
            if not str(value).strip():
                raise ValueError(f"Empty-machine tare {label} is required")
        if int(self.phidget_channel) < 0:
            raise ValueError("Empty-machine tare Phidget channel must not be negative")
        if int(self.sample_count) <= 0:
            raise ValueError("Empty-machine tare sample count must be positive")

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


def validate_empty_machine_tare(
    tare: Optional[EmptyMachineTare],
    parameters: TestParameters,
    config: Dict[str, Any],
    connected_phidget_serial_number: Optional[int] = None,
) -> Tuple[bool, str]:
    """Confirm that a session tare belongs to the current test and load-cell identity."""
    if tare is None:
        return False, "Capture an empty-machine tare before mounting and testing the AFO."
    mismatches = empty_machine_tare_identity_mismatches(
        tare,
        parameters.fixture_id,
        parameters.calibration_id,
        config,
    )
    if mismatches:
        return False, "; ".join(mismatches)
    expected_channel = int(config["hardware"]["phidget_channel"])
    if tare.phidget_channel != expected_channel:
        return False, "Phidget channel changed after the empty-machine tare was captured."
    configured_serial = config["hardware"].get("phidget_serial_number")
    if configured_serial is not None and tare.phidget_serial_number != int(configured_serial):
        return False, "Configured Phidget serial number does not match the tare record."
    if (
        connected_phidget_serial_number is not None
        and tare.phidget_serial_number is not None
        and tare.phidget_serial_number != int(connected_phidget_serial_number)
    ):
        return False, "Connected Phidget is not the device used for the empty-machine tare."
    return True, "Empty-machine tare is valid for this measurement session."


def empty_machine_tare_identity_mismatches(
    tare: Optional[EmptyMachineTare],
    fixture_id: str,
    calibration_id: str,
    config: Dict[str, Any],
) -> list[str]:
    """Return non-destructive identity-entry mismatches for a stored tare."""
    if tare is None:
        return []
    mismatches = []
    entered_fixture = str(fixture_id).strip()
    entered_calibration = str(calibration_id).strip()
    if entered_fixture != tare.fixture_id:
        mismatches.append(
            f"Fixture ID '{entered_fixture or '[empty]'}' does not match tare fixture "
            f"'{tare.fixture_id}'"
        )
    if entered_calibration != tare.calibration_id:
        mismatches.append(
            f"Calibration ID '{entered_calibration or '[empty]'}' does not match tare "
            f"calibration '{tare.calibration_id}'"
        )
    expected_channel = int(config["hardware"]["phidget_channel"])
    if tare.phidget_channel != expected_channel:
        mismatches.append(
            f"Phidget channel {expected_channel} does not match tare channel "
            f"{tare.phidget_channel}"
        )
    configured_serial = config["hardware"].get("phidget_serial_number")
    if configured_serial is not None and tare.phidget_serial_number != int(configured_serial):
        mismatches.append("Configured Phidget serial number does not match the tare record")
    return mismatches


PRESET_MOTION_FIELDS = (
    "cycles",
    "minimum_angle_deg",
    "maximum_angle_deg",
    "speed_deg_s",
    "acceleration_deg_s2",
)


def preset_motion_values(preset: Dict[str, Any]) -> Dict[str, Any]:
    return {key: preset[key] for key in PRESET_MOTION_FIELDS}


def preset_values_modified(values: Dict[str, Any], preset: Dict[str, Any]) -> bool:
    """Compare editable motion fields numerically with a loaded preset."""
    expected = preset_motion_values(preset)
    try:
        if int(str(values["cycles"]).strip()) != int(expected["cycles"]):
            return True
        for key in PRESET_MOTION_FIELDS[1:]:
            if not math.isclose(
                float(values[key]), float(expected[key]), rel_tol=0.0, abs_tol=1e-9
            ):
                return True
    except (KeyError, TypeError, ValueError):
        return True
    return False


def make_preset_metadata(
    preset_key: str,
    preset: Optional[Dict[str, Any]],
    parameters: TestParameters,
    loaded_at: str,
    modified: bool,
) -> Dict[str, Any]:
    actual = {
        "cycles": parameters.cycles,
        "minimum_angle_deg": parameters.min_angle_deg,
        "maximum_angle_deg": parameters.max_angle_deg,
        "speed_deg_s": parameters.commanded_afo_speed_deg_s,
        "acceleration_deg_s2": parameters.commanded_afo_acceleration_deg_s2,
    }
    return {
        "preset_key": preset_key,
        "preset_display_name": preset["display_name"] if preset else "Custom",
        "preset_version": preset["version"] if preset else None,
        "test_type": parameters.test_type,
        "original_preset_values": preset_motion_values(preset) if preset else {},
        "actual_commanded_values": actual,
        "preset_modified": bool(modified),
        "loaded_at": loaded_at,
    }


def load_tester_config(path: Optional[Path] = None) -> Dict[str, Any]:
    config_path = Path(path or DEFAULT_CONFIG_PATH)
    with config_path.open("r", encoding="utf-8") as handle:
        config = json.load(handle)

    required_sections = {
        "hardware", "motion", "controller", "load_cell", "torque",
        "acquisition", "logging", "reference", "test_presets",
    }
    missing = sorted(required_sections.difference(config))
    if missing:
        raise ValueError(f"Tester configuration is missing: {', '.join(missing)}")
    if config["motion"]["afo_degrees_per_odrive_turn"] <= 0:
        raise ValueError("afo_degrees_per_odrive_turn must be positive")
    if not 0 < config["motion"]["minimum_acceleration_deg_s2"] <= config["motion"]["maximum_acceleration_deg_s2"]:
        raise ValueError("Configured acceleration limits are invalid")
    if config["motion"]["minimum_constant_speed_span_deg"] <= 0:
        raise ValueError("minimum_constant_speed_span_deg must be positive")
    if not 0 < config["motion"]["minimum_speed_deg_s"] <= config["motion"]["maximum_speed_deg_s"]:
        raise ValueError("Configured speed limits are invalid")
    if config["motion"]["controller_velocity_safety_multiplier"] <= 1:
        raise ValueError("controller_velocity_safety_multiplier must be greater than 1")
    if config["motion"]["maximum_afo_angle_deg"] <= 0:
        raise ValueError("maximum_afo_angle_deg must be positive")
    if config["hardware"]["odrive_connection_timeout_s"] <= 0:
        raise ValueError("odrive_connection_timeout_s must be positive")
    if config["hardware"]["phidget_attachment_timeout_ms"] <= 0:
        raise ValueError("phidget_attachment_timeout_ms must be positive")
    if config["acquisition"]["sample_interval_ms"] <= 0:
        raise ValueError("sample_interval_ms must be positive")
    if config["acquisition"]["moving_average_window_samples"] <= 0:
        raise ValueError("moving_average_window_samples must be positive")
    if config["load_cell"]["mass_kg_per_voltage_ratio"] == 0:
        raise ValueError("mass_kg_per_voltage_ratio must be non-zero")
    if config["load_cell"]["tare_samples"] <= 0:
        raise ValueError("tare_samples must be positive")
    if config["torque"]["lever_arm_m"] <= 0:
        raise ValueError("lever_arm_m must be positive")
    if len(config["torque"]["force_angle_polynomial_deg"]) != 3:
        raise ValueError("force_angle_polynomial_deg must contain three coefficients")
    reference = config["reference"]
    for key in (
        "feedback_poll_interval_ms", "feedback_stale_after_ms", "checkpoint_interval_ms",
        "maximum_feedback_capture_ms", "settle_dwell_ms", "post_idle_observation_ms",
    ):
        if float(reference[key]) <= 0:
            raise ValueError(f"reference.{key} must be positive")
    if float(reference["feedback_stale_after_ms"]) <= float(
        reference["maximum_feedback_capture_ms"]
    ):
        raise ValueError(
            "reference.feedback_stale_after_ms must exceed maximum_feedback_capture_ms"
        )
    if not math.isfinite(float(reference["required_pos_vel_mapper_scale"])):
        raise ValueError("reference.required_pos_vel_mapper_scale must be finite")
    if float(reference["pos_vel_mapper_scale_tolerance"]) < 0:
        raise ValueError("reference.pos_vel_mapper_scale_tolerance must not be negative")
    if (
        reference["session_continuity_enabled"]
        and reference.get("odrive_uptime_units") not in ("seconds", "milliseconds")
    ):
        raise ValueError(
            "reference.odrive_uptime_units must be explicitly verified before "
            "session continuity is enabled"
        )
    if reference["watchdog_enabled"] and not reference["watchdog_health_coupled_verified"]:
        raise ValueError(
            "ODrive watchdog cannot be enabled until health-coupled feeding is verified"
        )
    for key in (
        "stationary_velocity_limit_deg_s", "settle_velocity_limit_deg_s",
        "idle_confirmation_timeout_s", "recovery_jog_step_deg",
        "recovery_jog_maximum_cumulative_deg",
        "recovery_speed_deg_s", "recovery_acceleration_deg_s2", "watchdog_timeout_s",
    ):
        if float(reference[key]) <= 0:
            raise ValueError(f"reference.{key} must be positive")
    if reference["phase_recovery_enabled"]:
        for key in ("phase_units", "phase_period", "controller_turns_per_phase_period", "phase_sign"):
            if reference.get(key) is None:
                raise ValueError(f"reference.{key} is required when phase recovery is enabled")
        if int(reference["phase_sign"]) not in (-1, 1):
            raise ValueError("reference.phase_sign must be -1 or +1")
    required_preset_fields = {
        "display_name", "version", "test_type", *PRESET_MOTION_FIELDS,
    }
    display_names = set()
    for key, preset in config["test_presets"].items():
        missing_fields = required_preset_fields.difference(preset)
        if missing_fields:
            raise ValueError(
                f"test_presets.{key} is missing: {', '.join(sorted(missing_fields))}"
            )
        if preset["test_type"] not in ("afo_test", "empty_machine_baseline"):
            raise ValueError(f"test_presets.{key}.test_type is invalid")
        if preset["display_name"] in display_names:
            raise ValueError("Test-preset display names must be unique")
        display_names.add(preset["display_name"])
    return config


def validate_test_parameters(values: Dict[str, Any], config: Dict[str, Any]) -> TestParameters:
    motion = config["motion"]
    required_text = {
        "file_prefix": "file prefix",
        "operator": "operator",
        "fixture_id": "fixture ID",
        "calibration_id": "calibration ID",
    }
    cleaned: Dict[str, str] = {}
    for key, label in required_text.items():
        cleaned[key] = str(values.get(key, "")).strip()
        if not cleaned[key]:
            raise ValueError(f"Enter a {label}.")
    test_type = str(values.get("test_type", "custom")).strip()
    if test_type not in ("custom", "afo_test", "empty_machine_baseline"):
        raise ValueError("Test type is invalid.")
    entered_afo_id = str(values.get("afo_id", "")).strip()
    if test_type == "empty_machine_baseline":
        cleaned["afo_id"] = "EMPTY_MACHINE_BASELINE"
    elif entered_afo_id:
        cleaned["afo_id"] = entered_afo_id
    else:
        raise ValueError("Enter an AFO ID.")

    try:
        cycles = int(str(values["cycles"]).strip())
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Number of cycles must be a whole number.") from exc
    numeric_fields = (
        ("speed_deg_s", "Speed"),
        ("acceleration_deg_s2", "Acceleration"),
        ("min_angle_deg", "Minimum angle"),
        ("max_angle_deg", "Maximum angle"),
    )
    numeric_values = {}
    for key, label in numeric_fields:
        try:
            numeric_values[key] = float(values[key])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"{label} must be numeric.") from exc
    speed = numeric_values["speed_deg_s"]
    acceleration = numeric_values["acceleration_deg_s2"]
    min_angle = numeric_values["min_angle_deg"]
    max_angle = numeric_values["max_angle_deg"]

    if str(values["cycles"]).strip() != str(cycles):
        raise ValueError("Cycles must be a whole number.")
    if not 1 <= cycles <= int(motion["maximum_cycles"]):
        raise ValueError(f"Cycles must be between 1 and {motion['maximum_cycles']}.")
    if not motion["minimum_speed_deg_s"] <= speed <= motion["maximum_speed_deg_s"]:
        raise ValueError(
            f"Speed must be between {motion['minimum_speed_deg_s']:g} and "
            f"{motion['maximum_speed_deg_s']:g} \N{DEGREE SIGN}/s."
        )
    if not motion["minimum_acceleration_deg_s2"] <= acceleration <= motion["maximum_acceleration_deg_s2"]:
        raise ValueError(
            f"Acceleration must be between {motion['minimum_acceleration_deg_s2']:g} and "
            f"{motion['maximum_acceleration_deg_s2']:g} \N{DEGREE SIGN}/s\N{SUPERSCRIPT TWO}."
        )
    maximum_angle = float(motion["maximum_afo_angle_deg"])
    if not 0 <= min_angle <= maximum_angle:
        raise ValueError(
            f"Minimum angle must be a magnitude between 0\N{DEGREE SIGN} and "
            f"{maximum_angle:g}\N{DEGREE SIGN}."
        )
    if not 0 <= max_angle <= maximum_angle:
        raise ValueError(
            f"Maximum angle must be a magnitude between 0\N{DEGREE SIGN} and "
            f"{maximum_angle:g}\N{DEGREE SIGN}."
        )
    if min_angle == 0 and max_angle == 0:
        raise ValueError("At least one angle limit must be greater than zero.")
    total_traverse = min_angle + max_angle
    constant_speed_span = constant_speed_span_deg(total_traverse, speed, acceleration)
    required_span = float(motion["minimum_constant_speed_span_deg"])
    if constant_speed_span + 1e-9 < required_span:
        minimum_traverse = speed * speed / acceleration + required_span
        raise ValueError(
            f"The selected speed, acceleration and angle range do not provide the required "
            f"{required_span:g}\N{DEGREE SIGN} constant-speed span. Commanded speed: "
            f"{speed:g}\N{DEGREE SIGN}/s; commanded acceleration: "
            f"{acceleration:g}\N{DEGREE SIGN}/s\N{SUPERSCRIPT TWO}; commanded range: "
            f"-{min_angle:g}\N{DEGREE SIGN} to +{max_angle:g}\N{DEGREE SIGN}; total ROM: "
            f"{total_traverse:g}\N{DEGREE SIGN}; expected constant-speed span: "
            f"{constant_speed_span:.2f}\N{DEGREE SIGN}. Increase total ROM to at least "
            f"{minimum_traverse:.2f}\N{DEGREE SIGN} or adjust speed/acceleration."
        )

    for phase, distance, move_speed, move_acceleration in (
        ("startup to positive endpoint", max_angle, speed, acceleration),
        ("endpoint-to-endpoint sweep", total_traverse, speed, acceleration),
        ("return to machine zero", max_angle,
         motion["neutral_return_speed_deg_s"], motion["neutral_return_acceleration_deg_s2"]),
    ):
        try:
            motion_timeout_seconds(distance, move_speed, move_acceleration, config)
        except ValueError as exc:
            raise ValueError(f"{phase}: {exc}") from exc

    return TestParameters(
        file_prefix=cleaned["file_prefix"],
        operator=cleaned["operator"],
        afo_id=cleaned["afo_id"],
        fixture_id=cleaned["fixture_id"],
        calibration_id=cleaned["calibration_id"],
        test_type=test_type,
        cycles=cycles,
        commanded_afo_speed_deg_s=speed,
        commanded_afo_acceleration_deg_s2=acceleration,
        min_angle_deg=min_angle,
        max_angle_deg=max_angle,
    )


def afo_degrees_to_odrive_turns(degrees: float, config: Dict[str, Any]) -> float:
    return float(degrees) / float(config["motion"]["afo_degrees_per_odrive_turn"])


def odrive_turns_to_afo_degrees(turns: float, config: Dict[str, Any]) -> float:
    return float(turns) * float(config["motion"]["afo_degrees_per_odrive_turn"])


def afo_speed_to_odrive_turns_s(speed_deg_s: float, config: Dict[str, Any]) -> float:
    return afo_degrees_to_odrive_turns(speed_deg_s, config)


def afo_acceleration_to_odrive_turns_s2(acceleration_deg_s2: float, config: Dict[str, Any]) -> float:
    return afo_degrees_to_odrive_turns(acceleration_deg_s2, config)


def constant_speed_span_deg(distance_deg: float, speed_deg_s: float, acceleration_deg_s2: float) -> float:
    """Return the distance remaining after symmetric acceleration and deceleration."""
    distance = abs(float(distance_deg))
    speed = abs(float(speed_deg_s))
    acceleration = abs(float(acceleration_deg_s2))
    if acceleration <= 0:
        raise ValueError("Acceleration must be positive")
    return max(0.0, distance - speed * speed / acceleration)


def calculate_load(voltage_ratio: float, tare_offset: float, config: Dict[str, Any]) -> Tuple[float, float, float]:
    """Return mass in kg, weight in grams, and force in newtons."""
    load_cell = config["load_cell"]
    mass_kg = (float(voltage_ratio) - float(tare_offset)) * float(load_cell["mass_kg_per_voltage_ratio"])
    force_n = mass_kg * float(load_cell["standard_gravity_m_s2"])
    return mass_kg, mass_kg * 1000.0, force_n


def calculate_torque_nm(force_n: float, afo_angle_deg: float, config: Dict[str, Any]) -> float:
    torque = config["torque"]
    quadratic, linear, intercept = torque["force_angle_polynomial_deg"]
    force_angle_deg = quadratic * afo_angle_deg ** 2 + linear * afo_angle_deg + intercept
    return float(force_n) * float(torque["lever_arm_m"]) * math.sin(math.radians(force_angle_deg))


def motion_timeout_seconds(
    distance_deg: float,
    speed_deg_s: float,
    acceleration_deg_s2: float,
    config: Dict[str, Any],
) -> float:
    """Estimate a trapezoidal-trajectory duration and add the safety margin."""
    motion = config["motion"]
    distance = abs(float(distance_deg))
    speed = max(abs(float(speed_deg_s)), float(motion["minimum_speed_deg_s"]))
    acceleration = max(
        abs(float(acceleration_deg_s2)),
        float(motion["minimum_acceleration_deg_s2"]),
    )

    # If the move is too short to reach the velocity limit, acceleration and
    # deceleration form a triangular profile. Otherwise it is trapezoidal.
    acceleration_distance = speed * speed / acceleration
    if distance <= acceleration_distance:
        estimate = 2.0 * math.sqrt(distance / acceleration) if distance else 0.0
    else:
        estimate = distance / speed + speed / acceleration
    estimate += float(motion["motion_timeout_margin_s"])
    maximum = float(motion["maximum_motion_timeout_s"])
    if not math.isfinite(estimate) or estimate > maximum:
        raise ValueError(
            f"Movement of {distance:g} deg at {speed:g} deg/s and "
            f"{acceleration:g} deg/s^2 requires {estimate:.2f} s including margin, "
            f"exceeding the {maximum:g} s maximum motion timeout. "
            "Reduce the angle range or increase speed/acceleration within validated limits."
        )
    return min(estimate, maximum)


def reconcile_run_outcome(
    candidate_completed: bool,
    stop_requested: bool,
    acquisition_error: Optional[str],
    current_status: str,
    current_error: Optional[str],
) -> Tuple[str, Optional[str]]:
    """Latch late stop/acquisition failures before a run is called completed."""
    if acquisition_error:
        if current_error and acquisition_error not in current_error:
            return "error", f"{current_error}; {acquisition_error}"
        return "error", acquisition_error
    if candidate_completed and stop_requested:
        return "aborted", current_error or "stop requested during final observation"
    if candidate_completed:
        return "completed", None
    return current_status, current_error


def sanitise_identifier(value: str, fallback: str = "run") -> str:
    cleaned = re.sub(r"[^A-Za-z0-9._-]+", "_", str(value).strip())
    cleaned = cleaned.strip("._-")
    return cleaned[:80] or fallback


def create_run_paths(parameters: TestParameters, config: Dict[str, Any], base_dir: Optional[Path] = None) -> Tuple[Path, Path]:
    root = Path(base_dir or APP_DIR)
    output_dir = root / config["logging"]["output_directory"]
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S_%f%z")
    name_parts = [sanitise_identifier(parameters.file_prefix)]
    # Empty-machine baselines use a reserved AFO identifier for metadata and
    # matching, but that internal label should not clutter the operator's file
    # name. Real AFO identifiers remain in AFO-test and custom-run filenames.
    if parameters.test_type != "empty_machine_baseline":
        name_parts.append(sanitise_identifier(parameters.afo_id, "AFO"))
    name_parts.append(timestamp)
    stem = "_".join(name_parts)
    csv_path = output_dir / f"{stem}_strain_data.csv"
    metadata_path = output_dir / f"{stem}_metadata.json"
    suffix = 1
    while csv_path.exists() or metadata_path.exists():
        csv_path = output_dir / f"{stem}_{suffix}_strain_data.csv"
        metadata_path = output_dir / f"{stem}_{suffix}_metadata.json"
        suffix += 1
    return csv_path, metadata_path


def git_revision(repo_dir: Optional[Path] = None) -> Dict[str, Any]:
    directory = Path(repo_dir or APP_DIR)
    result: Dict[str, Any] = {"commit": "unknown", "working_tree_dirty": None}
    try:
        commit = subprocess.run(
            ["git", "rev-parse", "HEAD"], cwd=directory, check=True,
            capture_output=True, text=True, timeout=2,
        )
        status = subprocess.run(
            ["git", "status", "--porcelain"], cwd=directory, check=True,
            capture_output=True, text=True, timeout=2,
        )
        result["commit"] = commit.stdout.strip()
        result["working_tree_dirty"] = bool(status.stdout.strip())
    except (OSError, subprocess.SubprocessError):
        pass
    return result


def make_run_metadata(
    parameters: TestParameters,
    config: Dict[str, Any],
    tare_offset: float,
    csv_path: Path,
    odrive_snapshot: Dict[str, Any],
    tare_metadata: Optional[Dict[str, Any]] = None,
    preset_metadata: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    return {
        "schema_version": 2,
        "run_status": "started",
        "started_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "completed_at": None,
        "csv_file": csv_path.name,
        "test_parameters": asdict(parameters),
        "test_parameter_status": (
            "operator-entered commanded values; not independent physical measurements"
        ),
        "protocol": dict(preset_metadata or {}),
        "calibration": {
            "calibration_id": parameters.calibration_id,
            "tare_offset_v_per_v": tare_offset,
            "empty_machine_tare": dict(tare_metadata or {}),
            **config["load_cell"],
            **config["torque"],
        },
        "motion_conversion": dict(config["motion"]),
        "hardware": dict(config["hardware"]),
        "odrive_active_configuration": odrive_snapshot,
        "software": {
            "version": config.get("software_version", "unknown"),
            **git_revision(),
        },
    }


def write_json_atomic(path: Path, payload: Dict[str, Any]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
    with temporary.open("x", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True, allow_nan=False)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(temporary, path)
