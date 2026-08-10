"""Hardware-independent configuration, conversion, and logging helpers for EAST."""

from __future__ import annotations

import json
import math
import os
import re
import subprocess
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
    "Commanded AFO Speed (deg/s)",
    "Commanded ODrive Velocity (turns/s)",
    "Raw ODrive Position (turns)",
    "Raw AFO Angle (deg)",
    "Moving Avg AFO Angle (deg)",
    "Raw Voltage Ratio (V/V)",
    "Tare Offset (V/V)",
    "Raw Mass (kg)",
    "Raw Weight (g)",
    "Moving Avg Weight (g)",
    "Raw Force (N)",
    "Raw Torque (Nm)",
    "Moving Avg Torque (Nm)",
    "Raw ODrive Velocity (turns/s)",
    "Measured AFO Velocity (deg/s)",
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
    cycles: int
    commanded_afo_speed_deg_s: float
    commanded_afo_acceleration_deg_s2: float
    min_angle_deg: float
    max_angle_deg: float


def load_tester_config(path: Optional[Path] = None) -> Dict[str, Any]:
    config_path = Path(path or DEFAULT_CONFIG_PATH)
    with config_path.open("r", encoding="utf-8") as handle:
        config = json.load(handle)

    required_sections = {
        "hardware", "motion", "controller", "load_cell", "torque",
        "acquisition", "logging",
    }
    missing = sorted(required_sections.difference(config))
    if missing:
        raise ValueError(f"Tester configuration is missing: {', '.join(missing)}")
    if config["motion"]["afo_degrees_per_odrive_turn"] <= 0:
        raise ValueError("afo_degrees_per_odrive_turn must be positive")
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
    return config


def validate_test_parameters(values: Dict[str, Any], config: Dict[str, Any]) -> TestParameters:
    motion = config["motion"]
    required_text = {
        "file_prefix": "file prefix",
        "operator": "operator",
        "afo_id": "AFO ID",
        "fixture_id": "fixture ID",
        "calibration_id": "calibration ID",
    }
    cleaned: Dict[str, str] = {}
    for key, label in required_text.items():
        cleaned[key] = str(values.get(key, "")).strip()
        if not cleaned[key]:
            raise ValueError(f"Enter a {label}.")

    try:
        cycles = int(str(values["cycles"]).strip())
        speed = float(values["speed_deg_s"])
        acceleration = float(values["acceleration_deg_s2"])
        min_angle = float(values["min_angle_deg"])
        max_angle = float(values["max_angle_deg"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError("Cycles, speed, acceleration, and angles must be numeric.") from exc

    if str(values["cycles"]).strip() != str(cycles):
        raise ValueError("Cycles must be a whole number.")
    if not 1 <= cycles <= int(motion["maximum_cycles"]):
        raise ValueError(f"Cycles must be between 1 and {motion['maximum_cycles']}.")
    if not motion["minimum_speed_deg_s"] <= speed <= motion["maximum_speed_deg_s"]:
        raise ValueError(
            f"Speed must be between {motion['minimum_speed_deg_s']} and "
            f"{motion['maximum_speed_deg_s']} deg/s."
        )
    if not motion["minimum_acceleration_deg_s2"] <= acceleration <= motion["maximum_acceleration_deg_s2"]:
        raise ValueError(
            f"Acceleration must be between {motion['minimum_acceleration_deg_s2']} and "
            f"{motion['maximum_acceleration_deg_s2']} deg/s^2."
        )
    maximum_angle = float(motion["maximum_afo_angle_deg"])
    if not 0 <= min_angle <= maximum_angle or not 0 <= max_angle <= maximum_angle:
        raise ValueError(f"Angle magnitudes must be between 0 and {maximum_angle} degrees.")
    if min_angle == 0 and max_angle == 0:
        raise ValueError("At least one angle limit must be greater than zero.")

    return TestParameters(
        file_prefix=cleaned["file_prefix"],
        operator=cleaned["operator"],
        afo_id=cleaned["afo_id"],
        fixture_id=cleaned["fixture_id"],
        calibration_id=cleaned["calibration_id"],
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


def motion_timeout_seconds(distance_deg: float, speed_deg_s: float, config: Dict[str, Any]) -> float:
    motion = config["motion"]
    estimate = abs(float(distance_deg)) / max(float(speed_deg_s), float(motion["minimum_speed_deg_s"]))
    estimate += float(motion["motion_timeout_margin_s"])
    return min(estimate, float(motion["maximum_motion_timeout_s"]))


def sanitise_identifier(value: str, fallback: str = "run") -> str:
    cleaned = re.sub(r"[^A-Za-z0-9._-]+", "_", str(value).strip())
    cleaned = cleaned.strip("._-")
    return cleaned[:80] or fallback


def create_run_paths(parameters: TestParameters, config: Dict[str, Any], base_dir: Optional[Path] = None) -> Tuple[Path, Path]:
    root = Path(base_dir or APP_DIR)
    output_dir = root / config["logging"]["output_directory"]
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S_%f%z")
    stem = "_".join([
        sanitise_identifier(parameters.file_prefix),
        sanitise_identifier(parameters.afo_id, "AFO"),
        timestamp,
    ])
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
) -> Dict[str, Any]:
    return {
        "schema_version": 1,
        "run_status": "started",
        "started_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "completed_at": None,
        "csv_file": csv_path.name,
        "test_parameters": asdict(parameters),
        "calibration": {
            "calibration_id": parameters.calibration_id,
            "tare_offset_v_per_v": tare_offset,
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
    temporary = path.with_suffix(path.suffix + ".tmp")
    with temporary.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")
    os.replace(temporary, path)
