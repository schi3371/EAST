"""Guarded ODrive-only motion diagnostic for the EAST tester.

Despite the historical filename, this is not an ODrive configuration backup.
The JSON file in ``Odrive Backup Config`` is the configuration backup.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from datetime import datetime
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent
sys.path.insert(0, str(PROJECT_DIR))

import odrive
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE
from odrive.enums import CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_TRAP_TRAJ

from east_core import (
    afo_acceleration_to_odrive_turns_s2,
    afo_degrees_to_odrive_turns,
    afo_speed_to_odrive_turns_s,
    load_tester_config,
    motion_timeout_seconds,
    odrive_turns_to_afo_degrees,
    sanitise_identifier,
    write_json_atomic,
)


def wait_and_log(writer, axis, target, zero, cycle, phase, parameters, config, start_time):
    tolerance = afo_degrees_to_odrive_turns(config["motion"]["position_tolerance_deg"], config)
    timeout = motion_timeout_seconds(
        parameters["distance_deg"], parameters["speed_deg_s"], config
    )
    deadline = time.monotonic() + timeout
    while abs(axis.pos_vel_mapper.pos_rel - target) > tolerance:
        if int(axis.active_errors):
            raise RuntimeError(f"ODrive active errors: {int(axis.active_errors)}")
        if time.monotonic() >= deadline:
            raise TimeoutError(f"Motion timed out during {phase} after {timeout:.1f} s")
        position = axis.pos_vel_mapper.pos_rel
        velocity = axis.pos_vel_mapper.vel
        writer.writerow([
            datetime.now().astimezone().isoformat(timespec="milliseconds"),
            f"{time.monotonic() - start_time:.6f}", cycle, phase,
            position, odrive_turns_to_afo_degrees(position - zero, config),
            velocity, odrive_turns_to_afo_degrees(velocity, config),
            parameters["speed_deg_s"], int(axis.active_errors),
        ])
        time.sleep(config["acquisition"]["sample_interval_ms"] / 1000.0)


def main():
    parser = argparse.ArgumentParser(description="Run an ODrive-only EAST motion diagnostic")
    parser.add_argument("--cycles", type=int, required=True)
    parser.add_argument("--speed-deg-s", type=float, required=True)
    parser.add_argument("--acceleration-deg-s2", type=float, required=True)
    parser.add_argument("--angle-deg", type=float, required=True)
    parser.add_argument("--prefix", default="odrive_motion_check")
    parser.add_argument("--confirm-hardware", action="store_true")
    args = parser.parse_args()
    if not args.confirm_hardware:
        raise SystemExit("Refusing to move hardware without --confirm-hardware")

    config = load_tester_config(PROJECT_DIR / "tester_config.json")
    motion = config["motion"]
    if not 1 <= args.cycles <= motion["maximum_cycles"]:
        raise SystemExit("Cycles are outside configured limits")
    if not motion["minimum_speed_deg_s"] <= args.speed_deg_s <= motion["maximum_speed_deg_s"]:
        raise SystemExit("Speed is outside configured limits")
    if not motion["minimum_acceleration_deg_s2"] <= args.acceleration_deg_s2 <= motion["maximum_acceleration_deg_s2"]:
        raise SystemExit("Acceleration is outside configured limits")
    if not 0 < args.angle_deg <= motion["maximum_afo_angle_deg"]:
        raise SystemExit("Angle is outside configured limits")

    hardware = config["hardware"]
    device = None
    output_dir = PROJECT_DIR / config["logging"]["output_directory"]
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S_%f%z")
    output_path = output_dir / f"{sanitise_identifier(args.prefix)}_{stamp}.csv"
    metadata_path = output_path.with_suffix(".json")
    metadata = None
    completed_cycles = 0
    status = "error"
    error = None
    try:
        device = odrive.find_any(
            serial_number=hardware["odrive_serial_number"],
            timeout=hardware["odrive_connection_timeout_s"],
        )
        if device is None:
            raise RuntimeError("ODrive not found")
        axis = getattr(device, f"axis{hardware['odrive_axis']}")
        if int(axis.active_errors):
            raise RuntimeError(f"ODrive has active errors: {int(axis.active_errors)}")

        velocity = afo_speed_to_odrive_turns_s(args.speed_deg_s, config)
        acceleration = afo_acceleration_to_odrive_turns_s2(args.acceleration_deg_s2, config)
        axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        axis.trap_traj.config.vel_limit = velocity
        axis.trap_traj.config.accel_limit = acceleration
        axis.trap_traj.config.decel_limit = acceleration
        axis.controller.config.vel_limit = velocity * motion["controller_velocity_safety_multiplier"]
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        zero = axis.pos_vel_mapper.pos_rel
        excursion = afo_degrees_to_odrive_turns(args.angle_deg, config)
        parameters = {
            "distance_deg": 2 * args.angle_deg,
            "speed_deg_s": args.speed_deg_s,
        }
        start_time = time.monotonic()
        metadata = {
            "schema_version": 1,
            "source": "Testing Scripts/odrive_backup.py",
            "run_status": "started",
            "started_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "csv_file": output_path.name,
            "hardware": hardware,
            "command": {
                "cycles": args.cycles,
                "speed_deg_s": args.speed_deg_s,
                "acceleration_deg_s2": args.acceleration_deg_s2,
                "angle_deg": args.angle_deg,
            },
            "motion_conversion": motion,
            "odrive_configuration": {
                "trajectory_velocity_limit_turns_s": axis.trap_traj.config.vel_limit,
                "trajectory_acceleration_limit_turns_s2": axis.trap_traj.config.accel_limit,
                "controller_velocity_limit_turns_s": axis.controller.config.vel_limit,
            },
        }
        write_json_atomic(metadata_path, metadata)
        with output_path.open("x", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow([
                "Timestamp ISO 8601", "Elapsed Time (s)", "Cycle", "Motion Phase",
                "ODrive Position (turns)", "AFO Angle (deg)",
                "ODrive Velocity (turns/s)", "AFO Velocity (deg/s)",
                "Commanded AFO Speed (deg/s)", "Axis Active Errors",
            ])
            for cycle in range(1, args.cycles + 1):
                for target, phase in ((zero + excursion, "moving_to_max"), (zero - excursion, "moving_to_min")):
                    axis.controller.input_pos = target
                    wait_and_log(writer, axis, target, zero, cycle, phase, parameters, config, start_time)
                completed_cycles = cycle
            axis.controller.input_pos = zero
            return_parameters = {"distance_deg": args.angle_deg, "speed_deg_s": args.speed_deg_s}
            wait_and_log(writer, axis, zero, zero, args.cycles, "returning_to_zero", return_parameters, config, start_time)
        status = "completed"
        print(f"Saved {output_path}")
    except Exception as exc:
        error = str(exc)
        raise
    finally:
        if device is not None:
            try:
                getattr(device, f"axis{hardware['odrive_axis']}").requested_state = AXIS_STATE_IDLE
            except Exception:
                pass
        if metadata is not None:
            metadata["run_status"] = status
            metadata["completed_at"] = datetime.now().astimezone().isoformat(timespec="milliseconds")
            metadata["completed_cycles"] = completed_cycles
            metadata["error"] = error
            write_json_atomic(metadata_path, metadata)


if __name__ == "__main__":
    main()
