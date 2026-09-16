"""Command-line EAST strain-test tool.

This script is retained for bench diagnostics. The GUI is the authoritative test
application. Hardware will not move unless --confirm-hardware is supplied.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_DIR = SCRIPT_DIR.parent
sys.path.insert(0, str(PROJECT_DIR))

import odrive
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE
from odrive.enums import CONTROL_MODE_POSITION_CONTROL, INPUT_MODE_TRAP_TRAJ
from Phidget22.Devices.VoltageRatioInput import VoltageRatioInput

from east_core import (
    CSV_COLUMNS,
    TestParameters,
    afo_acceleration_to_odrive_turns_s2,
    afo_degrees_to_odrive_turns,
    afo_speed_to_odrive_turns_s,
    calculate_load,
    calculate_torque_nm,
    constant_speed_span_deg,
    create_run_paths,
    load_tester_config,
    make_run_metadata,
    motion_timeout_seconds,
    odrive_turns_to_afo_degrees,
    write_json_atomic,
)
from east_odrive import ODriveAdapter
from east_reference import (
    AtomicStateStore,
    MotionState,
    ProcessLock,
    ReferenceManager,
    runtime_state_directory,
)


def parse_args():
    parser = argparse.ArgumentParser(description="Run a guarded EAST command-line strain test")
    parser.add_argument("--cycles", type=int, required=True)
    parser.add_argument("--speed-deg-s", type=float, required=True)
    parser.add_argument("--acceleration-deg-s2", type=float, required=True)
    parser.add_argument("--min-angle-deg", type=float, required=True)
    parser.add_argument("--max-angle-deg", type=float, required=True)
    parser.add_argument("--prefix", required=True)
    parser.add_argument("--operator", required=True)
    parser.add_argument("--afo-id", required=True)
    parser.add_argument("--fixture-id", required=True)
    parser.add_argument("--calibration-id", required=True)
    parser.add_argument(
        "--confirm-hardware", action="store_true",
        help="Required acknowledgement that the fixture is clear and E-stop is accessible",
    )
    parser.add_argument(
        "--allow-unreferenced-diagnostic",
        action="store_true",
        help=(
            "Explicitly allow current-position diagnostic zero. This invalidates normal-test "
            "reference trust and requires physical recovery in the GUI afterwards."
        ),
    )
    return parser.parse_args()


def wait_for_position(axis, target, distance_deg, speed_deg_s, acceleration_deg_s2, config):
    tolerance = afo_degrees_to_odrive_turns(config["motion"]["position_tolerance_deg"], config)
    timeout = motion_timeout_seconds(distance_deg, speed_deg_s, acceleration_deg_s2, config)
    deadline = time.monotonic() + timeout
    target = float(target)
    if not math.isfinite(target):
        raise RuntimeError("Target position is not finite")
    settled_since = None
    velocity_limit = afo_speed_to_odrive_turns_s(
        config["reference"]["settle_velocity_limit_deg_s"], config
    )
    dwell_s = config["reference"]["settle_dwell_ms"] / 1000.0
    while True:
        position = float(axis.pos_vel_mapper.pos_rel)
        velocity = float(axis.pos_vel_mapper.vel)
        if not math.isfinite(position) or not math.isfinite(velocity):
            raise RuntimeError("ODrive position/velocity feedback is not finite")
        if int(axis.active_errors):
            raise RuntimeError(f"ODrive active errors: {int(axis.active_errors)}")
        if time.monotonic() >= deadline:
            raise TimeoutError(f"Motion timeout after {timeout:.1f} s")
        if abs(position - target) <= tolerance and abs(velocity) <= velocity_limit:
            settled_since = settled_since or time.monotonic()
            if time.monotonic() - settled_since >= dwell_s:
                return
        else:
            settled_since = None
        yield


def main():
    args = parse_args()
    if not args.confirm_hardware:
        raise SystemExit("Refusing to move hardware without --confirm-hardware")
    if not args.allow_unreferenced_diagnostic:
        raise SystemExit(
            "This standalone diagnostic cannot prove the GUI neutral mapping. Re-run only with "
            "--allow-unreferenced-diagnostic, then physically recover neutral in the GUI."
        )

    config = load_tester_config(PROJECT_DIR / "tester_config.json")
    state_store = AtomicStateStore(runtime_state_directory())
    process_lock = ProcessLock(state_store.lock_path)
    process_lock.acquire()
    reference_manager = ReferenceManager(config, state_store)
    reference_manager.invalidate(
        "Standalone unreferenced strain diagnostic was authorized; GUI recovery is required"
    )
    motion = config["motion"]
    if not 1 <= args.cycles <= motion["maximum_cycles"]:
        raise SystemExit("Cycles are outside configured limits")
    if not motion["minimum_speed_deg_s"] <= args.speed_deg_s <= motion["maximum_speed_deg_s"]:
        raise SystemExit("Speed is outside configured limits")
    if not motion["minimum_acceleration_deg_s2"] <= args.acceleration_deg_s2 <= motion["maximum_acceleration_deg_s2"]:
        raise SystemExit("Acceleration is outside configured limits")
    if not 0 <= args.min_angle_deg <= motion["maximum_afo_angle_deg"]:
        raise SystemExit("Minimum angle magnitude is outside configured limits")
    if not 0 <= args.max_angle_deg <= motion["maximum_afo_angle_deg"]:
        raise SystemExit("Maximum angle magnitude is outside configured limits")
    if args.min_angle_deg == 0 and args.max_angle_deg == 0:
        raise SystemExit("At least one angle limit must be greater than zero")
    expected_full_span = constant_speed_span_deg(
        args.min_angle_deg + args.max_angle_deg,
        args.speed_deg_s,
        args.acceleration_deg_s2,
    )
    if expected_full_span < motion["minimum_constant_speed_span_deg"]:
        raise SystemExit(
            "The selected commanded speed, acceleration and angle range do not provide "
            f"the required {motion['minimum_constant_speed_span_deg']:g} deg constant-speed span"
        )

    parameters = TestParameters(
        file_prefix=args.prefix,
        operator=args.operator,
        afo_id=args.afo_id,
        fixture_id=args.fixture_id,
        calibration_id=args.calibration_id,
        cycles=args.cycles,
        commanded_afo_speed_deg_s=args.speed_deg_s,
        commanded_afo_acceleration_deg_s2=args.acceleration_deg_s2,
        min_angle_deg=args.min_angle_deg,
        max_angle_deg=args.max_angle_deg,
    )

    device = None
    adapter = None
    phidget = None
    metadata = None
    metadata_path = None
    status = "error"
    error = None
    sample_index = 0
    completed_cycles = 0
    try:
        hardware = config["hardware"]
        device = odrive.find_any(
            serial_number=hardware["odrive_serial_number"],
            timeout=hardware["odrive_connection_timeout_s"],
        )
        if device is None:
            raise RuntimeError("ODrive not found")
        axis = getattr(device, f"axis{hardware['odrive_axis']}")
        adapter = ODriveAdapter(
            device, hardware["odrive_axis"], AXIS_STATE_IDLE, AXIS_STATE_CLOSED_LOOP_CONTROL
        )
        reference_manager.write_checkpoint(
            MotionState.ARMING,
            adapter.snapshot(),
            clean_shutdown=False,
            extra={"source": "afo-strain-test-script.py", "unreferenced": True},
        )
        if int(axis.active_errors):
            raise RuntimeError(f"ODrive has active errors: {int(axis.active_errors)}")

        velocity_turns_s = afo_speed_to_odrive_turns_s(args.speed_deg_s, config)
        acceleration_turns_s2 = afo_acceleration_to_odrive_turns_s2(
            args.acceleration_deg_s2, config
        )
        axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        axis.trap_traj.config.vel_limit = velocity_turns_s
        axis.trap_traj.config.accel_limit = acceleration_turns_s2
        axis.trap_traj.config.decel_limit = acceleration_turns_s2
        axis.controller.config.vel_limit = velocity_turns_s * motion["controller_velocity_safety_multiplier"]

        phidget = VoltageRatioInput()
        if hardware["phidget_serial_number"] is not None:
            phidget.setDeviceSerialNumber(hardware["phidget_serial_number"])
        phidget.setChannel(hardware["phidget_channel"])
        phidget.openWaitForAttachment(hardware["phidget_attachment_timeout_ms"])
        phidget.setDataInterval(config["acquisition"]["sample_interval_ms"])
        tare_values = []
        for _ in range(config["load_cell"]["tare_samples"]):
            tare_values.append(phidget.getVoltageRatio())
            time.sleep(phidget.getDataInterval() / 1000.0)
        tare_offset = sum(tare_values) / len(tare_values)

        csv_path, metadata_path = create_run_paths(parameters, config, PROJECT_DIR)
        odrive_snapshot = {
            "axis": hardware["odrive_axis"],
            "trajectory_velocity_limit_turns_s": axis.trap_traj.config.vel_limit,
            "trajectory_acceleration_limit_turns_s2": axis.trap_traj.config.accel_limit,
            "controller_velocity_limit_turns_s": axis.controller.config.vel_limit,
        }
        metadata = make_run_metadata(parameters, config, tare_offset, csv_path, odrive_snapshot)
        metadata["source"] = "Testing Scripts/afo-strain-test-script.py"
        metadata["neutral_reference"] = {
            "status": "UNREFERENCED DIAGNOSTIC - NOT VALID FOR FORMAL TESTING",
            "reference_state": reference_manager.metadata_snapshot(),
        }
        try:
            metadata["hardware"]["connected_phidget_serial_number"] = phidget.getDeviceSerialNumber()
        except Exception:
            metadata["hardware"]["connected_phidget_serial_number"] = "unavailable"
        write_json_atomic(metadata_path, metadata)

        window = config["acquisition"]["moving_average_window_samples"]
        angle_window, weight_window, torque_window = deque(maxlen=window), deque(maxlen=window), deque(maxlen=window)
        start_time = time.monotonic()
        zero_turns = axis.pos_vel_mapper.pos_rel
        minimum_turns = zero_turns - afo_degrees_to_odrive_turns(args.min_angle_deg, config)
        maximum_turns = zero_turns + afo_degrees_to_odrive_turns(args.max_angle_deg, config)

        with csv_path.open("x", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(CSV_COLUMNS)
            adapter.enter_closed_loop_holding_current()
            for cycle in range(1, args.cycles + 1):
                for target, phase in (
                    (maximum_turns, "moving_to_max"),
                    (minimum_turns, "moving_to_min"),
                ):
                    distance = abs(odrive_turns_to_afo_degrees(
                        target - axis.pos_vel_mapper.pos_rel, config
                    ))
                    expected_constant_span = constant_speed_span_deg(
                        distance, args.speed_deg_s, args.acceleration_deg_s2
                    )
                    axis.controller.input_pos = target
                    for _ in wait_for_position(
                        axis, target, distance, args.speed_deg_s,
                        args.acceleration_deg_s2, config,
                    ):
                        ratio = phidget.getVoltageRatio()
                        position_turns = axis.pos_vel_mapper.pos_rel
                        angle_deg = odrive_turns_to_afo_degrees(position_turns - zero_turns, config)
                        raw_velocity = axis.pos_vel_mapper.vel
                        mass_kg, weight_g, force_n = calculate_load(ratio, tare_offset, config)
                        torque_nm = calculate_torque_nm(force_n, angle_deg, config)
                        angle_window.append(angle_deg)
                        weight_window.append(weight_g)
                        torque_window.append(torque_nm)
                        writer.writerow([
                            datetime.now().astimezone().isoformat(timespec="milliseconds"),
                            f"{time.monotonic() - start_time:.6f}", sample_index, cycle, phase,
                            args.speed_deg_s, args.acceleration_deg_s2,
                            -args.min_angle_deg, args.max_angle_deg, args.cycles,
                            args.prefix, args.operator, args.afo_id,
                            args.fixture_id, args.calibration_id,
                            velocity_turns_s,
                            distance, expected_constant_span, position_turns, angle_deg,
                            sum(angle_window) / len(angle_window), ratio, tare_offset, mass_kg,
                            weight_g, sum(weight_window) / len(weight_window), force_n, torque_nm,
                            sum(torque_window) / len(torque_window), raw_velocity,
                            odrive_turns_to_afo_degrees(raw_velocity, config), int(axis.active_errors),
                        ])
                        sample_index += 1
                        time.sleep(config["acquisition"]["sample_interval_ms"] / 1000.0)
                completed_cycles = cycle

            axis.controller.input_pos = zero_turns
            return_distance = abs(odrive_turns_to_afo_degrees(
                zero_turns - axis.pos_vel_mapper.pos_rel, config
            ))
            for _ in wait_for_position(
                axis, zero_turns, return_distance, args.speed_deg_s,
                args.acceleration_deg_s2, config,
            ):
                time.sleep(0.01)
        status = "completed"
    except KeyboardInterrupt:
        status = "aborted"
        error = "operator keyboard interrupt"
    except Exception as exc:
        error = str(exc)
        raise
    finally:
        if adapter is not None:
            try:
                idle_result = adapter.request_idle(
                    config["reference"]["idle_confirmation_timeout_s"]
                )
                snapshot = adapter.snapshot()
                reference_manager.write_checkpoint(
                    MotionState.IDLE if idle_result.confirmed else MotionState.FAULT,
                    snapshot,
                    clean_shutdown=False,
                    extra={"source": "afo-strain-test-script.py", "idle": idle_result.__dict__},
                )
            except Exception:
                pass
        if phidget is not None:
            try:
                phidget.close()
            except Exception:
                pass
        if metadata is not None and metadata_path is not None:
            metadata["run_status"] = status
            metadata["completed_at"] = datetime.now().astimezone().isoformat(timespec="milliseconds")
            metadata["completed_cycles"] = completed_cycles
            metadata["sample_count"] = sample_index
            metadata["error"] = error
            write_json_atomic(metadata_path, metadata)
        process_lock.release()


if __name__ == "__main__":
    main()
