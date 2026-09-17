"""Read-only EAST reference and ODrive capability inspection.

The --mock path deliberately imports no ODrive or Phidget modules, which makes
it suitable for CI and development-machine checks.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from east_core import load_tester_config
from east_odrive import ODriveAdapter
from east_reference import AtomicStateStore, ProcessLock, runtime_state_directory


class MockODrive:
    def __init__(self):
        controller_config = SimpleNamespace(
            absolute_setpoints=False,
            circular_setpoints=False,
            circular_setpoint_range=1.0,
            input_mode=5,
            control_mode=3,
            vel_limit=12.0,
        )
        self.axis0 = SimpleNamespace(
            active_errors=0,
            current_state=1,
            requested_state=1,
            disarm_reason=0,
            controller=SimpleNamespace(input_pos=0.0, config=controller_config),
            trap_traj=SimpleNamespace(
                config=SimpleNamespace(vel_limit=9.73, accel_limit=24.33, decel_limit=24.33)
            ),
            pos_vel_mapper=SimpleNamespace(
                pos_rel=0.0,
                vel=0.0,
                pos_abs=None,
                status=0,
                active_errors=0,
                config=SimpleNamespace(scale=1.0, offset_valid=False),
            ),
            commutation_mapper=SimpleNamespace(
                config=SimpleNamespace(scale=1.0, offset_valid=False)
            ),
            config=SimpleNamespace(
                load_encoder=0,
                commutation_encoder=0,
                motor=SimpleNamespace(motor_type=0),
                watchdog_timeout=0.0,
                enable_watchdog=False,
            ),
            watchdog_feed=lambda: None,
        )
        self.serial_number = "MOCK-EAST"
        self.fw_version_major = 0
        self.fw_version_minor = 6
        self.fw_version_revision = 10
        self.system_stats = SimpleNamespace(uptime=123456)
        self.rs485_encoder_group0 = SimpleNamespace(
            raw=0.25,
            status=0,
            active_errors=0,
            config=SimpleNamespace(mode="mock-rs485-mode"),
        )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Inspect EAST neutral-reference state and ODrive capabilities without writing hardware."
    )
    parser.add_argument(
        "--mock",
        action="store_true",
        help="Use an in-memory mock; does not import ODrive or Phidget drivers.",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=PROJECT_DIR / "tester_config.json",
        help="Tester configuration path.",
    )
    parser.add_argument(
        "--state-dir",
        type=Path,
        default=None,
        help="Override the runtime state directory for this inspection.",
    )
    return parser.parse_args()


def read_persisted_state(store):
    result = {"directory": str(store.directory), "reference": None, "checkpoint": None}
    try:
        reference = store.load_reference()
        result["reference"] = reference.to_dict() if reference else None
    except Exception as exc:
        result["reference_error"] = str(exc)
    try:
        result["checkpoint"] = store.load_checkpoint()
    except Exception as exc:
        result["checkpoint_error"] = str(exc)
    return result


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {str(key): json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def main():
    args = parse_args()
    config = load_tester_config(args.config)
    store = AtomicStateStore(args.state_dir or runtime_state_directory())
    lock = ProcessLock(store.lock_path)
    lock.acquire()
    try:
        if args.mock:
            device = MockODrive()
            mode = "mock"
        else:
            # Hardware dependency remains entirely outside module import and --mock paths.
            import odrive

            device = odrive.find_any(
                serial_number=config["hardware"]["odrive_serial_number"],
                timeout=config["hardware"]["odrive_connection_timeout_s"],
            )
            if device is None:
                raise RuntimeError("Configured ODrive was not found")
            mode = "live-read-only"
        adapter = ODriveAdapter(
            device,
            config["hardware"]["odrive_axis"],
            idle_state=1,
            closed_loop_state=8,
        )
        try:
            motion_compatibility = {
                "accepted": True,
                "details": adapter.validate_motion_capabilities(
                    config["reference"]["required_pos_vel_mapper_scale"],
                    config["reference"]["pos_vel_mapper_scale_tolerance"],
                ),
            }
        except Exception as exc:
            motion_compatibility = {"accepted": False, "error": str(exc)}
        report = {
            "mode": mode,
            "hardware_writes_performed": False,
            "warning": (
                "Read-only code path: no errors cleared, state requested, setpoint written, "
                "or configuration saved. Run with the GUI and other hardware clients closed."
            ),
            "runtime_state": read_persisted_state(store),
            "odrive": adapter.read_only_report(include_phase=True),
            "motion_compatibility": motion_compatibility,
            "reference_features": config["reference"],
        }
        print(
            json.dumps(
                json_safe(report),
                indent=2,
                sort_keys=True,
                allow_nan=False,
                default=str,
            )
        )
    finally:
        lock.release()


if __name__ == "__main__":
    main()
