"""Guarded Phidget/load-cell diagnostic logger for the EAST tester."""

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

from Phidget22.Devices.VoltageRatioInput import VoltageRatioInput

from east_core import calculate_load, load_tester_config, sanitise_identifier, write_json_atomic


def main():
    parser = argparse.ArgumentParser(description="Log raw and calibrated EAST load-cell readings")
    parser.add_argument("--duration-s", type=float, default=30.0)
    parser.add_argument("--calibration-id", required=True)
    parser.add_argument("--prefix", default="load_cell_check")
    parser.add_argument("--confirm-hardware", action="store_true")
    args = parser.parse_args()
    if not args.confirm_hardware:
        raise SystemExit("Refusing to open hardware without --confirm-hardware")
    if args.duration_s <= 0:
        raise SystemExit("Duration must be positive")

    config = load_tester_config(PROJECT_DIR / "tester_config.json")
    hardware = config["hardware"]
    output_dir = PROJECT_DIR / config["logging"]["output_directory"]
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S_%f%z")
    output_path = output_dir / f"{sanitise_identifier(args.prefix)}_{stamp}.csv"
    metadata_path = output_path.with_suffix(".json")

    channel = VoltageRatioInput()
    metadata = None
    started = time.monotonic()
    sample_count = 0
    status = "error"
    error = None
    try:
        if hardware["phidget_serial_number"] is not None:
            channel.setDeviceSerialNumber(hardware["phidget_serial_number"])
        channel.setChannel(hardware["phidget_channel"])
        channel.openWaitForAttachment(hardware["phidget_attachment_timeout_ms"])
        channel.setDataInterval(config["acquisition"]["sample_interval_ms"])

        tare_samples = []
        for _ in range(config["load_cell"]["tare_samples"]):
            tare_samples.append(channel.getVoltageRatio())
            time.sleep(channel.getDataInterval() / 1000.0)
        tare_offset = sum(tare_samples) / len(tare_samples)

        metadata = {
            "schema_version": 1,
            "source": "Testing Scripts/phidget-force.py",
            "run_status": "started",
            "started_at": datetime.now().astimezone().isoformat(timespec="milliseconds"),
            "calibration_id": args.calibration_id,
            "tare_offset_v_per_v": tare_offset,
            "load_cell_configuration": config["load_cell"],
            "phidget_channel": hardware["phidget_channel"],
            "configured_phidget_serial_number": hardware["phidget_serial_number"],
            "connected_phidget_serial_number": channel.getDeviceSerialNumber(),
            "sample_interval_ms": config["acquisition"]["sample_interval_ms"],
            "csv_file": output_path.name,
        }
        write_json_atomic(metadata_path, metadata)

        start = time.monotonic()
        with output_path.open("x", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow([
                "Timestamp ISO 8601", "Elapsed Time (s)", "Calibration ID",
                "Raw Voltage Ratio (V/V)", "Tare Offset (V/V)",
                "Mass (kg)", "Weight (g)", "Force (N)",
            ])
            while time.monotonic() - start < args.duration_s:
                ratio = channel.getVoltageRatio()
                mass_kg, weight_g, force_n = calculate_load(ratio, tare_offset, config)
                writer.writerow([
                    datetime.now().astimezone().isoformat(timespec="milliseconds"),
                    f"{time.monotonic() - start:.6f}", args.calibration_id,
                    ratio, tare_offset, mass_kg, weight_g, force_n,
                ])
                sample_count += 1
                time.sleep(config["acquisition"]["sample_interval_ms"] / 1000.0)
        status = "completed"
        print(f"Saved {output_path}")
    except Exception as exc:
        error = str(exc)
        raise
    finally:
        try:
            channel.close()
        except Exception:
            pass
        finally:
            if metadata is not None:
                duration = time.monotonic() - started
                metadata["run_status"] = status
                metadata["completed_at"] = datetime.now().astimezone().isoformat(timespec="milliseconds")
                metadata["duration_s"] = duration
                metadata["sample_count"] = sample_count
                metadata["achieved_average_sample_rate_hz"] = sample_count / duration if duration else 0.0
                metadata["error"] = error
                write_json_atomic(metadata_path, metadata)


if __name__ == "__main__":
    main()
