#!/usr/bin/env python3
"""Standalone P3022/HUB0007 communication check. Never controls EAST or ODrive.

Run without arguments for a 60-second read. --list only discovers devices;
--demo exercises the display/logging without opening any hardware.
The 0-5 V -> 0-360 degree conversion is nominal, NOT an EAST calibration.
"""

import argparse
import csv
import math
import queue
import sys
import threading
import time
from datetime import datetime
from pathlib import Path


def nominal_angle(voltage):
    if not math.isfinite(voltage) or not 0.0 <= voltage <= 5.0:
        return None
    return voltage * 72.0


ELECTRICAL_MAX_V = 5.3
ESTIMATE_FIELDS = [
    "estimated_shaft_angle_deg", "angle_estimate_full_scale_voltage_v",
    "angle_estimate_basis", "electrical_range_valid", "angle_estimate_status",
]


def estimate_fields(voltage, full_scale_voltage):
    """Provisional linear estimate, never clipped/wrapped or treated as calibration.

    Full scale is an explicit assumption, not a simultaneous VCC measurement.
    Preserve voltages outside the interface range but do not interpret as angles.
    """
    if not math.isfinite(full_scale_voltage) or not 0 < full_scale_voltage <= ELECTRICAL_MAX_V:
        raise ValueError("Assumed full-scale voltage must be finite and in (0, 5.3] V")
    finite = math.isfinite(voltage)
    electrical_valid = finite and 0 <= voltage <= ELECTRICAL_MAX_V
    if not finite:
        status = "non_finite_voltage"
    elif not electrical_valid:
        status = "outside_hub_electrical_range"
    elif voltage > full_scale_voltage:
        status = "uncalibrated_above_assumed_full_scale"
    else:
        status = "uncalibrated_assumed_linear_scale"
    return {
        "estimated_shaft_angle_deg": f"{360.0 * voltage / full_scale_voltage:.5f}" if electrical_valid else "",
        "angle_estimate_full_scale_voltage_v": format(full_scale_voltage, ".12g"),
        "angle_estimate_basis": "assumed_linear_0_to_full_scale_over_360_deg_not_calibrated",
        "electrical_range_valid": electrical_valid,
        "angle_estimate_status": status,
    }


def display_sample(elapsed, voltage, estimate):
    angle = estimate["estimated_shaft_angle_deg"]
    text = f"{elapsed:7.2f} s | {voltage:7.4f} V | "
    if angle:
        text += f"estimated shaft angle {float(angle):8.3f} deg (UNCALIBRATED)"
        if estimate["angle_estimate_status"] == "uncalibrated_above_assumed_full_scale":
            text += " | ABOVE assumed full scale; linear extrapolation, not rollover-corrected"
        elif voltage > 5.0:
            text += " | above nominal 5 V; within hub electrical range"
    else:
        text += estimate["angle_estimate_status"]
    return text


def relative_angle(angle, reference_voltage, direction):
    if angle is None or reference_voltage is None:
        return None
    # Signed difference across the 0/360-degree rollover; single-turn only.
    return direction * ((angle - reference_voltage * 72.0 + 180.0) % 360.0 - 180.0)


def discover(Manager, seconds=3.0):
    found = {}
    lock = threading.Lock()
    manager = Manager()

    def attached(_manager, channel):
        info = {
            "serial": channel.getDeviceSerialNumber(),
            "sku": channel.getDeviceSKU(),
            "name": channel.getDeviceName(),
        }
        with lock:
            found[(info["serial"], info["sku"])] = info

    manager.setOnAttachHandler(attached)
    try:
        manager.open()
        time.sleep(seconds)
    finally:
        manager.close()
    with lock:
        return sorted(found.values(), key=lambda item: (item["serial"], item["sku"]))


def run(args):
    channel = None
    samples = queue.Queue()
    started = time.monotonic()
    serial = "DEMO"
    effective_interval = args.interval_ms
    output = None
    count = 0
    values = []
    completed = False
    try:
        if not args.demo:
            try:
                from Phidget22.Devices.Manager import Manager
                from Phidget22.Devices.VoltageInput import VoltageInput
            except (ImportError, OSError) as exc:
                raise RuntimeError(
                    "Phidgets support is missing from this Python environment. "
                    "Install with: python -m pip install -r requirements.txt\n"
                    "See README.md for Mac/Windows setup. Details: " + str(exc)
                ) from exc

            print("Looking for connected Phidgets (3 seconds)...", flush=True)
            devices = discover(Manager)
            for device in devices:
                print("  {sku}: serial {serial} ({name})".format(**device))
            if args.list:
                if not devices:
                    print("No Phidgets detected. Check USB connection and the Control Panel.")
                    return 1
                return 0
            hubs = [item for item in devices if item["sku"].startswith("HUB0007")]
            if args.serial is not None:
                hubs = [item for item in hubs if item["serial"] == args.serial]
            if len(hubs) != 1:
                raise RuntimeError(
                    "Expected exactly one HUB0007, found {}. "
                    "Connect it, or select it with --serial NUMBER. "
                    "This test will not open the load-cell Bridge.".format(len(hubs))
                )
            serial = hubs[0]["serial"]
            channel = VoltageInput()
            channel.setDeviceSerialNumber(serial)
            channel.setHubPort(0)
            channel.setIsHubPortDevice(True)
            channel.setChannel(0)
            channel.setOnDetachHandler(
                lambda _ch: samples.put(("error", "Hub disconnected; readings stopped."))
            )
            channel.setOnErrorHandler(
                lambda _ch, code, description: samples.put(
                    ("error", "Phidget error {}: {}".format(code, description))
                )
            )
            print("Opening HUB0007 port 0 as VoltageInput...", flush=True)
            channel.openWaitForAttachment(5000)
            effective_interval = max(
                channel.getMinDataInterval(),
                min(args.interval_ms, channel.getMaxDataInterval()),
            )
            channel.setDataInterval(effective_interval)
            channel.setVoltageChangeTrigger(0.0)
            channel.setOnVoltageChangeHandler(
                lambda _ch, voltage: samples.put(
                    ("sample", (time.monotonic(), datetime.now().astimezone().isoformat(
                        timespec="milliseconds"), voltage))
                )
            )

        print("\n{}P3022 QUICK TEST | no motor commands | uncalibrated estimate".format(
            "SIMULATED DATA / " if args.demo else ""
        ))
        print("No automatic zeroing. Ctrl+C stops. Interval: {} ms.".format(effective_interval))
        print(f"Angle estimate assumes 0-{args.full_scale_voltage:g} V over 360 degrees. "
              "This is NOT calibrated angle or a measured supply voltage.")
        if args.reference_voltage is not None:
            print("Legacy NOMINAL 5 V relative angle uses reference {:.6f} V; NOT EAST machine zero.".format(
                args.reference_voltage
            ))
        else:
            print("No reference supplied: displaying raw voltage and estimated shaft angle.")
        log_dir = args.output_dir or Path(__file__).resolve().parent / "results"
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S_%f%z")
        output = log_dir / "{}_{}.csv".format(
            "DEMO_p3022" if args.demo else "p3022_{}".format(serial), stamp
        )
        print("CSV: {}\n".format(output), flush=True)
        started = time.monotonic()
        next_print = 0.0
        with output.open("x", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow([
                "host_timestamp_iso", "host_elapsed_s", "hub_serial", "voltage_v",
                "nominal_shaft_angle_deg", "reference_voltage_v", "nominal_relative_angle_deg",
                "nominal_range_valid", "data_source",
            ] + ESTIMATE_FIELDS)
            while args.duration == 0 or time.monotonic() - started < args.duration:
                if args.demo:
                    elapsed = time.monotonic() - started
                    sample_time = time.monotonic()
                    timestamp = datetime.now().astimezone().isoformat(timespec="milliseconds")
                    voltage = 2.5 + 0.1 * math.sin(elapsed)
                    time.sleep(effective_interval / 1000.0)
                else:
                    try:
                        kind, payload = samples.get(timeout=max(3.0, effective_interval / 1000.0 * 3))
                    except queue.Empty as exc:
                        raise RuntimeError("No fresh voltage readings. Check connection and close other sensor windows.") from exc
                    if kind == "error":
                        raise RuntimeError(payload)
                    sample_time, timestamp, voltage = payload
                elapsed = sample_time - started
                if elapsed < 0:
                    continue
                estimate = estimate_fields(voltage, args.full_scale_voltage)
                angle = nominal_angle(voltage)
                relative = relative_angle(angle, args.reference_voltage, args.direction)
                writer.writerow([
                    timestamp, "{:.6f}".format(elapsed), serial, "{:.8f}".format(voltage),
                    "" if angle is None else "{:.5f}".format(angle),
                    "" if args.reference_voltage is None else args.reference_voltage,
                    "" if relative is None else "{:.5f}".format(relative),
                    angle is not None, "simulation" if args.demo else "HUB0007_VoltageInput",
                ] + [estimate[field] for field in ESTIMATE_FIELDS])
                count += 1
                if math.isfinite(voltage):
                    values.append(voltage)
                if elapsed >= next_print:
                    text = display_sample(elapsed, voltage, estimate)
                    if relative is not None:
                        text += " | legacy nominal relative {:+8.3f} deg".format(relative)
                    print(text, flush=True)
                    handle.flush()
                    next_print = elapsed + 0.2
        completed = True
        return 0
    except KeyboardInterrupt:
        print("\nStopped by user.")
        return 0
    except Exception as exc:
        print("\nTEST STOPPED: {}".format(exc), file=sys.stderr)
        print("Close the sensor test window in Phidget Control Panel/EAST before retrying. "
              "Use --list to check discovery.", file=sys.stderr)
        return 1
    finally:
        if channel is not None:
            try:
                channel.close()
            except Exception as exc:
                print("Could not close channel cleanly: {}".format(exc), file=sys.stderr)
        if output is not None:
            print("\n{}: {} readings saved to {}".format(
                "Finished" if completed else "Partial recording", count, output
            ))
        if values:
            print("Observed voltage range (all finite readings, including above 5 V): {:.6f} to {:.6f} V.".format(min(values), max(values)))
            print("Changing voltage supports communication; it does not establish angle accuracy.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--list", action="store_true", help="Discover Phidgets without opening an input")
    mode.add_argument("--demo", action="store_true", help="Simulated readings; no hardware opened")
    parser.add_argument("--serial", type=int, help="Serial number of the HUB0007 to use")
    parser.add_argument("--duration", type=float, default=60.0, help="Seconds; 0 runs until Ctrl+C")
    parser.add_argument("--interval-ms", type=int, default=100, help="Requested measurement interval")
    parser.add_argument("--full-scale-voltage", type=float, default=5.0,
                        help="Explicit assumed voltage at 360 deg (default 5 V); NOT calibration or measured VCC")
    parser.add_argument("--reference-voltage", type=float, help="Previously measured reference voltage; no auto-zero")
    parser.add_argument("--direction", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--output-dir", type=Path, help="CSV folder; default: results beside this script")
    args = parser.parse_args()
    if not math.isfinite(args.full_scale_voltage) or not 0 < args.full_scale_voltage <= ELECTRICAL_MAX_V:
        parser.error("--full-scale-voltage must be finite and in (0, 5.3] V")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must be finite and non-negative")
    if args.interval_ms < 1:
        parser.error("--interval-ms must be at least 1")
    if args.serial is not None and args.serial < 1:
        parser.error("--serial must be positive")
    if args.reference_voltage is not None and nominal_angle(args.reference_voltage) is None:
        parser.error("--reference-voltage must be finite and between 0 and 5 V")
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
