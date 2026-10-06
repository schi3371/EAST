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

        print("\n{}P3022 QUICK TEST | no motor commands | nominal angle only".format(
            "SIMULATED DATA / " if args.demo else ""
        ))
        print("No automatic zeroing. Ctrl+C stops. Interval: {} ms.".format(effective_interval))
        if args.reference_voltage is not None:
            print("Relative angle uses your supplied reference {:.6f} V; NOT EAST machine zero.".format(
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
            ])
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
                angle = nominal_angle(voltage)
                relative = relative_angle(angle, args.reference_voltage, args.direction)
                writer.writerow([
                    timestamp, "{:.6f}".format(elapsed), serial, "{:.8f}".format(voltage),
                    "" if angle is None else "{:.5f}".format(angle),
                    "" if args.reference_voltage is None else args.reference_voltage,
                    "" if relative is None else "{:.5f}".format(relative),
                    angle is not None, "simulation" if args.demo else "HUB0007_VoltageInput",
                ])
                count += 1
                if angle is not None:
                    values.append(voltage)
                if elapsed >= next_print:
                    text = "{:7.2f} s | {:7.4f} V | {}".format(
                        elapsed, voltage, "estimated shaft angle {:8.3f} deg".format(angle)
                        if angle is not None else "OUTSIDE nominal 0-5 V range",
                    )
                    if relative is not None:
                        text += " | relative {:+8.3f} deg".format(relative)
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
            print("Observed voltage range: {:.6f} to {:.6f} V.".format(min(values), max(values)))
            print("Changing voltage supports communication; it does not establish angle accuracy.")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--list", action="store_true", help="Discover Phidgets without opening an input")
    mode.add_argument("--demo", action="store_true", help="Simulated readings; no hardware opened")
    parser.add_argument("--serial", type=int, help="Serial number of the HUB0007 to use")
    parser.add_argument("--duration", type=float, default=60.0, help="Seconds; 0 runs until Ctrl+C")
    parser.add_argument("--interval-ms", type=int, default=100, help="Requested measurement interval")
    parser.add_argument("--reference-voltage", type=float, help="Previously measured reference voltage; no auto-zero")
    parser.add_argument("--direction", type=int, choices=(-1, 1), default=1)
    parser.add_argument("--output-dir", type=Path, help="CSV folder; default: results beside this script")
    args = parser.parse_args()
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
