"""Read-only, headless timing probe for EAST motor and optional Phidget input."""

from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from east_core import load_tester_config
from east_odrive import ODriveAdapter


def summary(values):
    ordered = sorted(values)
    if not ordered:
        return {"count": 0, "median": None, "p95": None, "p99": None, "maximum": None}

    def percentile(p):
        index = (len(ordered) - 1) * p
        lower = int(index)
        fraction = index - lower
        return ordered[lower] + (ordered[min(lower + 1, len(ordered) - 1)] - ordered[lower]) * fraction

    return {"count": len(ordered), "median": statistics.median(ordered),
            "p95": percentile(0.95), "p99": percentile(0.99), "maximum": ordered[-1]}


class MockMapper:
    pos_rel = 0.0
    vel = 0.0


class MockAxis:
    pos_vel_mapper = MockMapper()
    active_errors = 0
    current_state = 1
    disarm_reason = 0


class MockDevice:
    axis0 = MockAxis()

    class system_stats:
        uptime = 1.0


class MockSensor:
    def getVoltageRatio(self):
        return 0.0

    def close(self):
        pass


def open_real(config, include_sensor):
    import odrive

    hardware = config["hardware"]
    device = odrive.find_any(serial_number=hardware["odrive_serial_number"],
                            timeout=hardware["odrive_connection_timeout_s"])
    if device is None:
        raise RuntimeError("ODrive not found")
    sensor = None
    if include_sensor:
        from Phidget22.Devices.VoltageRatioInput import VoltageRatioInput
        sensor = VoltageRatioInput()
        try:
            if hardware["phidget_serial_number"] is not None:
                sensor.setDeviceSerialNumber(hardware["phidget_serial_number"])
            sensor.setChannel(hardware["phidget_channel"])
            sensor.openWaitForAttachment(hardware["phidget_attachment_timeout_ms"])
        except Exception:
            sensor.close()
            raise
    return device, sensor


def run(config, duration_s, interval_s, real_hardware=False, include_sensor=False,
        max_capture_s=None, sensor_recording=None, rotary_recording=None):
    if sensor_recording and not include_sensor:
        raise ValueError("--sensor-recording requires --with-sensor")
    sensor = recorder = rotary = rotary_stop = rotary_thread = hardware_lock = None
    rotary_statistics = None
    recording_errors = []
    durations = defaultdict(list)
    errors = []
    positions = []
    try:
        if real_hardware:
            from east_reference import ProcessLock, runtime_state_directory
            hardware_lock = ProcessLock(runtime_state_directory() / "hardware.lock")
            hardware_lock.acquire()
        device, sensor = open_real(config, include_sensor) if real_hardware else (
            MockDevice(), MockSensor() if include_sensor else None)
        adapter = ODriveAdapter(device, config["hardware"]["odrive_axis"] if real_hardware else 0,
                                idle_state=1, closed_loop_state=8)
        from east_runtime import CSVRecorder
        if sensor_recording:
            if not include_sensor:
                raise ValueError("--sensor-recording requires --with-sensor")
            with Path(sensor_recording).open("x", newline="", encoding="utf-8") as handle:
                import csv
                csv.writer(handle).writerow(["host_monotonic_s", "voltage_ratio", "mode"])
            recorder = CSVRecorder(sensor_recording, 100, lambda exc: recording_errors.append(str(exc)))
            recorder.start()
        if rotary_recording:
            from east_rotary_sensor import RotarySensorService, RotaryReader, RotaryRecording
            import threading
            rotary = RotarySensorService(config["rotary_sensor"], Path(__file__).resolve().parents[1])
            if real_hardware:
                rotary.connect()
                deadline = time.monotonic() + 8
                while rotary.reader.latest is None and not rotary.reader.error and time.monotonic() < deadline:
                    time.sleep(.01)
                if rotary.reader.latest is None:
                    raise RuntimeError(rotary.reader.error or "Rotary channel delivered no samples")
            else:
                reader = RotaryReader(config["rotary_sensor"]["hub_serial"], "Voltage ratio")
                reader.accepting = True
                reader.metadata = {"input_mode":"Voltage ratio", "interval_ms":8, "hub_serial":750256}
                rotary.reader = reader
                rotary_stop = threading.Event()
                def emit():
                    while not rotary_stop.is_set():
                        reader._sample(None, .39772982090527514)
                        rotary_stop.wait(.008)
                rotary_thread = threading.Thread(target=emit, name="diagnostic-simulated-rotary", daemon=True)
                rotary_thread.start()
            # Diagnostic recording preserves low-rate/out-of-range raw events.
            # Normal test startup still uses the original strict measurement gate.
            rotary.recording = RotaryRecording(rotary.reader, rotary.calibration, rotary_recording, time.monotonic())
        next_read = time.monotonic()
        deadline = next_read + duration_s
        while time.monotonic() < deadline:
            try:
                snapshot = adapter.snapshot(max_capture_duration_s=max_capture_s)
                positions.append(snapshot.captured_monotonic_s)
                durations["lock_wait_s"].append(snapshot.lock_wait_s)
                durations["snapshot_s"].append(snapshot.capture_duration_s)
                for name, (started, finished) in snapshot.property_brackets_monotonic_s.items():
                    durations[f"{name}_s"].append(finished - started)
                if sensor is not None:
                    started = time.monotonic()
                    ratio = sensor.getVoltageRatio()
                    if recorder:
                        recorder.append((time.monotonic(), ratio, "REAL READ-ONLY" if real_hardware else "SIMULATED"))
                    durations["sensor_read_s"].append(time.monotonic() - started)
            except Exception as exc:
                rejected = getattr(exc, "snapshot", None)
                if rejected is not None:
                    durations["lock_wait_s"].append(rejected.lock_wait_s)
                    durations["snapshot_s"].append(rejected.capture_duration_s)
                    for name, (started, finished) in rejected.property_brackets_monotonic_s.items():
                        durations[f"{name}_s"].append(finished-started)
                errors.append({"at_monotonic_s":time.monotonic(),"error":str(exc),
                               "rejected_snapshot":rejected.to_dict() if rejected is not None else None})
            next_read = max(next_read + interval_s, time.monotonic())
            time.sleep(max(0.0, min(next_read - time.monotonic(), deadline - time.monotonic())))
    except Exception as exc:
        errors.append({"phase":"setup/acquisition", "at_monotonic_s":time.monotonic(), "error":str(exc)})
    finally:
        if recorder:
            try: recorder.finish()
            except Exception as exc: recording_errors.append(str(exc))
        if rotary_stop:
            rotary_stop.set()
            rotary_thread.join(1)
        if rotary:
            try: rotary_statistics = rotary.finish_recording()
            except Exception as exc:
                rotary_statistics = rotary.last_recording_statistics
                recording_errors.append(str(exc))
            try:
                if real_hardware: rotary.shutdown()
            except Exception as exc: recording_errors.append(str(exc))
        if sensor is not None:
            sensor.close()
        if hardware_lock is not None:
            hardware_lock.release()
    gaps = [later - earlier for earlier, later in zip(positions, positions[1:])]
    return {"mode": "REAL READ-ONLY" if real_hardware else "SIMULATED / NO HARDWARE",
            "rotary_recording_statistics": rotary_statistics, "sensor_enabled": include_sensor, "sensor_recording_errors": recording_errors,
            "capture_evaluation_limit_s": max_capture_s, "production_limits_changed": False, "requested_interval_s": interval_s,
            "sample_count": len(positions), "error_count": len(errors), "errors": errors,
            "actual_rate_hz": ((len(positions) - 1) / (positions[-1] - positions[0])
                               if len(positions) > 1 and positions[-1] > positions[0] else 0.0),
            "gap_s": summary(gaps),
            "latency_s": {name: summary(values) for name, values in sorted(durations.items())}}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--real-hardware", action="store_true",
                        help="Explicitly connect for read-only diagnostics")
    parser.add_argument("--with-sensor", action="store_true")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--interval-ms", type=float, default=8.0)
    parser.add_argument("--capture-limit-ms", type=float, default=None,
                        help="Diagnostic evaluation only; does not modify production config")
    parser.add_argument("--sensor-recording", type=Path,
                        help="Bounded asynchronous Bridge CSV recording (original host timestamps)")
    parser.add_argument("--rotary-recording", type=Path,
                        help="Independent raw P3022 event CSV, using accepted profile; simulate unless --real-hardware")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if args.duration <= 0 or args.interval_ms <= 0 or (args.capture_limit_ms is not None and args.capture_limit_ms <= 0):
        parser.error("duration, interval and evaluation limit must be positive")
    if args.duration <= 0 or args.interval_ms <= 0:
        parser.error("duration and interval must be positive")
    result = run(load_tester_config(), args.duration, args.interval_ms / 1000.0,
                 args.real_hardware, args.with_sensor,
                 args.capture_limit_ms / 1000 if args.capture_limit_ms else None, args.sensor_recording, args.rotary_recording)
    text = json.dumps(result, indent=2)
    if args.output:
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
