"""Shared event-driven rotary measurement. No motor writes or Tk imports."""
from collections import deque
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import queue
import threading
import time
import uuid


ROTARY_CSV_COLUMNS = [
    "Sensor Timestamp ISO 8601", "Sensor Elapsed Time (s)",
    "Sensor Host Monotonic Time (s)", "Sensor Sample Index", "Sensor Connection ID",
    "Rotary Voltage Ratio (V/V)", "Measured Fixture Angle (deg)",
    "Rotary Sample Valid", "Rotary Sample Status", "Rotary Calibration ID",
    "Sensor Host Monotonic Time (ns)", "Sensor Timestamp Tied With Previous Event",
]


class RotaryError(RuntimeError):
    pass


class RotaryCalibration:
    def __init__(self, path, config):
        data = Path(path).read_bytes()
        self.profile = json.loads(data)
        self.sha256 = hashlib.sha256(data).hexdigest()
        p = self.profile
        if p.get("status") != "accepted_for_measurement":
            raise RotaryError("Rotary calibration is not accepted for measurement")
        for key in ("hub_serial", "hub_port", "channel", "input_mode"):
            if p.get(key) != config.get(key):
                raise RotaryError(f"Rotary calibration {key} does not match configuration")
        if p["input_mode"] != "Voltage ratio":
            raise RotaryError("The accepted calibration requires voltage-ratio input")
        self.slope = float(p["slope_deg_per_ratio"])
        self.intercept = float(p["intercept_deg"])
        self.minimum, self.maximum = map(float, p["checked_angle_range_deg"])
        self.margin = float(config["range_margin_deg"])
        if (not all(math.isfinite(v) for v in (self.slope, self.intercept, self.minimum, self.maximum, self.margin))
                or self.slope == 0 or self.minimum >= self.maximum or self.margin < 0):
            raise RotaryError("Invalid rotary calibration coefficients or checked range")
        if not p.get("calibration_id") or not p.get("mounting_reference"):
            raise RotaryError("Rotary calibration identity/reference is missing")

    def evaluate(self, ratio):
        value = float(ratio)
        if not math.isfinite(value) or not 0 <= value <= 1:
            return None, False, "invalid_voltage_ratio"
        angle = self.slope * value + self.intercept
        if not self.minimum-self.margin <= angle <= self.maximum+self.margin:
            return angle, False, "outside_checked_angle_range"
        low, high = self.profile["fit_angle_range_deg"]
        return angle, True, ("within_fit_range" if low <= angle <= high else "user_checked_extension")

    def metadata(self):
        return {"profile": self.profile, "profile_sha256": self.sha256,
                "range_margin_deg": self.margin, "motor_control_changed": False}


class ReaderSubscription:
    """Independent event queue, sharing the existing physical channel."""
    def __init__(self, source, events):
        self.source, self.events = source, events

    def __getattr__(self, name):
        return getattr(self.source, name)

    @property
    def timed_out(self):
        return self.source.timed_out

    @timed_out.setter
    def timed_out(self, value):
        self.source.timed_out = value

    def unsubscribe(self):
        self.source.unsubscribe(self.events)


class RotaryReader:
    CONNECT_TIMEOUT_S = 8.0

    def __init__(self, serial, mode, interval_ms=8, *, channel_factory=None):
        self.serial, self.mode, self.requested_interval_ms = int(serial), mode, int(interval_ms)
        self.events = queue.Queue(maxsize=4096)
        self.stop = threading.Event()
        self.lock = threading.RLock()
        self.phase = "Starting sensor worker"
        self.started_monotonic = None
        self.timed_out = False
        self.metadata = None
        self.latest = None
        self.error = None
        self.connection_id = uuid.uuid4().hex
        self.samples = 0
        self.first_sample_time = None
        self.maximum_gap_s = 0.0
        self.timestamp_ties = 0
        self.arrivals = deque(maxlen=256)
        self.listeners = []
        self.accepting = False
        self.channel_factory = channel_factory
        self.thread = threading.Thread(target=self._run, name="rotary-sensor-reader", daemon=True)

    def start(self):
        self.started_monotonic = time.monotonic()
        self.thread.start()

    def subscribe(self):
        events = queue.Queue(maxsize=8192)
        with self.lock:
            if self.metadata and not self.stop.is_set():
                events.put_nowait(("connected", dict(self.metadata)))
            self.listeners.append(events)
        return ReaderSubscription(self, events)

    def unsubscribe(self, events):
        with self.lock:
            if events in self.listeners:
                self.listeners.remove(events)

    def _emit(self, kind, payload):
        with self.lock:
            for events in [self.events, *self.listeners]:
                try:
                    events.put_nowait((kind, payload))
                except queue.Full:
                    self.error = "Rotary event buffer overflow; samples cannot be silently dropped"
                    self.stop.set()

    def _progress(self, phase):
        self.phase = phase
        self._emit("progress", phase)

    def _failure(self, message):
        with self.lock:
            self.error = str(message)
            self.stop.set()
        self._emit("error", self.error)

    def _sample(self, _channel, value):
        # Only delivered change events count as samples, even for unchanged values.
        with self.lock:
            if not self.accepting or self.stop.is_set():
                return
            timestamp_ns = time.monotonic_ns()
            timestamp = timestamp_ns / 1_000_000_000
            tied = False
            if self.latest:
                gap_ns = timestamp_ns-self.latest[5]
                if gap_ns < 0:
                    self._failure(f"Rotary host clock moved backwards: {timestamp_ns} < {self.latest[5]} ns")
                    return
                tied = gap_ns == 0
                self.timestamp_ties += int(tied)
                gap = gap_ns / 1_000_000_000
                self.maximum_gap_s = max(self.maximum_gap_s, gap)
            self.samples += 1
            if self.first_sample_time is None:
                self.first_sample_time = timestamp
            self.arrivals.append(timestamp_ns)
            self.latest = (timestamp, datetime.now().astimezone().isoformat(timespec="microseconds"),
                           float(value), self.samples, self.connection_id, timestamp_ns, tied)
            self._emit("sample", self.latest)
            if not math.isfinite(float(value)) or not 0 <= float(value) <= (1.0 if self.mode == "Voltage ratio" else 5.3):
                self._failure("Invalid rotary reading; original event retained")

    def snapshot(self):
        with self.lock:
            span = self.arrivals[-1]-self.arrivals[0] if self.arrivals else 0
            rate = (len(self.arrivals)-1)*1_000_000_000/span if len(self.arrivals)>1 and span>0 else 0.0
            metadata = dict(self.metadata or {})
            metadata["timestamp_tie_count"] = self.timestamp_ties
            return self.latest, metadata, self.error, self.samples, rate

    def _run(self):
        channel = None
        try:
            self._progress("Loading Phidget driver")
            if self.mode not in ("Voltage ratio", "Voltage"):
                raise RotaryError("Unsupported sensor input mode")
            if self.channel_factory:
                channel = self.channel_factory()
            elif self.mode == "Voltage ratio":
                from Phidget22.Devices.VoltageRatioInput import VoltageRatioInput
                channel = VoltageRatioInput()
            elif self.mode == "Voltage":
                from Phidget22.Devices.VoltageInput import VoltageInput
                channel = VoltageInput()
            else:
                raise RotaryError("Unsupported sensor input mode")
            channel.setDeviceSerialNumber(self.serial)
            channel.setHubPort(0)
            channel.setIsHubPortDevice(True)
            channel.setChannel(0)
            channel.setOnDetachHandler(lambda *_args: self._failure("Rotary sensor detached"))
            channel.setOnErrorHandler(lambda _channel, *details: self._failure("Rotary driver error: "+str(details)))
            if self.mode == "Voltage ratio":
                channel.setOnVoltageRatioChangeHandler(self._sample)
            else:
                channel.setOnVoltageChangeHandler(self._sample)
            self._progress(f"Opening hub {self.serial}, port 0, {self.mode}")
            channel.openWaitForAttachment(4000)
            if self.stop.is_set():
                return
            self._progress("Checking attached hub and configuring sampling")
            hub = channel.getHub()
            sku = hub.getDeviceSKU()
            if not sku.startswith("HUB0007"):
                raise RotaryError(f"Expected parent HUB0007, received {sku}")
            if (hub.getDeviceSerialNumber() != self.serial
                    or channel.getDeviceSerialNumber() != self.serial
                    or channel.getHubPort() != 0 or not channel.getIsHubPortDevice()):
                raise RotaryError("Attached rotary hub/address does not match configuration")
            minimum, maximum = channel.getMinDataInterval(), channel.getMaxDataInterval()
            interval = max(minimum, min(self.requested_interval_ms, maximum))
            channel.setDataInterval(interval)
            if self.mode == "Voltage ratio":
                channel.setVoltageRatioChangeTrigger(0.0)
            else:
                channel.setVoltageChangeTrigger(0.0)
            actual_interval = channel.getDataInterval()
            with self.lock:
                self.metadata = {"hub_serial": self.serial, "hub_port": 0, "channel": 0,
                    "sku": sku, "channel_sku": channel.getDeviceSKU(), "input_mode": self.mode,
                    "requested_interval_ms": self.requested_interval_ms, "interval_ms": actual_interval,
                    "minimum_interval_ms": minimum, "maximum_interval_ms": maximum,
                    "connection_id": self.connection_id, "change_trigger": 0.0,
                    "host_clock": time.get_clock_info("monotonic").implementation,
                    "host_clock_resolution_s": time.get_clock_info("monotonic").resolution,
                    "timestamp_units": "integer host monotonic nanoseconds plus seconds for joining",
                    "timestamp_basis": "host arrival of Phidget change event; not hardware-synchronised"}
                self.accepting = True
                self._emit("connected", dict(self.metadata))
            self._progress("Waiting for delivered sensor samples")
            deadline = time.monotonic()+3.0
            while not self.stop.wait(0.02):
                if self.latest is None and time.monotonic() >= deadline:
                    raise RotaryError("No delivered rotary samples within three seconds")
        except Exception as exc:
            self._failure(f"{self.phase}: {type(exc).__name__}: {exc}")
        finally:
            self.accepting = False
            self.stop.set()
            if channel is not None:
                try:
                    channel.close()
                except Exception as exc:
                    self._failure("Rotary close failed: "+str(exc))
            self._emit("closed", None)


class RotaryRecording:
    """Write every delivered sample independently of the load-cell logging loop."""
    def __init__(self, reader, calibration, path, started):
        self.reader, self.calibration, self.started = reader, calibration, started
        self.stop = threading.Event()
        self.error = None
        self.count = self.invalid_count = 0
        self.first = self.last = None
        self.first_ns = self.last_ns = None
        self.maximum_gap_s = 0.0
        self.timestamp_ties = 0
        self.handle = Path(path).open("x", newline="", encoding="utf-8")
        self.writer = csv.writer(self.handle)
        self.writer.writerow(ROTARY_CSV_COLUMNS)
        self.subscription = reader.subscribe()
        self.thread = threading.Thread(target=self._run, name="rotary-recording", daemon=True)
        self.thread.start()

    def _run(self):
        try:
            while not self.stop.is_set() or not self.subscription.events.empty():
                try:
                    kind, sample = self.subscription.events.get(timeout=0.02)
                except queue.Empty:
                    continue
                if kind in ("error", "closed"):
                    self.error = str(sample) if kind == "error" else "Rotary channel closed during recording"
                    self.stop.set()
                    continue
                if kind != "sample" or sample[0] < self.started:
                    continue
                angle, valid, reason = self.calibration.evaluate(sample[2])
                if self.last_ns is not None:
                    self.maximum_gap_s = max(self.maximum_gap_s, (sample[5]-self.last_ns)/1_000_000_000)
                self.first = sample[0] if self.first is None else self.first
                self.last = sample[0]
                self.first_ns = sample[5] if self.first_ns is None else self.first_ns
                self.last_ns = sample[5]
                self.count += 1
                self.timestamp_ties += int(sample[6])
                self.invalid_count += int(not valid)
                self.writer.writerow([sample[1], f"{sample[0]-self.started:.9f}", f"{sample[0]:.9f}",
                    sample[3], sample[4], repr(sample[2]), "" if angle is None else f"{angle:.8f}",
                    valid, reason, self.calibration.profile["calibration_id"], sample[5], sample[6]])
                if self.count % 100 == 0:
                    self.handle.flush()
        except Exception as exc:
            self.error = "Rotary sample recording failed: "+str(exc)
            self.reader._failure(self.error)
        finally:
            try:
                self.handle.flush()
                self.handle.close()
            except Exception as exc:
                self.error = "Rotary sample flush failed: "+str(exc)

    def close(self):
        self.subscription.unsubscribe()
        self.stop.set()
        self.thread.join(timeout=2.0)
        if self.thread.is_alive():
            raise RotaryError("Rotary recording worker did not finish")
        if self.error:
            raise RotaryError(self.error)
        if self.invalid_count or not self.count:
            raise RotaryError("Rotary recording contains invalid samples or no delivered samples")
        return self.statistics()

    def statistics(self):
        return {"delivered_samples": self.count, "invalid_samples": self.invalid_count,
                "achieved_event_rate_hz": (self.count-1)*1_000_000_000/(self.last_ns-self.first_ns)
                    if self.count > 1 and self.last_ns > self.first_ns else None,
                "maximum_inter_sample_gap_s": self.maximum_gap_s,
                "tied_timestamp_events": self.timestamp_ties,
                "recording_error": self.error}


class RotarySensorService:
    def __init__(self, config, base_path, *, reader_factory=RotaryReader):
        self.config, self.reader_factory = config, reader_factory
        self.reader = None
        self.profile_error = None
        self.calibration = None
        self.recording = None
        self.last_recording_statistics = {}
        try:
            self.calibration = RotaryCalibration(Path(base_path)/config["calibration_profile"], config)
        except Exception as exc:
            self.profile_error = str(exc)

    def connect(self, mode="Voltage ratio"):
        if self.reader and self.reader.thread.is_alive():
            if self.reader.mode != mode or self.reader.stop.is_set():
                raise RotaryError("Close the existing sensor channel before changing mode/reconnecting")
            return self.reader
        self.reader = self.reader_factory(self.config["hub_serial"], mode, self.config["target_interval_ms"])
        self.reader.start()
        return self.reader

    def measurement(self, *, now=None, require=False):
        # Finalization can clear the recorder concurrently with GUI/acquisition reads.
        reader, recording = self.reader, self.recording
        reason = self.profile_error or "Rotary sensor is not connected"
        result = {"valid": False, "reason": reason, "angle_deg": None, "sample": None,
                  "sample_age_s": None, "achieved_rate_hz": 0.0, "metadata": {}}
        if reader and self.calibration:
            sample, metadata, error, count, rate = reader.snapshot()
            result.update(sample=sample, metadata=metadata, achieved_rate_hz=rate)
            reason = error or ("Rotary sensor channel is closed" if reader.stop.is_set() else None)
            if not reason and metadata.get("input_mode") != "Voltage ratio":
                reason = "Normal tests require the calibrated voltage-ratio mode"
            if not reason and sample is None:
                reason = "Waiting for delivered rotary samples"
            if sample is not None:
                age = (time.monotonic() if now is None else now)-sample[0]
                angle, valid, status = self.calibration.evaluate(sample[2])
                result.update(angle_deg=angle, sample_age_s=age, sample_status=status)
                if not reason and not 0 <= age <= self.config["stale_after_ms"]/1000.0:
                    reason = "Rotary sensor readings are stale"
                if not reason and not valid:
                    reason = status
            if not reason and count < self.config["minimum_startup_samples"]:
                reason = "Waiting for rotary sampling-rate check"
            configured_rate = 1000.0/metadata.get("interval_ms", self.config["target_interval_ms"])
            if not reason and (configured_rate < self.config["minimum_event_rate_hz"]
                               or rate < max(self.config["minimum_event_rate_hz"], configured_rate*0.75)):
                reason = "Rotary delivered sampling rate is too low"
            if not reason and recording and recording.error:
                reason = recording.error
            if not reason and recording and recording.invalid_count:
                reason = "Rotary recording contains an invalid/out-of-range sample"
            result.update(valid=not bool(reason), reason=reason or "Ready")
        if require and not result["valid"]:
            raise RotaryError(result["reason"])
        return result

    def start_recording(self, path, started):
        self.measurement(require=True)
        if self.recording:
            raise RotaryError("A rotary recording is already active")
        self.recording = RotaryRecording(self.reader, self.calibration, path, started)

    def finish_recording(self):
        recording = self.recording
        if not recording:
            return {}
        try:
            return recording.close()
        finally:
            self.last_recording_statistics = recording.statistics()
            if not recording.thread.is_alive():
                self.recording = None

    def metadata(self):
        return {"required_for_normal_tests": True,
                "calibration": self.calibration.metadata() if self.calibration else None,
                "profile_error": self.profile_error, "connection": self.measurement()["metadata"],
                "angle_use": "Independent measurement; motor commands, zero and torque calculation unchanged"}

    def disconnect(self):
        if self.reader:
            self.reader.stop.set()

    def shutdown(self):
        self.disconnect()
        if self.reader:
            self.reader.thread.join(timeout=5)
            if self.reader.thread.is_alive():
                raise RotaryError("Rotary sensor worker has not closed")
        return self.finish_recording()
