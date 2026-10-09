"""Independent P3022 measurement window within EAST; never commands motor motion."""
import csv
from datetime import datetime
import math
from pathlib import Path
import queue
import threading
import time

import customtkinter as ctk

from east_core import write_json_atomic
from east_rotary_calibration import (
    INPUT_MODES, SAMPLE_FIELDS, POINT_FIELDS, StationaryCaptureGuard,
    finite_number, reading_valid, summarise_hold, fit_calibration, write_table,
)


class RotaryReader:
    """Open/read/close the selected hub only in this worker; no Tk calls."""
    CONNECT_TIMEOUT_S = 8.0

    def __init__(self, serial, mode):
        self.serial, self.mode = serial, mode
        self.events = queue.Queue()
        self.stop = threading.Event()
        self.phase = "Starting sensor worker"
        self.started_monotonic = None
        self.timed_out = False
        self.thread = threading.Thread(target=self._run, name="rotary-sensor-reader", daemon=True)

    def start(self):
        self.started_monotonic = time.monotonic()
        self.thread.start()

    def _progress(self, phase):
        self.phase = phase
        self.events.put(("progress", phase))

    def _run(self):
        channel = None
        try:
            self._progress("Loading Phidget driver")
            # Address the hub explicitly; never open the load-cell Bridge.
            if self.mode == "Voltage ratio":
                from Phidget22.Devices.VoltageRatioInput import VoltageRatioInput
                channel = VoltageRatioInput()
            elif self.mode == "Voltage":
                from Phidget22.Devices.VoltageInput import VoltageInput
                channel = VoltageInput()
            else:
                raise ValueError("Unsupported sensor input mode")
            channel.setDeviceSerialNumber(self.serial)
            channel.setHubPort(0)
            channel.setIsHubPortDevice(True)
            channel.setChannel(0)

            def error(_channel, *details):
                self.events.put(("error", "Sensor disconnected/error: " + str(details)))
                self.stop.set()

            channel.setOnDetachHandler(error)
            channel.setOnErrorHandler(error)
            self._progress(f"Opening hub {self.serial}, port 0, {self.mode} (4-second attachment timeout)")
            channel.openWaitForAttachment(4000)
            if self.stop.is_set():
                return
            self._progress("Checking attached hub and configuring sampling")
            # A built-in analog channel identifies itself as VoltageRatioInput_PORT
            # or VoltageInput_PORT. Validate its parent hub, not the channel SKU.
            channel_sku = channel.getDeviceSKU()
            hub = channel.getHub()
            sku = hub.getDeviceSKU()
            hub_serial = hub.getDeviceSerialNumber()
            if not sku.startswith("HUB0007"):
                raise RuntimeError(f"Expected parent HUB0007, received {sku}; no calibration acquired")
            if (hub_serial != self.serial or channel.getDeviceSerialNumber() != self.serial
                    or channel.getHubPort() != 0 or not channel.getIsHubPortDevice()):
                raise RuntimeError("Attached sensor does not match the selected hub serial and analog port 0; no calibration acquired")
            interval = max(channel.getMinDataInterval(), min(100, channel.getMaxDataInterval()))
            channel.setDataInterval(interval)
            if self.mode == "Voltage ratio":
                channel.setVoltageRatioChangeTrigger(0.0)
                get_reading = channel.getVoltageRatio
            else:
                channel.setVoltageChangeTrigger(0.0)
                get_reading = channel.getVoltage
            self._progress("Waiting for the first sensor reading")
            metadata = {"hub_serial": hub_serial, "sku": sku, "channel_sku": channel_sku,
                        "interval_ms": interval, "input_mode": self.mode}
            attached_at = time.monotonic()
            connected = False
            while not self.stop.is_set():
                try:
                    value = get_reading()
                except Exception as exc:
                    # Phidget's first value can be UNKNOWNVAL (0x33) immediately
                    # after attachment. Retry only this initial, documented state.
                    if not connected and getattr(exc, "code", None) == 0x33 and time.monotonic() - attached_at < 3:
                        self.stop.wait(0.1)
                        continue
                    raise
                if self.stop.is_set():
                    break
                if not math.isfinite(value):
                    raise ValueError("Sensor returned a non-finite reading")
                if not connected:
                    self.events.put(("connected", metadata))
                    connected = True
                    self.phase = "Reading sensor"
                self.events.put(("sample", (time.monotonic(),
                    datetime.now().astimezone().isoformat(timespec="milliseconds"), value)))
                # Explicit reads produce stationary samples even when no change
                # callback arrives. Wait is interruptible for disconnect/shutdown.
                if self.stop.wait(interval / 1000.0):
                    break
        except Exception as exc:
            self.events.put(("error", f"{self.phase}: {type(exc).__name__}: {exc}"))
        finally:
            if channel is not None:
                try:
                    channel.close()
                except Exception as exc:
                    self.events.put(("error", "Sensor close failed: " + str(exc)))
            self.events.put(("closed", None))


class RotaryCalibrationWindow:
    def __init__(self, master, app, *, preview=False):
        self.app, self.preview = app, preview
        self.reader = None
        self.sensor = None
        self.latest = None
        self.capture = None
        self.points = []
        self.session_dir = None
        self.session_metadata = None
        self.identity = None
        self.closed = False
        self.closing = False
        self.window = ctk.CTkToplevel(master)
        # A modeless child window avoids native macOS automatic tab grouping.
        self.window.transient(master)
        self.window.title("EAST — Rotary Sensor Calibration" + (" (PREVIEW)" if preview else ""))
        self.window.geometry("820x710")
        self.window.minsize(680, 540)
        self.window.configure(fg_color="#f8fafc")
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        if app is not None:
            self.window.bind("<Escape>", lambda _event: app.stop_logging())
        panel = ctk.CTkScrollableFrame(self.window, fg_color="#ffffff")
        panel.pack(fill="both", expand=True, padx=14, pady=14)
        panel.grid_columnconfigure((0, 1, 2), weight=1)
        ctk.CTkLabel(panel, text="Independent rotary-sensor calibration", font=("Arial", 21, "bold"),
                     text_color="#0f172a").grid(row=0, column=0, columnspan=3, sticky="w", pady=6)
        instructions = (
            "Use EAST Manual Mode to position the empty fixture. Then measure its ACTUAL angle "
            "with the square/protractor. The motor command is only a positioning aid.\n"
            "Enter the measured signed change from physical 90° (e.g. 85° → −5°, 95° → +5°). "
            "Capture 0 and points on both sides; repeat from both directions. "
            "Use separate Validation points to check the fitted calibration.\n"
            "No sensor fit is applied to motor control, machine zero, tare or strain-test angles."
        )
        ctk.CTkLabel(panel, text=instructions, wraplength=720, justify="left", anchor="w",
                     text_color="#334155").grid(row=1, column=0, columnspan=3, sticky="ew", pady=8)
        self.serial = self._entry(panel, 2, 0, "HUB0007 serial", "750256")
        ctk.CTkLabel(panel, text="Sensor input mode").grid(row=2, column=1, sticky="w")
        self.mode = ctk.StringVar(value="Voltage ratio")
        self.mode_menu = ctk.CTkOptionMenu(panel, values=list(INPUT_MODES), variable=self.mode)
        self.mode_menu.grid(row=3, column=1, sticky="ew", padx=4)
        self.supply = self._entry(panel, 2, 2, "Measured VCC (optional metadata)", "")
        self.connect_button = ctk.CTkButton(panel, text="Connect Sensor", command=self.connect)
        self.connect_button.grid(row=4, column=0, sticky="ew", padx=4, pady=8)
        self.disconnect_button = ctk.CTkButton(panel, text="Disconnect Sensor", command=self.disconnect,
                                              fg_color="#64748b")
        self.disconnect_button.grid(row=4, column=1, sticky="ew", padx=4, pady=8)
        self.new_button = ctk.CTkButton(panel, text="New Calibration Session", command=self.new_session,
                                       fg_color="#64748b")
        self.new_button.grid(row=4, column=2, sticky="ew", padx=4, pady=8)
        self.live_label = ctk.CTkLabel(panel, text="Sensor not connected", font=("Arial", 17, "bold"),
                                      text_color="#0f172a", anchor="w")
        self.live_label.grid(row=5, column=0, columnspan=3, sticky="ew", pady=5)
        self.angle = self._entry(panel, 6, 0, "Independent angle from 90° (deg)", "0")
        self.repeat = self._entry(panel, 6, 1, "Repeat number", "1")
        self.duration = self._entry(panel, 6, 2, "Stationary hold (seconds)", "10")
        self.instrument = self._entry(panel, 8, 0, "Independent angle instrument", "Trojan square/protractor")
        self.resolution = self._entry(panel, 8, 1, "Reference graduation spacing (deg)", "")
        ctk.CTkLabel(panel, text="Point purpose").grid(row=8, column=2, sticky="w")
        self.role = ctk.StringVar(value="Calibration")
        self.role_menu = ctk.CTkOptionMenu(panel, values=["Calibration", "Validation"], variable=self.role)
        self.role_menu.grid(row=9, column=2, sticky="ew", padx=4)
        ctk.CTkLabel(panel, text="Approach direction").grid(row=10, column=0, sticky="w", pady=(8, 0))
        self.approach = ctk.StringVar(value="Increasing angle")
        self.approach_menu = ctk.CTkOptionMenu(panel, variable=self.approach,
            values=["Increasing angle", "Decreasing angle", "Not recorded"])
        self.approach_menu.grid(row=11, column=0, sticky="ew", padx=4)
        self.acknowledged = ctk.BooleanVar(value=False)
        self.ack = ctk.CTkCheckBox(panel, text="Empty fixture; angle independently measured; stationary",
                                  variable=self.acknowledged)
        self.ack.grid(row=11, column=1, columnspan=2, sticky="w", padx=4)
        self.capture_button = ctk.CTkButton(panel, text="Capture Stationary Point", command=self.start_capture,
                                           fg_color="#15803d")
        self.capture_button.grid(row=12, column=0, sticky="ew", padx=4, pady=10)
        self.cancel_button = ctk.CTkButton(panel, text="Cancel Capture", command=lambda: self.cancel_capture("Cancelled by operator"),
                                          fg_color="#b45309")
        self.cancel_button.grid(row=12, column=1, sticky="ew", padx=4, pady=10)
        self.fit_button = ctk.CTkButton(panel, text="Fit + Save Calibration Report", command=self.fit)
        self.fit_button.grid(row=12, column=2, sticky="ew", padx=4, pady=10)
        self.status = ctk.CTkLabel(panel, text="Motor controls remain in EAST Manual Mode. They are blocked during each hold.",
                                  justify="left", wraplength=720, anchor="w", text_color="#92400e")
        self.status.grid(row=13, column=0, columnspan=3, sticky="ew", pady=4)
        self.table = ctk.CTkTextbox(panel, height=150, wrap="none", font=("Courier", 12))
        self.table.grid(row=14, column=0, columnspan=3, sticky="ew", pady=6)
        self.result = ctk.CTkLabel(panel, text="No calibration fitted. Reference graduation spacing limits the conclusion.",
                                  wraplength=720, justify="left", anchor="w", text_color="#334155")
        self.result.grid(row=15, column=0, columnspan=3, sticky="ew", pady=6)
        self.path_label = ctk.CTkLabel(panel, text="Files are saved automatically when a point is captured.",
                                      wraplength=720, justify="left", anchor="w", text_color="#64748b")
        self.path_label.grid(row=16, column=0, columnspan=3, sticky="ew", pady=5)
        self._controls()
        self.window.after(50, self._tick)

    @staticmethod
    def _entry(panel, row, column, label, value):
        ctk.CTkLabel(panel, text=label, anchor="w").grid(row=row, column=column, sticky="ew", padx=4, pady=(5, 0))
        entry = ctk.CTkEntry(panel)
        entry.grid(row=row + 1, column=column, sticky="ew", padx=4, pady=3)
        entry.insert(0, value)
        return entry

    def _controls(self):
        # CTk configuration can process nested Tk events. A failure handled during
        # a refresh must not be overwritten by the outer refresh's old state.
        if getattr(self, "_updating_controls", False):
            self._controls_pending = True
            return
        self._updating_controls = True
        self._controls_pending = False
        try:
            busy = self.capture is not None
            active = self.reader is not None
            for widget in (self.serial, self.mode_menu):
                widget.configure(state="disabled" if active or self.points else "normal")
            for widget in (self.angle, self.repeat, self.duration, self.role_menu, self.approach_menu, self.ack):
                widget.configure(state="disabled" if busy else "normal")
            for widget in (self.instrument, self.resolution, self.supply):
                widget.configure(state="disabled" if busy or self.session_dir else "normal")
            self.connect_button.configure(state="normal" if not active and not self.preview else "disabled")
            self.disconnect_button.configure(state="normal" if active else "disabled")
            self.capture_button.configure(state="normal" if self.sensor and not busy and not self.preview else "disabled")
            self.cancel_button.configure(state="normal" if busy else "disabled")
            self.fit_button.configure(state="normal" if self.points and not busy else "disabled")
            self.new_button.configure(state="disabled" if busy else "normal")
        finally:
            self._updating_controls = False
            if self._controls_pending:
                self._controls_pending = False
                self._controls()

    def connect(self):
        if self.reader is not None or self.preview:
            return
        try:
            serial = int(self.serial.get())
            if serial <= 0:
                raise ValueError("Hub serial must be positive")
            self.sensor, self.latest = None, None
            self.reader = RotaryReader(serial, self.mode.get())
            self._sensor_status(f"Connecting hub {serial}, port 0 as {self.mode.get()}...")
            self.reader.start()
            self._controls()
        except Exception as exc:
            if self.reader is not None and not self.reader.thread.is_alive():
                self.reader = None
            self._sensor_status("Sensor connection failed: " + str(exc))
            self._controls()

    def _sensor_status(self, message):
        self.status.configure(text=message)
        if self.app is not None:
            self.app.update_terminal("ROTARY SENSOR: " + message + "\n")

    def disconnect(self):
        self.cancel_capture("Sensor disconnected by operator")
        if self.reader:
            self.reader.stop.set()
            self.status.configure(text="Disconnecting sensor...")

    def _context(self):
        app = self.app
        operator, fixture = app.operator_input.get().strip(), app.fixture_id_input.get().strip()
        if not operator or not fixture:
            raise ValueError("Enter Operator ID and Fixture ID in EAST first")
        mapping = app.reference_manager.require_verified()
        return {"operator_id": operator, "fixture_id": fixture,
                "reference_id": mapping.reference_id, "hub_serial": self.sensor["hub_serial"],
                "input_mode": self.sensor["input_mode"]}

    def start_capture(self):
        guard = None
        handle = None
        try:
            if self.capture or not self.sensor or not self.latest or time.monotonic() - self.latest[0] > 1:
                raise ValueError("Wait for fresh connected sensor readings")
            if not self.acknowledged.get():
                raise ValueError("Confirm the empty fixture, independent angle and stationary condition")
            angle = finite_number(self.angle.get(), "Independent angle")
            duration = finite_number(self.duration.get(), "Hold duration")
            repeat = int(self.repeat.get())
            if not -90 < angle < 90 or not 2 <= duration <= 30 or repeat < 1:
                raise ValueError("Angle must be within ±90°, hold 2–30 seconds, repeat ≥1")
            instrument = self.instrument.get().strip()
            if not instrument:
                raise ValueError("Name the independent angle instrument")
            resolution = finite_number(self.resolution.get(), "Graduation spacing") if self.resolution.get().strip() else None
            supply = finite_number(self.supply.get(), "Measured supply") if self.supply.get().strip() else None
            if resolution is not None and resolution <= 0 or supply is not None and not 0 < supply <= 5.3:
                raise ValueError("Graduation spacing must be positive; supply metadata must be in (0, 5.3] V")
            context = self._context()
            if self.identity is not None and self.identity != context:
                raise ValueError("Operator, fixture, zero, hub or input mode changed; start a new calibration session")
            guard = StationaryCaptureGuard(self.app)
            if self.session_dir is None:
                root = Path(self.app.system_config["logging"]["output_directory"])
                if not root.is_absolute():
                    root = Path(__file__).resolve().parent / root
                stamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S_%f%z")
                self.session_dir = root / "Rotary Calibration" / f"rotary_{stamp}"
                self.session_dir.mkdir(parents=True, exist_ok=False)
                self.identity = context
                self.session_metadata = {
                    **context, "created_at": datetime.now().astimezone().isoformat(),
                    "sensor": dict(self.sensor), "instrument": instrument,
                    "reference_graduation_spacing_deg": resolution, "measured_supply_v": supply,
                    "supply_use": "operator-reported metadata only; never used to normalise ratio readings",
                    "machine_zero": self.app.reference_manager.record.to_dict(),
                    "software_version": self.app.system_config["software_version"],
                    "accepted_motor_conversion": self.app.system_config["motion"]["afo_degrees_per_odrive_turn"],
                    "timestamp_basis": "host callback arrival; not validated for dynamic speed",
                    "reference_angle_basis": "independent operator measurement, not EAST command/derived angle",
                }
                write_json_atomic(self.session_dir / "session_metadata.json", self.session_metadata)
            raw_path = self.session_dir / "raw_samples.csv"
            new_file = not raw_path.exists()
            handle = raw_path.open("a", newline="", encoding="utf-8")
            writer = csv.DictWriter(handle, fieldnames=SAMPLE_FIELDS, extrasaction="ignore")
            if new_file:
                writer.writeheader()
            mode = self.sensor["input_mode"]
            metadata = {"point_id": len(self.points) + 1, "role": self.role.get(),
                        "reference_angle_deg": angle, "repeat": repeat, "approach": self.approach.get(),
                        "input_mode": mode, "unit": "V/V" if mode == "Voltage ratio" else "V"}
            self.capture = {"guard": guard, "handle": handle, "writer": writer, "samples": [],
                            "metadata": metadata, "start": time.monotonic(), "duration": duration}
            self.acknowledged.set(False)
            self.app._refresh_motion_controls()
            self._controls()
        except Exception as exc:
            if handle:
                handle.close()
            if guard:
                guard.close()
            self.status.configure(text="Capture blocked: " + str(exc))

    def _record_sample(self, sample):
        capture = self.capture
        if not capture or not capture["start"] <= sample[0] <= capture["start"] + capture["duration"]:
            return
        feedback, feedback_error = None, None
        try:
            feedback = capture["guard"].check()
        except Exception as exc:
            feedback_error = exc
        mode = capture["metadata"]["input_mode"]
        row = {**capture["metadata"], "host_monotonic_s": sample[0], "host_timestamp_iso": sample[1],
               "raw_reading": sample[2], "sensor_valid": reading_valid(sample[2], mode),
               "motor_position_turns": feedback.position_turns if feedback else None,
               "motor_velocity_turns_s": feedback.velocity_turns_s if feedback else None,
               "motor_feedback_valid": feedback is not None, "hub_serial": self.sensor["hub_serial"],
               "measured_supply_v": self.session_metadata["measured_supply_v"]}
        capture["writer"].writerow(row)
        capture["handle"].flush()
        capture["samples"].append(row)
        if feedback_error:
            raise feedback_error
        if not row["sensor_valid"]:
            raise ValueError("Out-of-range sensor reading; raw data preserved")

    def cancel_capture(self, reason="Cancelled"):
        if self.capture:
            self._finish_capture(reason)

    def _finish_capture(self, failure=None):
        capture, self.capture = self.capture, None
        if not capture:
            return
        try:
            point = summarise_hold(capture["samples"], capture["metadata"], failure=failure)
            self.points.append(point)
            write_table(self.session_dir / "points.csv", POINT_FIELDS, self.points)
            write_json_atomic(self.session_dir / "points.json", {"metadata": self.session_metadata, "points": self.points})
            message = (f"Point {point['point_id']}: {point['samples']} samples. "
                       + ("Recorded; ready for next position." if point["valid"] else "EXCLUDED: " + point["reason"]))
            self.status.configure(text=message)
            self.path_label.configure(text=str(self.session_dir))
            self._show_points()
            self.result.configure(text="Points changed. Fit again to update the provisional report.")
            # Do not leave an old report looking like it includes newer points.
            write_json_atomic(self.session_dir / "calibration_report.json", {
                "status": "fit_required_points_changed", "metadata": self.session_metadata,
                "point_count": len(self.points), "motor_control_changed": False})
        except Exception as exc:
            self.status.configure(text="Saving point failed; raw file retained: " + str(exc))
        finally:
            capture["handle"].close()
            capture["guard"].close()
            self.app._refresh_motion_controls()
            self._controls()

    def _show_points(self):
        self.table.configure(state="normal")
        self.table.delete("1.0", "end")
        self.table.insert("end", "ID Purpose       Angle   Rep    Sensor mean       SD     N  Status\n")
        for p in self.points:
            mean = f"{p['raw_mean']:.8f}" if p['raw_mean'] is not None else "unavailable"
            sd = f"{p['raw_sd']:.6f}" if p['raw_sd'] is not None else "   n/a"
            self.table.insert("end", f"{p['point_id']:2} {p['role']:11} {p['reference_angle_deg']:+6.2f} "
                f"{p['repeat']:4} {mean:>14} {sd:>9} {p['samples']:5} {'recorded' if p['valid'] else 'EXCLUDED'}\n")
        self.table.configure(state="disabled")

    def fit(self):
        try:
            model = fit_calibration(self.points)
            write_json_atomic(self.session_dir / "calibration_report.json", {
                "metadata": self.session_metadata, "model": model,
                "points": self.points, "assessment": "provisional; assess independent validation and reference uncertainty"})
            message = (f"PROVISIONAL local fit: angle = {model['slope']:.6g} × reading {model['intercept']:+.6g} deg.\n"
                       f"Calibration RMSE {model['rmse_deg']:.4f}°; max residual {model['max_abs_residual_deg']:.4f}°. "
                       f"Range {model['reference_angle_range_deg']}.\n")
            if model['validation_count']:
                message += (f"Independent validation: {model['validation_count']} holds; "
                            f"RMSE {model['validation_rmse_deg']:.4f}°; max error {model['validation_max_abs_error_deg']:.4f}°.\n")
            else:
                message += "No independent validation points yet.\n"
            if "motor_conversion_fit" in model:
                motor = model['motor_conversion_fit']
                message += f"Independent angle versus motor turns: {motor['slope']:+.6g} deg/turn (magnitude {abs(motor['slope']):.6g}).\n"
            message += "No automatic acceptance or changes to EAST control/calibration. Assess protractor resolution."
            self.result.configure(text=message)
        except Exception as exc:
            self.result.configure(text="Fit blocked: " + str(exc))

    def new_session(self):
        if self.capture:
            return
        self.points, self.session_dir, self.session_metadata, self.identity = [], None, None, None
        self.result.configure(text="New session. Previous files remain unchanged.")
        self.path_label.configure(text="Files will be saved when a point is captured.")
        self._show_points()
        self._controls()

    def _tick(self):
        if self.closed:
            return
        try:
            reader = self.reader
            if reader:
                # Bound queue draining so sensor events cannot monopolise Tk.
                for _ in range(200):
                    try:
                        kind, payload = reader.events.get_nowait()
                    except queue.Empty:
                        break
                    if reader.timed_out and kind != "closed":
                        continue
                    if kind == "progress":
                        if not reader.stop.is_set():
                            self._sensor_status(payload)
                    elif kind == "connected":
                        if reader.stop.is_set():
                            continue
                        self.sensor = payload
                        self._sensor_status(f"Sensor connected: hub {payload['hub_serial']}, {payload['input_mode']}, {payload['interval_ms']} ms. No angle calibration applied.")
                        self._controls()
                    elif kind == "sample":
                        if reader.stop.is_set():
                            continue
                        self.latest = payload
                        self._record_sample(payload)
                    elif kind == "error":
                        reader.stop.set()
                        self.cancel_capture(payload)
                        self.sensor, self.latest = None, None
                        self._sensor_status("Sensor connection/read failed: " + payload)
                        self._controls()
                    elif kind == "closed":
                        self.cancel_capture("Sensor channel closed")
                        self.sensor, self.latest = None, None
                        # Retain ownership until the worker has actually exited.
                if (not self.sensor and not reader.stop.is_set()
                        and reader.started_monotonic is not None
                        and time.monotonic() - reader.started_monotonic >= reader.CONNECT_TIMEOUT_S):
                    reader.timed_out = True
                    reader.stop.set()
                    self.sensor, self.latest = None, None
                    self._sensor_status(f"Connection timed out while: {reader.phase}. Hub {reader.serial}, port 0. "
                        "Check the hub serial, Phidget Windows driver and other sensor programs. "
                        "Waiting for the driver to close; do not open a second sensor session.")
                    self._controls()
            if (self.reader and self.reader.started_monotonic is not None
                    and not self.reader.thread.is_alive()):
                self.reader = None
                self.sensor, self.latest = None, None
                self._controls()
            fresh = self.latest and time.monotonic() - self.latest[0] <= 1
            if fresh and self.sensor:
                unit = "V/V" if self.sensor["input_mode"] == "Voltage ratio" else "V"
                self.live_label.configure(text=f"Raw sensor: {self.latest[2]:.8f} {unit} — uncalibrated")
            else:
                self.live_label.configure(text="Sensor reading unavailable / stale")
            if self.capture:
                self.capture["guard"].check()
                if not fresh:
                    raise ValueError("Sensor readings became stale during capture")
                elapsed = time.monotonic() - self.capture["start"]
                self.status.configure(text=f"Capturing stationary point: {elapsed:.1f} / {self.capture['duration']:g} s. Motor steps blocked.")
                if elapsed >= self.capture["duration"]:
                    self._finish_capture()
            if self.closing and self.reader is None:
                self.closed = True
                self.window.destroy()
        except Exception as exc:
            self.cancel_capture(str(exc))
            self._sensor_status("Sensor GUI error: " + str(exc))
        finally:
            # An error in a queued event must never silently stop the UI pump.
            if not self.closed:
                self.window.after(50, self._tick)

    def close(self):
        self.closing = True
        self.disconnect()

    def shutdown(self):
        """Used by EAST shutdown: preserve partial hold and wait for sensor cleanup."""
        self.cancel_capture("EAST shutting down")
        self.closed = True
        if self.reader:
            self.reader.stop.set()
            self.reader.thread.join(timeout=5.0)
            if self.reader.thread.is_alive():
                raise RuntimeError("Sensor reader has not closed; wait before restarting EAST")
