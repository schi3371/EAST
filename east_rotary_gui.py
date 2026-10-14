"""Independent P3022 measurement and bounded empty-fixture calibration positioning."""
import csv
from datetime import datetime
import math
from pathlib import Path
import queue
import threading
import time

import customtkinter as ctk

from east_calibration_motion import CalibrationPositioningReference
from east_core import write_json_atomic
from east_runtime import CSVRecorder
from east_rotary_calibration import (
    INPUT_MODES, SAMPLE_FIELDS, POINT_FIELDS, RawMotorCaptureGuard, SensorOnlyCaptureGuard,
    finite_number, reading_valid, summarise_hold, fit_calibration, write_table,
)


from east_rotary_sensor import RotaryReader


class RotaryCalibrationWindow:
    def __init__(self, master, app, *, preview=False):
        self.app, self.preview = app, preview
        self.reader = None
        self._shared_sensor = getattr(app, "rotary_sensor", None) if not preview else None
        self.sensor = None
        self.latest = None
        self.capture = None
        self.persistence_busy = False
        self.pending_capture_cleanup = None
        self.points = []
        self.session_dir = None
        self.session_metadata = None
        self.identity = None
        self.closed = False
        self.closing = False
        self.positioning_reference = None
        self.positioning_events = []
        self.motor_epoch = 0
        self.last_motor_connection = None
        self.last_motor_uptime = None
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
            "Use the calibration movement controls below to position the empty fixture. Then measure its ACTUAL angle "
            "with the square/protractor. The motor command is only a positioning aid.\n"
            "Enter the measured change from physical 90°: dorsiflexion positive, plantarflexion negative. "
            "Capture 0 and points on both sides; repeat from both directions. "
            "Use separate Validation points to check the fitted calibration.\n"
            "Sensor only captures remain available without motor zero; motor turns are unverified and excluded from its fit.\n"
            "Sensor + motor captures record stationary raw turns without requiring machine zero or load-cell tare.\n"
            "New fits remain provisional. Normal tests use the separately accepted sensor profile; motor control, machine zero and tare are unchanged."
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
        if self._shared_sensor:
            # The main green Connect owns all hardware in the normal application.
            self.connect_button.grid_remove()
            self.disconnect_button.grid_remove()
            self.new_button.grid_configure(column=0, columnspan=3)
            self.serial.delete(0, "end")
            self.serial.insert(0, str(self._shared_sensor.config["hub_serial"]))
            self.mode_menu.configure(values=["Voltage ratio"])
        self.live_label = ctk.CTkLabel(panel, text="Sensor not connected", font=("Arial", 17, "bold"),
                                      text_color="#0f172a", anchor="w")
        self.live_label.grid(row=5, column=0, columnspan=3, sticky="ew", pady=5)
        self.angle = self._entry(panel, 6, 0, "Independent angle from 90° (deg)", "0")
        self.repeat = self._entry(panel, 6, 1, "Repeat number", "1")
        self.duration = self._entry(panel, 6, 2, "Stationary hold (seconds)", "10")
        self.instrument = self._entry(panel, 8, 0, "Independent angle instrument", "WOLF WDAM425 angle finder")
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
        self.capture_mode = ctk.StringVar(value="Sensor + motor")
        self.capture_mode_menu = ctk.CTkOptionMenu(panel, values=["Sensor + motor", "Sensor only"],
                                                variable=self.capture_mode)
        self.capture_mode_menu.grid(row=13, column=0, sticky="ew", padx=4)
        self.status.grid_configure(column=1, columnspan=2)
        self.status.configure(wraplength=480)
        self.table = ctk.CTkTextbox(panel, height=150, wrap="none", font=("Courier", 12))
        self.table.grid(row=14, column=0, columnspan=3, sticky="ew", pady=6)
        self.result = ctk.CTkLabel(panel, text="No calibration fitted. Reference graduation spacing limits the conclusion.",
                                  wraplength=720, justify="left", anchor="w", text_color="#334155")
        self.result.grid(row=15, column=0, columnspan=3, sticky="ew", pady=6)
        self.path_label = ctk.CTkLabel(panel, text="Files are saved automatically when a point is captured.",
                                      wraplength=720, justify="left", anchor="w", text_color="#64748b")
        self.path_label.grid(row=16, column=0, columnspan=3, sticky="ew", pady=5)
        ctk.CTkLabel(panel, text="Empty-fixture calibration movement", font=("Arial", 17, "bold"),
                     text_color="#0f172a").grid(row=17, column=0, columnspan=3, sticky="w", pady=(14, 5))
        ctk.CTkLabel(panel, text=(
            "Machine zero and empty-machine tare are NOT required for these controls. "
            "After reconnecting, confirm the last positioning angle only if the fixture has not moved; "
            "otherwise enter its current measured angle. This is an approximate positioning reference, "
            "not a measured calibration point or verified machine zero. Existing conversion and estimated "
            "travel bounds apply. Enclosure/E-stop, feedback checks and movement timeouts remain active."
        ), wraplength=720, justify="left").grid(row=18, column=0, columnspan=3, sticky="ew")
        self.positioning_angle = self._entry(panel, 19, 0, "Current positioning angle (deg)", "")
        self.step = self._entry(panel, 19, 1, "Increment per click", "0.25")
        self.step_units = ctk.StringVar(value="Degrees")
        ctk.CTkLabel(panel, text="Step units").grid(row=19, column=2, sticky="w")
        self.step_units_menu = ctk.CTkOptionMenu(panel, values=["Degrees", "Motor turns"],
            variable=self.step_units, command=lambda _value: self.step.delete(0, "end"))
        self.step_units_menu.grid(row=20, column=2, sticky="ew", padx=4)
        self.motion_acknowledged = ctk.BooleanVar(value=False)
        self.motion_ack = ctk.CTkCheckBox(panel, text="Empty fixture; current positioning angle confirmed; clear for movement",
                                        variable=self.motion_acknowledged)
        self.motion_ack.grid(row=21, column=0, columnspan=3, sticky="w", pady=8)
        self.reconnect_motor_button = ctk.CTkButton(panel, text="Reconnect ODrive", command=self.reconnect_motor)
        self.reconnect_motor_button.grid(row=22, column=0, sticky="ew", padx=4, pady=4)
        self.enable_motion_button = ctk.CTkButton(panel, text="Enable / Resume Calibration Motion", command=self.enable_positioning)
        self.enable_motion_button.grid(row=22, column=1, columnspan=2, sticky="ew", padx=4, pady=4)
        self.left_button = ctk.CTkButton(panel, text="Step Left / Negative", command=lambda: self.move(-1))
        self.left_button.grid(row=23, column=0, sticky="ew", padx=4, pady=4)
        self.right_button = ctk.CTkButton(panel, text="Step Right / Positive", command=lambda: self.move(1))
        self.right_button.grid(row=23, column=1, sticky="ew", padx=4, pady=4)
        self.stop_button = ctk.CTkButton(panel, text="Stop", fg_color="#b91c1c",
                                       command=lambda: self.app.stop_logging() if self.app else None)
        self.stop_button.grid(row=23, column=2, sticky="ew", padx=4, pady=4)
        self.motion_status = ctk.CTkLabel(panel, text="Calibration positioning disabled; no verified zero is created.",
                                         wraplength=720, justify="left", anchor="w")
        self.motion_status.grid(row=24, column=0, columnspan=3, sticky="ew", pady=5)
        if app is not None and app.reference_manager.verified:
            try:
                feedback = app.get_feedback()
                mapping = app.reference_manager.require_verified()
                nominal = (feedback.position_turns - mapping.neutral_position_turns) * app.system_config["motion"]["afo_degrees_per_odrive_turn"]
                self.positioning_angle.insert(0, f"{nominal:.6g}")
            except Exception:
                pass
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
            busy = self.capture is not None or self.persistence_busy or (self.app is not None and self.app.engine.busy is not None)
            moving = self.app is not None and self.app.motion_coordinator.owner is not None
            active = self.reader is not None
            for widget in (self.serial, self.mode_menu):
                self.app._set_widget_state(widget, 'disabled' if active or self.points or getattr(self, '_shared_sensor', None) else 'normal')
            for widget in (self.angle, self.repeat, self.duration, self.role_menu, self.approach_menu, self.ack):
                self.app._set_widget_state(widget, 'disabled' if busy else 'normal')
            for widget in (self.instrument, self.resolution, self.supply):
                self.app._set_widget_state(widget, 'disabled' if busy or self.session_dir else 'normal')
            if hasattr(self, "capture_mode_menu"):
                self.app._set_widget_state(self.capture_mode_menu, 'disabled' if busy else 'normal')
            self.app._set_widget_state(self.connect_button, 'normal' if not active and (not self.preview) else 'disabled')
            self.app._set_widget_state(self.disconnect_button, 'normal' if active and (not moving) else 'disabled')
            self.app._set_widget_state(self.capture_button, 'normal' if self.sensor and (not busy) and (not self.preview) else 'disabled')
            self.app._set_widget_state(self.cancel_button, 'normal' if busy else 'disabled')
            self.app._set_widget_state(self.fit_button, 'normal' if self.points and (not busy) else 'disabled')
            self.app._set_widget_state(self.new_button, 'disabled' if busy or moving else 'normal')
            if hasattr(self, "enable_motion_button"):
                ready = bool(self.positioning_reference and self.positioning_reference.active)
                for widget in (self.positioning_angle, self.motion_ack, self.step, self.step_units_menu,
                               self.reconnect_motor_button, self.enable_motion_button):
                    self.app._set_widget_state(widget, 'disabled' if busy or moving or self.preview else 'normal')
                for widget in (self.left_button, self.right_button):
                    self.app._set_widget_state(widget, 'normal' if ready and (not busy) and (not moving) and (not self.preview) else 'disabled')
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
            if self._shared_sensor:
                if serial != self._shared_sensor.config["hub_serial"]:
                    raise ValueError("Use the configured hub serial; replacement sensors require a matching calibration profile")
                physical_reader = self._shared_sensor.connect(self.mode.get())
                self.reader = physical_reader.subscribe()
            else:
                self.reader = RotaryReader(serial, self.mode.get())
            self._sensor_status(f"Connecting hub {serial}, port 0 as {self.mode.get()}...")
            if not self._shared_sensor:
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
        if self.app is not None and self.app.strain_test_active:
            self._sensor_status("Stop the normal test before disconnecting the sensor")
            return
        self.cancel_capture("Sensor disconnected by operator")
        if self.reader:
            self.reader.stop.set()
            self.status.configure(text="Disconnecting sensor...")

    def _context(self):
        app = self.app
        operator, fixture = app.operator_input.get().strip(), app.fixture_id_input.get().strip()
        if not operator or not fixture:
            raise ValueError("Enter Operator ID and Fixture ID in EAST first")
        # Sensor calibration uses independent physical angles. A reconnect or
        # lost machine zero does not erase those points or create a new session.
        reference_id = (self.identity["reference_id"] if self.identity else
                        app.reference_manager.record.reference_id if app.reference_manager.record else None)
        return {"operator_id": operator, "fixture_id": fixture,
                "reference_id": reference_id, "hub_serial": self.sensor["hub_serial"],
                "input_mode": self.sensor["input_mode"]}

    def start_capture(self):
        try:
            if self.persistence_busy or self.app.engine.busy is not None:
                raise ValueError("Wait for calibration/hardware persistence")
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
            motor_frame = self._motor_frame()
            if self.identity is not None and self.identity != context:
                raise ValueError("Operator, fixture, zero, hub or input mode changed; start a new calibration session")
            capture_mode = self.capture_mode.get()
            if capture_mode not in ("Sensor + motor", "Sensor only"):
                raise ValueError("Select a supported capture mode")
            role, approach = self.role.get(), self.approach.get()
            self.persistence_busy = True
            self.status.configure(text="Preparing calibration recording...")
            def prepare():
                self._prepare_capture(context, motor_frame, capture_mode, instrument, resolution,
                                      supply, angle, repeat, duration, role, approach)
            if not self.app._submit_lifecycle("calibration recording setup", prepare):
                self.persistence_busy = False
            self._controls()
        except Exception as exc:
            self.status.configure(text="Capture blocked: " + str(exc))

    def _prepare_capture(self, context, motor_frame, capture_mode, instrument, resolution,
                         supply, angle, repeat, duration, role, approach):
        guard = None
        recorder = None
        try:
            guard = SensorOnlyCaptureGuard(self.app) if capture_mode == "Sensor only" else RawMotorCaptureGuard(self.app)
            if capture_mode == "Sensor + motor":
                self.app._validate_live_motion_config()
                self.app._revalidate_hardware_identity()
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
                    "machine_zero": self.app.reference_manager.record.to_dict() if self.app.reference_manager.record else None,
                    "machine_zero_verified_at_session_start": bool(self.app.reference_manager.verified),
                    "capture_modes": "Recorded per point; sensor-only motor turns are unverified observations",
                    "software_version": self.app.system_config["software_version"],
                    "accepted_motor_conversion": self.app.system_config["motion"]["afo_degrees_per_odrive_turn"],
                    "timestamp_basis": "host arrival of delivered sensor events; not hardware-synchronised",
                    "reference_angle_basis": "independent operator measurement, not EAST command/derived angle",
                }
                write_json_atomic(self.session_dir / "session_metadata.json", self.session_metadata)
            raw_path = self.session_dir / "raw_samples.csv"
            write_json_atomic(self.session_dir / "calibration_motion_events.json", {"events": self.positioning_events})
            if not raw_path.exists():
                with raw_path.open("x", newline="", encoding="utf-8") as handle:
                    csv.writer(handle).writerow(SAMPLE_FIELDS)
            recorder = CSVRecorder(raw_path, 100, lambda exc: None, max_pending_rows=8192)
            recorder.start()
            mode = self.sensor["input_mode"]
            metadata = {"point_id": len(self.points) + 1, "role": role,
                        "reference_angle_deg": angle, "repeat": repeat, "approach": approach,
                        "input_mode": mode, "unit": "V/V" if mode == "Voltage ratio" else "V",
                        "capture_mode": capture_mode,
                        "motor_reference_id": getattr(guard, "reference_id", None),
                        "motor_coordinate_frame": motor_frame,
                        "machine_zero_verified": bool(self.app.reference_manager.verified),
                        "positioning_reference_basis": "operator_confirmed_calibration_only" if self.positioning_reference else "none"}
            capture = {"guard": guard, "recorder": recorder, "samples": [],
                       "metadata": metadata, "start": time.monotonic(), "duration": duration}
            self.app.ui_message_queue.put(("calibration_prepared", self, capture, None))
        except Exception as exc:
            if recorder:
                try: recorder.finish(1)
                except Exception: pass
            if guard:
                if recorder is not None and recorder.thread.is_alive():
                    self.pending_capture_cleanup = {"guard":guard,"recorder":recorder}
                else:
                    guard.close()
            self.app.ui_message_queue.put(("calibration_prepared", self, None, str(exc)))
            raise

    def _capture_ready(self, capture, error):
        self.persistence_busy = False
        if error:
            self.status.configure(text="Capture blocked: " + error)
        else:
            self.capture = capture
            self.acknowledged.set(False)
            if self.closing:
                self.cancel_capture("Window closed during recording setup")
        self.app._refresh_motion_controls()
        self._controls()

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
               "sensor_host_monotonic_ns": sample[5] if len(sample)>5 else None,
               "sensor_timestamp_tied": sample[6] if len(sample)>6 else None,
               "raw_reading": sample[2], "sensor_valid": reading_valid(sample[2], mode),
               "motor_position_turns": feedback.position_turns if feedback else None,
               "motor_velocity_turns_s": feedback.velocity_turns_s if feedback else None,
               "motor_feedback_valid": feedback is not None and capture["metadata"]["capture_mode"] != "Sensor only",
               "motor_feedback_status": ("unverified_observation" if feedback else "unavailable")
                   if capture["metadata"]["capture_mode"] == "Sensor only" else ("stationary_raw_turns" if feedback else "invalid"),
               "hub_serial": self.sensor["hub_serial"],
               "measured_supply_v": self.session_metadata["measured_supply_v"]}
        capture["recorder"].append([row.get(name) for name in SAMPLE_FIELDS])
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
        self.persistence_busy = True
        self.status.configure(text="Saving calibration point...")
        # The capture guard retains observation ownership until durability finishes.
        if not self.app._submit_lifecycle("calibration point finalization", lambda: self._save_capture(capture, failure)):
            self.capture = capture
            self.persistence_busy = False

    def _save_capture(self, capture, failure):
        message = None
        try:
            capture["recorder"].finish(timeout_s=3.0)
            point = summarise_hold(capture["samples"], capture["metadata"], failure=failure)
            self.points.append(point)
            write_table(self.session_dir / "points.csv", POINT_FIELDS, self.points)
            write_json_atomic(self.session_dir / "points.json", {"metadata": self.session_metadata, "points": self.points})
            message = (f"Point {point['point_id']}: {point['samples']} samples. "
                       + (("Recorded sensor only; motor turns excluded from conversion fit." if point.get("capture_mode") == "Sensor only"
                           else "Recorded; ready for next position.") if point["valid"] else "EXCLUDED: " + point["reason"]))
            # Do not leave an old report looking like it includes newer points.
            write_json_atomic(self.session_dir / "calibration_report.json", {
                "status": "fit_required_points_changed", "metadata": self.session_metadata,
                "point_count": len(self.points), "motor_control_changed": False})
        except Exception as exc:
            message = "Saving point failed; raw file retained: " + str(exc)
        finally:
            if not capture["recorder"].thread.is_alive():
                capture["guard"].close()
            else:
                message = (message or "Persistence blocked") + "; recorder and observation ownership retained."
                self.pending_capture_cleanup = capture
            self.app.ui_message_queue.put(("calibration_saved", self, message))

    def _capture_saved(self, message):
        self.persistence_busy = False
        self.status.configure(text=message)
        self.path_label.configure(text=str(self.session_dir))
        self._show_points()
        self.result.configure(text="Points changed. Fit again to update the provisional report.")
        self.app._refresh_motion_controls()
        self._controls()

    def _show_points(self):
        self.table.configure(state="normal")
        self.table.delete("1.0", "end")
        self.table.insert("end", "ID Purpose       Angle   Rep    Sensor mean       SD     N  Status / Capture mode\n")
        for p in self.points:
            mean = f"{p['raw_mean']:.8f}" if p['raw_mean'] is not None else "unavailable"
            sd = f"{p['raw_sd']:.6f}" if p['raw_sd'] is not None else "   n/a"
            self.table.insert("end", f"{p['point_id']:2} {p['role']:11} {p['reference_angle_deg']:+6.2f} "
                f"{p['repeat']:4} {mean:>14} {sd:>9} {p['samples']:5} {'recorded' if p['valid'] else 'EXCLUDED'} / {p.get('capture_mode', 'Sensor + motor')}\n")
        self.table.configure(state="disabled")

    def fit(self):
        if self.persistence_busy or self.capture:
            return
        self.persistence_busy = True
        if not self.app._submit_lifecycle("calibration fit", self._fit_worker):
            self.persistence_busy = False
        self._controls()

    def _fit_worker(self):
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
            else:
                message += model.get("motor_conversion_fit_error", "Motor conversion unavailable: insufficient paired raw-turn points.") + "\n"
            message += "No automatic acceptance or changes to EAST control/calibration. Assess protractor resolution."
            self.app.ui_message_queue.put(("calibration_fit", self, message))
        except Exception as exc:
            self.app.ui_message_queue.put(("calibration_fit", self, "Fit blocked: " + str(exc)))

    def new_session(self):
        if self.persistence_busy or self.capture or self.app is not None and self.app.motion_coordinator.owner is not None:
            return
        self.suspend_positioning("New calibration session")
        self.positioning_reference = None
        self.positioning_events = []
        self.points, self.session_dir, self.session_metadata, self.identity = [], None, None, None
        self.result.configure(text="New session. Previous files remain unchanged.")
        self.path_label.configure(text="Files will be saved when a point is captured.")
        self._show_points()
        self._controls()

    def _positioning_event(self, action, **details):
        event = {"timestamp": datetime.now().astimezone().isoformat(), "action": action, **details}
        self.positioning_events.append(event)
        path = self.session_dir / "calibration_motion_events.json" if self.session_dir is not None else None
        events = list(self.positioning_events)
        def persist():
            if path is not None:
                write_json_atomic(path, {"events": events})
            self.app.state_store.append_event("rotary_calibration_positioning", event)
        self.app._submit_lifecycle("calibration positioning audit", persist)

    def _motor_frame(self):
        app = self.app
        if app is None:
            return None
        try:
            feedback = app.get_feedback()
        except Exception:
            return f"connection-{app.odrive_connection_generation}-epoch-{self.motor_epoch}"
        connection = app.odrive_adapter
        uptime = feedback.system_uptime
        if connection is not self.last_motor_connection:
            self.last_motor_connection = connection
            self.motor_epoch += 1
            self.last_motor_uptime = None
        if self.last_motor_uptime is not None and uptime is not None and uptime < self.last_motor_uptime:
            self.motor_epoch += 1
            self.cancel_capture("Controller uptime reset; motor coordinate frame changed")
            app.reference_manager.invalidate_local("Controller reset detected during rotary calibration")
            self.suspend_positioning("Controller reset detected")
        self.last_motor_uptime = uptime
        return f"connection-{app.odrive_connection_generation}-epoch-{self.motor_epoch}"

    def suspend_positioning(self, reason):
        reference = self.positioning_reference
        if reference is not None:
            reference.suspend()
            self.positioning_angle.delete(0, "end")
            self.positioning_angle.insert(0, f"{reference.angle(reference.last_position_turns):.6g}")
        self.motion_acknowledged.set(False)
        self.motion_status.configure(text=f"Calibration positioning disabled: {reason}. Confirm current position after reconnect.")
        self._controls()

    def reconnect_motor(self):
        if self.app is not None and not self.preview:
            self.app.connect_system()
            self._controls()

    def enable_positioning(self):
        try:
            if self.capture or self.preview or self.closing:
                raise ValueError("Finish capture before enabling calibration motion")
            reference = CalibrationPositioningReference.establish(
                self.app, self.positioning_angle.get(), acknowledged=self.motion_acknowledged.get())
            frame = self._motor_frame()
            self._positioning_event("operator_established_reference",
                nominal_angle_deg=reference.angle(reference.last_position_turns),
                raw_motor_turns=reference.last_position_turns,
                motor_coordinate_frame=frame,
                basis="operator_confirmed_positioning_only_not_machine_zero", machine_zero_changed=False,
                tare_required=False)
            self.positioning_reference = reference
            self.motion_acknowledged.set(False)
            self._update_positioning()
        except Exception as exc:
            self.motion_status.configure(text="Calibration motion blocked: " + str(exc))
        self._controls()

    def move(self, direction):
        if self.app is None or self.preview or self.capture or self.closing:
            return
        self.app._start_calibration_step(self, direction, self.step.get(), self.step_units.get())
        self._controls()

    def _update_positioning(self):
        reference = self.positioning_reference
        if reference is None or not reference.active:
            return
        try:
            feedback = reference.check(self.app)
            if feedback.current_state == 1 and self.app.motion_coordinator.owner is None:
                reference.stationary(self.app, feedback)
                reference.last_position_turns = feedback.position_turns
            self.motion_status.configure(text=(
                f"Raw motor: {feedback.position_turns:.8f} turns | "
                f"estimated positioning angle: {reference.angle(feedback.position_turns):+.3f}°\n"
                "Calibration only; not physical angle or verified machine zero. No load-cell tare required."
            ))
        except Exception as exc:
            self.suspend_positioning(str(exc))

    def _tick(self):
        if self.closed:
            return
        try:
            if getattr(self, "_shared_sensor", None) and not self.closing:
                current = self._shared_sensor.reader
                if current and (self.reader is None or self.reader.source is not current) and not current.stop.is_set():
                    if self.reader:
                        self.cancel_capture("Rotary connection changed")
                        self.reader.unsubscribe()
                    self.reader = current.subscribe()
                    if hasattr(self, "mode"):
                        self.mode.set(current.mode)
            pending = self.pending_capture_cleanup
            if pending is not None and pending["recorder"].done.is_set() and not pending["recorder"].thread.is_alive():
                # All I/O has exited. Releasing a passive observation cannot arm.
                pending["guard"].close()
                self.pending_capture_cleanup = None
                self.status.configure(text="Failed calibration recording retired; raw files remain incomplete. Retry after checking storage.")
            self._motor_frame()
            self._update_positioning()
            self._controls()
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
                if getattr(self, "_shared_sensor", None):
                    self.reader.unsubscribe()
                self.reader = None
                self.sensor, self.latest = None, None
                self._controls()
            fresh = self.latest and time.monotonic() - self.latest[0] <= 1
            if fresh and self.sensor:
                unit = "V/V" if self.sensor["input_mode"] == "Voltage ratio" else "V"
                text = f"Raw sensor: {self.latest[2]:.8f} {unit}"
                if getattr(self, "_shared_sensor", None):
                    accepted = self._shared_sensor.measurement()
                    if accepted["valid"]:
                        text += f"\nAccepted measured angle: {accepted['angle_deg']:+.3f}°"
                self.live_label.configure(text=text)
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
            if self.closing and self.reader is None and not self.persistence_busy and self.pending_capture_cleanup is None:
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
        if self.positioning_reference is not None:
            self.positioning_reference.suspend()
        if self.app is not None and self.app.motion_coordinator.owner == "rotary-calibration-step":
            self.app.stop_logging()
        self.closing = True
        if self._shared_sensor:
            self.cancel_capture("Calibration window closed")
            if self.reader:
                self.reader.unsubscribe()
            self.reader = None
            self.sensor, self.latest = None, None
        else:
            self.disconnect()

    def shutdown(self):
        """Used by EAST shutdown: preserve partial hold and wait for sensor cleanup."""
        self.cancel_capture("EAST shutting down")
        if self.positioning_reference is not None:
            self.positioning_reference.suspend()
        self.closed = True
        if self.reader and self._shared_sensor:
            self.reader.unsubscribe()
            self.reader = None
        if self.reader:
            self.reader.stop.set()
            self.reader.thread.join(timeout=5.0)
            if self.reader.thread.is_alive():
                raise RuntimeError("Sensor reader has not closed; wait before restarting EAST")
