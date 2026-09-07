import os
import sys
import queue
from datetime import datetime
from pathlib import Path

import concurrent.futures

import tkinter as tk
import customtkinter as ctk
from CTkMessagebox import CTkMessagebox

from customtkinter import set_default_color_theme

import csv
import threading
import time
import ctypes

from PIL import Image

from Phidget22.Devices.VoltageRatioInput import VoltageRatioInput

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_POSITION_CONTROL,
    INPUT_MODE_TRAP_TRAJ,
)

# Import PyQtGraph for plotting
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QFont

from east_core import (
    CSV_COLUMNS,
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
    validate_test_parameters,
    write_json_atomic,
)

# Runtime tare state. Fixed calibration values are stored in tester_config.json.
offset = 0
calibrated = False

# Variable to track if plot window is open
plot_window_open = False
# Global variables for plot data
angle_data = []
torque_data = []
plot_window = None
plot_curve = None
plot_timer = None

APP_NAME = "EAST"
APP_VERSION = "1.1.1-gui"

BG = "#f8fafc"
PANEL = "#ffffff"
PANEL_SOFT = "#eef2f7"
TEXT = "#0f172a"
MUTED = "#64748b"
GREEN = "#16a34a"
BLUE = "#2563eb"
RED = "#dc2626"
AMBER = "#d97706"


def resource_path(relative_path):
    """Resolve bundled assets and source-tree assets without relying on the CWD."""
    base = Path(getattr(sys, "_MEIPASS", Path(__file__).resolve().parent))
    return base / relative_path

class MovingAverageFilter:
    def __init__(self, window_size):
        self.window_size = window_size
        self.values = []

    def add_value(self, value):
        self.values.append(value)
        if len(self.values) > self.window_size:
            self.values.pop(0)

    def get_smoothed_value(self):
        if not self.values:
            return None
        return sum(self.values) / len(self.values)


class TestStopped(Exception):
    """Raised inside a motion wait when an operator or acquisition fault stops a run."""

class MyInterface:
    def __init__(self, master):
        self.master = master
        self.master.title(f"{APP_NAME} {APP_VERSION}")

        self.system_config = load_tester_config(resource_path("tester_config.json"))
        self.odrive_controller = None
        self.voltage_ratio_input = None
        self.run_parameters = None
        self.run_metadata = None
        self.metadata_file_name = None
        self.strain_file_name = None
        self.test_started_monotonic = None
        self.test_stop_event = threading.Event()
        self.neutral_stop_event = threading.Event()
        self.finalize_lock = threading.Lock()
        self.run_finalized = True
        self.motion_phase = "idle"
        self.commanded_odrive_velocity = 0.0
        self.commanded_afo_acceleration = 0.0
        self.current_nominal_distance_deg = 0.0
        self.current_expected_constant_speed_span_deg = 0.0
        self.ui_message_queue = queue.Queue()
        self.plot_data_queue = queue.Queue()
        self.header_images = []

        self.strain_test_active = False
        self.strain_data_buffer = []
        self.starting_position = 0
        self.connected_neutral_position = None
        self.current_cycle = 0
        
        # Add continuous movement flags
        self.continuous_movement_active = False
        self.neutral_motion_active = False
        self.movement_direction = None
        self.movement_timer = None
        
        # Add manual mode flag
        self.manual_mode = ctk.BooleanVar(value=False)
        
        self.sample_count = 0
        
        # Moving average filters for both plot and data logging
        filter_window = self.system_config["acquisition"]["moving_average_window_samples"]
        self.angle_filter = MovingAverageFilter(window_size=filter_window)
        self.weight_filter = MovingAverageFilter(window_size=filter_window)
        self.torque_filter = MovingAverageFilter(window_size=filter_window)

        set_default_color_theme("blue")
        ctk.set_appearance_mode("light")

        self.setup_ui()
        self.master.after(50, self._drain_ui_queues)
        self.master.after(20, self._process_qt_events)
        self.master.after(350, self.create_plot_window)

        # Bind window events
        self.master.bind('<Configure>', self.on_window_move)
        self.master.bind('<Unmap>', self.on_window_minimize)
        self.master.bind('<Map>', self.on_window_restore)
        self.master.bind('<Escape>', lambda _event: self.stop_logging())

    def on_window_move(self, event):
        """Window move handler kept for compatibility with existing bindings."""
        return

    def on_window_minimize(self, event):
        """Hide plot window when main window is minimized"""
        if hasattr(self, 'plot_container'):
            self.plot_container.hide()

    def on_window_restore(self, event):
        """Show plot window when main window is restored"""
        if hasattr(self, 'plot_container'):
            self.plot_container.show()
        elif plot_window_open:
            # If plot window was open but container was lost, recreate it
            self.create_plot_window()

    def _process_qt_events(self):
        """Keep the independent PyQtGraph window responsive beside Tk."""
        qt_app = QApplication.instance()
        if qt_app is not None:
            qt_app.processEvents()
        try:
            self.master.after(20, self._process_qt_events)
        except tk.TclError:
            pass

    def create_header_logo(self, parent, candidate_names, fallback_text, column, width):
        """Place a transparent logo directly on the application background."""
        logo_path = next(
            (resource_path(f"images/{name}") for name in candidate_names
             if resource_path(f"images/{name}").exists()),
            None,
        )
        if logo_path is not None:
            image = Image.open(logo_path).convert("RGBA")
            image.thumbnail((width - 30, 50), Image.Resampling.LANCZOS)
            logo_image = ctk.CTkImage(
                light_image=image, dark_image=image, size=image.size
            )
            self.header_images.append(logo_image)
            label = ctk.CTkLabel(
                parent, image=logo_image, text="", fg_color="transparent"
            )
        else:
            label = ctk.CTkLabel(
                parent,
                text=fallback_text,
                font=("Arial", 15, "bold"),
                text_color=TEXT,
                fg_color="transparent",
            )
        label.grid(row=0, column=column, padx=10, pady=4, sticky="nsew")

    def setup_ui(self):
        self.master.configure(fg_color=BG)

        shell = ctk.CTkFrame(self.master, fg_color=BG)
        shell.pack(fill="both", expand=True, padx=18, pady=8)
        shell.grid_columnconfigure(0, weight=1)
        shell.grid_rowconfigure(1, weight=1)

        header = ctk.CTkFrame(shell, fg_color=BG)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        header.grid_columnconfigure(0, weight=1)
        header.grid_columnconfigure(1, weight=2)
        header.grid_columnconfigure(2, weight=1)

        self.create_header_logo(
            header,
            ("epic_lab_logo.png", "epic_lab_logo.jpg", "EPIC_Lab_logo.png", "EPIC Lab Logo.png"),
            "EPIC Lab",
            0,
            220,
        )

        title_panel = ctk.CTkFrame(header, fg_color=BG)
        title_panel.grid(row=0, column=1, sticky="nsew")
        ctk.CTkLabel(
            title_panel, text=APP_NAME, font=("Arial", 40, "bold"), text_color=TEXT
        ).pack()
        ctk.CTkLabel(
            title_panel,
            text=f"{APP_VERSION} | lab laptop layout",
            font=("Arial", 13, "bold"),
            text_color=MUTED,
        ).pack()

        self.create_header_logo(
            header,
            (
                "university_of_sydney_logo.png",
                "university_of_sydney_logo.jpg",
                "usyd_logo.png",
                "USYD_logo.png",
                "University of Sydney Logo.png",
            ),
            "University of Sydney",
            2,
            260,
        )

        body = ctk.CTkFrame(shell, fg_color=BG)
        body.grid(row=1, column=0, sticky="nsew")
        body.grid_columnconfigure((0, 1), weight=1, uniform="main_body")
        body.grid_rowconfigure(0, weight=1)

        controls = ctk.CTkScrollableFrame(
            body, fg_color=PANEL, corner_radius=10, scrollbar_button_color="#cbd5e1"
        )
        controls.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        controls.grid_columnconfigure(0, weight=1)

        self.status_label = ctk.CTkLabel(
            controls,
            text="DISCONNECTED",
            text_color=RED,
            font=("Arial", 13, "bold"),
            anchor="w",
        )
        self.status_label.grid(row=0, column=0, sticky="ew", padx=12, pady=(8, 4))

        inputs_frame = ctk.CTkFrame(controls, fg_color=PANEL_SOFT, corner_radius=8)
        inputs_frame.grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 6))
        inputs_frame.grid_columnconfigure((0, 1), weight=1)

        ctk.CTkLabel(
            inputs_frame,
            text="Test Parameters",
            font=("Arial", 15, "bold"),
            text_color=TEXT,
            anchor="w",
        ).grid(row=0, column=0, columnspan=2, padx=8, pady=(8, 3), sticky="ew")

        fields = (
            ("file_name_input", "File Name Prefix", 0, 0),
            ("cycles_input", "Number of Cycles", 0, 1),
            ("speed_input", "Speed (\N{DEGREE SIGN}/s)", 1, 0),
            ("acceleration_input", "Acceleration (\N{DEGREE SIGN}/s\N{SUPERSCRIPT TWO})", 1, 1),
            ("min_angle_input", "Minimum Angle (\N{DEGREE SIGN})", 2, 0),
            ("max_angle_input", "Maximum Angle (\N{DEGREE SIGN})", 2, 1),
            ("operator_input", "Operator ID", 3, 0),
            ("afo_id_input", "AFO ID", 3, 1),
            ("fixture_id_input", "Fixture ID", 4, 0),
            ("calibration_id_input", "Calibration ID", 4, 1),
        )
        for attribute, label_text, row, column in fields:
            field = ctk.CTkFrame(inputs_frame, fg_color="transparent")
            field.grid(row=row + 1, column=column, padx=6, pady=3, sticky="ew")
            field.grid_columnconfigure(0, weight=1)
            ctk.CTkLabel(
                field,
                text=label_text,
                font=("Arial", 11, "bold"),
                text_color=TEXT,
                anchor="w",
            ).grid(row=0, column=0, pady=(0, 2), sticky="ew")
            entry = ctk.CTkEntry(
                field,
                width=180,
                height=30,
                placeholder_text="",
                corner_radius=6,
            )
            entry.grid(row=1, column=0, sticky="ew")
            setattr(self, attribute, entry)
        self.min_angle_input.bind("<KeyRelease>", self.validate_angle_input)
        self.max_angle_input.bind("<KeyRelease>", self.validate_angle_input)
        for entry in (
            self.cycles_input,
            self.speed_input,
            self.acceleration_input,
            self.min_angle_input,
            self.max_angle_input,
        ):
            entry.bind("<KeyRelease>", self.update_parameter_summary, add="+")

        button_frame = ctk.CTkFrame(controls, fg_color=PANEL, corner_radius=0)
        button_frame.grid(row=2, column=0, sticky="ew", padx=10, pady=(0, 6))
        button_frame.grid_columnconfigure((0, 1), weight=1)
        self.parameter_summary_label = ctk.CTkLabel(
            button_frame,
            text="",
            text_color=MUTED,
            font=("Arial", 11, "bold"),
            anchor="w",
            justify="left",
            wraplength=390,
        )
        self.parameter_summary_label.grid(
            row=0, column=0, columnspan=2, padx=5, pady=(0, 3), sticky="ew"
        )
        button_defs = (
            ("Connect", GREEN, "#15803d", self.connect_system),
            ("Start", BLUE, "#1d4ed8", self.start_strain_test),
            ("Stop", RED, "#b91c1c", self.stop_logging),
            ("Reset Form", AMBER, "#b45309", self.reset_display),
        )
        self.buttons = []
        for index, (label, colour, hover, command) in enumerate(button_defs):
            button = ctk.CTkButton(
                button_frame,
                text=label,
                command=command,
                fg_color=colour,
                hover_color=hover,
                corner_radius=8,
                height=34,
                font=("Arial", 13, "bold"),
            )
            button.grid(row=1 + index // 2, column=index % 2, padx=5, pady=3, sticky="ew")
            self.buttons.append(button)
        self.update_parameter_summary()

        manual_control_frame = ctk.CTkFrame(controls, fg_color=PANEL_SOFT, corner_radius=8)
        manual_control_frame.grid(row=3, column=0, sticky="ew", padx=10, pady=(0, 8))
        manual_control_frame.grid_columnconfigure((0, 1, 2), weight=1)

        self.step_angle_input = ctk.CTkEntry(
            manual_control_frame,
            placeholder_text="Step Angle (0-10 deg)",
            height=30,
            corner_radius=6,
        )
        self.step_angle_input.grid(
            row=0, column=0, columnspan=2, sticky="ew", padx=6, pady=(6, 4)
        )
        self.step_angle_input.bind("<KeyRelease>", self.validate_step_angle)

        self.manual_mode_toggle = ctk.CTkSwitch(
            manual_control_frame,
            text="Manual Mode",
            variable=self.manual_mode,
            onvalue=True,
            offvalue=False,
            command=self.toggle_manual_mode,
        )
        self.manual_mode_toggle.grid(row=0, column=2, padx=6, pady=(6, 4))

        self.left_arrow = ctk.CTkButton(
            manual_control_frame,
            text="<",
            width=60,
            height=34,
            command=self.move_motor_left,
            fg_color=MUTED,
            hover_color="#475569",
            corner_radius=8,
            font=("Arial", 20, "bold"),
        )
        self.left_arrow.grid(row=1, column=0, padx=6, pady=(0, 4), sticky="ew")
        self.left_arrow.bind(
            "<ButtonPress-1>", lambda _event: self.begin_continuous_movement("left")
        )
        self.left_arrow.bind("<ButtonRelease-1>", lambda _event: self.stop_continuous_movement())

        self.right_arrow = ctk.CTkButton(
            manual_control_frame,
            text=">",
            width=60,
            height=34,
            command=self.move_motor_right,
            fg_color=MUTED,
            hover_color="#475569",
            corner_radius=8,
            font=("Arial", 20, "bold"),
        )
        self.right_arrow.grid(row=1, column=1, padx=6, pady=(0, 4), sticky="ew")
        self.right_arrow.bind(
            "<ButtonPress-1>", lambda _event: self.begin_continuous_movement("right")
        )
        self.right_arrow.bind("<ButtonRelease-1>", lambda _event: self.stop_continuous_movement())

        self.continuous_mode = ctk.BooleanVar(value=False)
        self.mode_toggle = ctk.CTkSwitch(
            manual_control_frame,
            text="Continuous Mode",
            variable=self.continuous_mode,
            onvalue=True,
            offvalue=False,
        )
        self.mode_toggle.grid(row=1, column=2, padx=6, pady=(0, 4))

        self.neutral_button = ctk.CTkButton(
            manual_control_frame,
            text="Return to 90 deg (0 turns)",
            command=self.return_to_neutral,
            fg_color="#0f766e",
            hover_color="#115e59",
            corner_radius=8,
            height=34,
            font=("Arial", 13, "bold"),
        )
        self.neutral_button.grid(
            row=2, column=0, columnspan=3, padx=6, pady=(0, 6), sticky="ew"
        )

        for widget in (
            self.left_arrow,
            self.right_arrow,
            self.step_angle_input,
            self.mode_toggle,
            self.neutral_button,
        ):
            widget.configure(state="disabled")
        self.buttons[1].configure(state="disabled")

        terminal_frame = ctk.CTkFrame(body, fg_color=PANEL, corner_radius=10)
        terminal_frame.grid(row=0, column=1, sticky="nsew")
        terminal_frame.grid_columnconfigure(0, weight=1)
        terminal_frame.grid_rowconfigure(1, weight=1)
        ctk.CTkLabel(
            terminal_frame,
            text="Session Terminal",
            font=("Arial", 18, "bold"),
            text_color=TEXT,
            anchor="w",
        ).grid(row=0, column=0, sticky="ew", padx=14, pady=(10, 5))
        self.terminal = ctk.CTkTextbox(
            terminal_frame,
            height=220,
            fg_color=BG,
            text_color=TEXT,
            border_width=1,
            border_color="#cbd5e1",
            corner_radius=8,
            wrap="word",
        )
        self.terminal.grid(row=1, column=0, sticky="nsew", padx=14, pady=(0, 10))

        # Handle window closing event
        self.master.protocol("WM_DELETE_WINDOW", self.on_close)
        self.update_terminal(f"{APP_NAME} {APP_VERSION}\nRunning from: {Path(__file__).resolve()}\n")
 
    def find_odrive_with_timeout(serial_number, timeout=5):
        start_time = time.time()
        while time.time() - start_time < timeout:
            try:
                odrv = odrive.find_any(serial_number=serial_number, timeout=1)  # Short timeout for each attempt
                if odrv:
                    return odrv
            except Exception:
                pass
            time.sleep(0.1)  # Short delay between attempts
        return None


    def get_axis(self):
        if self.odrive_controller is None:
            raise RuntimeError("ODrive is not connected")
        axis_number = int(self.system_config["hardware"]["odrive_axis"])
        return getattr(self.odrive_controller, f"axis{axis_number}")

    def set_status(self, text, colour=TEXT):
        if threading.current_thread() is not threading.main_thread():
            self.ui_message_queue.put(("status", text, colour))
            return
        self.status_label.configure(text=text, text_color=colour)

    def update_parameter_summary(self, _event=None):
        def entered(entry):
            return entry.get().strip() or "?"

        cycles = entered(self.cycles_input)
        speed = entered(self.speed_input)
        acceleration = entered(self.acceleration_input)
        minimum = entered(self.min_angle_input)
        maximum = entered(self.max_angle_input)
        self.parameter_summary_label.configure(
            text=(
                f"Commanded: {cycles} cycles | {speed}\N{DEGREE SIGN}/s | "
                f"{acceleration}\N{DEGREE SIGN}/s\N{SUPERSCRIPT TWO} | "
                f"-{minimum}\N{DEGREE SIGN} to +{maximum}\N{DEGREE SIGN}"
            )
        )

    def collect_test_parameters(self):
        return validate_test_parameters({
            "file_prefix": self.file_name_input.get(),
            "operator": self.operator_input.get(),
            "afo_id": self.afo_id_input.get(),
            "fixture_id": self.fixture_id_input.get(),
            "calibration_id": self.calibration_id_input.get(),
            "cycles": self.cycles_input.get(),
            "speed_deg_s": self.speed_input.get(),
            "acceleration_deg_s2": self.acceleration_input.get(),
            "min_angle_deg": self.min_angle_input.get(),
            "max_angle_deg": self.max_angle_input.get(),
        }, self.system_config)

    def set_test_inputs_state(self, state):
        for entry in (
            self.file_name_input, self.cycles_input, self.speed_input,
            self.acceleration_input, self.min_angle_input, self.max_angle_input,
            self.operator_input, self.afo_id_input, self.fixture_id_input,
            self.calibration_id_input,
        ):
            entry.configure(state=state)

    def configure_trajectory(self, speed_deg_s, acceleration_deg_s2):
        axis = self.get_axis()
        motion = self.system_config["motion"]
        trajectory_velocity = afo_speed_to_odrive_turns_s(speed_deg_s, self.system_config)
        trajectory_acceleration = afo_acceleration_to_odrive_turns_s2(
            acceleration_deg_s2, self.system_config
        )
        controller_limit = trajectory_velocity * float(motion["controller_velocity_safety_multiplier"])
        axis.controller.config.control_mode = CONTROL_MODE_POSITION_CONTROL
        axis.controller.config.input_mode = INPUT_MODE_TRAP_TRAJ
        axis.trap_traj.config.vel_limit = trajectory_velocity
        axis.trap_traj.config.accel_limit = trajectory_acceleration
        axis.trap_traj.config.decel_limit = trajectory_acceleration
        axis.controller.config.vel_limit = controller_limit
        self.commanded_odrive_velocity = trajectory_velocity
        self.commanded_afo_acceleration = float(acceleration_deg_s2)

    def odrive_configuration_snapshot(self):
        axis = self.get_axis()
        def read(value, default=None):
            try:
                return value()
            except Exception:
                return default
        return {
            "axis": int(self.system_config["hardware"]["odrive_axis"]),
            "axis_active_errors": read(lambda: int(axis.active_errors)),
            "control_mode": read(lambda: int(axis.controller.config.control_mode)),
            "input_mode": read(lambda: int(axis.controller.config.input_mode)),
            "controller_velocity_limit_turns_s": read(lambda: float(axis.controller.config.vel_limit)),
            "trajectory_velocity_limit_turns_s": read(lambda: float(axis.trap_traj.config.vel_limit)),
            "trajectory_acceleration_limit_turns_s2": read(lambda: float(axis.trap_traj.config.accel_limit)),
            "trajectory_deceleration_limit_turns_s2": read(lambda: float(axis.trap_traj.config.decel_limit)),
            "position_gain": read(lambda: float(axis.controller.config.pos_gain)),
            "velocity_gain": read(lambda: float(axis.controller.config.vel_gain)),
            "velocity_integrator_gain": read(lambda: float(axis.controller.config.vel_integrator_gain)),
        }

    def safe_idle_motor(self, reason=None):
        self.stop_continuous_movement()
        if self.odrive_controller is None:
            return
        try:
            self.get_axis().requested_state = AXIS_STATE_IDLE
            self.motion_phase = "idle"
            if reason:
                self.update_terminal(f"Motor set to idle: {reason}\n")
        except Exception as exc:
            self.update_terminal(f"Unable to confirm ODrive idle state: {exc}\n")

    def enter_closed_loop(self):
        axis = self.get_axis()
        axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            if int(axis.active_errors):
                raise RuntimeError(f"ODrive active errors entering closed loop: {int(axis.active_errors)}")
            if int(axis.current_state) == int(AXIS_STATE_CLOSED_LOOP_CONTROL):
                return axis
            time.sleep(0.02)
        raise TimeoutError("ODrive did not enter closed-loop control within 2 seconds")

    def connect_system(self):
        if self.strain_test_active:
            self.update_terminal("Cannot reconnect while a strain test is active.\n")
            return
        self.clear_terminal()
        if self.odrive_controller is not None:
            self.safe_idle_motor("reconnect")
        self.buttons[1].configure(state="disabled")
        self.neutral_button.configure(state="disabled")
        self.connected_neutral_position = None
        self.set_status("CONNECTING", AMBER)
        serial_number = self.system_config["hardware"]["odrive_serial_number"]
        timeout_duration = self.system_config["hardware"]["odrive_connection_timeout_s"]

        try:
            with concurrent.futures.ThreadPoolExecutor() as executor:
                future = executor.submit(
                    MyInterface.find_odrive_with_timeout, serial_number, timeout_duration
                )
                self.odrive_controller = future.result(timeout=timeout_duration + 0.5)
            if self.odrive_controller is None:
                raise TimeoutError(f"ODrive {serial_number} was not found")

            axis = self.get_axis()
            prior_errors = int(axis.active_errors)
            if prior_errors:
                self.update_terminal(f"ODrive active errors before clear: {prior_errors}\n")
            self.odrive_controller.clear_errors()

            controller = self.system_config["controller"]
            if controller["apply_controller_gains"]:
                axis.controller.config.pos_gain = controller["position_gain"]
                axis.controller.config.vel_gain = controller["velocity_gain"]
                axis.controller.config.vel_integrator_gain = controller["velocity_integrator_gain"]

            motion = self.system_config["motion"]
            self.connected_neutral_position = float(motion["neutral_position_turns"])
            self.configure_trajectory(
                motion["manual_speed_deg_s"], motion["manual_acceleration_deg_s2"]
            )
            self.starting_position = axis.pos_vel_mapper.pos_rel
            self.safe_idle_motor()

            self.update_terminal(
                f"Connected to ODrive S1\nSerial number: {serial_number}\n"
                f"Axis errors after clear: {int(axis.active_errors)}\n"
                f"Fixed 90 degree neutral: {self.connected_neutral_position:.8f} turns.\n"
            )
            self.set_status("CONNECTED / IDLE", GREEN)
            if self.manual_mode.get():
                self.toggle_manual_mode()
            else:
                self.buttons[1].configure(state="normal")
        except (concurrent.futures.TimeoutError, TimeoutError) as exc:
            self.odrive_controller = None
            self.update_terminal(f"Connection timed out: {exc}\n")
            self.set_status("DISCONNECTED", RED)
        except Exception as exc:
            self.safe_idle_motor("connection/configuration error")
            self.odrive_controller = None
            self.update_terminal(f"Error connecting to ODrive: {exc}\n")
            self.set_status("ERROR", RED)

    def disconnect_odrive(self):
        """Disconnect from ODrive safely"""
        try:
            if self.odrive_controller:
                self.neutral_stop_event.set()
                self.safe_idle_motor("disconnect")
                self.odrive_controller = None
                self.connected_neutral_position = None
                self.buttons[1].configure(state="disabled")
                self.neutral_button.configure(state="disabled")
                self.set_status("DISCONNECTED", RED)
                self.update_terminal("ODrive disconnected and set to idle state\n")
        except Exception as e:
            self.update_terminal(f"Error disconnecting ODrive: {e}\n")

    def stop_logging(self):
        was_active = self.strain_test_active
        self.test_stop_event.set()
        self.neutral_stop_event.set()
        self.strain_test_active = False
        self.safe_idle_motor("operator stop")
        self.set_status("STOPPED / IDLE", AMBER)
        if was_active:
            self.update_terminal("Test stop requested; the data file will be finalized as aborted.\n")
        else:
            self.update_terminal("Motor is idle.\n")
            


    def reset_display(self):
        # Stop logging
        self.stop_logging()
        
        # Stop strain test if active
        if self.strain_test_active:
            self.stop_strain_test()
        
        # Clear terminal
        self.clear_terminal()

        for entry in (
            self.file_name_input, self.cycles_input, self.speed_input,
            self.acceleration_input, self.min_angle_input, self.max_angle_input,
            self.operator_input, self.afo_id_input, self.fixture_id_input,
            self.calibration_id_input,
        ):
            entry.delete(0, ctk.END)
        self.update_parameter_summary()

        self.safe_idle_motor("reset")
        if self.odrive_controller and not self.manual_mode.get():
            self.buttons[1].configure(state="normal")
            self.set_status("CONNECTED / IDLE", GREEN)


    def clear_terminal(self):
        self.terminal.delete(1.0, ctk.END)
        self.terminal.update()


    def update_terminal(self, message):
        if threading.current_thread() is not threading.main_thread():
            self.ui_message_queue.put(("terminal", message))
            return
        self.terminal.insert(ctk.END, message)
        self.terminal.see(ctk.END)  # Scroll to the end of the text

    def _drain_ui_queues(self):
        try:
            while True:
                item = self.ui_message_queue.get_nowait()
                if item[0] == "terminal":
                    self.update_terminal(item[1])
                elif item[0] == "status":
                    self.set_status(item[1], item[2])
                elif item[0] == "inputs":
                    self.set_test_inputs_state(item[1])
                elif item[0] == "run_buttons":
                    self.buttons[0].configure(state=item[1])
                    self.buttons[1].configure(
                        state=(
                            item[1]
                            if self.odrive_controller is not None
                            and self.connected_neutral_position is not None
                            else "disabled"
                        )
                    )
                    self.manual_mode_toggle.configure(state=item[1])
                elif item[0] == "neutral_finished":
                    self.neutral_motion_active = False
                    enabled = (
                        self.manual_mode.get()
                        and self.odrive_controller is not None
                        and self.connected_neutral_position is not None
                        and not self.strain_test_active
                    )
                    state = "normal" if enabled else "disabled"
                    self.left_arrow.configure(state=state)
                    self.right_arrow.configure(state=state)
                    self.neutral_button.configure(state=state)
        except queue.Empty:
            pass

        try:
            while True:
                angle, torque = self.plot_data_queue.get_nowait()
                self.update_plot_data(angle, torque)
        except queue.Empty:
            pass

        try:
            self.master.after(50, self._drain_ui_queues)
        except tk.TclError:
            pass

    def on_close(self):
        """Handle application closing"""
        # Calculate center position for the message box
        main_window_x = self.master.winfo_x()
        main_window_y = self.master.winfo_y()
        main_window_width = self.master.winfo_width()
        main_window_height = self.master.winfo_height()
        
        # Center coordinates
        center_x = main_window_x + (main_window_width // 2)
        center_y = main_window_y + (main_window_height // 2)
        
        msg = CTkMessagebox(
            title="Quit",
            message="Do you want to quit?",
            icon="question",
            option_1="Cancel",
            option_2="Yes",
            sound=True,
            button_hover_color="grey",  # Grey on hover
            button_width=120,  # Make buttons wider
            font=("Arial", 14),  # Larger font for text
            icon_size=(40, 40),  # Larger icon
            button_height=35,  # Taller buttons
            border_width=2,  # Add border for better visibility
            border_color="#444444",  # Dark grey border
            justify="center"  # Center the message text
        )
        
        # Position the message box (need to update after it's created)
        msg_width = 20  # Increased width for better button spacing
        msg_height = 200  # Approximate height of message box
        msg.geometry(f"+{center_x - msg_width//2}+{center_y - msg_height//2}")
        
        
        response = msg.get()
        
        if response == "Yes":
            try:
                # Stop any active processes first
                if self.strain_test_active:
                    self.stop_logging()

                self.neutral_stop_event.set()
                for thread_name in ("strain_thread", "data_collection_thread", "neutral_thread"):
                    thread = getattr(self, thread_name, None)
                    if thread and thread.is_alive() and thread is not threading.current_thread():
                        thread.join(timeout=2.0)
                
                # Close the plot window safely
                self.close_plot_window()
                
                # Disconnect from ODrive if connected
                if hasattr(self, 'odrive_controller') and self.odrive_controller:
                    try:
                        self.disconnect_odrive()
                    except Exception:
                        pass  # Ignore any errors during ODrive disconnection
                
                # Disconnect from Phidget if connected
                if hasattr(self, 'voltage_ratio_input') and self.voltage_ratio_input:
                    try:
                        self.voltage_ratio_input.close()
                    except Exception:
                        pass
                
                # Destroy the main window
                self.master.quit()
                self.master.destroy()
                
            except Exception as e:
                print(f"Error during shutdown: {e}")
                # Force quit if there's an error
                self.master.quit()
                self.master.destroy()

    def start_strain_test(self):
        """Start the strain test with the current motor settings"""
        # Clear plot data if plot window is open
        global angle_data, torque_data, plot_curve, plot_window
        if plot_window_open:
            angle_data = []
            torque_data = []
            if plot_curve is not None:
                plot_curve.setData(angle_data, torque_data)
                # Reset plot axes
                plot_window.setXRange(0, 1)  # Reset x-axis
                plot_window.setYRange(0, 1)  # Reset y-axis
                plot_window.enableAutoRange()  # Enable auto-ranging for both axes
        self.create_plot_window()

        if self.odrive_controller is None:
            self.update_terminal("No serial connection established. Please connect ODrive first.\n")
            return

        if self.connected_neutral_position is None:
            self.update_terminal(
                "The fixed 90 degree neutral reference is unavailable. Check tester_config.json.\n"
            )
            return

        axis = self.get_axis()
        tolerance_turns = afo_degrees_to_odrive_turns(
            self.system_config["motion"]["position_tolerance_deg"], self.system_config
        )
        if abs(axis.pos_vel_mapper.pos_rel - self.connected_neutral_position) > tolerance_turns:
            self.update_terminal(
                "Fixture is not at the fixed 90 degree neutral (0 turns). Enable manual mode "
                "and press 'Return to 90 deg (0 turns)' before starting.\n"
            )
            return

        if self.manual_mode.get():
            self.update_terminal("Disable manual mode before starting a strain test.\n")
            return
        
        if self.strain_test_active:
            self.update_terminal("Strain test already active\n")
            return
        
        try:
            parameters = self.collect_test_parameters()
        except ValueError as exc:
            CTkMessagebox(title="Input Error", message=str(exc))
            return

        confirmation = CTkMessagebox(
            title="Confirm Test",
            message=(
                f"AFO: {parameters.afo_id}\n"
                f"Commanded range: -{parameters.min_angle_deg:g}\N{DEGREE SIGN} to "
                f"+{parameters.max_angle_deg:g}\N{DEGREE SIGN}\n"
                f"Commanded speed: {parameters.commanded_afo_speed_deg_s:g}\N{DEGREE SIGN}/s\n"
                f"Commanded acceleration: {parameters.commanded_afo_acceleration_deg_s2:g}"
                f"\N{DEGREE SIGN}/s\N{SUPERSCRIPT TWO}\n"
                f"Expected constant-speed span: "
                f"{constant_speed_span_deg(parameters.min_angle_deg + parameters.max_angle_deg, parameters.commanded_afo_speed_deg_s, parameters.commanded_afo_acceleration_deg_s2):.2f}\N{DEGREE SIGN}\n"
                f"Commanded cycles: {parameters.cycles}\n\n"
                "Confirm the fixture is clear and the physical E-stop is accessible."
            ),
            icon="question", option_1="Cancel", option_2="Start"
        )
        if confirmation.get() != "Start":
            return

        self.set_test_inputs_state("disabled")
        self.buttons[0].configure(state="disabled")
        self.buttons[1].configure(state="disabled")
        self.manual_mode_toggle.configure(state="disabled")
        self.run_metadata = None
        self.metadata_file_name = None
        self.strain_file_name = None
        self.test_started_monotonic = time.monotonic()
        try:
            axis = self.get_axis()
            if int(axis.active_errors):
                raise RuntimeError(f"ODrive has active errors: {int(axis.active_errors)}")

            self.voltage_ratio_input = VoltageRatioInput()
            hardware = self.system_config["hardware"]
            if hardware["phidget_serial_number"] is not None:
                self.voltage_ratio_input.setDeviceSerialNumber(hardware["phidget_serial_number"])
            self.voltage_ratio_input.setChannel(hardware["phidget_channel"])
            self.voltage_ratio_input.openWaitForAttachment(hardware["phidget_attachment_timeout_ms"])
            self.voltage_ratio_input.setDataInterval(
                self.system_config["acquisition"]["sample_interval_ms"]
            )
            self.tare_scale()

            self.run_parameters = parameters
            self.configure_trajectory(
                parameters.commanded_afo_speed_deg_s,
                parameters.commanded_afo_acceleration_deg_s2,
            )
            self.starting_position = self.connected_neutral_position
            app_base = Path(sys.executable).resolve().parent if getattr(sys, "frozen", False) else Path(__file__).resolve().parent
            csv_path, metadata_path = create_run_paths(parameters, self.system_config, app_base)
            self.strain_file_name = str(csv_path)
            self.metadata_file_name = str(metadata_path)
            with open(self.strain_file_name, mode="x", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(CSV_COLUMNS)

            self.run_metadata = make_run_metadata(
                parameters, self.system_config, offset, csv_path,
                self.odrive_configuration_snapshot(),
            )
            self.run_metadata["software"]["gui_version"] = APP_VERSION
            self.run_metadata["neutral_reference"] = {
                "definition": "fixed configured 90 degree position",
                "odrive_pos_rel_turns": self.connected_neutral_position,
                "reference_status": self.system_config["motion"]["neutral_reference_status"],
            }
            try:
                self.run_metadata["hardware"]["connected_phidget_serial_number"] = (
                    self.voltage_ratio_input.getDeviceSerialNumber()
                )
            except Exception:
                self.run_metadata["hardware"]["connected_phidget_serial_number"] = "unavailable"
            write_json_atomic(metadata_path, self.run_metadata)
            self.run_finalized = False

            self.strain_data_buffer = []
            self.plot_update_counter = 0
            self.plot_update_interval = 5
            self.sample_count = 0
            self.current_cycle = 0
            self.completed_cycles = 0
            self.motion_phase = "preparing"
            filter_window = self.system_config["acquisition"]["moving_average_window_samples"]
            self.angle_filter = MovingAverageFilter(filter_window)
            self.weight_filter = MovingAverageFilter(filter_window)
            self.torque_filter = MovingAverageFilter(filter_window)
            self.test_started_monotonic = time.monotonic()
            self.test_stop_event.clear()
            self.acquisition_error = None
            self.strain_test_active = True
            self.data_collection_thread = threading.Thread(
                target=self.continuous_strain_read, name="strain-acquisition"
            )
            self.data_collection_thread.start()
            self.strain_thread = threading.Thread(
                target=self.strain_test_control, name="strain-motion"
            )
            self.strain_thread.start()
            self.set_status("TEST RUNNING", GREEN)
            self.update_terminal(
                f"Strain test started. Data: {self.strain_file_name}\n"
                f"Commanded ODrive trajectory speed: {self.commanded_odrive_velocity:.6f} turns/s\n"
                f"Commanded AFO acceleration: {self.commanded_afo_acceleration:g}"
                f"\N{DEGREE SIGN}/s\N{SUPERSCRIPT TWO}\n"
            )
        except Exception as exc:
            self.strain_test_active = False
            self.test_stop_event.set()
            self.safe_idle_motor("test initialization error")
            data_thread = getattr(self, "data_collection_thread", None)
            if data_thread and data_thread.is_alive():
                data_thread.join(timeout=2.0)
            if self.voltage_ratio_input is not None:
                try:
                    self.voltage_ratio_input.close()
                except Exception:
                    pass
                self.voltage_ratio_input = None
            self.set_status("ERROR / IDLE", RED)
            self.update_terminal(f"Error initializing strain test: {exc}\n")
            self.set_test_inputs_state("normal")
            self.buttons[0].configure(state="normal")
            self.manual_mode_toggle.configure(state="normal")
            if self.odrive_controller:
                self.buttons[1].configure(
                    state="normal" if self.connected_neutral_position is not None else "disabled"
                )
            if self.run_metadata is not None and not self.run_finalized:
                self.completed_cycles = 0
                self.finalize_run("error", str(exc))
    


    def get_current_weight(self):
        """Get the current weight reading from the scale in grams"""
        if not calibrated:
            return 0.0
        voltage_ratio = self.voltage_ratio_input.getVoltageRatio()
        _, weight_grams, _ = calculate_load(voltage_ratio, offset, self.system_config)
        return weight_grams

    def tare_scale(self):
        """Tare the Phidget scale"""
        global offset, calibrated
        num_samples = int(self.system_config["load_cell"]["tare_samples"])
        
        self.update_terminal("Taring scale...\n")
        offset = 0  # Reset offset before taking new samples
        for _ in range(num_samples):
            offset += self.voltage_ratio_input.getVoltageRatio()
            time.sleep(self.voltage_ratio_input.getDataInterval() / 1000.0)
        
        offset /= num_samples
        calibrated = True
        self.update_terminal(f"Scale tared. Offset: {offset}\n")
        current_weight = self.get_current_weight()
        self.update_terminal(f"Current weight: {current_weight:.2f} grams\n")


    
    def log_strain_data(self, voltage_ratio, cycle):
        """Log strain data to buffer"""
        global calibrated, offset
        
        try:
            if calibrated:
                wall_time = datetime.now().astimezone()
                monotonic_time = time.monotonic()
                axis = self.get_axis()
                current_pos_turns = axis.pos_vel_mapper.pos_rel
                relative_turns = current_pos_turns - self.starting_position
                relative_angle = odrive_turns_to_afo_degrees(relative_turns, self.system_config)
                velocity_turns_s = axis.pos_vel_mapper.vel
                afo_velocity_deg_s = odrive_turns_to_afo_degrees(
                    velocity_turns_s, self.system_config
                )
                mass_kg, raw_weight_grams, force_n = calculate_load(
                    voltage_ratio, offset, self.system_config
                )
                raw_torque_nm = calculate_torque_nm(force_n, relative_angle, self.system_config)
                
                # Apply moving average filter to angle, weight, and torque
                self.angle_filter.add_value(relative_angle)
                self.weight_filter.add_value(raw_weight_grams)
                self.torque_filter.add_value(raw_torque_nm)
                
                # Get smoothed values
                avg_angle = self.angle_filter.get_smoothed_value()
                avg_weight = self.weight_filter.get_smoothed_value()
                avg_torque = self.torque_filter.get_smoothed_value()
                
                if avg_angle is None or avg_weight is None or avg_torque is None:
                    return
                elapsed = monotonic_time - self.test_started_monotonic
                data_row = [
                    wall_time.isoformat(timespec="milliseconds"),
                    f"{elapsed:.6f}",
                    self.sample_count,
                    cycle,
                    self.motion_phase,
                    f"{self.run_parameters.commanded_afo_speed_deg_s:.6f}",
                    f"{self.run_parameters.commanded_afo_acceleration_deg_s2:.6f}",
                    f"{-self.run_parameters.min_angle_deg:.6f}",
                    f"{self.run_parameters.max_angle_deg:.6f}",
                    self.run_parameters.cycles,
                    self.run_parameters.file_prefix,
                    self.run_parameters.operator,
                    self.run_parameters.afo_id,
                    self.run_parameters.fixture_id,
                    self.run_parameters.calibration_id,
                    f"{self.commanded_odrive_velocity:.6f}",
                    f"{self.current_nominal_distance_deg:.6f}",
                    f"{self.current_expected_constant_speed_span_deg:.6f}",
                    f"{current_pos_turns:.8f}",
                    f"{relative_angle:.6f}",
                    f"{avg_angle:.6f}",
                    f"{voltage_ratio:.12g}",
                    f"{offset:.12g}",
                    f"{mass_kg:.9f}",
                    f"{raw_weight_grams:.6f}",
                    f"{avg_weight:.6f}",
                    f"{force_n:.6f}",
                    f"{raw_torque_nm:.6f}",
                    f"{avg_torque:.6f}",
                    f"{velocity_turns_s:.8f}",
                    f"{afo_velocity_deg_s:.6f}",
                    int(axis.active_errors),
                ]
                self.strain_data_buffer.append(data_row)
                if len(self.strain_data_buffer) >= self.system_config["acquisition"]["csv_flush_rows"]:
                    with open(self.strain_file_name, mode="a", newline="", encoding="utf-8") as file:
                        writer = csv.writer(file)
                        writer.writerows(self.strain_data_buffer)
                    self.strain_data_buffer = []
                
                # Update plot data if plot window is open (using moving average values)
                if plot_window_open:
                    self.plot_update_counter += 1
                    if self.plot_update_counter >= self.plot_update_interval:
                        self.plot_data_queue.put((relative_angle, avg_torque))
                        self.plot_update_counter = 0
                
                # Update terminal less frequently (every 20th sample)
                if self.sample_count % 125 == 0:
                    self.update_terminal(f"Raw Weight: {raw_weight_grams:.2f} g, Avg Weight: {avg_weight:.2f} g\n")
                    self.update_terminal(
                        f"ODrive-derived AFO angle: {relative_angle:.4f} deg, "
                        f"moving average: {avg_angle:.4f} deg\n"
                    )
                    self.update_terminal(f"Raw Torque: {raw_torque_nm:.4f} Nm, Avg Torque: {avg_torque:.4f} Nm\n")
                
                self.sample_count += 1
                
            else:
                self.update_terminal("Phidget is not calibrated yet!\n")
                
        except Exception as e:
            self.update_terminal(f"Error logging strain data: {str(e)}\n")
    
    def strain_test_control(self):
        """Control the motor and log strain data during the test"""
        final_status = "aborted"
        final_error = None
        try:
            if self.test_stop_event.is_set():
                raise TestStopped()
            axis = self.enter_closed_loop()
            self.update_terminal(f"Starting position (zero point): {self.starting_position} turns\n")
            parameters = self.run_parameters
            min_turns = afo_degrees_to_odrive_turns(parameters.min_angle_deg, self.system_config)
            max_turns = afo_degrees_to_odrive_turns(parameters.max_angle_deg, self.system_config)
            absolute_min = self.starting_position - min_turns
            absolute_max = self.starting_position + max_turns

            self.update_terminal(
                f"Commanded AFO range: -{parameters.min_angle_deg:g}\N{DEGREE SIGN} to "
                f"+{parameters.max_angle_deg:g}\N{DEGREE SIGN}\n"
                f"Commanded AFO speed: {parameters.commanded_afo_speed_deg_s:g}"
                f"\N{DEGREE SIGN}/s\n"
                f"Commanded AFO acceleration: {parameters.commanded_afo_acceleration_deg_s2:g}"
                f"\N{DEGREE SIGN}/s\N{SUPERSCRIPT TWO}\n"
            )
            for cycle in range(1, parameters.cycles + 1):
                self.current_cycle = cycle
                self.command_position_and_wait(
                    absolute_max, "moving_to_max", parameters.max_angle_deg
                )
                self.command_position_and_wait(
                    absolute_min, "moving_to_min", parameters.max_angle_deg + parameters.min_angle_deg
                )
                self.completed_cycles = cycle
                self.update_terminal(f"Completed cycle {cycle}/{parameters.cycles}\n")

            self.current_cycle = parameters.cycles
            self.command_position_and_wait(
                self.starting_position, "returning_to_zero", parameters.min_angle_deg
            )
            final_status = "completed"
            self.update_terminal(f"Strain test completed. Data saved to {self.strain_file_name}\n")
        except TestStopped:
            final_status = "aborted"
            final_error = self.acquisition_error or "operator stop"
            self.update_terminal("Strain test aborted.\n")
        except Exception as exc:
            final_status = "error"
            final_error = str(exc)
            self.update_terminal(f"Error during strain test: {exc}\n")
        finally:
            self.strain_test_active = False
            self.test_stop_event.set()
            if hasattr(self, "data_collection_thread"):
                self.data_collection_thread.join(timeout=2.0)
            self.safe_idle_motor(final_status)
            try:
                if self.voltage_ratio_input is not None:
                    self.voltage_ratio_input.close()
            except Exception:
                pass
            self.voltage_ratio_input = None
            self.finalize_run(final_status, final_error)

    def command_position_and_wait(self, target_turns, phase, nominal_distance_deg):
        if self.test_stop_event.is_set():
            raise TestStopped()
        axis = self.get_axis()
        self.motion_phase = phase
        self.current_nominal_distance_deg = float(nominal_distance_deg)
        self.current_expected_constant_speed_span_deg = constant_speed_span_deg(
            nominal_distance_deg,
            self.run_parameters.commanded_afo_speed_deg_s,
            self.run_parameters.commanded_afo_acceleration_deg_s2,
        )
        axis.controller.input_pos = target_turns
        timeout_s = motion_timeout_seconds(
            nominal_distance_deg,
            self.run_parameters.commanded_afo_speed_deg_s,
            self.run_parameters.commanded_afo_acceleration_deg_s2,
            self.system_config,
        )
        tolerance_turns = afo_degrees_to_odrive_turns(
            self.system_config["motion"]["position_tolerance_deg"], self.system_config
        )
        deadline = time.monotonic() + timeout_s
        while abs(axis.pos_vel_mapper.pos_rel - target_turns) > tolerance_turns:
            if self.test_stop_event.is_set():
                raise TestStopped()
            active_errors = int(axis.active_errors)
            if active_errors:
                raise RuntimeError(f"ODrive active errors during {phase}: {active_errors}")
            if time.monotonic() >= deadline:
                raise TimeoutError(f"Motion timed out during {phase} after {timeout_s:.1f} s")
            time.sleep(0.01)
    
    def continuous_strain_read(self):
        """Continuously read strain data while the test is running"""
        interval_s = self.system_config["acquisition"]["sample_interval_ms"] / 1000.0
        next_sample = time.monotonic()
        try:
            while not self.test_stop_event.is_set():
                voltage_ratio = self.voltage_ratio_input.getVoltageRatio()
                self.log_strain_data(voltage_ratio, self.current_cycle)
                next_sample += interval_s
                time.sleep(max(0.0, next_sample - time.monotonic()))
        except Exception as exc:
            self.acquisition_error = f"data acquisition error: {exc}"
            self.update_terminal(f"Error reading strain data: {exc}\n")
            self.test_stop_event.set()
        finally:
            if self.strain_data_buffer:
                with open(self.strain_file_name, mode="a", newline="", encoding="utf-8") as file:
                    writer = csv.writer(file)
                    writer.writerows(self.strain_data_buffer)
                self.strain_data_buffer = []

    def finalize_run(self, status, error=None):
        with self.finalize_lock:
            if self.run_finalized or self.run_metadata is None:
                return
            self.run_finalized = True
            self.run_metadata["run_status"] = status
            self.run_metadata["completed_at"] = datetime.now().astimezone().isoformat(timespec="milliseconds")
            self.run_metadata["sample_count"] = self.sample_count
            self.run_metadata["completed_cycles"] = self.completed_cycles
            self.run_metadata["error"] = error
            duration_s = max(0.0, time.monotonic() - self.test_started_monotonic)
            self.run_metadata["duration_s"] = duration_s
            self.run_metadata["achieved_average_sample_rate_hz"] = (
                self.sample_count / duration_s if duration_s > 0 else 0.0
            )
            try:
                self.run_metadata["odrive_final_configuration"] = self.odrive_configuration_snapshot()
            except Exception as exc:
                self.run_metadata["odrive_final_configuration_error"] = str(exc)
            try:
                write_json_atomic(Path(self.metadata_file_name), self.run_metadata)
            except Exception as exc:
                self.update_terminal(f"Failed to finalize metadata: {exc}\n")
            colour = GREEN if status == "completed" else AMBER if status == "aborted" else RED
            self.set_status(f"{status.upper()} / IDLE", colour)
            self.ui_message_queue.put(("inputs", "normal"))
            self.ui_message_queue.put(("run_buttons", "normal"))

    def stop_strain_test(self):
        """Stop the strain test"""
        if self.strain_test_active:
            self.test_stop_event.set()
            self.strain_test_active = False
            self.update_terminal("Stopping strain test...\n")
        else:
            self.update_terminal("No strain test active\n")

    def create_plot_window(self):
        """Create the independent torque-angle plot window."""
        global plot_window_open, plot_window, plot_curve, angle_data, torque_data
        
        try:
            # Reset data arrays
            angle_data = []
            torque_data = []
            
            # Get title from file name prefix field
            plot_title = self.file_name_input.get() or "AFO Strain Test"
            
            # Create plot window if not already open
            if not plot_window_open:
                # Match the light EAST interface while retaining a separate OS window.
                pg.setConfigOption('background', '#ffffff')
                pg.setConfigOption('foreground', TEXT)
                pg.setConfigOptions(antialias=True)  # Enable antialiasing globally
                
                # Create a normal QWidget container first
                self.plot_container = QWidget()
                self.plot_container.setWindowTitle("Torque vs AFO Angle")
                
                # Use a large default graph window for easier live-data viewing.
                self.plot_container.resize(1280, 840)
                self.plot_container.setMinimumSize(1040, 680)
                
                self.plot_container.setStyleSheet("""
                    QWidget {
                        background-color: #f8fafc;
                        border: 0px;
                    }
                """)
                
                # Create the plot widget with no navigation bar
                plot_window = pg.PlotWidget()
                plot_window.setBackground('#ffffff')
                
                # Enable antialiasing for the plot
                plot_window.setAntialiasing(True)
                
                # Set title with larger font
                title_style = {'color': TEXT, 'size': '18pt'}
                plot_window.setTitle(plot_title, **title_style)
                
                # Set axis labels with larger font and white color
                label_style = {'color': TEXT, 'font-size': '12pt'}
                plot_window.setLabel('left', 'Torque (Nm)', **label_style)
                plot_window.setLabel(
                    'bottom', 'ODrive-Derived AFO Angle (degrees)', **label_style
                )
                
                # Hide the navigation bar
                plot_window.hideButtons()
                
                # Set grid style
                plot_window.showGrid(x=True, y=True, alpha=0.3)
                
                # Customize axes with larger text
                for axis in [plot_window.getAxis('left'), plot_window.getAxis('bottom')]:
                    axis.setPen(color=TEXT, width=2)
                    axis.setTextPen(color=TEXT)
                    axis.setStyle(tickFont=QFont('Arial', 12))
                    axis.setTextPen(TEXT)
                
                # Add a legend with custom styling and larger text
                legend = plot_window.addLegend(
                    pen='#cbd5e1', brush=(248, 250, 252, 235), labelTextColor=TEXT
                )
                legend.setLabelTextSize('12pt')  # Increased legend text size
                
                # Create the data curve with line only (no symbols)
                plot_curve = plot_window.plot(
                    angle_data, 
                    torque_data, 
                    pen=pg.mkPen(
                        color=(37, 99, 235),
                        width=2,  # Maintain line width for clarity
                        cosmetic=True,  # Ensures consistent width during scaling
                        style=Qt.SolidLine  # Ensure solid line style
                    ),
                    name='Torque vs Angle',
                    antialias=True,  # Enable antialiasing for the curve
                    connect='all',  # Connect all points for smoother line
                    skipFiniteCheck=True  # Skip finite check for better performance
                )
                
                # Create layout and add plot widget to container
                layout = QVBoxLayout(self.plot_container)
                layout.setContentsMargins(15, 15, 15, 15)  # Add more padding around the plot
                layout.addWidget(plot_window)
                
                # Make the plot a normal window so it can be moved, minimized,
                # or placed behind the main GUI by the operator.
                self.plot_container.setWindowFlags(Qt.Window)
                
                # Show the container
                self.plot_container.show()
                self.plot_container.raise_()
                
                # Set up a timer for plot updates
                self.setup_plot_timer()
                
                plot_window_open = True
                self.update_terminal("Plot window created successfully\n")
            
            # Update title if plot already exists
            else:
                plot_window.setTitle(plot_title)
                self.plot_container.show()
                self.plot_container.raise_()
                self.update_terminal("Plot updated\n")
                
        except Exception as e:
            self.update_terminal(f"Error creating plot window: {str(e)}\n")
            plot_window_open = False

    def setup_plot_timer(self):
        """Set up a timer to update the plot periodically"""
        global plot_timer
        
        # Create a timer for updating the plot
        plot_timer = QTimer()
        plot_timer.timeout.connect(self.update_plot)
        plot_timer.start(8)  # Update plot every 8ms (125Hz) to match data collection rate
    
    def update_plot(self):
        """Update the plot with new data"""
        global plot_curve, angle_data, torque_data, plot_window_open
        
        # Check if plot window is still open
        if not plot_window_open:
            return
        
        # Update plot with new data if available
        if angle_data and torque_data:
            plot_curve.setData(angle_data, torque_data)
    
    def close_plot_window(self):
        """Close the plot window safely"""
        global plot_window_open, plot_window, plot_curve, angle_data, torque_data
        
        try:
            if hasattr(self, 'plot_container') and self.plot_container is not None:
                # Hide the container first
                self.plot_container.hide()
                
                # Clear the plot data
                if plot_curve is not None:
                    plot_curve.clear()
                angle_data = []
                torque_data = []
                
                # Delete the plot curve reference
                plot_curve = None
                
                # Close and delete the plot window
                if plot_window is not None:
                    plot_window.setParent(None)
                    plot_window = None
                
                # Close and delete the container
                self.plot_container.setParent(None)
                self.plot_container.deleteLater()
                self.plot_container = None
                
                plot_window_open = False
        except Exception as e:
            print(f"Error closing plot window: {e}")
            # Ensure flags are reset even if there's an error
            plot_window_open = False
            plot_window = None
            plot_curve = None
            self.plot_container = None

    def update_plot_data(self, angle, torque):
        """Add new data points to the plot"""
        global angle_data, torque_data, plot_window_open, plot_curve
        
        try:
            # Only update if plot window is open
            if plot_window_open and plot_curve is not None:
                # Add new data points
                angle_data.append(angle)  # Use the moving average values directly
                torque_data.append(torque)  # Use the moving average values directly
                
                # Keep a maximum number of points for performance
                max_points = 10000  # Keep high number of points for resolution
                if len(angle_data) > max_points:
                    # Keep more recent points for better resolution
                    angle_data = angle_data[-max_points:]
                    torque_data = torque_data[-max_points:]
                
                # Update the plot with the data
                plot_curve.setData(
                    angle_data, 
                    torque_data,
                    connect='all',  # Connect all points for smoother line
                    skipFiniteCheck=True  # Skip finite check for better performance
                )
                
                # Auto-scale the plot to show all data points
                plot_window.enableAutoRange()
        except Exception as e:
            self.update_terminal(f"Error updating plot: {str(e)}\n")

    def validate_step_angle(self, event=None):
        """Validate and constrain step angle input"""
        try:
            value = self.step_angle_input.get()
            if value:  # Only validate if there's a value
                angle = float(value)
                if angle < 0.01:
                    self.step_angle_input.delete(0, 'end')
                    self.step_angle_input.insert(0, "0.01")
                    self.update_terminal("Step angle must be at least 0.01 degrees\n")
                elif angle > 10:
                    self.step_angle_input.delete(0, 'end')
                    self.step_angle_input.insert(0, "10")
                    self.update_terminal("Step angle cannot exceed 10 degrees\n")
        except ValueError:
            # If the input is not a valid number, clear it
            self.step_angle_input.delete(0, 'end')

    def move_motor_left(self):
        """Move the motor to the left (negative direction)"""
        if not hasattr(self, 'odrive_controller') or not self.odrive_controller:
            self.update_terminal("ODrive not connected\n")
            return
            
        try:
            axis = self.prepare_manual_motion()
            current_pos = axis.pos_vel_mapper.pos_rel
            
            if self.continuous_mode.get():
                return
            else:
                # In step mode, move by the specified angle
                try:
                    step_angle = float(self.step_angle_input.get())
                    # Constrain step angle between 0.01 and 5 degrees
                    step_angle = max(0.01, min(10.0, step_angle))
                    step_turns = afo_degrees_to_odrive_turns(step_angle, self.system_config)
                    target_pos = self.clamp_manual_target(current_pos - step_turns)
                    axis.controller.input_pos = target_pos
                    self.update_terminal(f"Moved left by {step_angle:.2f} degrees\n")
                except ValueError:
                    self.update_terminal("Please enter a valid step angle between 0 and 10 degrees\n")
                    
        except Exception as e:
            self.update_terminal(f"Error moving motor: {e}\n")

    def move_motor_right(self):
        """Move the motor to the right (positive direction)"""
        if not hasattr(self, 'odrive_controller') or not self.odrive_controller:
            self.update_terminal("ODrive not connected\n")
            return
            
        try:
            axis = self.prepare_manual_motion()
            current_pos = axis.pos_vel_mapper.pos_rel
            
            if self.continuous_mode.get():
                return
            else:
                # In step mode, move by the specified angle
                try:
                    step_angle = float(self.step_angle_input.get())
                    # Constrain step angle between 0.01 and 5 degrees
                    step_angle = max(0.01, min(10.0, step_angle))
                    step_turns = afo_degrees_to_odrive_turns(step_angle, self.system_config)
                    target_pos = self.clamp_manual_target(current_pos + step_turns)
                    axis.controller.input_pos = target_pos
                    self.update_terminal(f"Moved right by {step_angle:.2f} degrees\n")
                except ValueError:
                    self.update_terminal("Please enter a valid step angle between 0 and 10 degrees\n")
                    
        except Exception as e:
            self.update_terminal(f"Error moving motor: {e}\n")

    def start_continuous_movement(self):
        """Start continuous movement in the current direction"""
        if not self.continuous_movement_active:
            return

        try:
            axis = self.prepare_manual_motion()
            current_pos = axis.pos_vel_mapper.pos_rel
            increment = afo_degrees_to_odrive_turns(
                self.system_config["motion"]["manual_continuous_increment_deg"],
                self.system_config,
            )
            
            if self.movement_direction == "left":
                target_pos = current_pos - increment
            else:  # right
                target_pos = current_pos + increment
                
            target_pos = self.clamp_manual_target(target_pos)
            axis.controller.input_pos = target_pos
            
            # Schedule the next movement
            self.movement_timer = self.master.after(
                self.system_config["motion"]["manual_update_interval_ms"],
                self.start_continuous_movement,
            )
            
        except Exception as e:
            self.update_terminal(f"Error in continuous movement: {e}\n")
            self.stop_continuous_movement()

    def begin_continuous_movement(self, direction):
        if not self.continuous_mode.get() or self.continuous_movement_active:
            return
        self.continuous_movement_active = True
        self.movement_direction = direction
        self.start_continuous_movement()

    def stop_continuous_movement(self):
        """Stop continuous movement"""
        self.continuous_movement_active = False
        if self.movement_timer:
            self.master.after_cancel(self.movement_timer)
            self.movement_timer = None

    def prepare_manual_motion(self):
        if not self.manual_mode.get():
            raise RuntimeError("Enable manual mode before commanding manual movement")
        if self.strain_test_active:
            raise RuntimeError("Manual movement is disabled while a strain test is active")
        if self.neutral_motion_active:
            raise RuntimeError("Wait for the neutral return to finish")
        motion = self.system_config["motion"]
        self.configure_trajectory(
            motion["manual_speed_deg_s"], motion["manual_acceleration_deg_s2"]
        )
        axis = self.enter_closed_loop()
        if int(axis.active_errors):
            self.safe_idle_motor("manual movement error")
            raise RuntimeError(f"ODrive has active errors: {int(axis.active_errors)}")
        self.set_status("MANUAL / ACTIVE", AMBER)
        return axis

    def return_to_neutral(self):
        """Return to the fixed 90 degree encoder position from configuration."""
        if self.odrive_controller is None:
            self.update_terminal("Connect the ODrive before returning to neutral.\n")
            return
        if self.connected_neutral_position is None:
            self.update_terminal("The fixed 90 degree neutral reference is unavailable.\n")
            return
        if not self.manual_mode.get():
            self.update_terminal("Enable manual mode before returning to neutral.\n")
            return
        if self.strain_test_active or self.neutral_motion_active:
            self.update_terminal("Neutral return is unavailable while another motion is active.\n")
            return

        confirmation = CTkMessagebox(
            title="Return to Neutral",
            message=(
                "Return to the fixed 90 degree target at 0 ODrive turns?\n\n"
                "Confirm the fixture is clear and the physical E-stop is accessible."
            ),
            icon="question",
            option_1="Cancel",
            option_2="Return",
        )
        if confirmation.get() != "Return":
            return

        self.stop_continuous_movement()
        self.neutral_stop_event.clear()
        self.neutral_motion_active = True
        self.left_arrow.configure(state="disabled")
        self.right_arrow.configure(state="disabled")
        self.neutral_button.configure(state="disabled")
        self.neutral_thread = threading.Thread(
            target=self._return_to_neutral_worker,
            name="neutral-return",
        )
        self.neutral_thread.start()

    def _return_to_neutral_worker(self):
        try:
            motion = self.system_config["motion"]
            self.configure_trajectory(
                motion["manual_speed_deg_s"], motion["manual_acceleration_deg_s2"]
            )
            axis = self.enter_closed_loop()
            neutral_target = self.connected_neutral_position
            if neutral_target is None:
                raise RuntimeError("Neutral reference is unavailable")
            current_turns = axis.pos_vel_mapper.pos_rel
            distance_deg = abs(
                odrive_turns_to_afo_degrees(
                    current_turns - neutral_target, self.system_config
                )
            )
            timeout_s = motion_timeout_seconds(
                distance_deg,
                motion["manual_speed_deg_s"],
                motion["manual_acceleration_deg_s2"],
                self.system_config,
            )
            tolerance_turns = afo_degrees_to_odrive_turns(
                motion["position_tolerance_deg"], self.system_config
            )
            self.motion_phase = "returning_to_connected_zero"
            axis.controller.input_pos = neutral_target
            self.update_terminal(
                f"Returning to fixed 90 degree neutral at {neutral_target:.8f} turns.\n"
            )
            deadline = time.monotonic() + timeout_s
            while abs(axis.pos_vel_mapper.pos_rel - neutral_target) > tolerance_turns:
                if self.neutral_stop_event.is_set():
                    raise TestStopped()
                active_errors = int(axis.active_errors)
                if active_errors:
                    raise RuntimeError(
                        f"ODrive active errors during neutral return: {active_errors}"
                    )
                if time.monotonic() >= deadline:
                    raise TimeoutError(
                        f"Neutral return timed out after {timeout_s:.1f} seconds"
                    )
                time.sleep(0.01)
            self.update_terminal("Neutral reference reached.\n")
            self.set_status("MANUAL / NEUTRAL", "#0f766e")
        except TestStopped:
            self.safe_idle_motor("neutral return stopped")
            self.update_terminal("Neutral return stopped.\n")
        except Exception as exc:
            self.safe_idle_motor("neutral return error")
            self.set_status("ERROR / IDLE", RED)
            self.update_terminal(f"Neutral return failed: {exc}\n")
        finally:
            self.motion_phase = "idle"
            self.ui_message_queue.put(("neutral_finished",))

    def clamp_manual_target(self, target_turns):
        maximum_angle = self.system_config["motion"]["maximum_afo_angle_deg"]
        travel_turns = afo_degrees_to_odrive_turns(maximum_angle, self.system_config)
        lower = self.starting_position - travel_turns
        upper = self.starting_position + travel_turns
        clamped = max(lower, min(upper, target_turns))
        if clamped != target_turns:
            self.update_terminal(f"Manual travel limited to +/-{maximum_angle:g} degrees from zero.\n")
        return clamped

    def toggle_manual_mode(self):
        """Handle manual mode toggle"""
        if self.strain_test_active:
            self.manual_mode.set(False)
            self.update_terminal("Manual mode cannot be changed during a strain test.\n")
            return
        if self.manual_mode.get():
            # Enable manual controls
            self.left_arrow.configure(state="normal")
            self.right_arrow.configure(state="normal")
            self.step_angle_input.configure(state="normal")
            self.mode_toggle.configure(state="normal")
            self.neutral_button.configure(
                state=(
                    "normal"
                    if self.odrive_controller is not None
                    and self.connected_neutral_position is not None
                    else "disabled"
                )
            )
            
            # Disable Start button in manual mode
            self.buttons[1].configure(state="disabled")
            
            # Disable input fields
            self.speed_input.configure(state="disabled")
            self.acceleration_input.configure(state="disabled")
            self.min_angle_input.configure(state="disabled")
            self.max_angle_input.configure(state="disabled")
            self.cycles_input.configure(state="disabled")
            if self.odrive_controller:
                try:
                    self.prepare_manual_motion()
                except Exception as exc:
                    self.update_terminal(f"Unable to enable manual motion: {exc}\n")
        else:
            self.neutral_stop_event.set()
            self.stop_continuous_movement()
            self.safe_idle_motor("manual mode disabled")
            # Disable manual controls
            self.left_arrow.configure(state="disabled")
            self.right_arrow.configure(state="disabled")
            self.step_angle_input.configure(state="disabled")
            self.mode_toggle.configure(state="disabled")
            self.neutral_button.configure(state="disabled")
            
            # Enable Start button when not in manual mode (only if connected)
            if hasattr(self, 'odrive_controller') and self.odrive_controller:
                self.buttons[1].configure(
                    state="normal" if self.connected_neutral_position is not None else "disabled"
                )
            
            # Enable input fields
            self.speed_input.configure(state="normal")
            self.acceleration_input.configure(state="normal")
            self.min_angle_input.configure(state="normal")
            self.max_angle_input.configure(state="normal")
            self.cycles_input.configure(state="normal")

    def validate_angle_input(self, event=None):
        """Validate angle magnitudes against the configured AFO travel limit."""
        try:
            # Get the widget that triggered the event
            widget = event.widget
            
            # Get the current value
            value = widget.get()
            if value:  # Only validate if there's a value
                angle = float(value)
                maximum_angle = self.system_config["motion"]["maximum_afo_angle_deg"]
                if angle > maximum_angle:
                    widget.delete(0, 'end')
                    widget.insert(0, f"{maximum_angle:g}")
                    self.update_terminal(f"Angle magnitude cannot exceed {maximum_angle:g} degrees\n")
                elif angle < 0:
                    widget.delete(0, 'end')
                    widget.insert(0, "0")
                    self.update_terminal("Enter angle limits as positive magnitudes\n")
        except ValueError:
            # If the input is not a valid number, clear it
            widget.delete(0, 'end')

def create_about_dialog(root):
    about_dialog = ctk.CTkToplevel(root)
    about_dialog.geometry("520x430")
    about_dialog.configure(fg_color=BG)
    about_dialog.title("About")
    about_dialog.transient(root)
    try:
        icon_path = resource_path("images/icon.ico")
        if icon_path.exists():
            about_dialog.iconbitmap(str(icon_path))
    except Exception:
        pass

    content = ctk.CTkFrame(about_dialog, fg_color=PANEL, corner_radius=10)
    content.pack(fill="both", expand=True, padx=24, pady=24)
    ctk.CTkLabel(
        content, text=APP_NAME, font=("Arial", 34, "bold"), text_color=TEXT
    ).pack(pady=(28, 4))
    ctk.CTkLabel(
        content,
        text=f"EPIC AFO Stiffness Tester\nVersion {APP_VERSION}",
        font=("Arial", 14, "bold"),
        text_color=MUTED,
        justify="center",
    ).pack(pady=(0, 24))
    ctk.CTkLabel(
        content,
        text=(
            "EAST controls the motorised benchtop AFO stiffness tester, "
            "records load and motion data, and displays the live torque-angle plot.\n\n"
            "Confirm the fixture is clear and the physical E-stop is accessible "
            "before enabling motion."
        ),
        font=("Arial", 13),
        text_color=TEXT,
        wraplength=400,
        justify="left",
    ).pack(padx=24)
    ctk.CTkButton(
        content, text="Close", command=about_dialog.destroy, width=140
    ).pack(pady=28)

def main():
    print(f"Starting {APP_NAME} {APP_VERSION} from {Path(__file__).resolve()}")

    # Ensure there's only one QApplication instance
    if not QApplication.instance():
        app = QApplication(sys.argv)
    else:
        app = QApplication.instance()
    
    root = ctk.CTk()
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    window_width = min(1040, max(860, screen_width - 80))
    # Keep the main controls compact while retaining the existing responsive cap.
    window_height = int(min(700, max(620, screen_height - 100)) * 0.8)
    root.geometry(f"{window_width}x{window_height}")
    root.minsize(860, 496)
    root.resizable(True, True)

    app_instance = MyInterface(root)
    root.protocol("WM_DELETE_WINDOW", app_instance.on_close)

    # Set the window icon
    try:
        icon_path = resource_path("images/icon.ico")
        if icon_path.exists():
            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(str(icon_path))
            root.iconbitmap(str(icon_path))
    except Exception as e:
        print(f"Failed to set icon: {e}")

    # Create menubar
    menubar = tk.Menu(root)
    file_menu = tk.Menu(menubar, tearoff=0)
    menubar.add_cascade(label="File", menu=file_menu)
    file_menu.add_command(label="Help", command=lambda: create_about_dialog(root))
    root.configure(menu=menubar)

    root.update_idletasks()
    
    root.mainloop()
    
    # Cleanup Qt application
    if QApplication.instance():
        QApplication.instance().quit()

if __name__ == "__main__":
    main()
