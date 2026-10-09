import argparse
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
import math
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

from east_core import (
    CSV_COLUMNS,
    EmptyMachineTare,
    afo_acceleration_to_odrive_turns_s2,
    afo_degrees_to_odrive_turns,
    afo_speed_to_odrive_turns_s,
    calculate_load,
    calculate_torque_nm,
    constant_speed_span_deg,
    create_run_paths,
    load_tester_config,
    make_preset_metadata,
    make_run_metadata,
    motion_timeout_seconds,
    manual_step_to_turns,
    odrive_turns_to_afo_degrees,
    reconcile_run_outcome,
    empty_machine_tare_identity_mismatches,
    preset_values_modified,
    validate_empty_machine_tare,
    validate_test_parameters,
    write_json_atomic,
)
from east_odrive import ODriveAdapter
from east_reference import (
    AtomicStateStore,
    FeedbackError,
    MotionConflictError,
    MotionCoordinator,
    MotionState,
    ProcessLock,
    ReferenceConfidence,
    ReferenceError,
    ReferenceManager,
    ReferenceRequiredError,
    evaluate_continuity,
    hardware_fingerprints_match,
    persist_idle_continuity_checkpoint,
    runtime_state_directory,
    wait_for_settle,
)

# Variable to track if plot window is open
plot_window_open = False
# Global variables for plot data
angle_data = []
torque_data = []
plot_window = None
plot_curve = None

from east_gui_tabs import OutputTabs

APP_NAME = "EAST"
APP_VERSION = "1.5.3-rotary-hub-identity"

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
        self.odrive_adapter = None
        self.hardware_fingerprint = None
        self.voltage_ratio_input = None
        self.empty_machine_tare = None
        self.protocol_display_to_key = {
            "Custom": "custom",
            **{
                preset["display_name"]: key
                for key, preset in self.system_config["test_presets"].items()
            },
        }
        self.protocol_var = ctk.StringVar(value="Custom")
        self.loaded_preset_key = "custom"
        self.preset_loaded_at = datetime.now().astimezone().isoformat(
            timespec="milliseconds"
        )
        self.run_parameters = None
        self.run_metadata = None
        self.metadata_file_name = None
        self.strain_file_name = None
        self.test_started_monotonic = None
        self.test_stop_event = threading.Event()
        self.neutral_stop_event = threading.Event()
        self.continuous_stop_event = threading.Event()
        self.monitor_stop_event = threading.Event()
        self.watchdog_stop_event = threading.Event()
        self.finalize_lock = threading.Lock()
        self.run_finalized = True
        self.operator_stop_requested = False
        self.acquisition_done_event = threading.Event()
        self.motion_phase = "idle"
        self.commanded_odrive_velocity = 0.0
        self.commanded_afo_acceleration = 0.0
        self.current_nominal_distance_deg = 0.0
        self.current_expected_constant_speed_span_deg = 0.0
        self.ui_message_queue = queue.Queue()
        self.plot_data_queue = queue.Queue()
        self.header_images = []

        self.state_store = AtomicStateStore(runtime_state_directory())
        self.process_lock = ProcessLock(self.state_store.lock_path)
        self.startup_block_reason = None
        try:
            self.process_lock.acquire()
        except MotionConflictError as exc:
            self.startup_block_reason = str(exc)
        self.reference_manager = ReferenceManager(self.system_config, self.state_store)
        self.motion_coordinator = MotionCoordinator()
        self.feedback_lock = threading.RLock()
        self.latest_feedback = None
        self.monitor_error = None
        self.monitor_thread = None
        self.expected_axis_state = None
        self.expected_disarm_reason = None
        self.control_health_lock = threading.RLock()
        self.last_control_health_monotonic = time.monotonic()
        self.run_motion_token = None
        self.neutral_motion_token = None
        self.continuous_motion_token = None
        self.strain_thread = None
        self.data_collection_thread = None
        self.neutral_thread = None
        self.manual_step_thread = None
        self.continuous_thread = None
        self.recovery_thread = None
        self.recovery_window = None
        self.rotary_calibration = None
        self.recovery_origin_turns = None
        self.recovery_cumulative_deg = 0.0
        self.recovery_session_acknowledged = False

        self.strain_test_active = False
        self.strain_data_buffer = []
        self.current_cycle = 0
        
        # Add continuous movement flags
        self.continuous_movement_active = False
        self.neutral_motion_active = False
        self.movement_direction = None
        self.movement_timer = None
        
        # Add manual mode flag
        self.manual_mode = ctk.BooleanVar(value=False)
        self.manual_step_units = ctk.StringVar(value="Degrees")
        
        self.sample_count = 0
        
        # Moving average filters for both plot and data logging
        filter_window = self.system_config["acquisition"]["moving_average_window_samples"]
        self.angle_filter = MovingAverageFilter(window_size=filter_window)
        self.weight_filter = MovingAverageFilter(window_size=filter_window)
        self.torque_filter = MovingAverageFilter(window_size=filter_window)

        set_default_color_theme("blue")
        ctk.set_appearance_mode("light")

        self.setup_ui()
        self.update_reference_display()
        if self.startup_block_reason:
            self.update_terminal(f"HARDWARE CONTROLS BLOCKED: {self.startup_block_reason}\n")
            self.buttons[0].configure(state="disabled")
        self.master.after(50, self._drain_ui_queues)
        self.master.after_idle(self.update_plot)

        self.master.bind('<Escape>', lambda _event: self.stop_logging())
        self.master.bind_all('<Escape>', lambda _event: self.stop_logging())

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
        self.fixture_id_input.bind("<KeyRelease>", self._on_tare_identity_edit, add="+")
        self.calibration_id_input.bind("<KeyRelease>", self._on_tare_identity_edit, add="+")
        for entry in (
            self.cycles_input,
            self.speed_input,
            self.acceleration_input,
            self.min_angle_input,
            self.max_angle_input,
        ):
            entry.bind("<KeyRelease>", self.update_parameter_summary, add="+")

        protocol_frame = ctk.CTkFrame(inputs_frame, fg_color="transparent")
        protocol_frame.grid(
            row=6, column=0, columnspan=2, padx=6, pady=(6, 4), sticky="ew"
        )
        protocol_frame.grid_columnconfigure((1, 2, 3), weight=1)
        ctk.CTkLabel(
            protocol_frame,
            text="Protocol:",
            font=("Arial", 11, "bold"),
            text_color=TEXT,
        ).grid(row=0, column=0, padx=(0, 6), sticky="w")
        self.protocol_menu = ctk.CTkOptionMenu(
            protocol_frame,
            variable=self.protocol_var,
            values=list(self.protocol_display_to_key),
            command=lambda _selection: self.update_protocol_status(),
            width=205,
            height=30,
        )
        self.protocol_menu.grid(
            row=0, column=1, columnspan=3, padx=(0, 0), sticky="ew"
        )
        self.load_preset_button = ctk.CTkButton(
            protocol_frame,
            text="Load Preset",
            command=self.load_selected_preset,
            width=105,
            height=30,
            fg_color=BLUE,
            hover_color="#1d4ed8",
        )
        self.load_preset_button.grid(
            row=1, column=0, columnspan=2, padx=(0, 3), pady=(5, 0), sticky="ew"
        )
        self.reset_fields_button = ctk.CTkButton(
            protocol_frame,
            text="Reset Test Fields",
            command=self.reset_test_fields,
            width=130,
            height=30,
            fg_color=AMBER,
            hover_color="#b45309",
        )
        self.reset_fields_button.grid(
            row=1, column=2, columnspan=2, padx=(3, 0), pady=(5, 0), sticky="ew"
        )
        self.protocol_status_label = ctk.CTkLabel(
            inputs_frame,
            text="Active protocol: Custom",
            text_color=MUTED,
            font=("Arial", 10, "bold"),
            anchor="w",
            justify="left",
            wraplength=390,
        )
        self.protocol_status_label.grid(
            row=7, column=0, columnspan=2, padx=8, pady=(0, 7), sticky="ew"
        )

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
            if index == 2:
                button.grid(row=2, column=0, columnspan=2, padx=5, pady=3, sticky="ew")
            else:
                button.grid(row=1, column=index, padx=5, pady=3, sticky="ew")
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
        self.manual_step_units_menu = ctk.CTkOptionMenu(
            manual_control_frame, values=["Degrees", "Motor turns"],
            variable=self.manual_step_units, command=self._manual_step_unit_changed,
            height=30, width=130, state="disabled",
        )
        self.manual_step_units_menu.grid(row=0, column=0, sticky="ew", padx=6, pady=(6, 4))
        self.step_angle_input.grid(row=0, column=1, sticky="ew", padx=6, pady=(6, 4))
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

        self.manual_step_hint = ctk.CTkLabel(
            manual_control_frame, text="", font=("Arial", 10), anchor="w",
            justify="left", wraplength=390, text_color=MUTED,
        )
        self.manual_step_hint.grid(row=2, column=0, columnspan=3, sticky="ew", padx=6, pady=(2, 2))
        self.manual_position_label = ctk.CTkLabel(
            manual_control_frame, text="Motor feedback unavailable: connect ODrive",
            font=("Arial", 11, "bold"), anchor="w", justify="left", wraplength=390,
            text_color=MUTED,
        )
        self.manual_position_label.grid(row=3, column=0, columnspan=3, sticky="ew", padx=6, pady=(2, 4))
        self._update_manual_step_hint()

        machine_zero_frame = ctk.CTkFrame(
            manual_control_frame, fg_color="#fff7ed", corner_radius=8
        )
        machine_zero_frame.grid(
            row=4, column=0, columnspan=3, padx=6, pady=(4, 6), sticky="ew"
        )
        machine_zero_frame.grid_columnconfigure((0, 1), weight=1)
        ctk.CTkLabel(
            machine_zero_frame,
            text="Machine Zero \N{EM DASH} Fixture at 90\N{DEGREE SIGN}",
            font=("Arial", 14, "bold"),
            text_color=TEXT,
            anchor="w",
        ).grid(row=0, column=0, columnspan=2, padx=8, pady=(8, 2), sticky="ew")
        ctk.CTkLabel(
            machine_zero_frame,
            text=(
                "Use the supplied square to confirm that the moving fixture is at 90\N{DEGREE SIGN} "
                "to the fixed machine reference. Use the slow Jog Left and Jog Right controls "
                "to make small adjustments until the fixture is aligned with the square. Then "
                "select 'Set Machine Zero \N{EM DASH} Fixture at 90\N{DEGREE SIGN}'. This defines "
                "machine angle 0\N{DEGREE SIGN} for the current ODrive power session."
            ),
            font=("Arial", 10),
            text_color=TEXT,
            anchor="w",
            justify="left",
            wraplength=390,
        ).grid(row=1, column=0, columnspan=2, padx=8, pady=(0, 5), sticky="ew")
        self.reference_status_label = ctk.CTkLabel(
            machine_zero_frame,
            text="Machine zero: SETUP REQUIRED",
            font=("Arial", 11, "bold"),
            text_color=AMBER,
            anchor="w",
            justify="left",
            wraplength=390,
        )
        self.reference_status_label.grid(
            row=2, column=0, columnspan=2, padx=8, pady=(0, 5), sticky="ew"
        )
        self.reference_action_button = ctk.CTkButton(
            machine_zero_frame,
            text="Set Machine Zero \N{EM DASH} Fixture at 90\N{DEGREE SIGN}",
            command=self.open_reference_recovery,
            fg_color=AMBER,
            hover_color="#b45309",
            state="disabled",
            height=34,
        )
        self.reference_action_button.grid(row=3, column=0, padx=(8, 4), pady=(0, 8), sticky="ew")

        self.neutral_button = ctk.CTkButton(
            machine_zero_frame,
            text="Return to Machine Zero \N{EM DASH} 90\N{DEGREE SIGN}",
            command=self.return_to_machine_zero,
            fg_color="#0f766e",
            hover_color="#115e59",
            corner_radius=8,
            height=34,
            font=("Arial", 12, "bold"),
        )
        self.neutral_button.grid(
            row=3, column=1, padx=(4, 8), pady=(0, 8), sticky="ew"
        )

        tare_frame = ctk.CTkFrame(manual_control_frame, fg_color="#ecfdf5", corner_radius=8)
        tare_frame.grid(row=5, column=0, columnspan=3, padx=6, pady=(0, 6), sticky="ew")
        tare_frame.grid_columnconfigure((0, 1), weight=1)
        self.tare_status_label = ctk.CTkLabel(
            tare_frame,
            text="Empty-machine tare: REQUIRED",
            font=("Arial", 11, "bold"),
            text_color=AMBER,
            anchor="w",
            justify="left",
            wraplength=390,
        )
        self.tare_status_label.grid(
            row=0, column=0, columnspan=2, padx=8, pady=(7, 3), sticky="ew"
        )
        self.tare_button = ctk.CTkButton(
            tare_frame,
            text="Tare Empty Machine",
            command=self.tare_empty_machine,
            fg_color=GREEN,
            hover_color="#15803d",
            state="disabled",
            height=34,
        )
        self.tare_button.grid(row=1, column=0, padx=(8, 4), pady=(0, 8), sticky="ew")
        self.clear_session_tare_button = ctk.CTkButton(
            tare_frame,
            text="Clear Session Tare",
            command=self.clear_session_tare,
            fg_color=AMBER,
            hover_color="#b45309",
            state="disabled",
            height=34,
        )
        self.clear_session_tare_button.grid(
            row=1, column=1, padx=(4, 8), pady=(0, 8), sticky="ew"
        )

        self.rotary_calibration_button = ctk.CTkButton(
            manual_control_frame, text="Rotary Sensor Calibration",
            command=self.open_rotary_calibration, fg_color=BLUE,
            hover_color="#1d4ed8", height=34,
        )
        self.rotary_calibration_button.grid(
            row=6, column=0, columnspan=3, padx=6, pady=(2, 8), sticky="ew"
        )

        for widget in (
            self.left_arrow,
            self.right_arrow,
            self.step_angle_input,
            self.mode_toggle,
            self.neutral_button,
            self.tare_button,
        ):
            widget.configure(state="disabled")
        self.buttons[1].configure(state="disabled")

        self.output_tabs = OutputTabs(
            body, fg_color=PANEL, corner_radius=10,
            segmented_button_selected_color=BLUE,
            segmented_button_selected_hover_color="#1d4ed8",
            command=self._on_output_tab_changed,
        )
        self.output_tabs.grid(row=0, column=1, sticky="nsew")
        terminal_tab = self.output_tabs.add("Session Terminal")
        self.plot_container = self.output_tabs.add("Plot")
        terminal_tab.grid_columnconfigure(0, weight=1)
        terminal_tab.grid_rowconfigure(0, weight=1)
        self.terminal = ctk.CTkTextbox(
            terminal_tab,
            height=220,
            fg_color=BG,
            text_color=TEXT,
            border_width=1,
            border_color="#cbd5e1",
            corner_radius=8,
            wrap="word",
        )
        self.terminal.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
        self.output_tabs.set("Session Terminal")
        self.create_plot_window(select_plot=False)

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
        self.update_protocol_status()

    @staticmethod
    def _replace_entry_value(entry, value):
        entry.delete(0, ctk.END)
        entry.insert(0, str(value))

    def _motion_field_values(self):
        return {
            "cycles": self.cycles_input.get(),
            "minimum_angle_deg": self.min_angle_input.get(),
            "maximum_angle_deg": self.max_angle_input.get(),
            "speed_deg_s": self.speed_input.get(),
            "acceleration_deg_s2": self.acceleration_input.get(),
        }

    def _loaded_preset(self):
        if self.loaded_preset_key == "custom":
            return None
        return self.system_config["test_presets"][self.loaded_preset_key]

    def active_protocol_name(self):
        preset = self._loaded_preset()
        if preset is None:
            return "Custom"
        name = f"{preset['display_name']} v{preset['version']}"
        if preset_values_modified(self._motion_field_values(), preset):
            name += " \N{EM DASH} MODIFIED"
        return name

    def update_protocol_status(self):
        if not hasattr(self, "protocol_status_label"):
            return
        active_name = self.active_protocol_name()
        selected_display = self.protocol_var.get()
        selected_key = self.protocol_display_to_key[selected_display]
        text = f"Active protocol: {active_name}"
        if selected_key != self.loaded_preset_key:
            text += (
                f"\nSelected: {selected_display} \N{EM DASH} press Load Preset to apply. "
                f"Active protocol remains: {active_name}."
            )
            colour = AMBER
        elif active_name.endswith("MODIFIED"):
            colour = AMBER
        elif self.loaded_preset_key == "custom":
            colour = MUTED
        else:
            colour = GREEN
        self.protocol_status_label.configure(text=text, text_color=colour)

    def load_selected_preset(self):
        selected_display = self.protocol_var.get()
        selected_key = self.protocol_display_to_key[selected_display]
        preset = (
            None
            if selected_key == "custom"
            else self.system_config["test_presets"][selected_key]
        )
        entered_values = self._motion_field_values()
        has_entered_values = any(str(value).strip() for value in entered_values.values())
        replacing_values = has_entered_values and (
            preset is None or preset_values_modified(entered_values, preset)
        )

        messages = []
        if replacing_values:
            messages.append("This will replace the motion values currently entered.")
        if preset and preset["test_type"] == "empty_machine_baseline":
            messages.append(
                "Confirm the AFO and all removable loads are removed before running "
                "this empty-machine baseline."
            )
        if messages:
            confirmation = CTkMessagebox(
                title="Load Protocol Preset",
                message="\n\n".join(messages),
                icon="question",
                option_1="Cancel",
                option_2="Load Preset",
            )
            if confirmation.get() != "Load Preset":
                return

        motion_entries = {
            "cycles": self.cycles_input,
            "minimum_angle_deg": self.min_angle_input,
            "maximum_angle_deg": self.max_angle_input,
            "speed_deg_s": self.speed_input,
            "acceleration_deg_s2": self.acceleration_input,
        }
        if preset is None:
            for entry in motion_entries.values():
                entry.delete(0, ctk.END)
        else:
            for key, entry in motion_entries.items():
                self._replace_entry_value(entry, preset[key])
        self.loaded_preset_key = selected_key
        self.preset_loaded_at = datetime.now().astimezone().isoformat(
            timespec="milliseconds"
        )
        self.update_parameter_summary()

    def current_preset_metadata(self, parameters):
        preset = self._loaded_preset()
        modified = bool(
            preset and preset_values_modified(self._motion_field_values(), preset)
        )
        return make_preset_metadata(
            self.loaded_preset_key,
            preset,
            parameters,
            self.preset_loaded_at,
            modified,
        )

    def baseline_matching_metadata(self, parameters):
        record = self.reference_manager.record
        mapping = self.reference_manager.require_verified()
        return {
            "purpose": (
                "Preserves the dimensions required to pair loaded-machine and unloaded "
                "empty-machine torque-angle sweeps during later analysis."
            ),
            "automatic_subtraction_applied": False,
            "correction_equation": (
                "AFO torque = loaded-machine torque - matched empty-machine torque"
            ),
            "test_type": parameters.test_type,
            "fixture_id": parameters.fixture_id,
            "calibration_id": parameters.calibration_id,
            "machine_zero_reference_id": record.reference_id,
            "machine_zero_reference_generation": record.generation,
            "session_mapping_verified_at": mapping.verified_at,
            "session_mapping_verification_method": mapping.verification_method,
            "minimum_angle_deg": parameters.min_angle_deg,
            "maximum_angle_deg": parameters.max_angle_deg,
            "total_rom_deg": parameters.min_angle_deg + parameters.max_angle_deg,
            "speed_deg_s": parameters.commanded_afo_speed_deg_s,
            "acceleration_deg_s2": parameters.commanded_afo_acceleration_deg_s2,
            "movement_direction_source": (
                "Match increasing_machine_angle and decreasing_machine_angle CSV rows "
                "separately."
            ),
            "tare_captured_at": self.empty_machine_tare.captured_at,
            "tare_phidget_serial_number": self.empty_machine_tare.phidget_serial_number,
            "tare_phidget_channel": self.empty_machine_tare.phidget_channel,
        }

    def collect_test_parameters(self):
        preset = self._loaded_preset()
        return validate_test_parameters({
            "file_prefix": self.file_name_input.get(),
            "operator": self.operator_input.get(),
            "afo_id": self.afo_id_input.get(),
            "fixture_id": self.fixture_id_input.get(),
            "calibration_id": self.calibration_id_input.get(),
            "test_type": preset["test_type"] if preset else "custom",
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
        self.protocol_menu.configure(state=state)
        self.load_preset_button.configure(state=state)
        self.reset_fields_button.configure(state=state)
        if state == "disabled":
            self.clear_session_tare_button.configure(state="disabled")

    def get_feedback(self, require_fresh=True):
        with self.feedback_lock:
            snapshot = self.latest_feedback
        if snapshot is None:
            raise FeedbackError("No ODrive feedback is available")
        stale_after_s = self.system_config["reference"]["feedback_stale_after_ms"] / 1000.0
        snapshot.validate(
            now_monotonic_s=time.monotonic(),
            max_age_s=stale_after_s if require_fresh else None,
            max_capture_duration_s=(
                self.system_config["reference"]["maximum_feedback_capture_ms"] / 1000.0
            ),
        )
        return snapshot

    def _read_adapter_feedback(self, include_phase=False):
        if self.odrive_adapter is None:
            raise FeedbackError("ODrive is not connected")
        return self.odrive_adapter.snapshot(
            include_phase=include_phase,
            max_capture_duration_s=(
                self.system_config["reference"]["maximum_feedback_capture_ms"] / 1000.0
            ),
        )

    def update_reference_display(self):
        if threading.current_thread() is not threading.main_thread():
            self.ui_message_queue.put(("reference",))
            return
        manager = self.reference_manager
        if manager.verified:
            mapping = manager.require_verified()
            text = (
                "Machine zero: VERIFIED\n"
                f"Physical 90\N{DEGREE SIGN} = machine 0\N{DEGREE SIGN} "
                f"({mapping.neutral_position_turns:.8f} session turns)"
            )
            colour = GREEN
        elif manager.confidence == ReferenceConfidence.FAULT:
            text = f"Machine zero: FAULT\n{manager.reason}"
            colour = RED
        else:
            text = f"Machine zero: SETUP REQUIRED\n{manager.reason}"
            colour = AMBER
        if hasattr(self, "reference_status_label"):
            self.reference_status_label.configure(text=text, text_color=colour)
        self.update_tare_display()
        self._refresh_motion_controls()

    def update_tare_display(self):
        if not hasattr(self, "tare_status_label"):
            return
        tare = self.empty_machine_tare
        if tare is None:
            self.tare_status_label.configure(
                text="Empty-machine tare: REQUIRED", text_color=AMBER
            )
            return
        mismatches = empty_machine_tare_identity_mismatches(
            tare,
            self.fixture_id_input.get(),
            self.calibration_id_input.get(),
            self.system_config,
        )
        if mismatches:
            self.tare_status_label.configure(
                text=(
                    "Empty-machine tare: IDENTITY MISMATCH\n"
                    + "\n".join(f"- {reason}" for reason in mismatches)
                ),
                text_color=RED,
            )
            return
        self.tare_status_label.configure(
            text=(
                "Empty-machine tare: VALID\n"
                f"{tare.captured_at} | offset {tare.offset_v_per_v:.12g} V/V"
            ),
            text_color=GREEN,
        )

    def _on_tare_identity_edit(self, _event=None):
        self.update_tare_display()
        self._refresh_motion_controls()

    def invalidate_empty_machine_tare(self, reason):
        if self.empty_machine_tare is None:
            return
        self.empty_machine_tare = None
        self.update_terminal(f"Empty-machine tare invalidated: {reason}.\n")
        self.update_tare_display()
        self._refresh_motion_controls()

    def _refresh_motion_controls(self):
        if not hasattr(self, "buttons"):
            return
        connected = self.odrive_adapter is not None
        verified = self.reference_manager.verified
        tare_ready = self.empty_machine_tare is not None and not (
            empty_machine_tare_identity_mismatches(
                self.empty_machine_tare,
                self.fixture_id_input.get(),
                self.calibration_id_input.get(),
                self.system_config,
            )
        )
        idle_ui = (
            not self.strain_test_active
            and self.motion_coordinator.owner is None
            and self.motion_coordinator.motor_idle_confirmed
        )
        self.reference_action_button.configure(
            state="normal" if connected and idle_ui else "disabled"
        )
        self.buttons[1].configure(
            state=(
                "normal"
                if connected and verified and tare_ready and idle_ui and not self.manual_mode.get()
                else "disabled"
            )
        )
        self.manual_mode_toggle.configure(state="normal" if connected and verified and idle_ui else "disabled")
        manual_enabled = connected and verified and idle_ui and self.manual_mode.get()
        manual_state = "normal" if manual_enabled else "disabled"
        for widget in (
            self.left_arrow,
            self.right_arrow,
            self.step_angle_input,
            self.manual_step_units_menu,
            self.mode_toggle,
        ):
            widget.configure(state=manual_state)
        if self.manual_step_units.get() == "Motor turns":
            self.mode_toggle.configure(state="disabled")
        self.neutral_button.configure(
            state="normal" if connected and verified and idle_ui else "disabled"
        )
        self.tare_button.configure(
            state="normal" if connected and verified and idle_ui else "disabled"
        )
        self.clear_session_tare_button.configure(
            state=(
                "normal"
                if self.empty_machine_tare is not None
                and not self.strain_test_active
                and self.motion_coordinator.owner is None
                else "disabled"
            )
        )

    def _start_feedback_monitor(self):
        self.monitor_stop_event.set()
        prior = self.monitor_thread
        if prior and prior.is_alive() and prior is not threading.current_thread():
            prior.join(timeout=1.0)
        self.monitor_stop_event = threading.Event()
        self.monitor_error = None
        self.monitor_thread = threading.Thread(
            target=self._feedback_monitor_loop,
            name="odrive-feedback-monitor",
            daemon=True,
        )
        self.monitor_thread.start()

    def _stop_feedback_monitor(self):
        self.monitor_stop_event.set()
        thread = self.monitor_thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.5)
        self.monitor_thread = None

    def _start_watchdog_if_enabled(self):
        cfg = self.system_config["reference"]
        if not cfg["watchdog_enabled"]:
            return
        if not cfg["watchdog_health_coupled_verified"]:
            raise RuntimeError(
                "Watchdog feeding is blocked until health-coupled operation is verified"
            )
        self.odrive_adapter.configure_watchdog(True, cfg["watchdog_timeout_s"])
        self.watchdog_stop_event = threading.Event()
        self.watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            name="odrive-watchdog-feed",
            daemon=True,
        )
        self.watchdog_thread.start()

    def _watchdog_loop(self):
        timeout_s = self.system_config["reference"]["watchdog_timeout_s"]
        interval = timeout_s / 3.0
        try:
            while not self.watchdog_stop_event.wait(interval):
                adapter = self.odrive_adapter
                if adapter is None:
                    return
                with self.control_health_lock:
                    health_age = time.monotonic() - self.last_control_health_monotonic
                if health_age >= timeout_s / 2.0:
                    raise RuntimeError(
                        f"control health heartbeat is stale ({health_age:.3f} s)"
                    )
                adapter.feed_watchdog()
        except Exception as exc:
            self.monitor_error = f"watchdog feed failed: {exc}"
            self.test_stop_event.set()
            self.neutral_stop_event.set()
            self.continuous_stop_event.set()
            self.motion_coordinator.request_stop()
            try:
                self.reference_manager.invalidate(self.monitor_error, fault=True)
            except Exception:
                pass
            adapter = self.odrive_adapter
            if adapter is not None:
                idle_result = adapter.request_idle(
                    self.system_config["reference"]["idle_confirmation_timeout_s"]
                )
                if idle_result.confirmed:
                    self.motion_coordinator.confirm_motor_idle()
            self.ui_message_queue.put(("terminal", f"SAFETY STOP: {self.monitor_error}\n"))
            self.ui_message_queue.put(("status", "FAULT / IDLE REQUESTED", RED))
            self.ui_message_queue.put(("reference",))

    def _stop_watchdog(self, disable=True):
        self.watchdog_stop_event.set()
        thread = getattr(self, "watchdog_thread", None)
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        if (
            disable
            and self.odrive_adapter is not None
            and self.system_config["reference"]["watchdog_enabled"]
        ):
            try:
                self.odrive_adapter.configure_watchdog(
                    False, self.system_config["reference"]["watchdog_timeout_s"]
                )
            except Exception as exc:
                self.update_terminal(f"Unable to disable ODrive watchdog: {exc}\n")
        self.watchdog_thread = None

    def _feedback_monitor_loop(self):
        reference_cfg = self.system_config["reference"]
        interval_s = reference_cfg["feedback_poll_interval_ms"] / 1000.0
        checkpoint_s = reference_cfg["checkpoint_interval_ms"] / 1000.0
        next_checkpoint = time.monotonic()
        try:
            while not self.monitor_stop_event.is_set():
                adapter = self.odrive_adapter
                if adapter is None:
                    return
                snapshot = adapter.snapshot(
                    include_phase=bool(reference_cfg["phase_recovery_enabled"]),
                    max_capture_duration_s=(
                        reference_cfg["maximum_feedback_capture_ms"] / 1000.0
                    ),
                )
                with self.feedback_lock:
                    self.latest_feedback = snapshot
                if snapshot.active_errors:
                    raise FeedbackError(f"ODrive active errors: {snapshot.active_errors}")
                expected_state = self.expected_axis_state
                if expected_state is not None and snapshot.current_state != expected_state:
                    raise FeedbackError(
                        f"ODrive left expected powered state {expected_state} "
                        f"(reported {snapshot.current_state})"
                    )
                expected_disarm = self.expected_disarm_reason
                if (
                    expected_state is not None
                    and expected_disarm is not None
                    and snapshot.disarm_reason != expected_disarm
                ):
                    raise FeedbackError(
                        "ODrive disarm reason changed during powered motion "
                        f"({expected_disarm} -> {snapshot.disarm_reason})"
                    )
                if self.expected_axis_state is None:
                    self._mark_control_health()
                if time.monotonic() >= next_checkpoint:
                    motion_state = (
                        MotionState.MOVING
                        if self.motion_coordinator.owner is not None
                        else MotionState.IDLE
                    )
                    self.reference_manager.write_checkpoint(
                        motion_state,
                        snapshot,
                        clean_shutdown=False,
                        extra={"motion_owner": self.motion_coordinator.owner},
                    )
                    next_checkpoint = time.monotonic() + checkpoint_s
                self.monitor_stop_event.wait(interval_s)
        except Exception as exc:
            self.monitor_error = str(exc)
            self.test_stop_event.set()
            self.neutral_stop_event.set()
            self.continuous_stop_event.set()
            self.motion_coordinator.request_stop()
            try:
                self.reference_manager.invalidate(
                    f"Feedback monitor failure: {exc}", fault=True
                )
            except Exception:
                pass
            adapter = self.odrive_adapter
            result = adapter.request_idle(
                self.system_config["reference"]["idle_confirmation_timeout_s"]
            ) if adapter else None
            if result and result.confirmed:
                self.motion_coordinator.confirm_motor_idle()
            self.ui_message_queue.put(("terminal", f"SAFETY STOP: {exc}\n"))
            self.ui_message_queue.put(("status", "FAULT / IDLE REQUESTED", RED))
            self.ui_message_queue.put(("reference",))

    def _mark_control_health(self):
        with self.control_health_lock:
            self.last_control_health_monotonic = time.monotonic()

    def _validate_live_motion_config(self):
        if self.odrive_adapter is None:
            raise RuntimeError("ODrive is not connected")
        cfg = self.system_config["reference"]
        return self.odrive_adapter.validate_motion_capabilities(
            cfg["required_pos_vel_mapper_scale"],
            cfg["pos_vel_mapper_scale_tolerance"],
        )

    def _revalidate_hardware_identity(self, require_reference=False):
        if self.odrive_adapter is None or self.hardware_fingerprint is None:
            raise RuntimeError("Connected ODrive identity is unavailable")
        live = self.odrive_adapter.fingerprint()
        matches, reason = hardware_fingerprints_match(
            self.hardware_fingerprint.to_dict(), live
        )
        if not matches:
            self.reference_manager.invalidate(
                f"Live ODrive identity/configuration changed: {reason}", fault=True
            )
            raise RuntimeError(reason)
        if require_reference:
            record = self.reference_manager.record
            mapping = self.reference_manager.require_verified()
            if record is None:
                raise ReferenceRequiredError("Saved machine-zero record is unavailable")
            configured_conversion = float(
                self.system_config["motion"]["afo_degrees_per_odrive_turn"]
            )
            if abs(record.afo_degrees_per_odrive_turn - configured_conversion) > 1e-12:
                reason = "Motion conversion changed since machine zero was established"
                self.reference_manager.invalidate(reason, fault=True)
                raise ReferenceRequiredError(reason)
            for saved in (
                record.hardware_fingerprint,
                mapping.hardware_fingerprint,
            ):
                matches, reason = hardware_fingerprints_match(saved, live)
                if not matches:
                    self.reference_manager.invalidate(reason, fault=True)
                    raise ReferenceRequiredError(reason)
        return live

    def _require_matching_reference_fixture(self, fixture_id):
        record = self.reference_manager.record
        if record is None:
            raise ReferenceRequiredError("Saved machine-zero record is unavailable")
        if str(fixture_id).strip() != record.fixture_id:
            reason = "Fixture ID does not match the saved machine-zero record"
            raise ReferenceRequiredError(reason)

    def configure_trajectory(self, speed_deg_s, acceleration_deg_s2):
        if self.odrive_adapter is None:
            raise RuntimeError("ODrive is not connected")
        motion = self.system_config["motion"]
        trajectory_velocity = afo_speed_to_odrive_turns_s(speed_deg_s, self.system_config)
        trajectory_acceleration = afo_acceleration_to_odrive_turns_s2(
            acceleration_deg_s2, self.system_config
        )
        controller_limit = trajectory_velocity * float(motion["controller_velocity_safety_multiplier"])
        self.odrive_adapter.configure_trajectory(
            trajectory_velocity,
            trajectory_acceleration,
            controller_limit,
            CONTROL_MODE_POSITION_CONTROL,
            INPUT_MODE_TRAP_TRAJ,
        )
        self.commanded_odrive_velocity = trajectory_velocity
        self.commanded_afo_acceleration = float(acceleration_deg_s2)

    def odrive_configuration_snapshot(self):
        if self.odrive_adapter is None:
            raise RuntimeError("ODrive is not connected")
        return self.odrive_adapter.read_only_report(include_phase=False)

    def safe_idle_motor(self, reason=None, invalidate_reference=False, token=None):
        active_token = self.motion_coordinator.active_token
        if token is not None and active_token not in (None, token):
            self.update_terminal(
                f"Ignored stale cleanup from {token.owner}; "
                f"motion is owned by {active_token.owner}.\n"
            )
            return None
        self.continuous_movement_active = False
        self.continuous_stop_event.set()
        if self.movement_timer:
            try:
                self.master.after_cancel(self.movement_timer)
            except Exception:
                pass
            self.movement_timer = None
        self.motion_coordinator.request_stop()
        self.expected_axis_state = None
        self.expected_disarm_reason = None
        if self.odrive_adapter is None:
            return None
        try:
            result = self.odrive_adapter.request_idle(
                self.system_config["reference"]["idle_confirmation_timeout_s"]
            )
            self.motion_phase = "idle"
            if result.confirmed:
                self.motion_coordinator.confirm_motor_idle()
            if invalidate_reference or not result.confirmed:
                self.reference_manager.invalidate(
                    reason or result.detail,
                    fault=not result.confirmed,
                )
            try:
                snapshot = self._read_adapter_feedback()
                with self.feedback_lock:
                    self.latest_feedback = snapshot
                self.reference_manager.write_checkpoint(
                    MotionState.IDLE if result.confirmed else MotionState.FAULT,
                    snapshot,
                    clean_shutdown=False,
                    extra={"idle_result": result.__dict__, "reason": reason},
                )
            except Exception as checkpoint_exc:
                self.update_terminal(f"Unable to persist idle checkpoint: {checkpoint_exc}\n")
                if result.confirmed:
                    self.reference_manager.invalidate(
                        f"Runtime-state write failed: {checkpoint_exc}", fault=True
                    )
            if reason:
                self.update_terminal(
                    f"Motor idle {'confirmed' if result.confirmed else 'NOT CONFIRMED'}: {reason}\n"
                )
            if threading.current_thread() is threading.main_thread():
                self.update_reference_display()
            else:
                self.ui_message_queue.put(("reference",))
            return result
        except Exception as exc:
            self.update_terminal(f"Unable to confirm ODrive idle state: {exc}\n")
            try:
                self.reference_manager.invalidate(
                    f"Idle confirmation failed: {exc}", fault=True
                )
            except Exception:
                pass
            if threading.current_thread() is threading.main_thread():
                self.update_reference_display()
            else:
                self.ui_message_queue.put(("reference",))
            return None

    def enter_closed_loop(self, token, allow_unreferenced=False):
        if self.odrive_adapter is None:
            raise RuntimeError("ODrive is not connected")
        if not allow_unreferenced:
            self.reference_manager.require_verified()
        self._validate_live_motion_config()
        self._revalidate_hardware_identity(require_reference=not allow_unreferenced)
        self.motion_coordinator.assert_active(token)
        snapshot = self.get_feedback()
        if snapshot.active_errors:
            raise RuntimeError(f"ODrive has active errors: {snapshot.active_errors}")
        # A durable 'arming' checkpoint must succeed before any enable request.
        self.reference_manager.write_checkpoint(
            MotionState.ARMING,
            snapshot,
            clean_shutdown=False,
            extra={"motion_owner": token.owner},
        )
        cfg = self.system_config["reference"]
        self.motion_coordinator.submit_command(
            token,
            lambda: self.odrive_adapter.request_closed_loop_holding_current(
                cancelled=lambda: self._motion_cancelled(token),
                expected_mapper_scale=cfg["required_pos_vel_mapper_scale"],
                mapper_scale_tolerance=cfg["pos_vel_mapper_scale_tolerance"],
            ),
        )
        armed = self.odrive_adapter.wait_for_state(
            AXIS_STATE_CLOSED_LOOP_CONTROL,
            cancelled=lambda: self._motion_cancelled(token),
            timeout_s=2.0,
            progress=self._mark_control_health,
        )
        self.motion_coordinator.assert_active(token)
        self.expected_disarm_reason = armed.disarm_reason
        self.expected_axis_state = int(AXIS_STATE_CLOSED_LOOP_CONTROL)
        self._mark_control_health()
        return armed

    def _motion_cancelled(self, token):
        try:
            self.motion_coordinator.assert_active(token)
            return False
        except MotionConflictError:
            return True

    def _submit_position(self, token, target_turns):
        if self.odrive_adapter is None:
            raise RuntimeError("ODrive disconnected during motion")
        self.motion_coordinator.submit_command(
            token,
            lambda: self.odrive_adapter.command_position(target_turns),
        )
        self._mark_control_health()

    def _powered_wait_requirements(self):
        return {
            "expected_state": int(AXIS_STATE_CLOSED_LOOP_CONTROL),
            "expected_disarm_reason": self.expected_disarm_reason,
            "progress": self._mark_control_health,
        }

    def connect_system(self):
        active_threads = [
            getattr(self, name, None)
            for name in (
                "strain_thread", "data_collection_thread", "neutral_thread",
                "manual_step_thread", "continuous_thread", "recovery_thread",
            )
        ]
        if (
            self.strain_test_active
            or self.motion_coordinator.owner is not None
            or any(thread and thread.is_alive() for thread in active_threads)
        ):
            self.update_terminal("Cannot reconnect until all motion workers have exited.\n")
            return
        if self.startup_block_reason or not self.process_lock.held:
            self.update_terminal(
                f"Hardware controls are blocked: {self.startup_block_reason or 'process lock unavailable'}\n"
            )
            return
        self.clear_terminal()
        if self.odrive_adapter is not None:
            self.safe_idle_motor("reconnect")
            self._stop_feedback_monitor()
        self.buttons[1].configure(state="disabled")
        self.neutral_button.configure(state="disabled")
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

            self.odrive_adapter = ODriveAdapter(
                self.odrive_controller,
                self.system_config["hardware"]["odrive_axis"],
                AXIS_STATE_IDLE,
                AXIS_STATE_CLOSED_LOOP_CONTROL,
            )
            self._validate_live_motion_config()
            if self.system_config["controller"]["apply_controller_gains"]:
                raise RuntimeError(
                    "Runtime controller-gain writes are disabled in the machine-zero build"
                )
            self.hardware_fingerprint = self.odrive_adapter.fingerprint()
            idle_result = self.odrive_adapter.request_idle(
                self.system_config["reference"]["idle_confirmation_timeout_s"]
            )
            if not idle_result.confirmed:
                raise RuntimeError(idle_result.detail)
            self.motion_coordinator.confirm_motor_idle()
            initial = self._read_adapter_feedback(
                include_phase=bool(self.system_config["reference"]["phase_recovery_enabled"])
            )
            with self.feedback_lock:
                self.latest_feedback = initial

            record = self.reference_manager.record
            if record is not None:
                identity_matches, identity_reason = hardware_fingerprints_match(
                    record.hardware_fingerprint, self.hardware_fingerprint
                )
                if not identity_matches:
                    self.reference_manager.invalidate(
                        f"{identity_reason}; physical recovery required"
                    )
                else:
                    checkpoint = self.state_store.load_checkpoint()
                    if checkpoint:
                        valid, reason, mapping = evaluate_continuity(
                            checkpoint,
                            initial,
                            self.hardware_fingerprint,
                            self.system_config,
                            reference_record=record,
                        )
                        if valid and mapping is not None:
                            self.reference_manager.mapping = mapping
                            self.reference_manager.confidence = ReferenceConfidence.VERIFIED
                            self.reference_manager.reason = reason
                        else:
                            phase_valid, phase_reason = self.reference_manager.recover_from_phase(
                                initial, self.hardware_fingerprint
                            )
                            if not phase_valid:
                                self.reference_manager.invalidate(
                                    f"{reason}; {phase_reason}"
                                )
                    else:
                        phase_valid, phase_reason = self.reference_manager.recover_from_phase(
                            initial, self.hardware_fingerprint
                        )
                        if not phase_valid:
                            self.reference_manager.invalidate(
                                f"No continuity checkpoint; {phase_reason}"
                            )
            self._start_feedback_monitor()
            self._start_watchdog_if_enabled()

            if initial.active_errors:
                self.reference_manager.invalidate(
                    f"ODrive has active errors ({initial.active_errors}); clear and reconnect",
                    fault=True,
                )

            self.update_terminal(
                f"Connected to ODrive S1\nSerial number: {serial_number}\n"
                f"Axis active errors: {initial.active_errors}\n"
                "No errors were cleared and no encoder/controller configuration was saved.\n"
                f"Runtime state: {self.state_store.directory}\n"
            )
            if self.reference_manager.verified:
                self.set_status("CONNECTED / MACHINE ZERO VERIFIED", GREEN)
            else:
                self.set_status("CONNECTED / MACHINE-ZERO SETUP REQUIRED", AMBER)
                self.update_terminal(
                    "Normal tests and manual motion are blocked until machine zero is verified.\n"
                )
            self.update_reference_display()
        except (concurrent.futures.TimeoutError, TimeoutError) as exc:
            self._stop_watchdog(disable=True)
            self._stop_feedback_monitor()
            self.odrive_controller = None
            self.odrive_adapter = None
            self.update_terminal(f"Connection timed out: {exc}\n")
            self.set_status("DISCONNECTED", RED)
        except Exception as exc:
            self.safe_idle_motor("connection/configuration error")
            self._stop_watchdog(disable=True)
            self._stop_feedback_monitor()
            self.odrive_controller = None
            self.odrive_adapter = None
            self.update_terminal(f"Error connecting to ODrive: {exc}\n")
            self.set_status("ERROR", RED)
        finally:
            self.update_reference_display()

    def disconnect_odrive(self):
        """Disconnect from ODrive safely"""
        if self.motion_coordinator.owner is not None:
            self.stop_logging()
            self.update_terminal(
                "Disconnect requested during motion. Wait for the owning worker to exit, "
                "then disconnect again.\n"
            )
            return
        try:
            if self.odrive_controller:
                self.neutral_stop_event.set()
                self.continuous_stop_event.set()
                idle_result = self.safe_idle_motor("manual disconnect")
                self._stop_watchdog(disable=True)
                self._stop_feedback_monitor()
                continuity_saved = False
                checkpoint_snapshot = None
                try:
                    if idle_result and idle_result.confirmed:
                        checkpoint_snapshot = self._read_adapter_feedback()
                    continuity_saved = persist_idle_continuity_checkpoint(
                        self.reference_manager,
                        checkpoint_snapshot,
                        bool(idle_result and idle_result.confirmed),
                        "manual disconnect",
                        expected_idle_state=int(AXIS_STATE_IDLE),
                    )
                except Exception as exc:
                    self.update_terminal(
                        f"Unable to save manual-disconnect continuity evidence: {exc}\n"
                    )
                self.odrive_controller = None
                self.odrive_adapter = None
                self.hardware_fingerprint = None
                with self.feedback_lock:
                    self.latest_feedback = None
                self.reference_manager.mapping = None
                self.reference_manager.confidence = ReferenceConfidence.RECOVERY_REQUIRED
                self.reference_manager.reason = (
                    "ODrive disconnected; continuity must be proven on reconnect"
                )
                self.buttons[1].configure(state="disabled")
                self.neutral_button.configure(state="disabled")
                self.set_status("DISCONNECTED", RED)
                self.update_terminal(
                    "ODrive disconnected and set to idle state. "
                    f"Clean continuity checkpoint: {'saved' if continuity_saved else 'not saved'}.\n"
                )
                self.update_reference_display()
        except Exception as e:
            self.update_terminal(f"Error disconnecting ODrive: {e}\n")

    def stop_logging(self):
        if self.rotary_calibration is not None and not self.rotary_calibration.closed:
            self.rotary_calibration.cancel_capture("Operator Stop/Escape")
        was_active = self.strain_test_active
        if was_active:
            self.operator_stop_requested = True
        self.test_stop_event.set()
        self.neutral_stop_event.set()
        self.continuous_stop_event.set()
        idle_result = self.safe_idle_motor("operator stop")
        self.set_status(
            "STOPPED / IDLE" if idle_result and idle_result.confirmed else "STOPPED / IDLE UNCONFIRMED",
            AMBER if idle_result and idle_result.confirmed else RED,
        )
        if was_active:
            self.update_terminal("Test stop requested; the data file will be finalized as aborted.\n")
        else:
            self.update_terminal(
                "Motor idle confirmed.\n"
                if idle_result and idle_result.confirmed
                else "Motor idle could not be confirmed.\n"
            )
            


    def reset_test_fields(self):
        """Clear run-specific inputs without changing session safety state."""
        if self.strain_test_active or self.motion_coordinator.owner is not None:
            CTkMessagebox(
                title="Reset Blocked",
                message="Stop the active movement before resetting test fields.",
            )
            return
        for entry in (
            self.file_name_input, self.cycles_input, self.speed_input,
            self.acceleration_input, self.min_angle_input, self.max_angle_input,
            self.afo_id_input,
        ):
            entry.delete(0, ctk.END)
        self.protocol_var.set("Custom")
        self.loaded_preset_key = "custom"
        self.preset_loaded_at = datetime.now().astimezone().isoformat(
            timespec="milliseconds"
        )
        self.update_parameter_summary()
        self.update_tare_display()
        self._refresh_motion_controls()
        self.update_terminal(
            "Run-specific test fields reset. Machine zero and session tare were preserved.\n"
        )

    def clear_session_tare(self):
        """Explicitly discard only the current session tare after confirmation."""
        if self.empty_machine_tare is None:
            return
        if self.strain_test_active or self.motion_coordinator.owner is not None:
            CTkMessagebox(
                title="Clear Session Tare Blocked",
                message="Stop the active movement before clearing the session tare.",
            )
            return
        confirmation = CTkMessagebox(
            title="Clear Session Tare",
            message=(
                "This will discard only the stored empty-machine tare. Machine zero, "
                "connection, operator, fixture, calibration, and all test fields will remain "
                "unchanged.\n\n"
                "Before the next run, confirm/return to verified machine zero and capture a "
                "new empty-machine tare with the AFO and removable loads removed."
            ),
            icon="warning",
            option_1="Cancel",
            option_2="Clear Session Tare",
        )
        if confirmation.get() != "Clear Session Tare":
            return
        self.invalidate_empty_machine_tare("session tare cleared by operator")


    def clear_terminal(self):
        self.terminal.delete(1.0, ctk.END)
        self.terminal.update()


    def update_terminal(self, message):
        if threading.current_thread() is not threading.main_thread():
            self.ui_message_queue.put(("terminal", message))
            return
        self.terminal.insert(ctk.END, message)
        self.terminal.see(ctk.END)  # Scroll to the end of the text

    def show_resizable_confirmation(self, title, message, confirm_text="Start"):
        """Show a modal confirmation whose actions remain visible on small screens."""
        result = {"confirmed": False}
        dialog = ctk.CTkToplevel(self.master)
        dialog.title(title)
        dialog.configure(fg_color=BG)
        dialog.transient(self.master)
        dialog.resizable(True, True)

        screen_width = dialog.winfo_screenwidth()
        screen_height = dialog.winfo_screenheight()
        width = min(560, max(380, screen_width - 80))
        height = min(390, max(280, screen_height - 120))
        x = max(20, (screen_width - width) // 2)
        y = max(20, (screen_height - height) // 2)
        dialog.geometry(f"{width}x{height}+{x}+{y}")
        dialog.minsize(min(420, width), min(280, height))
        dialog.grid_columnconfigure(0, weight=1)
        dialog.grid_rowconfigure(0, weight=1)

        summary = ctk.CTkTextbox(
            dialog,
            wrap="word",
            font=("Arial", 14),
            fg_color=PANEL,
            text_color=TEXT,
            corner_radius=8,
        )
        summary.grid(row=0, column=0, sticky="nsew", padx=14, pady=(14, 8))
        summary.insert("1.0", message)
        summary.configure(state="disabled")

        actions = ctk.CTkFrame(dialog, fg_color="transparent")
        actions.grid(row=1, column=0, sticky="ew", padx=14, pady=(0, 14))
        actions.grid_columnconfigure((0, 1), weight=1)

        def finish(confirmed):
            result["confirmed"] = confirmed
            try:
                dialog.grab_release()
            except tk.TclError:
                pass
            dialog.destroy()

        ctk.CTkButton(
            actions,
            text="Cancel",
            command=lambda: finish(False),
            fg_color=MUTED,
            hover_color="#475569",
            height=38,
        ).grid(row=0, column=0, sticky="ew", padx=(0, 6))
        start_button = ctk.CTkButton(
            actions,
            text=confirm_text,
            command=lambda: finish(True),
            fg_color=GREEN,
            hover_color="#15803d",
            height=38,
        )
        start_button.grid(row=0, column=1, sticky="ew", padx=(6, 0))

        dialog.protocol("WM_DELETE_WINDOW", lambda: finish(False))
        dialog.bind("<Escape>", lambda _event: finish(False))
        dialog.bind("<Return>", lambda _event: finish(True))
        dialog.update_idletasks()
        dialog.grab_set()
        start_button.focus_set()
        self.master.wait_window(dialog)
        return result["confirmed"]

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
                    self._refresh_motion_controls()
                    self.manual_mode_toggle.configure(state=item[1])
                elif item[0] == "neutral_finished":
                    self.neutral_motion_active = False
                    self._refresh_motion_controls()
                elif item[0] == "reference":
                    self.update_reference_display()
                elif item[0] == "recovery_motion_finished":
                    self.update_reference_display()
                    if self.recovery_window is not None and self.recovery_window.winfo_exists():
                        for button in self.recovery_jog_buttons:
                            button.configure(state="normal")
                        self.recovery_set_button.configure(state="normal")
                        self.recovery_position_label.configure(
                            text=self._recovery_position_text()
                        )
        except queue.Empty:
            pass

        try:
            while True:
                angle, torque = self.plot_data_queue.get_nowait()
                self.update_plot_data(angle, torque)
        except queue.Empty:
            pass

        self.update_manual_position_display()
        try:
            self.master.after(50, self._drain_ui_queues)
        except tk.TclError:
            pass

    def _recovery_position_text(self):
        try:
            snapshot = self.get_feedback()
            return (
                f"Session position: {snapshot.position_turns:.8f} turns | "
                f"Jog path used: {self.recovery_cumulative_deg:.2f} deg"
            )
        except Exception as exc:
            return f"Feedback unavailable: {exc}"

    def open_rotary_calibration(self):
        if self.startup_block_reason:
            self.update_terminal(f"Sensor calibration blocked: {self.startup_block_reason}\n")
            return
        if self.rotary_calibration is not None and not self.rotary_calibration.closed:
            self.rotary_calibration.window.lift()
            return
        from east_rotary_gui import RotaryCalibrationWindow
        self.rotary_calibration = RotaryCalibrationWindow(self.master, self)

    def open_reference_recovery(self):
        if self.odrive_adapter is None:
            self.update_terminal("Connect the ODrive before machine-zero setup.\n")
            return
        if self.motion_coordinator.owner is not None or self.strain_test_active:
            self.update_terminal("Machine-zero setup is blocked while motion is active.\n")
            return
        if self.recovery_window is not None and self.recovery_window.winfo_exists():
            self.recovery_window.lift()
            return
        try:
            self._validate_live_motion_config()
            self._revalidate_hardware_identity()
            snapshot = self.get_feedback()
        except Exception as exc:
            self.update_terminal(f"Cannot start machine-zero setup: {exc}\n")
            return
        self.recovery_origin_turns = snapshot.position_turns
        self.recovery_cumulative_deg = 0.0
        self.recovery_session_acknowledged = False

        window = ctk.CTkToplevel(self.master)
        self.recovery_window = window
        window.title("EAST Machine Zero Setup")
        window.geometry("650x540")
        window.minsize(590, 500)
        window.configure(fg_color=BG)
        window.transient(self.master)
        window.protocol("WM_DELETE_WINDOW", self.close_reference_recovery)
        window.bind("<Escape>", lambda _event: self.stop_logging())

        panel = ctk.CTkFrame(window, fg_color=PANEL, corner_radius=8)
        panel.pack(fill="both", expand=True, padx=18, pady=18)
        ctk.CTkLabel(
            panel,
            text="Machine Zero \N{EM DASH} Fixture at 90\N{DEGREE SIGN}",
            font=("Arial", 20, "bold"),
            text_color=TEXT,
        ).pack(anchor="w", padx=16, pady=(14, 5))
        ctk.CTkLabel(
            panel,
            text=(
                "Use the supplied square to confirm that the moving fixture is at 90\N{DEGREE SIGN} "
                "to the fixed machine reference. Use the slow Jog Left and Jog Right controls "
                "to make small adjustments until the fixture is aligned with the square. Then "
                "select 'Set Machine Zero \N{EM DASH} Fixture at 90\N{DEGREE SIGN}'. This defines "
                "machine angle 0\N{DEGREE SIGN} for the current ODrive power session."
            ),
            font=("Arial", 12),
            text_color=TEXT,
            wraplength=540,
            justify="left",
        ).pack(anchor="w", padx=16, pady=(0, 10))
        self.recovery_position_label = ctk.CTkLabel(
            panel,
            text=self._recovery_position_text(),
            font=("Arial", 12, "bold"),
            text_color=MUTED,
        )
        self.recovery_position_label.pack(anchor="w", padx=16, pady=5)

        self.recovery_acknowledgement = ctk.BooleanVar(value=False)
        self.recovery_ack_checkbox = ctk.CTkCheckBox(
            panel,
            text=(
                "For this setup session: fixture is clear, E-stop is accessible, "
                "and I am observing the mechanism"
            ),
            variable=self.recovery_acknowledgement,
        )
        self.recovery_ack_checkbox.pack(anchor="w", padx=16, pady=8)

        jog_frame = ctk.CTkFrame(panel, fg_color=PANEL_SOFT, corner_radius=6)
        jog_frame.pack(fill="x", padx=16, pady=6)
        step = self.system_config["reference"]["recovery_jog_step_deg"]
        left = ctk.CTkButton(
            jog_frame,
            text=f"Jog Left ({step:g}\N{DEGREE SIGN})",
            command=lambda: self.recovery_jog(-1),
            fg_color=MUTED,
            hover_color="#475569",
        )
        right = ctk.CTkButton(
            jog_frame,
            text=f"Jog Right ({step:g}\N{DEGREE SIGN})",
            command=lambda: self.recovery_jog(1),
            fg_color=MUTED,
            hover_color="#475569",
        )
        left.pack(side="left", fill="x", expand=True, padx=6, pady=8)
        right.pack(side="left", fill="x", expand=True, padx=6, pady=8)
        self.recovery_jog_buttons = (left, right)

        self.recovery_set_button = ctk.CTkButton(
            panel,
            text="Set Machine Zero \N{EM DASH} Fixture at 90\N{DEGREE SIGN}",
            command=self.set_machine_zero,
            fg_color="#0f766e",
            hover_color="#115e59",
            height=36,
        )
        self.recovery_set_button.pack(fill="x", padx=16, pady=(10, 5))

        if self.system_config["reference"]["assisted_measured_angle_enabled"]:
            measured = ctk.CTkFrame(panel, fg_color="transparent")
            measured.pack(fill="x", padx=16, pady=5)
            self.recovery_angle_entry = ctk.CTkEntry(
                measured, placeholder_text="Measured displacement from machine zero (deg)"
            )
            self.recovery_uncertainty_entry = ctk.CTkEntry(
                measured, placeholder_text="Uncertainty (deg)"
            )
            self.recovery_angle_entry.pack(side="left", fill="x", expand=True, padx=(0, 5))
            self.recovery_uncertainty_entry.pack(side="left", fill="x", expand=True, padx=5)
            ctk.CTkButton(
                measured,
                text="Use Measurement",
                command=self.set_neutral_from_measurement,
                width=130,
            ).pack(side="left", padx=(5, 0))

        ctk.CTkButton(
            panel,
            text="Cancel / Keep Motion Blocked",
            command=self.close_reference_recovery,
            fg_color="#94a3b8",
            hover_color=MUTED,
        ).pack(fill="x", padx=16, pady=(5, 14))

    def close_reference_recovery(self):
        self.neutral_stop_event.set()
        if self.motion_coordinator.owner == "reference-recovery-jog":
            self.safe_idle_motor("machine-zero setup closed")
        if self.recovery_window is not None:
            try:
                self.recovery_window.destroy()
            except tk.TclError:
                pass
        self.recovery_window = None
        self.recovery_session_acknowledged = False

    def _accept_recovery_session_acknowledgement(self):
        if self.recovery_session_acknowledged:
            return True
        if not self.recovery_acknowledgement.get():
            return False
        self.recovery_session_acknowledged = True
        self.recovery_ack_checkbox.configure(state="disabled")
        return True

    def recovery_jog(self, direction):
        if not self._accept_recovery_session_acknowledgement():
            CTkMessagebox(
                title="Acknowledgement Required",
                message="Confirm the recovery safety acknowledgement before jogging.",
            )
            return
        cfg = self.system_config["reference"]
        next_total = self.recovery_cumulative_deg + abs(cfg["recovery_jog_step_deg"])
        if next_total > cfg["recovery_jog_maximum_cumulative_deg"] + 1e-9:
            self.update_terminal("Recovery jog blocked at its cumulative travel limit.\n")
            return
        token = None
        try:
            token = self.motion_coordinator.acquire("reference-recovery-jog")
            self.invalidate_empty_machine_tare("machine-zero fixture adjustment")
            self.reference_manager.invalidate("Unreferenced recovery jog performed")
        except Exception as exc:
            if token is not None:
                self.motion_coordinator.release(token)
            self.update_terminal(f"Recovery jog blocked: {exc}\n")
            return
        for button in self.recovery_jog_buttons:
            button.configure(state="disabled")
        self.recovery_set_button.configure(state="disabled")
        self.recovery_thread = threading.Thread(
            target=self._recovery_jog_worker,
            args=(token, direction, next_total),
            name="reference-recovery-jog",
            daemon=True,
        )
        self.recovery_thread.start()

    def _recovery_jog_worker(self, token, direction, next_total):
        try:
            cfg = self.system_config["reference"]
            self.configure_trajectory(
                cfg["recovery_speed_deg_s"], cfg["recovery_acceleration_deg_s2"]
            )
            self.enter_closed_loop(token, allow_unreferenced=True)
            snapshot = self.get_feedback()
            step_turns = afo_degrees_to_odrive_turns(
                direction * cfg["recovery_jog_step_deg"], self.system_config
            )
            target = snapshot.position_turns + step_turns
            origin_limit = afo_degrees_to_odrive_turns(
                cfg["recovery_jog_maximum_cumulative_deg"], self.system_config
            )
            if abs(target - self.recovery_origin_turns) > origin_limit + 1e-9:
                raise ReferenceError("Recovery target exceeds bounded travel")
            self.reference_manager.write_checkpoint(
                MotionState.MOVING,
                snapshot,
                clean_shutdown=False,
                extra={"phase": "unreferenced_recovery_jog", "target_turns": target},
            )
            self._submit_position(token, target)
            wait_for_settle(
                self.get_feedback,
                target_turns=target,
                tolerance_turns=afo_degrees_to_odrive_turns(
                    self.system_config["motion"]["position_tolerance_deg"], self.system_config
                ),
                velocity_limit_turns_s=afo_speed_to_odrive_turns_s(
                    cfg["settle_velocity_limit_deg_s"], self.system_config
                ),
                dwell_s=cfg["settle_dwell_ms"] / 1000.0,
                timeout_s=motion_timeout_seconds(
                    cfg["recovery_jog_step_deg"],
                    cfg["recovery_speed_deg_s"],
                    cfg["recovery_acceleration_deg_s2"],
                    self.system_config,
                ),
                stale_after_s=cfg["feedback_stale_after_ms"] / 1000.0,
                cancelled=lambda: self._motion_cancelled(token),
                **self._powered_wait_requirements(),
            )
            self.recovery_cumulative_deg = next_total
            self.update_terminal(
                f"Recovery jog complete ({self.recovery_cumulative_deg:.2f} deg path used).\n"
            )
        except Exception as exc:
            self.update_terminal(f"Recovery jog failed: {exc}\n")
        finally:
            self.safe_idle_motor("recovery jog complete", token=token)
            self.motion_coordinator.release(token)
            self.ui_message_queue.put(("recovery_motion_finished",))

    def set_machine_zero(self):
        if not self._accept_recovery_session_acknowledgement():
            CTkMessagebox(
                title="Acknowledgement Required",
                message="Confirm the safety acknowledgement before setting machine zero.",
            )
            return
        confirmation = CTkMessagebox(
            title="Set Machine Zero",
            message=(
                "Confirm the empty moving fixture is mechanically aligned at physical "
                "90\N{DEGREE SIGN} using the supplied square.\n\n"
                "This records the current encoder position as machine angle 0\N{DEGREE SIGN}. "
                "The motor will not move."
            ),
            icon="question",
            option_1="Cancel",
            option_2="Set Machine Zero",
        )
        if confirmation.get() != "Set Machine Zero":
            return
        try:
            if self.motion_coordinator.owner is not None:
                raise MotionConflictError("Wait for recovery motion to finish")
            self._validate_live_motion_config()
            live_fingerprint = self._revalidate_hardware_identity()
            idle_result = self.odrive_adapter.request_idle(
                self.system_config["reference"]["idle_confirmation_timeout_s"]
            )
            if not idle_result.confirmed:
                raise RuntimeError("ODrive idle could not be confirmed")
            self.motion_coordinator.confirm_motor_idle()
            snapshot = self._read_adapter_feedback(
                include_phase=bool(self.system_config["reference"]["phase_recovery_enabled"])
            )
            with self.feedback_lock:
                self.latest_feedback = snapshot
            self.reference_manager.establish_at_physical_neutral(
                snapshot,
                live_fingerprint,
                self.operator_input.get(),
                self.fixture_id_input.get(),
                acknowledgement=True,
            )
            self.invalidate_empty_machine_tare("machine zero was re-established")
            self.update_terminal(
                "Machine zero saved: fixture mechanically aligned at physical 90 degrees; "
                "machine angle defined as 0 degrees.\n"
            )
            self.set_status("CONNECTED / MACHINE ZERO VERIFIED", GREEN)
            self.update_reference_display()
            self.close_reference_recovery()
        except Exception as exc:
            self.update_terminal(f"Unable to set machine zero: {exc}\n")
            CTkMessagebox(title="Machine Zero Not Set", message=str(exc))

    def set_neutral_from_measurement(self):
        try:
            if not self._accept_recovery_session_acknowledgement():
                raise ReferenceError("Machine-zero setup acknowledgement is required")
            if self.motion_coordinator.owner is not None:
                raise MotionConflictError("Wait for all motion workers to finish")
            self._validate_live_motion_config()
            live_fingerprint = self._revalidate_hardware_identity()
            idle_result = self.odrive_adapter.request_idle(
                self.system_config["reference"]["idle_confirmation_timeout_s"]
            )
            if not idle_result.confirmed:
                raise RuntimeError("ODrive idle could not be confirmed")
            self.motion_coordinator.confirm_motor_idle()
            snapshot = self._read_adapter_feedback(include_phase=True)
            with self.feedback_lock:
                self.latest_feedback = snapshot
            self.reference_manager.verify_measured_displacement(
                snapshot,
                live_fingerprint,
                float(self.recovery_angle_entry.get()),
                float(self.recovery_uncertainty_entry.get()),
                self.operator_input.get(),
                self.fixture_id_input.get(),
                True,
            )
            self.update_terminal(
                "Measured-displacement machine-zero recovery completed; "
                "no motor motion was commanded.\n"
            )
            self.update_reference_display()
            self.close_reference_recovery()
        except Exception as exc:
            self.update_terminal(f"Measured-angle recovery failed: {exc}\n")

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
                # Latch cancellation before stopping the motor. Every worker must
                # exit before clean continuity evidence can be written.
                if self.strain_test_active:
                    self.operator_stop_requested = True
                self.test_stop_event.set()
                self.neutral_stop_event.set()
                self.continuous_stop_event.set()

                idle_result = None
                if self.odrive_adapter is not None:
                    idle_result = self.safe_idle_motor("application shutdown")
                    self._stop_watchdog(disable=True)

                deadline = time.monotonic() + 3.0
                worker_names = (
                    "strain_thread",
                    "data_collection_thread",
                    "neutral_thread",
                    "manual_step_thread",
                    "continuous_thread",
                    "recovery_thread",
                )
                for thread_name in worker_names:
                    thread = getattr(self, thread_name, None)
                    if thread and thread.is_alive() and thread is not threading.current_thread():
                        thread.join(timeout=max(0.0, deadline - time.monotonic()))
                live_workers = [
                    name
                    for name in worker_names
                    if (thread := getattr(self, name, None)) is not None and thread.is_alive()
                ]
                if live_workers:
                    print(
                        "Shutdown continued with worker(s) still active: "
                        + ", ".join(live_workers)
                    )
                
                if self.rotary_calibration is not None:
                    self.rotary_calibration.shutdown()

                # Close the plot window safely
                self.close_plot_window()
                
                # Confirm idle and capture fresh feedback before writing clean continuity evidence.
                if self.odrive_adapter is not None:
                    self._stop_feedback_monitor()
                    snapshot = None
                    continuity_ready = bool(
                        idle_result
                        and idle_result.confirmed
                        and not live_workers
                        and self.motion_coordinator.owner is None
                    )
                    try:
                        if continuity_ready:
                            snapshot = self._read_adapter_feedback()
                        persist_idle_continuity_checkpoint(
                            self.reference_manager,
                            snapshot,
                            continuity_ready,
                            "application shutdown",
                            expected_idle_state=int(AXIS_STATE_IDLE),
                        )
                    except Exception as checkpoint_exc:
                        print(f"Unable to save shutdown continuity evidence: {checkpoint_exc}")
                    self.odrive_controller = None
                    self.odrive_adapter = None
                
                # Disconnect from Phidget if connected
                if hasattr(self, 'voltage_ratio_input') and self.voltage_ratio_input:
                    try:
                        self.voltage_ratio_input.close()
                    except Exception:
                        pass
                
                # Destroy the main window
                self.process_lock.release()
                self.master.quit()
                self.master.destroy()
                
            except Exception as e:
                print(f"Error during shutdown: {e}")
                # Force quit if there's an error
                self.process_lock.release()
                self.master.quit()
                self.master.destroy()

    def start_strain_test(self):
        """Start the strain test with the current motor settings"""
        # Clear the embedded plot for a new run.
        global angle_data, torque_data
        if plot_window_open:
            angle_data = []
            torque_data = []
            self.update_plot()
        self.create_plot_window()

        if self.odrive_controller is None:
            self.update_terminal("No serial connection established. Please connect ODrive first.\n")
            return

        if not self.reference_manager.verified:
            self.update_terminal(
                f"Machine zero is not verified: {self.reference_manager.reason}\n"
                "Open Machine Zero setup and align the fixture at physical 90 degrees.\n"
            )
            return

        try:
            snapshot = self.get_feedback()
            current_angle = self.reference_manager.angle_from_position(snapshot.position_turns)
        except ReferenceError as exc:
            self.update_terminal(f"Cannot verify machine zero before test: {exc}\n")
            return
        if abs(current_angle) > self.system_config["motion"]["position_tolerance_deg"]:
            self.update_terminal(
                "Fixture is away from verified machine zero. Press "
                "'Return to Machine Zero \N{EM DASH} 90\N{DEGREE SIGN}' before starting. "
                "Do not redefine machine zero.\n"
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
        try:
            self._require_matching_reference_fixture(parameters.fixture_id)
        except ReferenceError as exc:
            self.update_terminal(f"Test blocked: {exc}\n")
            return
        tare_valid, tare_reason = validate_empty_machine_tare(
            self.empty_machine_tare, parameters, self.system_config
        )
        if not tare_valid:
            self.update_terminal(f"Test blocked: {tare_reason}\n")
            CTkMessagebox(title="Empty-Machine Tare Required", message=tare_reason)
            return

        baseline_test = parameters.test_type == "empty_machine_baseline"
        test_type_label = {
            "afo_test": "AFO TEST",
            "empty_machine_baseline": "EMPTY-MACHINE BASELINE",
            "custom": "CUSTOM",
        }[parameters.test_type]
        active_protocol = self.active_protocol_name()
        safety_line = (
            "Remove the AFO and all removable loads before starting."
            if baseline_test
            else "Confirm the correct AFO is installed."
        )
        afo_line = "" if baseline_test else f"AFO: {parameters.afo_id}\n"
        confirmation_message = (
            f"{test_type_label}  |  {active_protocol}\n\n"
            f"{safety_line}\n\n"
            f"{afo_line}"
            f"Motion: -{parameters.min_angle_deg:g}\N{DEGREE SIGN} to "
            f"+{parameters.max_angle_deg:g}\N{DEGREE SIGN}  |  "
            f"{parameters.commanded_afo_speed_deg_s:g}\N{DEGREE SIGN}/s  |  "
            f"{parameters.commanded_afo_acceleration_deg_s2:g}"
            f"\N{DEGREE SIGN}/s\N{SUPERSCRIPT TWO}\n"
            f"Cycles: {parameters.cycles}  |  Constant-speed span: "
            f"{constant_speed_span_deg(parameters.min_angle_deg + parameters.max_angle_deg, parameters.commanded_afo_speed_deg_s, parameters.commanded_afo_acceleration_deg_s2):.2f}\N{DEGREE SIGN}\n\n"
            "Fixture clear  |  Physical E-stop accessible"
        )
        if not self.show_resizable_confirmation(
            title=("Confirm Empty-Machine Baseline" if baseline_test else "Confirm Test"),
            message=confirmation_message,
            confirm_text=("Start Baseline" if baseline_test else "Start"),
        ):
            return

        self.test_stop_event.clear()
        self.acquisition_error = None
        self.operator_stop_requested = False
        self.acquisition_done_event.clear()
        try:
            self.run_motion_token = self.motion_coordinator.acquire("strain-test")
        except MotionConflictError as exc:
            self.update_terminal(f"Cannot start test: {exc}\n")
            return
        self.strain_test_active = True

        self.set_test_inputs_state("disabled")
        self.buttons[0].configure(state="disabled")
        self.buttons[1].configure(state="disabled")
        self.manual_mode_toggle.configure(state="disabled")
        self.run_metadata = None
        self.metadata_file_name = None
        self.strain_file_name = None
        self.test_started_monotonic = time.monotonic()
        try:
            snapshot = self.get_feedback()
            if snapshot.active_errors:
                raise RuntimeError(f"ODrive has active errors: {snapshot.active_errors}")

            self.voltage_ratio_input = self._open_load_cell()
            connected_phidget_serial = self.voltage_ratio_input.getDeviceSerialNumber()
            tare_valid, tare_reason = validate_empty_machine_tare(
                self.empty_machine_tare,
                parameters,
                self.system_config,
                connected_phidget_serial_number=connected_phidget_serial,
            )
            if not tare_valid:
                self.invalidate_empty_machine_tare(tare_reason)
                raise RuntimeError(tare_reason)

            if self.test_stop_event.is_set():
                raise TestStopped("Test was stopped during initialization")
            self.motion_coordinator.assert_active(self.run_motion_token)

            self.run_parameters = parameters
            preset_metadata = self.current_preset_metadata(parameters)
            self.configure_trajectory(
                parameters.commanded_afo_speed_deg_s,
                parameters.commanded_afo_acceleration_deg_s2,
            )
            app_base = Path(sys.executable).resolve().parent if getattr(sys, "frozen", False) else Path(__file__).resolve().parent
            csv_path, metadata_path = create_run_paths(parameters, self.system_config, app_base)
            self.strain_file_name = str(csv_path)
            self.metadata_file_name = str(metadata_path)
            with open(self.strain_file_name, mode="x", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(CSV_COLUMNS)

            self.run_metadata = make_run_metadata(
                parameters,
                self.system_config,
                self.empty_machine_tare.offset_v_per_v,
                csv_path,
                self.odrive_configuration_snapshot(),
                tare_metadata=self.empty_machine_tare.to_dict(),
                preset_metadata=preset_metadata,
            )
            self.run_metadata["software"]["gui_version"] = APP_VERSION
            self.run_metadata["cycle_definition"] = (
                "Cycle 0: startup to positive endpoint. Each numbered cycle: "
                "positive to negative to positive endpoint, with both sweeps settled. "
                "Final machine-zero return is excluded from analysis sweeps."
            )
            self.run_metadata["move_distance_definition"] = (
                "Nominal Move Distance (deg) stores absolute target minus validated "
                "pre-command feedback position, converted to AFO degrees."
            )
            self.run_metadata["machine_zero_reference"] = (
                self.reference_manager.metadata_snapshot()
            )
            self.run_metadata["baseline_matching"] = self.baseline_matching_metadata(
                parameters
            )
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
            token = self.run_motion_token
            self.safe_idle_motor("test initialization error", token=token)
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
            if token is not None:
                self.motion_coordinator.release(token)
            if self.run_motion_token == token:
                self.run_motion_token = None
            self.set_test_inputs_state("normal")
            self.buttons[0].configure(state="normal")
            self.manual_mode_toggle.configure(state="normal")
            if self.odrive_controller:
                self._refresh_motion_controls()
            if self.run_metadata is not None and not self.run_finalized:
                self.completed_cycles = 0
                self.finalize_run(
                    "error",
                    str(exc),
                    idle_confirmed=self.motion_coordinator.motor_idle_confirmed,
                )
    


    def _open_load_cell(self):
        device = VoltageRatioInput()
        hardware = self.system_config["hardware"]
        try:
            if hardware["phidget_serial_number"] is not None:
                device.setDeviceSerialNumber(hardware["phidget_serial_number"])
            device.setChannel(hardware["phidget_channel"])
            device.openWaitForAttachment(hardware["phidget_attachment_timeout_ms"])
            device.setDataInterval(self.system_config["acquisition"]["sample_interval_ms"])
            return device
        except Exception:
            try:
                device.close()
            except Exception:
                pass
            raise

    def tare_empty_machine(self):
        """Capture a session tare with no AFO/load fitted and the fixture at machine zero."""
        if self.odrive_adapter is None or not self.reference_manager.verified:
            self.update_terminal("Verify machine zero before capturing the empty-machine tare.\n")
            return
        if self.motion_coordinator.owner is not None or self.strain_test_active:
            self.update_terminal("Empty-machine tare is blocked while motion is active.\n")
            return
        operator = self.operator_input.get().strip()
        fixture_id = self.fixture_id_input.get().strip()
        calibration_id = self.calibration_id_input.get().strip()
        if not operator or not fixture_id or not calibration_id:
            CTkMessagebox(
                title="Tare Details Required",
                message="Enter Operator ID, Fixture ID, and Calibration ID before taring.",
            )
            return
        confirmation = CTkMessagebox(
            title="Tare Empty Machine",
            message=(
                "Confirm the AFO and all removable test loads are removed, the fixture is at "
                "verified machine zero, and nothing is touching or preloading the load cell.\n\n"
                "This will replace the current session tare."
            ),
            icon="question",
            option_1="Cancel",
            option_2="Tare Empty Machine",
        )
        if confirmation.get() != "Tare Empty Machine":
            return

        device = None
        try:
            self._validate_live_motion_config()
            self._revalidate_hardware_identity(require_reference=True)
            self._require_matching_reference_fixture(fixture_id)
            idle_result = self.odrive_adapter.request_idle(
                self.system_config["reference"]["idle_confirmation_timeout_s"]
            )
            if not idle_result.confirmed:
                raise RuntimeError("ODrive idle could not be confirmed")
            self.motion_coordinator.confirm_motor_idle()
            snapshot = self._read_adapter_feedback()
            snapshot.validate()
            if snapshot.active_errors:
                raise RuntimeError(f"ODrive has active errors: {snapshot.active_errors}")
            if snapshot.current_state != int(AXIS_STATE_IDLE):
                raise RuntimeError("ODrive did not report idle during tare")
            angle = self.reference_manager.angle_from_position(snapshot.position_turns)
            tolerance = float(self.system_config["motion"]["position_tolerance_deg"])
            if abs(angle) > tolerance:
                raise RuntimeError(
                    "Fixture is away from machine zero. Return to machine zero before taring."
                )

            device = self._open_load_cell()
            sample_count = int(self.system_config["load_cell"]["tare_samples"])
            samples = []
            self.update_terminal(
                f"Capturing empty-machine tare from {sample_count} load-cell samples...\n"
            )
            for _ in range(sample_count):
                value = float(device.getVoltageRatio())
                if not math.isfinite(value):
                    raise RuntimeError("Load-cell tare sample was not finite")
                samples.append(value)
                time.sleep(device.getDataInterval() / 1000.0)
            connected_serial = int(device.getDeviceSerialNumber())
            self.empty_machine_tare = EmptyMachineTare(
                offset_v_per_v=sum(samples) / len(samples),
                captured_at=datetime.now().astimezone().isoformat(timespec="milliseconds"),
                operator_id=operator,
                fixture_id=fixture_id,
                calibration_id=calibration_id,
                phidget_serial_number=connected_serial,
                phidget_channel=int(self.system_config["hardware"]["phidget_channel"]),
                sample_count=sample_count,
            )
            self.update_terminal(
                "Empty-machine tare captured. Mounting an AFO will not trigger another tare.\n"
            )
            self.update_tare_display()
            self._refresh_motion_controls()
        except Exception as exc:
            self.update_terminal(f"Empty-machine tare failed: {exc}\n")
            CTkMessagebox(title="Tare Not Captured", message=str(exc))
        finally:
            if device is not None:
                try:
                    device.close()
                except Exception:
                    pass

    def get_current_weight(self):
        """Get the current weight reading from the scale in grams"""
        if self.empty_machine_tare is None:
            return 0.0
        voltage_ratio = self.voltage_ratio_input.getVoltageRatio()
        _, weight_grams, _ = calculate_load(
            voltage_ratio, self.empty_machine_tare.offset_v_per_v, self.system_config
        )
        return weight_grams

    @staticmethod
    def movement_direction_for_velocity(afo_velocity_deg_s):
        if afo_velocity_deg_s > 0:
            return "increasing_machine_angle"
        if afo_velocity_deg_s < 0:
            return "decreasing_machine_angle"
        return "stationary"


    
    def log_strain_data(self, voltage_ratio, cycle):
        """Log strain data to buffer"""
        try:
            if self.empty_machine_tare is not None:
                tare_offset = self.empty_machine_tare.offset_v_per_v
                wall_time = datetime.now().astimezone()
                monotonic_time = time.monotonic()
                snapshot = self.get_feedback()
                current_pos_turns = snapshot.position_turns
                relative_angle = self.reference_manager.angle_from_position(current_pos_turns)
                velocity_turns_s = snapshot.velocity_turns_s
                afo_velocity_deg_s = odrive_turns_to_afo_degrees(
                    velocity_turns_s, self.system_config
                )
                mass_kg, raw_weight_grams, force_n = calculate_load(
                    voltage_ratio, tare_offset, self.system_config
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
                    self.movement_direction_for_velocity(afo_velocity_deg_s),
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
                    self.run_parameters.test_type,
                    self.run_metadata["protocol"]["preset_display_name"],
                    self.run_metadata["protocol"]["preset_version"] or "",
                    self.run_metadata["protocol"]["preset_modified"],
                    f"{self.commanded_odrive_velocity:.6f}",
                    f"{self.current_nominal_distance_deg:.6f}",
                    f"{self.current_expected_constant_speed_span_deg:.6f}",
                    f"{current_pos_turns:.8f}",
                    f"{relative_angle:.6f}",
                    f"{avg_angle:.6f}",
                    f"{voltage_ratio:.12g}",
                    f"{tare_offset:.12g}",
                    f"{mass_kg:.9f}",
                    f"{raw_weight_grams:.6f}",
                    f"{avg_weight:.6f}",
                    f"{force_n:.6f}",
                    f"{raw_torque_nm:.6f}",
                    f"{avg_torque:.6f}",
                    f"{velocity_turns_s:.8f}",
                    f"{afo_velocity_deg_s:.6f}",
                    snapshot.active_errors,
                ]
                self.strain_data_buffer.append(data_row)
                if len(self.strain_data_buffer) >= self.system_config["acquisition"]["csv_flush_rows"]:
                    with open(self.strain_file_name, mode="a", newline="", encoding="utf-8") as file:
                        writer = csv.writer(file)
                        writer.writerows(self.strain_data_buffer)
                        file.flush()
                        os.fsync(file.fileno())
                    self.strain_data_buffer = []
                
                # Collect plot data regardless of which output tab is visible.
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
                raise RuntimeError("Empty-machine tare is not available")
                
        except Exception as e:
            self.update_terminal(f"Error logging strain data: {str(e)}\n")
            raise
    
    def strain_test_control(self):
        """Control the motor and log strain data during the test"""
        final_status = "aborted"
        final_error = None
        idle_confirmed = False
        candidate_completed = False
        token = self.run_motion_token
        try:
            if token is None:
                raise RuntimeError("Test motion ownership was not acquired")
            if self.test_stop_event.is_set():
                raise TestStopped()
            self.enter_closed_loop(token)
            mapping = self.reference_manager.require_verified()
            self.update_terminal(
                f"Verified machine zero (fixture physical 90 degrees): "
                f"{mapping.neutral_position_turns:.8f} session turns\n"
            )
            parameters = self.run_parameters
            absolute_min = self.reference_manager.target_for_angle(-parameters.min_angle_deg)
            absolute_max = self.reference_manager.target_for_angle(parameters.max_angle_deg)

            self.update_terminal(
                f"Commanded AFO range: -{parameters.min_angle_deg:g}\N{DEGREE SIGN} to "
                f"+{parameters.max_angle_deg:g}\N{DEGREE SIGN}\n"
                f"Commanded AFO speed: {parameters.commanded_afo_speed_deg_s:g}"
                f"\N{DEGREE SIGN}/s\n"
                f"Commanded AFO acceleration: {parameters.commanded_afo_acceleration_deg_s2:g}"
                f"\N{DEGREE SIGN}/s\N{SUPERSCRIPT TWO}\n"
            )
            self.current_cycle = 0
            self.command_position_and_wait(
                absolute_max, "moving_to_initial_max", token
            )
            for cycle in range(1, parameters.cycles + 1):
                self.current_cycle = cycle
                self.command_position_and_wait(
                    absolute_min,
                    "moving_to_min",
                    token,
                )
                self.command_position_and_wait(absolute_max, "moving_to_max", token)
                if self.test_stop_event.is_set() or self.acquisition_error:
                    raise TestStopped()
                self.completed_cycles = cycle
                self.update_terminal(f"Completed cycle {cycle}/{parameters.cycles}\n")

            # A successful run uses the dedicated, conservative neutral-return profile.
            motion = self.system_config["motion"]
            self.configure_trajectory(
                motion["neutral_return_speed_deg_s"],
                motion["neutral_return_acceleration_deg_s2"],
            )
            neutral_target = mapping.neutral_position_turns
            self.command_position_and_wait(
                neutral_target,
                "returning_to_machine_zero",
                token,
                speed_deg_s=motion["neutral_return_speed_deg_s"],
                acceleration_deg_s2=motion["neutral_return_acceleration_deg_s2"],
            )
            idle_result = self.safe_idle_motor("successful machine-zero return", token=token)
            if not idle_result or not idle_result.confirmed:
                raise RuntimeError("Test finished but ODrive idle could not be confirmed")
            idle_confirmed = True
            self.motion_phase = "post_idle_observation"
            self.observe_post_idle_neutral(neutral_target)
            candidate_completed = True
        except TestStopped:
            if self.acquisition_error:
                final_status = "error"
                final_error = self.acquisition_error
                self.update_terminal("Strain test stopped by a data acquisition error.\n")
            else:
                final_status = "aborted"
                final_error = "operator stop"
                self.update_terminal("Strain test aborted.\n")
        except Exception as exc:
            final_status = "error"
            final_error = str(exc)
            self.update_terminal(f"Error during strain test: {exc}\n")
        finally:
            stop_was_requested = self.test_stop_event.is_set()
            self.test_stop_event.set()
            # Motor idle is requested before waiting for acquisition or closing devices.
            if not idle_confirmed:
                idle_result = self.safe_idle_motor(final_status, token=token)
                if not idle_result or not idle_result.confirmed:
                    final_status = "error"
                    suffix = "ODrive idle was not confirmed"
                    final_error = f"{final_error}; {suffix}" if final_error else suffix
                else:
                    idle_confirmed = True
            if hasattr(self, "data_collection_thread"):
                self.data_collection_thread.join(timeout=2.0)
                if self.data_collection_thread.is_alive():
                    final_status = "error"
                    suffix = "data acquisition thread did not stop within 2 seconds"
                    final_error = f"{final_error}; {suffix}" if final_error else suffix
            final_status, final_error = reconcile_run_outcome(
                candidate_completed=candidate_completed,
                stop_requested=stop_was_requested or self.operator_stop_requested,
                acquisition_error=self.acquisition_error,
                current_status=final_status,
                current_error=final_error,
            )
            try:
                if self.voltage_ratio_input is not None:
                    self.voltage_ratio_input.close()
            except Exception:
                pass
            self.voltage_ratio_input = None
            if token is not None:
                self.motion_coordinator.release(token)
            if self.run_motion_token == token:
                self.run_motion_token = None
            self.strain_test_active = False
            if final_status == "completed":
                self.update_terminal(
                    f"Strain test completed. Data saved to {self.strain_file_name}\n"
                )
            self.finalize_run(
                final_status,
                final_error,
                idle_confirmed=idle_confirmed,
            )

    def command_position_and_wait(
        self,
        target_turns,
        phase,
        token,
        speed_deg_s=None,
        acceleration_deg_s2=None,
    ):
        if self.test_stop_event.is_set():
            raise TestStopped()
        self.motion_coordinator.assert_active(token)
        if self.odrive_adapter is None:
            raise RuntimeError("ODrive disconnected during motion")
        self.motion_phase = phase
        snapshot = self.get_feedback()
        actual_distance_deg = abs(odrive_turns_to_afo_degrees(
            float(target_turns) - snapshot.position_turns, self.system_config
        ))
        # Retain the legacy CSV column name; its value is feedback-derived.
        self.current_nominal_distance_deg = actual_distance_deg
        effective_speed = (
            self.run_parameters.commanded_afo_speed_deg_s
            if speed_deg_s is None else float(speed_deg_s)
        )
        effective_acceleration = (
            self.run_parameters.commanded_afo_acceleration_deg_s2
            if acceleration_deg_s2 is None else float(acceleration_deg_s2)
        )
        self.current_expected_constant_speed_span_deg = constant_speed_span_deg(
            actual_distance_deg,
            effective_speed,
            effective_acceleration,
        )
        timeout_s = motion_timeout_seconds(
            actual_distance_deg,
            effective_speed,
            effective_acceleration,
            self.system_config,
        )
        tolerance_turns = afo_degrees_to_odrive_turns(
            self.system_config["motion"]["position_tolerance_deg"], self.system_config
        )
        velocity_limit_turns_s = afo_speed_to_odrive_turns_s(
            self.system_config["reference"]["settle_velocity_limit_deg_s"],
            self.system_config,
        )
        self.reference_manager.write_checkpoint(
            MotionState.MOVING,
            snapshot,
            clean_shutdown=False,
            extra={"phase": phase, "target_turns": float(target_turns),
                   "actual_distance_deg": actual_distance_deg, "timeout_s": timeout_s},
        )
        self._submit_position(token, target_turns)
        try:
            wait_for_settle(
                self.get_feedback,
                target_turns=float(target_turns),
                tolerance_turns=tolerance_turns,
                velocity_limit_turns_s=velocity_limit_turns_s,
                dwell_s=self.system_config["reference"]["settle_dwell_ms"] / 1000.0,
                timeout_s=timeout_s,
                stale_after_s=self.system_config["reference"]["feedback_stale_after_ms"] / 1000.0,
                cancelled=lambda: self.test_stop_event.is_set() or self._motion_cancelled(token),
                **self._powered_wait_requirements(),
            )
        except MotionConflictError as exc:
            raise TestStopped() from exc
        except TimeoutError as exc:
            latest = self.latest_feedback
            position = getattr(latest, "position_turns", None)
            remaining = None if position is None else odrive_turns_to_afo_degrees(
                float(target_turns) - position, self.system_config
            )
            raise TimeoutError(
                f"{phase}: target={float(target_turns):.6f} turns, "
                f"distance={actual_distance_deg:.6f} deg, speed={effective_speed:g} deg/s, "
                f"acceleration={effective_acceleration:g} deg/s^2, timeout={timeout_s:.2f} s; "
                f"latest position={position} turns, remaining error={remaining} deg. {exc}"
            ) from exc

    def observe_post_idle_neutral(self, neutral_target):
        self.observe_idle_neutral(
            neutral_target,
            cancelled=lambda: self.test_stop_event.is_set()
            or self.acquisition_error is not None,
        )

    def observe_idle_neutral(self, neutral_target, cancelled=lambda: False):
        duration_s = self.system_config["reference"]["post_idle_observation_ms"] / 1000.0
        tolerance_turns = afo_degrees_to_odrive_turns(
            self.system_config["motion"]["position_tolerance_deg"], self.system_config
        )
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            if cancelled():
                raise TestStopped()
            snapshot = self.get_feedback()
            if snapshot.current_state != int(AXIS_STATE_IDLE):
                raise RuntimeError("ODrive left idle during post-test observation")
            if snapshot.active_errors:
                raise RuntimeError(
                    f"ODrive error during post-test observation: {snapshot.active_errors}"
                )
            if abs(snapshot.position_turns - neutral_target) > tolerance_turns:
                raise RuntimeError("Machine-zero position drifted after ODrive entered idle")
            time.sleep(0.02)
    
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
                try:
                    with open(self.strain_file_name, mode="a", newline="", encoding="utf-8") as file:
                        writer = csv.writer(file)
                        writer.writerows(self.strain_data_buffer)
                        file.flush()
                        os.fsync(file.fileno())
                    self.strain_data_buffer = []
                except Exception as exc:
                    self.acquisition_error = f"CSV flush error: {exc}"
                    self.test_stop_event.set()
                    self.update_terminal(f"SAFETY STOP: {self.acquisition_error}\n")
            self.acquisition_done_event.set()

    def finalize_run(self, status, error=None, idle_confirmed=False):
        with self.finalize_lock:
            if self.run_finalized or self.run_metadata is None:
                return
            self.run_finalized = True
            self.run_metadata["run_status"] = status
            self.run_metadata["completed_at"] = datetime.now().astimezone().isoformat(timespec="milliseconds")
            self.run_metadata["sample_count"] = self.sample_count
            self.run_metadata["completed_cycles"] = self.completed_cycles
            self.run_metadata["error"] = error
            self.run_metadata["machine_zero_reference_final"] = (
                self.reference_manager.metadata_snapshot()
            )
            self.run_metadata["idle_confirmation"] = {
                "confirmed": bool(idle_confirmed),
                "motion_owner": self.motion_coordinator.owner,
                "monitor_error": self.monitor_error,
            }
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
                status = "error"
                self.run_metadata["run_status"] = "error"
                self.run_metadata["error"] = (
                    f"{error}; metadata finalization failed: {exc}"
                    if error else f"metadata finalization failed: {exc}"
                )
                self.update_terminal(f"RUN ERROR: failed to finalize metadata: {exc}\n")
            colour = GREEN if status == "completed" else AMBER if status == "aborted" else RED
            idle_label = "IDLE" if idle_confirmed else "IDLE UNCONFIRMED"
            self.set_status(f"{status.upper()} / {idle_label}", colour)
            self.ui_message_queue.put(("inputs", "normal"))
            self.ui_message_queue.put(("run_buttons", "normal"))

    def stop_strain_test(self):
        """Stop the strain test"""
        if self.strain_test_active:
            self.test_stop_event.set()
            self.update_terminal("Stopping strain test...\n")
        else:
            self.update_terminal("No strain test active\n")

    def _on_output_tab_changed(self):
        if self.output_tabs.get() == "Plot":
            self.master.after_idle(self.update_plot)

    def create_plot_window(self, *, select_plot=True):
        """Initialise the embedded Plot tab; retain the legacy caller name."""
        global plot_window_open, plot_window, plot_curve
        self.plot_title = self.file_name_input.get() or "AFO Strain Test"
        canvas = getattr(self, "plot_canvas", None)
        if canvas is None or not canvas.winfo_exists():
            self.plot_canvas = tk.Canvas(
                self.plot_container, background=PANEL, highlightthickness=0
            )
            self.plot_canvas.pack(fill="both", expand=True, padx=4, pady=4)
            self.plot_canvas.bind("<Configure>", lambda _event: self.update_plot())
            self.plot_container.bind("<Escape>", lambda _event: self.stop_logging())
            plot_curve = None
        plot_window = self.plot_canvas
        plot_window_open = True
        if select_plot:
            self.output_tabs.set("Plot")
        self.master.after_idle(self.update_plot)

    def update_plot(self):
        """Render the visible Plot tab on the Tk main thread."""
        global plot_curve, angle_data, torque_data

        canvas = getattr(self, "plot_canvas", None)
        if (not plot_window_open or canvas is None or not canvas.winfo_exists()
                or self.output_tabs.get() != "Plot"):
            return

        canvas.delete("plot")
        width = max(canvas.winfo_width(), 300)
        height = max(canvas.winfo_height(), 240)
        left, right, top, bottom = 68, 16, 50, 64
        x0, x1 = left, width - right
        y0, y1 = top, height - bottom

        canvas.create_text(
            width / 2, 24, text=self.plot_title, fill=TEXT,
            font=("Arial", 13, "bold"), tags="plot",
        )
        canvas.create_text(
            width / 2, height - 22,
            text="ODrive-Derived AFO Angle (degrees)", fill=TEXT,
            font=("Arial", 9, "bold"), tags="plot",
        )
        canvas.create_text(
            18, height / 2, text="Torque (Nm)", fill=TEXT,
            font=("Arial", 11, "bold"), angle=90, tags="plot",
        )

        if angle_data and torque_data:
            x_min, x_max = min(angle_data), max(angle_data)
            y_min, y_max = min(torque_data), max(torque_data)
        else:
            x_min, x_max, y_min, y_max = 0.0, 1.0, 0.0, 1.0
        if x_min == x_max:
            x_min -= 0.5
            x_max += 0.5
        if y_min == y_max:
            y_min -= 0.5
            y_max += 0.5
        x_pad = (x_max - x_min) * 0.05
        y_pad = (y_max - y_min) * 0.08
        x_min, x_max = x_min - x_pad, x_max + x_pad
        y_min, y_max = y_min - y_pad, y_max + y_pad

        for index in range(6):
            fraction = index / 5.0
            x = x0 + fraction * (x1 - x0)
            y = y1 - fraction * (y1 - y0)
            canvas.create_line(x, y0, x, y1, fill="#e2e8f0", tags="plot")
            canvas.create_line(x0, y, x1, y, fill="#e2e8f0", tags="plot")
            canvas.create_text(
                x, y1 + 18,
                text=f"{x_min + fraction * (x_max - x_min):.2f}",
                fill=MUTED, font=("Arial", 9), tags="plot",
            )
            canvas.create_text(
                x0 - 10, y,
                text=f"{y_min + fraction * (y_max - y_min):.2f}",
                fill=MUTED, font=("Arial", 9), anchor="e", tags="plot",
            )
        canvas.create_rectangle(x0, y0, x1, y1, outline=TEXT, width=2, tags="plot")
        canvas.create_line(
            x1 - 145, y0 + 18, x1 - 105, y0 + 18,
            fill=BLUE, width=2, tags="plot",
        )
        canvas.create_text(
            x1 - 98, y0 + 18, text="Torque vs Angle", anchor="w",
            fill=TEXT, font=("Arial", 9, "bold"), tags="plot",
        )

        if len(angle_data) >= 2:
            max_render_points = 2500
            step = max(1, len(angle_data) // max_render_points)
            samples = list(zip(angle_data[::step], torque_data[::step]))
            if samples[-1] != (angle_data[-1], torque_data[-1]):
                samples.append((angle_data[-1], torque_data[-1]))
            coordinates = []
            for angle, torque in samples:
                x = x0 + (angle - x_min) / (x_max - x_min) * (x1 - x0)
                y = y1 - (torque - y_min) / (y_max - y_min) * (y1 - y0)
                coordinates.extend((x, y))
            plot_curve = canvas.create_line(
                *coordinates, fill=BLUE, width=2, tags="plot"
            )

    def close_plot_window(self):
        """Release the embedded plot during application shutdown."""
        global plot_window_open, plot_window, plot_curve, angle_data, torque_data
        canvas = getattr(self, "plot_canvas", None)
        if canvas is not None and canvas.winfo_exists():
            canvas.destroy()
        plot_window_open = False
        plot_window = None
        plot_curve = None
        angle_data = []
        torque_data = []
        self.plot_canvas = None

    def update_plot_data(self, angle, torque):
        """Add a data point and redraw from the Tk main thread."""
        global angle_data, torque_data

        try:
            if plot_window_open:
                angle_data.append(angle)
                torque_data.append(torque)
                max_points = 10000
                if len(angle_data) > max_points:
                    angle_data = angle_data[-max_points:]
                    torque_data = torque_data[-max_points:]
                self.update_plot()
        except Exception as e:
            self.update_terminal(f"Error updating plot: {str(e)}\n")

    def _manual_step_unit_changed(self, _selection=None):
        # Never reinterpret an old numeric entry in a new unit.
        self.step_angle_input.delete(0, "end")
        if self.manual_step_units.get() == "Motor turns":
            self.continuous_mode.set(False)
        self._update_manual_step_hint()
        self._refresh_motion_controls()

    def _update_manual_step_hint(self):
        units = self.manual_step_units.get()
        conversion = self.system_config["motion"]["afo_degrees_per_odrive_turn"]
        if units == "Motor turns":
            self.step_angle_input.configure(placeholder_text="Motor turns per click (e.g. 1)")
            text = (f"Step: {0.01 / conversion:.6g}-{10 / conversion:.6g} motor turns. "
                    "Direct turn command; existing travel limits apply.")
        else:
            self.step_angle_input.configure(placeholder_text="Degrees per click (0.01-10)")
            text = "Degree steps use the configured motor conversion. Continuous mode uses degrees."
        self.manual_step_hint.configure(text=text)

    def update_manual_position_display(self):
        # Called only by the Tk queue-drain timer; use cached, freshness-checked feedback.
        if not hasattr(self, "manual_position_label"):
            return
        try:
            snapshot = self.get_feedback()
            text = f"ODrive motor position (session): {snapshot.position_turns:.8f} turns"
            if self.reference_manager.verified:
                zero = self.reference_manager.require_verified().neutral_position_turns
                delta = snapshot.position_turns - zero
                angle = odrive_turns_to_afo_degrees(delta, self.system_config)
                text += f"\nFrom machine zero: {delta:+.8f} turns | motor-derived {angle:+.3f} deg"
            else:
                text += "\nFrom machine zero: unavailable until physical 90 deg is verified"
            self.manual_position_label.configure(text=text, text_color=TEXT)
        except Exception as exc:
            self.manual_position_label.configure(text=f"Motor feedback unavailable: {exc}", text_color=AMBER)

    def validate_step_angle(self, event=None):
        # Validate on submission, not while typing fractional values.
        return

    def move_motor_left(self):
        self._start_manual_step(-1)

    def move_motor_right(self):
        self._start_manual_step(1)

    def _start_manual_step(self, direction):
        if self.continuous_mode.get():
            return
        try:
            self.prepare_manual_motion()
            units = self.manual_step_units.get()
            requested = float(self.step_angle_input.get())
            step_turns = manual_step_to_turns(requested, units, self.system_config)
            token = self.motion_coordinator.acquire("manual-step")
        except Exception as exc:
            self.update_terminal(f"Manual step blocked: {exc}\n")
            return
        self._refresh_motion_controls()
        self.manual_step_thread = threading.Thread(
            target=self._manual_step_worker,
            args=(token, direction, step_turns, requested, units),
            name="manual-step",
            daemon=True,
        )
        self.manual_step_thread.start()

    def _manual_step_worker(self, token, direction, step_turns, requested, units):
        try:
            motion = self.system_config["motion"]
            current = self.get_feedback().position_turns
            target = current + direction * step_turns
            bounded = self.clamp_manual_target(target)
            if not math.isclose(target, bounded, rel_tol=0.0, abs_tol=1e-10):
                raise ValueError("Full requested step exceeds travel limits; reduce the step. No movement commanded.")
            distance_deg = abs(odrive_turns_to_afo_degrees(step_turns, self.system_config))
            timeout_s = motion_timeout_seconds(
                distance_deg, motion["manual_speed_deg_s"],
                motion["manual_acceleration_deg_s2"], self.system_config,
            )
            self.configure_trajectory(motion["manual_speed_deg_s"], motion["manual_acceleration_deg_s2"])
            self.enter_closed_loop(token)
            snapshot = self.get_feedback()
            self.reference_manager.write_checkpoint(
                MotionState.MOVING, snapshot, clean_shutdown=False,
                extra={"phase": "manual_step", "target_turns": target,
                       "requested_step": direction * requested, "step_units": units},
            )
            self._submit_position(token, target)
            wait_for_settle(
                self.get_feedback, target,
                afo_degrees_to_odrive_turns(motion["position_tolerance_deg"], self.system_config),
                afo_speed_to_odrive_turns_s(self.system_config["reference"]["settle_velocity_limit_deg_s"], self.system_config),
                self.system_config["reference"]["settle_dwell_ms"] / 1000.0,
                timeout_s,
                self.system_config["reference"]["feedback_stale_after_ms"] / 1000.0,
                lambda: self._motion_cancelled(token),
                expected_state=int(AXIS_STATE_CLOSED_LOOP_CONTROL),
                expected_disarm_reason=self.expected_disarm_reason,
                progress=self._mark_control_health,
            )
            settled = self.get_feedback().position_turns
            self.update_terminal(
                f"Manual step complete: requested {direction * requested:+.6g} {units.lower()}; "
                f"start {current:.8f}, settled {settled:.8f}, "
                f"measured displacement {settled - current:+.8f} motor turns.\n"
            )
        except Exception as exc:
            self.update_terminal(f"Manual step failed: {exc}\n")
        finally:
            self.safe_idle_motor("manual step complete", token=token)
            self.motion_coordinator.release(token)
            self.ui_message_queue.put(("reference",))

    def start_continuous_movement(self):
        """Continuous movement is handled by one owned worker while the button is held."""
        return

    def begin_continuous_movement(self, direction):
        if not self.continuous_mode.get() or self.continuous_movement_active:
            return
        try:
            self.prepare_manual_motion()
            token = self.motion_coordinator.acquire("manual-continuous")
        except Exception as exc:
            self.update_terminal(f"Continuous movement blocked: {exc}\n")
            return
        self.continuous_movement_active = True
        self.movement_direction = direction
        self.continuous_stop_event = threading.Event()
        self.continuous_motion_token = token
        self._refresh_motion_controls()
        self.continuous_thread = threading.Thread(
            target=self._continuous_movement_worker,
            args=(token, direction),
            name="manual-continuous",
            daemon=True,
        )
        self.continuous_thread.start()

    def _continuous_movement_worker(self, token, direction):
        try:
            motion = self.system_config["motion"]
            self.configure_trajectory(
                motion["manual_speed_deg_s"], motion["manual_acceleration_deg_s2"]
            )
            self.enter_closed_loop(token)
            increment = afo_degrees_to_odrive_turns(
                motion["manual_continuous_increment_deg"], self.system_config
            )
            if direction == "left":
                increment = -increment
            while not self.continuous_stop_event.is_set():
                self.motion_coordinator.assert_active(token)
                snapshot = self.get_feedback()
                target = self.clamp_manual_target(snapshot.position_turns + increment)
                self.reference_manager.write_checkpoint(
                    MotionState.MOVING,
                    snapshot,
                    clean_shutdown=False,
                    extra={"phase": "manual_continuous", "target_turns": target},
                )
                self._submit_position(token, target)
                self.continuous_stop_event.wait(motion["manual_update_interval_ms"] / 1000.0)
        except MotionConflictError:
            pass
        except Exception as exc:
            self.update_terminal(f"Continuous movement failed: {exc}\n")
        finally:
            self.safe_idle_motor("continuous manual movement stopped", token=token)
            self.motion_coordinator.release(token)
            self.continuous_movement_active = False
            self.continuous_motion_token = None
            self.ui_message_queue.put(("reference",))

    def stop_continuous_movement(self):
        """Stop continuous movement"""
        self.continuous_movement_active = False
        self.continuous_stop_event.set()
        if self.movement_timer:
            self.master.after_cancel(self.movement_timer)
            self.movement_timer = None
        if self.motion_coordinator.owner == "manual-continuous":
            self.safe_idle_motor("continuous manual movement stop requested")

    def prepare_manual_motion(self):
        if not self.manual_mode.get():
            raise RuntimeError("Enable manual mode before commanding manual movement")
        if self.strain_test_active:
            raise RuntimeError("Manual movement is disabled while a strain test is active")
        if self.neutral_motion_active:
            raise RuntimeError("Wait for the machine-zero return to finish")
        self.reference_manager.require_verified()
        snapshot = self.get_feedback()
        if snapshot.active_errors:
            raise RuntimeError(f"ODrive has active errors: {snapshot.active_errors}")

    def return_to_machine_zero(self):
        """Return to the saved machine-zero encoder position for this controller session."""
        if self.odrive_controller is None:
            self.update_terminal("Connect the ODrive before returning to machine zero.\n")
            return
        if not self.reference_manager.verified:
            self.update_terminal(f"Machine zero is unavailable: {self.reference_manager.reason}\n")
            return
        if self.strain_test_active or self.neutral_motion_active:
            self.update_terminal("Machine-zero return is unavailable while another motion is active.\n")
            return

        confirmation = CTkMessagebox(
            title="Return to Machine Zero",
            message=(
                "Return to verified machine zero (fixture physical 90 degrees)?\n\n"
                "Confirm the fixture is clear and the physical E-stop is accessible."
            ),
            icon="question",
            option_1="Cancel",
            option_2="Return",
        )
        if confirmation.get() != "Return":
            return

        self.stop_continuous_movement()
        try:
            token = self.motion_coordinator.acquire("neutral-return")
        except MotionConflictError as exc:
            self.update_terminal(f"Machine-zero return blocked: {exc}\n")
            return
        self.neutral_stop_event = threading.Event()
        self.neutral_motion_token = token
        self.neutral_motion_active = True
        self.left_arrow.configure(state="disabled")
        self.right_arrow.configure(state="disabled")
        self.neutral_button.configure(state="disabled")
        self.neutral_thread = threading.Thread(
            target=self._return_to_neutral_worker,
            args=(token,),
            name="neutral-return",
        )
        self.neutral_thread.start()

    def _return_to_neutral_worker(self, token):
        try:
            motion = self.system_config["motion"]
            self.configure_trajectory(
                motion["neutral_return_speed_deg_s"],
                motion["neutral_return_acceleration_deg_s2"],
            )
            self.enter_closed_loop(token)
            neutral_target = self.reference_manager.require_verified().neutral_position_turns
            current_turns = self.get_feedback().position_turns
            distance_deg = abs(
                odrive_turns_to_afo_degrees(
                    current_turns - neutral_target, self.system_config
                )
            )
            timeout_s = motion_timeout_seconds(
                distance_deg,
                motion["neutral_return_speed_deg_s"],
                motion["neutral_return_acceleration_deg_s2"],
                self.system_config,
            )
            tolerance_turns = afo_degrees_to_odrive_turns(
                motion["position_tolerance_deg"], self.system_config
            )
            self.motion_phase = "returning_to_machine_zero"
            snapshot = self.get_feedback()
            self.reference_manager.write_checkpoint(
                MotionState.MOVING,
                snapshot,
                clean_shutdown=False,
                extra={"phase": self.motion_phase, "target_turns": neutral_target},
            )
            self._submit_position(token, neutral_target)
            self.update_terminal(
                f"Returning to machine zero at {neutral_target:.8f} session turns.\n"
            )
            wait_for_settle(
                self.get_feedback,
                neutral_target,
                tolerance_turns,
                afo_speed_to_odrive_turns_s(
                    self.system_config["reference"]["settle_velocity_limit_deg_s"],
                    self.system_config,
                ),
                self.system_config["reference"]["settle_dwell_ms"] / 1000.0,
                timeout_s,
                self.system_config["reference"]["feedback_stale_after_ms"] / 1000.0,
                lambda: self.neutral_stop_event.is_set() or self._motion_cancelled(token),
                expected_state=int(AXIS_STATE_CLOSED_LOOP_CONTROL),
                expected_disarm_reason=self.expected_disarm_reason,
                progress=self._mark_control_health,
            )
            idle_result = self.safe_idle_motor("machine zero reached", token=token)
            if not idle_result or not idle_result.confirmed:
                raise RuntimeError("Machine zero reached but ODrive idle was not confirmed")
            self.observe_idle_neutral(
                neutral_target,
                cancelled=lambda: self.neutral_stop_event.is_set(),
            )
            self.update_terminal("Machine zero reached.\n")
            self.set_status("MACHINE ZERO / IDLE", "#0f766e")
        except TestStopped:
            self.safe_idle_motor("machine-zero return stopped", token=token)
            self.update_terminal("Machine-zero return stopped.\n")
        except Exception as exc:
            self.safe_idle_motor("machine-zero return error", token=token)
            self.set_status("ERROR / IDLE", RED)
            self.update_terminal(f"Machine-zero return failed: {exc}\n")
        finally:
            self.motion_phase = "idle"
            self.motion_coordinator.release(token)
            self.neutral_motion_token = None
            self.ui_message_queue.put(("neutral_finished",))

    def clamp_manual_target(self, target_turns):
        maximum_angle = self.system_config["motion"]["maximum_afo_angle_deg"]
        lower = self.reference_manager.target_for_angle(-maximum_angle)
        upper = self.reference_manager.target_for_angle(maximum_angle)
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
            if not self.reference_manager.verified:
                self.manual_mode.set(False)
                self.update_terminal(
                    f"Manual mode blocked: {self.reference_manager.reason}\n"
                )
                self._refresh_motion_controls()
                return
            # Enable manual controls
            self.left_arrow.configure(state="normal")
            self.right_arrow.configure(state="normal")
            self.step_angle_input.configure(state="normal")
            self.mode_toggle.configure(state="normal")
            self.neutral_button.configure(
                state=(
                    "normal"
                    if self.odrive_controller is not None
                    and self.reference_manager.verified
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
            self.set_status("MANUAL / READY", AMBER)
            self._refresh_motion_controls()
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
                self._refresh_motion_controls()
            
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


def configure_window_icon(window, *, main_window=False):
    """The .ico icon and taskbar identity are Windows-specific decoration."""
    if sys.platform != "win32":
        return
    try:
        icon_path = resource_path("images/icon.ico")
        if icon_path.exists():
            if main_window:
                ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID(str(icon_path))
            window.iconbitmap(str(icon_path))
    except Exception as exc:
        print(f"Failed to set Windows icon: {exc}")


def check_environment():
    """Load runtime libraries only: no device discovery, connections or writes."""
    import platform
    from importlib.metadata import version
    from Phidget22.Phidget import Phidget
    import fibre.libfibre as libfibre

    print(f"Platform: {platform.system()} {platform.machine()}")
    print(f"Python: {platform.python_version()} | {sys.executable}")
    print(f"Tk: {tk.TkVersion} (GUI launch must be checked separately)")
    for package in ("customtkinter", "CTkMessagebox", "Pillow", "odrive", "Phidget22"):
        print(f"{package}: {version(package)}")
    print(f"ODrive native library: {libfibre.lib_path}")
    try:
        phidget_library_version = Phidget.getLibraryVersion()
    except OSError as exc:
        print(f"Phidget native library failed to load: {exc}", file=sys.stderr)
        print("See docs/MAC_SETUP.md or the Phidgets installation guide.", file=sys.stderr)
        return 1
    print(f"Phidget native library: {phidget_library_version}")
    print("Environment check passed. No hardware was opened or commanded.")
    return 0

def create_about_dialog(root):
    about_dialog = ctk.CTkToplevel(root)
    about_dialog.geometry("520x430")
    about_dialog.configure(fg_color=BG)
    about_dialog.title("About")
    about_dialog.transient(root)
    configure_window_icon(about_dialog)

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
    parser = argparse.ArgumentParser(description="EAST AFO stiffness tester")
    parser.add_argument(
        "--check-environment", action="store_true",
        help="check Python/native libraries without opening hardware or the GUI",
    )
    args = parser.parse_args()
    if args.check_environment:
        return check_environment()

    print(f"Starting {APP_NAME} {APP_VERSION} from {Path(__file__).resolve()}")

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

    configure_window_icon(root, main_window=True)

    # Create menubar
    menubar = tk.Menu(root)
    file_menu = tk.Menu(menubar, tearoff=0)
    menubar.add_cascade(label="File", menu=file_menu)
    file_menu.add_command(label="Help", command=lambda: create_about_dialog(root))
    root.configure(menu=menubar)

    root.update_idletasks()
    
    root.mainloop()

if __name__ == "__main__":
    sys.exit(main())
