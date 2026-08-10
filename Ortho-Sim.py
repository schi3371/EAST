import os
import sys
import queue
from datetime import datetime
from pathlib import Path

import concurrent.futures

import pywinstyles

import tkinter as tk
import customtkinter as ctk
from CTkMessagebox import CTkMessagebox

from customtkinter import set_default_color_theme

import csv
import threading
import time
import ctypes
import webbrowser

import PIL
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
        self.master.title("OrthoSim")

        self.system_config = load_tester_config(resource_path("tester_config.json"))
        self.odrive_controller = None
        self.voltage_ratio_input = None
        self.run_parameters = None
        self.run_metadata = None
        self.metadata_file_name = None
        self.strain_file_name = None
        self.test_started_monotonic = None
        self.test_stop_event = threading.Event()
        self.finalize_lock = threading.Lock()
        self.run_finalized = True
        self.motion_phase = "idle"
        self.commanded_odrive_velocity = 0.0
        self.ui_message_queue = queue.Queue()
        self.plot_data_queue = queue.Queue()

        self.strain_test_active = False
        self.strain_data_buffer = []
        self.starting_position = 0
        self.current_cycle = 0
        
        # Add continuous movement flags
        self.continuous_movement_active = False
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

        set_default_color_theme("dark-blue")
        ctk.set_appearance_mode("dark")

        self.setup_ui()
        self.master.after(50, self._drain_ui_queues)

        # Bind window events
        self.master.bind('<Configure>', self.on_window_move)
        self.master.bind('<Unmap>', self.on_window_minimize)
        self.master.bind('<Map>', self.on_window_restore)
        self.master.bind('<FocusIn>', self.on_window_restore)  # Add binding for focus events
        self.master.bind('<Escape>', lambda _event: self.stop_logging())

    def on_window_move(self, event):
        """Update plot window position when main window moves"""
        # Only respond to window movement events
        if event.widget == self.master and hasattr(self, 'plot_container'):
            # Update plot window position relative to main window
            self.plot_container.move(self.master.winfo_x() + 450, self.master.winfo_y() + 190)

    def on_window_minimize(self, event):
        """Hide plot window when main window is minimized"""
        if hasattr(self, 'plot_container'):
            self.plot_container.hide()

    def on_window_restore(self, event):
        """Show plot window when main window is restored"""
        if hasattr(self, 'plot_container'):
            self.plot_container.show()
            self.plot_container.raise_()
        elif plot_window_open:
            # If plot window was open but container was lost, recreate it
            self.create_plot_window()

    def setup_ui(self):
        
        image = PIL.Image.open(resource_path("images/background_image.png"))
        background_image = ctk.CTkImage(image, size=(1242, 786))

        # Create a bg label
        bg_lbl = ctk.CTkLabel(self.master, text="", image=background_image)
        bg_lbl.place(x=0, y=0)

        # Header Frame
        header_frame = ctk.CTkFrame(master=self.master, bg_color="#000001", fg_color="#000001")  # Use CTkFrame
        pywinstyles.set_opacity(header_frame, color="#000001") # just add this line
        header_frame.pack(pady=10, padx=10)

        IMAGE_WIDTH = 255*1.5
        IMAGE_HEIGHT = 68.2*1.5

        image = ctk.CTkImage(
            light_image=Image.open(resource_path("images/SpinSync_logo.png")),
            dark_image=Image.open(resource_path("images/SpinSync_logo.png")),
            size=(IMAGE_WIDTH, IMAGE_HEIGHT),
        )
        

        # Create a label to display the image
        image_label = ctk.CTkLabel(header_frame, image=image, text='', corner_radius=60)
        image_label.grid(row=0, column=0, columnspan=3, pady=10, padx=10)  # Span across all columns


        # Create frame for input ranges
        inputs_frame = ctk.CTkFrame(self.master, bg_color="#000001", fg_color="#000001")  # Use CTkFrame
        pywinstyles.set_opacity(inputs_frame, color="#000001") # just add this line
        inputs_frame.place(x=45, y=95)

        # Create input fields in a grid form for input values 

        self.file_name_input = ctk.CTkEntry(inputs_frame, width=345/2-5, placeholder_text="File Name (Prefix)")
        self.file_name_input.grid(row=1, column=0, padx=5, pady=5)

        self.cycles_input = ctk.CTkEntry(inputs_frame, width=345/2-5, placeholder_text="Cycles")
        self.cycles_input.grid(row=1, column=1, padx=5, pady=5)

        self.speed_input = ctk.CTkEntry(inputs_frame, width=345/2-5, placeholder_text="Speed (Degrees/Second)")
        self.speed_input.grid(row=2, column=0, padx=5, pady=5)

        self.acceleration_input = ctk.CTkEntry(inputs_frame, width=345/2-5, placeholder_text="Acceleration (Degrees/s^2)")
        self.acceleration_input.grid(row=2, column=1, padx=5, pady=5)

        self.min_angle_input = ctk.CTkEntry(inputs_frame, width=345/2-5, placeholder_text="Min Angle (Degrees)")
        self.min_angle_input.grid(row=3, column=0, padx=5, pady=5)
        self.min_angle_input.bind('<KeyRelease>', self.validate_angle_input)

        self.max_angle_input = ctk.CTkEntry(inputs_frame, width=345/2-5, placeholder_text="Max Angle (Degrees)")
        self.max_angle_input.grid(row=3, column=1, padx=5, pady=5)
        self.max_angle_input.bind('<KeyRelease>', self.validate_angle_input)

        self.operator_input = ctk.CTkEntry(inputs_frame, width=345/2-5, placeholder_text="Operator")
        self.operator_input.grid(row=4, column=0, padx=5, pady=5)

        self.afo_id_input = ctk.CTkEntry(inputs_frame, width=345/2-5, placeholder_text="AFO ID")
        self.afo_id_input.grid(row=4, column=1, padx=5, pady=5)

        self.fixture_id_input = ctk.CTkEntry(inputs_frame, width=345/2-5, placeholder_text="Fixture ID")
        self.fixture_id_input.grid(row=5, column=0, padx=5, pady=5)

        self.calibration_id_input = ctk.CTkEntry(inputs_frame, width=345/2-5, placeholder_text="Calibration ID")
        self.calibration_id_input.grid(row=5, column=1, padx=5, pady=5)

        self.status_label = ctk.CTkLabel(
            inputs_frame, text="DISCONNECTED", text_color="#ff6b6b",
            font=("Arial", 12, "bold"), anchor="w"
        )
        self.status_label.grid(row=0, column=0, columnspan=2, sticky="w", padx=5, pady=(0, 2))

        # Create four buttons stacked vertically
        button_names = ["Connect", "Start", "Stop", "Reset"]
        commands = [self.connect_system, self.start_strain_test, self.stop_logging, self.reset_display]
        button_colour = ["#28a745", "#007bff", "#dc3545", "#cc8400"]
        start_button_position_x = 50
        start_button_position_y = 320

        button_positions_y = [start_button_position_y+0, start_button_position_y+37, start_button_position_y+74, start_button_position_y+111]
        self.buttons = []
        for name, command, colour, position_y in zip(button_names, commands, button_colour, button_positions_y):
            button = ctk.CTkButton(self.master, text=name, command=command, hover_color="grey", width=340, fg_color=colour, corner_radius=20, bg_color="#000001")
            button.pack(pady=5, padx = 20)  # Use pack with pady for vertical spacing
            self.buttons.append(button)
            self.buttons[-1].place(x=start_button_position_x, y= position_y)
            pywinstyles.set_opacity(button, color="#000001") # just add this line

        # Add manual control frame below the buttons
        manual_control_frame = ctk.CTkFrame(self.master, fg_color="#000001", bg_color="#000001")
        manual_control_frame.place(x=45, y=465)
        pywinstyles.set_opacity(manual_control_frame, color="#000001") # just add this line

        # Add step angle input with validation
        self.step_angle_input = ctk.CTkEntry(manual_control_frame, width=150, placeholder_text="Step Angle (0-10 deg)")
        self.step_angle_input.grid(row=0, column=0, padx=5, pady=5)
        self.step_angle_input.bind('<KeyRelease>', self.validate_step_angle)

        # Add manual mode toggle
        self.manual_mode_toggle = ctk.CTkSwitch(manual_control_frame, text="Manual Mode", 
                                              variable=self.manual_mode,
                                              onvalue=True, offvalue=False,
                                              command=self.toggle_manual_mode)
        self.manual_mode_toggle.grid(row=0, column=1, padx=5, pady=5)

        # Add arrow buttons frame
        arrow_frame = ctk.CTkFrame(manual_control_frame, fg_color="#000001", bg_color="#000001")
        arrow_frame.grid(row=1, column=0, columnspan=2, padx=5, pady=5)

        # Add left arrow button
        self.left_arrow = ctk.CTkButton(arrow_frame, text="←", width=50, height=50,
                                      command=self.move_motor_left)
        self.left_arrow.grid(row=0, column=0, padx=5, pady=5)
        self.left_arrow.bind('<ButtonPress-1>', lambda _event: self.begin_continuous_movement("left"))
        self.left_arrow.bind('<ButtonRelease-1>', lambda e: self.stop_continuous_movement())

        # Add right arrow button
        self.right_arrow = ctk.CTkButton(arrow_frame, text="→", width=50, height=50,
                                       command=self.move_motor_right)
        self.right_arrow.grid(row=0, column=1, padx=5, pady=5)
        self.right_arrow.bind('<ButtonPress-1>', lambda _event: self.begin_continuous_movement("right"))
        self.right_arrow.bind('<ButtonRelease-1>', lambda e: self.stop_continuous_movement())

        # Add continuous/step mode toggle next to arrows
        self.continuous_mode = ctk.BooleanVar(value=False)
        self.mode_toggle = ctk.CTkSwitch(arrow_frame, text="Continuous Mode", 
                                       variable=self.continuous_mode,
                                       onvalue=True, offvalue=False)
        self.mode_toggle.grid(row=0, column=2, padx=5, pady=5)

        # Disable manual control buttons initially
        self.left_arrow.configure(state="disabled")
        self.right_arrow.configure(state="disabled")
        self.step_angle_input.configure(state="disabled")
        self.mode_toggle.configure(state="disabled")
        
        # Disable Start button initially (until system is connected)
        self.buttons[1].configure(state="disabled")  # Index 1 is the "Start" button

        # Create frame for the bottom section (terminal)
        terminal_frame = ctk.CTkFrame(master=self.master, bg_color="#000001", fg_color="#000001")  # Use CTkFrame
        pywinstyles.set_opacity(terminal_frame, color="#000001") # just add this line
        terminal_frame.place(x=35, y=575)

        # Terminal (text output)
        self.terminal = ctk.CTkTextbox(terminal_frame, height=125, width=350, corner_radius=20)
        self.terminal.pack(pady=10, padx=10)


        # Create frame for the footer section with a larger width
        footer_frame = ctk.CTkFrame(master=self.master, width=200, bg_color="#000001", corner_radius=20)  # Set a larger width
        # pywinstyles.set_opacity(footer_frame, value=0.85, color="#000001") # just add this line
        pywinstyles.set_opacity(footer_frame, value=0.85, color="#000001") # just add this line

        # footer_frame.pack(pady=10, padx=10)  # Use fill='x' to make the frame fill the entire width
        footer_frame.place(x=35, y=720)

        # Developer label
        developer_label = ctk.CTkLabel(footer_frame, text="Developed By: ", anchor="w", font=("Arial", 12, "bold"), text_color="white")
        developer_label.grid(row=0, column=0, sticky="w", padx=10, pady=5)  # Adjust padx as needed

        # Developer's name with hyperlink
        developer_name_label = ctk.CTkLabel(footer_frame, text="Brock Cooper", anchor="w", cursor="hand2", text_color="#007bff", font=("Arial", 12, "bold"))
        developer_name_label.grid(row=0, column=0, sticky="w", padx=95, pady=5)  # Adjust padx as needed
        developer_name_label.bind("<Button-1>", lambda event: self.open_website("https://brockcooper.au"))

        # space
        space_label = ctk.CTkLabel(footer_frame, text="", anchor="e")
        space_label.grid(row=0, column=2, sticky="e", padx=454, pady=5)  # Adjust padx as needed

        # Version label
        version_label = ctk.CTkLabel(footer_frame, text="Version 1.1.0", anchor="e", font=("Arial", 12, "bold"), text_color="white")
        version_label.grid(row=0, column=2, sticky="e", padx=10, pady=5)  # Adjust padx as needed

        # Handle window closing event
        self.master.protocol("WM_DELETE_WINDOW", self.on_close)
 
    def open_website(self, url):
            webbrowser.open_new(url)
            
    def change_theme(self, choice):
        ctk.set_appearance_mode(choice)

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

    def set_status(self, text, colour="#ffffff"):
        if threading.current_thread() is not threading.main_thread():
            self.ui_message_queue.put(("status", text, colour))
            return
        self.status_label.configure(text=text, text_color=colour)

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
        self.set_status("CONNECTING", "#ffd166")
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
            self.configure_trajectory(
                motion["manual_speed_deg_s"], motion["manual_acceleration_deg_s2"]
            )
            self.starting_position = axis.pos_vel_mapper.pos_rel
            self.safe_idle_motor()

            self.update_terminal(
                f"Connected to ODrive S1\nSerial number: {serial_number}\n"
                f"Axis errors after clear: {int(axis.active_errors)}\n"
            )
            self.set_status("CONNECTED / IDLE", "#4dd4ac")
            if self.manual_mode.get():
                self.toggle_manual_mode()
            else:
                self.buttons[1].configure(state="normal")
        except (concurrent.futures.TimeoutError, TimeoutError) as exc:
            self.odrive_controller = None
            self.update_terminal(f"Connection timed out: {exc}\n")
            self.set_status("DISCONNECTED", "#ff6b6b")
        except Exception as exc:
            self.safe_idle_motor("connection/configuration error")
            self.odrive_controller = None
            self.update_terminal(f"Error connecting to ODrive: {exc}\n")
            self.set_status("ERROR", "#ff6b6b")

    def disconnect_odrive(self):
        """Disconnect from ODrive safely"""
        try:
            if self.odrive_controller:
                self.safe_idle_motor("disconnect")
                self.odrive_controller = None
                self.buttons[1].configure(state="disabled")
                self.set_status("DISCONNECTED", "#ff6b6b")
                self.update_terminal("ODrive disconnected and set to idle state\n")
        except Exception as e:
            self.update_terminal(f"Error disconnecting ODrive: {e}\n")

    def stop_logging(self):
        was_active = self.strain_test_active
        self.test_stop_event.set()
        self.strain_test_active = False
        self.safe_idle_motor("operator stop")
        self.set_status("STOPPED / IDLE", "#ffd166")
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

        self.file_name_input.configure(placeholder_text="File Name (Prefix)")
        self.cycles_input.configure(placeholder_text="Cycles")
        self.speed_input.configure(placeholder_text="Speed (Degrees/S)")
        self.acceleration_input.configure(placeholder_text="Acceleration (Degrees/s^2)")
        self.min_angle_input.configure(placeholder_text="Min Angle (Degrees)")
        self.max_angle_input.configure(placeholder_text="Max Angle (Degrees)")

        self.safe_idle_motor("reset")
        if self.odrive_controller and not self.manual_mode.get():
            self.buttons[1].configure(state="normal")
            self.set_status("CONNECTED / IDLE", "#4dd4ac")


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
                    self.buttons[1].configure(state=item[1] if self.odrive_controller else "disabled")
                    self.manual_mode_toggle.configure(state=item[1])
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

                for thread_name in ("strain_thread", "data_collection_thread"):
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

        if self.odrive_controller is None:
            self.update_terminal("No serial connection established. Please connect ODrive first.\n")
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
                f"Range: -{parameters.min_angle_deg:g} to +{parameters.max_angle_deg:g} deg\n"
                f"Speed: {parameters.commanded_afo_speed_deg_s:g} deg/s\n"
                f"Cycles: {parameters.cycles}\n\n"
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
            self.starting_position = axis.pos_vel_mapper.pos_rel
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
            self.set_status("TEST RUNNING", "#4dd4ac")
            self.update_terminal(
                f"Strain test started. Data: {self.strain_file_name}\n"
                f"Trajectory speed: {self.commanded_odrive_velocity:.6f} turns/s\n"
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
            self.set_status("ERROR / IDLE", "#ff6b6b")
            self.update_terminal(f"Error initializing strain test: {exc}\n")
            self.set_test_inputs_state("normal")
            self.buttons[0].configure(state="normal")
            self.manual_mode_toggle.configure(state="normal")
            if self.odrive_controller:
                self.buttons[1].configure(state="normal")
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
                    f"{self.commanded_odrive_velocity:.6f}",
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
                    self.update_terminal(f"Raw Angle: {relative_angle:.4f} deg, Avg Angle: {avg_angle:.4f} deg\n")
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
                f"Command range: -{parameters.min_angle_deg:g} to +{parameters.max_angle_deg:g} deg\n"
                f"Commanded AFO speed: {parameters.commanded_afo_speed_deg_s:g} deg/s\n"
            )
            for cycle in range(1, parameters.cycles + 1):
                self.current_cycle = cycle
                self.command_position_and_wait(
                    absolute_max, "moving_to_max", parameters.max_angle_deg + parameters.min_angle_deg
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
        axis.controller.input_pos = target_turns
        timeout_s = motion_timeout_seconds(
            nominal_distance_deg,
            self.run_parameters.commanded_afo_speed_deg_s,
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
            colour = "#4dd4ac" if status == "completed" else "#ffd166" if status == "aborted" else "#ff6b6b"
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
        """Create a plot window that stays on top of the main window"""
        global plot_window_open, plot_window, plot_curve, angle_data, torque_data
        
        try:
            # Reset data arrays
            angle_data = []
            torque_data = []
            
            # Get title from file name prefix field
            plot_title = self.file_name_input.get() or "AFO Strain Test"
            
            # Create plot window if not already open
            if not plot_window_open:
                # Configure PyQtGraph appearance
                pg.setConfigOption('background', '#2b2b2b')  # Dark gray background
                pg.setConfigOption('foreground', 'w')  # White text and lines
                pg.setConfigOptions(antialias=True)  # Enable antialiasing globally
                
                # Create a QWidget container first
                self.plot_container = QWidget()
                
                # Set fixed size
                self.plot_container.setFixedSize(750, 550)
                
                # Set rounded corners using stylesheet
                self.plot_container.setStyleSheet("""
                    QWidget {
                        background-color: #2b2b2b;
                        border: 0px solid #444444;
                    }
                """)
                
                # Create the plot widget with no navigation bar
                plot_window = pg.PlotWidget()
                plot_window.setBackground('#2b2b2b')
                
                # Enable antialiasing for the plot
                plot_window.setAntialiasing(True)
                
                # Set title with larger font
                title_style = {'color': '#ffffff', 'size': '18pt'}
                plot_window.setTitle(plot_title, **title_style)
                
                # Set axis labels with larger font and white color
                label_style = {'color': '#ffffff', 'font-size': '12pt'}
                plot_window.setLabel('left', 'Torque (Nm)', **label_style)
                plot_window.setLabel('bottom', 'AFO Angle (degrees)', **label_style)
                
                # Hide the navigation bar
                plot_window.hideButtons()
                
                # Set grid style
                plot_window.showGrid(x=True, y=True, alpha=0.3)
                
                # Customize axes with larger text
                for axis in [plot_window.getAxis('left'), plot_window.getAxis('bottom')]:
                    axis.setPen(color='white', width=2)
                    axis.setTextPen(color='white')
                    axis.setStyle(tickFont=QFont('Arial', 12))
                    # Make the axis numbers white
                    axis.setTextPen('w')
                
                # Add a legend with custom styling and larger text
                legend = plot_window.addLegend(pen='w', brush=(50, 50, 50, 200), labelTextColor='w')
                legend.setLabelTextSize('12pt')  # Increased legend text size
                
                # Create the data curve with line only (no symbols)
                plot_curve = plot_window.plot(
                    angle_data, 
                    torque_data, 
                    pen=pg.mkPen(
                        color=(255, 215, 0),  # Gold color
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
                
                # Position the window relative to the main window
                self.plot_container.move(self.master.winfo_x() + 450, self.master.winfo_y() + 190)
                
                # Set window flags for a child window that stays on top of main window
                self.plot_container.setWindowFlags(
                    Qt.Tool |  # Makes it a tool window that stays on top of its parent
                    Qt.CustomizeWindowHint |  # Keeps custom window appearance
                    Qt.FramelessWindowHint  # Removes the window frame
                )
                
                # Show the container
                self.plot_container.show()
                self.plot_container.raise_()  # Ensure it's on top
                
                # Apply rounded corners to the actual window using win32 API
                try:
                    import win32gui
                    import win32con
                    from ctypes import windll, c_int, byref
                    
                    # Get the window handle
                    hwnd = self.plot_container.winId().__int__()
                    
                    # Define the region
                    region = win32gui.CreateRoundRectRgn(0, 0, 750, 550, 40, 40)
                    
                    # Set the window region
                    win32gui.SetWindowRgn(hwnd, region, True)
                    
                    # Make sure the window is layered for transparency
                    style = win32gui.GetWindowLong(hwnd, win32con.GWL_EXSTYLE)
                    win32gui.SetWindowLong(hwnd, win32con.GWL_EXSTYLE, style | win32con.WS_EX_LAYERED)
                    
                    # Set the window transparency
                    windll.user32.SetLayeredWindowAttributes(hwnd, 0, 255, win32con.LWA_ALPHA)
                except Exception as e:
                    print(f"Error applying rounded corners: {e}")
                
                # Set up a timer for plot updates
                self.setup_plot_timer()
                
                plot_window_open = True
                self.update_terminal("Plot window created successfully\n")
            
            # Update title if plot already exists
            else:
                plot_window.setTitle(plot_title)
                self.plot_container.raise_()  # Ensure window is on top when updating
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
        motion = self.system_config["motion"]
        self.configure_trajectory(
            motion["manual_speed_deg_s"], motion["manual_acceleration_deg_s2"]
        )
        axis = self.enter_closed_loop()
        if int(axis.active_errors):
            self.safe_idle_motor("manual movement error")
            raise RuntimeError(f"ODrive has active errors: {int(axis.active_errors)}")
        self.set_status("MANUAL / ACTIVE", "#ffd166")
        return axis

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
            self.stop_continuous_movement()
            self.safe_idle_motor("manual mode disabled")
            # Disable manual controls
            self.left_arrow.configure(state="disabled")
            self.right_arrow.configure(state="disabled")
            self.step_angle_input.configure(state="disabled")
            self.mode_toggle.configure(state="disabled")
            
            # Enable Start button when not in manual mode (only if connected)
            if hasattr(self, 'odrive_controller') and self.odrive_controller:
                self.buttons[1].configure(state="normal")
            
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
    icon_path = str(resource_path("images/icon.ico"))

    # Set the icon for the about dialog
    about_dialog = ctk.CTk()
    
    about_dialog.geometry("560x775")  # Adjust dimensions as needed
    about_dialog.title("About")
    about_dialog.attributes("-topmost", True)  # Set the window to be topmost
    about_dialog.iconbitmap(icon_path)  # Set the icon for the about dialog

    # Frame for content
    content_frame = ctk.CTkFrame(master=about_dialog, width=2000, height=200)
    content_frame.pack(padx=25, pady=25)

    # Application name label (customize text and font)
    app_name_label = ctk.CTkLabel(master=content_frame,
                                text="OrthoSim",
                                font=("Arial", 18, "bold"))
    app_name_label.pack(pady=10)

    # Version label (customize text and font)
    version_label = ctk.CTkLabel(master=content_frame,
                                text="Version: 1.1.0",
                                font=("Arial", 12, "bold"))
    version_label.pack()

    # Author label (customize text and font)
    author_label = ctk.CTkLabel(master=content_frame,
                                text="Developed by: Brock Cooper",
                                font=("Arial", 12, "bold"))
    author_label.pack()

    # Usage
    description_label = ctk.CTkLabel(master=content_frame,
                                text="\n\
This program was designed specifically to be used with the custom AFO tester.\n\n\
The program will display the current angle and weight of the AFO on the plot window.\n\n\
The program will also display the current cycle number and the total cycles in the strain test.\n\n\
In order to start logging data fill in all the fields and click the connect button.\n\n\
The program will connect to the AFO tester and display if the connection is successful.\n\n\
To start a test click the start button and the program will begin logging data.\n\n\
You should see the terminal window start to display the data as the program is logging.\n\n\
The weight and angle values update on the plot window.",
                                font=("Arial", 12), padx=20)
    description_label.pack()


    # Copyright label (customize text and font)
    copyright_label = ctk.CTkLabel(master=content_frame,
                                text="Copyright © 2024 Brock Cooper",
                                font=("Arial", 10))
    copyright_label.pack()

    # Close button
    close_button = ctk.CTkButton(master=content_frame,
                                text="Close",
                                command=about_dialog.destroy)
    close_button.pack(pady=20)


    about_dialog.mainloop()

def main():
    # Ensure there's only one QApplication instance
    if not QApplication.instance():
        app = QApplication(sys.argv)
    else:
        app = QApplication.instance()
    
    root = ctk.CTk()
    root.geometry("1242x786")
    root.resizable(False, False)

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
    
    # Create plot window with delay
    root.after(500, app_instance.create_plot_window)
    root.update_idletasks()
    
    root.mainloop()
    
    # Cleanup Qt application
    if QApplication.instance():
        QApplication.instance().quit()

if __name__ == "__main__":
    main()
