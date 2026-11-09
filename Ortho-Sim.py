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

from Phidget22.Devices.VoltageRatioInput import VoltageRatioInput

import odrive
from odrive.enums import (
    AXIS_STATE_CLOSED_LOOP_CONTROL,
    AXIS_STATE_IDLE,
    CONTROL_MODE_POSITION_CONTROL,
    INPUT_MODE_TRAP_TRAJ,
)

from east_smooth_motion import PRESET as SMOOTH_PAIR
from east_motion_model import X_LIMIT, ORIGINAL_METHOD, UPDATED_METHOD, WIDER_METHOD, selected_method, selected_variant, CALIBRATION_HASHES, RunModel, enabled as motion_model_enabled, validate_range as validate_model_range, provenance as model_provenance

from east_core import (
    CSV_COLUMNS,
    ROTARY_RUN_COLUMNS,
    MOTOR_TIMING_COLUMNS,
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
from east_rotary_homing import ZERO_TOLERANCE_DEG, RotaryZeroMiss
from east_odrive import ODriveAdapter
from east_shutdown_ui import BlockedShutdownDialog
from east_execution import TestEngine, TransportOwner, Lifecycle, AcquisitionLoop, RetainedConnectionRecovery, FeedbackMonitor
from east_runtime import CSVRecorder, CheckpointWriter, write_worker_diagnostics
from east_rotary_sensor import RotarySensorService, RotaryError
from east_reference import (
    AtomicStateStore,
    FeedbackError,
    FeedbackAcquisitionTimeout,
    IdleFeedbackRetry,
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

from east_operator_ui import (
    APP_NAME, APP_VERSION, BG, PANEL, PANEL_SOFT, TEXT, MUTED, GREEN, BLUE,
    RED, AMBER, OperatorView, draw_operator_plot, resource_path,
)

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

class MyInterface(OperatorView):
    def __init__(self, master):
        self.master = master
        self.master.title(f"{APP_NAME} {APP_VERSION}")

        self.system_config = load_tester_config(resource_path("tester_config.json"))
        self.odrive_controller = None
        self.odrive_adapter = None
        self.hardware_fingerprint = None
        self.voltage_ratio_input = None
        self.rotary_sensor = RotarySensorService(self.system_config["rotary_sensor"], resource_path("."))
        self.last_rotary_logged_key = None
        self.last_rotary_notice = None
        self.rotary_samples_file_name = None
        self.empty_machine_tare = None
        self.protocol_display_to_key = {
            "Custom": "custom",
            **{
                preset["display_name"]: key
                for key, preset in self.system_config["test_presets"].items()
                if not preset.get("hidden_in_menu", False)
            },
        }
        self.motion_method_var = ctk.StringVar(value=selected_method(self.system_config))
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
        self.stop_worker_lock = threading.Lock()
        self.stop_worker = None
        self.stop_token = None
        self.stop_operation = None
        self.stop_sequence = 0
        self.stop_result = None
        self.last_idle_observation = None
        self.stop_started_monotonic = None
        self.stop_diagnostic_recorded = False
        self.checkpoint_writer = None
        self.csv_recorder = None
        self.failure_metadata_worker = None
        self.metadata_workers = []
        self.shutdown_worker = None
        self.shutdown_requested = False
        self.shutdown_started_monotonic = None
        self.isolated_exit_requested = False
        self.isolated_exit_offered = False
        self.disconnect_worker = None
        self.acquisition_overruns = 0
        self.maximum_acquisition_lateness_s = 0.0
        self.run_finalized = True
        self.operator_stop_requested = False
        self.acquisition_done_event = threading.Event()
        self.motion_phase = "idle"
        self.commanded_odrive_velocity = 0.0
        self.commanded_afo_acceleration = 0.0
        self.current_nominal_distance_deg = 0.0
        self.current_expected_constant_speed_span_deg = 0.0
        self.ui_message_queue = queue.Queue()
        self.plot_data_queue = queue.Queue(maxsize=4096)
        self.engine = TestEngine(self.ui_message_queue)
        self.transport_owner = None
        self.shutdown_dialog = None
        self.ui_timing = {"drains": 0, "maximum_drain_s": 0.0, "maximum_gap_s": 0.0}
        self.last_ui_drain = time.monotonic()
        self.operator_context = {}
        self.monitor_timings = []
        self.header_images = []

        self.state_store = AtomicStateStore(runtime_state_directory())
        self.process_lock = ProcessLock(self.state_store.lock_path)
        self.startup_block_reason = None
        self.previous_unsafe_exit = False
        self.unsafe_exit_recovered_in_session = False
        try:
            self.process_lock.acquire()
            self.previous_unsafe_exit = self.state_store.unsafe_exit_present()
            # The marker is set before hardware use, so even a blocked exit-time
            # persistence thread cannot make the next launch look clean.
            self.state_store.mark_unsafe_exit()
        except (MotionConflictError, OSError) as exc:
            self.startup_block_reason = str(exc)
        self.reference_manager = ReferenceManager(self.system_config, self.state_store)
        self.motion_coordinator = MotionCoordinator()
        self.feedback_lock = threading.RLock()
        self.latest_feedback = None
        self.monitor_error = None
        self.feedback_fault_reason = None
        self.feedback_retry_pending = False
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
        self.cleanup_thread = None
        self.neutral_thread = None
        self.manual_step_thread = None
        self.continuous_thread = None
        self.recovery_thread = None
        self.recovery_window = None
        self.rotary_calibration = None
        self.rotary_homing = None
        self.motion_mapping = None
        self.run_motion_mapping_context = None
        self.odrive_connection_generation = 0
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

        self.master.bind('<Control-Shift-D>', lambda _event: self.save_stationary_diagnostics())
        self.master.bind('<Escape>', lambda _event: self.stop_logging())
        self.master.bind_all('<Escape>', lambda _event: self.stop_logging())


 
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


    @staticmethod
    def _replace_entry_value(entry, value):
        entry.delete(0, ctk.END)
        entry.insert(0, str(value))





    def select_motion_method(self, selection):
        """Session selection only. Never change a frozen run or batch."""
        previous = selected_method(self.system_config)
        blocked = (self.strain_test_active or self.motion_coordinator.owner is not None
                   or self.engine.busy is not None or self.motion_mapping_active()
                   or self.rotary_homing_active() or self.shutdown_requested
                   or not self.run_finalized or self.motion_coordinator.stopping)
        if blocked or selection not in (ORIGINAL_METHOD, UPDATED_METHOD, WIDER_METHOD):
            self.motion_method_var.set(previous)
            self.update_terminal("Motion method changes are blocked during movement, a sequence or unresolved cleanup.\n")
            return False
        settings = dict(self.system_config['motion'].get('experimental_model',{}))
        settings['enabled'] = selection != ORIGINAL_METHOD
        settings['variant'] = 'candidate' if selection == WIDER_METHOD else 'current'
        self.system_config['motion']['experimental_model'] = settings
        self.motion_method_var.set(selection)
        self.update_protocol_status()
        self.update_terminal(f"Motion method selected: {selection}. Applies to the next normal test or protocol batch. Motion Modelling identification stays Original.\n")
        return True

    def load_selected_preset(self):
        if self.motion_mapping_active() is True:
            self.update_terminal("Preset changes are blocked during an active sequence.\n")
            return
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
        # CTkEntry ignores replacement while disabled (sequence speed is read-only).
        for entry in motion_entries.values():
            entry.configure(state="normal")
        if preset is None:
            for entry in motion_entries.values():
                entry.delete(0, ctk.END)
        else:
            for key, entry in motion_entries.items():
                value = preset['speeds_deg_s'][0] if key == 'speed_deg_s' and preset.get('execution_kind') == 'speed_sequence' else preset[key]
                self._replace_entry_value(entry, value)
        self.loaded_preset_key = selected_key
        self.preset_loaded_at = datetime.now().astimezone().isoformat(
            timespec="milliseconds"
        )
        self.update_parameter_summary()
        self.set_test_inputs_state('normal')

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
        if threading.current_thread() is not threading.main_thread():
            self.ui_message_queue.put(("inputs", state))
            return
        if state == "normal" and self.motion_coordinator.shutting_down:
            return
        if state == "normal" and (self.motion_mapping_active() or self.rotary_homing_active()):
            state = "disabled"
        for entry in (
            self.file_name_input, self.cycles_input, self.speed_input,
            self.acceleration_input, self.min_angle_input, self.max_angle_input,
            self.operator_input, self.afo_id_input, self.fixture_id_input,
            self.calibration_id_input,
        ):
            entry.configure(state=state)
        preset = self._loaded_preset()
        if state == 'normal' and preset and preset.get('execution_kind') == 'speed_sequence':
            self.speed_input.configure(state='disabled')
        selector = getattr(self,"motion_method_menu",None)
        if selector is not None:
            selector.configure(state=state)
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
        if self.rotary_reference_verified():
            mapping = manager.require_verified()
            text = "Rotary zero: VERIFIED"
            engineering = getattr(self, "show_engineering", None)
            if engineering is not None and engineering.get():
                text += f"\nMotor reference: {mapping.neutral_position_turns:.8f} session turns"
            colour = GREEN
        elif manager.confidence == ReferenceConfidence.FAULT:
            text = f"Rotary zero: HOME REQUIRED\n{manager.reason}"
            colour = RED
        else:
            text = f"Rotary zero: HOME REQUIRED\n{manager.reason}"
            colour = AMBER
        if hasattr(self, "reference_status_label"):
            self.reference_status_label.configure(text=text, text_color=colour)
        self.update_tare_display()
        self._refresh_motion_controls()

    def update_tare_display(self):
        if threading.current_thread() is not threading.main_thread():
            self.ui_message_queue.put(("tare_display",))
            return
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
        self.engine.record("tare", {"valid":False,"reason":reason})
        self.update_terminal(f"Empty-machine tare invalidated: {reason}.\n")
        self.update_tare_display()
        self._refresh_motion_controls()

    @staticmethod
    def _set_widget_state(widget, state):
        if widget.cget("state") != state:
            widget.configure(state=state)

    def _submit_lifecycle(self, name, callback):
        if self.motion_coordinator.shutting_down or self.engine.closing:
            self.update_terminal(f"{name} blocked during shutdown.\n")
            return False
        self.operator_context = {
            "operator": self.operator_input.get().strip(),
            "fixture": self.fixture_id_input.get().strip(),
            "calibration": self.calibration_id_input.get().strip(),
        }
        future = self.engine.submit(name, callback)
        if future.done() and future.exception() is not None:
            self.update_terminal(f"{name} blocked: {future.exception()}\n")
            return False
        self.set_test_inputs_state("disabled")
        self._refresh_motion_controls()
        return True

    def _transport_motion_allowed(self):
        return (not self.motion_coordinator.shutting_down
                and not self.motion_coordinator.stopping
                and self.monitor_thread is not None and self.monitor_thread.is_alive()
                and not self.feedback_retry_pending and not self.monitor_error)

    def motion_mapping_active(self):
        window = getattr(self, "motion_mapping", None)
        return window is not None and getattr(window, "active", False) is True

    def _refresh_motion_controls(self):
        if threading.current_thread() is not threading.main_thread():
            self.ui_message_queue.put(("run_buttons",))
            return
        if not hasattr(self, "buttons"):
            return
        self._retire_stop_worker()
        if hasattr(self,"run_timing_label"):
            from east_run_timing import timing_text
            self.run_timing_label.configure(text=timing_text(self))
        connected = self.odrive_adapter is not None
        verified = self.rotary_reference_verified()
        tare_ready = self.empty_machine_tare is not None and not (
            empty_machine_tare_identity_mismatches(
                self.empty_machine_tare,
                self.fixture_id_input.get(),
                self.calibration_id_input.get(),
                self.system_config,
            )
        )
        mapping_active = self.motion_mapping_active() or self.rotary_homing_active()
        idle_ui = (
            not mapping_active
            and self.engine.busy is None
            and not self.strain_test_active
            and not self.motion_coordinator.shutting_down
            and not self.motion_coordinator.stopping
            and not self.shutdown_requested
            and not self.feedback_retry_pending
            and (self.stop_worker is None or not self.stop_worker.is_alive())
            and (self.disconnect_worker is None or not self.disconnect_worker.is_alive())
            and (self.cleanup_thread is None or not self.cleanup_thread.is_alive())
            and not any(worker.is_alive() for worker in self.metadata_workers)
            and self.motion_coordinator.owner is None
            and self.motion_coordinator.motor_idle_confirmed
        )
        healthy = (not self.monitor_error and not self.feedback_retry_pending
                   and self.monitor_thread is not None and self.monitor_thread.is_alive())
        try:
            self.get_feedback()
        except FeedbackError:
            healthy = False
        recovery_ui = (not mapping_active and self.engine.busy is None and not self.shutdown_requested
            and self.motion_coordinator.owner is None
            and not any(worker and worker.is_alive() for worker in (
                self.stop_worker, self.disconnect_worker, self.cleanup_thread,
                self.strain_thread, self.data_collection_thread, self.manual_step_thread,
                self.neutral_thread, self.continuous_thread, self.recovery_thread)))
        self._set_widget_state(self.buttons[0], 'normal' if recovery_ui and (not self.startup_block_reason) else 'disabled')
        idle_ui = idle_ui and healthy
        self._set_widget_state(self.reference_action_button, 'normal' if connected and recovery_ui and not self.startup_block_reason else 'disabled')
        self._set_widget_state(self.buttons[1], 'normal' if connected and tare_ready and idle_ui and self.run_finalized and (not self.manual_mode.get()) and self.rotary_sensor.measurement()['valid'] else 'disabled')
        self._set_widget_state(self.manual_mode_toggle, 'normal' if connected and verified and idle_ui else 'disabled')
        manual_enabled = connected and verified and idle_ui and self.manual_mode.get()
        manual_state = "normal" if manual_enabled else "disabled"
        for widget in (
            self.left_arrow,
            self.right_arrow,
            self.step_angle_input,
            self.manual_step_units_menu,
            self.mode_toggle,
        ):
            self._set_widget_state(widget, manual_state)
        if self.manual_step_units.get() == "Motor turns":
            self._set_widget_state(self.mode_toggle, 'disabled')
        self._set_widget_state(self.neutral_button, 'normal' if connected and recovery_ui and not self.startup_block_reason else 'disabled')
        self._set_widget_state(self.tare_button, 'normal' if connected and verified and idle_ui else 'disabled')
        self._set_widget_state(self.clear_session_tare_button, 'normal' if not mapping_active and self.engine.busy is None and self.empty_machine_tare is not None and (not self.motion_coordinator.shutting_down) and (not self.strain_test_active) and (self.motion_coordinator.owner is None) else 'disabled')

    def _persistence_fault(self, domain, error):
        self.engine.record(domain, {"healthy":False,"reason":str(error)})
        self._request_coordinated_stop(f"{domain}: {error}", fault=True)

    def _start_feedback_monitor(self):
        self.monitor_stop_event.set()
        prior = self.monitor_thread
        if prior and prior.is_alive() and prior is not threading.current_thread():
            prior.join(timeout=1.0)
        if prior and prior.is_alive():
            raise RuntimeError("Previous feedback monitor is still running")
        prior_writer = self.checkpoint_writer
        if prior_writer is not None:
            prior_writer.stop(timeout_s=.1)
            if prior_writer.thread.is_alive():
                raise RuntimeError("Previous checkpoint writer is still running")
        self.monitor_stop_event = threading.Event()
        self.feedback_retry_pending = False
        self.checkpoint_writer = CheckpointWriter(
            self.reference_manager,
            lambda exc: self._persistence_fault("persistence_health", exc),
        )
        self.checkpoint_writer.start()
        monitor_connection = self.odrive_connection_generation
        monitor_adapter = self.odrive_adapter
        self.monitor_thread = threading.Thread(
            target=self._feedback_monitor_loop,
            args=(monitor_connection, monitor_adapter),
            name="odrive-feedback-monitor",
            daemon=True,
        )
        self.monitor_thread.start()

    def _stop_feedback_monitor(self):
        self.monitor_stop_event.set()
        thread = self.monitor_thread
        if thread and thread.is_alive() and thread is not threading.current_thread():
            thread.join(timeout=0.2 if threading.current_thread() is threading.main_thread() else 1.5)
        if not thread or not thread.is_alive():
            self.monitor_thread = None
        writer = self.checkpoint_writer
        if writer is not None:
            writer.stop(timeout_s=0.0 if threading.current_thread() is threading.main_thread() else 1.0)
            if not writer.thread.is_alive():
                if writer.error is not None:
                    self.update_terminal(f"Retired failed checkpoint writer: {writer.error}\n")
                self.checkpoint_writer = None

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
            self._request_coordinated_stop(self.monitor_error, fault=True)
            self.ui_message_queue.put(("terminal", f"SAFETY STOP: {self.monitor_error}\n"))
            self.ui_message_queue.put(("status", "FAULT / IDLE UNCONFIRMED", RED))
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
        if thread is None or not thread.is_alive():
            self.watchdog_thread = None

    def _record_feedback_observation(self, snapshot, connection_generation, adapter):
        """Publish a successful read without clearing a latched feedback fault."""
        with self.feedback_lock:
            if (connection_generation != self.odrive_connection_generation
                    or adapter is not self.odrive_adapter):
                return False
            previous = self.latest_feedback
            if previous is None or snapshot.captured_monotonic_s >= previous.captured_monotonic_s:
                self.latest_feedback = snapshot
            if getattr(self, "engine", None) is not None:
                observation = {
                    "position_at": snapshot.captured_monotonic_s,
                    "recorded_at": time.monotonic(),
                    "connection_generation": connection_generation,
                }
                self.engine.record("feedback_latest", observation)
                if self.feedback_fault_reason is None:
                    self.engine.record("feedback_health", {"healthy": True, **observation})
            return True

    def _latch_feedback_fault(self, reason, connection_generation=None, adapter=None):
        """Latch unhealthy feedback until validated connection recovery clears it."""
        with self.feedback_lock:
            if (connection_generation is not None
                    and connection_generation != self.odrive_connection_generation):
                return False
            if adapter is not None and adapter is not self.odrive_adapter:
                return False
            self.feedback_fault_reason = str(reason)
            self.monitor_error = str(reason)
            if getattr(self, "engine", None) is not None:
                self.engine.record("feedback_health", {
                    "healthy": False,
                    "reason": self.feedback_fault_reason,
                    "connection_generation": self.odrive_connection_generation,
                })
            return True

    def _clear_feedback_fault_after_recovery(self, snapshot, connection_generation, adapter):
        """Clear the latch only after the caller validates an explicit recovery."""
        snapshot.validate(
            now_monotonic_s=time.monotonic(),
            max_age_s=self.system_config["reference"]["feedback_stale_after_ms"] / 1000.0,
            max_capture_duration_s=(
                self.system_config["reference"]["maximum_feedback_capture_ms"] / 1000.0
            ),
        )
        if snapshot.active_errors or snapshot.current_state != int(AXIS_STATE_IDLE):
            raise RuntimeError("Feedback fault recovery requires fresh error-free controller idle")
        with self.feedback_lock:
            if (connection_generation != self.odrive_connection_generation
                    or adapter is not self.odrive_adapter):
                raise RuntimeError("Recovered feedback belongs to a retired connection")
            self.feedback_fault_reason = None
            self.monitor_error = None
            self.latest_feedback = snapshot
            if getattr(self, "engine", None) is not None:
                observation = {
                    "position_at": snapshot.captured_monotonic_s,
                    "recorded_at": time.monotonic(),
                    "connection_generation": connection_generation,
                    "validated_recovery": True,
                }
                self.engine.record("feedback_latest", observation)
                self.engine.record("feedback_health", {"healthy": True, **observation})
                self.engine.record("feedback_recovery", {
                    "validated": True,
                    "connection_generation": connection_generation,
                    "position_at": snapshot.captured_monotonic_s,
                })

    def _feedback_monitor_loop(self, monitor_connection, monitor_adapter):
        with self.feedback_lock:
            initial = self.latest_feedback
        def publish(snapshot):
            if not self._record_feedback_observation(
                    snapshot, monitor_connection, monitor_adapter):
                raise RuntimeError("Retired feedback monitor publication rejected")
            if hasattr(self, "monitor_timings"):
                self.monitor_timings.append({"position_at": snapshot.captured_monotonic_s,
                    "capture_s": snapshot.capture_duration_s, "lock_wait_s": snapshot.lock_wait_s,
                    "queue_wait_s": snapshot.transport_queue_wait_s,
                    "property_brackets": snapshot.property_brackets_monotonic_s})
                if len(self.monitor_timings)>2048:
                    del self.monitor_timings[:len(self.monitor_timings)-2048]
        def checkpoint(snapshot):
            with self.stop_worker_lock:
                if not self.motion_coordinator.stopping:
                    state = MotionState.MOVING if self.motion_coordinator.owner is not None else MotionState.IDLE
                    self.checkpoint_writer.submit(state,snapshot,{"motion_owner":self.motion_coordinator.owner})
        def delay(error, rejected):
            self.feedback_retry_pending=True
            self.checkpoint_writer.submit_event("idle_feedback_sample_rejected", {
                "duration_s":error.snapshot.capture_duration_s,"retry":rejected,"motor_reference_changed":False})
            self.ui_message_queue.put(("terminal", f"Feedback delay while idle: {error}. Sample rejected; retry {rejected}. Motion blocked pending fresh feedback.\n"))
            self.ui_message_queue.put(("status", "FEEDBACK DELAY / RETRYING", AMBER))
            self.ui_message_queue.put(("reference",))
        def restored():
            self.feedback_retry_pending=False
            self.ui_message_queue.put(("terminal", "Fresh idle feedback restored; machine zero retained.\n"))
            self.ui_message_queue.put(("status", "IDLE / FEEDBACK RESTORED", GREEN))
            self.ui_message_queue.put(("reference",))
        def feedback_fault(reason):
            if not self._latch_feedback_fault(
                    reason, monitor_connection, monitor_adapter):
                return
            service = getattr(self, "feedback_service", None)
            snapshot = getattr(service, "fault_snapshot", None)
            if snapshot is not None and getattr(self, "strain_test_active", False) and self.run_metadata is not None:
                self.run_metadata["drive_fault_evidence"] = {
                    "phase": self.motion_phase, "feedback": snapshot.to_dict(),
                }
            self._request_coordinated_stop(reason, fault=True)
        self.feedback_service = FeedbackMonitor(
            lambda:(monitor_adapter if monitor_connection == self.odrive_connection_generation
                    and monitor_adapter is self.odrive_adapter else None),
            self.system_config,self.monitor_stop_event,initial,
            lambda:(self.expected_axis_state,self.expected_disarm_reason),
            lambda:self.motion_coordinator.owner,publish,checkpoint,self._mark_control_health,
            delay,restored,feedback_fault,
            allow_motion_retry=lambda:bool(getattr(self,'run_segmented_comparison',False)
                and (getattr(self,'run_motion_mapping_context',None) or {}).get('stage') in ('commissioning','model_comparison')
                and not self.operator_stop_requested and not self.test_stop_event.is_set()),
            on_motion_delay=lambda exc:self.ui_message_queue.put(('terminal',
                f'Comparison feedback bundle rejected ({exc.snapshot.capture_duration_s:.3f} s). Retrying once without publishing it; prior good feedback must remain fresh.\n')))
        self.feedback_service.run()

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

    def configure_trajectory(self, speed_deg_s, acceleration_deg_s2, turns_per_degree=None, *, filtered=False):
        if self.odrive_adapter is None:
            raise RuntimeError("ODrive is not connected")
        motion = self.system_config["motion"]
        trajectory_velocity = afo_speed_to_odrive_turns_s(speed_deg_s, self.system_config)
        trajectory_acceleration = afo_acceleration_to_odrive_turns_s2(
            acceleration_deg_s2, self.system_config
        )
        if turns_per_degree is not None:
            if not math.isfinite(turns_per_degree) or turns_per_degree <= 0:
                raise ValueError("Invalid model trajectory conversion")
            trajectory_velocity = float(speed_deg_s)*turns_per_degree
            trajectory_acceleration = float(acceleration_deg_s2)*turns_per_degree
            trajectory_velocity = min(trajectory_velocity, afo_speed_to_odrive_turns_s(motion["maximum_speed_deg_s"], self.system_config))
            trajectory_acceleration = min(trajectory_acceleration, afo_acceleration_to_odrive_turns_s2(motion["maximum_acceleration_deg_s2"], self.system_config))
        controller_limit = trajectory_velocity * float(motion["controller_velocity_safety_multiplier"])
        input_mode = INPUT_MODE_TRAP_TRAJ
        options = {}
        if filtered:
            from odrive.enums import INPUT_MODE_POS_FILTER
            from east_smooth_motion import BANDWIDTH
            input_mode = INPUT_MODE_POS_FILTER
            options["filter_bandwidth"] = BANDWIDTH
            # The filtered preset keeps one drive configuration through its
            # neutral return. Give the return reference the same headroom as a sweep.
            return_velocity = afo_speed_to_odrive_turns_s(
                motion["neutral_return_speed_deg_s"], self.system_config
            )
            controller_limit = max(trajectory_velocity, return_velocity) * float(
                motion["controller_velocity_safety_multiplier"]
            )
        self.odrive_adapter.configure_trajectory(
            trajectory_velocity,
            trajectory_acceleration,
            controller_limit,
            CONTROL_MODE_POSITION_CONTROL,
            input_mode,
            **options,
        )
        self.commanded_odrive_velocity = trajectory_velocity
        self.commanded_odrive_acceleration = trajectory_acceleration
        self.commanded_afo_acceleration = float(acceleration_deg_s2)

    def odrive_configuration_snapshot(self):
        if self.odrive_adapter is None:
            raise RuntimeError("ODrive is not connected")
        return self.odrive_adapter.read_only_report(include_phase=False)

    def _write_runtime_checkpoint(self, motion_state, snapshot, clean_shutdown=False, extra=None):
        writer = self.checkpoint_writer
        if writer is not None:
            writer.write_and_wait(motion_state, snapshot, clean_shutdown, extra)
        else:
            self.reference_manager.write_checkpoint(motion_state, snapshot,
                                                    clean_shutdown=clean_shutdown, extra=extra)

    def safe_idle_motor(self, reason=None, invalidate_reference=False, token=None, *, retain_owner=False):
        active_token = self.motion_coordinator.active_token
        if token is not None and active_token not in (None, token):
            self.update_terminal(
                f"Ignored stale cleanup from {token.owner}; "
                f"motion is owned by {active_token.owner}.\n"
            )
            return None
        self.continuous_movement_active = False
        self.continuous_stop_event.set()
        if self.movement_timer and threading.current_thread() is threading.main_thread():
            try:
                self.master.after_cancel(self.movement_timer)
            except Exception:
                pass
            self.movement_timer = None
        if retain_owner:
            if token is None:
                raise MotionConflictError("Owned zero observation requires a motion token")
            self.motion_coordinator.assert_active(token)
        else:
            self.motion_coordinator.request_stop()
        self.expected_axis_state = None
        self.expected_disarm_reason = None
        if self.odrive_adapter is None:
            return None
        try:
            result = self.odrive_adapter.request_idle(
                self.system_config["reference"]["idle_confirmation_timeout_s"]
            )
            if result.confirmed:
                self.last_idle_observation = {
                    "observed_at": result.observed_at,
                    "observed_monotonic_s": result.observed_monotonic_s,
                    "connection_generation": self.odrive_connection_generation,
                    "state": result.current_state,
                    "active_errors": result.active_errors,
                }
                self.motion_coordinator.confirm_motor_idle()
                if getattr(self, "engine", None) is not None:
                    self.engine.record("idle", dict(self.last_idle_observation))
            self.motion_phase = "idle"
            if invalidate_reference or not result.confirmed:
                self.reference_manager.invalidate_local(
                    reason or result.detail,
                    fault=not result.confirmed,
                )
            snapshot = None
            feedback_failure = None
            try:
                snapshot = self._read_adapter_feedback() if result.confirmed else None
                if snapshot is not None:
                    with self.feedback_lock:
                        self.latest_feedback = snapshot
            except Exception as feedback_exc:
                feedback_failure = str(feedback_exc)
                feedback_fault = f"Idle observed, fresh feedback unavailable: {feedback_exc}"
                self._latch_feedback_fault(
                    feedback_fault, self.odrive_connection_generation, self.odrive_adapter)
                self.reference_manager.invalidate_local(self.monitor_error, fault=True)
                self.update_terminal(self.monitor_error + "\n")
                self.ui_message_queue.put(("status", "IDLE OBSERVED / FEEDBACK FAULT", RED))
            try:
                self._write_runtime_checkpoint(
                    MotionState.IDLE if result.confirmed else MotionState.FAULT,
                    snapshot, clean_shutdown=False,
                    extra={"idle_result":result.__dict__,"reason":reason,"feedback_failure":feedback_failure},
                )
                if getattr(self, "engine", None) is not None:
                    self.engine.record("persistence_health", {"healthy":True,"operation":"idle checkpoint"})
            except Exception as checkpoint_exc:
                self.update_terminal(f"Unable to persist idle checkpoint: {checkpoint_exc}\n")
                self.reference_manager.invalidate_local(f"Runtime-state write failed: {checkpoint_exc}", fault=True)
                if getattr(self, "engine", None) is not None:
                    self.engine.record("persistence_health", {"healthy":False,"reason":str(checkpoint_exc)})
                self.ui_message_queue.put(("status", "IDLE OBSERVED / PERSISTENCE FAULT", RED))
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
                self.reference_manager.invalidate_local(
                    f"Idle confirmation failed: {exc}", fault=True
                )
            except Exception:
                pass
            if threading.current_thread() is threading.main_thread():
                self.update_reference_display()
            else:
                self.ui_message_queue.put(("reference",))
            return None

    def _persist_interim_outcome(self, reason, fault):
        with self.finalize_lock:
            if self.run_metadata is None or not self.metadata_file_name:
                return
            operation = getattr(self, "stop_operation", None)
            isolated = getattr(self, "isolated_exit_requested", False)
            if (operation is not None and operation["metadata_file"] != self.metadata_file_name
                    and not isolated):
                return
            fault = isolated or fault or bool(operation and operation["fault"])
            if self.run_finalized and not fault:
                return
            self.run_metadata["run_status"] = "error" if fault else "aborted"
            causes = operation["reasons"] if operation and operation["metadata_file"] == self.metadata_file_name else [reason]
            if isolated and reason not in causes:
                causes = [*causes, reason]
            self.run_metadata["error"] = "; ".join(causes)
            self.run_metadata["outcome_pending_cleanup"] = True
            self.run_metadata["idle_confirmation"] = {
                "confirmed": False,
                "motion_owner": self.motion_coordinator.owner,
                "monitor_error": self.monitor_error,
            }
            self.run_metadata["sample_count"] = self.sample_count
            self.run_metadata["acquisition_overruns"] = self.acquisition_overruns
            self.run_metadata["maximum_acquisition_lateness_s"] = self.maximum_acquisition_lateness_s
            try:
                write_json_atomic(Path(self.metadata_file_name), self.run_metadata)
            except Exception as exc:
                self.ui_message_queue.put(("terminal", f"RUN ERROR: could not persist incomplete outcome: {exc}\n"))

    def _request_coordinated_stop(self, reason, fault=False):
        # Worker callers only latch an Event; Tk and batch finalization stay on the UI thread.
        mapping = getattr(self, "motion_mapping", None)
        if mapping is not None and getattr(mapping, "active", False) is True:
            mapping.cancelled.set()
        owner = getattr(self, "transport_owner", None)
        if owner is not None:
            owner.cancel_motion()
        engine = getattr(self, "engine", None)
        if engine is not None:
            engine.cancel_current()
            engine.transition(Lifecycle.STOPPING)
        self.test_stop_event.set()
        self.neutral_stop_event.set()
        self.continuous_stop_event.set()
        if fault:
            if engine is not None:
                engine.record("fault", {"reason":reason})
            self.monitor_error = reason
            self.acquisition_error = reason if self.strain_test_active else getattr(self, "acquisition_error", None)
            self.reference_manager.invalidate_local(reason, fault=True)
            self.ui_message_queue.put(("reference",))
        if self.odrive_adapter is None and self.motion_coordinator.owner is None:
            self.ui_message_queue.put(("terminal", f"Stop acknowledged; ODrive is not connected: {reason}.\n"))
            return
        self.motion_coordinator.request_stop()
        with self.stop_worker_lock:
            token = self.motion_coordinator.active_token
            connection = self.odrive_connection_generation
            previous = self.stop_worker
            operation = self.stop_operation
            if (previous is not None and previous.is_alive() and operation is not None
                    and not operation["closed"] and operation["connection"] == connection
                    and operation["token"] == token):
                new_reason = reason not in operation["reasons"]
                if new_reason:
                    operation["reasons"].append(reason)
                # request_stop above invalidated the idle acknowledgement even
                # for a duplicate non-fault Stop. The running worker must obtain
                # a fresh acknowledgement before retiring this revision.
                operation["revision"] += 1
                self.stop_result = None
                if fault:
                    operation["fault"] = True
                    self.motion_coordinator.begin_async_stop()
                if fault and operation["metadata_file"] is not None:
                    worker = threading.Thread(target=self._persist_interim_outcome,
                                              args=(reason, True), name="east-fault-metadata", daemon=True)
                    self.metadata_workers.append(worker)
                    worker.start()
                return
            self.motion_coordinator.begin_async_stop()
            self.expected_axis_state = None
            self.expected_disarm_reason = None
            self.stop_sequence += 1
            operation = {"sequence": self.stop_sequence, "connection": connection,
                         "token": token, "reasons": [reason], "fault": fault,
                         "revision": 0, "closed": False,
                         "metadata_file": (self.metadata_file_name if self.run_metadata is not None
                                           and (self.strain_test_active or not self.run_finalized
                                                or self.run_metadata.get("outcome_pending_cleanup")) else None)}
            self.stop_operation = operation
            if operation["metadata_file"] is not None and not self.run_finalized:
                self.failure_metadata_worker = threading.Thread(
                    target=self._persist_interim_outcome,
                    args=(reason, fault), name="east-incomplete-metadata", daemon=True,
                )
                self.metadata_workers.append(self.failure_metadata_worker)
                self.failure_metadata_worker.start()
            self.stop_token = token
            self.stop_result = None
            self.stop_started_monotonic = time.monotonic()
            self.stop_diagnostic_recorded = False
            self.stop_worker = threading.Thread(
                target=self._coordinated_stop_worker,
                args=(operation, previous if previous is not None and previous.is_alive() else None),
                name="east-idle-stop", daemon=True,
            )
            self.stop_worker.start()
        self.ui_message_queue.put(("status", "STOPPING / IDLE UNCONFIRMED", RED))
        self.ui_message_queue.put(("terminal", f"Stop latched: {reason}; awaiting idle confirmation.\n"))

    def _coordinated_stop_worker(self, operation, predecessor=None):
        try:
            if predecessor is not None:
                predecessor.join()
            while True:
                with self.stop_worker_lock:
                    revision = operation["revision"]
                    fault = operation["fault"]
                    reason = "; ".join(operation["reasons"])
                result = self.safe_idle_motor(reason, invalidate_reference=fault,
                                              token=operation["token"])
                with self.stop_worker_lock:
                    if operation["revision"] != revision:
                        # Another Stop invalidated the acknowledgement during this
                        # attempt. Reconfirm idle, including any fault upgrade.
                        continue
                    operation["closed"] = True
                    if self.stop_operation is operation:
                        self.stop_result = result
                    break
            if getattr(self, "engine", None) is not None:
                self.engine.transition(Lifecycle.FAULT if operation["fault"] or not (result and result.confirmed) else Lifecycle.READY)
            self.ui_message_queue.put((
                "status",
                "STOPPED / IDLE" if result and result.confirmed
                else "STOPPED / IDLE UNCONFIRMED",
                AMBER if result and result.confirmed else RED,
            ))
        except Exception as exc:
            with self.stop_worker_lock:
                operation["closed"] = True
                if self.stop_operation is operation:
                    self.stop_result = None
            self.ui_message_queue.put(("terminal", f"Idle worker failed: {exc}\n"))
            self.ui_message_queue.put(("status", "STOPPED / IDLE UNCONFIRMED", RED))
    def _retire_stop_worker(self):
        with self.stop_worker_lock:
            worker = self.stop_worker
            if (worker is not None and not worker.is_alive()
                    and self.stop_operation is not None and self.stop_operation["closed"]):
                self.motion_coordinator.complete_async_stop()

    def _diagnose_stalled_workers(self):
        workers = (self.stop_worker, self.strain_thread, self.data_collection_thread,
                   self.failure_metadata_worker, self.disconnect_worker, self.shutdown_worker,
                   getattr(self, "watchdog_thread", None), *self.metadata_workers)
        if (not any(worker is not None and worker.is_alive() for worker in workers)
                or self.stop_diagnostic_recorded
                or self.stop_started_monotonic is None
                or time.monotonic() - self.stop_started_monotonic < 2.0):
            return
        self.stop_diagnostic_recorded = True
        self.set_status("STOPPED / CLEANUP PENDING" if self.motion_coordinator.motor_idle_confirmed
                        else "STOPPED / IDLE UNCONFIRMED", RED)
        self.update_terminal("Stop/cleanup is still running; idle or data finalization may be unconfirmed. Use the physical E-stop if motion persists.\n")
        gui_timing = dict(self.ui_timing)
        motor_timings = list(self.monitor_timings)
        def write_stacks():
            try:
                path = self.state_store.directory / "stalled_workers.log"
                write_worker_diagnostics(path, [self.stop_worker, self.monitor_thread,
                    self.strain_thread, self.data_collection_thread,
                    self.disconnect_worker, self.shutdown_worker,
                    self.checkpoint_writer.thread if self.checkpoint_writer else None],
                    f"feedback_age_s={time.monotonic() - self.latest_feedback.captured_monotonic_s if self.latest_feedback else 'unavailable'} "
                    f"feedback_capture_s={self.latest_feedback.capture_duration_s if self.latest_feedback else 'unavailable'} "
                    f"odrive_lock_wait_s={self.odrive_adapter.last_lock_wait_s if self.odrive_adapter else 'unavailable'} "
                    f"odrive_transport_s={self.odrive_adapter.last_transport_duration_s if self.odrive_adapter else 'unavailable'} "
                    f"checkpoint_disk_s={self.checkpoint_writer.last_disk_duration_s if self.checkpoint_writer else 'unavailable'}")
                write_json_atomic(path.with_suffix(".timing.json"), {"gui": gui_timing, "motor": motor_timings})
                self.ui_message_queue.put(("terminal", f"Worker stack diagnostic: {path}\n"))
            except Exception as exc:
                self.ui_message_queue.put(("terminal", f"Worker diagnostics failed: {exc}\n"))
        threading.Thread(target=write_stacks, name="east-stack-diagnostic", daemon=True).start()

    def enter_closed_loop(self, token, allow_unreferenced=False):
        if getattr(self, "monitor_error", None):
            raise FeedbackError("Recover the stopped feedback monitor before motor enable")
        if self.feedback_retry_pending:
            raise FeedbackError("Motor enable blocked while idle feedback is being rechecked")
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
        self._write_runtime_checkpoint(
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
        # Publish the actual arming observation before settle checks can consume
        # the earlier cached idle state. Older monitor bundles cannot overwrite it.
        with self.feedback_lock:
            self.latest_feedback = armed
            self.expected_disarm_reason = armed.disarm_reason
            self.expected_axis_state = int(AXIS_STATE_CLOSED_LOOP_CONTROL)
        self._mark_control_health()
        return armed

    def _motion_cancelled(self, token):
        if self.feedback_retry_pending:
            return True
        try:
            self.motion_coordinator.assert_active(token)
            return False
        except MotionConflictError:
            return True

    def _submit_position(self, token, target_turns):
        if self.feedback_retry_pending:
            raise FeedbackError("Position command blocked while idle feedback is being rechecked")
        if self.odrive_adapter is None:
            raise RuntimeError("ODrive disconnected during motion")
        from east_execution import MotionCommandCancelled
        try:
            self.motion_coordinator.submit_command(
                token,
                lambda: self.odrive_adapter.command_position(
                    target_turns, cancelled=lambda: self._motion_cancelled(token)),
            )
        except (MotionCommandCancelled,MotionConflictError) as exc:
            if (self.operator_stop_requested and self.test_stop_event.is_set()
                    and not self.monitor_error and not getattr(self,'acquisition_error',None)):
                raise TestStopped() from exc
            raise
        self._mark_control_health()

    def _powered_wait_requirements(self):
        return {
            "expected_state": int(AXIS_STATE_CLOSED_LOOP_CONTROL),
            "expected_disarm_reason": self.expected_disarm_reason,
            "progress": self._mark_control_health,
        }

    def connect_rotary_sensor(self):
        if self.motion_coordinator.shutting_down:
            self.update_terminal("Sensor reconnect blocked during shutdown.\n")
            return
        if self.strain_test_active or self.motion_coordinator.owner is not None:
            self.update_terminal("Sensor reconnect is blocked while a test/capture/movement is active.\n")
            return
        try:
            self.rotary_sensor.connect()
            self.last_rotary_notice = None
            serial = self.system_config["rotary_sensor"]["hub_serial"]
            self.update_terminal(f"Connecting rotary sensor: HUB0007 {serial}, port 0, voltage ratio.\n")
        except Exception as exc:
            self.update_terminal(f"Rotary connection failed: {exc}\n")

    def _poll_rotary_sensor(self):
        reader = self.rotary_sensor.reader
        if reader:
            for _ in range(200):
                try:
                    kind, payload = reader.events.get_nowait()
                except queue.Empty:
                    break
                if kind in ("progress", "error"):
                    self.update_terminal(f"ROTARY SENSOR: {payload}\n")
                elif kind == "connected":
                    self.update_terminal(
                        f"Rotary channel attached; requested {payload['requested_interval_ms']} ms, "
                        f"configured {payload['interval_ms']} ms; waiting for sample-rate check.\n")
        value = self.rotary_sensor.measurement()
        self.engine.record("rotary_health", {"valid":value["valid"],"reason":value["reason"],"age_s":value.get("sample_age_s"),"rate_hz":value.get("achieved_rate_hz")})
        notice = (value.get("sample_status") if value.get("sample_status")=="linear_extrapolation_unchecked" else "ready") if value["valid"] else value["reason"]
        label = getattr(self, 'homing_live_label', None)
        if label is not None:
            angle_text = f"{value['angle_deg']:+.3f}°" if value['valid'] else value['reason']
            if value.get('sample_status')=='linear_extrapolation_unchecked':angle_text += ' — EXTRAPOLATED (unchecked)'
            label.configure(text=f"Rotary: {angle_text} | Zero band ±0.10° | 0.5 s + idle check")
        if notice != self.last_rotary_notice:
            if notice=="linear_extrapolation_unchecked":
                self.update_terminal("Comparison measurement uses linear extrapolation outside checked calibration range; accuracy there is unverified.\n")
            elif value["valid"]:
                self.update_terminal(f"Rotary sensor ready: {value['achieved_rate_hz']:.1f} delivered Hz; "
                    "accepted calibration loaded.\n")
            else:
                self.update_terminal(f"Rotary sensor not ready: {notice}\n")
            self.last_rotary_notice = notice
        if self.strain_test_active and not value["valid"] and not self.test_stop_event.is_set():
            if self.run_metadata is not None:
                self.run_metadata.setdefault('rotary_failure_evidence',{k:value.get(k) for k in
                    ('reason','sample_age_s','angle_deg','sample_status','achieved_rate_hz','sample')})
            self._request_coordinated_stop("Rotary sensor: " + value["reason"], fault=True)
        self._refresh_motion_controls()

    def _rotary_acquisition_travel_limit(self):
        from east_feedback_commissioning import policy
        context = getattr(self, "run_motion_mapping_context", None) or {}
        if context.get('protocol_key') == SMOOTH_PAIR:
            return 11.
        selected = policy(self.system_config, context.get("feedback_condition_key", context.get("protocol_key", "")))
        return selected["travel_guard_deg"] if selected else 15.1

    def _test_feedback(self):
        value = self.rotary_sensor.measurement(require=True)
        if (getattr(self,"run_rotary_targeted",False) is True
                or (getattr(self, 'run_motion_mapping_context', None) or {}).get('protocol_key') == SMOOTH_PAIR):
            limit = self._rotary_acquisition_travel_limit()
            if abs(value["angle_deg"]) > limit:
                raise RotaryError(f"Rotary acquisition travel guard exceeded: measured {value['angle_deg']:+.4f} degrees; limit +/-{limit:g}")
        return self.get_feedback()

    def _ensure_load_cell(self):
        if self.voltage_ratio_input is not None:
            try:
                if self.voltage_ratio_input.getAttached():
                    return self.voltage_ratio_input
            except Exception:
                pass
            try:
                self.voltage_ratio_input.close()
            except Exception:
                pass
        self.voltage_ratio_input = self._open_load_cell()
        serial = self.voltage_ratio_input.getDeviceSerialNumber()
        if self.empty_machine_tare and serial != self.empty_machine_tare.phidget_serial_number:
            self.invalidate_empty_machine_tare("connected load-cell interface changed")
        return self.voltage_ratio_input

    def _reuse_live_odrive_connection(self):
        """Reuse a healthy live adapter; never restore a lost machine-zero mapping."""
        if (self.odrive_adapter is None or self.monitor_error or self.feedback_retry_pending
                or self.monitor_thread is None or not self.monitor_thread.is_alive()):
            return False
        try:
            previous = self.get_feedback()
            current = self._read_adapter_feedback()
            self._revalidate_hardware_identity()
            current.validate(now_monotonic_s=time.monotonic(),
                max_age_s=self.system_config["reference"]["feedback_stale_after_ms"]/1000)
            if (current.active_errors or current.current_state != int(AXIS_STATE_IDLE)
                    or current.disarm_reason != previous.disarm_reason
                    or previous.system_uptime is None or current.system_uptime is None
                    or current.system_uptime < previous.system_uptime):
                return False
            velocity_limit = self.system_config["reference"]["stationary_velocity_limit_deg_s"]
            factor = self.system_config["motion"]["afo_degrees_per_odrive_turn"]
            if (abs(current.velocity_turns_s*factor) > velocity_limit
                    or abs(current.position_turns-previous.position_turns)*factor
                    > self.system_config["motion"]["position_tolerance_deg"]):
                return False
            return True
        except Exception:
            return False

    def _recover_retained_connection(self):
        self.engine.transition(Lifecycle.CONNECTING)
        self.motion_coordinator.request_stop()
        self.transport_owner.cancel_motion()
        self.reference_manager.invalidate_local("Coordinate continuity unproven while recovering feedback")
        previous = self.latest_feedback
        def retire():
            self._stop_feedback_monitor()
            if self.monitor_thread is not None or self.checkpoint_writer is not None:
                raise RuntimeError("Recovery waiting for prior monitor/checkpoint worker; connection retained")
        def observed_idle(result):
            self.motion_coordinator.confirm_motor_idle()
            self.last_idle_observation = result.__dict__.copy()
            self.engine.record("idle", self.last_idle_observation)
        retire()
        try:
            evidence = RetainedConnectionRecovery().run(
                self.odrive_adapter, lambda: None, self.system_config, previous, self.hardware_fingerprint,
                on_idle=observed_idle)
        except Exception as retained_error:
            # A retired Fibre object may never reconnect after USB loss. Replace
            # it only after its sole dispatcher exits, cancelling every old request.
            if not self.transport_owner.close(timeout_s=.5):
                raise RuntimeError("Old transport is still executing; recovery retained its connection") from retained_error
            self.update_terminal(f"Retained connection could not supply healthy feedback: {retained_error}. Rediscovering while motion stays blocked.\n")
            device = MyInterface.find_odrive_with_timeout(
                self.system_config["hardware"]["odrive_serial_number"],
                self.system_config["hardware"]["odrive_connection_timeout_s"])
            if device is None or self.engine.closing or self.engine.cancel_event.is_set():
                raise RuntimeError("Recovery discovery failed or was cancelled; reference remains invalid")
            self.odrive_connection_generation += 1
            self.transport_owner = TransportOwner(self.odrive_connection_generation, self._transport_motion_allowed)
            self.odrive_controller = device
            self.odrive_adapter = ODriveAdapter(device,self.system_config["hardware"]["odrive_axis"],
                AXIS_STATE_IDLE,AXIS_STATE_CLOSED_LOOP_CONTROL,transport=self.transport_owner)
            self._validate_live_motion_config()
            evidence = RetainedConnectionRecovery().run(
                self.odrive_adapter, lambda: None, self.system_config, on_idle=observed_idle)
        self.motion_coordinator.confirm_motor_idle()
        self._retire_stop_worker()
        self.last_idle_observation = evidence.idle.__dict__.copy()
        self.engine.record("idle", self.last_idle_observation)
        self.engine.record("reference_continuity", {"proven": evidence.continuity_proven, "reason": evidence.reason})
        self.reference_manager.invalidate_local(evidence.reason)
        with self.feedback_lock:
            self.latest_feedback = evidence.feedback
        self.hardware_fingerprint = evidence.fingerprint
        self._clear_feedback_fault_after_recovery(
            evidence.feedback, self.odrive_connection_generation, self.odrive_adapter)
        self._start_feedback_monitor()
        self.engine.transition(Lifecycle.READY)
        self.set_status("MONITOR RESTORED / ROTARY HOMING REQUIRED", AMBER)
        self.update_terminal(evidence.reason + " No test was restarted.\n")
        self.update_reference_display()

    def _retry_incomplete_outcome(self):
        if self.run_metadata is None or not self.metadata_file_name:
            return
        if self.run_motion_token is not None or any(worker.is_alive() for worker in self.metadata_workers):
            raise RuntimeError("Previous run still owns cleanup; outcome retry deferred")
        if not self.run_finalized:
            self.finalize_run("error",self.run_metadata.get("error") or "Previous finalization failed",
                              idle_confirmed=self.motion_coordinator.motor_idle_confirmed)
            if not self.run_finalized:
                raise RuntimeError("Previous outcome still cannot be persisted; check storage and retry Connect")
        if self.run_metadata.get("outcome_pending_cleanup"):
            with self.finalize_lock:
                self.run_metadata["outcome_pending_cleanup"] = False
                self.run_metadata["idle_confirmation"]["confirmed"] = self.motion_coordinator.motor_idle_confirmed
                self.run_metadata["idle_confirmation"]["motion_owner"] = None
                try:
                    write_json_atomic(Path(self.metadata_file_name),self.run_metadata)
                except Exception:
                    self.run_metadata["outcome_pending_cleanup"] = True
                    raise

    def connect_system(self):
        if self.motion_coordinator.shutting_down:
            self.update_terminal("Reconnect blocked during shutdown.\n")
            return
        if self.rotary_calibration is not None and not self.rotary_calibration.closed:
            self.rotary_calibration.suspend_positioning("ODrive connect/recovery requested")
        self._submit_lifecycle("connect/recover", self._connect_system_worker)

    def _connect_system_worker(self):
        self._retire_stop_worker()
        if self.motion_coordinator.shutting_down:
            self.update_terminal("Reconnect blocked during shutdown; restart EAST after recovery.\n")
            return
        active_threads = [
            getattr(self, name, None)
            for name in (
                "strain_thread", "data_collection_thread", "neutral_thread",
                "manual_step_thread", "continuous_thread", "recovery_thread",
                "stop_worker", "disconnect_worker", "cleanup_thread",
            )
        ]
        recorder = getattr(self, "csv_recorder", None)
        if (
            self.strain_test_active
            or self.motion_coordinator.owner is not None
            or any(thread and thread.is_alive() for thread in active_threads)
            or any(worker.is_alive() for worker in self.metadata_workers)
            or (recorder is not None and recorder.thread.is_alive())
        ):
            self.update_terminal("Cannot reconnect until all motion workers have exited.\n")
            return
        if self.startup_block_reason or not self.process_lock.held:
            self.update_terminal(
                f"Hardware controls are blocked: {self.startup_block_reason or 'process lock unavailable'}\n"
            )
            return
        if self._reuse_live_odrive_connection():
            self.connect_rotary_sensor()
            try:
                self._ensure_load_cell()
            except Exception as exc:
                self.update_terminal(f"Load-cell connection failed: {exc}\n")
            self._retry_incomplete_outcome()
            self.update_terminal("Connect retry reused the live ODrive session; machine zero was not changed.\n")
            self.update_reference_display()
            return
        self.clear_terminal()
        if self.odrive_adapter is not None:
            self._recover_retained_connection()
            self.connect_rotary_sensor()
            self._ensure_load_cell()
            self._retry_incomplete_outcome()
            return
        self.set_status("CONNECTING", AMBER)
        serial_number = self.system_config["hardware"]["odrive_serial_number"]
        timeout_duration = self.system_config["hardware"]["odrive_connection_timeout_s"]

        try:
            self.engine.transition(Lifecycle.CONNECTING)
            self.odrive_controller = MyInterface.find_odrive_with_timeout(serial_number, timeout_duration)
            if self.engine.closing:
                raise RuntimeError("Connection cancelled by shutdown")
            self.odrive_connection_generation += 1
            self.transport_owner = TransportOwner(self.odrive_connection_generation, self._transport_motion_allowed)
            if self.odrive_controller is None:
                raise TimeoutError(f"ODrive {serial_number} was not found")

            self.odrive_adapter = ODriveAdapter(
                self.odrive_controller,
                self.system_config["hardware"]["odrive_axis"],
                AXIS_STATE_IDLE,
                AXIS_STATE_CLOSED_LOOP_CONTROL,
                transport=self.transport_owner,
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
                if self.previous_unsafe_exit:
                    self.reference_manager.invalidate(
                        "Previous power-isolated exit: rotary homing required",
                        fault=True,
                    )
                else:
                    identity_matches, identity_reason = hardware_fingerprints_match(
                        record.hardware_fingerprint, self.hardware_fingerprint
                    )
                    if not identity_matches:
                        self.reference_manager.invalidate(
                            f"{identity_reason}; rotary homing required"
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
            self._clear_feedback_fault_after_recovery(
                initial, self.odrive_connection_generation, self.odrive_adapter)
            self._start_feedback_monitor()
            self._connection_zero_pending=True
            self.engine.transition(Lifecycle.READY)
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
            if self.transport_owner is not None and not self.transport_owner.close(timeout_s=.2):
                raise RuntimeError("Transport still executing; connection retained")
            self.transport_owner = None
            self.odrive_controller = None
            self.odrive_adapter = None
            self.update_terminal(f"Connection timed out: {exc}\n")
            self.set_status("DISCONNECTED", RED)
        except Exception as exc:
            if self.odrive_adapter is not None:
                self._request_coordinated_stop("connection/configuration error", fault=True)
            else:
                self.odrive_controller = None
            self.update_terminal(f"Error connecting to ODrive: {exc}\n")
            self.set_status("ERROR / IDLE UNCONFIRMED" if self.odrive_adapter is not None else "ERROR", RED)
        finally:
            # Sensor acquisition is independent of ODrive zero/reconnect recovery.
            self.connect_rotary_sensor()
            try:
                self._ensure_load_cell()
                self.update_terminal("Load-cell interface connected.\n")
            except Exception as exc:
                self.update_terminal(f"Load-cell connection failed: {exc}\n")
            self.update_reference_display()

    def disconnect_odrive(self):
        """Request idle promptly, then detach only after background confirmation."""
        if self.motion_coordinator.shutting_down:
            return
        if self.motion_coordinator.owner is not None:
            self.stop_logging()
            self.update_terminal(
                "Disconnect requested during motion. Wait for the owning worker to exit, "
                "then disconnect again.\n"
            )
            return
        if self.odrive_adapter is None or (self.disconnect_worker is not None and self.disconnect_worker.is_alive()):
            return
        if not self.motion_coordinator.motor_idle_confirmed and (
            self.stop_worker is None or not self.stop_worker.is_alive()
        ):
            self.update_terminal("Disconnect blocked: idle is unconfirmed. Use the physical E-stop if needed.\n")
            return
        self._request_coordinated_stop("manual disconnect")
        self.disconnect_worker = threading.Thread(target=self._disconnect_after_idle_worker,
                                                  name="east-disconnect", daemon=True)
        self.disconnect_worker.start()

    def _disconnect_after_idle_worker(self):
        try:
            stop = self.stop_worker
            if stop is not None:
                stop.join(timeout=3.0)
                if stop.is_alive():
                    raise RuntimeError("Stop transport still executing; connection retained")
            if not self.motion_coordinator.motor_idle_confirmed:
                raise RuntimeError("ODrive idle was not confirmed; connection retained")
            self._stop_watchdog(disable=True)
            self._stop_feedback_monitor()
            if self.monitor_thread is not None or self.checkpoint_writer is not None:
                raise RuntimeError("Feedback/checkpoint worker did not exit; connection retained")
            snapshot = self._read_adapter_feedback()
            continuity_ready = self.reference_manager.verified and not self.monitor_error
            continuity_saved = persist_idle_continuity_checkpoint(
                self.reference_manager, snapshot, continuity_ready, "manual disconnect",
                expected_idle_state=int(AXIS_STATE_IDLE),
            )
            if self.transport_owner is not None and not self.transport_owner.close(timeout_s=.5):
                raise RuntimeError("Transport owner did not exit; connection retained")
            self.transport_owner = None
            self.odrive_controller = None
            self.odrive_adapter = None
            self.engine.transition(Lifecycle.DISCONNECTED)
            self.hardware_fingerprint = None
            with self.feedback_lock:
                self.latest_feedback = None
            self.reference_manager.mapping = None
            self.reference_manager.confidence = ReferenceConfidence.RECOVERY_REQUIRED
            self.reference_manager.reason = "ODrive disconnected; continuity must be proven on reconnect"
            self.ui_message_queue.put(("disconnected", continuity_saved))
        except Exception as exc:
            self.ui_message_queue.put(("terminal", f"Disconnect incomplete: {exc}\n"))
            self.ui_message_queue.put(("status", "DISCONNECT INCOMPLETE", RED))

    def stop_logging(self):
        if self.rotary_calibration is not None and not self.rotary_calibration.closed:
            self.rotary_calibration.cancel_capture("Operator Stop/Escape")
        if self.shutdown_requested:
            self.test_stop_event.set()
            self.neutral_stop_event.set()
            self.continuous_stop_event.set()
            self.update_terminal("Shutdown already has Stop latched; waiting for cleanup or power-isolated exit.\n")
            return
        was_active = self.strain_test_active
        if was_active:
            self.operator_stop_requested = True
        self._request_coordinated_stop("operator stop")
        if was_active:
            self.update_terminal("Test stop requested; the data file will be finalized as aborted.\n")
            


    def reset_test_fields(self):
        """Clear run-specific inputs without changing session safety state."""
        if self.motion_mapping_active() is True or self.strain_test_active or self.motion_coordinator.owner is not None:
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
        if threading.current_thread() is not threading.main_thread():
            self.ui_message_queue.put(("clear_terminal",))
            return
        self.terminal.delete(1.0, ctk.END)
        self.terminal.update()


    def update_terminal(self, message):
        if threading.current_thread() is not threading.main_thread():
            self.ui_message_queue.put(("terminal", message))
            return
        self.terminal.insert(ctk.END, message)
        self.terminal.see(ctk.END)  # Scroll to the end of the text
        mapping = getattr(self, "motion_mapping", None)
        if mapping is not None and getattr(mapping, "closed", True) is False:
            mapping.append_terminal(message)


    def _drain_ui_queues(self):
        drain_started = time.monotonic()
        self.ui_timing["maximum_gap_s"] = max(self.ui_timing["maximum_gap_s"], drain_started-self.last_ui_drain)
        self.last_ui_drain = drain_started
        self._retire_stop_worker()
        try:
            for _ in range(100):
                item = self.ui_message_queue.get_nowait()
                if item[0] == "calibration_prepared":
                    item[1]._capture_ready(item[2], item[3])
                elif item[0] == "calibration_saved":
                    item[1]._capture_saved(item[2])
                elif item[0] == "calibration_fit":
                    item[1].persistence_busy = False
                    item[1].result.configure(text=item[2])
                    item[1]._controls()
                elif item[0] == "engine_state":
                    self._refresh_motion_controls()
                elif item[0] == "engine_done":
                    if item[3]:
                        self.update_terminal(f"{item[1]} failed: {item[3]}\n")
                    if not self.strain_test_active and self.engine.busy is None:
                        self.set_test_inputs_state("normal")
                    self.update_reference_display()
                elif item[0] == "tare_display":
                    self.update_tare_display()
                elif item[0] == "close_recovery":
                    self.close_reference_recovery()
                elif item[0] == "clear_terminal":
                    self.clear_terminal()
                elif item[0] == "terminal":
                    self.update_terminal(item[1])
                elif item[0] == "status":
                    self.set_status(item[1], item[2])
                elif item[0] == "inputs":
                    self.set_test_inputs_state(item[1])
                elif item[0] == "run_buttons":
                    self._refresh_motion_controls()
                elif item[0] == "offer_isolated_exit":
                    self._offer_isolated_exit(item[1])
                elif item[0] == "neutral_finished":
                    self.neutral_motion_active = False
                    self._refresh_motion_controls()
                elif item[0] == "reference":
                    self.update_reference_display()
                elif item[0] == "shutdown_ready":
                    if self.isolated_exit_requested:
                        continue
                    self.close_plot_window()
                    self.master.quit()
                    self.master.destroy()
                    return
                elif item[0] == "disconnected":
                    self.buttons[1].configure(state="disabled")
                    self.neutral_button.configure(state="disabled")
                    self.set_status("DISCONNECTED", RED)
                    self.update_terminal("ODrive disconnected after confirmed idle. "
                                         f"Clean continuity checkpoint: {'saved' if item[1] else 'not saved'}.\n")
                    self.update_reference_display()
        except queue.Empty:
            pass

        try:
            plot_changed = False
            for _ in range(200):
                angle, torque = self.plot_data_queue.get_nowait()
                self.update_plot_data(angle, torque, redraw=False)
                plot_changed = True
        except queue.Empty:
            pass
        if plot_changed:
            self.update_plot()

        self._poll_rotary_homing()
        self._poll_rotary_sensor()
        self._poll_connection_zero()
        self._diagnose_stalled_workers()
        self.update_manual_position_display()
        self.ui_timing["drains"] += 1
        self.ui_timing["maximum_drain_s"] = max(self.ui_timing["maximum_drain_s"], time.monotonic()-drain_started)
        try:
            self.master.after(50, self._drain_ui_queues)
        except tk.TclError:
            pass


    def rotary_homing_active(self):
        return getattr(getattr(self, 'rotary_homing', None), 'active', False) is True

    def rotary_reference_verified(self):
        if not self.reference_manager.verified:
            return False
        sensor = self.rotary_sensor
        reader, calibration = sensor.reader, sensor.calibration
        if reader is None or calibration is None:
            return False
        method = f'rotary_sensor_homing:{calibration.sha256}:{reader.connection_id}'
        mapping = self.reference_manager.require_verified()
        return (mapping.verification_method == method
                and self.reference_manager.record.fixture_id == (self.fixture_id_input.get().strip()
                    if threading.current_thread() is threading.main_thread() else self.operator_context['fixture']))

    def rotary_zero_ready(self, rotary_only=False):
        value = self.rotary_sensor.measurement(require=True)
        if not self.rotary_reference_verified():
            return False
        self.get_feedback()  # Motor health/freshness remains required; its derived angle is not physical zero.
        return abs(value['angle_deg']) <= ZERO_TOLERANCE_DEG

    def _poll_connection_zero(self):
        """One stationary-only verification per successful connection; never auto-home."""
        if not getattr(self, '_connection_zero_pending', False):
            return
        from east_motion_mapping import ready_for_next
        if (self.rotary_homing_active() or self.motion_mapping_active() or self.manual_mode.get()
                or self.startup_block_reason or self.monitor_error or self.feedback_retry_pending
                or not ready_for_next(self)):
            return
        value=self.rotary_sensor.measurement()
        if not value['valid'] or not self.operator_input.get().strip() or not self.fixture_id_input.get().strip():
            return
        if self.rotary_calibration is not None and not self.rotary_calibration.closed:
            return
        self._connection_zero_pending=False
        if abs(value['angle_deg']) > ZERO_TOLERANCE_DEG:
            self.update_terminal('Automatic zero check: outside ±0.10 degrees; use Check Zero / Home. No movement commanded.\n')
            return
        self.update_terminal('Automatically verifying stationary rotary zero after connection. No movement commanded.\n')
        self.open_rotary_homing(verification_only=True)

    def open_rotary_homing(self, resume_test=False, verification_only=False):
        """One explicit action: refresh unhealthy feedback, home, persist zero."""
        from types import SimpleNamespace
        from east_motion_mapping import workers_finished
        from east_rotary_homing import HomingPolicy
        if self.rotary_homing_active():
            return
        try:
            if (self.shutdown_requested or self.startup_block_reason or self.engine.busy is not None
                    or self.strain_test_active or self.motion_mapping_active()
                    or self.motion_coordinator.owner is not None or not workers_finished(self)):
                raise ValueError('Finish the current operation/cleanup before homing')
            if self.odrive_adapter is None:
                raise ValueError('Connect ODrive first')
            if self.rotary_calibration is not None and not self.rotary_calibration.closed:
                raise ValueError('Close Rotary Sensor Calibration; use the live angle in this panel')
            value = self.rotary_sensor.measurement()
            in_band = bool(value['valid'] and abs(value['angle_deg']) <= ZERO_TOLERANCE_DEG)
            centre_requested = (not verification_only and not resume_test
                                and self.homing_acknowledged.get())
            verify_only = in_band and not centre_requested
            if verification_only and not in_band:
                raise ValueError('Automatic stationary zero check requires a fresh reading within ±0.10 degrees; no movement commanded')
            if not verify_only and not self.homing_acknowledged.get():
                raise ValueError('Outside zero band: confirm the clear fixture/AFO and unchanged sensor mounting, then press Check Zero / Home')
            if self.manual_mode.get():
                self.manual_mode.set(False)
                self.update_terminal('Manual Mode exited for automatic rotary homing.\n')
            value = self.rotary_sensor.measurement()
            parameters = self.collect_test_parameters() if resume_test else None
            condition = ('Empty fixture' if parameters.test_type == 'empty_machine_baseline' else 'AFO fitted') if parameters else self.homing_condition.get()
            if condition not in ('Empty fixture','AFO fitted'):
                raise ValueError('Select the fixture condition')
            state = SimpleNamespace(closed=False, active=True, phase='homing', future=None,
                                    fixture_condition=condition, resume_parameters=parameters, verify_only=verify_only,
                                    centre_requested=centre_requested)
            self.rotary_homing = state
            self.homing_acknowledged.set(False)
            self.operator_context = {'operator':self.operator_input.get().strip(),
                'fixture':self.fixture_id_input.get().strip(),'calibration':self.calibration_id_input.get().strip()}
            needs_recovery = bool(not value['valid'] or self.monitor_error or self.feedback_retry_pending
                or self.monitor_thread is None or not self.monitor_thread.is_alive()
                or not self.motion_coordinator.motor_idle_confirmed)
            try:
                self.get_feedback()
            except ReferenceError:
                needs_recovery = True
            if needs_recovery:
                state.phase = 'recovering'
                self.update_terminal('Refreshing ODrive feedback before rotary homing. No interrupted test will resume.\n')
                state.future = self.engine.submit('recover feedback for rotary homing',self._connect_system_worker)
            else:
                state.future = self.start_rotary_homing(state,HomingPolicy())
            self.set_test_inputs_state('disabled')
            self._refresh_motion_controls()
        except Exception as exc:
            if getattr(self, 'rotary_homing', None) is not None:
                self.rotary_homing.active = False
            self.update_terminal(f'Homing blocked: {exc}\n')
            self.update_reference_display()

    def _poll_rotary_homing(self):
        from east_rotary_homing import HomingPolicy
        state = getattr(self, 'rotary_homing', None)
        if state is None or getattr(state,'active',False) is not True or getattr(state,'managed_by_batch',False):
            return
        future = state.future
        if future is None or not future.done():
            return
        try:
            result = future.result()
            if self.shutdown_requested:
                raise RuntimeError('Shutdown cancelled homing preparation')
            if state.phase in ('recovering','waiting_sensor'):
                if self.engine.cancel_event.is_set():
                    raise RuntimeError('Stop cancelled feedback recovery')
                if state.phase == 'recovering':
                    state.phase = 'waiting_sensor'
                    state.sensor_deadline = time.monotonic()+2.0
                sensor = self.rotary_sensor.measurement()
                if not sensor['valid']:
                    if time.monotonic() < state.sensor_deadline:
                        return
                    raise RotaryError(f'Fresh rotary feedback unavailable: {sensor["reason"]}')
                state.phase = 'homing'
                state.future = self.start_rotary_homing(state,HomingPolicy())
                return
            if result['evidence']['outcome']=='sensor_zero_reached':
                from east_motion_mapping import ready_for_next
                if not ready_for_next(self):return
            state.active = False
            if self.engine.cancel_event.is_set() and result['evidence']['outcome'] == 'sensor_zero_reached':
                raise RuntimeError('Stop cancelled test preparation')
            if result['evidence']['outcome'] == 'sensor_zero_reached' and state.resume_parameters is not None:
                if self.collect_test_parameters() != state.resume_parameters:
                    raise RuntimeError('Test settings changed; start a new test explicitly')
                self._starting_after_homing = True
                try:
                    self.start_strain_test()
                finally:
                    self._starting_after_homing = False
        except Exception as exc:
            state.active = False
            self.update_terminal(f'Rotary homing stopped: {exc}\n')
        if not self.strain_test_active and self.engine.busy is None:
            self.set_test_inputs_state('normal')
        self.update_reference_display()


    def start_comparison_zero_preparation(self,owner):
        from types import SimpleNamespace
        from east_rotary_homing import HomingPolicy
        from east_motion_mapping import ready_for_next
        if (self.motion_mapping is not owner or not owner.active or not owner.comparison_suite or owner.pilot_suite
                or not owner.pending_zero_preparation or owner.cancelled.is_set() or not ready_for_next(self)):
            raise RuntimeError('Comparison preparation ownership/cleanup changed')
        request=owner.requests[len(owner.records)]
        if request.parameters.test_type!='empty_machine_baseline':raise RuntimeError('Automatic comparison preparation requires empty fixture')
        value=self.rotary_sensor.measurement(require=True)
        state=SimpleNamespace(closed=False,active=True,phase='homing',future=None,fixture_condition='Empty fixture',
            resume_parameters=None,managed_by_batch=True,parent_batch_id=request.batch_id,parent_run_id=request.run_id,
            purpose='pre_run_zero_preparation',verify_only=abs(value['angle_deg'])<=ZERO_TOLERANCE_DEG)
        self.rotary_homing=state
        try:state.future=self.start_rotary_homing(state,HomingPolicy(),batch_owner=owner)
        except Exception:
            state.active=False
            raise
        return state.future

    def start_suite_zero_recovery(self, owner):
        from types import SimpleNamespace
        from east_rotary_homing import HomingPolicy
        from east_motion_mapping import zero_recovery_candidate
        if self.motion_mapping is not owner or not zero_recovery_candidate(self,owner.request):
            raise RuntimeError('Suite zero recovery is not eligible; preserve the failure and stop')
        # The official homing transaction enforces fresh measurements, ownership,
        # calibrated sensor range, cumulative travel, progress and timeout.
        self.rotary_sensor.measurement(require=True)
        state=SimpleNamespace(closed=False,active=True,phase='homing',future=None,
            fixture_condition='Empty fixture',resume_parameters=None,managed_by_batch=True,
            parent_batch_id=owner.request.batch_id,parent_run_id=owner.request.run_id)
        self.rotary_homing=state
        try:
            state.future=self.start_rotary_homing(state,HomingPolicy(),batch_owner=owner)
        except Exception:
            state.active=False
            raise
        return state.future

    def start_rotary_homing(self, window, policy, batch_owner=None):
        """Admit bounded rotary homing with the declared fixture/AFO condition."""
        from east_motion_mapping import ready_for_next
        from east_calibration_motion import CalibrationPositioningReference
        from east_core import APP_DIR
        policy.validate()
        if window is not self.rotary_homing or window.closed:
            raise ValueError("Home action is no longer current")
        allowed_batch = (batch_owner is not None and self.motion_mapping is batch_owner
                         and batch_owner.active and batch_owner.preset_snapshot.get('test_type') == 'empty_machine_baseline'
                         and (batch_owner.pending_zero_recovery or getattr(batch_owner,'pending_zero_preparation',False)) and not batch_owner.cancelled.is_set()
                         and getattr(window,'managed_by_batch',False))
        if batch_owner is not None and not allowed_batch:
            raise ValueError('Invalid suite zero-recovery ownership')
        if not ready_for_next(self) or (self.motion_mapping_active() and not allowed_batch) or self.monitor_error or self.feedback_retry_pending:
            raise ValueError("Finish cleanup and recover healthy ODrive feedback first")
        if self.startup_block_reason or self.manual_mode.get():
            raise ValueError("Resolve startup and turn Manual Mode off")
        if self.rotary_calibration is not None and not self.rotary_calibration.closed:
            raise ValueError("Close Rotary Sensor Calibration before homing")
        if self.recovery_window is not None and self.recovery_window.winfo_exists():
            raise ValueError("Close Machine Zero Setup before automatic homing")
        value = self.rotary_sensor.measurement(require=True)
        reference = CalibrationPositioningReference.establish(self, value['angle_deg'], acknowledged=True,
            homing_recovery=True)
        reference.check(self, stationary=True)
        value = self.rotary_sensor.measurement(require=True)
        source = value['sample'][4]
        calibration = self.rotary_sensor.calibration.sha256
        base = Path(sys.executable).resolve().parent if getattr(sys, 'frozen', False) else APP_DIR
        path = base/self.system_config['logging']['output_directory']/(
            'rotary_home_'+datetime.now().strftime('%Y%m%dT%H%M%S_%f')+'.json')
        window.evidence_path=str(path)
        token = self.motion_coordinator.acquire('rotary-home')
        self.stop_started_monotonic = None
        self.stop_diagnostic_recorded = False
        self.operator_context = {'operator':self.operator_input.get().strip(),
                                 'fixture':self.fixture_id_input.get().strip(),
                                 'calibration':self.calibration_id_input.get().strip()}
        future = self.engine.submit('rotary homing', lambda:self._rotary_homing_worker(
            policy, reference, token, source, calibration, path))
        if future.done() and future.exception() is not None:
            self.motion_coordinator.release(token)
            raise future.exception()
        self.set_test_inputs_state('disabled')
        self._refresh_motion_controls()
        return future

    def _rotary_homing_worker(self, policy, reference, token, source, calibration, path):
        from east_rotary_homing import home
        import statistics
        raw_path = path.with_name(path.stem+'_rotary_samples.csv')
        body = {'schema':'east-rotary-home-2', 'official_zero_method':True,
                'started_at':datetime.now().astimezone().isoformat(),
                'zero_reference_written':False, 'policy':policy.__dict__,
                'fixture_condition':getattr(self.rotary_homing, 'fixture_condition', 'AFO fitted'),
                'operator':reference.operator_id, 'fixture_id':reference.fixture_id,
                'rotary_calibration_sha256':calibration, 'sensor_connection_id':source,
                'rotary_calibration':self.rotary_sensor.calibration.metadata(),
                'machine_zero_before':self.reference_manager.metadata_snapshot(),
                'control_settings':{'speed_deg_s':self.system_config['reference']['recovery_speed_deg_s'],
                    'acceleration_deg_s2':self.system_config['reference']['recovery_acceleration_deg_s2'],
                    'nominal_deg_per_turn':reference.factor},
                'raw_rotary_file':str(raw_path), 'outcome':'preparing',
                'corrections':[], 'observations':[], 'idle_confirmed':False}
        state=getattr(self,'rotary_homing',None)
        if getattr(state,'managed_by_batch',False):
            body['purpose']=getattr(state,'purpose','separate_suite_zero_recovery')
            body['parent_batch_id']=state.parent_batch_id
            body['parent_run_id']=state.parent_run_id
        recording_started = False
        result = None
        failure = None
        deadline = time.monotonic()+policy.timeout_s
        armed = False
        verify_only = getattr(self.rotary_homing, 'verify_only', False) is True
        body['verification_only_no_motion'] = verify_only
        centre_requested = getattr(state, 'centre_requested', False)
        centre_target_deg = .01 if centre_requested else .04
        body['centring_target_deg'] = 0.0
        body['centring_mean_limit_deg'] = centre_target_deg
        observing_idle = False

        def check():
            if observing_idle:
                if self.motion_coordinator.active_token != token or not self.motion_coordinator.motor_idle_confirmed:
                    raise FeedbackError('Idle homing ownership/confirmation changed')
            else:
                self.motion_coordinator.assert_active(token)
            if self.engine.cancel_event.is_set() or time.monotonic() > deadline:
                raise RuntimeError('Homing stopped or timed out')
            if self.monitor_error or self.feedback_retry_pending:
                raise FeedbackError('Healthy ODrive feedback is required for homing')
            snapshot = reference.check(self)
            expected_state = int(AXIS_STATE_CLOSED_LOOP_CONTROL) if armed else int(AXIS_STATE_IDLE)
            if snapshot.current_state != expected_state or snapshot.active_errors:
                raise FeedbackError('ODrive state/errors changed during homing')
            if armed and snapshot.disarm_reason != self.expected_disarm_reason:
                raise FeedbackError('ODrive disarm reason changed during homing')
            value = self.rotary_sensor.measurement(require=True)
            if value['sample'][4] != source or self.rotary_sensor.calibration.sha256 != calibration:
                raise RotaryError('Rotary connection or calibration changed during homing')
            self._mark_control_health()
            return snapshot, value

        def observe(duration, *, require_band=False):
            from east_zero_window import collect_stationary_window, ZERO_IDLE_SETTLE_S, ZERO_IDLE_TIMEOUT_S
            def read():
                snapshot,value=check();sample=value['sample']
                body['observations'].append({'host_monotonic_s':time.monotonic(),
                    'rotary_sequence':sample[3],'rotary_sample_host_s':sample[0],
                    'angle_deg':value['angle_deg'],'motor_turns':snapshot.position_turns,
                    'motor_velocity_turns_s':snapshot.velocity_turns_s,
                    'stationary':abs(snapshot.velocity_turns_s*reference.factor)<=self.system_config['reference']['settle_velocity_limit_deg_s'],
                    'motor_feedback_host_s':snapshot.captured_monotonic_s})
                return dict(stationary=abs(snapshot.velocity_turns_s*reference.factor)<=self.system_config['reference']['settle_velocity_limit_deg_s'],
                    angle_deg=value['angle_deg'],sequence=sample[3],motor_turns=snapshot.position_turns)
            def within_band(observation):
                return (observation['min_deg'] >= -policy.tolerance_deg
                        and observation['max_deg'] <= policy.tolerance_deg)

            budget = deadline - time.monotonic()
            settle_s = ZERO_IDLE_SETTLE_S if require_band else 0.0
            if require_band:
                budget = min(budget, ZERO_IDLE_TIMEOUT_S)
            result = collect_stationary_window(
                read, duration, max(duration + settle_s, budget),
                settle_s=settle_s,
                position_tolerance_turns=afo_degrees_to_odrive_turns(
                    self.system_config['motion']['position_tolerance_deg'], self.system_config),
                accept_window=within_band if require_band else None,
                sleep=lambda t:self.engine.cancel_event.wait(t),
            )
            body['stationary_observation_resets']=body.get('stationary_observation_resets',0)+result['stationary_resets']
            return result

        def move(delta):
            nonlocal armed
            if verify_only:
                raise RotaryError("Zero verification cannot command movement; use Check Zero / Home after confirming clearance")
            snapshot, value = check()
            step = delta/reference.factor
            target = reference.target(snapshot.position_turns, 1 if step > 0 else -1, abs(step))
            if not armed:
                self.enter_closed_loop(token, allow_unreferenced=True)
                armed = True
                check()
            event = {'before_deg':value['angle_deg'], 'nominal_step_deg':delta,
                     'target_motor_turns':target, 'requested_at_host_s':time.monotonic()}
            body['corrections'].append(event)
            self._write_runtime_checkpoint(MotionState.MOVING, self.get_feedback(), clean_shutdown=False,
                extra={'phase':'rotary_home', 'target_turns':target, 'zero_reference_written':False})
            write_json_atomic(path,{**body, 'outcome':'running'})
            self._submit_position(token, target)
            event['submitted_at_host_s'] = time.monotonic()
            self.update_terminal(f"Rotary homing: sensor {value['angle_deg']:+.3f} deg; correction {delta:+.3f} nominal deg.\n")
            cfg = self.system_config['reference']
            wait_for_settle(lambda:check()[0], target,
                min(afo_degrees_to_odrive_turns(self.system_config['motion']['position_tolerance_deg'],self.system_config),abs(step)*0.25),
                afo_speed_to_odrive_turns_s(cfg['settle_velocity_limit_deg_s'],self.system_config),
                cfg['settle_dwell_ms']/1000.0,
                min(max(0.1,deadline-time.monotonic()),motion_timeout_seconds(abs(delta),cfg['recovery_speed_deg_s'],cfg['recovery_acceleration_deg_s2'],self.system_config)),
                cfg['feedback_stale_after_ms']/1000.0,
                lambda:self.engine.cancel_event.is_set() or self._motion_cancelled(token),
                **self._powered_wait_requirements())
            event['settled_motor_turns'] = check()[0].position_turns

        try:
            write_json_atomic(path, body)
            self._validate_live_motion_config()
            self._revalidate_hardware_identity(require_reference=False)
            check()
            if not verify_only:
                self.configure_trajectory(self.system_config['reference']['recovery_speed_deg_s'],
                                          self.system_config['reference']['recovery_acceleration_deg_s2'])
            self.rotary_sensor.start_recording(raw_path, time.monotonic())
            recording_started = True
            self.update_terminal(f'Rotary homing started. Evidence: {path}\n')
            if verify_only:
                observation = observe(policy.dwell_s, require_band=True)
                if observation['min_deg'] < -policy.tolerance_deg or observation['max_deg'] > policy.tolerance_deg:
                    raise RotaryError('Zero check crossed ±0.10 degrees; zeroing is required')
                result = dict(nominal_travel_deg=0.0)
            else:
                result = home(policy, observe, move, check, centre_mean_deg=centre_target_deg)
            idle = self.safe_idle_motor('rotary homing finished',token=token)
            armed = False
            if not idle or not idle.confirmed:
                raise FeedbackError('Homing idle was not confirmed')
            body['idle_confirmed'] = True
            observing_idle = True
            after_idle = observe(2.0 if centre_requested else policy.dwell_s, require_band=True)
            body['post_idle_observation'] = after_idle
            if centre_requested:
                precision = {
                    "tolerance_deg": .01,
                    "duration_s": after_idle["duration_s"],
                    "mean_deg": after_idle["mean_deg"],
                    "min_deg": after_idle["min_deg"],
                    "max_deg": after_idle["max_deg"],
                    "mean_within_band": abs(after_idle["mean_deg"]) <= .01,
                    "all_observed_samples_within_band": (
                        after_idle["min_deg"] >= -.01 and after_idle["max_deg"] <= .01),
                }
                body["fine_zero_check"] = precision
                self.update_terminal(
                    f"Fine zero check ({precision['duration_s']:.2f} s): "
                    f"mean {precision['mean_deg']:+.4f}°, "
                    f"range [{precision['min_deg']:+.4f}°, {precision['max_deg']:+.4f}°]. "
                    f"All observed readings within ±0.01°: "
                    f"{'yes' if precision['all_observed_samples_within_band'] else 'no'}.\n"
                )
            if after_idle['min_deg'] < -policy.tolerance_deg or after_idle['max_deg'] > policy.tolerance_deg:
                raise RotaryError(
                    "Fixture left the stopping band after motor idle and did not settle "
                    f"(last stationary range {after_idle['min_deg']:+.4f}° to {after_idle['max_deg']:+.4f}°)"
                )
            body.update(outcome='sensor_zero_reached', final_observation=after_idle,
                        candidate_zero_motor_turns=check()[0].position_turns,
                        nominal_travel_deg=result['nominal_travel_deg'])
        except Exception as exc:
            failure = str(exc)
            body.update(outcome='failed',error=failure)
            cancelled = self.engine.cancel_event.is_set() and not self.monitor_error
            if cancelled:
                body['outcome'] = 'cancelled'
            self._request_coordinated_stop(f'Rotary homing: {exc}',fault=not cancelled)
            stop = self.stop_worker
            if stop is not None:
                stop.join(timeout=self.system_config['reference']['idle_confirmation_timeout_s']+0.25)
                self._retire_stop_worker()
            body['idle_confirmed'] = bool(self.motion_coordinator.motor_idle_confirmed)
        finally:
            if recording_started:
                try:
                    body['rotary_recording_statistics'] = self.rotary_sensor.finish_recording()
                except Exception as exc:
                    failure = f'{failure or ""}; rotary recording finalization failed: {exc}'
                    body.update(outcome='failed',error=failure)
            if body['outcome'] == 'sensor_zero_reached':
                try:
                    snapshot, latest = check()
                    reference.stationary(self,snapshot)
                    if abs(latest['angle_deg']) > policy.tolerance_deg:
                        raise RotaryError('Sensor left zero before reference commit')
                    mapping = self.reference_manager.establish_from_rotary_zero(
                        snapshot, self.hardware_fingerprint, reference.operator_id,
                        reference.fixture_id, calibration, source, body['final_observation'])
                    body['zero_reference_written'] = True
                    body['reference_id'] = mapping.reference_id
                    self.unsafe_exit_recovered_in_session = True
                    self.engine.record('rotary_zero', {'verified':True,'reference_id':mapping.reference_id,
                        'calibration_sha256':calibration,'final_angle_deg':latest['angle_deg']})
                except Exception as exc:
                    body.update(outcome='failed',error=f'Rotary zero commit failed: {exc}')
                    self.reference_manager.invalidate_local(str(exc),fault=True)
            if body['outcome'] == 'failed':
                self.reference_manager.invalidate_local(body.get('error','Homing incomplete'),fault=True)
            # Retain ownership if Stop/recording cleanup is unresolved.
            stop = self.stop_worker
            recorder = self.rotary_sensor.recording
            recorder_alive = recorder is not None and recorder.thread.is_alive()
            if (self.motion_coordinator.motor_idle_confirmed and not recorder_alive
                    and (stop is None or not stop.is_alive())):
                self.motion_coordinator.release(token)
            body['motion_owner_after_cleanup'] = self.motion_coordinator.owner
            if self.motion_coordinator.owner is not None:
                body.update(outcome='failed',error=body.get('error','Cleanup remains pending'))
            body['completed_at'] = datetime.now().astimezone().isoformat()
            body['machine_zero_after'] = self.reference_manager.metadata_snapshot()
            try:
                write_json_atomic(path,body)
            except Exception as exc:
                self.reference_manager.invalidate_local(f'Homing evidence could not be saved: {exc}',fault=True)
                body.update(outcome='failed',error=str(exc))
                raise
            finally:
                self._schedule_rotary_homing_retirement(token, path, body)
            self.ui_message_queue.put(('reference',))
            self.ui_message_queue.put(('run_buttons',))
        message = (f"Rotary machine zero verified within ±{policy.tolerance_deg:g}° and saved. Evidence: "+str(path)
                   if body['outcome']=='sensor_zero_reached' else f"Homing failed: {body.get('error')}. Evidence: {path}")
        self.update_terminal(message+'\n')
        return {'message':message, 'evidence':body}

    def _schedule_rotary_homing_retirement(self, token, path, body):
        if self.motion_coordinator.active_token == token:
            # Reuse the app's tracked cleanup slot. Ownership remains held until
            # outstanding Stop/recording workers retire and idle is confirmed.
            self.cleanup_thread = threading.Thread(target=self._retire_rotary_homing,
                args=(token, self.rotary_sensor.recording, path, dict(body)),
                name='east-rotary-home-cleanup', daemon=True)
            self.cleanup_thread.start()

    def _retire_rotary_homing(self, token, recorder, path, body):
        while self.motion_coordinator.active_token == token:
            stop = self.stop_worker
            if (self.motion_coordinator.motor_idle_confirmed
                    and (stop is None or not stop.is_alive())
                    and (recorder is None or not recorder.thread.is_alive())):
                self._retire_stop_worker()
                self.motion_coordinator.release(token)
                body['motion_owner_after_cleanup'] = self.motion_coordinator.owner
                body['idle_confirmed'] = True
                body['cleanup_completed_at'] = datetime.now().astimezone().isoformat()
                try:
                    write_json_atomic(path,body)
                except Exception as exc:
                    self.update_terminal(f'Homing cleanup metadata failed: {exc}\n')
                self.ui_message_queue.put(('run_buttons',))
                return
            if self.motion_coordinator.shutting_down:
                return  # Existing shutdown owns the remaining isolation/recovery path.
            time.sleep(0.05)

    def open_three_speed_protocol(self):
        if self.motion_coordinator.shutting_down or self.startup_block_reason:
            self.update_terminal('Three-speed protocol blocked during shutdown/unresolved startup.\n')
            return False
        from east_protocol_batch_gui import ThreeSpeedProtocolWindow
        current = self.motion_mapping
        if current is not None and not current.closed:
            if current.active:
                current.window.lift()
                return False
            if isinstance(current,ThreeSpeedProtocolWindow) and current.preset_key == self.loaded_preset_key:
                current.window.lift()
                return False
            current.close()
        self.motion_mapping = ThreeSpeedProtocolWindow(self.master,self,self.loaded_preset_key)
        return False

    def open_motion_mapping(self):
        if self.motion_coordinator.shutting_down or self.startup_block_reason:
            self.update_terminal("Motion modelling blocked during shutdown or unresolved startup.\n")
            return
        from east_motion_mapping_gui import MotionMappingWindow
        if self.motion_mapping is not None and not self.motion_mapping.closed:
            if self.motion_mapping.active or isinstance(self.motion_mapping,MotionMappingWindow):
                self.motion_mapping.window.lift()
                return
            self.motion_mapping.close()
        self.motion_mapping = MotionMappingWindow(self.master, self)

    def open_rotary_calibration(self):
        if self.rotary_homing_active():
            self.update_terminal("Wait for rotary homing to finish before calibration.\n")
            return
        if self.motion_mapping_active():
            self.update_terminal("Finish or stop motion modelling before opening calibration.\n")
            return
        if self.motion_coordinator.shutting_down:
            self.update_terminal("Calibration blocked during shutdown.\n")
            return
        if self.startup_block_reason:
            self.update_terminal(f"Sensor calibration blocked: {self.startup_block_reason}\n")
            return
        if self.rotary_calibration is not None and not self.rotary_calibration.closed:
            self.rotary_calibration.window.lift()
            return
        from east_rotary_gui import RotaryCalibrationWindow
        self.rotary_calibration = RotaryCalibrationWindow(self.master, self)

    def open_reference_recovery(self):
        """Compatibility entry: all zero recovery is now rotary-based."""
        return self.open_rotary_homing()


    def close_reference_recovery(self):
        self.recovery_window = None


    def set_machine_zero(self):
        return self.open_rotary_homing()


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
            if self.shutdown_requested:
                self.update_terminal("Shutdown is already in progress. If it remains blocked, use the power-isolated exit only after isolating motor power.\n")
                return
            self.shutdown_requested = True
            batch = getattr(self, 'motion_mapping', None)
            if batch is not None and getattr(batch, 'active', False) is True:
                batch.cancelled.set()
            self.engine.begin_close()
            self.shutdown_started_monotonic = time.monotonic()
            self.motion_coordinator.begin_shutdown()
            self.buttons[0].configure(state="disabled")
            self.set_test_inputs_state("disabled")
            self._refresh_motion_controls()
            if self.strain_test_active:
                self.operator_stop_requested = True
            if self.odrive_adapter is not None:
                self._request_coordinated_stop("application shutdown")
            else:
                self.test_stop_event.set()
            self._poll_shutdown_ready()

    def _poll_shutdown_ready(self):
        if self.isolated_exit_requested:
            return
        self._retire_stop_worker()
        worker_names = (
            "stop_worker", "strain_thread", "data_collection_thread", "neutral_thread",
            "manual_step_thread", "continuous_thread", "recovery_thread",
            "failure_metadata_worker", "cleanup_thread", "disconnect_worker", "watchdog_thread",
        )
        live = [name for name in worker_names if
                (worker := getattr(self, name, None)) is not None and worker.is_alive()]
        if self.engine.busy is not None:
            live.append("test engine: " + self.engine.busy)
        batch = getattr(self, 'motion_mapping', None)
        if batch is not None and getattr(batch, 'active', False) is True:
            live.append('batch outcome/manifest')
        recorder = getattr(self, "csv_recorder", None)
        if recorder is not None and recorder.thread.is_alive():
            live.append("CSV recorder")
        if live or any(worker.is_alive() for worker in self.metadata_workers) or self.motion_coordinator.owner is not None:
            self.set_status("SHUTDOWN WAITING / IDLE OBSERVED" if self.motion_coordinator.motor_idle_confirmed else "SHUTDOWN WAITING / IDLE UNCONFIRMED", RED)
            if time.monotonic() - self.shutdown_started_monotonic >= 3.0:
                self._offer_isolated_exit("Workers or motion ownership have not cleared")
            self.master.after(200, self._poll_shutdown_ready)
            return
        if self.odrive_adapter is not None and not self.motion_coordinator.motor_idle_confirmed:
            self.set_status("SHUTDOWN BLOCKED / IDLE UNCONFIRMED", RED)
            self._offer_isolated_exit("Idle not confirmed")
            return
        self.shutdown_worker = threading.Thread(target=self._shutdown_cleanup_worker,
                                                name="east-shutdown-cleanup", daemon=True)
        self.shutdown_worker.start()
        self.master.after(200, self._poll_shutdown_cleanup)

    def _poll_shutdown_cleanup(self):
        if self.isolated_exit_requested:
            return
        if self.shutdown_worker is not None and self.shutdown_worker.is_alive():
            if time.monotonic() - self.shutdown_started_monotonic >= 3.0:
                self._offer_isolated_exit("Shutdown cleanup is still blocked")
            self.master.after(200, self._poll_shutdown_cleanup)

    def _offer_isolated_exit(self, reason):
        if self.isolated_exit_offered:
            if self.shutdown_dialog is not None:
                self.shutdown_dialog.show()
            return
        self.isolated_exit_offered = True
        idle_observed = self.motion_coordinator.motor_idle_confirmed
        self.set_status("SHUTDOWN BLOCKED / CLEANUP" if idle_observed else "SHUTDOWN BLOCKED / IDLE UNCONFIRMED", RED)
        if idle_observed and self.last_idle_observation is not None:
            reason += f". Controller idle was observed at {self.last_idle_observation.get('observed_at')} but cleanup remains incomplete"
        self.update_terminal(
            f"{reason}. Engage the physical E-stop and isolate motor power before using "
            "Exit After Motor Power Is Isolated. Normal controls remain locked.\n"
        )
        self.isolated_exit_button.grid()
        self.shutdown_dialog = BlockedShutdownDialog(ctk, self.master, reason,
            lambda: self.exit_after_motor_power_is_isolated(acknowledged=True))

    def exit_after_motor_power_is_isolated(self, acknowledged=False):
        if self.isolated_exit_requested or not self.shutdown_requested or not self.isolated_exit_offered:
            return
        confirmed = acknowledged or self.show_resizable_confirmation(
            title="Motor Power Isolation Required",
            message=(
                "FIRST engage the physical E-stop, switch off/isolate the motor power supply, "
                "and verify that motor power is isolated.\n\n"
                "This acknowledges your physical action; EAST cannot electronically verify idle or isolation. "
                "The run will be marked incomplete, machine-zero continuity will be invalid, "
                "and buffered data may be lost. A blocked thread may require abrupt process exit."
            ),
            confirm_text="I Confirm Isolation",
            dangerous=True,
        )
        if not confirmed:
            return
        self.isolated_exit_requested = True
        self.shutdown_requested = True
        self.motion_coordinator.begin_shutdown()
        self._request_coordinated_stop("power-isolated exit; idle unconfirmed", fault=True)
        self.set_status("POWER ISOLATED BY OPERATOR / EXITING", RED)
        self.isolated_exit_button.configure(state="disabled")
        threading.Thread(target=self._isolated_exit_worker,
                         name="east-power-isolated-exit", daemon=True).start()
        self._schedule_isolated_exit_deadline()

    def _schedule_isolated_exit_deadline(self):
        # Independent of Tk, which may itself be stalled. ProcessLock is
        # released by the OS when the acknowledged process exit occurs.
        timer = threading.Timer(4.0, self._force_process_exit)
        timer.daemon = True
        timer.start()

    def _isolated_exit_worker(self):
        reason = "Operator confirmed physical motor-power isolation; controller idle unconfirmed"
        try:
            self.state_store.mark_unsafe_exit()
            self.reference_manager.mapping = None
            self.reference_manager.confidence = ReferenceConfidence.FAULT
            self.reference_manager.reason = reason
            if self.metadata_file_name and self.run_metadata is None:
                self.run_metadata = {"run_status": "started", "error": None}
            if (self.run_metadata is not None and self.metadata_file_name
                    and (self.strain_test_active or not self.run_finalized
                         or self.run_metadata.get("outcome_pending_cleanup"))):
                self._persist_interim_outcome(reason, True)
            recorder = getattr(self, "csv_recorder", None)
            buffered_rows = recorder.unsaved_rows() if recorder is not None else getattr(self, "strain_data_buffer", [])
            if (self.acquisition_done_event.is_set() and self.strain_file_name
                    and buffered_rows):
                recovery_path = Path(self.strain_file_name).with_name(
                    Path(self.strain_file_name).stem + "_unsaved_buffer.csv"
                )
                with recovery_path.open(mode="x", newline="", encoding="utf-8") as handle:
                    writer = csv.writer(handle)
                    writer.writerow(CSV_COLUMNS + ROTARY_RUN_COLUMNS + MOTOR_TIMING_COLUMNS)
                    writer.writerows(buffered_rows)
                    handle.flush()
                    os.fsync(handle.fileno())
                self.ui_message_queue.put(("terminal", f"Uncertain final CSV buffer saved separately: {recovery_path}\n"))
            writer = self.checkpoint_writer
            if writer is not None:
                writer.submit(MotionState.FAULT, None, {"reason": reason})
        except Exception as exc:
            self.ui_message_queue.put(("terminal", f"Power-isolated exit persistence incomplete: {exc}\n"))
        finally:
            self._force_process_exit()

    def _force_process_exit(self):
        os._exit(1)

    def _shutdown_cleanup_worker(self):
        try:
            if self.isolated_exit_requested or self.motion_coordinator.owner is not None:
                raise RuntimeError("Motion ownership or isolated exit prevents clean shutdown")
            motion_workers = ("stop_worker", "strain_thread", "data_collection_thread",
                              "neutral_thread", "manual_step_thread", "continuous_thread",
                              "recovery_thread", "failure_metadata_worker", "cleanup_thread",
                              "disconnect_worker", "watchdog_thread")
            if any((worker := getattr(self, name, None)) is not None and worker.is_alive()
                   for name in motion_workers) or any(worker.is_alive() for worker in self.metadata_workers):
                raise RuntimeError("Motion/data worker survived shutdown admission")
            self._stop_watchdog(disable=True)
            self._stop_feedback_monitor()
            if self.monitor_thread is not None or self.checkpoint_writer is not None or self.watchdog_thread is not None:
                raise RuntimeError("Feedback/checkpoint/watchdog worker did not exit")
            if self.odrive_adapter is not None:
                snapshot = self._read_adapter_feedback()
                if (self.motion_coordinator.owner is not None or self.isolated_exit_requested
                        or any((worker := getattr(self, name, None)) is not None and worker.is_alive()
                               for name in motion_workers)
                        or any(worker.is_alive() for worker in self.metadata_workers)):
                    raise RuntimeError("Motion ownership changed during shutdown")
                continuity_ready = (self.reference_manager.verified and not self.monitor_error
                                    and self.motion_coordinator.motor_idle_confirmed
                                    and (not self.previous_unsafe_exit or self.unsafe_exit_recovered_in_session))
                continuity_saved = persist_idle_continuity_checkpoint(
                    self.reference_manager, snapshot, continuity_ready, "application shutdown",
                    expected_idle_state=int(AXIS_STATE_IDLE))
                if continuity_ready and not continuity_saved:
                    raise RuntimeError("Clean idle continuity was not persisted")
            if self.rotary_calibration is not None:
                self.rotary_calibration.shutdown()
            self.rotary_sensor.shutdown()
            if self.voltage_ratio_input is not None:
                self.voltage_ratio_input.close()
                self.voltage_ratio_input = None
            if (self.motion_coordinator.owner is not None or self.isolated_exit_requested
                    or any((worker := getattr(self, name, None)) is not None and worker.is_alive()
                           for name in motion_workers)
                    or any(worker.is_alive() for worker in self.metadata_workers)):
                raise RuntimeError("Motion worker survived cleanup")
            if self.transport_owner is not None and not self.transport_owner.close(timeout_s=.5):
                raise RuntimeError("Transport owner remains active")
            if not self.engine.close(timeout_s=.5):
                raise RuntimeError("Test engine remains active")
            if (not self.isolated_exit_offered
                    and (self.odrive_adapter is None and not self.previous_unsafe_exit
                         or self.odrive_adapter is not None and continuity_saved)):
                self.state_store.clear_unsafe_exit_after_recovery()
            self.odrive_controller = None
            self.odrive_adapter = None
            if self.isolated_exit_requested:
                self.state_store.mark_unsafe_exit()
            else:
                self.process_lock.release()
                self.ui_message_queue.put(("shutdown_ready",))
        except Exception as exc:
            self.ui_message_queue.put(("terminal", f"Shutdown incomplete: {exc}. Keep the E-stop accessible.\n"))
            self.ui_message_queue.put(("status", "SHUTDOWN INCOMPLETE", RED))
            self.ui_message_queue.put(("offer_isolated_exit", str(exc)))

    def start_strain_test(self, mapping_request=None):
        mapping = getattr(self, "motion_mapping", None)
        mapping_active = mapping is not None and getattr(mapping, "active", False) is True
        if mapping_request is not None:
            if not mapping_active or not mapping.accepts(mapping_request):
                self.update_terminal("Batch request is no longer authorized.\n")
                return False
        elif mapping_active:
            self.update_terminal("Use the active sequence window while its batch is running.\n")
            return False
        if self.motion_coordinator.shutting_down:
            self.update_terminal("Start blocked during shutdown.\n")
            return
        if mapping_request is None and not mapping_active:
            preset = self._loaded_preset()
            if isinstance(preset, dict) and preset.get('execution_kind') == 'speed_sequence':
                return self.open_three_speed_protocol()
        if self.engine.busy is not None:
            self.update_terminal("Start blocked while hardware lifecycle work is in progress.\n")
            return
        """Start the strain test with the current motor settings"""
        self._retire_stop_worker()
        if self.motion_coordinator.shutting_down:
            self.update_terminal("Start blocked during shutdown. Restart EAST after recovery.\n")
            return
        if any(worker.is_alive() for worker in self.metadata_workers):
            self.update_terminal("Start blocked while previous run metadata is still being saved.\n")
            return
        if not self.run_finalized or (self.run_metadata and self.run_metadata.get("outcome_pending_cleanup")):
            self.update_terminal("Cannot start: previous run outcome or cleanup is not finalized.\n")
            return
        if self.disconnect_worker is not None and self.disconnect_worker.is_alive():
            self.update_terminal("Cannot start while ODrive disconnect is in progress.\n")
            return
        if self.stop_worker is not None and (
            self.stop_worker.is_alive() or not self.motion_coordinator.motor_idle_confirmed
        ):
            self.update_terminal("Cannot start: prior stop has not completed with confirmed idle.\n")
            return
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

        try:
            zero_value = self.rotary_sensor.measurement(require=True)
            pilot_start = getattr(mapping_request, "stage", "") == "rotary_feedback_pilot"
            # The pilot worker verifies a full stationary window before any motion.
            ready = self.rotary_reference_verified() if pilot_start else self.rotary_zero_ready()
        except (ReferenceError, RotaryError) as exc:
            self.update_terminal(f"Test blocked: {exc}\n")
            return False
        if mapping_request is None and getattr(self, '_starting_after_homing', False) is not True:
            if abs(zero_value['angle_deg']) <= ZERO_TOLERANCE_DEG:
                self.update_terminal('Checking stationary rotary zero without movement before test.\n')
                self.open_rotary_homing(resume_test=True)
                return False
        if not ready:
            if mapping_request is not None or getattr(self, '_starting_after_homing', False) is True:
                self.update_terminal("Test blocked: rotary zero is not stable; use Home / Return to Zero.\n")
                return False
            self.update_terminal("Preparing rotary zero before starting a new test.\n")
            self.open_rotary_homing(resume_test=True)
            return False

        if self.manual_mode.get():
            self.update_terminal("Disable manual mode before starting a strain test.\n")
            return
        
        if self.strain_test_active:
            self.update_terminal("Strain test already active\n")
            return
        
        try:
            parameters = mapping_request.parameters if mapping_request is not None else self.collect_test_parameters()
            use_model = motion_model_enabled(self.system_config) and getattr(mapping_request, "metadata_key", None) != "motion_modelling" and not (mapping_request is not None and mapping_request.context().get("purpose") == "nonlinear_motion_identification")
            override = getattr(mapping_request,"command_model_enabled",None)
            if override is not None:
                use_model = override
            selected_before_confirmation = selected_method(self.system_config)
            variant = getattr(mapping_request, 'model_variant', selected_variant(self.system_config))
            if use_model:
                validate_model_range(parameters.min_angle_deg, parameters.max_angle_deg)
                if self.rotary_sensor.calibration.sha256 not in CALIBRATION_HASHES:
                    raise ValueError("Motion model requires its original rotary calibration; review/refit after calibration changes")
        except ValueError as exc:
            self.update_terminal(f"Test blocked: {exc}\n")
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
        active_protocol = (mapping_request.preset_metadata["preset_display_name"] if mapping_request is not None
                           else self.active_protocol_name())
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
            + ((WIDER_METHOD if variant == "candidate" else UPDATED_METHOD) + "\n" if use_model else "Legacy fixed conversion.\n") +
            "Fixture clear  |  Physical E-stop accessible"
        )
        if mapping_request is None and not self.show_resizable_confirmation(
            title=("Confirm Empty-Machine Baseline" if baseline_test else "Confirm Test"),
            message=confirmation_message,
            confirm_text=("Start Baseline" if baseline_test else "Start"),
        ):
            return

        if mapping_request is None and selected_method(self.system_config) != selected_before_confirmation:
            self.update_terminal("Motion method changed during confirmation; review and start again.\n")
            return False
        if self.stop_worker is not None:
            if self.stop_worker.is_alive() or not self.motion_coordinator.motor_idle_confirmed:
                self.update_terminal("Cannot start: previous stop has not completed with confirmed idle.\n")
                return
            self.stop_worker = None
        self.test_stop_event.clear()
        self.acquisition_error = None
        self.operator_stop_requested = False
        self.acquisition_done_event.clear()
        try:
            self.run_motion_token = self.motion_coordinator.acquire("strain-test")
        except MotionConflictError as exc:
            self.update_terminal(f"Cannot start test: {exc}\n")
            return
        # Admission above has retired the previous stop. Do not diagnose this
        # new run using a timer from an earlier, fully completed Stop operation.
        self.stop_started_monotonic = None
        self.stop_diagnostic_recorded = False
        self.strain_test_active = True
        self.csv_recorder = None
        self.data_collection_thread = None
        self.strain_thread = None
        self.rotary_samples_file_name = None

        self.set_test_inputs_state("disabled")
        self.buttons[0].configure(state="disabled")
        self.buttons[1].configure(state="disabled")
        self.manual_mode_toggle.configure(state="disabled")
        self.run_rotary_targeted = bool(getattr(mapping_request,"rotary_targeted",False))
        self.run_stationary_holds = bool(getattr(mapping_request,"stationary_holds",False))
        self.run_motion_model_enabled = use_model
        self.run_motion_model = None
        self.run_feedback_window = None
        self.run_original_zero_anchor = None
        self.run_model_variant = variant
        self.run_segmented_comparison = getattr(mapping_request,'stage','') in ('commissioning','model_comparison','rotary_feedback_pilot')
        self.run_feedback_pilot = getattr(mapping_request,'feedback_pilot',False)
        self.run_metadata = None
        self.metadata_file_name = None
        self.strain_file_name = None
        self.run_motion_mapping_context = mapping_request.context() if mapping_request is not None else None
        self.run_batch_metadata_key = getattr(mapping_request, 'metadata_key', 'motion_modelling') if mapping_request is not None else None
        preset_metadata = (mapping_request.preset_metadata if mapping_request is not None
                           else self.current_preset_metadata(parameters))
        accepted = self._submit_lifecycle("test initialization", lambda: self._initialize_test_worker(parameters, preset_metadata))
        if not accepted:
            self._request_coordinated_stop("Test initialization could not be scheduled", fault=True)
            self.cleanup_thread = threading.Thread(
                target=self._cleanup_failed_start, args=("Test initialization could not be scheduled",),
                name="east-failed-start-cleanup", daemon=True,
            )
            self.cleanup_thread.start()
        return accepted

    def _initialize_test_worker(self, parameters, preset_metadata):
        self.test_started_monotonic = time.monotonic()
        try:
            snapshot = self.get_feedback()
            if snapshot.active_errors:
                raise RuntimeError(f"ODrive has active errors: {snapshot.active_errors}")

            self.voltage_ratio_input = self._ensure_load_cell()
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

            # The UI check may precede confirmation by seconds. Recheck on the service.
            self._validate_live_motion_config()
            self._revalidate_hardware_identity(require_reference=True)
            self.rotary_sensor.measurement(require=True)
            snapshot = self.get_feedback()
            pilot_start_zero = None
            context = getattr(self, "run_motion_mapping_context", None) or {}
            if context.get("stage") == "rotary_feedback_pilot":
                reference = self.reference_manager.require_verified()
                pilot_start_zero = self.observe_idle_neutral(
                    reference.neutral_position_turns,
                    cancelled=lambda: self.test_stop_event.is_set() or self.acquisition_error is not None,
                    record_return=False,
                )
            elif not self.rotary_zero_ready(rotary_only=self.run_rotary_targeted or self.run_motion_model_enabled):
                raise RuntimeError("Rotary zero changed during test confirmation; home again before testing")
            self.run_parameters = parameters
            self.configure_trajectory(
                parameters.commanded_afo_speed_deg_s,
                parameters.commanded_afo_acceleration_deg_s2,
                filtered=context.get('protocol_key') == SMOOTH_PAIR,
            )
            app_base = Path(sys.executable).resolve().parent if getattr(sys, "frozen", False) else Path(__file__).resolve().parent
            csv_path, metadata_path = create_run_paths(parameters, self.system_config, app_base)
            self.strain_file_name = str(csv_path)
            self.metadata_file_name = str(metadata_path)
            with open(self.strain_file_name, mode="x", newline="", encoding="utf-8") as file:
                writer = csv.writer(file)
                writer.writerow(CSV_COLUMNS + ROTARY_RUN_COLUMNS + MOTOR_TIMING_COLUMNS)

            self.run_metadata = make_run_metadata(
                parameters,
                self.system_config,
                self.empty_machine_tare.offset_v_per_v,
                csv_path,
                self.odrive_configuration_snapshot(),
                tare_metadata=self.empty_machine_tare.to_dict(),
                preset_metadata=preset_metadata,
            )
            if pilot_start_zero is not None:
                self.run_metadata["pilot_start_zero_check"] = pilot_start_zero
            context = getattr(self, "run_motion_mapping_context", None)
            if isinstance(context, dict):
                key = getattr(self, 'run_batch_metadata_key', None) or 'motion_modelling'
                self.run_metadata[key] = dict(context)
            self.run_metadata['rotary_zero_reference'] = {
                'method':self.reference_manager.require_verified().verification_method,
                'calibration_sha256':self.rotary_sensor.calibration.sha256,
                'tolerance_deg':ZERO_TOLERANCE_DEG, 'angle_at_initialization_deg':self.rotary_sensor.measurement(require=True)['angle_deg']}
            self.run_metadata["experimental_motion_model"] = (model_provenance() if self.run_motion_model_enabled else {"enabled":False,"mode":"legacy_fixed_conversion"})
            self.run_metadata["experimental_motion_model"]["enabled"] = self.run_motion_model_enabled
            self.run_metadata["motion_method"] = ("Wider cubic candidate (frozen)" if getattr(self,"run_model_variant","current")=="candidate" else UPDATED_METHOD) if self.run_motion_model_enabled else ORIGINAL_METHOD
            self.run_metadata["rotary_targeted_acquisition"] = {"enabled":self.run_rotary_targeted,"stationary_holds":self.run_stationary_holds,"endpoint_accuracy_not_independent_model_validation":self.run_rotary_targeted,"endpoint_records":[],"holds":[],"bulk_fraction":0.8,"fine_speed_deg_s":1.0}
            self.run_metadata["schema_version"] = 4
            self.run_metadata["software"]["gui_version"] = APP_VERSION
            self.rotary_samples_file_name = str(csv_path.with_name(csv_path.stem.replace("_strain_data", "")+"_rotary_samples.csv"))
            self.run_metadata["rotary_sensor"] = self.rotary_sensor.metadata()
            if self.run_motion_model_enabled:
                self.run_metadata["rotary_sensor"]["angle_use"] = "Calibrated angle anchors experimental feed-forward targets and checks zero return; raw motor-derived columns and torque geometry retain legacy conversion"
            self.run_metadata["rotary_sensor"]["raw_samples_file"] = Path(self.rotary_samples_file_name).name
            if 'protocol_batch' in self.run_metadata:
                self.run_metadata['protocol_batch_files'] = {
                    'batch_manifest':self.run_metadata['protocol_batch']['batch_manifest'],
                    'metadata_file':str(metadata_path),'strain_file':str(csv_path),
                    'rotary_file':self.rotary_samples_file_name}
            self.run_metadata["rotary_sensor"]["join_method"] = "Latest delivered sample at load-cell read; age and sequence recorded; no interpolation"
            self.run_metadata["motor_timing_provenance"] = {
                "position_timestamp": "host completion of position remote read, not end of the multi-property bundle",
                "brackets": "host monotonic seconds; not hardware synchronisation",
                "capture_duration": "transport queue wait plus IO lock wait and all requested property reads",
                "uptime": "omitted during powered monitor captures; read during idle/identity/recovery diagnostics",
                "limits": "production 150 ms capture and 250 ms position freshness unchanged",
            }
            self.run_metadata["torque_angle_source"] = "ODrive-derived AFO angle; existing geometry calculation unchanged"
            self.run_metadata["plot_angle_source"] = "Measured Fixture Angle from calibrated rotary sensor"
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
            self.acquisition_overruns = 0
            self.maximum_acquisition_lateness_s = 0.0
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
            self.last_rotary_logged_key = None
            if self.test_stop_event.is_set() or self.engine.cancel_event.is_set():
                raise TestStopped("Test cancelled before recording/motion startup")
            service=getattr(self,'feedback_service',None)
            if service is not None and hasattr(service,'begin_run_statistics'):
                self.run_feedback_window=service.begin_run_statistics()
            self.rotary_sensor.start_recording(self.rotary_samples_file_name, self.test_started_monotonic,
                allow_linear_extrapolation=(getattr(self,'run_segmented_comparison',False)
                    and (self.run_motion_mapping_context or {}).get('stage') in ('commissioning','model_comparison')))
            self.csv_recorder = CSVRecorder(
                self.strain_file_name,
                self.system_config["acquisition"]["csv_flush_rows"],
                lambda exc: self._persistence_fault("recording_health", exc),
            )
            self.csv_recorder.start()
            self.data_collection_thread = threading.Thread(
                target=self.continuous_strain_read, name="strain-acquisition"
            )
            self.data_collection_thread.start()
            self.strain_thread = threading.Thread(
                target=self.strain_test_control, name="strain-motion"
            )
            self.strain_thread.start()
            self.engine.transition(Lifecycle.MOVING)
            self.set_status("TEST RUNNING", GREEN)
            self.update_terminal(
                f"Strain test started. Data: {self.strain_file_name}\n"
                f"Commanded ODrive trajectory speed: {self.commanded_odrive_velocity:.6f} turns/s\n"
                f"Commanded AFO acceleration: {self.commanded_afo_acceleration:g}"
                f"\N{DEGREE SIGN}/s\N{SUPERSCRIPT TWO}\n"
            )
        except Exception as exc:
            self._request_coordinated_stop(f"test initialization error: {exc}", fault=True)
            self.update_terminal(f"Error initializing strain test: {exc}\n")
            if self.strain_thread is None or not self.strain_thread.is_alive():
                self.cleanup_thread = threading.Thread(
                    target=self._cleanup_failed_start, args=(str(exc),),
                    name="east-failed-start-cleanup", daemon=True,
                )
                self.cleanup_thread.start()

    def _cleanup_failed_start(self, error):
        stop = self.stop_worker
        if stop is not None:
            stop.join(timeout=self.system_config["reference"]["idle_confirmation_timeout_s"] + 0.25)
            self._retire_stop_worker()
        data = self.data_collection_thread
        if data is not None and data.is_alive():
            data.join(timeout=2.0)
        data_alive = data is not None and data.is_alive()
        recorder = getattr(self, "csv_recorder", None)
        if recorder is not None and not recorder.done.is_set():
            try:
                recorder.finish(timeout_s=3.0)
            except Exception as exc:
                self.ui_message_queue.put(("terminal", f"CSV recorder cleanup incomplete: {exc}\n"))
        if not data_alive:
            try:
                statistics = self.rotary_sensor.finish_recording()
                if self.run_metadata is not None:
                    self.run_metadata.setdefault("rotary_sensor", {})["recording_statistics"] = statistics
            except Exception as exc:
                self.update_terminal(f"Rotary cleanup failed: {exc}\n")
        token = self.run_motion_token
        if (token is not None and self.motion_coordinator.motor_idle_confirmed
                and (stop is None or not stop.is_alive())
                and not data_alive
                and (recorder is None or not recorder.thread.is_alive())
                and not any(worker.is_alive() for worker in self.metadata_workers)):
            self.motion_coordinator.release(token)
            self.run_motion_token = None
        self.strain_test_active = False
        if self.run_metadata is None:
            self.run_finalized = True
        if self.run_metadata is not None and not self.run_finalized:
            self.finalize_run("error", error,
                              idle_confirmed=self.motion_coordinator.motor_idle_confirmed)
        elif self.metadata_file_name and self.run_metadata is None:
            self.run_metadata = {
                "run_status": "error", "error": f"Initialization failed: {error}",
                "outcome_pending_cleanup": bool(token is not None and self.run_motion_token == token),
                "idle_confirmation": {"confirmed": bool(self.motion_coordinator.motor_idle_confirmed),
                                      "motion_owner": self.motion_coordinator.owner},
                "strain_data_file": Path(self.strain_file_name).name if self.strain_file_name else None,
                "software": {"gui_version": APP_VERSION},
            }
            try:
                write_json_atomic(Path(self.metadata_file_name), self.run_metadata)
            except Exception as exc:
                self.ui_message_queue.put(("terminal", f"Initialization error metadata not saved: {exc}\n"))
        live = [worker for worker in (stop, data, recorder.thread if recorder else None,
                                      *self.metadata_workers)
                if worker is not None and worker.is_alive()]
        if live and token is not None:
            self.cleanup_thread = threading.Thread(
                target=self._release_run_ownership_when_workers_exit,
                args=(token, live, data_alive), name="east-failed-start-delayed-cleanup", daemon=True,
            )
            self.cleanup_thread.start()
        self.ui_message_queue.put(("inputs", "normal"))
        self.ui_message_queue.put(("run_buttons", "normal"))
    


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
        if self.motion_coordinator.shutting_down:
            self.update_terminal("Tare blocked during shutdown.\n")
            return
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

        self._submit_lifecycle("empty-machine tare", lambda: self._tare_worker(operator, fixture_id, calibration_id))

    def _tare_worker(self, operator, fixture_id, calibration_id):
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
            if not self.rotary_zero_ready():
                raise RuntimeError("Home to rotary zero before capturing unloaded tare")

            device = self._ensure_load_cell()
            sample_count = int(self.system_config["load_cell"]["tare_samples"])
            samples = []
            self.update_terminal(
                f"Capturing empty-machine tare from {sample_count} load-cell samples...\n"
            )
            for _ in range(sample_count):
                if self.engine.closing or self.engine.cancel_event.is_set() or self.motion_coordinator.stopping:
                    raise RuntimeError("Tare cancelled by Stop/shutdown")
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
            self.engine.record("tare", {"valid":True,"identity":self.empty_machine_tare.to_dict()})
            self.update_terminal(
                "Empty-machine tare captured. Mounting an AFO will not trigger another tare.\n"
            )
            self.update_tare_display()
            self._refresh_motion_controls()
        except Exception as exc:
            self.update_terminal(f"Empty-machine tare failed: {exc}\n")
            raise
        finally:
            # Keep the one main Bridge channel connected; normal Start never tares.
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
                rotary = self.rotary_sensor.measurement(require=True)
                if getattr(self,"run_rotary_targeted",False) is True:
                    limit = self._rotary_acquisition_travel_limit()
                    if abs(rotary["angle_deg"]) > limit:
                        raise RuntimeError(f"Rotary acquisition travel guard exceeded: measured {rotary['angle_deg']:+.4f} degrees; limit +/-{limit:g}")
                rotary_sample = rotary["sample"]
                rotary_key = (rotary_sample[4], rotary_sample[3])
                rotary_new_sample = rotary_key != self.last_rotary_logged_key
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
                    f"{rotary_sample[2]:.12g}",
                    f"{rotary['angle_deg']:.8f}",
                    rotary_sample[1],
                    f"{rotary_sample[0]:.9f}",
                    f"{rotary['sample_age_s']:.9f}",
                    rotary_sample[3],
                    rotary_sample[4],
                    rotary_new_sample,
                    rotary["valid"],
                    rotary["sample_status"],
                    self.rotary_sensor.calibration.profile["calibration_id"],
                    f"{rotary['achieved_rate_hz']:.3f}",
                    f"{snapshot.captured_monotonic_s:.9f}",
                    rotary_sample[5],
                    rotary_sample[6],
                    snapshot.position_started_monotonic_s,
                    snapshot.position_finished_monotonic_s,
                    snapshot.velocity_started_monotonic_s,
                    snapshot.velocity_finished_monotonic_s,
                    max(0.0, monotonic_time-snapshot.captured_monotonic_s),
                    snapshot.capture_started_monotonic_s,
                    snapshot.capture_duration_s,
                    snapshot.transport_queue_wait_s,
                    snapshot.lock_wait_s,
                ]
                self.last_rotary_logged_key = rotary_key
                recorder = getattr(self, "csv_recorder", None)
                if recorder is not None:
                    recorder.append(data_row)
                else:
                    self.strain_data_buffer.append(data_row)
                
                # Collect plot data regardless of which output tab is visible.
                if plot_window_open:
                    self.plot_update_counter += 1
                    if self.plot_update_counter >= self.plot_update_interval:
                        try:
                            self.plot_data_queue.put_nowait((rotary["angle_deg"], avg_torque))
                        except queue.Full:
                            # Display only: raw measurement already queued for recording.
                            pass
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
        from east_rotary_homing import RotaryZeroMiss
        from east_full_model_suite import ComparisonMotionMiss
        final_status = "aborted"
        final_error = None
        idle_confirmed = False
        candidate_completed = False
        token = self.run_motion_token
        motion_context = getattr(self, 'run_motion_mapping_context', None) or {}
        smooth_pair = motion_context.get('protocol_key') == SMOOTH_PAIR
        try:
            if token is None:
                raise RuntimeError("Test motion ownership was not acquired")
            if self.test_stop_event.is_set():
                raise TestStopped()
            self.rotary_sensor.measurement(require=True)
            self.enter_closed_loop(token)
            mapping = self.reference_manager.require_verified()
            self.update_terminal(
                f"Verified machine zero (fixture physical 90 degrees): "
                f"{mapping.neutral_position_turns:.8f} session turns\n"
            )
            parameters = self.run_parameters
            absolute_min = self.reference_manager.target_for_angle(-parameters.min_angle_deg)
            absolute_max = self.reference_manager.target_for_angle(parameters.max_angle_deg)
            from east_stream_screen import SCREEN_PRESET, StreamScreenModel
            from east_full_model_suite import PRESET as FULL_COMPARISON
            screen_key = motion_context.get('protocol_key')
            stream_screen = screen_key in (SCREEN_PRESET, FULL_COMPARISON, SMOOTH_PAIR)
            from east_feedback_commissioning import condition_for
            commissioning = condition_for(motion_context.get(
                'feedback_condition_key', motion_context.get('protocol_key', '')))
            if getattr(self, "run_motion_model_enabled", False) is True:
                validate_model_range(parameters.min_angle_deg, parameters.max_angle_deg)
                anchor = self._test_feedback()
                if abs(anchor.velocity_turns_s*float(self.system_config["motion"]["afo_degrees_per_odrive_turn"])) > float(self.system_config["reference"]["stationary_velocity_limit_deg_s"]):
                    raise RuntimeError("Model anchor requires stationary motor feedback")
                model_class=RunModel
                if getattr(self,'run_model_variant','current')=='candidate':
                    from east_wide_candidate import WideRunModel
                    model_class=WideRunModel
                rotary_angle = self.rotary_sensor.measurement(require=True)["angle_deg"]
                if stream_screen:
                    self.run_motion_model = StreamScreenModel.anchored(self.run_model_variant, anchor.position_turns, rotary_angle,
                        full_range=screen_key in (FULL_COMPARISON, SMOOTH_PAIR))
                    self.run_metadata["motion_method"] = ("Smoothed wider cubic" if smooth_pair else f"Streamed {self.run_model_variant}") + (
                        " + rotary feedback" if self.run_feedback_pilot else " feedforward")
                else:
                    self.run_motion_model = model_class.anchored(anchor.position_turns, rotary_angle)
                self.run_metadata["experimental_motion_model"].update(self.run_motion_model.metadata())
                if commissioning:
                    self.run_metadata["experimental_motion_model"]["zero_alignment"] = (
                        "increasing branch anchored at measured zero; fixed origin across all legs")
                if stream_screen:
                    self.update_terminal(f"{self.run_metadata['motion_method']}; fixed rotary anchor and common trajectory.\n")
                else:
                    self.update_terminal("Experimental inverse-cubic test motion active; rotary-anchored targets, secant-average speed conversion.\n")

            if (getattr(self,'run_segmented_comparison',False) and not self.run_motion_model_enabled
                    and (self.run_motion_mapping_context or {}).get('stage') in ('commissioning','model_comparison')):
                from east_motion_coordinates import LinearPoseAnchor
                anchor=self._test_feedback();measured=self.rotary_sensor.measurement(require=True)['angle_deg']
                if abs(anchor.velocity_turns_s*float(self.system_config['motion']['afo_degrees_per_odrive_turn']))>self.system_config['reference']['stationary_velocity_limit_deg_s']:
                    raise RuntimeError('Original pose anchor requires stationary feedback')
                self.run_original_zero_anchor=LinearPoseAnchor(anchor.position_turns,measured,
                    float(self.system_config['motion']['afo_degrees_per_odrive_turn']))
                self.run_metadata['original_run_anchor']=self.run_original_zero_anchor.metadata()
                self.run_metadata['original_run_segments']=[]

            self.update_terminal(
                f"Commanded AFO range: -{parameters.min_angle_deg:g}\N{DEGREE SIGN} to "
                f"+{parameters.max_angle_deg:g}\N{DEGREE SIGN}\n"
                f"Commanded AFO speed: {parameters.commanded_afo_speed_deg_s:g}"
                f"\N{DEGREE SIGN}/s\n"
                f"Commanded AFO acceleration: {parameters.commanded_afo_acceleration_deg_s2:g}"
                f"\N{DEGREE SIGN}/s\N{SUPERSCRIPT TWO}\n"
            )
            self.current_cycle = 0
            if getattr(self,"run_stationary_holds",False) is True:
                self.run_direction_holds(token)
            else:
                self.command_test_angle(parameters.max_angle_deg, absolute_max, "moving_to_initial_max", token)
                if self.run_motion_model is not None and not stream_screen and commissioning is None:
                    # The first approach supplies a known increasing branch. A stationary
                    # zero has no identifiable branch; refine only the coordinate origin,
                    # never coefficients, before locking it for all numbered sweeps.
                    snapshot = self._test_feedback()
                    measured = self.rotary_sensor.measurement(require=True)["angle_deg"]
                    initial_alignment = dict(origin_turns=self.run_motion_model.origin_turns,
                        anchor_motor_turns=self.run_motion_model.anchor_turns,
                        anchor_rotary_angle_deg=self.run_motion_model.anchor_angle_deg)
                    self.run_motion_model = type(self.run_motion_model).observed(snapshot.position_turns, measured, 1)
                    details = self.run_metadata["experimental_motion_model"]
                    segments = details["segments"]
                    details.update(self.run_motion_model.metadata())
                    details["segments"] = segments
                    details["initial_alignment"] = initial_alignment
                    details["numbered_cycle_origin_locked_after_initial_positioning"] = True
                    self.update_terminal("Model origin aligned once after initial increasing positioning; frozen for numbered cycles.\n")
                for cycle in range(1, parameters.cycles + 1):
                    self.current_cycle = cycle
                    self.command_test_angle(-parameters.min_angle_deg, absolute_min, "moving_to_min", token)
                    self.command_test_angle(parameters.max_angle_deg, absolute_max, "moving_to_max", token)
                    if self.test_stop_event.is_set() or self.acquisition_error:
                        raise TestStopped()
                    self.completed_cycles = cycle
                    self.update_terminal(f"Completed cycle {cycle}/{parameters.cycles}\n")

            motion = self.system_config["motion"]
            engineering_return = (motion_context.get('stage')
                                  or motion_context.get('purpose') == 'nonlinear_motion_identification')
            if not engineering_return or parameters.test_type == 'afo_test':
                self.complete_rotary_zero_return(token)
                idle_confirmed = True
            else:
                # Engineering comparisons measure the model's unassisted return.
                if not smooth_pair:
                    self.configure_trajectory(
                        motion["neutral_return_speed_deg_s"],
                        motion["neutral_return_acceleration_deg_s2"],
                    )
                neutral_target = self.command_test_angle(
                    0.0, mapping.neutral_position_turns, "returning_to_machine_zero", token,
                    speed_deg_s=motion["neutral_return_speed_deg_s"],
                    acceleration_deg_s2=motion["neutral_return_acceleration_deg_s2"],
                )
                idle_result = self.safe_idle_motor("trajectory return complete; verifying final zero", token=token)
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
        except ComparisonMotionMiss as exc:
            final_status = "error"
            final_error = str(exc)
            result = self.safe_idle_motor('saving incomplete comparison before automatic homing', token=token)
            idle_confirmed = bool(result and result.confirmed)
            self.update_terminal(f"Comparison run incomplete: {exc}. Saving failed attempt; automatic homing precedes the next condition.\n")
        except RotaryZeroMiss as exc:
            final_status = "error"
            final_error = str(exc)
            mapping = getattr(self, "motion_mapping", None)
            if (mapping is not None and getattr(mapping, "active", False)
                    and getattr(mapping, "preset_snapshot", {}).get("allow_bounded_zero_recovery")):
                self.update_terminal(
                    f"Measurement cycles complete. Unassisted zero return recorded: {exc}\n"
                    "Saving recordings, then automatically homing before continuing.\n")
            else:
                self.update_terminal(f"Error during strain test: {exc}\n")
        except Exception as exc:
            final_status = "error"
            final_error = str(exc)
            self.update_terminal(f"Error during strain test: {exc}\n")
        finally:
            stop_was_requested = self.test_stop_event.is_set()
            self.test_stop_event.set()
            # Motor idle is requested before waiting for acquisition or closing devices.
            if not idle_confirmed:
                self._request_coordinated_stop(final_error or final_status,
                                               fault=final_status == "error")
                worker = self.stop_worker
                if worker is not None and worker is not threading.current_thread():
                    worker.join(timeout=self.system_config["reference"]["idle_confirmation_timeout_s"] + 0.25)
                self._retire_stop_worker()
                idle_result = self.stop_result if worker is not None and not worker.is_alive() else None
                if not idle_result or not idle_result.confirmed:
                    final_status = "error"
                    suffix = "ODrive idle was not confirmed"
                    final_error = f"{final_error}; {suffix}" if final_error else suffix
                else:
                    idle_confirmed = True
            if idle_confirmed and smooth_pair:
                try:
                    motion = self.system_config["motion"]
                    self.configure_trajectory(motion["neutral_return_speed_deg_s"],
                                              motion["neutral_return_acceleration_deg_s2"])
                except Exception as exc:
                    candidate_completed = False
                    final_status = "error"
                    detail = f"Could not restore native ODrive mode after idle: {exc}"
                    final_error = f"{final_error}; {detail}" if final_error else detail
            data_alive = False
            if self.data_collection_thread is not None:
                self.data_collection_thread.join(timeout=2.0)
                data_alive = self.data_collection_thread.is_alive()
                if data_alive:
                    final_status = "error"
                    suffix = "data acquisition thread did not stop within 2 seconds"
                    final_error = f"{final_error}; {suffix}" if final_error else suffix
            if not data_alive:
                try:
                    rotary_stats = self.rotary_sensor.finish_recording()
                    if self.run_metadata is not None:
                        self.run_metadata["rotary_sensor"]["recording_statistics"] = rotary_stats
                except Exception as exc:
                    self.acquisition_error = str(exc)
                    if self.run_metadata is not None:
                        self.run_metadata["rotary_sensor"]["recording_statistics"] = self.rotary_sensor.last_recording_statistics
            final_status, final_error = reconcile_run_outcome(
                candidate_completed=candidate_completed,
                stop_requested=stop_was_requested or self.operator_stop_requested,
                acquisition_error=self.acquisition_error,
                current_status=final_status,
                current_error=final_error,
            )
            # A surviving stop/acquisition worker keeps ownership until it exits.
            live_cleanup = [thread for thread in (self.stop_worker, self.data_collection_thread,
                                                  self.csv_recorder.thread if self.csv_recorder else None,
                                                  *self.metadata_workers)
                            if thread is not None and thread.is_alive()]
            if live_cleanup and token is not None:
                self.cleanup_thread = threading.Thread(
                    target=self._release_run_ownership_when_workers_exit,
                    args=(token, live_cleanup, data_alive), name="east-run-cleanup", daemon=True,
                )
                self.cleanup_thread.start()
            elif token is not None:
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

    def _release_run_ownership_when_workers_exit(self, token, workers, finish_rotary=False):
        for worker in workers:
            worker.join()
        self._retire_stop_worker()
        cleanup_ok = True
        if finish_rotary:
            try:
                statistics = self.rotary_sensor.finish_recording()
                with self.finalize_lock:
                    if self.run_metadata is not None:
                        self.run_metadata.setdefault("rotary_sensor", {})["recording_statistics"] = statistics
                        write_json_atomic(Path(self.metadata_file_name), self.run_metadata)
            except Exception as exc:
                cleanup_ok = False
                self.ui_message_queue.put(("terminal", f"Delayed rotary cleanup failed: {exc}\n"))
        if self.motion_coordinator.motor_idle_confirmed:
            self.motion_coordinator.release(token)
            if self.run_motion_token == token:
                self.run_motion_token = None
            self.ui_message_queue.put(("reference",))
            if self.run_finalized and cleanup_ok:
                with self.finalize_lock:
                    if self.run_metadata is not None:
                        self.run_metadata["outcome_pending_cleanup"] = False
                        self.run_metadata["idle_confirmation"]["confirmed"] = True
                        self.run_metadata["idle_confirmation"]["motion_owner"] = None
                        try:
                            write_json_atomic(Path(self.metadata_file_name), self.run_metadata)
                        except Exception as exc:
                            cleanup_ok = False
                            self.run_metadata["outcome_pending_cleanup"] = True
                            self.ui_message_queue.put(("terminal", f"Cleanup metadata update failed: {exc}\n"))
                if cleanup_ok:
                    self.ui_message_queue.put(("inputs", "normal"))
                    self.ui_message_queue.put(("run_buttons", "normal"))

    def check_rotary_acquisition(self, token):
        self.motion_coordinator.assert_active(token)
        if self.test_stop_event.is_set() or self.acquisition_error:
            raise TestStopped()
        snapshot = self._test_feedback()
        if snapshot.current_state != int(AXIS_STATE_CLOSED_LOOP_CONTROL) or snapshot.active_errors:
            raise FeedbackError('Controller state/errors changed during rotary acquisition')
        if snapshot.disarm_reason != self.expected_disarm_reason:
            raise FeedbackError('Controller disarm reason changed during rotary acquisition')
        value = self.rotary_sensor.measurement(require=True)
        self._mark_control_health()
        limit = self._rotary_acquisition_travel_limit()
        if abs(value['angle_deg']) > limit:
            raise RuntimeError(f"Rotary acquisition travel guard exceeded: measured {value['angle_deg']:+.4f} degrees; limit +/-{limit:g}")
        return snapshot,value

    def observe_rotary_hold(self, duration, token):
        import statistics
        start=time.monotonic();baseline=self.check_rotary_acquisition(token)[1]['sample'][3]
        samples=set();angles=[]
        while time.monotonic()-start < duration:
            snapshot,value=self.check_rotary_acquisition(token)
            if abs(snapshot.velocity_turns_s*float(self.system_config['motion']['afo_degrees_per_odrive_turn'])) > self.system_config['reference']['settle_velocity_limit_deg_s']:
                raise RuntimeError('Motor not stationary during rotary acquisition hold')
            seq=value['sample'][3]
            if seq>baseline and seq not in samples:
                samples.add(seq);angles.append(value['angle_deg'])
            if self.test_stop_event.wait(.01):
                raise TestStopped()
        if len(angles)<5:
            raise RuntimeError('Insufficient fresh rotary samples during acquisition hold')
        return dict(mean_deg=statistics.mean(angles),min_deg=min(angles),max_deg=max(angles),
            sd_deg=statistics.pstdev(angles),samples=len(angles),duration_s=time.monotonic()-start,
            started_at_monotonic_s=start,completed_at_monotonic_s=time.monotonic())

    def command_rotary_endpoint(self, angle_deg, phase, token, speed_deg_s=None, acceleration_deg_s2=None):
        from east_rotary_positioning import reach
        speed=self.run_parameters.commanded_afo_speed_deg_s if speed_deg_s is None else float(speed_deg_s)
        acceleration=self.run_parameters.commanded_afo_acceleration_deg_s2 if acceleration_deg_s2 is None else float(acceleration_deg_s2)
        evidence={'phase':phase,'cycle':self.current_cycle,'target_angle_deg':float(angle_deg),'bulk':[],'fine':[]}
        self.run_metadata['rotary_targeted_acquisition']['endpoint_records'].append(evidence)
        def move(delta, fine=False):
            snapshot,_=self.check_rotary_acquisition(token)
            step=afo_degrees_to_odrive_turns(delta,self.system_config)
            target=snapshot.position_turns+step
            sp,acc=(1.0,5.0) if fine else (speed,acceleration)
            self.configure_trajectory(sp,acc)
            evidence['fine' if fine else 'bulk'].append(dict(nominal_delta_deg=delta,target_motor_turns=target,speed_deg_s=sp,acceleration_deg_s2=acc))
            self.command_position_and_wait(target,('rotary_approach_' if fine else '')+phase,token,sp,acc)
        result=reach(angle_deg,lambda:self.check_rotary_acquisition(token)[1]['angle_deg'],
            lambda d:move(d),lambda d:move(d,True),lambda duration:self.observe_rotary_hold(duration,token),
            lambda:self.check_rotary_acquisition(token))
        evidence['result']=result
        self.motion_phase='rotary_settled_'+phase
        return self.check_rotary_acquisition(token)[0].position_turns

    def run_direction_holds(self, token):
        snapshot,_=self.check_rotary_acquisition(token)
        self.command_position_and_wait(snapshot.position_turns,'zero_stability_settle',token,1.0,5.0)
        self.motion_phase='zero_stability_30s'
        self.run_metadata['rotary_targeted_acquisition']['zero_stability']=self.observe_rotary_hold(30.,token)
        for cycle in range(1,4):
            self.current_cycle=cycle
            for target in (-10.,-5.,0.,5.,10.):
                for direction in (1,-1):
                    self.command_rotary_endpoint(target-direction*2.,'hold_preposition',token,5.,100.)
                    self.command_rotary_endpoint(target,'hold_target',token,1.,5.)
                    self.motion_phase=f'stationary_hold_{target:g}_{direction:+d}'
                    result=self.observe_rotary_hold(2.,token)
                    self.run_metadata['rotary_targeted_acquisition']['holds'].append(dict(cycle=cycle,
                        target_angle_deg=target,planned_approach_direction=direction,**result))
            self.completed_cycles=cycle
            self.update_terminal(f'Completed stationary hold repeat {cycle}/3\n')

    def command_rotary_feedback_pilot(self,target_angle,phase,token):
        from east_execution import MotionCommandCancelled
        try:
            return self._command_rotary_feedback_pilot(target_angle,phase,token)
        except (MotionCommandCancelled,MotionConflictError) as exc:
            # Stop may cancel a pending transport write or invalidate ownership
            # between the loop guard and dispatch. Preserve actual health faults.
            if (self.operator_stop_requested and self.test_stop_event.is_set()
                    and not self.monitor_error and not self.acquisition_error):
                raise TestStopped() from exc
            raise

        except (RuntimeError, ValueError) as exc:
            from east_full_model_suite import PRESET, RECOVERABLE_PREFIXES, ComparisonMotionMiss
            context = getattr(self, 'run_motion_mapping_context', None) or {}
            if context.get('protocol_key') == PRESET and str(exc).startswith(RECOVERABLE_PREFIXES):
                raise ComparisonMotionMiss(str(exc)) from exc
            raise

    def _command_rotary_feedback_pilot(self,target_angle,phase,token):
        from east_rotary_feedback import step,ramp,PERIOD_S
        from east_wide_candidate import curve
        from east_feedback_commissioning import policy
        context = getattr(self, "run_motion_mapping_context", None) or {}
        commissioning = policy(self.system_config, context.get("feedback_condition_key", context.get("protocol_key", "")))
        period = commissioning["reference_period_s"] if commissioning else PERIOD_S
        half_range = commissioning["half_range_deg"] if commissioning else 5.
        speed = commissioning["speed_deg_s"] if commissioning else .5
        acceleration = commissioning["ramp_acceleration_deg_s2"] if commissioning else .5
        expected_acceleration = acceleration if commissioning else 5.
        if (self.run_parameters.test_type != 'empty_machine_baseline'
                or abs(target_angle) > half_range
                or self.run_parameters.min_angle_deg != half_range
                or self.run_parameters.max_angle_deg != half_range
                or self.run_parameters.commanded_afo_speed_deg_s != speed
                or self.run_parameters.commanded_afo_acceleration_deg_s2 != expected_acceleration):
            raise RuntimeError('Feedback parameters differ from the explicitly selected fixed condition')
        motor_speed = commissioning["motor_speed_nominal_deg_s"] if commissioning else .75
        motor_acceleration = commissioning["motor_acceleration_nominal_deg_s2"] if commissioning else 5.
        guard_arguments = ({key: commissioning[key] for key in (
            "max_command_rate_turns_s", "tracking_guard_deg", "travel_guard_deg", "tracking_error_action")} if commissioning else {})
        self.motion_phase=phase
        snapshot,value=self.check_rotary_acquisition(token)
        start={'moving_to_initial_max':0.,'moving_to_min':self.run_parameters.max_angle_deg,
            'moving_to_max':-self.run_parameters.min_angle_deg,'returning_to_machine_zero':self.run_parameters.max_angle_deg}[phase]
        direction=1 if target_angle>start else -1
        model=self.run_motion_model
        if model is None:raise RuntimeError('Feedback pilot requires frozen candidate')
        self.configure_trajectory(motor_speed, motor_acceleration)
        self._write_runtime_checkpoint(MotionState.MOVING,snapshot,clean_shutdown=False,extra={'phase':phase,'control':'rotary_feedback_pilot'})
        log=self.run_metadata.setdefault('rotary_feedback_pilot',{'enabled':bool(self.run_feedback_pilot),'streamed_reference':True,'period_s':period,'proportional_gain':.3 if self.run_feedback_pilot else 0.,
            'correction_limit_deg':.1,'tracking_guard_deg':1.,'travel_guard_deg':6.,'samples':[]})
        _, planned_duration = ramp(start, target_angle, speed, 0., acceleration=acceleration)
        leg_timeout = planned_duration + 6. if commissioning else 30.
        if commissioning and leg_timeout > commissioning["maximum_leg_timeout_s"]:
            raise RuntimeError("Commissioning leg exceeds the configured motion timeout")
        if commissioning:
            log["tracking_guard_deg"] = commissioning["tracking_guard_deg"]
            log["travel_guard_deg"] = commissioning["travel_guard_deg"]
            log["tracking_error_action"] = commissioning["tracking_error_action"]
            log["commissioning_policy"] = commissioning
            log.setdefault("legs", []).append(dict(
                phase=phase, reference_speed_deg_s=speed,
                reference_acceleration_deg_s2=acceleration, planned_duration_s=planned_duration,
                timeout_s=leg_timeout, motor_speed_turns_s=self.commanded_odrive_velocity,
                motor_acceleration_turns_s2=self.commanded_odrive_acceleration))
        begun=previous=time.monotonic();last=snapshot.position_turns;deadline=begun+leg_timeout
        while True:
            if self.test_stop_event.is_set() or self.acquisition_error:raise TestStopped()
            if time.monotonic()>deadline:raise RuntimeError('Rotary pilot leg timed out')
            snapshot,value=self.check_rotary_acquisition(token)
            now=time.monotonic();elapsed=now-begun
            desired,duration=ramp(start,target_angle,speed,elapsed,acceleration=acceleration)
            x=snapshot.position_turns-model.origin_turns
            if hasattr(model, "slope"):
                slope = model.slope(snapshot.position_turns, direction)
            else:
                slope = (curve(x + .0001, direction) - curve(x - .0001, direction)) / .0002
            row=step(desired,value['angle_deg'],model.target(desired,direction),last,max(.001,now-previous),slope,gain=.3 if self.run_feedback_pilot else 0., **guard_arguments)
            row.update(host_monotonic_s=now,motor_feedback_host_s=snapshot.captured_monotonic_s,
                rotary_sequence=value['sample'][3],rotary_sample_host_s=value['sample'][0],phase=phase,cycle=self.current_cycle,
                measured_motor_turns=snapshot.position_turns,direction=direction)
            log['samples'].append(row)
            last=row['command_target_turns'];self._submit_position(token,last);row['command_issued_at_monotonic_s']=time.monotonic();previous=now
            if elapsed>=duration+1.:break
            if self.test_stop_event.wait(period):raise TestStopped()
        # Stop issuing updates, settle the final command through the existing
        # supervised wait path. No endpoint acceptance is inferred from this.
        self.command_position_and_wait(last,phase,token,motor_speed,motor_acceleration,distance_deg=.5)
        return last

    def command_smoothed_angle(self, target, phase, token, speed=None, acceleration=None):
        from east_smooth_motion import BANDWIDTH, PERIOD_S, FilteredPositionSweep, Sweep, runnable_study_conditions

        parameters = self.run_parameters
        if (parameters.test_type != 'empty_machine_baseline'
                or (parameters.min_angle_deg, parameters.max_angle_deg, parameters.cycles) != (10., 10., 3)
                or (parameters.commanded_afo_speed_deg_s, parameters.commanded_afo_acceleration_deg_s2)
                not in runnable_study_conditions(self.system_config["motion"])):
            raise RuntimeError('Smoothed motion requires the fixed empty-fixture pair')
        returning = phase == 'returning_to_machine_zero'
        start = {'moving_to_initial_max': 0., 'moving_to_min': 10.,
                 'moving_to_max': -10., 'returning_to_machine_zero': 10.}[phase]
        speed = parameters.commanded_afo_speed_deg_s if speed is None else speed
        acceleration = parameters.commanded_afo_acceleration_deg_s2 if acceleration is None else acceleration
        sweep = Sweep.plan(start, target, speed, acceleration)
        self.motion_phase = phase
        snapshot, _ = self.check_rotary_acquisition(token)
        self._write_runtime_checkpoint(MotionState.MOVING, snapshot, clean_shutdown=False,
                                       extra={'phase': phase, 'control': 'filtered_position'})
        snapshot, rotary = self.check_rotary_acquisition(token)
        if abs(start - rotary['angle_deg']) > .5:
            raise RuntimeError('Smoothed sweep start differs from its planned endpoint')
        planner = FilteredPositionSweep(sweep, self.run_motion_model, snapshot.position_turns,
                                       feedback=self.run_feedback_pilot and not returning)
        log = self.run_metadata.setdefault('smoothed_motion', {
            'input_mode': 'POS_FILTER', 'bandwidth_per_s': BANDWIDTH,
            'command_period_s': PERIOD_S, 'maximum_dispatch_gap_s': .15,
            'feedback': bool(self.run_feedback_pilot), 'gain': planner.GAIN,
            'deadband_deg': planner.DEAD_BAND_DEG, 'trim_cap_deg': planner.TRIM_CAP_DEG,
            'trim_rate_deg_s': planner.TRIM_RATE_DEG_S, 'filter_tau_s': planner.FILTER_TAU_S,
            'samples': [], 'legs': []})
        log['legs'].append(dict(phase=phase, cycle=self.current_cycle,
                                planned_duration_s=sweep.duration, feedback=planner.feedback))
        origin = rotary['sample'][0]
        rotary_source = rotary['sample'][4]
        row = planner.command(0., measured=rotary['angle_deg'], sample_time=0.)
        self._submit_position(token, row['target_motor_turns'])
        previous = time.monotonic()
        while True:
            row.update(phase=phase, cycle=self.current_cycle, measured_angle_deg=rotary['angle_deg'],
                       rotary_sample_host_s=rotary['sample'][0], rotary_sequence=rotary['sample'][3],
                       measured_motor_turns=snapshot.position_turns,
                       command_issued_at_monotonic_s=previous)
            log['samples'].append(row)
            if row['time_s'] >= sweep.duration:
                break
            grid_due = origin + (math.floor((previous - origin) / PERIOD_S) + 1) * PERIOD_S
            due = max(grid_due, previous + PERIOD_S / 2)
            if self.test_stop_event.wait(max(0., due - time.monotonic())):
                raise TestStopped()
            snapshot, rotary = self.check_rotary_acquisition(token)
            now = time.monotonic()
            if now - previous > .15:
                raise RuntimeError('Smoothed motion command dispatch gap exceeded 150 ms')
            if rotary['sample'][4] != rotary_source:
                raise RuntimeError('Rotary connection changed during smoothed motion')
            try:
                row = planner.command(now - origin, measured=rotary['angle_deg'],
                                      sample_time=rotary['sample'][0] - origin)
            except ValueError as exc:
                log['rejected_command'] = dict(reason=str(exc), phase=phase, cycle=self.current_cycle,
                    host_monotonic_s=now, rotary_sample_host_s=rotary['sample'][0],
                    rotary_sequence=rotary['sample'][3], rotary_angle_deg=rotary['angle_deg'])
                raise
            if abs(row['angle_deg'] - rotary['angle_deg']) > 2.:
                raise RuntimeError('Smoothed motion tracking error exceeded 2 degrees')
            self._submit_position(token, row['target_motor_turns'])
            previous = time.monotonic()
        target_turns = row['target_motor_turns']
        self.command_position_and_wait(target_turns, phase, token, sweep.peak,
                                       acceleration, distance_deg=.5)
        return target_turns

    def command_test_angle(self, angle_deg, legacy_target, phase, token, speed_deg_s=None, acceleration_deg_s2=None):
        """Model targets only for normal tests. Manual/homing/identification stay legacy."""
        if getattr(self,'run_rotary_targeted',False) is True:
            return self.command_rotary_endpoint(angle_deg,phase,token,speed_deg_s,acceleration_deg_s2)
        model = getattr(self, 'run_motion_model', None)
        if (getattr(self, 'run_motion_mapping_context', None) or {}).get('protocol_key') == SMOOTH_PAIR:
            return self.command_smoothed_angle(angle_deg, phase, token, speed_deg_s, acceleration_deg_s2)
        if getattr(self,'run_segmented_comparison',False) and self.run_motion_mapping_context.get('stage')=='rotary_feedback_pilot':
            return self.command_rotary_feedback_pilot(angle_deg,phase,token)
        if model is None:
            original_anchor=getattr(self,'run_original_zero_anchor',None)
            if original_anchor is not None:
                legacy_target=original_anchor.target(float(angle_deg))
                self.run_metadata['original_run_segments'].append(dict(phase=phase,cycle=self.current_cycle,
                    target_angle_deg=float(angle_deg),target_motor_turns=legacy_target,conversion_degrees_per_turn=original_anchor.degrees_per_turn))
            speed=self.run_parameters.commanded_afo_speed_deg_s if speed_deg_s is None else float(speed_deg_s)
            start=self._test_feedback().position_turns
            distance=abs(legacy_target-start)*2.055
            import math
            count=math.ceil(distance/speed/30) if getattr(self,'run_segmented_comparison',False) else 1
            for i in range(1,count+1):
                self.command_position_and_wait(start+(legacy_target-start)*i/count,phase,token,speed_deg_s,acceleration_deg_s2,distance_deg=distance/count)
            return legacy_target
        self.motion_coordinator.assert_active(token)
        if self.test_stop_event.is_set():
            raise TestStopped()
        snapshot = self._test_feedback()
        rotary = self.rotary_sensor.measurement(require=True)
        measured = rotary['angle_deg']
        direction = 1 if angle_deg > measured else -1
        target = model.target(angle_deg,direction)
        delta_turns = target-snapshot.position_turns
        delta_angle = float(angle_deg)-float(measured)
        if abs(delta_angle) < 0.01 or delta_turns*direction <= 0:
            raise RuntimeError('Model target direction/anchor is inconsistent with live position; home and review the model')
        # Always check both start and destination lie in the fitted motor interval.
        from east_motion_model import curve
        curve(snapshot.position_turns-model.origin_turns,direction)
        ratio = abs(delta_turns/delta_angle)
        if not 0.3 <= ratio <= 0.7:
            raise RuntimeError('Model trajectory ratio is outside the experimental bounds')
        speed = self.run_parameters.commanded_afo_speed_deg_s if speed_deg_s is None else float(speed_deg_s)
        acceleration = self.run_parameters.commanded_afo_acceleration_deg_s2 if acceleration_deg_s2 is None else float(acceleration_deg_s2)
        self.configure_trajectory(speed,acceleration,turns_per_degree=ratio)
        fitted_bounds=model.metadata()['fitted_motor_range_turns']
        self.run_metadata['experimental_motion_model']['segments'].append(dict(
            phase=phase,cycle=self.current_cycle,target_angle_deg=float(angle_deg),direction=direction,
            target_motor_turns=target,start_motor_turns=snapshot.position_turns,start_rotary_angle_deg=measured,
            turns_per_degree_secant=ratio,nominal_speed_deg_s=speed,nominal_acceleration_deg_s2=acceleration,
            issued_at_monotonic_s=time.monotonic(),motor_feedback_at_monotonic_s=snapshot.captured_monotonic_s,
            rotary_sample_age_s=rotary.get("sample_age_s"),
            motor_velocity_turns_s=self.commanded_odrive_velocity,motor_acceleration_turns_s2=self.commanded_odrive_acceleration,
            motor_rate_capped=self.commanded_odrive_velocity < speed*ratio or self.commanded_odrive_acceleration < acceleration*ratio,
            outside_training_speed_range=not 0.5 <= speed <= 5.0,
            extrapolated=not (fitted_bounds[0]<=target-model.origin_turns<=fitted_bounds[1] and fitted_bounds[0]<=snapshot.position_turns-model.origin_turns<=fitted_bounds[1])))
        self.update_terminal(f'Model target {angle_deg:g} deg -> {target:.6f} motor turns ({phase})\n')
        if getattr(self,'run_segmented_comparison',False) and abs(delta_angle)/speed > 30:
            # Bound every move without extending the global timeout. Stops at
            # intermediate positions are labelled; no rotary endpoint correction.
            import math
            count=math.ceil(abs(delta_angle)/speed/30)
            self.run_metadata['experimental_motion_model']['segments'][-1]['submoves']=count
            for i in range(1,count+1):
                subtarget=snapshot.position_turns+delta_turns*i/count
                self.command_position_and_wait(subtarget,phase,token,speed_deg_s,acceleration_deg_s2,distance_deg=abs(delta_angle)/count)
        else:
            self.command_position_and_wait(target,phase,token,speed_deg_s,acceleration_deg_s2, distance_deg=abs(delta_angle))
        return target

    def command_position_and_wait(
        self,
        target_turns,
        phase,
        token,
        speed_deg_s=None,
        acceleration_deg_s2=None,
        distance_deg=None,
        *,
        position_tolerance_turns=None,
    ):
        if self.test_stop_event.is_set():
            raise TestStopped()
        self.motion_coordinator.assert_active(token)
        if self.odrive_adapter is None:
            raise RuntimeError("ODrive disconnected during motion")
        self.motion_phase = phase
        snapshot = self._test_feedback()
        actual_distance_deg = abs(odrive_turns_to_afo_degrees(
            float(target_turns) - snapshot.position_turns, self.system_config
        ))
        if distance_deg is not None:
            actual_distance_deg = float(distance_deg)
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
        if position_tolerance_turns is not None:
            tolerance_turns = min(tolerance_turns, position_tolerance_turns)
        velocity_limit_turns_s = afo_speed_to_odrive_turns_s(
            self.system_config["reference"]["settle_velocity_limit_deg_s"],
            self.system_config,
        )
        self._write_runtime_checkpoint(
            MotionState.MOVING,
            snapshot,
            clean_shutdown=False,
            extra={"phase": phase, "target_turns": float(target_turns),
                   "actual_distance_deg": actual_distance_deg, "timeout_s": timeout_s},
        )
        self._submit_position(token, target_turns)
        try:
            wait_for_settle(
                self._test_feedback,
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

    def complete_rotary_zero_return(self, token):
        """Return directly from the final endpoint using the GUI homing policy."""
        from copy import deepcopy
        from east_rotary_homing import home, HomingPolicy, RotaryZeroMiss
        from east_zero_window import collect_stationary_window
        policy = HomingPolicy()
        deadline = time.monotonic() + policy.timeout_s
        source = self.rotary_sensor.measurement(require=True)['sample'][4]
        calibration = self.rotary_sensor.calibration.sha256
        factor = float(self.system_config['motion']['afo_degrees_per_odrive_turn'])
        cfg = self.system_config['reference']
        evidence = dict(target_angle_deg=0., tolerance_deg=policy.tolerance_deg,
            policy=policy.__dict__, corrections=[], attempts=[], status='running',
            method='direct_rotary_feedback',
            reference_changed=False, tare_changed=False)
        self.run_metadata['automatic_zero_return'] = evidence
        total_travel = 0.
        armed = True

        def check():
            self.motion_coordinator.assert_active(token)
            if self.test_stop_event.is_set() or self.operator_stop_requested:
                raise TestStopped()
            if self.acquisition_error or self.monitor_error or self.feedback_retry_pending:
                raise FeedbackError(self.acquisition_error or self.monitor_error or 'Feedback recovery pending')
            if time.monotonic() >= deadline:
                raise RuntimeError('Automatic rotary zero return timed out')
            self.reference_manager.require_verified()
            snapshot = self.get_feedback()
            expected = int(AXIS_STATE_CLOSED_LOOP_CONTROL if armed else AXIS_STATE_IDLE)
            if snapshot.current_state != expected or snapshot.active_errors:
                raise FeedbackError('Controller state/errors changed during automatic zero return')
            if armed and snapshot.disarm_reason != self.expected_disarm_reason:
                raise FeedbackError('Controller disarm reason changed during automatic zero return')
            value = self.rotary_sensor.measurement(require=True)
            if value['sample'][4] != source or self.rotary_sensor.calibration.sha256 != calibration:
                raise RotaryError('Rotary connection/calibration changed during automatic zero return')
            self._mark_control_health()
            return snapshot, value

        def idle():
            nonlocal armed
            check()
            result = self.safe_idle_motor('observing rotary zero after settling', token=token, retain_owner=True)
            armed = False
            if not result or not result.confirmed:
                raise FeedbackError('Automatic zero return could not confirm idle')
            check()

        def observe(duration):
            def read():
                snapshot, value = check()
                return dict(angle_deg=value['angle_deg'], sequence=value['sample'][3],
                    motor_turns=snapshot.position_turns,
                    stationary=abs(snapshot.velocity_turns_s*factor) <= cfg['settle_velocity_limit_deg_s'])
            return collect_stationary_window(read, duration, max(.01, deadline-time.monotonic()),
                position_tolerance_turns=afo_degrees_to_odrive_turns(
                    self.system_config['motion']['position_tolerance_deg'], self.system_config))

        def move(delta):
            nonlocal total_travel
            snapshot, value = check()
            if len(evidence['corrections']) >= policy.maximum_corrections or total_travel+abs(delta) > policy.maximum_travel_deg:
                raise RuntimeError('Automatic zero return correction budget exhausted')
            target = snapshot.position_turns + delta/factor
            event = dict(before_deg=value['angle_deg'], nominal_step_deg=delta, target_motor_turns=target)
            evidence['corrections'].append(event)
            total_travel += abs(delta)
            self.command_position_and_wait(target, 'zero_return_correction', token,
                cfg['recovery_speed_deg_s'], cfg['recovery_acceleration_deg_s2'],
                distance_deg=abs(delta), position_tolerance_turns=abs(delta / factor) / 4)
            event['after_deg'] = check()[1]['angle_deg']

        try:
            check()
            self.motion_phase = 'zero_return_correction'
            self.configure_trajectory(cfg['recovery_speed_deg_s'], cfg['recovery_acceleration_deg_s2'])
            self.update_terminal('Measurement cycles complete. Returning directly to zero using the rotary sensor.\n')
            while True:
                result = home(policy, observe, move, check)
                evidence['attempts'].append(result)
                idle()
                self.motion_phase = 'corrected_zero_observation'
                try:
                    self.observe_post_idle_neutral(self.get_feedback().position_turns)
                except RotaryZeroMiss:
                    passed = False
                else:
                    passed = True
                observation = self.run_metadata['zero_return_check']
                observation.pop('unassisted_angle_deg', None)
                observation['method'] = 'direct_rotary_feedback'
                evidence['final_observation'] = deepcopy(observation)
                if passed:
                    break
                check()
                self.enter_closed_loop(token)
                armed = True
                self.motion_phase = 'zero_return_correction'
            check()
            final_idle = self.safe_idle_motor('rotary zero verified; return complete', token=token)
            if not final_idle or not final_idle.confirmed:
                raise FeedbackError('Final zero return idle was not confirmed')
            evidence.update(status='completed', nominal_travel_deg=total_travel,
                idle_confirmed=True, correction_count=len(evidence['corrections']))
            self.update_terminal('Return complete: rotary zero stable within ±0.10°.\n')
        except Exception as exc:
            evidence.update(status='failed', error=str(exc), nominal_travel_deg=total_travel)
            raise

    def observe_post_idle_neutral(self, neutral_target):
        self.observe_idle_neutral(
            neutral_target,
            cancelled=lambda: self.test_stop_event.is_set()
            or self.acquisition_error is not None,
        )

    def observe_idle_neutral(self, neutral_target, cancelled=lambda: False, *, record_return=True):
        from east_rotary_homing import RotaryZeroMiss
        from east_zero_window import collect_stationary_window, ZERO_IDLE_SETTLE_S, ZERO_IDLE_TIMEOUT_S
        cfg=self.system_config['reference'];duration=cfg['post_idle_observation_ms']/1000.
        tolerance_turns=afo_degrees_to_odrive_turns(self.system_config['motion']['position_tolerance_deg'],self.system_config)
        rotary_required=self.strain_test_active
        factor=self.system_config['motion']['afo_degrees_per_odrive_turn']
        def read():
            if cancelled():raise TestStopped()
            snapshot=self.get_feedback()
            if snapshot.current_state!=int(AXIS_STATE_IDLE) or snapshot.active_errors:
                raise FeedbackError('Controller left error-free idle during stationary zero observation')
            if not rotary_required and abs(snapshot.position_turns-neutral_target)>tolerance_turns:
                raise RuntimeError('Machine-zero position drifted after ODrive entered idle')
            value=self.rotary_sensor.measurement(require=True) if rotary_required else None
            return dict(stationary=abs(snapshot.velocity_turns_s*factor)<=cfg['settle_velocity_limit_deg_s'],
                motor_turns=snapshot.position_turns,angle_deg=value['angle_deg'] if value else None,
                sequence=value['sample'][3] if value else None)
        def within_rotary_band(observation):
            return (observation['min_deg'] >= -ZERO_TOLERANCE_DEG
                    and observation['max_deg'] <= ZERO_TOLERANCE_DEG)

        settle_s = ZERO_IDLE_SETTLE_S if rotary_required else 0.0
        timeout = (max(ZERO_IDLE_TIMEOUT_S, duration + settle_s) if rotary_required
                   else cfg['idle_confirmation_timeout_s'] + duration)
        observation = collect_stationary_window(
            read, duration, timeout, settle_s=settle_s,
            position_tolerance_turns=tolerance_turns,
            accept_window=within_rotary_band if rotary_required else None,
        )
        if rotary_required:
            passed = within_rotary_band(observation)
            if not record_return:
                if not passed:
                    raise RotaryError(
                        "Pilot start zero did not remain within ±0.10° before the deadline "
                        f"(observed {observation['min_deg']:+.4f}° to {observation['max_deg']:+.4f}°)."
                    )
                return observation
            self.run_metadata['zero_return_check']=dict(passed=passed,unassisted_angle_deg=observation['mean_deg'],
                tolerance_deg=ZERO_TOLERANCE_DEG,phase='post_idle_stationary_window',
                window_policy='rolling_after_settling_grace',
                nominal_motor_target_turns=neutral_target,**observation)
            if not passed:
                raise RotaryZeroMiss(f"Model return missed rotary zero: mean {observation['mean_deg']:+.4f} degrees, "
                    f"observed range [{observation['min_deg']:+.4f}, {observation['max_deg']:+.4f}]; rotary homing required")

    def continuous_strain_read(self):
        acquisition = AcquisitionLoop(
            self.voltage_ratio_input.getVoltageRatio,
            lambda ratio: self.log_strain_data(ratio, self.current_cycle),
            self.test_stop_event,
            self.system_config["acquisition"]["sample_interval_ms"] / 1000.0,
        )
        try:
            acquisition.run()
        except Exception as exc:
            if isinstance(exc,RotaryError) and self.run_metadata is not None:
                self.run_metadata.setdefault('rotary_failure_evidence',exc.details)
            self.acquisition_error = f"data acquisition error: {exc}"
            self.update_terminal(f"Error reading strain data: {exc}\n")
            self._request_coordinated_stop(self.acquisition_error, fault=True)
        finally:
            self.acquisition_overruns = acquisition.overruns
            self.maximum_acquisition_lateness_s = acquisition.maximum_lateness_s
            recorder = getattr(self, "csv_recorder", None)
            if recorder is not None:
                try:
                    recorder.finish(timeout_s=3.0)
                except Exception as exc:
                    self.acquisition_error = f"CSV recorder finalization error: {exc}"
                    self._request_coordinated_stop(self.acquisition_error, fault=True)
                    self.update_terminal(f"SAFETY STOP: {self.acquisition_error}\n")
            elif self.strain_data_buffer:
                try:
                    with open(self.strain_file_name, mode="a", newline="", encoding="utf-8") as file:
                        writer = csv.writer(file)
                        writer.writerows(self.strain_data_buffer)
                        file.flush()
                        os.fsync(file.fileno())
                    self.strain_data_buffer = []
                except Exception as exc:
                    self.acquisition_error = f"CSV flush error: {exc}"
                    self._request_coordinated_stop(self.acquisition_error, fault=True)
                    self.update_terminal(f"SAFETY STOP: {self.acquisition_error}\n")
            self.acquisition_done_event.set()

    def finalize_run(self, status, error=None, idle_confirmed=False):
        with self.finalize_lock:
            if self.run_finalized or self.run_metadata is None:
                return
            operation = getattr(self, "stop_operation", None)
            if (operation is not None and operation["metadata_file"] == self.metadata_file_name
                    and operation["fault"]):
                status = "error"
                causes = "; ".join(operation["reasons"])
                error = f"{error}; {causes}" if error and causes not in error else error or causes
                if self.stop_worker is not None and self.stop_worker.is_alive():
                    idle_confirmed = False
            if getattr(self, "isolated_exit_requested", False):
                status = "error"
                error = (f"{error}; power-isolated exit" if error else "power-isolated exit")
                idle_confirmed = False
            self.run_metadata["run_status"] = status
            self.run_metadata["completed_at"] = datetime.now().astimezone().isoformat(timespec="milliseconds")
            self.run_metadata["sample_count"] = self.sample_count
            self.run_metadata["completed_cycles"] = self.completed_cycles
            self.run_metadata["error"] = error
            self.run_metadata["outcome_pending_cleanup"] = (
                not idle_confirmed or self.motion_coordinator.owner is not None
            )
            self.run_metadata["acquisition_overruns"] = self.acquisition_overruns
            self.run_metadata["maximum_acquisition_lateness_s"] = self.maximum_acquisition_lateness_s
            recorder = getattr(self, "csv_recorder", None)
            service = getattr(self, "feedback_service", None)
            self.run_metadata["motor_feedback_connection_statistics"] = service.statistics() if service else None
            window=getattr(self,'run_feedback_window',None)
            self.run_metadata["motor_feedback_statistics"] = (service.finish_run_statistics(window)
                if service is not None and window is not None else self.run_metadata['motor_feedback_connection_statistics'])
            engine = getattr(self, "engine", None)
            if engine is not None and hasattr(engine, "evidence"):
                self.run_metadata["engine_evidence"] = dict(engine.evidence)
            self.run_metadata["last_idle_observation"] = getattr(self, "last_idle_observation", None)
            self.run_metadata["gui_timing"] = dict(getattr(self, "ui_timing", {}))
            self.run_metadata["csv_recorder"] = {
                "rows_persisted": recorder.written_rows if recorder else None,
                "pending_rows": recorder.pending.qsize() if recorder else None,
                "error": str(recorder.error) if recorder and recorder.error else None,
                "worker_complete": recorder.done.is_set() if recorder else None,
            }
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
            self.run_metadata["odrive_final_configuration_error"] = (
                "Live diagnostic omitted during finalization to avoid blocking run outcome"
            )
            try:
                write_json_atomic(Path(self.metadata_file_name), self.run_metadata)
                self.run_finalized = True
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
            if self.run_finalized and self.motion_coordinator.motor_idle_confirmed and self.motion_coordinator.owner is None:
                self.ui_message_queue.put(("inputs", "normal"))
                self.ui_message_queue.put(("run_buttons", "normal"))

    def stop_strain_test(self):
        """Stop the strain test"""
        if self.strain_test_active:
            self.test_stop_event.set()
            self.update_terminal("Stopping strain test...\n")
        else:
            self.update_terminal("No strain test active\n")

    def save_stationary_diagnostics(self):
        tab = self.output_tabs.get()
        gui = dict(self.ui_timing)
        motor = list(self.monitor_timings)
        rotary = self.rotary_sensor.measurement()
        rotary_summary = {k: v for k, v in rotary.items() if k != "sample"}
        path = self.state_store.directory / ("stationary_" + datetime.now().strftime("%Y%m%dT%H%M%S") + ".json")
        def save():
            write_json_atomic(path, {"mode": "GUI observations / hardware verification NOT TESTED",
                "output_tab": tab, "gui": gui, "motor": motor, "rotary": rotary_summary,
                "engine_state": self.engine.lifecycle.value})
            self.update_terminal(f"Stationary diagnostic observations saved: {path}\n")
        self._submit_lifecycle("save diagnostics", save)

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

        plot_curve = draw_operator_plot(canvas, self.plot_title, angle_data, torque_data)

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

    def update_plot_data(self, angle, torque, redraw=True):
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
                if redraw:
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

    def _start_calibration_step(self, window, direction, requested, units):
        """Discrete calibration-only positioning; never changes machine zero/tare."""
        try:
            if self.engine.busy is not None:
                raise ValueError("Wait for hardware lifecycle operation")
            if window is not self.rotary_calibration or window.closed or window.closing:
                raise ValueError("Open the rotary calibration window first")
            if self.strain_test_active or self.startup_block_reason:
                raise ValueError("Finish the active test and resolve startup faults")
            reference = window.positioning_reference
            if reference is None:
                raise ValueError("Confirm the current calibration positioning angle first")
            snapshot = reference.check(self, stationary=True)
            step_turns = manual_step_to_turns(requested, units, self.system_config)
            reference.target(snapshot.position_turns, direction, step_turns)
            self.operator_context = {"operator": self.operator_input.get().strip(), "fixture": self.fixture_id_input.get().strip()}
            token = self.motion_coordinator.acquire("rotary-calibration-step")
        except Exception as exc:
            self.update_terminal(f"Calibration step blocked: {exc}\n")
            return
        self._refresh_motion_controls()
        self.manual_step_thread = threading.Thread(
            target=self._manual_step_worker,
            args=(token, direction, step_turns, float(requested), units, reference),
            name="rotary-calibration-step", daemon=True,
        )
        self.manual_step_thread.start()

    def _manual_step_worker(self, token, direction, step_turns, requested, units, calibration_reference=None):
        try:
            self._validate_live_motion_config()
            self._revalidate_hardware_identity(require_reference=calibration_reference is None)
            motion = self.system_config["motion"]
            current = self.get_feedback().position_turns
            target = current + direction * step_turns
            if calibration_reference is not None:
                calibration_reference.check(self, stationary=True)
                bounded = calibration_reference.target(current, direction, step_turns)
            else:
                bounded = self.clamp_manual_target(target)
            if not math.isclose(target, bounded, rel_tol=0.0, abs_tol=1e-10):
                raise ValueError("Full requested step exceeds travel limits; reduce the step. No movement commanded.")
            distance_deg = abs(odrive_turns_to_afo_degrees(step_turns, self.system_config))
            speed = (self.system_config["reference"]["recovery_speed_deg_s"]
                     if calibration_reference is not None else motion["manual_speed_deg_s"])
            acceleration = (self.system_config["reference"]["recovery_acceleration_deg_s2"]
                            if calibration_reference is not None else motion["manual_acceleration_deg_s2"])
            timeout_s = motion_timeout_seconds(distance_deg, speed, acceleration, self.system_config)
            self.configure_trajectory(speed, acceleration)
            if calibration_reference is not None:
                self.enter_closed_loop(token, allow_unreferenced=True)
                calibration_reference.check(self)
            else:
                self.enter_closed_loop(token)
            snapshot = self.get_feedback()
            self._write_runtime_checkpoint(
                MotionState.MOVING, snapshot, clean_shutdown=False,
                extra={"phase": "rotary_calibration_step" if calibration_reference is not None else "manual_step",
                       "target_turns": target, "requested_step": direction * requested, "step_units": units,
                       "calibration_only": calibration_reference is not None},
            )
            self._submit_position(token, target)
            wait_for_settle(
                (lambda: calibration_reference.check(self)) if calibration_reference is not None else self.get_feedback, target,
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
            if calibration_reference is not None:
                try:
                    calibration_reference.last_position_turns = calibration_reference.check(self, stationary=True).position_turns
                except Exception:
                    calibration_reference.suspend()
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
                self._write_runtime_checkpoint(
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
            self._request_coordinated_stop("continuous manual movement stop requested")

    def prepare_manual_motion(self):
        if self.engine.busy is not None:
            raise RuntimeError("Wait for hardware lifecycle operation")
        if self.motion_coordinator.shutting_down:
            raise RuntimeError("Manual motion blocked during shutdown")
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
        return self.open_rotary_homing()


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
            self._request_coordinated_stop("manual mode disabled")
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
