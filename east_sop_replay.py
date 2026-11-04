"""Offline SOP capture from a recorded three-speed manifest and strain CSVs."""
from copy import deepcopy
import csv
import json
import math
from pathlib import Path, PureWindowsPath
import subprocess
from datetime import datetime
import tkinter as tk
from tkinter import filedialog, messagebox

import customtkinter as ctk

from east_operator_ui import APP_NAME, APP_VERSION, AMBER, GREEN, PANEL, RED, OperatorView, draw_operator_plot
from east_motion_model import ORIGINAL_METHOD
from east_protocol_batch import make_protocol_plan
from east_protocol_batch_gui import ThreeSpeedProtocolWindow


def read_recording(path):
    manifest = json.loads(path.read_text(encoding="utf-8"))
    runs = []
    for run in manifest["runs"]:
        filename = run.get("strain_file")
        if not filename:
            continue
        points = []
        with (path.parent / PureWindowsPath(filename).name).open(
            encoding="utf-8-sig", newline=""
        ) as stream:
            for row in csv.DictReader(stream):
                try:
                    angle = float(row["Measured Fixture Angle (deg)"])
                    torque = float(row["Moving Avg Torque (Nm)"])
                except (KeyError, ValueError, TypeError):
                    continue
                if math.isfinite(angle) and math.isfinite(torque):
                    points.append((angle, torque))
        if points:
            runs.append((run, points))
    if not runs:
        raise ValueError("No plottable strain records beside this manifest")
    return manifest, runs


class ReplayWindow(OperatorView):
    """The operator widgets backed by a local recording and Tk-only actions."""

    def __init__(self, root, path, config, output):
        self.master = self.root = root
        self.output, self.path = output, path
        self.manifest, self.runs = read_recording(path)
        self.system_config = deepcopy(config)
        self.header_images = []
        self.connected = False
        self.dialog = None
        self.batch = None
        self.manual_mode = ctk.BooleanVar(value=False)
        self.manual_step_units = ctk.StringVar(value="Degrees")
        self.motion_method_var = ctk.StringVar(value=ORIGINAL_METHOD)
        self.protocol_display_to_key = {
            "Custom": "custom",
            **{p["display_name"]: key for key, p in config["test_presets"].items()},
        }
        self.loaded_preset_key = (
            "three_speed_empty_baseline"
            if self.manifest["test_type"] == "empty_machine_baseline"
            else "three_speed_afo"
        )
        self.preset = config["test_presets"][self.loaded_preset_key]
        self.protocol_var = ctk.StringVar(value=self.preset["display_name"])
        root.title(APP_NAME)
        height = min(1050, root.winfo_screenheight() - 90)
        ctk.set_widget_scaling(min(1.0, height / 1000))
        root.geometry(f"1100x{height}")
        root.minsize(900, 650)
        tools = ctk.CTkFrame(root)
        tools.pack(fill="x", padx=12, pady=8)
        ctk.CTkLabel(tools, text="Offline preview").pack(side="left", padx=8)
        self.views = ("Setup", "Confirmation", "Recorded plot", "Recorded outcome")
        self.view = ctk.CTkOptionMenu(tools, values=self.views, command=self.render)
        self.view.pack(side="left", padx=8, pady=6)
        self.run = ctk.CTkOptionMenu(
            tools, values=[str(i + 1) for i in range(len(self.runs))],
            command=lambda _: self.render(self.view.get()),
        )
        self.run.pack(side="left", padx=8)
        ctk.CTkButton(tools, text="Capture PNG", command=self.capture).pack(side="right", padx=8)
        ctk.CTkButton(tools, text="Capture all views", command=self.capture_all).pack(side="right", padx=8)
        self.setup_ui()
        self.frame = self.operator_shell
        self.connect_system()
        self.view.set("Recorded plot")
        self.render("Recorded plot")

    def on_close(self):
        self.root.destroy()

    def connect_system(self):
        self.connected = not self.connected
        self.render(self.view.get())

    def load_selected_preset(self):
        key = self.protocol_display_to_key[self.protocol_var.get()]
        if key in ("three_speed_afo", "three_speed_empty_baseline"):
            self.loaded_preset_key = key
            self.preset = self.system_config["test_presets"][key]
            self.render("Setup")
        else:
            self.update_protocol_status()

    def update_terminal(self, message):
        self.terminal.insert("end", message)
        self.terminal.see("end")

    def create_plot_window(self, *, select_plot=True):
        self.plot_canvas = tk.Canvas(self.plot_container, background=PANEL, highlightthickness=0)
        self.plot_canvas.pack(fill="both", expand=True, padx=4, pady=4)
        self.plot_canvas.bind("<Configure>", lambda _: self.update_plot())
        if select_plot:
            self.output_tabs.set("Plot")

    def update_plot(self):
        if self.output_tabs.get() != "Plot":
            return
        points = self.runs[int(self.run.get()) - 1][1] if self.connected else []
        angles, torques = zip(*points[-10000:]) if points else ([], [])
        title = self.file_name_input.get() or "AFO Strain Test"
        draw_operator_plot(self.plot_canvas, title, angles, torques)

    def start_strain_test(self):
        if self.connected:
            self.view.set("Confirmation")
            self.render("Confirmation")

    def stop_logging(self):
        self.view.set("Setup")
        self.render("Setup")

    def offline_control(self, *_):
        pass

    _manual_step_unit_changed = _on_tare_identity_edit = _update_manual_step_hint = offline_control
    validate_angle_input = validate_step_angle = select_motion_method = offline_control
    begin_continuous_movement = stop_continuous_movement = offline_control
    move_motor_left = move_motor_right = toggle_manual_mode = offline_control
    open_motion_mapping = open_rotary_calibration = open_rotary_homing = offline_control
    tare_empty_machine = clear_session_tare = exit_after_motor_power_is_isolated = offline_control
    reset_test_fields = offline_control
    _on_output_tab_changed = update_plot

    def render(self, view):
        for window in (self.dialog, self.batch):
            if window is not None and window.winfo_exists():
                window.destroy()
        self.dialog = self.batch = None
        self.frame = self.operator_shell
        manifest = self.manifest
        run, points = self.runs[int(self.run.get()) - 1]
        identity = manifest["identity"]
        context = run["context"]
        recorded = view in self.views[2:]
        params = run.get("parameters") or {}
        speed = context["planned_speed_deg_s"] if recorded else self.preset["speeds_deg_s"][0]
        prefix = identity.get("file_prefix") or manifest["batch_id"][:8]
        for entry, value in (
            (self.file_name_input, prefix),
            (self.afo_id_input, "EMPTY_MACHINE_BASELINE" if manifest["test_type"] ==
             "empty_machine_baseline" else identity.get("afo_id", "")),
            (self.operator_input, identity.get("operator", "")),
            (self.fixture_id_input, identity.get("fixture_id", "")),
            (self.calibration_id_input, identity.get("calibration_id", "")),
            (self.speed_input, speed),
            (self.cycles_input, manifest["cycles_per_speed"] if recorded else self.preset["cycles"]),
            (self.min_angle_input, params.get("min_angle_deg", self.preset["minimum_angle_deg"]) if recorded else self.preset["minimum_angle_deg"]),
            (self.max_angle_input, params.get("max_angle_deg", self.preset["maximum_angle_deg"]) if recorded else self.preset["maximum_angle_deg"]),
            (self.acceleration_input, params.get("commanded_afo_acceleration_deg_s2", self.preset["acceleration_deg_s2"]) if recorded else self.preset["acceleration_deg_s2"]),
        ):
            entry.configure(state="normal")
            entry.delete(0, "end")
            entry.insert(0, str(value))
        self.homing_condition.set(
            "Empty fixture" if manifest["test_type"] == "empty_machine_baseline" else "AFO fitted"
        )
        self.update_protocol_status()
        if recorded:
            name = manifest.get("protocol", {}).get("preset_display_name", self.preset["display_name"])
            self.protocol_status_label.configure(text=f"Active protocol: {name}")
        ready = self.connected
        self.buttons[1].configure(state="normal" if ready else "disabled")
        self.tare_button.configure(state="normal" if ready else "disabled")
        self.reference_action_button.configure(state="normal" if ready else "disabled")
        self.reference_status_label.configure(
            text="Rotary zero: VERIFIED" if ready else "Rotary zero: HOME REQUIRED",
            text_color=GREEN if ready else AMBER,
        )
        self.tare_status_label.configure(
            text="Empty-machine tare: VALID" if ready else "Empty-machine tare: REQUIRED",
            text_color=GREEN if ready else AMBER,
        )
        self.status_label.configure(
            text="CONNECTED / MACHINE ZERO VERIFIED" if ready else "DISCONNECTED",
            text_color=GREEN if ready else RED,
        )
        self.run_timing_label.configure(text="Elapsed / estimated remaining: shown when a test starts")
        self.terminal.delete("1.0", "end")
        self.update_terminal(f"{APP_NAME} {APP_VERSION}\n")
        if ready:
            tare = manifest.get("unloaded_tare") or {}
            if "offset_v_per_v" in tare and "captured_at" in tare:
                self.tare_status_label.configure(
                    text=f"Empty-machine tare: VALID\n{tare['captured_at']} | offset {tare['offset_v_per_v']:.12g} V/V"
                )
            self.update_terminal("Connected. Rotary zero verified.\nEmpty-machine tare: VALID\n")
        self.output_tabs.set("Session Terminal")
        if ready and view == "Confirmation":
            batch = PreviewBatch(self.master, self, self.loaded_preset_key)
            batch.window.withdraw()
            requests = make_protocol_plan(
                self.system_config, self.loaded_preset_key, batch.current_identity(),
                self._motion_field_values(), manifest["mounting_setup_id"], "",
                self.path, "", batch_id=manifest["batch_id"],
            )
            self.dialog, _ = self.build_confirmation(
                "Confirm Three-Speed Protocol", batch.confirmation_message(requests),
                "Run Sequence", False, {"confirmed": False},
            )
            batch.window.destroy()
            self.frame = self.dialog
        elif ready and view == "Recorded outcome":
            batch = PreviewBatch(self.master, self, self.loaded_preset_key)
            self.batch = batch.window
            outcome = manifest["outcome"]
            batch.status.configure(text=f"{outcome.upper()}: {manifest.get('error') or 'all runs saved'}")
            batch.fields["mounting_setup_id"].delete(0, "end")
            batch.fields["mounting_setup_id"].insert(0, manifest["mounting_setup_id"])
            for record in manifest["runs"]:
                ctx = record.get("context", {})
                batch.append_terminal(
                    f"{ctx.get('run_id', 'Run')} | {ctx.get('planned_speed_deg_s', '—')}°/s | "
                    f"{record.get('completed_cycles') or 0} cycles | {record.get('status', 'unknown')}\n"
                )
            batch.output.configure(text=f"Batch manifest: {self.path.name}")
            self.frame = self.batch
        elif ready and view == "Recorded plot":
            self.status_label.configure(text="TEST RUNNING", text_color=GREEN)
            self.run_timing_label.configure(
                text=f"{context['run_id']} ({context['run_index']}/{context['total_runs']}) | "
                     f"{speed:g}°/s | {run.get('completed_cycles') or 0} cycles"
            )
            self.update_terminal(f"{context['run_id']} | {speed:g}°/s | Fixture angle: {points[-1][0]:.2f}°\n")
            self.output_tabs.set("Plot")
            self.update_plot()


    def capture_all(self):
        initial_view = self.view.get()
        paths = []
        views = iter(self.views)

        def next_view():
            view = next(views, None)
            if view is None:
                finish()
                return
            self.view.set(view)
            self.render(view)
            self.root.after(300, save_view)

        def save_view():
            path = self.capture(report=False)
            if path is None:
                finish()
                return
            paths.append(path)
            next_view()

        def finish():
            self.view.set(initial_view)
            self.render(initial_view)
            if paths:
                messagebox.showinfo("Saved screenshots", f"{len(paths)} PNGs in {self.output}")

        next_view()

    def capture(self, report=True):
        self.root.update_idletasks()
        self.frame.winfo_toplevel().lift()
        self.root.update_idletasks()
        frame = self.frame
        rect = ",".join(str(n) for n in (frame.winfo_rootx(), frame.winfo_rooty(),
                                        frame.winfo_width(), frame.winfo_height()))
        self.output.mkdir(parents=True, exist_ok=True)
        state = self.view.get().lower().replace(" ", "-")
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
        path = self.output / f"{self.manifest['batch_id'][:8]}-{state}-{stamp}.png"
        try:
            subprocess.run(["/usr/sbin/screencapture", "-x", "-R", rect, str(path)], check=True)
        except (OSError, subprocess.CalledProcessError) as error:
            messagebox.showerror("Capture failed", str(error))
            return
        if path.exists():
            if report:
                messagebox.showinfo("Saved screenshot", str(path))
            return path
        else:
            messagebox.showerror("Capture failed", "Check macOS Screen Recording access for your launcher.")


class PreviewBatch(ThreeSpeedProtocolWindow):
    """Reuse the real batch layout with inert execution callbacks."""
    tick = start = stop = ReplayWindow.offline_control


def launch(root, repo):
    ctk.set_default_color_theme("blue")
    ctk.set_appearance_mode("light")
    path = filedialog.askopenfilename(parent=root, title="Select a recorded batch manifest",
                                      initialdir=repo / "EAST Logs", filetypes=[("Batch JSON", "*.json")])
    if not path:
        root.destroy()
        return
    try:
        config = json.loads((repo / "tester_config.json").read_text(encoding="utf-8"))
        ReplayWindow(root, Path(path), config, repo / "SOP Screenshots")
    except (OSError, ValueError, KeyError, TypeError) as error:
        messagebox.showerror("Cannot load recording", str(error), parent=root)
        root.destroy()
