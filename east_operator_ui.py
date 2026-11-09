"""Shared Tk presentation used by EAST and the offline SOP preview."""
from pathlib import Path
import sys
import tkinter as tk

import customtkinter as ctk
from PIL import Image

from east_core import FITTED_SPEED_SCREEN, preset_values_modified
from east_gui_tabs import OutputTabs
from east_motion_model import ORIGINAL_METHOD, UPDATED_METHOD, WIDER_METHOD, selected_method

APP_NAME = "EAST"
APP_VERSION = "1.17.0-reliability-audit"

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

class OperatorView:
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

        shell = self.operator_shell = ctk.CTkFrame(self.master, fg_color=BG)
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

        status_panel=ctk.CTkFrame(controls,fg_color="transparent")
        status_panel.grid(row=0,column=0,sticky="ew",padx=6,pady=2)
        status_panel.grid_columnconfigure(0,weight=1)
        self.status_label = ctk.CTkLabel(
            status_panel,
            text="DISCONNECTED",
            text_color=RED,
            font=("Arial", 13, "bold"),
            anchor="w",
        )
        self.status_label.grid(row=0, column=0, sticky="ew", padx=12, pady=(8, 4))

        self.run_timing_label=ctk.CTkLabel(status_panel,text="Elapsed / estimated remaining: shown when a test starts",
            font=("Arial",11),anchor="w",justify="left",wraplength=390)
        self.run_timing_label.grid(row=1,column=0,sticky="ew",padx=12,pady=(0,6))

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
        method_label = ctk.CTkLabel(protocol_frame, text="Motion method:", font=("Arial", 11, "bold"))
        method_label.grid(row=2,column=0,padx=(0,6),sticky="w")
        self.motion_method_menu = ctk.CTkOptionMenu(
            protocol_frame,variable=self.motion_method_var,values=[ORIGINAL_METHOD, UPDATED_METHOD, WIDER_METHOD],
            command=self.select_motion_method,width=205,height=30)
        self.motion_method_menu.grid(row=2,column=1,columnspan=3,sticky='ew',pady=(5,3))
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
                button.grid(row=1, column=0, columnspan=2, padx=5, pady=3, sticky="ew")
            else:
                button.grid(row=0, column=index, padx=5, pady=3, sticky="ew")
            self.buttons.append(button)
        self.isolated_exit_button = ctk.CTkButton(
            button_frame, text="Exit After Motor Power Is Isolated",
            command=self.exit_after_motor_power_is_isolated,
            fg_color=AMBER, hover_color="#b45309", height=34,
        )
        self.isolated_exit_button.grid(row=2, column=0, columnspan=2, padx=5, pady=3, sticky="ew")
        self.isolated_exit_button.grid_remove()
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
        self.reference_status_label = ctk.CTkLabel(
            machine_zero_frame,
            text="Rotary zero: HOME REQUIRED",
            font=("Arial", 11, "bold"),
            text_color=AMBER,
            anchor="w",
            justify="left",
            wraplength=390,
        )
        self.reference_status_label.grid(
            row=0, column=0, columnspan=2, padx=8, pady=(8, 5), sticky="ew"
        )
        self.reference_action_button = ctk.CTkButton(
            machine_zero_frame,
            text="Check Zero / Home",
            command=self.open_rotary_homing,
            fg_color=AMBER,
            hover_color="#b45309",
            state="disabled",
            height=34,
        )
        self.reference_action_button.grid(row=1, column=0, columnspan=2, padx=8, pady=(0, 5), sticky="ew")

        self.neutral_button = self.reference_action_button
        # Older code retains a neutral_button handle, but the main panel has one
        # rotary homing action rather than a second encoder-only return method.
        self.homing_condition = ctk.StringVar(value="AFO fitted")
        ctk.CTkOptionMenu(machine_zero_frame, variable=self.homing_condition,
            values=["Empty fixture", "AFO fitted"]).grid(row=2,column=0,columnspan=2,
                padx=8,pady=4,sticky="ew")
        self.homing_acknowledged = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(machine_zero_frame, text="Allow centring movement: fixture/AFO clear, sensor unchanged, E-stop accessible",
            variable=self.homing_acknowledged, font=("Arial",10)).grid(row=3,column=0,
                columnspan=2,padx=8,pady=4,sticky="ew")
        self.homing_live_label = ctk.CTkLabel(machine_zero_frame,
            text="Sensor zero band: ±0.10° | 0.5 s settling + post-idle check",wraplength=390)
        self.homing_live_label.grid(row=4,column=0,columnspan=2,padx=8,pady=(4,8),sticky="ew")

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

        self.motion_mapping_button = ctk.CTkButton(
            manual_control_frame, text="Motion Modelling", command=self.open_motion_mapping,
            fg_color=BLUE, hover_color="#1d4ed8", height=34,
        )
        self.motion_mapping_button.grid(
            row=7, column=0, columnspan=3, padx=6, pady=(0, 8), sticky="ew"
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

        engineering_widgets = (
            method_label, self.motion_method_menu, self.manual_step_units_menu,
            self.step_angle_input, self.manual_mode_toggle, self.left_arrow,
            self.right_arrow, self.mode_toggle, self.manual_step_hint,
            self.manual_position_label, self.rotary_calibration_button,
            self.motion_mapping_button, self.clear_session_tare_button,
        )
        routine_presets = ["Custom", *(
            preset["display_name"] for preset in self.system_config["test_presets"].values()
            if not preset.get("protocol_kind") and not preset.get("hidden_in_menu", False)
        )]
        show_engineering = self.show_engineering = ctk.BooleanVar(value=False)

        def update_engineering_view():
            visible = show_engineering.get()
            selected = self.protocol_var.get()
            loaded = self._loaded_preset()
            if not visible and (
                self.manual_mode.get() or selected not in routine_presets
                or (loaded and loaded.get("protocol_kind"))
            ):
                show_engineering.set(True)
                return
            for widget in engineering_widgets:
                if visible:
                    widget.grid()
                else:
                    widget.grid_remove()
            self.protocol_menu.configure(
                values=list(self.protocol_display_to_key) if visible else routine_presets
            )

        ctk.CTkSwitch(
            manual_control_frame, text="Engineering controls",
            variable=show_engineering, command=update_engineering_view,
        ).grid(row=8, column=0, columnspan=3, sticky="w", padx=8, pady=8)
        update_engineering_view()

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


    def _loaded_preset(self):
        if self.loaded_preset_key == "custom":
            return None
        return self.system_config["test_presets"][self.loaded_preset_key]


    def active_protocol_name(self):
        preset = self._loaded_preset()
        if preset is None:
            return "Custom"
        name = f"{preset['display_name']} v{preset['version']}"
        if preset.get('protocol_kind') == 'remaining_motion_data':
            return name + ' — 10 staged empty-machine runs; +/-10 repeats, +/-15 identification, stationary holds'
        if preset.get('protocol_kind') == 'rotary_feedback_pilot':
            count = len(preset['speeds_deg_s'])
            if self.loaded_preset_key == "feedback_commissioning_sequence":
                return f'{name} — {count // 2} stages, feedforward/feedback at each'
            return f'{name} — {count} planned runs at ±{preset["maximum_angle_deg"]:g}°, {preset["speeds_deg_s"][0]:g}°/s'
        if self.loaded_preset_key == FITTED_SPEED_SCREEN:
            return name + ' — 15 runs, 5 cycles each at ±5°, ±10° and ±15°'
        if preset.get('execution_kind') == 'speed_sequence':
            speeds = preset['speeds_deg_s']
            name += ' — ' + ' → '.join(f'{speed:g}' for speed in speeds) + '°/s; cycles EACH speed'
            try:
                cycles = int(self.cycles_input.get())
                name += f' ({len(speeds)*cycles} total numbered cycles)'
            except (TypeError,ValueError):
                pass
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
        text += f" | {selected_method(self.system_config)}"
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


    def _motion_field_values(self):
        return {
            "cycles": self.cycles_input.get(),
            "minimum_angle_deg": self.min_angle_input.get(),
            "maximum_angle_deg": self.max_angle_input.get(),
            "speed_deg_s": self.speed_input.get(),
            "acceleration_deg_s2": self.acceleration_input.get(),
        }


    def update_parameter_summary(self, _event=None):
        """Keep active/pending protocol status current without a duplicate summary."""
        self.update_protocol_status()


    def show_resizable_confirmation(self, title, message, confirm_text="Start", dangerous=False):
        """Show a modal confirmation whose actions remain visible on small screens."""
        result = {"confirmed": False}
        dialog, focus = self.build_confirmation(
            title, message, confirm_text, dangerous, result,
        )
        dialog.update_idletasks()
        dialog.grab_set()
        focus.focus_set()
        self.master.wait_window(dialog)
        return result["confirmed"]

    def build_confirmation(self, title, message, confirm_text, dangerous, result):
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

        cancel_button = ctk.CTkButton(
            actions,
            text="Cancel",
            command=lambda: finish(False),
            fg_color=MUTED,
            hover_color="#475569",
            height=38,
        )
        cancel_button.grid(row=0, column=0, sticky="ew", padx=(0, 6))
        start_button = ctk.CTkButton(
            actions,
            text=confirm_text,
            command=lambda: finish(True),
            fg_color=RED if dangerous else GREEN,
            hover_color="#b91c1c" if dangerous else "#15803d",
            height=38,
        )
        start_button.grid(row=0, column=1, sticky="ew", padx=(6, 0))

        dialog.protocol("WM_DELETE_WINDOW", lambda: finish(False))
        dialog.bind("<Escape>", lambda _event: finish(False))
        if not dangerous:
            dialog.bind("<Return>", lambda _event: finish(True))
        return dialog, cancel_button if dangerous else start_button


def draw_operator_plot(canvas, title, angle_data, torque_data):
    canvas.delete("plot")
    width = max(canvas.winfo_width(), 300)
    height = max(canvas.winfo_height(), 240)
    left, right, top, bottom = 68, 16, 50, 64
    x0, x1 = left, width - right
    y0, y1 = top, height - bottom

    canvas.create_text(
        width / 2, 24, text=title, fill=TEXT,
        font=("Arial", 13, "bold"), tags="plot",
    )
    canvas.create_text(
        width / 2, height - 22,
        text="Measured Fixture Angle (degrees)", fill=TEXT,
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
        step = max(1, (len(angle_data)+max_render_points-1) // max_render_points)
        samples = list(zip(angle_data[::step], torque_data[::step]))
        if samples[-1] != (angle_data[-1], torque_data[-1]):
            samples.append((angle_data[-1], torque_data[-1]))
        coordinates = []
        for angle, torque in samples:
            x = x0 + (angle - x_min) / (x_max - x_min) * (x1 - x0)
            y = y1 - (torque - y_min) / (y_max - y_min) * (y1 - y0)
            coordinates.extend((x, y))
        return canvas.create_line(
            *coordinates, fill=BLUE, width=2, tags="plot"
        )
