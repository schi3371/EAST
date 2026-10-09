"""Mac-safe EAST GUI layout preview.

This script does not import ODrive, Phidget, PyQtGraph, pywinstyles, or the
main hardware-control GUI. It is only for checking EAST branding, window size,
lab-laptop layout, logos, and optional graph-window behaviour on a Mac.
"""

from __future__ import annotations

import math
import json
import sys
import tkinter as tk
from datetime import datetime
from pathlib import Path
from tkinter import messagebox

import customtkinter as ctk
from PIL import Image


APP_NAME = "EAST"
APP_VERSION = "1.4.1-manual-motor-turns-preview"
ROOT_DIR = Path(__file__).resolve().parents[1]
IMAGE_DIR = ROOT_DIR / "images"

BG = "#f8fafc"
PANEL = "#ffffff"
PANEL_SOFT = "#eef2f7"
PANEL_DARK = "#f1f5f9"
TEXT = "#0f172a"
MUTED = "#64748b"
GOLD = "#2563eb"
GREEN = "#16a34a"
BLUE = "#2563eb"
RED = "#dc2626"
AMBER = "#d97706"


class EastGuiPreview:
    def __init__(self, root: ctk.CTk) -> None:
        self.root = root
        self.root.title(f"{APP_NAME} {APP_VERSION}")
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        window_width = min(1040, max(860, screen_width - 80))
        window_height = int(min(700, max(620, screen_height - 100)) * 0.8)
        self.root.geometry(f"{window_width}x{window_height}")
        self.root.minsize(860, 496)
        self.root.resizable(True, True)
        self.root.configure(fg_color=BG)

        self.logo_images: list[ctk.CTkImage] = []
        self.plot_window: ctk.CTkToplevel | None = None
        self.floating_canvas: tk.Canvas | None = None
        self.connected = False
        self.reference_verified = False
        self.tare_valid = False
        self.tare_fixture_id: str | None = None
        self.tare_calibration_id: str | None = None
        self.config = json.loads((ROOT_DIR / "tester_config.json").read_text(encoding="utf-8"))
        self.protocol_display_to_key = {
            "Custom": "custom",
            **{
                preset["display_name"]: key
                for key, preset in self.config["test_presets"].items()
            },
        }
        self.protocol_var = ctk.StringVar(value="Custom")
        self.loaded_preset_key = "custom"
        self.preset_loaded_at = datetime.now().astimezone().isoformat(
            timespec="milliseconds"
        )

        ctk.set_appearance_mode("light")
        ctk.set_default_color_theme("blue")

        self._build_ui()
        self._log(f"{APP_NAME} {APP_VERSION}")
        self._log("Preview only. No hardware modules are imported.")
        self._log(f"Running from: {Path(__file__).resolve()}")
        if "--scroll-bottom" in sys.argv:
            self.root.after(250, lambda: self.controls._parent_canvas.yview_moveto(1.0))
        if "--no-auto-plot" not in sys.argv:
            self.root.after(250, self.show_plot)

    def _build_ui(self) -> None:
        shell = ctk.CTkFrame(self.root, fg_color=BG)
        shell.pack(fill="both", expand=True, padx=18, pady=8)
        shell.grid_columnconfigure(0, weight=1)
        shell.grid_rowconfigure(1, weight=1)

        self._build_header(shell)
        self._build_body(shell)

    def _build_header(self, parent: ctk.CTkFrame) -> None:
        header = ctk.CTkFrame(parent, fg_color=BG)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 6))
        header.grid_columnconfigure(0, weight=1)
        header.grid_columnconfigure(1, weight=2)
        header.grid_columnconfigure(2, weight=1)

        self._logo_card(
            header,
            ("epic_lab_logo.png", "epic_lab_logo.jpg", "EPIC_Lab_logo.png", "EPIC Lab Logo.png"),
            "EPIC Lab",
            0,
            width=220,
        )

        title_panel = ctk.CTkFrame(header, fg_color=BG)
        title_panel.grid(row=0, column=1, sticky="nsew")
        ctk.CTkLabel(
            title_panel,
            text=APP_NAME,
            font=("Arial", 40, "bold"),
            text_color=TEXT,
        ).pack()
        ctk.CTkLabel(
            title_panel,
            text=f"{APP_VERSION} | lab laptop layout",
            font=("Arial", 13, "bold"),
            text_color=MUTED,
        ).pack()

        self._logo_card(
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
            width=260,
        )

    def _build_body(self, parent: ctk.CTkFrame) -> None:
        body = ctk.CTkFrame(parent, fg_color=BG)
        body.grid(row=1, column=0, sticky="nsew")
        body.grid_columnconfigure((0, 1), weight=1, uniform="main_body")
        body.grid_rowconfigure(0, weight=1)

        controls = ctk.CTkScrollableFrame(
            body,
            fg_color=PANEL,
            corner_radius=10,
            scrollbar_button_color="#cbd5e1",
        )
        controls.grid(row=0, column=0, sticky="nsew", padx=(0, 12))
        controls.grid_columnconfigure(0, weight=1)
        self.controls = controls

        self.status = ctk.CTkLabel(
            controls,
            text="PREVIEW / NO HARDWARE",
            text_color="#0369a1",
            font=("Arial", 13, "bold"),
            anchor="w",
        )
        self.status.grid(row=0, column=0, sticky="ew", padx=12, pady=(8, 4))

        self._build_inputs(controls)
        self._build_buttons(controls)
        self._build_manual_controls(controls)

        right_panel = ctk.CTkFrame(body, fg_color=PANEL, corner_radius=10)
        right_panel.grid(row=0, column=1, sticky="nsew")
        right_panel.grid_columnconfigure(0, weight=1)
        right_panel.grid_rowconfigure(1, weight=1)

        ctk.CTkLabel(
            right_panel,
            text="Session Terminal",
            font=("Arial", 18, "bold"),
            text_color=TEXT,
            anchor="w",
        ).grid(row=0, column=0, sticky="ew", padx=14, pady=(10, 5))

        self.terminal = ctk.CTkTextbox(
            right_panel,
            height=220,
            fg_color="#f8fafc",
            text_color=TEXT,
            border_width=1,
            border_color="#cbd5e1",
            corner_radius=8,
            wrap="word",
        )
        self.terminal.grid(row=1, column=0, sticky="nsew", padx=14, pady=(0, 10))

    def _build_inputs(self, parent: ctk.CTkFrame) -> None:
        fields_panel = ctk.CTkFrame(parent, fg_color=PANEL_SOFT, corner_radius=8)
        fields_panel.grid(row=1, column=0, sticky="ew", padx=10, pady=(0, 6))
        fields_panel.grid_columnconfigure((0, 1), weight=1)

        ctk.CTkLabel(
            fields_panel,
            text="Test Parameters",
            font=("Arial", 15, "bold"),
            text_color=TEXT,
            anchor="w",
        ).grid(row=0, column=0, columnspan=2, padx=8, pady=(8, 3), sticky="ew")

        fields = [
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
        ]
        for attribute, label_text, row, column in fields:
            field = ctk.CTkFrame(fields_panel, fg_color="transparent")
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
        for entry in (
            self.cycles_input,
            self.speed_input,
            self.acceleration_input,
            self.min_angle_input,
            self.max_angle_input,
        ):
            entry.bind("<KeyRelease>", self._update_parameter_summary, add="+")
        self.fixture_id_input.bind("<KeyRelease>", self._update_tare_identity, add="+")
        self.calibration_id_input.bind("<KeyRelease>", self._update_tare_identity, add="+")

        protocol = ctk.CTkFrame(fields_panel, fg_color="transparent")
        protocol.grid(row=6, column=0, columnspan=2, padx=6, pady=(6, 4), sticky="ew")
        protocol.grid_columnconfigure((1, 2, 3), weight=1)
        ctk.CTkLabel(
            protocol, text="Protocol:", font=("Arial", 11, "bold"), text_color=TEXT
        ).grid(row=0, column=0, padx=(0, 6), sticky="w")
        self.protocol_menu = ctk.CTkOptionMenu(
            protocol,
            variable=self.protocol_var,
            values=list(self.protocol_display_to_key),
            command=lambda _selection: self._update_protocol_status(),
            height=30,
        )
        self.protocol_menu.grid(row=0, column=1, columnspan=3, sticky="ew")
        ctk.CTkButton(
            protocol,
            text="Load Preset",
            command=self._mock_load_preset,
            height=30,
            fg_color=BLUE,
            hover_color="#1d4ed8",
        ).grid(row=1, column=0, columnspan=2, padx=(0, 3), pady=(5, 0), sticky="ew")
        ctk.CTkButton(
            protocol,
            text="Reset Test Fields",
            command=self._mock_reset_test_fields,
            height=30,
            fg_color=AMBER,
            hover_color="#b45309",
        ).grid(row=1, column=2, columnspan=2, padx=(3, 0), pady=(5, 0), sticky="ew")
        self.protocol_status = ctk.CTkLabel(
            fields_panel,
            text="Active protocol: Custom",
            text_color=MUTED,
            font=("Arial", 10, "bold"),
            anchor="w",
            justify="left",
            wraplength=390,
        )
        self.protocol_status.grid(
            row=7, column=0, columnspan=2, padx=8, pady=(0, 7), sticky="ew"
        )

    def _build_buttons(self, parent: ctk.CTkFrame) -> None:
        buttons = ctk.CTkFrame(parent, fg_color=PANEL, corner_radius=0)
        buttons.grid(row=2, column=0, sticky="ew", padx=10, pady=(0, 6))
        buttons.grid_columnconfigure((0, 1), weight=1)

        self.parameter_summary = ctk.CTkLabel(
            buttons,
            text="",
            text_color=MUTED,
            font=("Arial", 11, "bold"),
            anchor="w",
            justify="left",
            wraplength=390,
        )
        self.parameter_summary.grid(
            row=0, column=0, columnspan=2, padx=5, pady=(0, 3), sticky="ew"
        )

        button_defs = [
            ("Connect", GREEN, "#15803d", self._mock_connect),
            ("Start", BLUE, "#1d4ed8", self._mock_start),
            ("Stop", RED, "#b91c1c", self._mock_stop),
        ]
        self.action_buttons = []
        for index, (label, colour, hover, command) in enumerate(button_defs):
            button = ctk.CTkButton(
                buttons,
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
            self.action_buttons.append(button)
        self.action_buttons[1].configure(state="disabled")
        self._update_parameter_summary()

    def _preview_step_unit_changed(self, selection):
        self.preview_step_entry.delete(0, "end")
        turns = selection == "Motor turns"
        if turns:
            self.preview_continuous_mode.set(False)
        self.preview_continuous_switch.configure(state="disabled" if turns else "normal")
        conversion = self.config["motion"]["afo_degrees_per_odrive_turn"]
        self.preview_step_entry.configure(placeholder_text="Motor turns per click (e.g. 1)" if turns else "Degrees per click (0.01-10)")
        self.preview_step_hint.configure(text=(
            f"Step: {0.01 / conversion:.6g}-{10 / conversion:.6g} motor turns. Direct turn command; existing travel limits apply."
            if turns else "Degree steps use the configured motor conversion. Continuous mode uses degrees."
        ))

    def _build_manual_controls(self, parent: ctk.CTkFrame) -> None:
        manual = ctk.CTkFrame(parent, fg_color=PANEL_SOFT, corner_radius=8)
        manual.grid(row=3, column=0, sticky="ew", padx=10, pady=(0, 8))
        manual.grid_columnconfigure((0, 1, 2), weight=1)

        self.preview_step_units = ctk.StringVar(value="Degrees")
        self.preview_units_menu = ctk.CTkOptionMenu(
            manual, values=["Degrees", "Motor turns"], variable=self.preview_step_units,
            command=self._preview_step_unit_changed, height=30, width=130,
        )
        self.preview_units_menu.grid(row=0, column=0, sticky="ew", padx=6, pady=(6, 4))
        self.preview_step_entry = ctk.CTkEntry(
            manual, placeholder_text="Degrees per click (0.01-10)", height=30, corner_radius=6,
        )
        self.preview_step_entry.grid(row=0, column=1, sticky="ew", padx=6, pady=(6, 4))

        self.preview_manual_switch = ctk.CTkSwitch(manual, text="Manual Mode", state="disabled")
        self.preview_manual_switch.grid(row=0, column=2, padx=6, pady=(6, 4))

        ctk.CTkButton(
            manual,
            text="<",
            command=lambda: self._log("Preview left step."),
            width=60,
            height=34,
            corner_radius=8,
            fg_color="#64748b",
            hover_color="#475569",
            font=("Arial", 20, "bold"),
        ).grid(row=1, column=0, padx=6, pady=(0, 4), sticky="ew")

        ctk.CTkButton(
            manual,
            text=">",
            command=lambda: self._log("Preview right step."),
            width=60,
            height=34,
            corner_radius=8,
            fg_color="#64748b",
            hover_color="#475569",
            font=("Arial", 20, "bold"),
        ).grid(row=1, column=1, padx=6, pady=(0, 4), sticky="ew")

        self.preview_continuous_mode = ctk.BooleanVar(value=False)
        self.preview_continuous_switch = ctk.CTkSwitch(manual, text="Continuous Mode", variable=self.preview_continuous_mode)
        self.preview_continuous_switch.grid(row=1, column=2, padx=6, pady=(0, 4))

        self.preview_step_hint = ctk.CTkLabel(
            manual, text="Degree steps use the configured motor conversion. Continuous mode uses degrees.",
            font=("Arial", 10), anchor="w", justify="left", wraplength=390, text_color=MUTED,
        )
        self.preview_step_hint.grid(row=2, column=0, columnspan=3, sticky="ew", padx=6, pady=(2, 2))
        ctk.CTkLabel(
            manual, text="ODrive motor position (session): 0.00000000 turns (PREVIEW)\n"
                         "From machine zero: unavailable until physical 90 deg is verified",
            font=("Arial", 11, "bold"), anchor="w", justify="left", wraplength=390,
            text_color=MUTED,
        ).grid(row=3, column=0, columnspan=3, sticky="ew", padx=6, pady=(2, 4))

        machine_zero = ctk.CTkFrame(manual, fg_color="#fff7ed", corner_radius=8)
        machine_zero.grid(row=4, column=0, columnspan=3, padx=6, pady=(4, 6), sticky="ew")
        machine_zero.grid_columnconfigure((0, 1), weight=1)
        ctk.CTkLabel(
            machine_zero,
            text="Machine Zero \N{EM DASH} Fixture at 90\N{DEGREE SIGN}",
            font=("Arial", 14, "bold"),
            text_color=TEXT,
            anchor="w",
        ).grid(row=0, column=0, columnspan=2, padx=8, pady=(8, 2), sticky="ew")
        ctk.CTkLabel(
            machine_zero,
            text=(
                "Use the supplied square to confirm that the moving fixture is at 90\N{DEGREE SIGN} "
                "to the fixed machine reference. Use the slow Jog Left and Jog Right controls "
                "to make small adjustments until the fixture is aligned with the square. Then "
                "select 'Set Machine Zero \N{EM DASH} Fixture at 90\N{DEGREE SIGN}'. This defines "
                "machine angle 0\N{DEGREE SIGN} for the current ODrive power session."
            ),
            font=("Arial", 10),
            text_color=TEXT,
            justify="left",
            anchor="w",
            wraplength=390,
        ).grid(row=1, column=0, columnspan=2, padx=8, pady=(0, 5), sticky="ew")
        self.reference_status = ctk.CTkLabel(
            machine_zero,
            text="Machine zero: SETUP REQUIRED\nNo hardware in preview",
            text_color=AMBER,
            font=("Arial", 11, "bold"),
            justify="left",
            anchor="w",
        )
        self.reference_status.grid(row=2, column=0, columnspan=2, sticky="ew", padx=8, pady=(0, 5))
        self.reference_button = ctk.CTkButton(
            machine_zero,
            text="Set Machine Zero \N{EM DASH} Fixture at 90\N{DEGREE SIGN}",
            command=self._show_recovery_preview,
            fg_color=AMBER,
            hover_color="#b45309",
            state="disabled",
            height=34,
        )
        self.reference_button.grid(row=3, column=0, padx=(8, 4), pady=(0, 8), sticky="ew")
        ctk.CTkButton(
            machine_zero,
            text="Return to Machine Zero \N{EM DASH} 90\N{DEGREE SIGN}",
            command=self._mock_return_neutral,
            fg_color="#0f766e",
            hover_color="#115e59",
            corner_radius=8,
            height=34,
            font=("Arial", 12, "bold"),
        ).grid(row=3, column=1, padx=(4, 8), pady=(0, 8), sticky="ew")

        tare = ctk.CTkFrame(manual, fg_color="#ecfdf5", corner_radius=8)
        tare.grid(row=5, column=0, columnspan=3, padx=6, pady=(0, 6), sticky="ew")
        tare.grid_columnconfigure((0, 1), weight=1)
        self.tare_status = ctk.CTkLabel(
            tare,
            text="Empty-machine tare: REQUIRED",
            text_color=AMBER,
            font=("Arial", 11, "bold"),
            anchor="w",
        )
        self.tare_status.grid(
            row=0, column=0, columnspan=2, padx=8, pady=(7, 3), sticky="ew"
        )
        self.tare_button = ctk.CTkButton(
            tare,
            text="Tare Empty Machine",
            command=self._mock_tare,
            fg_color=GREEN,
            hover_color="#15803d",
            state="disabled",
            height=34,
        )
        self.tare_button.grid(row=1, column=0, padx=(8, 4), pady=(0, 8), sticky="ew")
        self.clear_session_tare_button = ctk.CTkButton(
            tare,
            text="Clear Session Tare",
            command=self._mock_clear_session_tare,
            fg_color=AMBER,
            hover_color="#b45309",
            state="disabled",
            height=34,
        )
        self.clear_session_tare_button.grid(
            row=1, column=1, padx=(4, 8), pady=(0, 8), sticky="ew"
        )

    def _logo_card(
        self,
        parent: ctk.CTkFrame,
        candidates: tuple[str, ...],
        fallback: str,
        column: int,
        width: int,
    ) -> None:
        logo_path = next((IMAGE_DIR / name for name in candidates if (IMAGE_DIR / name).exists()), None)
        if logo_path:
            image = self._prepare_logo_image(logo_path, max_size=(width - 30, 50))
            logo = ctk.CTkImage(light_image=image, dark_image=image, size=image.size)
            self.logo_images.append(logo)
            label = ctk.CTkLabel(parent, image=logo, text="", fg_color="transparent")
        else:
            label = ctk.CTkLabel(
                parent,
                text=fallback,
                font=("Arial", 15, "bold"),
                text_color="#111827",
                fg_color="transparent",
            )
        label.grid(row=0, column=column, padx=10, pady=4, sticky="nsew")

    @staticmethod
    def _prepare_logo_image(path: Path, max_size: tuple[int, int]) -> Image.Image:
        image = Image.open(path).convert("RGBA")
        image.thumbnail(max_size, Image.LANCZOS)
        return image

    def _log(self, message: str) -> None:
        self.terminal.insert("end", message + "\n")
        self.terminal.see("end")

    def _update_parameter_summary(self, _event=None) -> None:
        def entered(entry: ctk.CTkEntry) -> str:
            return entry.get().strip() or "?"

        self.parameter_summary.configure(
            text=(
                f"Commanded: {entered(self.cycles_input)} cycles | "
                f"{entered(self.speed_input)}\N{DEGREE SIGN}/s | "
                f"{entered(self.acceleration_input)}\N{DEGREE SIGN}/s\N{SUPERSCRIPT TWO} | "
                f"-{entered(self.min_angle_input)}\N{DEGREE SIGN} to "
                f"+{entered(self.max_angle_input)}\N{DEGREE SIGN}"
            )
        )
        self._update_protocol_status()

    def _motion_values(self) -> dict[str, str]:
        return {
            "cycles": self.cycles_input.get(),
            "minimum_angle_deg": self.min_angle_input.get(),
            "maximum_angle_deg": self.max_angle_input.get(),
            "speed_deg_s": self.speed_input.get(),
            "acceleration_deg_s2": self.acceleration_input.get(),
        }

    def _preset_modified(self, preset: dict) -> bool:
        values = self._motion_values()
        try:
            if int(values["cycles"]) != int(preset["cycles"]):
                return True
            return any(
                not math.isclose(float(values[key]), float(preset[key]), abs_tol=1e-9)
                for key in (
                    "minimum_angle_deg",
                    "maximum_angle_deg",
                    "speed_deg_s",
                    "acceleration_deg_s2",
                )
            )
        except (TypeError, ValueError):
            return True

    def _update_protocol_status(self) -> None:
        if not hasattr(self, "protocol_status"):
            return
        if self.loaded_preset_key == "custom":
            active_name = "Custom"
        else:
            preset = self.config["test_presets"][self.loaded_preset_key]
            active_name = f"{preset['display_name']} v{preset['version']}"
            if self._preset_modified(preset):
                active_name += " \N{EM DASH} MODIFIED"
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
        self.protocol_status.configure(text=text, text_color=colour)

    @staticmethod
    def _set_entry(entry: ctk.CTkEntry, value: object) -> None:
        entry.delete(0, "end")
        entry.insert(0, str(value))

    def _mock_load_preset(self) -> None:
        selected_key = self.protocol_display_to_key[self.protocol_var.get()]
        preset = None if selected_key == "custom" else self.config["test_presets"][selected_key]
        has_values = any(value.strip() for value in self._motion_values().values())
        messages = []
        if has_values and (preset is None or self._preset_modified(preset)):
            messages.append("This will replace the motion values currently entered.")
        if preset and preset["test_type"] == "empty_machine_baseline":
            messages.append(
                "Confirm the AFO and all removable loads are removed before running this "
                "empty-machine baseline."
            )
        if messages and not messagebox.askokcancel(
            "Load Protocol Preset", "\n\n".join(messages), parent=self.root
        ):
            return
        entries = {
            "cycles": self.cycles_input,
            "minimum_angle_deg": self.min_angle_input,
            "maximum_angle_deg": self.max_angle_input,
            "speed_deg_s": self.speed_input,
            "acceleration_deg_s2": self.acceleration_input,
        }
        if preset is None:
            for entry in entries.values():
                entry.delete(0, "end")
        else:
            for key, entry in entries.items():
                self._set_entry(entry, preset[key])
        self.loaded_preset_key = selected_key
        self.preset_loaded_at = datetime.now().astimezone().isoformat(
            timespec="milliseconds"
        )
        self._update_parameter_summary()
        self._log(f"Loaded protocol: {self.protocol_var.get()}.")

    def _mock_connect(self) -> None:
        self.connected = True
        self.reference_verified = False
        self.tare_valid = False
        self.tare_fixture_id = None
        self.tare_calibration_id = None
        self.status.configure(
            text="CONNECTED / MACHINE-ZERO SETUP REQUIRED (PREVIEW)", text_color=AMBER
        )
        self.reference_status.configure(
            text="Machine zero: SETUP REQUIRED\nPhysical 90 deg verification required",
            text_color=AMBER,
        )
        self.tare_status.configure(text="Empty-machine tare: REQUIRED", text_color=AMBER)
        self.tare_button.configure(state="disabled")
        self.clear_session_tare_button.configure(state="disabled")
        self.reference_button.configure(state="normal")
        self.action_buttons[1].configure(state="disabled")
        self.preview_manual_switch.configure(state="disabled")
        self._log("Mock connect: normal movement remains blocked pending machine-zero setup.")

    def _mock_start(self) -> None:
        if not self.reference_verified:
            self._log("Mock start blocked: machine zero is not verified.")
            return
        if not self.tare_valid:
            self._log("Mock start blocked: empty-machine tare is required.")
            return
        self._log("Mock start pressed. No motor command was sent.")
        self.show_plot()

    def _mock_stop(self) -> None:
        self.status.configure(text="STOPPED PREVIEW", text_color="#b91c1c")
        self._log("Mock stop pressed.")

    def _mock_reset_test_fields(self) -> None:
        for entry in (
            self.file_name_input,
            self.cycles_input,
            self.speed_input,
            self.acceleration_input,
            self.min_angle_input,
            self.max_angle_input,
            self.afo_id_input,
        ):
            entry.delete(0, "end")
        self.protocol_var.set("Custom")
        self.loaded_preset_key = "custom"
        self.preset_loaded_at = datetime.now().astimezone().isoformat(
            timespec="milliseconds"
        )
        self._update_parameter_summary()
        self._update_tare_identity()
        self._log("Run-specific test fields reset; session state was preserved.")

    def _mock_clear_session_tare(self) -> None:
        if not self.tare_valid:
            return
        if not messagebox.askokcancel(
            "Clear Session Tare",
            "Discard only the stored empty-machine tare? Machine zero, connection, operator, "
            "fixture, calibration, and all test fields will remain unchanged. A new unloaded "
            "tare is required before the next run.",
            parent=self.root,
        ):
            return
        self.tare_valid = False
        self.tare_fixture_id = None
        self.tare_calibration_id = None
        self.tare_status.configure(text="Empty-machine tare: REQUIRED", text_color=AMBER)
        self.clear_session_tare_button.configure(state="disabled")
        self.action_buttons[1].configure(state="disabled")
        self._log("Stored session tare cleared; all other session and test fields were preserved.")

    def _update_tare_identity(self, _event=None) -> None:
        if not self.tare_valid:
            return
        mismatches = []
        if self.fixture_id_input.get().strip() != self.tare_fixture_id:
            mismatches.append("Fixture ID does not match the stored tare")
        if self.calibration_id_input.get().strip() != self.tare_calibration_id:
            mismatches.append("Calibration ID does not match the stored tare")
        if mismatches:
            self.tare_status.configure(
                text="Empty-machine tare: IDENTITY MISMATCH\n" + "\n".join(mismatches),
                text_color=RED,
            )
            self.action_buttons[1].configure(state="disabled")
        else:
            self.tare_status.configure(
                text="Empty-machine tare: VALID\nPreview session offset 0.000000 V/V",
                text_color=GREEN,
            )
            if self.reference_verified:
                self.action_buttons[1].configure(state="normal")

    def _mock_return_neutral(self) -> None:
        self.status.configure(text="MACHINE ZERO RETURN PREVIEW", text_color="#0f766e")
        self._log("Mock machine-zero return pressed.")
        self._log("Real app commands only the verified session mapping after safety checks.")

    def _mock_tare(self) -> None:
        if not self.reference_verified:
            self._log("Preview tare blocked: verify machine zero first.")
            return
        self.tare_valid = True
        self.tare_fixture_id = self.fixture_id_input.get().strip()
        self.tare_calibration_id = self.calibration_id_input.get().strip()
        self.tare_status.configure(
            text="Empty-machine tare: VALID\nPreview session offset 0.000000 V/V",
            text_color=GREEN,
        )
        self.action_buttons[1].configure(state="normal")
        self.clear_session_tare_button.configure(state="normal")
        self._log("Preview empty-machine tare captured with no AFO/load fitted.")

    def _show_recovery_preview(self) -> None:
        if not self.connected:
            return
        dialog = ctk.CTkToplevel(self.root)
        dialog.title("EAST Machine Zero Setup Preview")
        dialog.geometry("650x510")
        dialog.configure(fg_color=BG)
        dialog.transient(self.root)
        panel = ctk.CTkFrame(dialog, fg_color=PANEL, corner_radius=8)
        panel.pack(fill="both", expand=True, padx=18, pady=18)
        ctk.CTkLabel(
            panel,
            text="Machine Zero \N{EM DASH} Fixture at 90\N{DEGREE SIGN}",
            font=("Arial", 20, "bold"),
            text_color=TEXT,
        ).pack(anchor="w", padx=16, pady=(14, 6))
        ctk.CTkLabel(
            panel,
            text=(
                "Use the supplied square to confirm that the moving fixture is at 90\N{DEGREE SIGN} "
                "to the fixed machine reference. Use the slow Jog Left and Jog Right controls "
                "to make small adjustments until the fixture is aligned with the square. Then "
                "select 'Set Machine Zero \N{EM DASH} Fixture at 90\N{DEGREE SIGN}'. This defines "
                "machine angle 0\N{DEGREE SIGN} for the current ODrive power session."
            ),
            wraplength=540,
            justify="left",
            text_color=TEXT,
        ).pack(anchor="w", padx=16, pady=(0, 10))
        acknowledgement = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(
            panel,
            text="Fixture is clear, E-stop is accessible, and I am observing the mechanism",
            variable=acknowledgement,
        ).pack(anchor="w", padx=16, pady=8)
        jog = ctk.CTkFrame(panel, fg_color=PANEL_SOFT, corner_radius=6)
        jog.pack(fill="x", padx=16, pady=6)
        ctk.CTkButton(
            jog,
            text="Jog Left (0.25\N{DEGREE SIGN})",
            command=lambda: self._log("Preview setup jog left 0.25 deg"),
        ).pack(side="left", fill="x", expand=True, padx=6, pady=8)
        ctk.CTkButton(
            jog,
            text="Jog Right (0.25\N{DEGREE SIGN})",
            command=lambda: self._log("Preview setup jog right 0.25 deg"),
        ).pack(side="left", fill="x", expand=True, padx=6, pady=8)

        def set_neutral():
            if not acknowledgement.get():
                self._log("Preview machine zero not set: acknowledgement is required.")
                return
            self.reference_verified = True
            self.reference_status.configure(
                text=(
                    "Machine zero: VERIFIED\n"
                    "Physical 90 deg = machine 0 deg (0.00000000 preview session turns)"
                ),
                text_color=GREEN,
            )
            self.status.configure(
                text="CONNECTED / MACHINE ZERO VERIFIED (PREVIEW)", text_color=GREEN
            )
            self.action_buttons[1].configure(state="disabled")
            self.preview_manual_switch.configure(state="normal")
            self.tare_button.configure(state="normal")
            self._log("Preview machine zero verified. No hardware moved.")
            dialog.destroy()

        ctk.CTkButton(
            panel,
            text="Set Machine Zero \N{EM DASH} Fixture at 90\N{DEGREE SIGN}",
            command=set_neutral,
            fg_color="#0f766e",
            hover_color="#115e59",
            height=36,
        ).pack(fill="x", padx=16, pady=(12, 6))
        ctk.CTkButton(
            panel,
            text="Cancel / Keep Motion Blocked",
            command=dialog.destroy,
            fg_color="#94a3b8",
            hover_color=MUTED,
        ).pack(fill="x", padx=16, pady=(0, 14))

    def show_plot(self) -> None:
        if self.plot_window is None or not self.plot_window.winfo_exists():
            self.plot_window = ctk.CTkToplevel(self.root)
            self.plot_window.title("Torque vs AFO Angle Preview")
            self.plot_window.geometry("640x420")
            self.plot_window.minsize(520, 340)
            self.plot_window.configure(fg_color=PANEL_DARK)
            self.plot_window.protocol("WM_DELETE_WINDOW", self.close_plot)
            self._make_plot_window_float()

            self.floating_canvas = tk.Canvas(self.plot_window, bg="#ffffff", highlightthickness=0)
            self.floating_canvas.pack(fill="both", expand=True, padx=15, pady=15)
            self.floating_canvas.bind("<Configure>", lambda _event: self._draw_floating_plot())

        self.plot_window.deiconify()
        self.plot_window.lift()
        self._draw_floating_plot()
        self._log("Floating preview plot shown.")

    def close_plot(self) -> None:
        if self.plot_window is not None and self.plot_window.winfo_exists():
            self.plot_window.destroy()
        self.plot_window = None
        self.floating_canvas = None
        self._log("Floating preview plot closed.")

    def _draw_floating_plot(self) -> None:
        self._draw_plot_on_canvas(self.floating_canvas, compact=False)

    def _make_plot_window_float(self) -> None:
        if self.plot_window is None:
            return
        try:
            self.plot_window.attributes("-topmost", True)
        except tk.TclError:
            pass
        if sys.platform == "darwin":
            try:
                self.plot_window.tk.call(
                    "tk::unsupported::MacWindowStyle",
                    "style",
                    self.plot_window._w,
                    "floating",
                    "closeBox",
                )
            except tk.TclError:
                pass

    def _draw_plot_on_canvas(self, canvas: tk.Canvas | None, compact: bool) -> None:
        if canvas is None:
            return
        canvas.delete("all")
        width = max(canvas.winfo_width(), 280 if compact else 400)
        height = max(canvas.winfo_height(), 220 if compact else 250)
        margin = 55
        left, right = margin, width - 25
        top, bottom = 42, height - margin

        title_size = 12 if compact else 14
        canvas.create_text(width / 2, 20, text="AFO Strain Test Preview", fill=TEXT, font=("Arial", title_size, "bold"))
        for i in range(5):
            x = left + i * (right - left) / 4
            y = top + i * (bottom - top) / 4
            canvas.create_line(x, top, x, bottom, fill="#e2e8f0")
            canvas.create_line(left, y, right, y, fill="#e2e8f0")
        canvas.create_line(left, bottom, right, bottom, fill=TEXT, width=2)
        canvas.create_line(left, bottom, left, top, fill=TEXT, width=2)
        canvas.create_text(
            width / 2,
            height - 18,
            text="ODrive-Derived AFO Angle (degrees)",
            fill=TEXT,
        )
        canvas.create_text(18, height / 2, text="Torque (Nm)", fill=TEXT, angle=90)

        points = []
        for i in range(160):
            x_norm = i / 159
            angle = -10 + 20 * x_norm
            torque = 0.15 * angle + 0.5 * math.sin(angle / 2.5)
            y_norm = (torque + 2.0) / 4.0
            x = left + x_norm * (right - left)
            y = bottom - y_norm * (bottom - top)
            points.extend((x, y))

        canvas.create_line(*points, fill=GOLD, width=2, smooth=True)


def main() -> None:
    if "--smoke-test" in sys.argv:
        print(f"{APP_NAME} {APP_VERSION} smoke test OK")
        print("No hardware modules imported.")
        return

    print(f"Starting {APP_NAME} {APP_VERSION} from {Path(__file__).resolve()}")
    root = ctk.CTk()
    EastGuiPreview(root)
    root.mainloop()


if __name__ == "__main__":
    main()
