"""Mac-safe EAST GUI layout preview.

This script does not import ODrive, Phidget, PyQtGraph, pywinstyles, or the
main hardware-control GUI. It is only for checking EAST branding, window size,
lab-laptop layout, logos, and optional graph-window behaviour on a Mac.
"""

from __future__ import annotations

import math
import sys
import tkinter as tk
from pathlib import Path

import customtkinter as ctk
from PIL import Image


APP_NAME = "EAST"
APP_VERSION = "1.2.0-neutral-recovery-preview"
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
        self.root.geometry("1040x700")
        self.root.minsize(860, 620)
        self.root.configure(fg_color=BG)

        self.logo_images: list[ctk.CTkImage] = []
        self.plot_window: ctk.CTkToplevel | None = None
        self.floating_canvas: tk.Canvas | None = None
        self.connected = False
        self.reference_verified = False

        ctk.set_appearance_mode("light")
        ctk.set_default_color_theme("blue")

        self._build_ui()
        self._log(f"{APP_NAME} {APP_VERSION}")
        self._log("Preview only. No hardware modules are imported.")
        self._log(f"Running from: {Path(__file__).resolve()}")
        if "--no-auto-plot" not in sys.argv:
            self.root.after(250, self.show_plot)

    def _build_ui(self) -> None:
        shell = ctk.CTkFrame(self.root, fg_color=BG)
        shell.pack(fill="both", expand=True, padx=28, pady=20)
        shell.grid_columnconfigure(0, weight=1)
        shell.grid_rowconfigure(1, weight=1)

        self._build_header(shell)
        self._build_body(shell)
        self._build_footer(shell)

    def _build_header(self, parent: ctk.CTkFrame) -> None:
        header = ctk.CTkFrame(parent, fg_color=BG)
        header.grid(row=0, column=0, sticky="ew", pady=(0, 16))
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
            font=("Arial", 52, "bold"),
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
        controls.grid(row=0, column=0, sticky="nsew", padx=(0, 18))
        controls.grid_columnconfigure(0, weight=1)

        self.status = ctk.CTkLabel(
            controls,
            text="PREVIEW / NO HARDWARE",
            text_color="#0369a1",
            font=("Arial", 13, "bold"),
            anchor="w",
        )
        self.status.grid(row=0, column=0, sticky="ew", padx=18, pady=(16, 8))

        reference = ctk.CTkFrame(controls, fg_color="#fff7ed", corner_radius=8)
        reference.grid(row=1, column=0, sticky="ew", padx=16, pady=(0, 10))
        reference.grid_columnconfigure(0, weight=1)
        self.reference_status = ctk.CTkLabel(
            reference,
            text="Neutral reference: RECOVERY REQUIRED\nNo hardware in preview",
            text_color=AMBER,
            font=("Arial", 11, "bold"),
            justify="left",
            anchor="w",
        )
        self.reference_status.grid(row=0, column=0, sticky="ew", padx=8, pady=7)
        self.reference_button = ctk.CTkButton(
            reference,
            text="Verify / Recover",
            command=self._show_recovery_preview,
            width=118,
            height=28,
            fg_color=AMBER,
            hover_color="#b45309",
            state="disabled",
        )
        self.reference_button.grid(row=0, column=1, padx=8, pady=6)

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
        ).grid(row=0, column=0, sticky="ew", padx=18, pady=(16, 8))

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
        self.terminal.grid(row=1, column=0, sticky="nsew", padx=18, pady=(0, 18))

    def _build_inputs(self, parent: ctk.CTkFrame) -> None:
        fields_panel = ctk.CTkFrame(parent, fg_color=PANEL_SOFT, corner_radius=8)
        fields_panel.grid(row=2, column=0, sticky="ew", padx=16, pady=(0, 14))
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
            field.grid(row=row + 1, column=column, padx=8, pady=4, sticky="ew")
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
                height=34,
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

    def _build_buttons(self, parent: ctk.CTkFrame) -> None:
        buttons = ctk.CTkFrame(parent, fg_color=PANEL, corner_radius=0)
        buttons.grid(row=3, column=0, sticky="ew", padx=16, pady=(0, 14))
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
            row=0, column=0, columnspan=2, padx=6, pady=(0, 4), sticky="ew"
        )

        button_defs = [
            ("Connect", GREEN, "#15803d", self._mock_connect),
            ("Start", BLUE, "#1d4ed8", self._mock_start),
            ("Stop", RED, "#b91c1c", self._mock_stop),
            ("Reset", AMBER, "#b45309", self._mock_reset),
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
                height=38,
                font=("Arial", 13, "bold"),
            )
            button.grid(row=1 + index // 2, column=index % 2, padx=6, pady=6, sticky="ew")
            self.action_buttons.append(button)
        self.action_buttons[1].configure(state="disabled")
        self._update_parameter_summary()

    def _build_manual_controls(self, parent: ctk.CTkFrame) -> None:
        manual = ctk.CTkFrame(parent, fg_color=PANEL_SOFT, corner_radius=8)
        manual.grid(row=4, column=0, sticky="ew", padx=16, pady=(0, 16))
        manual.grid_columnconfigure((0, 1, 2), weight=1)

        ctk.CTkEntry(
            manual,
            placeholder_text="Step Angle (0-10 deg)",
            height=34,
            corner_radius=6,
        ).grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=(10, 8))

        self.preview_manual_switch = ctk.CTkSwitch(manual, text="Manual Mode", state="disabled")
        self.preview_manual_switch.grid(row=0, column=2, padx=8, pady=(10, 8))

        ctk.CTkButton(
            manual,
            text="<",
            command=lambda: self._log("Preview left step."),
            width=60,
            height=42,
            corner_radius=8,
            fg_color="#64748b",
            hover_color="#475569",
            font=("Arial", 20, "bold"),
        ).grid(row=1, column=0, padx=8, pady=(0, 10), sticky="ew")

        ctk.CTkButton(
            manual,
            text=">",
            command=lambda: self._log("Preview right step."),
            width=60,
            height=42,
            corner_radius=8,
            fg_color="#64748b",
            hover_color="#475569",
            font=("Arial", 20, "bold"),
        ).grid(row=1, column=1, padx=8, pady=(0, 10), sticky="ew")

        ctk.CTkSwitch(manual, text="Continuous Mode").grid(row=1, column=2, padx=8, pady=(0, 10))

        ctk.CTkButton(
            manual,
            text='Return to Verified 90 deg Neutral',
            command=self._mock_return_neutral,
            fg_color="#0f766e",
            hover_color="#115e59",
            corner_radius=8,
            height=40,
            font=("Arial", 13, "bold"),
        ).grid(row=2, column=0, columnspan=3, padx=8, pady=(0, 10), sticky="ew")

    def _build_footer(self, parent: ctk.CTkFrame) -> None:
        footer = ctk.CTkFrame(parent, fg_color=BG)
        footer.grid(row=2, column=0, sticky="ew", pady=(14, 0))
        footer.grid_columnconfigure(0, weight=1)
        ctk.CTkLabel(
            footer,
            text=f"Version {APP_VERSION}",
            font=("Arial", 12, "bold"),
            text_color=MUTED,
        ).grid(row=0, column=1, sticky="e")

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
            image = self._prepare_logo_image(logo_path, max_size=(width - 30, 68))
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

    def _mock_connect(self) -> None:
        self.connected = True
        self.reference_verified = False
        self.status.configure(text="CONNECTED / RECOVERY REQUIRED (PREVIEW)", text_color=AMBER)
        self.reference_status.configure(
            text="Neutral reference: RECOVERY REQUIRED\nPhysical 90 deg verification required",
            text_color=AMBER,
        )
        self.reference_button.configure(state="normal")
        self.action_buttons[1].configure(state="disabled")
        self.preview_manual_switch.configure(state="disabled")
        self._log("Mock connect: normal movement remains blocked pending neutral recovery.")

    def _mock_start(self) -> None:
        if not self.reference_verified:
            self._log("Mock start blocked: neutral reference is not verified.")
            return
        self._log("Mock start pressed. No motor command was sent.")
        self.show_plot()

    def _mock_stop(self) -> None:
        self.status.configure(text="STOPPED PREVIEW", text_color="#b91c1c")
        self._log("Mock stop pressed.")

    def _mock_reset(self) -> None:
        for entry in (
            self.file_name_input,
            self.cycles_input,
            self.speed_input,
            self.acceleration_input,
            self.min_angle_input,
            self.max_angle_input,
            self.operator_input,
            self.afo_id_input,
            self.fixture_id_input,
            self.calibration_id_input,
        ):
            entry.delete(0, "end")
        self._update_parameter_summary()
        self.terminal.delete("1.0", "end")
        self.status.configure(text="PREVIEW / NO HARDWARE", text_color="#0369a1")
        self.connected = False
        self.reference_verified = False
        self.reference_button.configure(state="disabled")
        self.action_buttons[1].configure(state="disabled")
        self.preview_manual_switch.configure(state="disabled")
        self.reference_status.configure(
            text="Neutral reference: RECOVERY REQUIRED\nNo hardware in preview",
            text_color=AMBER,
        )
        self._log(f"{APP_NAME} {APP_VERSION}")
        self._log("Preview reset.")

    def _mock_return_neutral(self) -> None:
        self.status.configure(text="NEUTRAL RETURN PREVIEW", text_color="#0f766e")
        self._log("Mock neutral return pressed.")
        self._log("Real app commands only the verified session mapping after safety checks.")

    def _show_recovery_preview(self) -> None:
        if not self.connected:
            return
        dialog = ctk.CTkToplevel(self.root)
        dialog.title("EAST Neutral Reference Recovery Preview")
        dialog.geometry("610x470")
        dialog.configure(fg_color=BG)
        dialog.transient(self.root)
        panel = ctk.CTkFrame(dialog, fg_color=PANEL, corner_radius=8)
        panel.pack(fill="both", expand=True, padx=18, pady=18)
        ctk.CTkLabel(
            panel,
            text="Verify Physical 90 Degree Neutral",
            font=("Arial", 20, "bold"),
            text_color=TEXT,
        ).pack(anchor="w", padx=16, pady=(14, 6))
        ctk.CTkLabel(
            panel,
            text=(
                "Preview of the restricted recovery workflow. Slow jog is bounded to "
                "+/-5 degrees and setting neutral records the current physical 90 degree "
                "position without moving the motor."
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
            jog, text="Jog -0.25 deg", command=lambda: self._log("Preview recovery jog -0.25 deg")
        ).pack(side="left", fill="x", expand=True, padx=6, pady=8)
        ctk.CTkButton(
            jog, text="Jog +0.25 deg", command=lambda: self._log("Preview recovery jog +0.25 deg")
        ).pack(side="left", fill="x", expand=True, padx=6, pady=8)

        def set_neutral():
            if not acknowledgement.get():
                self._log("Preview neutral not set: acknowledgement is required.")
                return
            self.reference_verified = True
            self.reference_status.configure(
                text="Neutral reference: VERIFIED\n90 deg = 0.00000000 preview session turns",
                text_color=GREEN,
            )
            self.status.configure(text="CONNECTED / REFERENCE VERIFIED (PREVIEW)", text_color=GREEN)
            self.action_buttons[1].configure(state="normal")
            self.preview_manual_switch.configure(state="normal")
            self._log("Preview physical neutral verified. No hardware moved.")
            dialog.destroy()

        ctk.CTkButton(
            panel,
            text="Set Current Physical Position as 90 deg Neutral",
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
