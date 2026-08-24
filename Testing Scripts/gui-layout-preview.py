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
APP_VERSION = "1.1.1-gui-preview"
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

        ctk.set_appearance_mode("light")
        ctk.set_default_color_theme("blue")

        self._build_ui()
        self._log(f"{APP_NAME} {APP_VERSION}")
        self._log("Preview only. No hardware modules are imported.")
        self._log(f"Running from: {Path(__file__).resolve()}")
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
        body.grid_columnconfigure(0, weight=0)
        body.grid_columnconfigure(1, weight=1)
        body.grid_rowconfigure(0, weight=1)

        controls = ctk.CTkFrame(body, fg_color=PANEL, corner_radius=10)
        controls.grid(row=0, column=0, sticky="nsw", padx=(0, 18))
        controls.grid_columnconfigure(0, weight=1)

        self.status = ctk.CTkLabel(
            controls,
            text="PREVIEW / NO HARDWARE",
            text_color="#0369a1",
            font=("Arial", 13, "bold"),
            anchor="w",
        )
        self.status.grid(row=0, column=0, sticky="ew", padx=18, pady=(16, 8))

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
        fields_panel.grid(row=1, column=0, sticky="ew", padx=16, pady=(0, 14))
        fields_panel.grid_columnconfigure((0, 1), weight=1)

        fields = [
            ("File Name (Prefix)", 0, 0),
            ("Cycles", 0, 1),
            ("Speed (Degrees/Second)", 1, 0),
            ("Acceleration (Degrees/s^2)", 1, 1),
            ("Min Angle (Degrees)", 2, 0),
            ("Max Angle (Degrees)", 2, 1),
            ("Operator", 3, 0),
            ("AFO ID", 3, 1),
            ("Fixture ID", 4, 0),
            ("Calibration ID", 4, 1),
        ]
        for placeholder, row, column in fields:
            entry = ctk.CTkEntry(
                fields_panel,
                width=180,
                height=34,
                placeholder_text=placeholder,
                corner_radius=6,
            )
            entry.grid(row=row, column=column, padx=8, pady=7, sticky="ew")

    def _build_buttons(self, parent: ctk.CTkFrame) -> None:
        buttons = ctk.CTkFrame(parent, fg_color=PANEL, corner_radius=0)
        buttons.grid(row=2, column=0, sticky="ew", padx=16, pady=(0, 14))
        buttons.grid_columnconfigure((0, 1), weight=1)

        button_defs = [
            ("Connect", GREEN, "#15803d", self._mock_connect),
            ("Start", BLUE, "#1d4ed8", self._mock_start),
            ("Stop", RED, "#b91c1c", self._mock_stop),
            ("Reset", AMBER, "#b45309", self._mock_reset),
        ]
        for index, (label, colour, hover, command) in enumerate(button_defs):
            ctk.CTkButton(
                buttons,
                text=label,
                command=command,
                fg_color=colour,
                hover_color=hover,
                corner_radius=8,
                height=38,
                font=("Arial", 13, "bold"),
            ).grid(row=index // 2, column=index % 2, padx=6, pady=6, sticky="ew")

    def _build_manual_controls(self, parent: ctk.CTkFrame) -> None:
        manual = ctk.CTkFrame(parent, fg_color=PANEL_SOFT, corner_radius=8)
        manual.grid(row=3, column=0, sticky="ew", padx=16, pady=(0, 16))
        manual.grid_columnconfigure((0, 1, 2), weight=1)

        ctk.CTkEntry(
            manual,
            placeholder_text="Step Angle (0-10 deg)",
            height=34,
            corner_radius=6,
        ).grid(row=0, column=0, columnspan=2, sticky="ew", padx=8, pady=(10, 8))

        ctk.CTkSwitch(manual, text="Manual Mode").grid(row=0, column=2, padx=8, pady=(10, 8))

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
            text='Return to Neutral (90 deg)',
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

    def _mock_connect(self) -> None:
        self.status.configure(text="CONNECTED PREVIEW / NO HARDWARE", text_color="#15803d")
        self._log("Mock connect pressed. No ODrive lookup was attempted.")

    def _mock_start(self) -> None:
        self._log("Mock start pressed. No motor command was sent.")
        self.show_plot()

    def _mock_stop(self) -> None:
        self.status.configure(text="STOPPED PREVIEW", text_color="#b91c1c")
        self._log("Mock stop pressed.")

    def _mock_reset(self) -> None:
        self.terminal.delete("1.0", "end")
        self.status.configure(text="PREVIEW / NO HARDWARE", text_color="#0369a1")
        self._log(f"{APP_NAME} {APP_VERSION}")
        self._log("Preview reset.")

    def _mock_return_neutral(self) -> None:
        self.status.configure(text="NEUTRAL RETURN PREVIEW", text_color="#0f766e")
        self._log("Mock neutral return pressed.")
        self._log("Real app should command the verified neutral/zeroed 90 deg position only after safety checks.")

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
        canvas.create_text(width / 2, height - 18, text="AFO Angle (degrees)", fill=TEXT)
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
