"""Read-only stiffness analysis in the operator's completed-run view."""
import json
import queue
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, ttk

import customtkinter as ctk

from east_stiffness import WINDOWS, analyse_stiffness


class StiffnessResults:
    def __init__(self, parent):
        self.parent = parent
        self.latest = None
        self.selected = None
        self.result = None
        self.replies = queue.Queue()
        self.request = 0

        controls = ctk.CTkFrame(parent)
        controls.pack(fill="x", padx=6, pady=6)
        for label, command in (("Latest run", self.show_latest), ("Open saved run", self.open_run),
                               ("Choose empty reference", self.choose_reference)):
            ctk.CTkButton(controls, text=label, command=command, width=120).pack(side="left", padx=3, pady=4)
        self.export_button = ctk.CTkButton(controls, text="Export", command=self.export, width=66, state="disabled")
        self.export_button.pack(side="left", padx=3)

        self.status = ctk.CTkLabel(parent, text="Complete a loaded test or open its saved metadata.",
                                   anchor="w", justify="left", wraplength=520)
        self.status.pack(fill="x", padx=10)
        self.table = ttk.Treeview(parent, columns=("sweep", "window", "slope", "sd"), show="headings", height=4)
        for key, label, width in (("sweep", "Sweep", 92), ("window", "Fit window (°)", 98),
                                  ("slope", "Stiffness (Nm/°)", 105), ("sd", "Cycle SD (Nm/°)", 115)):
            self.table.heading(key, text=label)
            self.table.column(key, anchor="center", width=width, minwidth=60)
        self.table.pack(fill="x", padx=8, pady=5)

        self.canvas = tk.Canvas(parent, background="white", highlightthickness=0, height=245)
        self.canvas.pack(fill="both", expand=True, padx=8, pady=4)
        self.canvas.bind("<Configure>", lambda _event: self.draw())

    def completed(self, path, visible):
        self.latest = Path(path)
        if visible:
            self.show_latest()

    def show_latest(self):
        if self.latest is not None:
            if self.selected != self.latest:
                self.calculate(self.latest)
        else:
            self.status.configure(text="No completed loaded run in this session. Open saved run to analyse a recording.")

    def open_run(self):
        path = filedialog.askopenfilename(parent=self.parent, title="Open loaded run metadata",
                                          filetypes=[("EAST metadata", "*_metadata.json")])
        if path:
            self.calculate(Path(path))

    def choose_reference(self):
        if self.selected is None:
            self.status.configure(text="Open a loaded run first.")
            return
        path = filedialog.askopenfilename(parent=self.parent, title="Choose matching empty fixture metadata",
                                          initialdir=self.selected.parent,
                                          filetypes=[("EAST metadata", "*_metadata.json")])
        if path:
            self.calculate(self.selected, Path(path))

    def calculate(self, path, reference=None):
        self.selected = path
        self.result = None
        self.export_button.configure(state="disabled")
        self.table.delete(*self.table.get_children())
        self.draw()
        self.status.configure(text=f"Analysing {path.name}")
        self.request += 1
        token = self.request

        def worker():
            try:
                result = analyse_stiffness(path, reference)
            except (OSError, ValueError, KeyError, TypeError) as error:
                self.replies.put((token, None, str(error)))
            else:
                self.replies.put((token, result, None))

        threading.Thread(target=worker, daemon=True).start()
        if token == 1:
            self.parent.after(100, self.poll)

    def poll(self):
        while True:
            try:
                token, result, error = self.replies.get_nowait()
            except queue.Empty:
                break
            if token != self.request:
                continue
            if error is not None:
                self.status.configure(text=f"{self.selected.name}\n{error}")
                continue
            self.result = result
            for row in result["branches"]:
                low, high = row["fit_window_deg"]
                self.table.insert("", "end", values=(row["direction"], f"{low:+} to {high:+}",
                                 f"{row['mean_nm_per_deg']:.4f}", f"{row['sd_nm_per_deg']:.4f}"))
            reference = result["reference_name"] or Path(result["empty_metadata"]).name
            excluded = result["excluded_cycles"]
            self.status.configure(text=f"AFO {result['specimen_id']} · {result['requested_speed_deg_s']:g}°/s requested\n"
                                  f"Reference: {reference}\n"
                                  f"Analysed cycles: {', '.join(map(str, result['cycles']))}"
                                  + (f" · Excluded for sweep coverage: {excluded}" if excluded else ""))
            self.export_button.configure(state="normal")
            self.draw()
        self.parent.after(120, self.poll)

    def export(self):
        if self.result is None:
            return
        path = filedialog.asksaveasfilename(parent=self.parent, title="Export stiffness results",
                                            initialdir=self.selected.parent,
                                            initialfile=self.selected.stem.replace("_metadata", "") + "_stiffness.json",
                                            defaultextension=".json", filetypes=[("Stiffness summary", "*.json")])
        if path:
            try:
                with Path(path).open("w", encoding="utf-8") as stream:
                    json.dump(self.result, stream, indent=2, allow_nan=False)
            except OSError as error:
                self.status.configure(text=f"Export failed: {error}")
            else:
                self.status.configure(text=f"Stiffness results saved to {path}")

    def draw(self):
        canvas = self.canvas
        canvas.delete("all")
        if self.result is None:
            return
        width = max(canvas.winfo_width(), 300)
        height = max(canvas.winfo_height(), 200)
        traces = [self.result[key] for key in ("fitted_torque_nm", "empty_torque_nm", "corrected_torque_nm")]
        values = [-value for pair in traces for curve in pair for value in curve]
        limit = max(max(map(abs, values)) * 1.05, 0.01)
        left, right, top, bottom = 58, width - 14, 34, height - 48

        def x(angle):
            return left + (angle + 8) / 16 * (right - left)

        def y(torque):
            return top + (limit - torque) / (2 * limit) * (bottom - top)

        for low, high in WINDOWS:
            canvas.create_rectangle(x(low), top, x(high), bottom, fill="#e6f3eb", outline="")
        for level in (-limit, -limit / 2, 0, limit / 2, limit):
            canvas.create_line(left, y(level), right, y(level), fill="#cbd5e1")
            canvas.create_text(left - 5, y(level), text=f"{level:.2g}", anchor="e", font=("Arial", 9))
        for angle in (-8, -4, 0, 4, 8):
            canvas.create_text(x(angle), bottom + 12, text=f"{angle:g}", font=("Arial", 9))
        canvas.create_text(14, (top + bottom) / 2, text="Resisting torque (Nm)", angle=90, font=("Arial", 9))
        canvas.create_text((left + right) / 2, bottom + 27, text="Measured rotary angle (°)", font=("Arial", 9))
        canvas.create_text((left + right) / 2, height - 8, text="Solid increasing · Dashed decreasing", font=("Arial", 8))

        for i, (pair, label, colour) in enumerate(zip(traces, ("Fitted", "Empty", "Corrected"),
                                                     ("#2559a5", "#ad5a20", "#167455"))):
            canvas.create_text(left + i * (right - left) / 3, 15, text=label, anchor="w",
                               fill=colour, font=("Arial", 10, "bold"))
            for index, curve in enumerate(pair):
                points = [coordinate for a, t in zip(self.result["angles_deg"], curve)
                          for coordinate in (x(a), y(-t))]
                canvas.create_line(*points, fill=colour, width=2, dash=(5, 3) if index else ())
