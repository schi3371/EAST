"""Offline SOP capture from a recorded three-speed manifest and strain CSVs."""
import csv
import json
import math
from pathlib import Path, PureWindowsPath
import subprocess
from datetime import datetime
import tkinter as tk
from tkinter import filedialog, messagebox

import customtkinter as ctk


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


class ReplayWindow:
    def __init__(self, root, path, config, output):
        self.root, self.output = root, output
        self.manifest, self.runs = read_recording(path)
        self.preset = config["test_presets"][
            "three_speed_empty_baseline" if self.manifest["test_type"] ==
            "empty_machine_baseline" else "three_speed_afo"
        ]
        root.title("EAST — SOP replay")
        root.geometry("1100x650")
        root.minsize(900, 560)
        tools = ctk.CTkFrame(root)
        tools.pack(fill="x", padx=12, pady=8)
        self.views = ("Setup", "Confirmation", "Recorded plot", "Recorded outcome")
        self.view = ctk.CTkOptionMenu(tools, values=self.views, command=self.render)
        self.view.pack(side="left", padx=8, pady=6)
        self.run = ctk.CTkOptionMenu(
            tools, values=[str(i + 1) for i in range(len(self.runs))],
            command=lambda _: self.render(self.view.get())
        )
        self.run.pack(side="left", padx=8)
        ctk.CTkButton(tools, text="Capture PNG", command=self.capture).pack(side="right", padx=8)
        ctk.CTkButton(tools, text="Capture all views", command=self.capture_all).pack(side="right", padx=8)
        self.frame = ctk.CTkFrame(root, fg_color="#f8fafc")
        self.frame.pack(fill="both", expand=True, padx=12, pady=(0, 12))
        self.view.set("Recorded plot")
        self.render("Recorded plot")

    def render(self, view):
        for child in self.frame.winfo_children():
            child.destroy()
        manifest = self.manifest
        example = view in self.views[:2]
        run, points = self.runs[int(self.run.get()) - 1]
        heading = ctk.CTkLabel(self.frame, text="EAST  |  " + view,
                               font=("Arial", 26, "bold"))
        heading.pack(anchor="w", padx=22, pady=(18, 4))
        identity = manifest["identity"]
        source = ("Illustrative setup" if example else "Recorded-data replay")
        ctk.CTkLabel(self.frame, text=f"{source}  ·  No hardware connected",
                     text_color="#1d4ed8").pack(anchor="w", padx=22)
        ctk.CTkLabel(self.frame, text=f"Source batch {manifest['batch_id']}").pack(anchor="w", padx=22)
        details = "\n".join(f"{label}  {identity.get(key) or '—'}" for label, key in (
            ("Operator", "operator"), ("AFO", "afo_id"), ("Fixture", "fixture_id"),
            ("Calibration", "calibration_id"),
        ))
        ctk.CTkLabel(self.frame, text=details, justify="left").pack(anchor="w", padx=22, pady=8)
        if example:
            p = self.preset
            description = (
                f"{p['display_name']}  ·  v{p['version']}\n"
                f"{' → '.join(f'{speed:g}' for speed in p['speeds_deg_s'])} °/s\n"
                f"−{p['minimum_angle_deg']:g}° to +{p['maximum_angle_deg']:g}°  ·  "
                f"{p['acceleration_deg_s2']:g} °/s²  ·  {p['cycles']} cycles per speed\n"
                f"{3 * p['cycles']} cycles total"
            )
            ctk.CTkLabel(self.frame, text=description, justify="left",
                         font=("Arial", 17)).pack(anchor="w", padx=22, pady=12)
            names = ("Connect", "Home / Return to Zero", "Tare Empty Machine", "Load Preset", "Start", "Stop")
            if view == "Confirmation":
                names = ("Cancel", "Run Three-Speed Sequence")
            for name in names:
                ctk.CTkButton(self.frame, text=name, state="disabled").pack(anchor="w", padx=22, pady=3)
        elif view == "Recorded outcome":
            lines = [f"Recorded series outcome  {manifest['outcome']}"]
            for record in manifest["runs"]:
                ctx = record.get("context", {})
                lines.append(
                    f"{ctx.get('planned_speed_deg_s', '—')} °/s  ·  "
                    f"{record.get('completed_cycles') or 0} cycles  ·  {record.get('status', 'unknown')}"
                )
            if manifest.get("error"):
                lines.append(manifest["error"])
            ctk.CTkLabel(self.frame, text="\n".join(lines), justify="left",
                         wraplength=820, font=("Arial", 17)).pack(anchor="w", padx=22, pady=12)
        else:
            speed = run["context"]["planned_speed_deg_s"]
            ctk.CTkLabel(self.frame, text=(
                f"Recorded {manifest['test_type'].replace('_', ' ')}  ·  Run {self.run.get()}  ·  "
                f"{speed:g} °/s  ·  {run.get('completed_cycles') or 0} recorded cycles"
            )).pack(anchor="w", padx=22)
            canvas = tk.Canvas(self.frame, bg="white", highlightthickness=0)
            canvas.pack(fill="both", expand=True, padx=22, pady=14)
            canvas.bind("<Configure>", lambda event: self.plot(canvas, points, event.width, event.height))

    @staticmethod
    def plot(canvas, points, width, height):
        canvas.delete("all")
        xs, ys = zip(*points)
        x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
        left, top, right, bottom = 80, 18, width - 24, height - 55
        for i in range(5):
            fraction = i / 4
            x = left + fraction * (right - left)
            y = bottom - fraction * (bottom - top)
            canvas.create_line(x, top, x, bottom, fill="#e2e8f0")
            canvas.create_line(left, y, right, y, fill="#e2e8f0")
            canvas.create_text(x, bottom + 13, text=f"{x0 + fraction * (x1 - x0):.1f}")
            canvas.create_text(left - 8, y, anchor="e", text=f"{y0 + fraction * (y1 - y0):.2f}")
        canvas.create_text((left + right) / 2, height - 14, text="Measured fixture angle (°)")
        canvas.create_text(19, height / 2, angle=90, text="Moving average torque (N·m)")
        coordinates = []
        for x, y in points:
            coordinates.extend((left + (x - x0) / (x1 - x0 or 1) * (right - left),
                                bottom - (y - y0) / (y1 - y0 or 1) * (bottom - top)))
        if len(coordinates) >= 4:
            canvas.create_line(*coordinates, fill="#2563eb", width=1.5)

    def capture_all(self):
        initial_view = self.view.get()
        paths = []
        for view in self.views:
            self.view.set(view)
            self.render(view)
            path = self.capture(report=False)
            if path is None:
                break
            paths.append(path)
        self.view.set(initial_view)
        self.render(initial_view)
        if paths:
            messagebox.showinfo("Saved screenshots", f"{len(paths)} PNGs in {self.output}")

    def capture(self, report=True):
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


def launch(root, repo):
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
