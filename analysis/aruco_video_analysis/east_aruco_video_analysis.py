#!/usr/bin/env python3
"""Measure EAST angular motion from four fixed ArUco markers and two colour targets.

The fixed A4 board supplies a per-frame planar transform. Yellow and magenta
targets are attached to the same rigid moving member. Their connecting line
supplies orientation, so translation of the mechanism does not change angle.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path
from statistics import median

import cv2
import numpy as np

MARKER_SIZE_MM = 45.0
MARKER_TOP_LEFT_MM = {
    0: (15.0, 20.0),
    1: (150.0, 20.0),
    2: (150.0, 232.0),
    3: (15.0, 232.0),
}


@dataclass
class ColourRange:
    hue_min: int
    hue_max: int
    sat_min: int = 80
    val_min: int = 70


DEFAULT_MAGENTA = ColourRange(140, 179)
DEFAULT_YELLOW = ColourRange(18, 40)


def marker_world_corners(marker_id: int) -> np.ndarray:
    x, y = MARKER_TOP_LEFT_MM[marker_id]
    s = MARKER_SIZE_MM
    return np.asarray([(x, y), (x + s, y), (x + s, y + s), (x, y + s)], dtype=np.float32)


def load_reference_centres(path: str | None) -> dict[int, tuple[float, float]] | None:
    if not path:
        return None
    payload = json.loads(Path(path).expanduser().read_text(encoding="utf-8"))
    markers = payload.get("marker_centres_mm", payload)
    result = {int(marker_id): (float(point[0]), float(point[1])) for marker_id, point in markers.items()}
    if set(result) != {0, 1, 2, 3}:
        raise ValueError("reference layout must define marker centres for IDs 0, 1, 2 and 3")
    return result


def largest_colour_centroid(hsv: np.ndarray, bounds: ColourRange, min_area: float) -> tuple[tuple[float, float] | None, float, np.ndarray]:
    low = np.array([bounds.hue_min, bounds.sat_min, bounds.val_min], dtype=np.uint8)
    high = np.array([bounds.hue_max, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, low, high)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, 0.0, mask
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    moments = cv2.moments(contour)
    if area < min_area or moments["m00"] == 0:
        return None, area, mask
    return (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]), area, mask


def make_homography(corners, ids, reference_centres=None) -> tuple[np.ndarray | None, list[int]]:
    if ids is None:
        return None, []
    image_points: list[np.ndarray] = []
    board_points: list[np.ndarray] = []
    seen: list[int] = []
    for marker_corners, marker_id in zip(corners, ids.flatten()):
        marker_id = int(marker_id)
        if marker_id not in MARKER_TOP_LEFT_MM:
            continue
        detected = np.asarray(marker_corners, dtype=np.float32).reshape(4, 2)
        if reference_centres is None:
            image_points.append(detected)
            board_points.append(marker_world_corners(marker_id))
        else:
            image_points.append(detected.mean(axis=0, keepdims=True))
            board_points.append(np.asarray([reference_centres[marker_id]], dtype=np.float32))
        seen.append(marker_id)
    required = 4 if reference_centres is not None else 3
    if len(seen) < required:
        return None, sorted(seen)
    h, _ = cv2.findHomography(np.vstack(image_points), np.vstack(board_points), cv2.RANSAC, 2.5)
    return h, sorted(seen)


def transform_point(point: tuple[float, float], homography: np.ndarray) -> tuple[float, float]:
    src = np.asarray([[[point[0], point[1]]]], dtype=np.float32)
    dst = cv2.perspectiveTransform(src, homography)[0, 0]
    return float(dst[0]), float(dst[1])


def circular_median_deg(values: list[float]) -> float:
    radians = np.radians(values)
    return math.degrees(math.atan2(float(np.median(np.sin(radians))), float(np.median(np.cos(radians)))))


def unwrap_degrees(values: list[float]) -> list[float]:
    return list(np.degrees(np.unwrap(np.radians(np.asarray(values, dtype=float)))))


def fit_speed(rows: list[dict], lower: float, upper: float, minimum_points: int = 8) -> list[dict]:
    """Fit contiguous sweeps through the central angle window."""
    valid = [r for r in rows if r["valid"]]
    if len(valid) < minimum_points:
        return []
    t = np.asarray([r["time_s"] for r in valid], dtype=float)
    a = np.asarray([r["angle_deg"] for r in valid], dtype=float)
    inside = (a >= lower) & (a <= upper)
    # Determine sweep direction from a short centred trend. This prevents
    # sub-pixel colour-centroid jitter from breaking one physical sweep into
    # many tiny fragments.
    window = min(9, len(a) if len(a) % 2 else len(a) - 1)
    window = max(window, 3)
    smooth = np.convolve(a, np.ones(window) / window, mode="same")
    lag = min(5, max(1, len(a) // 20))
    trend = np.empty_like(smooth)
    trend[:lag] = smooth[lag] - smooth[0]
    trend[lag:] = smooth[lag:] - smooth[:-lag]
    direction = np.sign(trend)
    # Carry the last clear direction across stationary/jitter samples.
    for i in range(1, len(direction)):
        if direction[i] == 0:
            direction[i] = direction[i - 1]
    fits: list[dict] = []
    start = None
    current_sign = None
    for i in range(len(a)):
        sign = direction[i]
        if inside[i] and not np.isnan(sign):
            if start is None:
                start, current_sign = i, sign
            elif sign != current_sign:
                if i - start >= minimum_points:
                    fits.append(_linear_fit(t[start:i], a[start:i], len(fits) + 1))
                start, current_sign = i, sign
        elif start is not None:
            if i - start >= minimum_points:
                fits.append(_linear_fit(t[start:i], a[start:i], len(fits) + 1))
            start, current_sign = None, None
    if start is not None and len(a) - start >= minimum_points:
        fits.append(_linear_fit(t[start:], a[start:], len(fits) + 1))
    return [fit for fit in fits if abs(fit["angle_span_deg"]) >= (upper - lower) * 0.75]


def _linear_fit(t: np.ndarray, a: np.ndarray, number: int) -> dict:
    slope, intercept = np.polyfit(t, a, 1)
    predicted = slope * t + intercept
    residual = float(np.sum((a - predicted) ** 2))
    total = float(np.sum((a - np.mean(a)) ** 2))
    return {
        "sweep": number,
        "direction": "increasing" if slope > 0 else "decreasing",
        "start_s": float(t[0]),
        "end_s": float(t[-1]),
        "points": int(len(t)),
        "angle_span_deg": float(a[-1] - a[0]),
        "signed_speed_deg_s": float(slope),
        "speed_deg_s": float(abs(slope)),
        "r_squared": float(1.0 - residual / total) if total > 0 else 1.0,
    }


def analyse(args: argparse.Namespace) -> None:
    video_path = Path(args.video).expanduser().resolve()
    output_dir = Path(args.output).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    capture = cv2.VideoCapture(str(video_path))
    if not capture.isOpened():
        raise SystemExit(f"Could not open video: {video_path}")
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    if not math.isfinite(fps) or fps <= 0:
        raise SystemExit("Video does not report a valid frame rate.")

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    reference_centres = load_reference_centres(args.reference_layout)
    magenta = ColourRange(args.magenta_hue_min, args.magenta_hue_max, args.saturation_min, args.value_min)
    yellow = ColourRange(args.yellow_hue_min, args.yellow_hue_max, args.saturation_min, args.value_min)

    rows: list[dict] = []
    raw_angles: list[float] = []
    frame_index = 0
    last_h = None
    overlay_writer = None
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)
        homography, seen = make_homography(corners, ids, reference_centres)
        if homography is not None:
            last_h = homography
        elif args.allow_reference_hold and last_h is not None:
            homography = last_h

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        magenta_px, magenta_area, _ = largest_colour_centroid(hsv, magenta, args.min_colour_area)
        yellow_px, yellow_area, _ = largest_colour_centroid(hsv, yellow, args.min_colour_area)
        valid = homography is not None and magenta_px is not None and yellow_px is not None and (len(seen) == 4 or args.allow_reference_hold)
        yellow_mm = magenta_mm = (math.nan, math.nan)
        raw_angle = math.nan
        if valid:
            yellow_mm = transform_point(yellow_px, homography)
            magenta_mm = transform_point(magenta_px, homography)
            dx = magenta_mm[0] - yellow_mm[0]
            dy = magenta_mm[1] - yellow_mm[1]
            separation = math.hypot(dx, dy)
            if separation < args.minimum_separation_mm:
                valid = False
            else:
                raw_angle = math.degrees(math.atan2(-dy, dx))
                raw_angles.append(raw_angle)
        rows.append({
            "frame": frame_index,
            "time_s": frame_index / fps,
            "valid": bool(valid),
            "aruco_ids": "|".join(map(str, seen)),
            "yellow_x_mm": yellow_mm[0], "yellow_y_mm": yellow_mm[1],
            "magenta_x_mm": magenta_mm[0], "magenta_y_mm": magenta_mm[1],
            "yellow_area_px": yellow_area, "magenta_area_px": magenta_area,
            "raw_orientation_deg": raw_angle,
        })

        if args.overlay:
            if overlay_writer is None:
                h_px, w_px = frame.shape[:2]
                overlay_writer = cv2.VideoWriter(str(output_dir / "detection_overlay.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w_px, h_px))
            display = frame.copy()
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(display, corners, ids)
            if yellow_px:
                cv2.circle(display, tuple(map(int, yellow_px)), 10, (0, 255, 255), 3)
            if magenta_px:
                cv2.circle(display, tuple(map(int, magenta_px)), 10, (255, 0, 255), 3)
            if yellow_px and magenta_px:
                cv2.line(display, tuple(map(int, yellow_px)), tuple(map(int, magenta_px)), (255, 255, 255), 2)
            cv2.putText(display, f"frame {frame_index} | ArUco {len(seen)}/4 | {'valid' if valid else 'INVALID'}", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 0) if valid else (0, 0, 255), 2)
            overlay_writer.write(display)
        frame_index += 1

    capture.release()
    if overlay_writer:
        overlay_writer.release()
    valid_rows = [r for r in rows if r["valid"]]
    if not valid_rows:
        raise SystemExit("No valid frames. Inspect lighting, ArUco visibility, and colour thresholds.")
    unwrapped = unwrap_degrees([r["raw_orientation_deg"] for r in valid_rows])
    for row, angle in zip(valid_rows, unwrapped):
        row["unwrapped_orientation_deg"] = angle
    neutral_limit = args.neutral_seconds
    neutral_values = [r["unwrapped_orientation_deg"] for r in valid_rows if r["time_s"] <= neutral_limit]
    if len(neutral_values) < max(5, int(fps * neutral_limit * 0.25)):
        raise SystemExit("Too few valid frames in the neutral period. Record at least 2 stationary seconds at machine zero.")
    neutral = float(median(neutral_values))
    sign = 1.0 if args.dorsiflexion_direction == "increasing" else -1.0
    for row in rows:
        if row["valid"]:
            row["angle_deg"] = sign * (row["unwrapped_orientation_deg"] - neutral)
        else:
            row["unwrapped_orientation_deg"] = math.nan
            row["angle_deg"] = math.nan

    fields = list(rows[0].keys())
    with (output_dir / "frame_data.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    sweeps = fit_speed(rows, -args.fit_limit_deg, args.fit_limit_deg)
    with (output_dir / "sweep_summary.csv").open("w", newline="", encoding="utf-8") as handle:
        fields = ["sweep", "direction", "start_s", "end_s", "points", "angle_span_deg", "signed_speed_deg_s", "speed_deg_s", "r_squared"]
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(sweeps)

    speeds = [s["speed_deg_s"] for s in sweeps]
    summary = {
        "video": str(video_path),
        "fps": fps,
        "frames": len(rows),
        "valid_frames": len(valid_rows),
        "valid_frame_percent": 100.0 * len(valid_rows) / len(rows),
        "neutral_seconds": args.neutral_seconds,
        "neutral_orientation_deg": neutral,
        "dorsiflexion_direction": args.dorsiflexion_direction,
        "fit_window_deg": [-args.fit_limit_deg, args.fit_limit_deg],
        "sweeps_found": len(sweeps),
        "mean_speed_deg_s": float(np.mean(speeds)) if speeds else None,
        "minimum_sweep_r_squared": min((s["r_squared"] for s in sweeps), default=None),
        "qc_pass": len(valid_rows) / len(rows) >= args.minimum_valid_fraction and bool(sweeps),
        "colour_thresholds": {"magenta": vars(magenta), "yellow": vars(yellow)},
    }
    (output_dir / "analysis_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("video", help="Input MOV or MP4 video")
    p.add_argument("--output", required=True, help="Output directory")
    p.add_argument("--reference-layout", help="JSON file containing measured centre coordinates for separately mounted IDs 0-3")
    p.add_argument("--neutral-seconds", type=float, default=2.0, help="Initial stationary machine-zero interval")
    p.add_argument("--dorsiflexion-direction", choices=("increasing", "decreasing"), default="increasing")
    p.add_argument("--fit-limit-deg", type=float, default=8.0, help="Fit speeds inside ± this angle")
    p.add_argument("--min-colour-area", type=float, default=120.0)
    p.add_argument("--minimum-separation-mm", type=float, default=60.0)
    p.add_argument("--minimum-valid-fraction", type=float, default=0.95)
    p.add_argument("--magenta-hue-min", type=int, default=DEFAULT_MAGENTA.hue_min)
    p.add_argument("--magenta-hue-max", type=int, default=DEFAULT_MAGENTA.hue_max)
    p.add_argument("--yellow-hue-min", type=int, default=DEFAULT_YELLOW.hue_min)
    p.add_argument("--yellow-hue-max", type=int, default=DEFAULT_YELLOW.hue_max)
    p.add_argument("--saturation-min", type=int, default=80)
    p.add_argument("--value-min", type=int, default=70)
    p.add_argument("--overlay", action="store_true", help="Write a detection-overlay MP4 for visual QC")
    p.add_argument("--allow-reference-hold", action="store_true", help="Reuse the last transform if a fixed marker is briefly missed; exploratory use only")
    return p


if __name__ == "__main__":
    analyse(parser().parse_args())
