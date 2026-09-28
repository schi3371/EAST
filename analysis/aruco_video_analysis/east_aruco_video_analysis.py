#!/usr/bin/env python3
"""Measure EAST angular motion from fixed ArUcos and two colour targets.

The formal path uses four fixed ArUcos and a measured reference layout for a
per-frame planar transform. The exploratory marker-only path uses the raw
yellow-to-magenta image vector and does not require ArUco detection. A hybrid
target detector finds each large colour region and then the small black centre
dot inside it, while retaining both measurements for quality control.
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


@dataclass
class TargetDetection:
    colour_centroid: tuple[float, float] | None
    dot_centroid: tuple[float, float] | None
    selected_centroid: tuple[float, float] | None
    selected_source: str
    colour_area_px: float
    dot_area_px: float
    dot_colour_disagreement_px: float
    mask: np.ndarray


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


def detect_target(
    frame: np.ndarray,
    hsv: np.ndarray,
    bounds: ColourRange,
    min_area: float,
    point_source: str,
    black_value_max: int,
    minimum_dot_area_px: float,
    maximum_dot_area_fraction: float,
    maximum_dot_offset_fraction: float,
) -> TargetDetection:
    low = np.array([bounds.hue_min, bounds.sat_min, bounds.val_min], dtype=np.uint8)
    high = np.array([bounds.hue_max, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, low, high)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return TargetDetection(None, None, None, "missing", 0.0, 0.0, math.nan, mask)
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    moments = cv2.moments(contour)
    if area < min_area or moments["m00"] == 0:
        return TargetDetection(None, None, None, "missing", area, 0.0, math.nan, mask)
    colour = (moments["m10"] / moments["m00"], moments["m01"] / moments["m00"])

    # Search for a compact dark component near the centre of the coloured
    # target. Restricting the search to the filled colour contour prevents the
    # printed outer outline and nearby dark machine parts becoming candidates.
    filled = np.zeros(mask.shape, dtype=np.uint8)
    cv2.drawContours(filled, [contour], -1, 255, thickness=cv2.FILLED)
    equivalent_radius = math.sqrt(area / math.pi)
    central = np.zeros(mask.shape, dtype=np.uint8)
    cv2.circle(
        central,
        (int(round(colour[0])), int(round(colour[1]))),
        max(2, int(round(equivalent_radius * maximum_dot_offset_fraction))),
        255,
        thickness=cv2.FILLED,
    )
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    search_mask = cv2.bitwise_and(filled, central)
    search_values = gray[search_mask > 0]
    # Use contrast against the coloured paper, because the tiny black dot is
    # strongly antialiased at long camera distances (the yellow dot can be
    # grey level 100-130 even though the magenta dot remains below 80).
    local_background = float(np.median(search_values)) if search_values.size else 255.0
    adaptive_dark_max = int(max(0, min(black_value_max, local_background - 25.0)))
    dark = cv2.inRange(gray, 0, adaptive_dark_max)
    dot_mask = cv2.bitwise_and(dark, search_mask)
    # The printed centre dots can be only 2-5 pixels across in a distant 4K
    # recording.  A 3x3 opening erased them, so use connected pixel area here
    # rather than contour area and do not morphologically filter the dot mask.
    component_count, _, component_stats, component_centres = cv2.connectedComponentsWithStats(
        dot_mask, connectivity=8
    )
    candidates = []
    for component_index in range(1, component_count):
        dot_area = float(component_stats[component_index, cv2.CC_STAT_AREA])
        if not (minimum_dot_area_px <= dot_area <= area * maximum_dot_area_fraction):
            continue
        point = tuple(float(value) for value in component_centres[component_index])
        offset = math.hypot(point[0] - colour[0], point[1] - colour[1])
        candidates.append((offset, -dot_area, point, dot_area))
    candidates.sort()
    dot = candidates[0][2] if candidates else None
    dot_area = candidates[0][3] if candidates else 0.0
    disagreement = candidates[0][0] if candidates else math.nan

    if point_source == "colour":
        selected, source = colour, "colour"
    elif point_source == "dot":
        selected, source = dot, "dot" if dot is not None else "missing"
    else:
        selected = dot if dot is not None else colour
        source = "dot" if dot is not None else "colour_fallback"
    return TargetDetection(colour, dot, selected, source, area, dot_area, disagreement, mask)


def make_homography(corners, ids, reference_centres=None, minimum_markers: int = 4) -> tuple[np.ndarray | None, list[int], str]:
    if ids is None:
        return None, [], "none"
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
    if len(seen) < minimum_markers:
        return None, sorted(seen), "none"
    image = np.vstack(image_points)
    board = np.vstack(board_points)
    if reference_centres is not None and len(seen) == 3:
        affine, _ = cv2.estimateAffine2D(image, board, method=cv2.RANSAC, ransacReprojThreshold=2.5)
        if affine is None:
            return None, sorted(seen), "none"
        h = np.vstack([affine, [0.0, 0.0, 1.0]])
        return h, sorted(seen), "three_marker_affine"
    h, _ = cv2.findHomography(image, board, cv2.RANSAC, 2.5)
    return h, sorted(seen), "four_marker_homography" if len(seen) >= 4 else "legacy_homography"


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

    detector = None
    reference_centres = None
    if args.reference_mode == "aruco":
        if not args.reference_layout:
            raise SystemExit("--reference-layout is required when --reference-mode aruco is used")
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        reference_centres = load_reference_centres(args.reference_layout)
    magenta = ColourRange(args.magenta_hue_min, args.magenta_hue_max, args.saturation_min, args.value_min)
    yellow = ColourRange(args.yellow_hue_min, args.yellow_hue_max, args.saturation_min, args.value_min)

    rows: list[dict] = []
    frame_index = 0
    last_h = None
    last_h_method = "none"
    overlay_writer = None
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        corners, ids, seen = [], None, []
        homography = None
        homography_method = "marker_only_raw_pixels"
        reference_held = False
        if args.reference_mode == "aruco":
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detector.detectMarkers(gray)
            homography, seen, homography_method = make_homography(
                corners, ids, reference_centres, args.minimum_aruco_markers
            )
            if homography is not None:
                last_h = homography
                last_h_method = homography_method
            elif args.allow_reference_hold and last_h is not None:
                homography = last_h
                homography_method = f"held_{last_h_method}"
                reference_held = True

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        magenta_detection = detect_target(
            frame, hsv, magenta, args.min_colour_area, args.point_source,
            args.black_value_max, args.minimum_dot_area_px,
            args.maximum_dot_area_fraction, args.maximum_dot_offset_fraction,
        )
        yellow_detection = detect_target(
            frame, hsv, yellow, args.min_colour_area, args.point_source,
            args.black_value_max, args.minimum_dot_area_px,
            args.maximum_dot_area_fraction, args.maximum_dot_offset_fraction,
        )
        magenta_px = magenta_detection.selected_centroid
        yellow_px = yellow_detection.selected_centroid
        dots_present = magenta_detection.dot_centroid is not None and yellow_detection.dot_centroid is not None
        valid_reference = args.reference_mode == "marker-only" or homography is not None
        valid = valid_reference and magenta_px is not None and yellow_px is not None
        if args.require_black_dots and not dots_present:
            valid = False
        yellow_measure = magenta_measure = (math.nan, math.nan)
        raw_angle = math.nan
        separation = math.nan
        if valid:
            if args.reference_mode == "aruco":
                yellow_measure = transform_point(yellow_px, homography)
                magenta_measure = transform_point(magenta_px, homography)
                minimum_separation = args.minimum_separation_mm
            else:
                yellow_measure = yellow_px
                magenta_measure = magenta_px
                minimum_separation = args.minimum_separation_px
            dx = magenta_measure[0] - yellow_measure[0]
            dy = magenta_measure[1] - yellow_measure[1]
            separation = math.hypot(dx, dy)
            if separation < minimum_separation:
                valid = False
            else:
                raw_angle = math.degrees(math.atan2(-dy, dx))
        rows.append({
            "frame": frame_index,
            "time_s": frame_index / fps,
            "valid": bool(valid),
            "reference_mode": args.reference_mode,
            "aruco_ids": "|".join(map(str, seen)),
            "reference_transform": homography_method,
            "reference_held": reference_held,
            "yellow_selected_x": yellow_measure[0], "yellow_selected_y": yellow_measure[1],
            "magenta_selected_x": magenta_measure[0], "magenta_selected_y": magenta_measure[1],
            "coordinate_unit": "mm" if args.reference_mode == "aruco" else "pixel",
            "target_separation": separation,
            "yellow_colour_x_px": yellow_detection.colour_centroid[0] if yellow_detection.colour_centroid else math.nan,
            "yellow_colour_y_px": yellow_detection.colour_centroid[1] if yellow_detection.colour_centroid else math.nan,
            "yellow_dot_x_px": yellow_detection.dot_centroid[0] if yellow_detection.dot_centroid else math.nan,
            "yellow_dot_y_px": yellow_detection.dot_centroid[1] if yellow_detection.dot_centroid else math.nan,
            "yellow_point_source": yellow_detection.selected_source,
            "yellow_colour_area_px": yellow_detection.colour_area_px,
            "yellow_dot_area_px": yellow_detection.dot_area_px,
            "yellow_dot_colour_disagreement_px": yellow_detection.dot_colour_disagreement_px,
            "magenta_colour_x_px": magenta_detection.colour_centroid[0] if magenta_detection.colour_centroid else math.nan,
            "magenta_colour_y_px": magenta_detection.colour_centroid[1] if magenta_detection.colour_centroid else math.nan,
            "magenta_dot_x_px": magenta_detection.dot_centroid[0] if magenta_detection.dot_centroid else math.nan,
            "magenta_dot_y_px": magenta_detection.dot_centroid[1] if magenta_detection.dot_centroid else math.nan,
            "magenta_point_source": magenta_detection.selected_source,
            "magenta_colour_area_px": magenta_detection.colour_area_px,
            "magenta_dot_area_px": magenta_detection.dot_area_px,
            "magenta_dot_colour_disagreement_px": magenta_detection.dot_colour_disagreement_px,
            "raw_orientation_deg": raw_angle,
        })

        if args.overlay:
            if overlay_writer is None:
                h_px, w_px = frame.shape[:2]
                overlay_writer = cv2.VideoWriter(str(output_dir / "detection_overlay.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w_px, h_px))
            display = frame.copy()
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(display, corners, ids)
            if yellow_detection.colour_centroid:
                cv2.circle(display, tuple(map(int, yellow_detection.colour_centroid)), 12, (0, 180, 255), 2)
            if magenta_detection.colour_centroid:
                cv2.circle(display, tuple(map(int, magenta_detection.colour_centroid)), 12, (180, 0, 180), 2)
            if yellow_detection.dot_centroid:
                cv2.drawMarker(display, tuple(map(int, yellow_detection.dot_centroid)), (255, 255, 255), cv2.MARKER_CROSS, 16, 2)
            if magenta_detection.dot_centroid:
                cv2.drawMarker(display, tuple(map(int, magenta_detection.dot_centroid)), (255, 255, 255), cv2.MARKER_CROSS, 16, 2)
            if yellow_px:
                cv2.circle(display, tuple(map(int, yellow_px)), 10, (0, 255, 255), 3)
            if magenta_px:
                cv2.circle(display, tuple(map(int, magenta_px)), 10, (255, 0, 255), 3)
            if yellow_px and magenta_px:
                cv2.line(display, tuple(map(int, yellow_px)), tuple(map(int, magenta_px)), (255, 255, 255), 2)
            status = f"frame {frame_index} | {args.reference_mode} | ArUco {len(seen)}/4 | {'valid' if valid else 'INVALID'}"
            cv2.putText(display, status, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 220, 0) if valid else (0, 0, 255), 2)
            cv2.putText(display, f"Y:{yellow_detection.selected_source} M:{magenta_detection.selected_source}", (20, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
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
    yellow_dot_frames = sum(math.isfinite(float(r["yellow_dot_x_px"])) for r in rows)
    magenta_dot_frames = sum(math.isfinite(float(r["magenta_dot_x_px"])) for r in rows)
    four_marker_frames = sum(r["aruco_ids"].split("|") == ["0", "1", "2", "3"] for r in rows)
    reference_held_frames = sum(bool(r["reference_held"]) for r in rows)
    colour_fallback_frames = sum(
        r["yellow_point_source"] == "colour_fallback" or r["magenta_point_source"] == "colour_fallback"
        for r in rows
    )
    formal_reference = (
        args.reference_mode == "aruco"
        and args.minimum_aruco_markers == 4
        and not args.allow_reference_hold
    )
    summary = {
        "video": str(video_path),
        "analysis_classification": "formal_candidate" if formal_reference else "exploratory",
        "reference_mode": args.reference_mode,
        "point_source": args.point_source,
        "require_black_dots": args.require_black_dots,
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
        "yellow_dot_detection_percent": 100.0 * yellow_dot_frames / len(rows),
        "magenta_dot_detection_percent": 100.0 * magenta_dot_frames / len(rows),
        "four_aruco_detection_percent": 100.0 * four_marker_frames / len(rows),
        "reference_held_frames": reference_held_frames,
        "colour_fallback_frames": colour_fallback_frames,
        "qc_pass": (
            len(valid_rows) / len(rows) >= args.minimum_valid_fraction
            and bool(sweeps)
            and (not args.require_black_dots or (yellow_dot_frames == len(rows) and magenta_dot_frames == len(rows)))
        ),
        "colour_thresholds": {"magenta": vars(magenta), "yellow": vars(yellow)},
    }
    (output_dir / "analysis_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("video", help="Input MOV or MP4 video")
    p.add_argument("--output", required=True, help="Output directory")
    p.add_argument("--reference-mode", choices=("aruco", "marker-only"), default="aruco", help="Use ArUco plane correction or exploratory raw yellow-magenta pixels")
    p.add_argument("--reference-layout", help="JSON file containing measured centre coordinates for separately mounted IDs 0-3")
    p.add_argument("--minimum-aruco-markers", type=int, choices=(3, 4), default=4, help="Three-marker affine mode is exploratory; four is required for the formal path")
    p.add_argument("--neutral-seconds", type=float, default=2.0, help="Initial stationary machine-zero interval")
    p.add_argument("--dorsiflexion-direction", choices=("increasing", "decreasing"), default="increasing")
    p.add_argument("--fit-limit-deg", type=float, default=8.0, help="Fit speeds inside ± this angle")
    p.add_argument("--min-colour-area", type=float, default=120.0)
    p.add_argument("--minimum-separation-mm", type=float, default=60.0)
    p.add_argument("--minimum-separation-px", type=float, default=80.0)
    p.add_argument("--point-source", choices=("hybrid", "dot", "colour"), default="hybrid", help="Hybrid prefers the black centre dot and falls back to the colour centroid")
    p.add_argument("--require-black-dots", action="store_true", help="Reject frames unless both black centre dots are detected")
    p.add_argument("--black-value-max", type=int, default=200, help="Absolute cap for the adaptive dark-dot threshold")
    p.add_argument("--minimum-dot-area-px", type=float, default=2.0)
    p.add_argument("--maximum-dot-area-fraction", type=float, default=0.12)
    p.add_argument("--maximum-dot-offset-fraction", type=float, default=0.55)
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
