#!/usr/bin/env python3
"""Batch analysis of Tracker point-mass exports for EAST speed verification.

The program deliberately uses only the Python standard library so it can run
on the EAST computer without installing scientific Python packages.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Iterable, Sequence


DEFAULT_NEUTRAL_DEG = 90.0
DEFAULT_ENDPOINT_THRESHOLD_DEG = 9.0
DEFAULT_FIT_LIMIT_DEG = 8.0
DEFAULT_EXPECTED_CYCLES = 3
DEFAULT_MIN_FIT_POINTS = 30
DEFAULT_MIN_R2 = 0.995


@dataclass(frozen=True)
class TrialSpec:
    file: Path
    video_id: str
    commanded_speed_deg_s: float
    trial: str
    expected_cycles: int = DEFAULT_EXPECTED_CYCLES
    neutral_position_angle_deg: float = DEFAULT_NEUTRAL_DEG
    dorsiflexion_direction: str = "decreasing"


@dataclass
class Sample:
    time_s: float
    x: float
    y: float
    position_angle_deg: float
    frame: int | None
    afo_angle_deg: float = math.nan
    detection_angle_deg: float = math.nan


@dataclass
class Segment:
    video_id: str
    trial: str
    cycle: int
    direction: str
    start_index: int
    end_index: int
    complete: bool
    included_by_protocol: bool = False
    exclusion_reason: str = ""
    fit_start_time_s: float | None = None
    fit_end_time_s: float | None = None
    fit_points: int = 0
    slope_deg_s: float | None = None
    speed_magnitude_deg_s: float | None = None
    intercept_deg: float | None = None
    r_squared: float | None = None
    commanded_speed_deg_s: float | None = None
    absolute_error_deg_s: float | None = None
    percentage_error: float | None = None


def _normalise_header(value: str) -> str:
    return (
        value.strip()
        .lower()
        .replace(" ", "_")
        .replace("position_angle", "theta_r")
        .replace("pos_angle", "theta_r")
        .replace("θ_{r}", "theta_r")
        .replace("θr", "theta_r")
    )


def read_tracker_export(path: Path) -> tuple[list[Sample], dict]:
    """Read a Tracker export, including its optional track-name row."""
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        rows = list(csv.reader(handle))

    header_index = None
    normalised: list[str] = []
    for index, row in enumerate(rows[:10]):
        candidate = [_normalise_header(cell) for cell in row]
        if "t" in candidate and "x" in candidate and "y" in candidate and "theta_r" in candidate:
            header_index = index
            normalised = candidate
            break
    if header_index is None:
        raise ValueError(
            f"{path}: could not find a header containing t, x, y and position angle θr"
        )

    indexes = {name: normalised.index(name) for name in ("t", "x", "y", "theta_r")}
    frame_index = normalised.index("frame") if "frame" in normalised else None
    samples: list[Sample] = []
    skipped_rows = 0
    for line_number, row in enumerate(rows[header_index + 1 :], start=header_index + 2):
        if not row or not row[indexes["t"]].strip():
            continue
        try:
            sample = Sample(
                time_s=float(row[indexes["t"]]),
                x=float(row[indexes["x"]]),
                y=float(row[indexes["y"]]),
                position_angle_deg=float(row[indexes["theta_r"]]),
                frame=(int(float(row[frame_index])) if frame_index is not None and row[frame_index].strip() else None),
            )
        except (IndexError, ValueError) as exc:
            skipped_rows += 1
            continue
        values = (sample.time_s, sample.x, sample.y, sample.position_angle_deg)
        if all(math.isfinite(value) for value in values):
            samples.append(sample)
        else:
            skipped_rows += 1

    if len(samples) < 2:
        raise ValueError(f"{path}: fewer than two valid data rows")
    if any(b.time_s <= a.time_s for a, b in zip(samples, samples[1:])):
        raise ValueError(f"{path}: time values must be strictly increasing")

    time_steps = [b.time_s - a.time_s for a, b in zip(samples, samples[1:])]
    median_dt = statistics.median(time_steps)
    gaps = [dt for dt in time_steps if dt > 1.5 * median_dt]
    return samples, {
        "source_header_row": header_index + 1,
        "valid_rows": len(samples),
        "skipped_rows": skipped_rows,
        "median_sample_interval_s": median_dt,
        "estimated_frame_rate_hz": 1.0 / median_dt,
        "time_gap_count": len(gaps),
        "maximum_time_gap_s": max(gaps, default=median_dt),
    }


def rolling_median(values: Sequence[float], window: int = 5) -> list[float]:
    if window < 1 or window % 2 == 0:
        raise ValueError("rolling median window must be a positive odd number")
    radius = window // 2
    return [
        statistics.median(values[max(0, i - radius) : min(len(values), i + radius + 1)])
        for i in range(len(values))
    ]


def prepare_angles(samples: list[Sample], spec: TrialSpec) -> dict:
    if spec.dorsiflexion_direction not in {"increasing", "decreasing"}:
        raise ValueError("dorsiflexion_direction must be 'increasing' or 'decreasing'")
    sign = 1.0 if spec.dorsiflexion_direction == "increasing" else -1.0
    for sample in samples:
        sample.afo_angle_deg = sign * (
            sample.position_angle_deg - spec.neutral_position_angle_deg
        )
    smoothed = rolling_median([sample.afo_angle_deg for sample in samples])
    for sample, value in zip(samples, smoothed):
        sample.detection_angle_deg = value

    angle_differences = []
    for sample in samples:
        calculated = math.degrees(math.atan2(sample.y, sample.x))
        difference = (sample.position_angle_deg - calculated + 180.0) % 360.0 - 180.0
        angle_differences.append(abs(difference))
    return {
        "minimum_position_angle_deg": min(s.position_angle_deg for s in samples),
        "maximum_position_angle_deg": max(s.position_angle_deg for s in samples),
        "minimum_afo_angle_deg": min(s.afo_angle_deg for s in samples),
        "maximum_afo_angle_deg": max(s.afo_angle_deg for s in samples),
        "maximum_xy_angle_disagreement_deg": max(angle_differences),
    }


def _first_index(samples: Sequence[Sample], start: int, predicate) -> int | None:
    for index in range(start, len(samples)):
        if predicate(samples[index].detection_angle_deg):
            return index
    return None


def detect_segments(
    samples: Sequence[Sample], spec: TrialSpec, endpoint_threshold_deg: float
) -> tuple[list[Segment], int, list[str]]:
    warnings: list[str] = []
    segments: list[Segment] = []
    startup_end = _first_index(samples, 0, lambda angle: angle >= endpoint_threshold_deg)
    if startup_end is None:
        warnings.append(
            "No positive dorsiflexion endpoint was detected. Check the angle direction, origin, "
            "neutral value, recording coverage and endpoint threshold."
        )
        return segments, 0, warnings

    departure = _first_index(samples, 0, lambda angle: abs(angle) >= 0.5) or 0
    segments.append(
        Segment(
            video_id=spec.video_id,
            trial=spec.trial,
            cycle=0,
            direction="Neutral_to_DF_startup",
            start_index=departure,
            end_index=startup_end,
            complete=True,
            exclusion_reason="startup movement is not a complete endpoint-to-endpoint sweep",
        )
    )

    cursor = startup_end
    completed_cycles = 0
    for cycle in range(1, spec.expected_cycles + 1):
        pf_end = _first_index(samples, cursor + 1, lambda angle: angle <= -endpoint_threshold_deg)
        if pf_end is None:
            segments.append(
                Segment(
                    spec.video_id,
                    spec.trial,
                    cycle,
                    "DF_to_PF",
                    cursor,
                    len(samples) - 1,
                    False,
                    exclusion_reason="recording ended before the plantarflexion endpoint",
                )
            )
            break
        segments.append(
            Segment(spec.video_id, spec.trial, cycle, "DF_to_PF", cursor, pf_end, True)
        )
        cursor = pf_end

        df_end = _first_index(samples, cursor + 1, lambda angle: angle >= endpoint_threshold_deg)
        if df_end is None:
            segments.append(
                Segment(
                    spec.video_id,
                    spec.trial,
                    cycle,
                    "PF_to_DF",
                    cursor,
                    len(samples) - 1,
                    False,
                    exclusion_reason="recording ended before the dorsiflexion endpoint",
                )
            )
            break
        segments.append(
            Segment(spec.video_id, spec.trial, cycle, "PF_to_DF", cursor, df_end, True)
        )
        cursor = df_end
        completed_cycles = cycle

    if completed_cycles < spec.expected_cycles:
        warnings.append(
            f"Detected {completed_cycles}/{spec.expected_cycles} complete cycles; trial is incomplete."
        )
    return segments, completed_cycles, warnings


def linear_regression(points: Sequence[tuple[float, float]]) -> tuple[float, float, float]:
    count = len(points)
    mean_x = sum(x for x, _ in points) / count
    mean_y = sum(y for _, y in points) / count
    ss_x = sum((x - mean_x) ** 2 for x, _ in points)
    if ss_x == 0:
        raise ValueError("fit contains no time variation")
    slope = sum((x - mean_x) * (y - mean_y) for x, y in points) / ss_x
    intercept = mean_y - slope * mean_x
    residual = sum((y - (slope * x + intercept)) ** 2 for x, y in points)
    total = sum((y - mean_y) ** 2 for _, y in points)
    r_squared = 1.0 - residual / total if total else 1.0
    return slope, intercept, r_squared


def fit_segments(
    samples: Sequence[Sample],
    segments: list[Segment],
    spec: TrialSpec,
    fit_limit_deg: float,
    min_fit_points: int,
    min_r_squared: float,
    trial_complete: bool,
) -> None:
    for segment in segments:
        segment.commanded_speed_deg_s = spec.commanded_speed_deg_s
        if segment.cycle == 0:
            continue
        if not segment.complete:
            continue
        fit_samples = [
            sample
            for sample in samples[segment.start_index : segment.end_index + 1]
            if -fit_limit_deg <= sample.afo_angle_deg <= fit_limit_deg
        ]
        segment.fit_points = len(fit_samples)
        if len(fit_samples) < min_fit_points:
            segment.exclusion_reason = (
                f"only {len(fit_samples)} points in the -{fit_limit_deg:g} to "
                f"+{fit_limit_deg:g} degree fit window"
            )
            continue
        points = [(sample.time_s, sample.afo_angle_deg) for sample in fit_samples]
        slope, intercept, r_squared = linear_regression(points)
        segment.fit_start_time_s = fit_samples[0].time_s
        segment.fit_end_time_s = fit_samples[-1].time_s
        segment.slope_deg_s = slope
        segment.speed_magnitude_deg_s = abs(slope)
        segment.intercept_deg = intercept
        segment.r_squared = r_squared
        segment.absolute_error_deg_s = abs(abs(slope) - spec.commanded_speed_deg_s)
        segment.percentage_error = (
            100.0 * segment.absolute_error_deg_s / spec.commanded_speed_deg_s
        )
        expected_sign = -1 if segment.direction == "DF_to_PF" else 1
        reasons = []
        if slope * expected_sign <= 0:
            reasons.append("fitted slope has the wrong direction")
        if r_squared < min_r_squared:
            reasons.append(f"R-squared is below {min_r_squared:g}")
        if not trial_complete:
            reasons.append("trial did not contain all expected complete cycles")
        segment.exclusion_reason = "; ".join(reasons)
        segment.included_by_protocol = not reasons


def _fmt(value):
    if value is None:
        return ""
    if isinstance(value, bool):
        return str(value).lower()
    if isinstance(value, float):
        return f"{value:.9g}"
    return value


def write_csv(path: Path, rows: Iterable[dict], fieldnames: Sequence[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({name: _fmt(row.get(name)) for name in fieldnames})


def write_cleaned_csv(path: Path, samples: Sequence[Sample]) -> None:
    fields = [
        "time_s",
        "frame",
        "x",
        "y",
        "position_angle_deg",
        "afo_angle_deg",
        "detection_angle_deg",
    ]
    write_csv(path, (asdict(sample) for sample in samples), fields)


def segment_row(segment: Segment) -> dict:
    row = asdict(segment)
    row.pop("start_index")
    row.pop("end_index")
    return row


def write_svg_plot(
    path: Path,
    samples: Sequence[Sample],
    segments: Sequence[Segment],
    spec: TrialSpec,
    fit_limit_deg: float,
) -> None:
    width, height = 1100, 560
    left, right, top, bottom = 75, 25, 45, 65
    plot_w, plot_h = width - left - right, height - top - bottom
    t_min, t_max = samples[0].time_s, samples[-1].time_s
    a_min = min(-11.0, min(sample.afo_angle_deg for sample in samples) - 0.5)
    a_max = max(11.0, max(sample.afo_angle_deg for sample in samples) + 0.5)

    def px(t):
        return left + (t - t_min) / (t_max - t_min) * plot_w

    def py(angle):
        return top + (a_max - angle) / (a_max - a_min) * plot_h

    raw_points = " ".join(f"{px(s.time_s):.2f},{py(s.afo_angle_deg):.2f}" for s in samples)
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{left}" y="25" font-family="sans-serif" font-size="18">{spec.video_id}: angle versus time</text>',
    ]
    for angle, colour, dash in [(-10, "#bbb", "4 4"), (-fit_limit_deg, "#88a", "5 4"), (0, "#999", ""), (fit_limit_deg, "#88a", "5 4"), (10, "#bbb", "4 4")]:
        y = py(angle)
        lines.append(
            f'<line x1="{left}" y1="{y:.2f}" x2="{width-right}" y2="{y:.2f}" '
            f'stroke="{colour}" stroke-width="1" stroke-dasharray="{dash}"/>'
        )
        lines.append(f'<text x="{left-10}" y="{y+4:.2f}" text-anchor="end" font-family="sans-serif" font-size="12">{angle:g}°</text>')
    lines.append(f'<polyline points="{raw_points}" fill="none" stroke="#2457a6" stroke-width="1.5"/>')
    for segment in segments:
        if segment.fit_start_time_s is None or segment.fit_end_time_s is None:
            continue
        colour = "#15803d" if segment.included_by_protocol else "#d97706"
        fit_points = [
            s for s in samples[segment.start_index : segment.end_index + 1]
            if -fit_limit_deg <= s.afo_angle_deg <= fit_limit_deg
        ]
        points = " ".join(f"{px(s.time_s):.2f},{py(s.afo_angle_deg):.2f}" for s in fit_points)
        lines.append(f'<polyline points="{points}" fill="none" stroke="{colour}" stroke-width="3"/>')
    lines.extend([
        f'<line x1="{left}" y1="{top}" x2="{left}" y2="{height-bottom}" stroke="black"/>',
        f'<line x1="{left}" y1="{height-bottom}" x2="{width-right}" y2="{height-bottom}" stroke="black"/>',
        f'<text x="{left+plot_w/2:.1f}" y="{height-18}" text-anchor="middle" font-family="sans-serif" font-size="14">Time (s)</text>',
        f'<text x="18" y="{top+plot_h/2:.1f}" transform="rotate(-90 18 {top+plot_h/2:.1f})" text-anchor="middle" font-family="sans-serif" font-size="14">AFO angle (degrees; dorsiflexion positive)</text>',
        '</svg>',
    ])
    path.write_text("\n".join(lines), encoding="utf-8")


def analyse_trial(spec: TrialSpec, output_root: Path, args) -> tuple[list[dict], dict]:
    samples, import_qc = read_tracker_export(spec.file)
    angle_qc = prepare_angles(samples, spec)
    segments, completed_cycles, warnings = detect_segments(
        samples, spec, args.endpoint_threshold
    )
    trial_complete = completed_cycles == spec.expected_cycles
    fit_segments(
        samples,
        segments,
        spec,
        args.fit_limit,
        args.min_fit_points,
        args.min_r_squared,
        trial_complete,
    )

    trial_dir = output_root / spec.video_id
    trial_dir.mkdir(parents=True, exist_ok=True)
    write_cleaned_csv(trial_dir / "cleaned_angle_data.csv", samples)
    segment_fields = list(segment_row(segments[0]).keys()) if segments else [
        "video_id", "trial", "cycle", "direction", "complete", "included_by_protocol",
        "exclusion_reason", "fit_start_time_s", "fit_end_time_s", "fit_points",
        "slope_deg_s", "speed_magnitude_deg_s", "intercept_deg", "r_squared",
        "commanded_speed_deg_s", "absolute_error_deg_s", "percentage_error",
    ]
    write_csv(trial_dir / "segment_results.csv", (segment_row(s) for s in segments), segment_fields)
    write_svg_plot(trial_dir / "angle_time_qc.svg", samples, segments, spec, args.fit_limit)

    valid_speeds = [
        s.speed_magnitude_deg_s
        for s in segments
        if s.included_by_protocol and s.speed_magnitude_deg_s is not None
    ]
    direction_means = {}
    for direction in ("DF_to_PF", "PF_to_DF"):
        values = [
            s.speed_magnitude_deg_s
            for s in segments
            if s.included_by_protocol and s.direction == direction and s.speed_magnitude_deg_s is not None
        ]
        direction_means[direction] = statistics.mean(values) if values else None
    mean_speed = statistics.mean(valid_speeds) if valid_speeds else None
    sd_speed = statistics.stdev(valid_speeds) if len(valid_speeds) > 1 else None
    trial_summary = {
        "video_id": spec.video_id,
        "trial": spec.trial,
        "source_file": str(spec.file),
        "commanded_speed_deg_s": spec.commanded_speed_deg_s,
        "expected_cycles": spec.expected_cycles,
        "completed_cycles": completed_cycles,
        "trial_complete": trial_complete,
        "primary_segment_count": len(valid_speeds),
        "mean_speed_deg_s": mean_speed,
        "sd_speed_deg_s": sd_speed,
        "cv_percent": (100 * sd_speed / mean_speed if sd_speed is not None and mean_speed else None),
        "mean_DF_to_PF_deg_s": direction_means["DF_to_PF"],
        "mean_PF_to_DF_deg_s": direction_means["PF_to_DF"],
        "mean_percentage_error": (
            100 * abs(mean_speed - spec.commanded_speed_deg_s) / spec.commanded_speed_deg_s
            if mean_speed is not None else None
        ),
    }
    qc = {
        "specification": {**asdict(spec), "file": str(spec.file)},
        "import": import_qc,
        "angles": angle_qc,
        "detection": {
            "endpoint_threshold_deg": args.endpoint_threshold,
            "fit_limit_deg": args.fit_limit,
            "completed_cycles": completed_cycles,
            "expected_cycles": spec.expected_cycles,
            "trial_complete": trial_complete,
        },
        "warnings": warnings,
        "trial_summary": trial_summary,
    }
    (trial_dir / "quality_control.json").write_text(
        json.dumps(qc, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    return [segment_row(segment) for segment in segments], trial_summary


def read_manifest(path: Path) -> list[TrialSpec]:
    with path.open("r", encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"file", "video_id", "commanded_speed_deg_s", "trial"}
        if not reader.fieldnames or not required.issubset(reader.fieldnames):
            raise ValueError(f"manifest must contain: {', '.join(sorted(required))}")
        specs = []
        for line, row in enumerate(reader, start=2):
            if not row.get("file", "").strip():
                continue
            source = Path(row["file"].strip()).expanduser()
            if not source.is_absolute():
                source = (path.parent / source).resolve()
            specs.append(
                TrialSpec(
                    file=source,
                    video_id=row["video_id"].strip(),
                    commanded_speed_deg_s=float(row["commanded_speed_deg_s"]),
                    trial=row["trial"].strip(),
                    expected_cycles=int(row.get("expected_cycles") or DEFAULT_EXPECTED_CYCLES),
                    neutral_position_angle_deg=float(
                        row.get("neutral_position_angle_deg") or DEFAULT_NEUTRAL_DEG
                    ),
                    dorsiflexion_direction=(
                        row.get("dorsiflexion_direction") or "decreasing"
                    ).strip().lower(),
                )
            )
    if not specs:
        raise ValueError("manifest contains no trials")
    ids = [spec.video_id for spec in specs]
    if len(ids) != len(set(ids)):
        raise ValueError("video_id values must be unique")
    return specs


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--manifest", type=Path, help="CSV manifest for batch analysis")
    source.add_argument("--input", type=Path, help="single Tracker TXT/CSV export")
    parser.add_argument("--video-id")
    parser.add_argument("--commanded-speed", type=float)
    parser.add_argument("--trial", default="1")
    parser.add_argument("--expected-cycles", type=int, default=DEFAULT_EXPECTED_CYCLES)
    parser.add_argument("--neutral-angle", type=float, default=DEFAULT_NEUTRAL_DEG)
    parser.add_argument(
        "--dorsiflexion-direction",
        choices=("increasing", "decreasing"),
        default="decreasing",
        help="direction of Tracker position angle during dorsiflexion",
    )
    parser.add_argument("--endpoint-threshold", type=float, default=DEFAULT_ENDPOINT_THRESHOLD_DEG)
    parser.add_argument("--fit-limit", type=float, default=DEFAULT_FIT_LIMIT_DEG)
    parser.add_argument("--min-fit-points", type=int, default=DEFAULT_MIN_FIT_POINTS)
    parser.add_argument("--min-r-squared", type=float, default=DEFAULT_MIN_R2)
    parser.add_argument("--output", type=Path, default=Path("tracker_analysis_results"))
    args = parser.parse_args(argv)
    if args.input and (not args.video_id or args.commanded_speed is None):
        parser.error("--input requires --video-id and --commanded-speed")
    if not 0 < args.fit_limit < args.endpoint_threshold:
        parser.error("fit limit must be positive and smaller than endpoint threshold")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    if args.manifest:
        specs = read_manifest(args.manifest.resolve())
    else:
        specs = [
            TrialSpec(
                file=args.input.expanduser().resolve(),
                video_id=args.video_id,
                commanded_speed_deg_s=args.commanded_speed,
                trial=args.trial,
                expected_cycles=args.expected_cycles,
                neutral_position_angle_deg=args.neutral_angle,
                dorsiflexion_direction=args.dorsiflexion_direction,
            )
        ]

    output = args.output.expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    all_segments: list[dict] = []
    all_trials: list[dict] = []
    failures = []
    for spec in specs:
        try:
            segments, summary = analyse_trial(spec, output, args)
            all_segments.extend(segments)
            all_trials.append(summary)
        except Exception as exc:
            failures.append({"video_id": spec.video_id, "file": str(spec.file), "error": str(exc)})

    segment_fields = [
        "video_id", "trial", "cycle", "direction", "complete", "included_by_protocol",
        "exclusion_reason", "fit_start_time_s", "fit_end_time_s", "fit_points",
        "slope_deg_s", "speed_magnitude_deg_s", "intercept_deg", "r_squared",
        "commanded_speed_deg_s", "absolute_error_deg_s", "percentage_error",
    ]
    trial_fields = [
        "video_id", "trial", "source_file", "commanded_speed_deg_s", "expected_cycles",
        "completed_cycles", "trial_complete", "primary_segment_count", "mean_speed_deg_s",
        "sd_speed_deg_s", "cv_percent", "mean_DF_to_PF_deg_s", "mean_PF_to_DF_deg_s",
        "mean_percentage_error",
    ]
    write_csv(output / "combined_segment_results.csv", all_segments, segment_fields)
    write_csv(output / "combined_trial_summary.csv", all_trials, trial_fields)
    (output / "analysis_failures.json").write_text(
        json.dumps(failures, indent=2) + "\n", encoding="utf-8"
    )

    print(f"Analysed {len(all_trials)}/{len(specs)} trial(s). Results: {output}")
    for row in all_trials:
        print(
            f"{row['video_id']}: cycles {row['completed_cycles']}/{row['expected_cycles']}, "
            f"complete={row['trial_complete']}, primary segments={row['primary_segment_count']}"
        )
    if failures:
        for failure in failures:
            print(f"ERROR {failure['video_id']}: {failure['error']}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
