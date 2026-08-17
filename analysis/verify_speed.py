"""Estimate measured AFO angular speed from an EAST strain CSV."""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path
from statistics import mean, stdev


TIME_COLUMN = "Elapsed Time (s)"
ANGLE_COLUMN = "Raw AFO Angle (deg)"
COMMAND_COLUMN = "Commanded AFO Speed (deg/s)"
PHASE_COLUMN = "Motion Phase"
CYCLE_COLUMN = "Cycle"
MOTION_PHASES = {"moving_to_max", "moving_to_min"}


def linear_slope(points):
    x_mean = mean(point[0] for point in points)
    y_mean = mean(point[1] for point in points)
    denominator = sum((point[0] - x_mean) ** 2 for point in points)
    if denominator == 0:
        raise ValueError("Segment contains no elapsed-time variation")
    return sum((x - x_mean) * (y - y_mean) for x, y in points) / denominator


def middle_motion_points(rows, exclusion_fraction):
    angles = [row[1] for row in rows]
    low, high = min(angles), max(angles)
    span = high - low
    if span <= 0:
        return []
    lower = low + span * exclusion_fraction
    upper = high - span * exclusion_fraction
    return [(elapsed, angle) for elapsed, angle, _command in rows if lower <= angle <= upper]


def analyse(csv_path, exclusion_fraction=0.2):
    groups = defaultdict(list)
    with Path(csv_path).open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        required = {TIME_COLUMN, ANGLE_COLUMN, COMMAND_COLUMN, PHASE_COLUMN, CYCLE_COLUMN}
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"CSV is missing required columns: {', '.join(sorted(missing))}")
        for row in reader:
            phase = row[PHASE_COLUMN]
            if phase not in MOTION_PHASES:
                continue
            key = (int(row[CYCLE_COLUMN]), phase)
            groups[key].append((
                float(row[TIME_COLUMN]),
                float(row[ANGLE_COLUMN]),
                float(row[COMMAND_COLUMN]),
            ))

    segment_results = []
    for (cycle, phase), rows in sorted(groups.items()):
        points = middle_motion_points(rows, exclusion_fraction)
        if len(points) < 5:
            continue
        signed_speed = linear_slope(points)
        commanded_speed = mean(row[2] for row in rows)
        measured_speed = abs(signed_speed)
        error_percent = 100.0 * (measured_speed - commanded_speed) / commanded_speed
        segment_results.append({
            "cycle": cycle,
            "phase": phase,
            "commanded_speed_deg_s": commanded_speed,
            "measured_speed_deg_s": measured_speed,
            "signed_slope_deg_s": signed_speed,
            "error_percent": error_percent,
            "included_samples": len(points),
        })
    if not segment_results:
        raise ValueError("No motion segment had enough data for analysis")
    return segment_results


def write_results(results, output_path):
    columns = list(results[0])
    with Path(output_path).open("x", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(results)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_file", type=Path)
    parser.add_argument(
        "--exclude-fraction", type=float, default=0.2,
        help="Fraction excluded at each end of every angular excursion (default: 0.2)",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not 0 <= args.exclude_fraction < 0.5:
        raise SystemExit("--exclude-fraction must be at least 0 and less than 0.5")

    results = analyse(args.csv_file, args.exclude_fraction)
    for row in results:
        print(
            f"cycle={row['cycle']} phase={row['phase']} "
            f"commanded={row['commanded_speed_deg_s']:.3f} deg/s "
            f"measured={row['measured_speed_deg_s']:.3f} deg/s "
            f"error={row['error_percent']:+.2f}% n={row['included_samples']}"
        )
    measured = [row["measured_speed_deg_s"] for row in results]
    commanded = mean(row["commanded_speed_deg_s"] for row in results)
    overall_error = 100.0 * (mean(measured) - commanded) / commanded
    variability = stdev(measured) if len(measured) > 1 else 0.0
    print(
        f"summary: commanded={commanded:.3f} deg/s "
        f"measured_mean={mean(measured):.3f} deg/s "
        f"measured_sd={variability:.3f} deg/s error={overall_error:+.2f}%"
    )
    if args.output:
        write_results(results, args.output)
        print(f"saved {args.output}")


if __name__ == "__main__":
    main()
