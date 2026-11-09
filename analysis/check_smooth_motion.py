"""Replay past command intervals against the new planner using synthetic sensing."""
import argparse
import csv
import json
import math
from pathlib import Path
from statistics import median

from east_smooth_motion import FilteredPositionSweep, Sweep
from east_stream_screen import StreamScreenModel


def check_run(metadata):
    old = json.loads(metadata.read_text())
    params = old["test_parameters"]
    speed = params["commanded_afo_speed_deg_s"]
    half_range = params["max_angle_deg"]
    samples = old["rotary_feedback_pilot"]["samples"]
    gaps = []
    for previous, current in zip(samples, samples[1:]):
        if (previous["phase"], previous["cycle"]) != (current["phase"], current["cycle"]):
            continue
        gap = current["command_issued_at_monotonic_s"] - previous["command_issued_at_monotonic_s"]
        if gap > 0:
            gaps.append(gap)
    if not gaps:
        raise ValueError(f"No positive within-leg intervals in {metadata.name}")
    model = StreamScreenModel.anchored("candidate", 0., 0., full_range=True)

    for feedback in (False, True):
        targets = (0., half_range, -half_range, half_range, 0.)
        motor = 0.
        gap_index = 0
        max_command_rate = max_trim = max_trim_rate = max_gap = 0.
        total_duration = 0.
        for start, target in zip(targets, targets[1:]):
            sweep = Sweep.plan(start, target, speed, params["commanded_afo_acceleration_deg_s2"])
            planner = FilteredPositionSweep(sweep, model, motor, feedback=feedback)
            current = planner.command(0., measured=start, sample_time=0.)
            assert current["target_motor_turns"] == motor
            while current["time_s"] < sweep.duration:
                gap = gaps[gap_index % len(gaps)]
                gap_index += 1
                sample_time = current["time_s"] + gap
                angle = sweep.at(sample_time)[0]
                # Deliberately excite trim with a bias plus high-frequency
                # ripple. The recorded intervals are real, sensor data here are synthetic.
                measured = angle - .65 + .04 * math.sin(2 * math.pi * 19 * sample_time)
                row = planner.command(sample_time, measured=measured, sample_time=sample_time)
                max_gap = max(max_gap, gap)
                max_trim = max(max_trim, abs(row["rotary_trim_deg"]))
                max_trim_rate = max(max_trim_rate, abs(row["rotary_trim_deg"] - current["rotary_trim_deg"]) / gap)
                delta = row["target_motor_turns"] - current["target_motor_turns"]
                max_command_rate = max(max_command_rate, abs(delta / gap))
                current = row
            assert current["rotary_trim_deg"] == 0.
            motor = current["target_motor_turns"]
            assert math.isclose(motor, model.target(target, sweep.direction), abs_tol=1e-12)
            total_duration += sweep.duration
        yield dict(metadata=metadata.name, speed_deg_s=speed, feedback=feedback,
                   samples=gap_index, historical_median_gap_s=median(gaps),
                   max_replayed_gap_s=max_gap, planned_cycle_s=total_duration,
                   max_command_rate_turns_s=max_command_rate,
                   max_trim_deg=max_trim, max_trim_rate_deg_s=max_trim_rate)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("primary_runs", type=Path)
    parser.add_argument("logs", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    with args.primary_runs.open(newline="") as file:
        metadata = [args.logs / row["metadata"] for row in csv.DictReader(file)
                    if row["method"] == "candidate"]
    if not metadata:
        parser.error("No wider cubic reference runs")
    rows = [row for path in metadata for row in check_run(path)]
    with args.output.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


if __name__ == "__main__":
    main()
