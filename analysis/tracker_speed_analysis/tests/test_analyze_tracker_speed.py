import csv
import tempfile
import unittest
from pathlib import Path

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from analyze_tracker_speed import (  # noqa: E402
    TrialSpec,
    detect_segments,
    fit_segments,
    prepare_angles,
    read_tracker_export,
)


def synthetic_samples(path: Path, complete_cycles: int, partial_return=False):
    points = []
    t = 0.0

    def add_ramp(a, b, count):
        nonlocal t
        for i in range(count):
            afo = a + (b - a) * i / (count - 1)
            theta = 90.0 - afo
            points.append((t, 100 * __import__("math").cos(__import__("math").radians(theta)),
                           100 * __import__("math").sin(__import__("math").radians(theta)), theta))
            t += 0.1

    add_ramp(0, 10, 101)
    for _ in range(complete_cycles):
        add_ramp(10, -10, 201)
        add_ramp(-10, 10, 201)
    if partial_return:
        add_ramp(10, -10, 201)
        add_ramp(-10, 5, 151)

    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["", "mass A", "", "", ""])
        writer.writerow(["t", "x", "y", "θr", ""])
        writer.writerows(points)


class TrackerAnalysisTests(unittest.TestCase):
    def test_tracker_two_row_header_and_angle_conversion(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trial.txt"
            synthetic_samples(path, 1)
            samples, qc = read_tracker_export(path)
            spec = TrialSpec(path, "V1", 1.0, "1", expected_cycles=1)
            prepare_angles(samples, spec)
            self.assertAlmostEqual(samples[0].afo_angle_deg, 0.0, places=6)
            self.assertAlmostEqual(max(s.afo_angle_deg for s in samples), 10.0, places=6)
            self.assertEqual(qc["skipped_rows"], 0)

    def test_three_cycles_produce_six_complete_analysis_segments(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trial.txt"
            synthetic_samples(path, 3)
            samples, _ = read_tracker_export(path)
            spec = TrialSpec(path, "V1", 1.0, "1", expected_cycles=3)
            prepare_angles(samples, spec)
            segments, completed, warnings = detect_segments(samples, spec, 9.0)
            fit_segments(samples, segments, spec, 8.0, 30, 0.995, True)
            movements = [s for s in segments if s.cycle > 0 and s.complete]
            self.assertEqual(completed, 3)
            self.assertEqual(len(movements), 6)
            self.assertEqual(sum(s.included_by_protocol for s in movements), 6)
            self.assertFalse(warnings)
            self.assertEqual(segments[0].cycle, 0)

    def test_incomplete_trial_is_retained_but_excluded(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "trial.txt"
            synthetic_samples(path, 0, partial_return=True)
            samples, _ = read_tracker_export(path)
            spec = TrialSpec(path, "V1", 1.0, "1", expected_cycles=3)
            prepare_angles(samples, spec)
            segments, completed, warnings = detect_segments(samples, spec, 9.0)
            fit_segments(samples, segments, spec, 8.0, 30, 0.995, False)
            self.assertEqual(completed, 0)
            self.assertTrue(warnings)
            self.assertTrue(any(s.complete and s.cycle == 1 for s in segments))
            self.assertFalse(any(s.included_by_protocol for s in segments))


if __name__ == "__main__":
    unittest.main()
