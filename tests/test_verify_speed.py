import csv
import tempfile
import unittest
from pathlib import Path

from analysis.verify_speed import analyse


class VerifySpeedTests(unittest.TestCase):
    def test_recovers_linear_speed_from_middle_of_motion(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "speed.csv"
            with path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=[
                    "Elapsed Time (s)", "ODrive-Derived AFO Angle (deg)",
                    "Commanded AFO Speed (deg/s)", "Motion Phase", "Cycle",
                ])
                writer.writeheader()
                for index in range(101):
                    elapsed = index * 0.01
                    writer.writerow({
                        "Elapsed Time (s)": elapsed,
                        "ODrive-Derived AFO Angle (deg)": 5.0 * elapsed,
                        "Commanded AFO Speed (deg/s)": 5.0,
                        "Motion Phase": "moving_to_max",
                        "Cycle": 1,
                    })
            results = analyse(path)
            self.assertEqual(len(results), 1)
            self.assertAlmostEqual(results[0]["odrive_derived_speed_deg_s"], 5.0, places=6)
            self.assertAlmostEqual(results[0]["odrive_derived_error_percent"], 0.0, places=6)

    def test_accepts_legacy_raw_afo_angle_heading(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "legacy.csv"
            with path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=[
                    "Elapsed Time (s)", "Raw AFO Angle (deg)",
                    "Commanded AFO Speed (deg/s)", "Motion Phase", "Cycle",
                ])
                writer.writeheader()
                for index in range(20):
                    elapsed = index * 0.02
                    writer.writerow({
                        "Elapsed Time (s)": elapsed,
                        "Raw AFO Angle (deg)": 4.0 * elapsed,
                        "Commanded AFO Speed (deg/s)": 4.0,
                        "Motion Phase": "moving_to_max",
                        "Cycle": 1,
                    })
            results = analyse(path)
            self.assertAlmostEqual(results[0]["odrive_derived_speed_deg_s"], 4.0)


if __name__ == "__main__":
    unittest.main()
