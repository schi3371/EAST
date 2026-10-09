import contextlib
import csv
import io
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

from p3022_quick_test import estimate_fields, display_sample, nominal_angle, run
from backfill_p3022_estimates import backfill


class EstimateTests(unittest.TestCase):
    def test_above_five_is_preserved_and_estimated_with_explicit_scale(self):
        result = estimate_fields(5.1333, 5.19)
        self.assertEqual(result["estimated_shaft_angle_deg"], "356.06705")
        self.assertTrue(result["electrical_range_valid"])
        self.assertIn("UNCALIBRATED", display_sample(1, 5.1333, result))
        self.assertIn("above nominal 5 V", display_sample(1, 5.1333, result))
        self.assertIsNone(nominal_angle(5.1333))

    def test_above_assumed_full_scale_is_not_clipped_or_wrapped(self):
        result = estimate_fields(5.193153, 5.19)
        self.assertGreater(float(result["estimated_shaft_angle_deg"]), 360)
        self.assertEqual(result["angle_estimate_status"], "uncalibrated_above_assumed_full_scale")
        self.assertIn("extrapolation", display_sample(1, 5.193153, result))

    def test_electrical_range_and_nonfinite_values_do_not_become_angles(self):
        for voltage in (-0.001, 5.3001, math.nan, math.inf):
            with self.subTest(voltage=voltage):
                self.assertEqual(estimate_fields(voltage, 5.19)["estimated_shaft_angle_deg"], "")
        self.assertTrue(estimate_fields(5.3, 5.19)["electrical_range_valid"])

    def test_invalid_scale_rejected(self):
        for scale in (0, -1, 5.31, math.inf, math.nan):
            with self.assertRaises(ValueError):
                estimate_fields(2, scale)

    def test_console_summary_includes_above_five(self):
        with tempfile.TemporaryDirectory() as folder:
            args = SimpleNamespace(demo=True, interval_ms=1, reference_voltage=None,
                                   output_dir=Path(folder), duration=0.005, direction=1,
                                   full_scale_voltage=5.19)
            output = io.StringIO()
            with patch("p3022_quick_test.math.sin", return_value=26), contextlib.redirect_stdout(output):
                self.assertEqual(run(args), 0)
            self.assertIn("5.100000 to 5.100000 V", output.getvalue())
            self.assertIn("estimated shaft angle", output.getvalue())
            with next(Path(folder).glob("*.csv")).open() as handle:
                rows = list(csv.DictReader(handle))
            self.assertTrue(rows)
            self.assertTrue(all(r["nominal_shaft_angle_deg"] == "" for r in rows))
            self.assertTrue(all(r["estimated_shaft_angle_deg"] for r in rows))


class MigrationTests(unittest.TestCase):
    def make_file(self, folder, name="p3022_example.csv"):
        path = Path(folder) / name
        path.write_bytes(b"host_elapsed_s,voltage_v,nominal_shaft_angle_deg,nominal_range_valid,data_source\r\n"
                         b"1.123456,5.13330000,,False,HUB0007_VoltageInput\r\n"
                         b"2.123456,2.50000000,180.00000,True,HUB0007_VoltageInput\r\n")
        return path

    def test_backup_and_old_fields_are_unchanged(self):
        with tempfile.TemporaryDirectory() as folder:
            path = self.make_file(folder)
            original = path.read_bytes()
            old_rows = list(csv.DictReader(io.StringIO(original.decode())))
            report = backfill([path], 5.19, apply=True)
            self.assertEqual((Path(report["backup_directory"]) / path.name).read_bytes(), original)
            rows = list(csv.DictReader(io.StringIO(path.read_text())))
            for old, new in zip(old_rows, rows):
                self.assertEqual(old, {k: new[k] for k in old})
            self.assertEqual(rows[0]["estimated_shaft_angle_deg"], "356.06705")
            self.assertIn("not_calibrated", rows[0]["angle_estimate_basis"])
            self.assertEqual(report["files"][0]["above_nominal_5v"], 1)
            with self.assertRaises(ValueError):
                backfill([path], 5.19, apply=True)

    def test_dry_run_does_not_change_files_or_create_backups(self):
        with tempfile.TemporaryDirectory() as folder:
            path = self.make_file(folder)
            original = path.read_bytes()
            backfill([path], 5.19)
            self.assertEqual(path.read_bytes(), original)
            self.assertEqual(len(list(Path(folder).iterdir())), 1)

    def test_bad_second_file_prevents_all_writes(self):
        with tempfile.TemporaryDirectory() as folder:
            path = self.make_file(folder)
            original = path.read_bytes()
            bad = Path(folder) / "p3022_bad.csv"
            bad.write_text("voltage_v\n5.1\n")
            with self.assertRaises(ValueError):
                backfill([path, bad], 5.19, apply=True)
            self.assertEqual(path.read_bytes(), original)
            self.assertFalse((Path(folder) / "originals_before_angle_estimates").exists())


if __name__ == "__main__":
    unittest.main()
