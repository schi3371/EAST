"""Regression checks for the thesis's per-cycle loading properties."""
import csv
import json
from pathlib import Path
import tempfile
import unittest

from east_stiffness import ANGLES, PHASES, analyse_stiffness, cycle_properties


class LoadingAnalysisTests(unittest.TestCase):
    def test_loading_fits_include_the_interpolated_neutral(self):
        increasing = [-2 * (angle + 1.125) for angle in ANGLES]
        decreasing = [-3 * (angle - .625) for angle in ANGLES]
        result = cycle_properties([increasing, decreasing])
        df, pf = result["loading_branches"]
        self.assertAlmostEqual(df["stiffness_nm_per_deg"], 2)
        self.assertAlmostEqual(pf["stiffness_nm_per_deg"], 3)
        self.assertEqual(df["fit_window_deg"], (-1.125, 8))
        self.assertEqual(pf["fit_window_deg"], (-8, .625))

    def test_loop_area_uses_signed_integral_before_magnitude(self):
        # These sweeps cross, so absolute pointwise separation gives a different answer.
        result = cycle_properties([[-a for a in ANGLES], [-2 * a for a in ANGLES]])
        self.assertAlmostEqual(result["corrected_loop_area_j"], 0, places=12)

    def test_absent_and_multiple_crossings_are_reported(self):
        valid = [-a for a in ANGLES]
        with self.assertRaisesRegex(ValueError, "no crossing"):
            cycle_properties([[1] * len(ANGLES), valid])
        with self.assertRaisesRegex(ValueError, "multiple crossings"):
            cycle_properties([[a * a - 4 for a in ANGLES], valid])

    def test_saved_runs_average_cycles_and_preserve_exclusion_reasons(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            reference = self.write_run(folder, "empty", empty=True)
            loaded = self.write_run(folder, "loaded")
            result = analyse_stiffness(loaded, reference)
            self.assertEqual(result["analysis_version"], 4)
            self.assertEqual(result["cycles"], [2, 3, 4, 5])
            self.assertEqual([b["outcome"] for b in result["branches"]], ["DF loading", "PF loading"])
            self.assertAlmostEqual(result["neutral_by_sweep"][0]["angle_deg"], -1.875)
            self.assertEqual(len(result["cycle_properties"]), 4)
            self.assertTrue(all(b["sd_nm_per_deg"] > 0 for b in result["branches"]))
            areas = [r["corrected_loop_area_j"] for r in result["cycle_properties"].values()]
            self.assertAlmostEqual(result["corrected_loop_area_j"], sum(areas) / 4)
            json.dumps(result, allow_nan=False)
            # Invalid later cycles are excluded from all mounting-session properties.
            loaded = self.write_run(folder, "loaded", invalid=(3, 4))
            result = analyse_stiffness(loaded, reference)
            self.assertEqual(result["cycles"], [2, 5])
            self.assertEqual(result["excluded_cycles"], [3, 4])
            self.assertIn("no crossing", result["excluded_cycle_reasons"][3])
            loaded = self.write_run(folder, "loaded", invalid=(3, 4, 5))
            with self.assertRaisesRegex(ValueError, "at least two usable"):
                analyse_stiffness(loaded, reference)
            # Three-cycle legacy recordings retain their two later cycles.
            loaded = self.write_run(folder, "loaded", cycles=3)
            self.assertEqual(analyse_stiffness(loaded, reference)["cycles"], [2, 3])

    def test_linked_reference_and_calibration_checks_still_apply(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            reference = self.write_run(folder, "empty", empty=True)
            loaded = self.write_run(folder, "loaded")
            empty = json.loads(reference.read_text())
            metadata = json.loads(loaded.read_text())
            metadata["protocol_batch"] = {"baseline_batch_id": "abc"}
            loaded.write_text(json.dumps(metadata))
            (folder / "three_speed_abc.json").write_text(json.dumps({
                "runs": [{"status": "completed", "parameters": empty["test_parameters"],
                          "metadata_file": reference.name}]}))
            self.assertEqual(analyse_stiffness(loaded)["empty_metadata"], str(reference))
            empty["calibration"]["lever_arm_m"] = 999
            reference.write_text(json.dumps(empty))
            with self.assertRaisesRegex(ValueError, "torque calibration"):
                analyse_stiffness(loaded)

    @staticmethod
    def write_run(folder, name, empty=False, cycles=5, invalid=()):
        csv_path = folder / (name + "_strain_data.csv")
        headers = ["Cycle", "Motion Phase", "Movement Direction", "Rotary Sample Valid",
                   "Measured Fixture Angle (deg)", "Raw Torque (Nm)"]
        rows = []
        for cycle in range(1, cycles + 1):
            for index, (phase, direction) in enumerate(PHASES.items()):
                root = (-1 - cycle * .25) if index == 0 else (.5 + cycle * .25)
                for angle in ANGLES:
                    torque = 0 if empty else -(2 + cycle * .1 + index) * (angle - root)
                    if cycle in invalid and not empty:
                        torque = 100
                    rows.append([cycle, phase, direction, "True", angle, torque])
        with csv_path.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(headers)
            writer.writerows(rows)
        metadata = {
            "run_status": "completed", "completed_cycles": cycles, "sample_count": len(rows),
            "csv_recorder": {"worker_complete": True, "error": None, "rows_persisted": len(rows)},
            "test_parameters": {"test_type": "empty_machine_baseline" if empty else "afo_test",
                                "afo_id": "AFO-01", "file_prefix": name,
                                "commanded_afo_speed_deg_s": 1, "commanded_afo_acceleration_deg_s2": 100,
                                "min_angle_deg": 10, "max_angle_deg": 10},
            "motion_method": "test", "torque_angle_source": "rotary",
            "rotary_zero_reference": {"calibration_sha256": "same"},
            "calibration": dict(mass_kg_per_voltage_ratio=1, standard_gravity_m_s2=9.81,
                                lever_arm_m=.4, force_angle_polynomial_deg=[0, 1]),
            "csv_file": csv_path.name,
        }
        path = folder / (name + "_metadata.json")
        path.write_text(json.dumps(metadata))
        return path


if __name__ == "__main__":
    unittest.main()
