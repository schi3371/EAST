import tempfile
import unittest
from dataclasses import asdict
from pathlib import Path

from east_core import (
    CSV_COLUMNS,
    afo_acceleration_to_odrive_turns_s2,
    afo_degrees_to_odrive_turns,
    afo_speed_to_odrive_turns_s,
    calculate_load,
    calculate_torque_nm,
    create_run_paths,
    load_tester_config,
    odrive_turns_to_afo_degrees,
    sanitise_identifier,
    validate_test_parameters,
)


def valid_values():
    return {
        "file_prefix": "verification",
        "operator": "Operator 1",
        "afo_id": "AFO-001",
        "fixture_id": "FIX-01",
        "calibration_id": "CAL-01",
        "cycles": "3",
        "speed_deg_s": "5",
        "acceleration_deg_s2": "10",
        "min_angle_deg": "4",
        "max_angle_deg": "4",
    }


class EastCoreTests(unittest.TestCase):
    def setUp(self):
        self.config = load_tester_config()

    def test_motion_conversions_are_consistent(self):
        turns = afo_degrees_to_odrive_turns(10.0, self.config)
        self.assertAlmostEqual(odrive_turns_to_afo_degrees(turns, self.config), 10.0)
        self.assertAlmostEqual(afo_speed_to_odrive_turns_s(10.0, self.config), turns)
        self.assertAlmostEqual(afo_acceleration_to_odrive_turns_s2(10.0, self.config), turns)

    def test_validate_test_parameters(self):
        parameters = validate_test_parameters(valid_values(), self.config)
        self.assertEqual(parameters.cycles, 3)
        self.assertEqual(parameters.commanded_afo_speed_deg_s, 5.0)
        self.assertEqual(asdict(parameters)["afo_id"], "AFO-001")

    def test_invalid_parameters_are_rejected(self):
        for field, value in (
            ("cycles", "0"),
            ("cycles", "1.5"),
            ("speed_deg_s", "0"),
            ("acceleration_deg_s2", "-1"),
            ("max_angle_deg", "13"),
            ("operator", ""),
        ):
            with self.subTest(field=field, value=value):
                values = valid_values()
                values[field] = value
                with self.assertRaises(ValueError):
                    validate_test_parameters(values, self.config)

    def test_load_and_torque_units(self):
        ratio_delta_for_one_kg = 1.0 / self.config["load_cell"]["mass_kg_per_voltage_ratio"]
        mass_kg, weight_g, force_n = calculate_load(ratio_delta_for_one_kg, 0.0, self.config)
        self.assertAlmostEqual(mass_kg, 1.0)
        self.assertAlmostEqual(weight_g, 1000.0)
        self.assertAlmostEqual(force_n, self.config["load_cell"]["standard_gravity_m_s2"])
        self.assertGreater(calculate_torque_nm(force_n, 0.0, self.config), 0)

    def test_output_paths_are_sanitised_and_unique(self):
        parameters = validate_test_parameters(valid_values(), self.config)
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first_csv, first_metadata = create_run_paths(parameters, self.config, root)
            first_csv.touch()
            second_csv, second_metadata = create_run_paths(parameters, self.config, root)
            self.assertNotEqual(first_csv, second_csv)
            self.assertNotEqual(first_metadata, second_metadata)
            self.assertEqual(first_csv.parent.name, "OrthoSim Logs")

    def test_csv_schema_has_unique_columns(self):
        self.assertEqual(len(CSV_COLUMNS), len(set(CSV_COLUMNS)))
        self.assertIn("Elapsed Time (s)", CSV_COLUMNS)
        self.assertIn("Measured AFO Velocity (deg/s)", CSV_COLUMNS)

    def test_identifier_sanitisation(self):
        self.assertEqual(sanitise_identifier("AFO 01 / left"), "AFO_01_left")


if __name__ == "__main__":
    unittest.main()
