import tempfile
import unittest
import json
from dataclasses import asdict
from pathlib import Path

from east_core import (
    CSV_COLUMNS,
    afo_acceleration_to_odrive_turns_s2,
    afo_degrees_to_odrive_turns,
    afo_speed_to_odrive_turns_s,
    calculate_load,
    calculate_torque_nm,
    constant_speed_span_deg,
    create_run_paths,
    load_tester_config,
    make_run_metadata,
    motion_timeout_seconds,
    odrive_turns_to_afo_degrees,
    reconcile_run_outcome,
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

    def test_motion_timeout_accounts_for_low_acceleration(self):
        timeout = motion_timeout_seconds(10.0, 5.0, 0.25, self.config)
        expected_motion_time = 2.0 * (10.0 / 0.25) ** 0.5
        self.assertAlmostEqual(
            timeout,
            expected_motion_time + self.config["motion"]["motion_timeout_margin_s"],
        )
        self.assertGreater(timeout, 9.0)

    def test_motion_timeout_accounts_for_trapezoidal_profile(self):
        timeout = motion_timeout_seconds(20.0, 5.0, 10.0, self.config)
        expected_motion_time = 20.0 / 5.0 + 5.0 / 10.0
        self.assertAlmostEqual(
            timeout,
            expected_motion_time + self.config["motion"]["motion_timeout_margin_s"],
        )

    def test_validate_test_parameters(self):
        parameters = validate_test_parameters(valid_values(), self.config)
        self.assertEqual(parameters.cycles, 3)
        self.assertEqual(parameters.commanded_afo_speed_deg_s, 5.0)
        self.assertEqual(parameters.commanded_afo_acceleration_deg_s2, 10.0)
        self.assertEqual(asdict(parameters)["afo_id"], "AFO-001")

    def test_provisional_speed_limit_is_20_deg_s(self):
        motion = self.config["motion"]
        self.assertEqual(motion["afo_degrees_per_odrive_turn"], 2.055)
        self.assertEqual(motion["maximum_speed_deg_s"], 20.0)
        self.assertEqual(motion["minimum_constant_speed_span_deg"], 5.0)
        self.assertEqual(motion["minimum_acceleration_deg_s2"], 0.1)
        self.assertEqual(motion["maximum_acceleration_deg_s2"], 100.0)
        self.assertEqual(motion["maximum_afo_angle_deg"], 15.0)

        values = valid_values()
        values.update({
            "speed_deg_s": "20",
            "acceleration_deg_s2": "100",
            "min_angle_deg": "4.5",
            "max_angle_deg": "4.5",
        })
        parameters = validate_test_parameters(values, self.config)
        self.assertEqual(parameters.commanded_afo_speed_deg_s, 20.0)

        values["speed_deg_s"] = "20.1"
        with self.assertRaisesRegex(ValueError, "between 0.1 and 20 \N{DEGREE SIGN}/s"):
            validate_test_parameters(values, self.config)

    def test_constant_speed_span_uses_entered_acceleration(self):
        self.assertAlmostEqual(constant_speed_span_deg(9.0, 20.0, 100.0), 5.0)

        values = valid_values()
        values.update({
            "speed_deg_s": "20",
            "acceleration_deg_s2": "50",
            "min_angle_deg": "4.5",
            "max_angle_deg": "4.5",
        })
        with self.assertRaisesRegex(ValueError, "required 5\N{DEGREE SIGN} constant-speed span") as raised:
            validate_test_parameters(values, self.config)
        self.assertIn("commanded acceleration: 50\N{DEGREE SIGN}/s\N{SUPERSCRIPT TWO}", str(raised.exception))
        self.assertIn("total ROM: 9\N{DEGREE SIGN}", str(raised.exception))

    def test_invalid_parameters_are_rejected(self):
        for field, value in (
            ("cycles", "0"),
            ("cycles", "1.5"),
            ("speed_deg_s", "0"),
            ("acceleration_deg_s2", "-1"),
            ("max_angle_deg", "16"),
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
            self.assertEqual(first_csv.parent.name, "EAST Logs")

    def test_csv_schema_has_unique_columns(self):
        self.assertEqual(len(CSV_COLUMNS), len(set(CSV_COLUMNS)))
        self.assertIn("Elapsed Time (s)", CSV_COLUMNS)
        self.assertIn("Commanded AFO Acceleration (deg/s^2)", CSV_COLUMNS)
        self.assertIn("Commanded Minimum Angle (deg)", CSV_COLUMNS)
        self.assertIn("Commanded Maximum Angle (deg)", CSV_COLUMNS)
        self.assertIn("Commanded Cycles", CSV_COLUMNS)
        self.assertIn("ODrive-Derived AFO Angle (deg)", CSV_COLUMNS)
        self.assertIn("ODrive-Derived AFO Velocity (deg/s)", CSV_COLUMNS)
        for identifier in ("File Name Prefix", "Operator ID", "AFO ID", "Fixture ID", "Calibration ID"):
            self.assertIn(identifier, CSV_COLUMNS)

    def test_metadata_preserves_commanded_acceleration_key(self):
        parameters = validate_test_parameters(valid_values(), self.config)
        metadata = make_run_metadata(
            parameters,
            self.config,
            tare_offset=0.001,
            csv_path=Path("verification.csv"),
            odrive_snapshot={},
        )
        logged = metadata["test_parameters"]
        self.assertEqual(logged["commanded_afo_acceleration_deg_s2"], 10.0)
        self.assertEqual(logged["commanded_afo_speed_deg_s"], 5.0)
        self.assertEqual(logged["min_angle_deg"], 4.0)
        self.assertEqual(logged["max_angle_deg"], 4.0)
        self.assertEqual(logged["cycles"], 3)
        self.assertIn("commanded values", metadata["test_parameter_status"])

    def test_identifier_sanitisation(self):
        self.assertEqual(sanitise_identifier("AFO 01 / left"), "AFO_01_left")

    def test_late_stop_or_acquisition_failure_cannot_complete_run(self):
        status, error = reconcile_run_outcome(True, True, None, "aborted", None)
        self.assertEqual(status, "aborted")
        self.assertIn("stop requested", error)

        status, error = reconcile_run_outcome(
            True, False, "CSV flush error: disk full", "aborted", None
        )
        self.assertEqual(status, "error")
        self.assertIn("disk full", error)

        status, error = reconcile_run_outcome(True, False, None, "aborted", None)
        self.assertEqual((status, error), ("completed", None))

    def test_watchdog_cannot_be_enabled_without_verified_health_coupling(self):
        with tempfile.TemporaryDirectory() as directory:
            config = json.loads(json.dumps(self.config))
            config["reference"]["watchdog_enabled"] = True
            config["reference"]["watchdog_health_coupled_verified"] = False
            path = Path(directory) / "tester_config.json"
            path.write_text(json.dumps(config), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "health-coupled"):
                load_tester_config(path)


if __name__ == "__main__":
    unittest.main()
