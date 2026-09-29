import tempfile
import unittest
import json
from dataclasses import asdict
from pathlib import Path

from east_core import (
    CSV_COLUMNS,
    EmptyMachineTare,
    afo_acceleration_to_odrive_turns_s2,
    afo_degrees_to_odrive_turns,
    afo_speed_to_odrive_turns_s,
    calculate_load,
    calculate_torque_nm,
    constant_speed_span_deg,
    create_run_paths,
    empty_machine_tare_identity_mismatches,
    load_tester_config,
    make_run_metadata,
    make_preset_metadata,
    motion_timeout_seconds,
    odrive_turns_to_afo_degrees,
    preset_motion_values,
    preset_values_modified,
    reconcile_run_outcome,
    sanitise_identifier,
    validate_empty_machine_tare,
    validate_test_parameters,
)


def valid_values():
    return {
        "file_prefix": "verification",
        "operator": "Operator 1",
        "afo_id": "AFO-001",
        "fixture_id": "FIX-01",
        "calibration_id": "CAL-01",
        "test_type": "custom",
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

    def test_quasi_static_sweep_uses_bounded_60_second_timeout(self):
        motion = self.config["motion"]
        self.assertEqual(motion["motion_timeout_margin_s"], 5.0)
        self.assertEqual(motion["maximum_motion_timeout_s"], 60.0)

        timeout = motion_timeout_seconds(20.0, 0.5, 100.0, self.config)
        self.assertAlmostEqual(timeout, 45.005)
        self.assertEqual(round(timeout, 2), 45.01)
        self.assertLessEqual(timeout, motion["maximum_motion_timeout_s"])

        values = valid_values()
        values.update(
            speed_deg_s="0.5",
            acceleration_deg_s2="100",
            min_angle_deg="10",
            max_angle_deg="10",
        )
        parameters = validate_test_parameters(values, self.config)
        self.assertEqual(parameters.commanded_afo_speed_deg_s, 0.5)

        with self.assertRaisesRegex(ValueError, "exceeding the 60 s maximum"):
            motion_timeout_seconds(30.0, 0.5, 100.0, self.config)

        self.assertAlmostEqual(
            motion_timeout_seconds(20.0, 1.0, 100.0, self.config), 25.01
        )
        self.assertAlmostEqual(
            motion_timeout_seconds(20.0, 5.0, 100.0, self.config), 9.05
        )

    def test_validate_test_parameters(self):
        parameters = validate_test_parameters(valid_values(), self.config)
        self.assertEqual(parameters.cycles, 3)
        self.assertEqual(parameters.commanded_afo_speed_deg_s, 5.0)
        self.assertEqual(parameters.commanded_afo_acceleration_deg_s2, 10.0)
        self.assertEqual(asdict(parameters)["afo_id"], "AFO-001")
        self.assertEqual(parameters.test_type, "custom")

    def test_standard_presets_have_the_approved_provisional_values(self):
        expected_motion = {
            "cycles": 3,
            "minimum_angle_deg": 10.0,
            "maximum_angle_deg": 10.0,
            "speed_deg_s": 5.0,
            "acceleration_deg_s2": 100.0,
        }
        afo = self.config["test_presets"]["standard_afo_test"]
        baseline = self.config["test_presets"]["standard_empty_machine_baseline"]
        self.assertEqual(preset_motion_values(afo), expected_motion)
        self.assertEqual(preset_motion_values(baseline), expected_motion)
        self.assertEqual(afo["version"], "1.0")
        self.assertEqual(afo["test_type"], "afo_test")
        self.assertEqual(baseline["test_type"], "empty_machine_baseline")

    def test_preset_modification_detection_compares_only_motion_values(self):
        preset = self.config["test_presets"]["standard_afo_test"]
        values = {key: str(value) for key, value in preset_motion_values(preset).items()}
        self.assertFalse(preset_values_modified(values, preset))
        values["speed_deg_s"] = "6"
        self.assertTrue(preset_values_modified(values, preset))

    def test_afo_test_requires_afo_id_but_baseline_uses_reserved_id(self):
        values = valid_values()
        values.update({
            "test_type": "afo_test",
            "afo_id": "",
        })
        with self.assertRaisesRegex(ValueError, "Enter an AFO ID"):
            validate_test_parameters(values, self.config)

        values["test_type"] = "empty_machine_baseline"
        parameters = validate_test_parameters(values, self.config)
        self.assertEqual(parameters.afo_id, "EMPTY_MACHINE_BASELINE")
        self.assertEqual(parameters.test_type, "empty_machine_baseline")

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

    def test_baseline_filename_omits_reserved_afo_identifier(self):
        values = valid_values()
        values.update({
            "file_prefix": "SPEED TEST",
            "test_type": "empty_machine_baseline",
            "afo_id": "",
        })
        parameters = validate_test_parameters(values, self.config)
        self.assertEqual(parameters.afo_id, "EMPTY_MACHINE_BASELINE")
        with tempfile.TemporaryDirectory() as temporary:
            csv_path, metadata_path = create_run_paths(
                parameters, self.config, Path(temporary)
            )
        self.assertTrue(csv_path.name.startswith("SPEED_TEST_"))
        self.assertNotIn("EMPTY_MACHINE_BASELINE", csv_path.name)
        self.assertNotIn("EMPTY_MACHINE_BASELINE", metadata_path.name)

    def test_afo_filename_still_includes_afo_identifier(self):
        values = valid_values()
        values["test_type"] = "afo_test"
        parameters = validate_test_parameters(values, self.config)
        with tempfile.TemporaryDirectory() as temporary:
            csv_path, metadata_path = create_run_paths(
                parameters, self.config, Path(temporary)
            )
        self.assertTrue(csv_path.name.startswith("verification_AFO-001_"))
        self.assertTrue(metadata_path.name.startswith("verification_AFO-001_"))

    def test_csv_schema_has_unique_columns(self):
        self.assertEqual(len(CSV_COLUMNS), len(set(CSV_COLUMNS)))
        self.assertIn("Elapsed Time (s)", CSV_COLUMNS)
        self.assertIn("Commanded AFO Acceleration (deg/s^2)", CSV_COLUMNS)
        self.assertIn("Commanded Minimum Angle (deg)", CSV_COLUMNS)
        self.assertIn("Commanded Maximum Angle (deg)", CSV_COLUMNS)
        self.assertIn("Commanded Cycles", CSV_COLUMNS)
        self.assertIn("ODrive-Derived AFO Angle (deg)", CSV_COLUMNS)
        self.assertIn("ODrive-Derived AFO Velocity (deg/s)", CSV_COLUMNS)
        self.assertIn("Movement Direction", CSV_COLUMNS)
        for protocol_column in (
            "Test Type", "Preset Name", "Preset Version", "Preset Modified",
        ):
            self.assertIn(protocol_column, CSV_COLUMNS)
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

    def test_preset_metadata_preserves_original_and_actual_values(self):
        values = valid_values()
        values.update({
            "test_type": "afo_test",
            "cycles": "3",
            "min_angle_deg": "10",
            "max_angle_deg": "10",
            "speed_deg_s": "6",
            "acceleration_deg_s2": "100",
        })
        parameters = validate_test_parameters(values, self.config)
        preset = self.config["test_presets"]["standard_afo_test"]
        protocol = make_preset_metadata(
            "standard_afo_test",
            preset,
            parameters,
            "2026-09-28T10:00:00+10:00",
            modified=True,
        )
        metadata = make_run_metadata(
            parameters,
            self.config,
            tare_offset=0.001,
            csv_path=Path("verification.csv"),
            odrive_snapshot={},
            preset_metadata=protocol,
        )
        self.assertEqual(metadata["protocol"]["preset_key"], "standard_afo_test")
        self.assertEqual(metadata["protocol"]["preset_version"], "1.0")
        self.assertEqual(metadata["protocol"]["test_type"], "afo_test")
        self.assertEqual(metadata["protocol"]["original_preset_values"]["speed_deg_s"], 5.0)
        self.assertEqual(metadata["protocol"]["actual_commanded_values"]["speed_deg_s"], 6.0)
        self.assertTrue(metadata["protocol"]["preset_modified"])

    def test_empty_machine_tare_is_bound_to_fixture_calibration_and_phidget(self):
        parameters = validate_test_parameters(valid_values(), self.config)
        valid, reason = validate_empty_machine_tare(
            None, parameters, self.config
        )
        self.assertFalse(valid)
        self.assertIn("Capture an empty-machine tare", reason)
        tare = EmptyMachineTare(
            offset_v_per_v=0.001,
            captured_at="2026-09-28T10:00:00+10:00",
            operator_id="SC",
            fixture_id="FIX-01",
            calibration_id="CAL-01",
            phidget_serial_number=12345,
            phidget_channel=self.config["hardware"]["phidget_channel"],
            sample_count=20,
        )
        valid, _ = validate_empty_machine_tare(
            tare, parameters, self.config, connected_phidget_serial_number=12345
        )
        self.assertTrue(valid)

        changed = valid_values()
        changed["fixture_id"] = "FIX-02"
        valid, reason = validate_empty_machine_tare(
            tare,
            validate_test_parameters(changed, self.config),
            self.config,
            connected_phidget_serial_number=12345,
        )
        self.assertFalse(valid)
        self.assertIn("does not match tare fixture", reason)

        fixture_mismatches = empty_machine_tare_identity_mismatches(
            tare, "FIX-02", "CAL-01", self.config
        )
        self.assertEqual(len(fixture_mismatches), 1)
        self.assertIn("Fixture ID", fixture_mismatches[0])
        self.assertEqual(
            empty_machine_tare_identity_mismatches(
                tare, "FIX-01", "CAL-01", self.config
            ),
            [],
        )

        calibration_mismatches = empty_machine_tare_identity_mismatches(
            tare, "FIX-01", "CAL-02", self.config
        )
        self.assertEqual(len(calibration_mismatches), 1)
        self.assertIn("Calibration ID", calibration_mismatches[0])

        valid, reason = validate_empty_machine_tare(
            tare, parameters, self.config, connected_phidget_serial_number=99999
        )
        self.assertFalse(valid)
        self.assertIn("Connected Phidget", reason)

    def test_stored_empty_machine_tare_preserves_afo_preload_and_metadata(self):
        parameters = validate_test_parameters(valid_values(), self.config)
        tare = EmptyMachineTare(
            offset_v_per_v=0.001,
            captured_at="2026-09-28T10:00:00+10:00",
            operator_id="SC",
            fixture_id=parameters.fixture_id,
            calibration_id=parameters.calibration_id,
            phidget_serial_number=12345,
            phidget_channel=self.config["hardware"]["phidget_channel"],
            sample_count=20,
        )
        ratio_delta = 0.25 / self.config["load_cell"]["mass_kg_per_voltage_ratio"]
        mass_kg, _, force_n = calculate_load(
            tare.offset_v_per_v + ratio_delta, tare.offset_v_per_v, self.config
        )
        self.assertAlmostEqual(mass_kg, 0.25)
        self.assertGreater(force_n, 0.0)
        self.assertGreater(calculate_torque_nm(force_n, 0.0, self.config), 0.0)
        metadata = make_run_metadata(
            parameters,
            self.config,
            tare.offset_v_per_v,
            Path("verification.csv"),
            {},
            tare_metadata=tare.to_dict(),
        )
        self.assertEqual(
            metadata["calibration"]["empty_machine_tare"]["sample_count"], 20
        )

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

    def test_continuity_cannot_be_enabled_without_verified_uptime_units(self):
        with tempfile.TemporaryDirectory() as directory:
            config = json.loads(json.dumps(self.config))
            config["reference"]["session_continuity_enabled"] = True
            config["reference"]["odrive_uptime_units"] = None
            path = Path(directory) / "tester_config.json"
            path.write_text(json.dumps(config), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "uptime_units"):
                load_tester_config(path)

    def test_feedback_stale_limit_must_exceed_capture_limit(self):
        with tempfile.TemporaryDirectory() as directory:
            config = json.loads(json.dumps(self.config))
            config["reference"]["feedback_stale_after_ms"] = 100
            config["reference"]["maximum_feedback_capture_ms"] = 100
            path = Path(directory) / "tester_config.json"
            path.write_text(json.dumps(config), encoding="utf-8")
            with self.assertRaisesRegex(ValueError, "must exceed"):
                load_tester_config(path)


if __name__ == "__main__":
    unittest.main()
