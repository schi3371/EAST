import ast
from datetime import datetime
import math
from pathlib import Path
import queue
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from east_core import load_tester_config
from east_reference import MotionCoordinator, MotionConflictError
from east_rotary_calibration import summarise_hold, fit_calibration, StationaryCaptureGuard


def point(angle, role="Calibration", mode="Voltage ratio", sign=1):
    return {"point_id": str(angle), "role": role, "reference_angle_deg": angle,
            "input_mode": mode, "raw_mean": 0.5 + sign * angle / 360,
            "motor_position_mean_turns": 3 + angle / 2.263, "valid": True}


class RotaryCalibrationTests(unittest.TestCase):
    def test_independent_angle_fit_and_motor_conversion_not_assumed_factor(self):
        data = [point(a) for a in (-10, -5, 0, 5, 10)]
        data += [point(2.5, "Validation"), point(-2.5, "Validation")]
        fit = fit_calibration(data)
        self.assertAlmostEqual(fit["slope"], 360)
        self.assertAlmostEqual(fit["fitted_zero_reading"], 0.5)
        self.assertAlmostEqual(fit["motor_conversion_fit"]["slope"], 2.263)
        self.assertLess(fit["validation_max_abs_error_deg"], 1e-10)
        self.assertFalse(fit["motor_control_changed"])

    def test_validation_is_not_used_to_fit(self):
        data = [point(a) for a in (-10, 0, 10)]
        wrong = point(5, "Validation")
        wrong["raw_mean"] += 0.01
        fit = fit_calibration(data + [wrong])
        self.assertAlmostEqual(fit["slope"], 360)
        self.assertAlmostEqual(fit["validation_max_abs_error_deg"], 3.6)

    def test_missing_zero_or_one_sided_points_rejected(self):
        for angles in ((-10, -5, 5, 10), (0, 5, 10), (-10, -5, 0), (0, 0, 10)):
            with self.assertRaises(ValueError):
                fit_calibration([point(a) for a in angles])

    def test_negative_sensor_direction_supported(self):
        fit = fit_calibration([point(a, sign=-1) for a in (-10, 0, 10)])
        self.assertAlmostEqual(fit["slope"], -360)

    def test_invalid_holds_excluded_and_mixed_modes_rejected(self):
        data = [point(a) for a in (-10, 0, 10)]
        bad = point(8)
        bad.update(valid=False, raw_mean=math.nan)
        self.assertEqual(fit_calibration(data + [bad])["count"], 3)
        with self.assertRaisesRegex(ValueError, "mix"):
            fit_calibration(data + [point(5, mode="Voltage")])

    def test_rollover_between_points_rejected(self):
        data = [point(a) for a in (-10, 0, 10)]
        for p, x in zip(data, (0.98, 0.995, 0.02)):
            p["raw_mean"] = x
        with self.assertRaisesRegex(ValueError, "rollover"):
            fit_calibration(data)

    def test_constant_and_nonfinite_readings_rejected(self):
        data = [point(a) for a in (-10, 0, 10)]
        for p in data:
            p["raw_mean"] = 0.5
        with self.assertRaises(ValueError):
            fit_calibration(data)
        data[0]["raw_mean"] = math.inf
        with self.assertRaises(ValueError):
            fit_calibration(data)

    def test_hold_statistics_and_abort_status_preserve_original_samples(self):
        samples = [dict(raw_reading=0.5 + i * 1e-5, sensor_valid=True,
                        motor_position_turns=2, motor_feedback_valid=True) for i in range(10)]
        before = [dict(s) for s in samples]
        metadata = dict(input_mode="Voltage ratio")
        result = summarise_hold(samples, metadata)
        self.assertTrue(result["valid"])
        self.assertEqual(result["samples"], 10)
        self.assertAlmostEqual(result["raw_mean"], 0.500045)
        aborted = summarise_hold(samples, metadata, failure="Operator Stop")
        self.assertFalse(aborted["valid"])
        self.assertEqual(aborted["reason"], "Operator Stop")
        self.assertEqual(samples, before)

    def test_bad_feedback_and_rollover_within_hold_are_excluded(self):
        samples = [dict(raw_reading=0.5, sensor_valid=True,
                        motor_position_turns=2, motor_feedback_valid=True) for _ in range(10)]
        samples[0]["motor_feedback_valid"] = False
        self.assertFalse(summarise_hold(samples, dict(input_mode="Voltage ratio"))["valid"])
        samples[0].update(motor_feedback_valid=True, raw_reading=0.99)
        samples[1]["raw_reading"] = 0.01
        self.assertIn("rollover", summarise_hold(samples, dict(input_mode="Voltage ratio"))["reason"])


class CaptureGuardTests(unittest.TestCase):
    def app(self):
        self.feedback = SimpleNamespace(position_turns=3, velocity_turns_s=0,
                                        active_errors=0, current_state=1)
        app = Mock()
        app.startup_block_reason = None
        app.strain_test_active = False
        app.system_config = load_tester_config()
        app.motion_coordinator = MotionCoordinator()
        app.reference_manager.require_verified.return_value = SimpleNamespace(reference_id="zero-1")
        app.reference_manager.record.fixture_id = "fixture"
        app.fixture_id_input.get.return_value = "fixture"
        app.get_feedback.side_effect = lambda: self.feedback
        return app

    def test_capture_owns_motion_gate_without_motor_writes(self):
        app = self.app()
        guard = StationaryCaptureGuard(app)
        self.assertEqual(app.motion_coordinator.owner, "rotary-static-capture")
        with self.assertRaises(MotionConflictError):
            app.motion_coordinator.acquire("manual-step")
        guard.check()
        guard.close()
        self.assertIsNone(app.motion_coordinator.owner)
        self.assertEqual(app.odrive_adapter.mock_calls, [])

    def test_movement_active_axis_stale_feedback_and_stop_are_rejected(self):
        for change in ("movement", "axis", "stale", "stop", "reference"):
            app = self.app()
            guard = StationaryCaptureGuard(app)
            if change == "movement":
                self.feedback.position_turns += 1
            elif change == "axis":
                self.feedback.current_state = 8
            elif change == "stale":
                app.get_feedback.side_effect = ValueError("stale")
            elif change == "stop":
                app.motion_coordinator.request_stop()
            else:
                app.reference_manager.require_verified.return_value.reference_id = "new-zero"
            with self.assertRaises((ValueError, MotionConflictError)):
                guard.check()
            guard.close()
            self.assertIsNone(app.motion_coordinator.owner)

    def test_failure_on_acquisition_releases_gate(self):
        app = self.app()
        app.get_feedback.side_effect = ValueError("stale")
        with self.assertRaises(ValueError):
            StationaryCaptureGuard(app)
        self.assertIsNone(app.motion_coordinator.owner)


class RotaryReaderTests(unittest.TestCase):
    def reader_class(self):
        # Load only the worker, without requiring GUI libraries for offline tests.
        path = Path(__file__).resolve().parents[1] / "east_rotary_gui.py"
        tree = ast.parse(path.read_text())
        worker = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "RotaryReader")
        namespace = dict(queue=queue, threading=threading, time=time, datetime=datetime)
        exec(compile(ast.Module(body=[worker], type_ignores=[]), str(path), "exec"), namespace)
        return namespace["RotaryReader"]

    def test_exact_hub_address_mode_and_close(self):
        for mode, module, constructor in (
            ("Voltage ratio", "Phidget22.Devices.VoltageRatioInput", "VoltageRatioInput"),
            ("Voltage", "Phidget22.Devices.VoltageInput", "VoltageInput"),
        ):
            reader = self.reader_class()(750256, mode)
            channel = Mock()
            channel.getDeviceSKU.return_value = "HUB0007"
            channel.getDeviceSerialNumber.return_value = 750256
            channel.getMinDataInterval.return_value = 1
            channel.getMaxDataInterval.return_value = 60000
            reader.stop.wait = Mock()
            fake = SimpleNamespace(**{constructor: Mock(return_value=channel)})
            with patch.dict("sys.modules", {module: fake}):
                reader._run()
            channel.setDeviceSerialNumber.assert_called_once_with(750256)
            channel.setHubPort.assert_called_once_with(0)
            channel.setIsHubPortDevice.assert_called_once_with(True)
            channel.close.assert_called_once()
            self.assertEqual(reader.events.get_nowait()[0], "connected")
            self.assertEqual(reader.events.get_nowait()[0], "closed")

    def test_attachment_failure_closes_channel_and_reports_error(self):
        reader = self.reader_class()(750256, "Voltage ratio")
        channel = Mock()
        channel.openWaitForAttachment.side_effect = RuntimeError("missing hub")
        fake = SimpleNamespace(VoltageRatioInput=Mock(return_value=channel))
        with patch.dict("sys.modules", {"Phidget22.Devices.VoltageRatioInput": fake}):
            reader._run()
        self.assertEqual(reader.events.get_nowait(), ("error", "missing hub"))
        channel.close.assert_called_once()


if __name__ == "__main__":
    unittest.main()
