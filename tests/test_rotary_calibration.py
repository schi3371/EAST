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
from east_rotary_calibration import summarise_hold, fit_calibration, StationaryCaptureGuard, SensorOnlyCaptureGuard


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


class SensorOnlyCaptureTests(unittest.TestCase):
    app = CaptureGuardTests.app
    def test_unverified_reference_and_stale_feedback_do_not_block_sensor_only(self):
        app = self.app()
        app.startup_block_reason = "Unverified machine zero"
        app.reference_manager.require_verified.side_effect = ValueError("zero invalid")
        app.get_feedback.side_effect = ValueError("no fresh motor feedback")
        app.motion_coordinator.request_stop()
        guard = SensorOnlyCaptureGuard(app)
        self.assertIsNone(guard.check())
        self.assertFalse(app.motion_coordinator.motor_idle_confirmed)
        with self.assertRaises(MotionConflictError):
            app.motion_coordinator.acquire("manual-step")
        guard.close()
        self.assertIsNone(app.motion_coordinator.owner)
        self.assertFalse(app.motion_coordinator.motor_idle_confirmed)
        with self.assertRaises(MotionConflictError):
            app.motion_coordinator.acquire("manual-step")
        app.reference_manager.require_verified.assert_not_called()
        self.assertEqual(app.odrive_adapter.mock_calls, [])

    def test_observation_reservation_cannot_submit_motor_command(self):
        app = self.app()
        guard = SensorOnlyCaptureGuard(app)
        command = Mock()
        with self.assertRaisesRegex(MotionConflictError, "Passive observation"):
            app.motion_coordinator.submit_command(guard.token, command)
        command.assert_not_called()
        guard.close()

    def test_faulted_idle_feedback_is_observation_only_without_clearing_fault(self):
        app = self.app()
        self.feedback.active_errors = 123
        app.reference_manager.require_verified.side_effect = ValueError("zero invalid")
        guard = SensorOnlyCaptureGuard(app)
        self.assertIs(guard.check(), self.feedback)
        self.assertEqual(self.feedback.active_errors, 123)
        guard.close()
        app.reference_manager.require_verified.assert_not_called()

    def test_active_motion_or_existing_owner_blocks_passive_capture(self):
        app = self.app()
        token = app.motion_coordinator.acquire("manual-step")
        with self.assertRaises(MotionConflictError):
            SensorOnlyCaptureGuard(app)
        app.motion_coordinator.release(token)
        self.feedback.current_state = 8
        with self.assertRaisesRegex(ValueError, "Stop motor"):
            SensorOnlyCaptureGuard(app)
        self.assertIsNone(app.motion_coordinator.owner)

    def test_position_change_aborts_sensor_only_hold(self):
        app = self.app()
        guard = SensorOnlyCaptureGuard(app)
        self.feedback.position_turns += 1
        with self.assertRaisesRegex(ValueError, "Movement"):
            guard.check()
        guard.close()

    def test_sensor_only_hold_can_be_valid_without_motor_feedback(self):
        samples = [dict(raw_reading=0.47, sensor_valid=True,
                        motor_position_turns=None, motor_feedback_valid=False) for _ in range(89)]
        hold = summarise_hold(samples, dict(input_mode="Voltage ratio", capture_mode="Sensor only"))
        self.assertTrue(hold["valid"])
        self.assertIsNone(hold["motor_position_mean_turns"])
        bad = summarise_hold(samples, dict(input_mode="Voltage ratio", capture_mode="Sensor + motor"))
        self.assertFalse(bad["valid"])
        samples[0]["sensor_valid"] = False
        self.assertFalse(summarise_hold(samples, dict(input_mode="Voltage ratio", capture_mode="Sensor only"))["valid"])

    def test_sensor_only_points_never_enter_motor_fit_even_if_turns_present(self):
        data = [point(a) for a in (-10, 0, 10)]
        for p in data:
            p["capture_mode"] = "Sensor only"
        model = fit_calibration(data)
        self.assertAlmostEqual(model["slope"], 360)
        self.assertNotIn("motor_conversion_fit", model)
        self.assertEqual(model["motor_conversion_point_count"], 0)

    def test_paired_fit_ignores_sensor_only_turns(self):
        data = [point(a) for a in (-10, 0, 10)]
        extra = point(5)
        extra.update(capture_mode="Sensor only", motor_position_mean_turns=1000)
        model = fit_calibration(data + [extra])
        self.assertAlmostEqual(model["motor_conversion_fit"]["slope"], 2.263)
        self.assertEqual(model["motor_conversion_point_count"], 3)


class RotaryConnectionUiTests(unittest.TestCase):
    def window(self):
        path = Path(__file__).resolve().parents[1] / "east_rotary_gui.py"
        tree = ast.parse(path.read_text())
        node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "RotaryCalibrationWindow")
        namespace = dict(time=time, queue=queue)
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), "exec"), namespace)
        window = namespace["RotaryCalibrationWindow"].__new__(namespace["RotaryCalibrationWindow"])
        window.closed, window.closing = False, False
        window.positioning_reference = None
        window._motor_frame = Mock(return_value="test-frame")
        window.sensor, window.latest, window.capture = None, None, None
        window.window, window.status, window.live_label = Mock(), Mock(), Mock()
        window.app = Mock()
        window._controls, window._record_sample, window.cancel_capture = Mock(), Mock(), Mock()
        reader = SimpleNamespace(events=queue.Queue(), timed_out=False, stop=threading.Event(),
            started_monotonic=time.monotonic(), CONNECT_TIMEOUT_S=8,
            phase="Opening hub", serial=750256, thread=Mock())
        reader.thread.is_alive.return_value = True
        window.reader = reader
        return window, reader

    def test_nested_gui_event_cannot_leave_connect_disabled_after_worker_exits(self):
        window, reader = self.window()
        window.preview, window.points, window.session_dir = False, [], None
        for name in ("serial", "mode_menu", "angle", "repeat", "duration", "role_menu",
                     "approach_menu", "ack", "instrument", "resolution", "supply",
                     "connect_button", "disconnect_button", "capture_button", "cancel_button",
                     "fit_button", "new_button"):
            setattr(window, name, Mock())
        window._controls = type(window)._controls.__get__(window)
        def nested_failure(**_kwargs):
            window.serial.configure.side_effect = None
            window.reader = None
            window._controls()
        window.serial.configure.side_effect = nested_failure
        window._controls()
        self.assertEqual(window.connect_button.configure.call_args.kwargs["state"], "normal")
        self.assertEqual(window.disconnect_button.configure.call_args.kwargs["state"], "disabled")
        self.assertFalse(window._updating_controls)

    def test_sensor_only_context_preserves_existing_session_without_verified_zero(self):
        window, reader = self.window()
        window.capture_mode = Mock(get=lambda: "Sensor only")
        window.app.operator_input.get.return_value = "SC"
        window.app.fixture_id_input.get.return_value = "fixture"
        window.app.reference_manager.require_verified.side_effect = ValueError("zero invalid")
        window.app.reference_manager.record = None
        window.sensor = dict(hub_serial=750256, input_mode="Voltage ratio")
        window.identity = dict(reference_id="previous-zero")
        self.assertEqual(window._context()["reference_id"], "previous-zero")
        window.identity = None
        self.assertIsNone(window._context()["reference_id"])
        window.app.reference_manager.require_verified.assert_not_called()
        window.capture_mode.get = lambda: "Sensor + motor"
        self.assertIsNone(window._context()["reference_id"])
        window.app.reference_manager.require_verified.assert_not_called()

    def test_connection_and_first_sample_reach_gui(self):
        window, reader = self.window()
        metadata = dict(hub_serial=750256, input_mode="Voltage ratio", interval_ms=100)
        sample = (time.monotonic(), "test", 0.5)
        reader.events.put(("connected", metadata))
        reader.events.put(("sample", sample))
        window._tick()
        self.assertEqual(window.sensor, metadata)
        self.assertEqual(window.latest, sample)
        self.assertIn("0.50000000 V/V", window.live_label.configure.call_args.kwargs["text"])
        window.window.after.assert_called_once()

    def test_hung_worker_gets_watchdog_error_and_late_connection_is_ignored(self):
        window, reader = self.window()
        reader.started_monotonic -= 9
        window._tick()
        self.assertTrue(reader.timed_out)
        self.assertTrue(reader.stop.is_set())
        self.assertIs(window.reader, reader)  # do not create an overlapping channel
        message = window.status.configure.call_args.kwargs["text"]
        self.assertIn("timed out", message)
        self.assertIn("Opening hub", message)
        window.app.update_terminal.assert_called_once()
        reader.events.put(("connected", dict(hub_serial=750256)))
        window._tick()
        self.assertIsNone(window.sensor)

    def test_driver_error_remains_visible_and_finished_worker_allows_retry(self):
        window, reader = self.window()
        reader.events.put(("error", "EPHIDGET_TIMEOUT: no matching device"))
        reader.events.put(("closed", None))
        reader.thread.is_alive.return_value = False
        window._tick()
        self.assertIsNone(window.reader)
        self.assertIn("no matching device", window.status.configure.call_args.kwargs["text"])
        window.app.update_terminal.assert_called_once()
        window.window.after.assert_called_once()

    def test_late_progress_does_not_overwrite_driver_error(self):
        window, reader = self.window()
        reader.events.put(("error", "driver failure"))
        reader.events.put(("progress", "Waiting for first reading"))
        window._tick()
        self.assertIn("driver failure", window.status.configure.call_args.kwargs["text"])

    def test_event_processing_error_does_not_stop_gui_pump(self):
        window, reader = self.window()
        reader.events.put(("progress", "Loading driver"))
        window.app.update_terminal.side_effect = RuntimeError("terminal widget error")
        with self.assertRaises(RuntimeError):
            window._tick()
        window.window.after.assert_called_once()

    def test_queue_drain_is_bounded(self):
        window, reader = self.window()
        for _ in range(250):
            reader.events.put(("sample", (time.monotonic(), "test", 0.5)))
        window._tick()
        self.assertEqual(reader.events.qsize(), 50)

    def test_closing_shared_window_unsubscribes_without_stopping_sensor_or_reattaching(self):
        window,reader=self.window()
        window._shared_sensor=SimpleNamespace(reader=reader)
        window.app.motion_coordinator.owner=None
        reader.unsubscribe=Mock()
        type(window).close(window)
        reader.unsubscribe.assert_called_once()
        self.assertFalse(reader.stop.is_set())
        self.assertIsNone(window.reader)
        window._tick()
        self.assertTrue(window.closed)
        window.window.destroy.assert_called_once()

    def test_main_connection_is_observed_without_a_second_phidget_channel(self):
        window,old_reader=self.window()
        window.reader=None
        physical=SimpleNamespace(stop=threading.Event(),subscribe=Mock(return_value=old_reader))
        window._shared_sensor=SimpleNamespace(reader=physical)
        window._tick()
        physical.subscribe.assert_called_once()
        self.assertIs(window.reader,old_reader)
        window.window.after.assert_called_once()


if __name__ == "__main__":
    unittest.main()
