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
        namespace = dict(queue=queue, threading=threading, time=time, datetime=datetime, math=math)
        exec(compile(ast.Module(body=[worker], type_ignores=[]), str(path), "exec"), namespace)
        return namespace["RotaryReader"]

    def test_exact_hub_address_mode_and_close(self):
        for mode, module, constructor in (
            ("Voltage ratio", "Phidget22.Devices.VoltageRatioInput", "VoltageRatioInput"),
            ("Voltage", "Phidget22.Devices.VoltageInput", "VoltageInput"),
        ):
            reader = self.reader_class()(750256, mode)
            channel = self.channel()
            channel.getDeviceSKU.return_value = constructor + "_PORT"
            reader.stop.wait = Mock(return_value=True)
            channel.getVoltageRatio.return_value = 0.5
            channel.getVoltage.return_value = 2.5
            fake = SimpleNamespace(**{constructor: Mock(return_value=channel)})
            with patch.dict("sys.modules", {module: fake}):
                reader._run()
            channel.setDeviceSerialNumber.assert_called_once_with(750256)
            channel.setHubPort.assert_called_once_with(0)
            channel.setIsHubPortDevice.assert_called_once_with(True)
            channel.close.assert_called_once()
            events = []
            while not reader.events.empty():
                events.append(reader.events.get_nowait())
            readings = [value for kind, value in events if kind == "sample"]
            self.assertEqual(len(readings), 1)
            metadata = next(value for kind, value in events if kind == "connected")
            self.assertEqual(metadata["sku"], "HUB0007")
            self.assertEqual(metadata["channel_sku"], constructor + "_PORT")
            self.assertEqual(readings[0][2], 0.5 if mode == "Voltage ratio" else 2.5)
            self.assertEqual([kind for kind, _ in events if kind != "progress"], ["connected", "sample", "closed"])
            channel.setOnVoltageRatioChangeHandler.assert_not_called()
            channel.setOnVoltageChangeHandler.assert_not_called()

    def test_attachment_failure_closes_channel_and_reports_error(self):
        reader = self.reader_class()(750256, "Voltage ratio")
        channel = Mock()
        channel.openWaitForAttachment.side_effect = RuntimeError("missing hub")
        fake = SimpleNamespace(VoltageRatioInput=Mock(return_value=channel))
        with patch.dict("sys.modules", {"Phidget22.Devices.VoltageRatioInput": fake}):
            reader._run()
        events = list(reader.events.queue)
        error = next(value for kind, value in events if kind == "error")
        self.assertIn("Opening hub 750256", error)
        self.assertIn("missing hub", error)
        self.assertFalse(any(kind == "connected" for kind, _ in events))
        channel.close.assert_called_once()


    def channel(self):
        channel = Mock()
        channel.getDeviceSKU.return_value = "VoltageRatioInput_PORT"
        channel.getHub.return_value.getDeviceSKU.return_value = "HUB0007"
        channel.getHub.return_value.getDeviceSerialNumber.return_value = 750256
        channel.getDeviceSerialNumber.return_value = 750256
        channel.getHubPort.return_value = 0
        channel.getIsHubPortDevice.return_value = True
        channel.getMinDataInterval.return_value = 1
        channel.getMaxDataInterval.return_value = 60000
        channel.getVoltageRatio.return_value = 0.5
        return channel

    def test_wrong_parent_hub_or_address_is_rejected_before_reading(self):
        for mismatch in ("hub_sku", "hub_serial", "channel_serial", "port", "port_device"):
            with self.subTest(mismatch=mismatch):
                reader = self.reader_class()(750256, "Voltage ratio")
                channel = self.channel()
                if mismatch == "hub_sku":
                    channel.getHub.return_value.getDeviceSKU.return_value = "HUB0000"
                elif mismatch == "hub_serial":
                    channel.getHub.return_value.getDeviceSerialNumber.return_value = 123
                elif mismatch == "channel_serial":
                    channel.getDeviceSerialNumber.return_value = 123
                elif mismatch == "port":
                    channel.getHubPort.return_value = 1
                else:
                    channel.getIsHubPortDevice.return_value = False
                fake = SimpleNamespace(VoltageRatioInput=Mock(return_value=channel))
                with patch.dict("sys.modules", {"Phidget22.Devices.VoltageRatioInput": fake}):
                    reader._run()
                events = list(reader.events.queue)
                self.assertTrue(any(kind == "error" for kind, _ in events))
                self.assertFalse(any(kind == "connected" for kind, _ in events))
                channel.getVoltageRatio.assert_not_called()
                channel.close.assert_called_once()

    def test_initial_unknown_value_is_retried_before_connection(self):
        reader = self.reader_class()(750256, "Voltage ratio")
        unknown = RuntimeError("first value unavailable")
        unknown.code = 51
        channel = self.channel()
        channel.getVoltageRatio.side_effect = [unknown, 0.5]
        reader.stop.wait = Mock(side_effect=[False, True])
        fake = SimpleNamespace(VoltageRatioInput=Mock(return_value=channel))
        with patch.dict("sys.modules", {"Phidget22.Devices.VoltageRatioInput": fake}):
            reader._run()
        events = list(reader.events.queue)
        self.assertFalse(any(kind == "error" for kind, _ in events))
        self.assertEqual(sum(kind == "sample" for kind, _ in events), 1)
        channel.close.assert_called_once()

    def test_getter_failure_is_reported_without_false_connection(self):
        reader = self.reader_class()(750256, "Voltage ratio")
        channel = self.channel()
        channel.getVoltageRatio.side_effect = RuntimeError("driver read failed")
        fake = SimpleNamespace(VoltageRatioInput=Mock(return_value=channel))
        with patch.dict("sys.modules", {"Phidget22.Devices.VoltageRatioInput": fake}):
            reader._run()
        events = list(reader.events.queue)
        self.assertIn("driver read failed", next(value for kind, value in events if kind == "error"))
        self.assertFalse(any(kind == "connected" for kind, _ in events))
        channel.close.assert_called_once()

    def test_cancel_during_attachment_prevents_configuration_or_reading(self):
        reader = self.reader_class()(750256, "Voltage ratio")
        channel = self.channel()
        channel.openWaitForAttachment.side_effect = lambda _timeout: reader.stop.set()
        fake = SimpleNamespace(VoltageRatioInput=Mock(return_value=channel))
        with patch.dict("sys.modules", {"Phidget22.Devices.VoltageRatioInput": fake}):
            reader._run()
        channel.getVoltageRatio.assert_not_called()
        self.assertFalse(any(kind == "connected" for kind, _ in list(reader.events.queue)))
        channel.close.assert_called_once()

    def test_stationary_values_are_sampled_without_change_callbacks(self):
        reader = self.reader_class()(750256, "Voltage ratio")
        channel = self.channel()
        reader.stop.wait = Mock(side_effect=[False, False, True])
        fake = SimpleNamespace(VoltageRatioInput=Mock(return_value=channel))
        with patch.dict("sys.modules", {"Phidget22.Devices.VoltageRatioInput": fake}):
            reader._run()
        values = [value[2] for kind, value in list(reader.events.queue) if kind == "sample"]
        self.assertEqual(values, [0.5, 0.5, 0.5])
        self.assertEqual(channel.getVoltageRatio.call_count, 3)
        channel.setOnVoltageRatioChangeHandler.assert_not_called()


class RotaryConnectionUiTests(unittest.TestCase):
    def window(self):
        path = Path(__file__).resolve().parents[1] / "east_rotary_gui.py"
        tree = ast.parse(path.read_text())
        node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "RotaryCalibrationWindow")
        namespace = dict(time=time, queue=queue)
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), "exec"), namespace)
        window = namespace["RotaryCalibrationWindow"].__new__(namespace["RotaryCalibrationWindow"])
        window.closed, window.closing = False, False
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
        window.window.after.assert_called_once()


if __name__ == "__main__":
    unittest.main()
