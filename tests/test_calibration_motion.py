import math
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from east_core import load_tester_config
from east_reference import MotionCoordinator, MotionConflictError
from east_calibration_motion import CalibrationPositioningReference
from east_rotary_calibration import RawMotorCaptureGuard, fit_calibration
from test_rotary_calibration import point
from test_strain_sequence import gui_method


class CalibrationMotionTests(unittest.TestCase):
    def app(self):
        self.feedback = SimpleNamespace(position_turns=4.0, velocity_turns_s=0,
            current_state=1, active_errors=0, system_uptime=100)
        app = SimpleNamespace(system_config=load_tester_config(), odrive_adapter=object(),
            odrive_connection_generation=1, startup_block_reason=None, strain_test_active=False,
            feedback_retry_pending=False, monitor_error=None, motion_coordinator=MotionCoordinator(),
            operator_input=Mock(), fixture_id_input=Mock(), reference_manager=Mock(),
            empty_machine_tare=None, _validate_live_motion_config=Mock(),
            _write_runtime_checkpoint=Mock(),
            _revalidate_hardware_identity=Mock(), get_feedback=Mock(side_effect=lambda: self.feedback))
        app.operator_input.get.return_value = "SC"
        app.fixture_id_input.get.return_value = "fixture"
        app.reference_manager.require_verified.side_effect = ValueError("zero invalid")
        return app

    def test_nonzero_anchor_never_sets_machine_zero_or_tares(self):
        app = self.app()
        ref = CalibrationPositioningReference.establish(app, -11, acknowledged=True)
        self.assertAlmostEqual(ref.angle(4), -11)
        self.assertAlmostEqual(ref.angle(ref.target(4, 1, 1)), -11 + 2.055)
        self.assertIsNone(app.empty_machine_tare)
        self.assertEqual(app.reference_manager.mock_calls, [])

    def test_reconnect_requires_explicit_reanchor_without_erasing_position(self):
        app = self.app()
        ref = CalibrationPositioningReference.establish(app, -10, acknowledged=True)
        app.odrive_adapter = object()
        self.feedback.position_turns = 0
        with self.assertRaisesRegex(ValueError, "Reconnect"):
            ref.check(app)
        self.assertFalse(ref.active)
        resumed = CalibrationPositioningReference.establish(app, -10, acknowledged=True)
        self.assertAlmostEqual(resumed.angle(0), -10)
        self.assertEqual(app.reference_manager.mock_calls, [])

    def test_missing_ack_bad_angle_motion_and_fault_block_anchor(self):
        for kind in ("ack", "angle", "moving", "fault", "retry", "owner", "operator"):
            app = self.app()
            ack, angle = True, 0
            if kind == "ack": ack = False
            if kind == "angle": angle = math.nan
            if kind == "moving": self.feedback.current_state = 8
            if kind == "fault": self.feedback.active_errors = 1
            if kind == "retry": app.feedback_retry_pending = True
            if kind == "owner": app.motion_coordinator.acquire("test")
            if kind == "operator": app.operator_input.get.return_value = ""
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                CalibrationPositioningReference.establish(app, angle, acknowledged=ack)

    def test_limits_block_full_step_and_identity_reset_closing_block_reference(self):
        app = self.app()
        ref = CalibrationPositioningReference.establish(app, 14, acknowledged=True)
        with self.assertRaisesRegex(ValueError, "travel limits"):
            ref.target(4, 1, 1)
        self.assertLess(ref.target(4, -1, 1), 4)
        self.feedback.system_uptime = 1
        with self.assertRaisesRegex(ValueError, "reset"):
            ref.check(app)
        ref = CalibrationPositioningReference.establish(app, 0, acknowledged=True)
        app.fixture_id_input.get.return_value = "different"
        with self.assertRaisesRegex(ValueError, "identity"):
            ref.check(app)
        app.fixture_id_input.get.return_value = "fixture"
        ref.suspend()
        with self.assertRaises(ValueError): ref.check(app)

    def test_paired_raw_capture_without_zero_never_enables_motion(self):
        app = self.app()
        guard = RawMotorCaptureGuard(app)
        self.assertIs(guard.check(), self.feedback)
        with self.assertRaises(MotionConflictError):
            app.motion_coordinator.acquire("rotary-calibration-step")
        with self.assertRaises(MotionConflictError):
            app.motion_coordinator.submit_command(guard.token, Mock())
        self.feedback.system_uptime = 0
        with self.assertRaisesRegex(ValueError, "reset"):
            guard.check()
        guard.close()
        self.assertEqual(app.reference_manager.mock_calls, [])

    def test_paired_capture_rejects_missing_feedback_fault_and_connection_change(self):
        app = self.app()
        guard = RawMotorCaptureGuard(app)
        app.get_feedback.side_effect = ValueError("stale")
        with self.assertRaisesRegex(ValueError, "Fresh"):
            guard.check()
        app.get_feedback.side_effect = lambda: self.feedback
        self.feedback.active_errors = 2
        with self.assertRaises(ValueError): guard.check()
        self.feedback.active_errors = 0
        app.odrive_adapter = object()
        with self.assertRaisesRegex(ValueError, "connection changed"):
            guard.check()
        guard.close()

    def test_motor_fits_do_not_mix_origins_after_reset(self):
        data = []
        for frame, offset in (("before-reset", 0), ("after-reset", 100)):
            for angle in (-10, 0, 10):
                p = point(angle)
                p.update(motor_coordinate_frame=frame,
                         motor_position_mean_turns=p["motor_position_mean_turns"] + offset)
                data.append(p)
        model = fit_calibration(data)
        self.assertNotIn("motor_conversion_fit", model)
        self.assertAlmostEqual(model["slope"], 360)
        for report in model["motor_conversion_fits_by_frame"]:
            self.assertAlmostEqual(report["fit"]["slope"], 2.263)

    def test_calibration_worker_uses_same_motion_gate_timeouts_and_cleanup(self):
        app = self.app()
        ref = CalibrationPositioningReference.establish(app, 0, acknowledged=True)
        token = app.motion_coordinator.acquire("rotary-calibration-step")
        app.configure_trajectory, app.enter_closed_loop, app._submit_position = Mock(), Mock(), Mock()
        app.update_terminal, app.safe_idle_motor, app.ui_message_queue = Mock(), Mock(), Mock()
        app.expected_disarm_reason, app._mark_control_health, app._motion_cancelled = 0, Mock(), Mock(return_value=False)
        wait = Mock()
        gui_method("_manual_step_worker", math=math, wait_for_settle=wait,
                   AXIS_STATE_CLOSED_LOOP_CONTROL=8)(app, token, 1, 1, 1, "Motor turns", ref)
        app.enter_closed_loop.assert_called_once_with(token, allow_unreferenced=True)
        app._submit_position.assert_called_once_with(token, 5)
        app.configure_trajectory.assert_called_once_with(1, 5)
        self.assertLess(wait.call_args.args[5], app.system_config["motion"]["maximum_motion_timeout_s"])
        app.safe_idle_motor.assert_called_once_with("manual step complete", token=token)
        self.assertIsNone(app.motion_coordinator.owner)
        app.reference_manager.require_verified.assert_not_called()

    def test_stop_fault_and_full_step_rejection_never_leave_motion_owned(self):
        for failure in ("limit", "stale", "timeout", "closed"):
            app = self.app()
            ref = CalibrationPositioningReference.establish(app, 14 if failure == "limit" else 0, acknowledged=True)
            token = app.motion_coordinator.acquire("rotary-calibration-step")
            app.configure_trajectory, app.enter_closed_loop, app._submit_position = Mock(), Mock(), Mock()
            app.update_terminal, app.safe_idle_motor, app.ui_message_queue = Mock(), Mock(), Mock()
            app.expected_disarm_reason, app._mark_control_health, app._motion_cancelled = 0, Mock(), Mock()
            if failure == "stale": app.get_feedback.side_effect = ValueError("stale")
            if failure == "closed": ref.suspend()
            wait = Mock(side_effect=TimeoutError("stalled")) if failure == "timeout" else Mock()
            gui_method("_manual_step_worker", math=math, wait_for_settle=wait,
                       AXIS_STATE_CLOSED_LOOP_CONTROL=8)(app, token, 1, 1, 1, "Motor turns", ref)
            with self.subTest(failure=failure):
                if failure != "timeout": app._submit_position.assert_not_called()
                app.safe_idle_motor.assert_called_once()
                self.assertIsNone(app.motion_coordinator.owner)

    def test_suspend_and_reconnect_preserve_points_and_last_nonzero_pose(self):
        app = self.app()
        ref = CalibrationPositioningReference.establish(app, -10, acknowledged=True)
        path = Path(__file__).resolve().parents[1] / "east_rotary_gui.py"
        tree = ast.parse(path.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == "RotaryCalibrationWindow")
        namespace = {}
        exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), "exec"), namespace)
        window = namespace["RotaryCalibrationWindow"].__new__(namespace["RotaryCalibrationWindow"])
        window.app, window.preview, window.positioning_reference = app, False, ref
        window.positioning_angle, window.motion_acknowledged, window.motion_status = Mock(), Mock(), Mock()
        window._controls = Mock()
        window.points, window.session_dir, window.sensor = [point(-10)], Path("existing-session"), {"hub_serial": 750256}
        points = window.points
        window.suspend_positioning("reconnect")
        self.assertFalse(ref.active)
        window.positioning_angle.insert.assert_called_once_with(0, "-10")
        self.assertIs(window.points, points)
        self.assertEqual(window.session_dir, Path("existing-session"))
        app.connect_system = Mock()
        window.reconnect_motor()
        app.connect_system.assert_called_once()
        self.assertIs(window.points, points)
        self.assertEqual(window.sensor["hub_serial"], 750256)
        self.assertEqual(app.reference_manager.mock_calls, [])


if __name__ == "__main__":
    unittest.main()
