import math
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import east_core
from test_strain_sequence import gui_method


class ManualStepTests(unittest.TestCase):
    def setUp(self):
        self.config = east_core.load_tester_config()

    def worker_app(self, current=7.25, settled=8.24):
        app = Mock()
        app.system_config = self.config
        app.get_feedback.side_effect = [
            SimpleNamespace(position_turns=current),
            SimpleNamespace(position_turns=current),
            SimpleNamespace(position_turns=settled),
        ]
        app.clamp_manual_target.side_effect = lambda target: target
        return app

    def run_worker(self, app, step=1, requested=1, units="Motor turns", direction=1, wait=None):
        wait = wait or Mock()
        gui_method("_manual_step_worker", math=math, wait_for_settle=wait,
                   AXIS_STATE_CLOSED_LOOP_CONTROL=8)(app, "token", direction, step, requested, units)
        return wait

    def test_turn_command_is_independent_of_conversion(self):
        for factor in (2.055, 2.263):
            self.config["motion"]["afo_degrees_per_odrive_turn"] = factor
            step = east_core.manual_step_to_turns("1", "Motor turns", self.config)
            self.assertEqual(step, 1)
            app = self.worker_app()
            wait = self.run_worker(app, step)
            app._submit_position.assert_called_once_with("token", 8.25)
            expected_timeout = east_core.motion_timeout_seconds(factor, 5, 10, self.config)
            self.assertAlmostEqual(wait.call_args.args[5], expected_timeout)
            self.assertIn("+0.99000000 motor turns", app.update_terminal.call_args.args[0])
            app.motion_coordinator.release.assert_called_once_with("token")
            app.safe_idle_motor.assert_called_once_with("manual step complete", token="token")

    def test_degree_steps_keep_existing_conversion(self):
        step = east_core.manual_step_to_turns(1, "Degrees", self.config)
        self.assertAlmostEqual(step, 1 / 2.055)
        app = self.worker_app(settled=7.25 + step)
        self.run_worker(app, step, units="Degrees")
        app._submit_position.assert_called_once_with("token", 7.25 + step)

    def test_negative_direction_subtracts_raw_turns(self):
        app = self.worker_app(settled=6.25)
        self.run_worker(app, direction=-1)
        app._submit_position.assert_called_once_with("token", 6.25)
        self.assertIn("-1 motor turns", app.update_terminal.call_args.args[0])

    def test_out_of_bounds_step_does_not_command_partial_motion(self):
        app = self.worker_app()
        app.clamp_manual_target.side_effect = lambda target: target - 0.1
        wait = self.run_worker(app)
        app._submit_position.assert_not_called()
        app.enter_closed_loop.assert_not_called()
        wait.assert_not_called()
        self.assertIn("No movement commanded", app.update_terminal.call_args.args[0])
        app.safe_idle_motor.assert_called_once()
        app.motion_coordinator.release.assert_called_once()

    def test_timeout_keeps_cleanup_and_does_not_report_success(self):
        app = self.worker_app()
        wait = Mock(side_effect=TimeoutError("stalled"))
        self.run_worker(app, wait=wait)
        self.assertIn("Manual step failed: stalled", app.update_terminal.call_args.args[0])
        app.safe_idle_motor.assert_called_once()
        app.motion_coordinator.release.assert_called_once()

    def test_invalid_input_and_unknown_units_are_rejected(self):
        for units in ("Degrees", "Motor turns"):
            for value in (0, -1, "", "abc", math.nan, math.inf, 100):
                with self.subTest(units=units, value=value), self.assertRaises(ValueError):
                    east_core.manual_step_to_turns(value, units, self.config)
        with self.assertRaises(ValueError):
            east_core.manual_step_to_turns(1, "Radians", self.config)

    def test_original_step_bounds_remain_equivalent_in_both_units(self):
        for degrees in (0.01, 10):
            turns = degrees / 2.055
            self.assertEqual(east_core.manual_step_to_turns(turns, "Motor turns", self.config), turns)
        for degrees in (0.009, 10.001):
            with self.assertRaises(ValueError):
                east_core.manual_step_to_turns(degrees / 2.055, "Motor turns", self.config)

    def test_live_readout_reports_raw_and_zero_relative_turns(self):
        app = Mock()
        app.system_config = self.config
        app.reference_manager.verified = True
        app.reference_manager.require_verified.return_value = SimpleNamespace(neutral_position_turns=3)
        app.get_feedback.return_value = SimpleNamespace(position_turns=4)
        gui_method("update_manual_position_display", TEXT="text", AMBER="amber")(app)
        text = app.manual_position_label.configure.call_args.kwargs["text"]
        self.assertIn("4.00000000 turns", text)
        self.assertIn("+1.00000000 turns", text)
        self.assertIn("motor-derived +2.055 deg", text)

    def test_unverified_zero_does_not_invent_displacement(self):
        app = Mock()
        app.reference_manager.verified = False
        app.get_feedback.return_value = SimpleNamespace(position_turns=4)
        gui_method("update_manual_position_display", TEXT="text", AMBER="amber")(app)
        self.assertIn("unavailable until physical 90 deg is verified", app.manual_position_label.configure.call_args.kwargs["text"])
        app.reference_manager.require_verified.assert_not_called()

    def test_stale_feedback_is_not_displayed_as_live(self):
        app = Mock()
        app.get_feedback.side_effect = RuntimeError("stale feedback")
        gui_method("update_manual_position_display", TEXT="text", AMBER="amber")(app)
        self.assertEqual(app.manual_position_label.configure.call_args.kwargs["text"], "Motor feedback unavailable: stale feedback")


if __name__ == "__main__":
    unittest.main()
