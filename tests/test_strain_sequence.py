"""Execute the GUI worker methods offline without importing GUI/hardware drivers."""
import ast
from pathlib import Path
from threading import Event
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import east_core
from east_reference import MotionConflictError, MotionState


class TestStopped(Exception):
    pass


def gui_method(name, **overrides):
    source = Path(__file__).resolve().parents[1] / "Ortho-Sim.py"
    tree = ast.parse(source.read_text())
    method = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = dict(vars(east_core), TestStopped=TestStopped,
                     MotionConflictError=MotionConflictError, MotionState=MotionState)
    namespace.update(overrides)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)
    return namespace[name]


class StrainSequenceTests(unittest.TestCase):
    def setUp(self):
        self.config = east_core.load_tester_config()

    def test_timeout_distances_and_cap(self):
        self.assertAlmostEqual(east_core.motion_timeout_seconds(10, 1, 100, self.config), 15.01)
        self.assertAlmostEqual(east_core.motion_timeout_seconds(20, 1, 100, self.config), 25.01)
        with self.assertRaisesRegex(ValueError, "exceeding"):
            east_core.motion_timeout_seconds(40, 1, 100, self.config)

    def test_feedback_distance_and_timeout_diagnostic(self):
        wait = Mock()
        app = Mock()
        app.motion_coordinator.assert_active = Mock()
        app.system_config = self.config
        app.test_stop_event = Event()
        app.run_parameters = SimpleNamespace(commanded_afo_speed_deg_s=1,
                                             commanded_afo_acceleration_deg_s2=100)
        app.get_feedback.return_value = SimpleNamespace(position_turns=-10 / 2.055)
        app.latest_feedback = app.get_feedback.return_value
        app._powered_wait_requirements.return_value = {}
        method = gui_method("command_position_and_wait", wait_for_settle=wait)
        method(app, 10 / 2.055, "moving_to_max", "token")
        self.assertAlmostEqual(app.current_nominal_distance_deg, 20)
        self.assertAlmostEqual(wait.call_args.kwargs["timeout_s"], 25.01)
        wait.side_effect = TimeoutError("stalled")
        with self.assertRaisesRegex(TimeoutError, "moving_to_max.*distance=20.*timeout=25.01"):
            method(app, 10 / 2.055, "moving_to_max", "token")

    def run_sequence(self, failure=None):
        app = Mock()
        app.system_config = self.config
        app.run_motion_token = "token"
        app.test_stop_event = Event()
        app.operator_stop_requested = False
        app.acquisition_error = None
        app.completed_cycles = 0
        app.run_parameters = SimpleNamespace(min_angle_deg=10, max_angle_deg=10, cycles=3,
                                            commanded_afo_speed_deg_s=1,
                                            commanded_afo_acceleration_deg_s2=100)
        app.reference_manager.require_verified.return_value = SimpleNamespace(neutral_position_turns=0)
        app.reference_manager.target_for_angle.side_effect = lambda angle: angle / 2.055
        app.data_collection_thread.is_alive.return_value = False
        moves = []
        def move(target, phase, token, **kwargs):
            moves.append((app.current_cycle, phase, target))
            if failure and len(moves) == 3:
                raise failure
        app.command_position_and_wait.side_effect = move
        gui_method("strain_test_control")(app)
        return app, moves

    def test_three_full_cycles_then_neutral_and_idle(self):
        app, moves = self.run_sequence()
        self.assertEqual([m[0] for m in moves], [0, 1, 1, 2, 2, 3, 3, 3])
        self.assertEqual([round(m[2] * 2.055) for m in moves], [10, -10, 10, -10, 10, -10, 10, 0])
        self.assertEqual(moves[0][1], "moving_to_initial_max")
        self.assertEqual(app.completed_cycles, 3)
        app.observe_post_idle_neutral.assert_called_once_with(0)
        app.safe_idle_motor.assert_called_once()
        self.assertEqual(app.finalize_run.call_args.args[0], "completed")

    def test_partial_cycle_is_not_completed(self):
        for failure in (TestStopped(), TimeoutError("stalled"), RuntimeError("feedback fault")):
            with self.subTest(failure=type(failure).__name__):
                app, moves = self.run_sequence(failure)
                self.assertEqual(app.completed_cycles, 0)
                self.assertEqual(len(moves), 3)
                app.safe_idle_motor.assert_called_once()
                app.observe_post_idle_neutral.assert_not_called()
                self.assertNotEqual(app.finalize_run.call_args.args[0], "completed")

    def test_stall_and_failure_to_settle_remain_bounded(self):
        from test_reference import FakeClock, snapshot
        from east_reference import wait_for_settle
        for position, velocity in ((0, 0), (1, 1)):
            clock = FakeClock()
            with self.assertRaises(TimeoutError):
                wait_for_settle(
                    lambda: snapshot(position=position, velocity=velocity, captured=clock.value),
                    1, 0.01, 0.02, 0.1, 0.5, 0.1, lambda: False,
                    clock.now, clock.sleep,
                )
            self.assertLess(clock.value, 0.52)

    def test_preflight_rejects_overlong_sweep(self):
        from test_east_core import valid_values
        values = valid_values()
        values.update(speed_deg_s="1", acceleration_deg_s2="100",
                      min_angle_deg="10", max_angle_deg="10")
        east_core.validate_test_parameters(values, self.config)
        self.config["motion"]["maximum_motion_timeout_s"] = 20
        with self.assertRaisesRegex(ValueError, "endpoint-to-endpoint sweep"):
            east_core.validate_test_parameters(values, self.config)
