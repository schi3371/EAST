"""Hardware-free commissioning policy and actual GUI execution checks."""
import json
from pathlib import Path
import unittest

from east_core import load_tester_config
from east_feedback_commissioning import CONDITIONS, PREFIX, policy
from east_protocol_batch import make_protocol_plan
from east_rotary_feedback import ramp, step
from east_wide_candidate import curve
import test_motion_model as harness
from test_three_speed_protocol import window_for


class CommissioningKernelTests(unittest.TestCase):
    def test_all_selected_stages_are_matched_pairs_inside_routine_limits(self):
        config = load_tester_config()
        for suffix, condition in CONDITIONS.items():
            key = PREFIX + suffix
            settings = dict(cycles=1, minimum_angle_deg=condition.half_range_deg,
                            maximum_angle_deg=condition.half_range_deg,
                            acceleration_deg_s2=condition.acceleration_deg_s2)
            identity = dict(operator='SC', fixture_id='0', calibration_id='0', afo_id='')
            with self.subTest(stage=key):
                plan = make_protocol_plan(config, key, identity, settings, 'setup', '', 'm.json', 'now')
                self.assertEqual([r.feedback_pilot for r in plan], [False, True])
                self.assertEqual(plan[0].parameters.commanded_afo_speed_deg_s, condition.speed_deg_s)
                limits = policy(config, key)
                self.assertLessEqual(limits['motor_speed_nominal_deg_s'], 20)
                self.assertLessEqual(limits['motor_acceleration_nominal_deg_s2'], 100)

    def test_wide_slow_sweep_has_a_realistic_reference_duration(self):
        _, duration = ramp(10, -10, .5, 0, acceleration=.5)
        self.assertEqual(duration, 41)
        self.assertAlmostEqual(ramp(10, -10, .5, duration, acceleration=.5)[0], -10)
        self.assertLess(duration + 6, load_tester_config()['motion']['maximum_motion_timeout_s'])

    def test_stage_limits_actually_bound_commands_and_reject_excessive_tracking_error(self):
        limits = policy(load_tester_config(), PREFIX + '05')
        arguments = {k: limits[k] for k in ('max_command_rate_turns_s', 'tracking_guard_deg', 'travel_guard_deg')}
        result = step(5, 5, 100, 0, .1, 2.3, **arguments)
        self.assertAlmostEqual(result['command_target_turns'], limits['max_command_rate_turns_s'] * .1)
        with self.assertRaises(RuntimeError):
            step(5, 0, 0, 0, .1, 2.3, **arguments)
        with self.assertRaises(ValueError):
            ramp(0, 5, 5, 0, acceleration=0)


class CommissioningIntegrationTests(unittest.TestCase):
    setUp = harness.ApplicationTests.setUp
    tearDown = harness.ApplicationTests.tearDown
    pump = harness.ApplicationTests.pump
    ready = harness.ApplicationTests.ready
    prepare = harness.ApplicationTests.prepare

    def setup_stage(self, suffix):
        app = self.prepare()
        app.rotary_sensor.config['stale_after_ms'] = 1000  # Isolate synthetic callback scheduling.
        condition = CONDITIONS[suffix]
        app.loaded_preset_key = PREFIX + suffix
        app.cycles_input.value = '1'
        app.min_angle_input.value = app.max_angle_input.value = str(condition.half_range_deg)
        app.acceleration_input.value = str(condition.acceleration_deg_s2)
        dispatch = app.__class__._submit_position.__get__(app)
        def follow(token, target):
            dispatch(token, target)
            if app.run_motion_model is not None:
                row = app.run_metadata['rotary_feedback_pilot']['samples'][-1]
                self.angle = curve(target - app.run_motion_model.origin_turns, row['direction'])
        app._submit_position = follow
        window, _ = window_for(app, self.module.ctk, app.loaded_preset_key)
        return app, window

    def test_complete_five_dps_ten_degree_pair(self):
        app, window = self.setup_stage('05')
        window.start()
        self.assertTrue(window.active, window.status.options)
        self.pump(lambda: (window.tick(), not window.active)[1], 65)
        self.assertEqual(window.batch_status, 'completed', window.batch_error)
        self.assertEqual(len(window.records), 2)
        for feedback, record in zip((False, True), window.records):
            metadata = json.loads(Path(record['metadata_file']).read_text())
            stream = metadata['rotary_feedback_pilot']
            self.assertEqual(stream['enabled'], feedback)
            self.assertEqual(stream['commissioning_policy']['speed_deg_s'], 5)
            self.assertEqual(stream['commissioning_policy']['ramp_acceleration_deg_s2'], 5)
            self.assertEqual(len(stream['legs']), 4)
            self.assertNotIn('initial_alignment', metadata['experimental_motion_model'])
            self.assertEqual(metadata['completed_cycles'], 1)
            self.assertTrue(metadata['zero_return_check']['passed'])

    def test_twenty_dps_stage_stop_uses_existing_cleanup(self):
        app, window = self.setup_stage('09')
        window.fields['start_run'].value = '2'
        window.start()
        self.assertTrue(window.active, window.status.options)
        self.pump(lambda: (window.tick(), len((app.run_metadata or {}).get(
            'rotary_feedback_pilot', {}).get('samples', [])) >= 5)[1], 12)
        window.stop()
        self.pump(lambda: (window.tick(), not window.active)[1], 12)
        self.assertEqual(window.batch_status, 'cancelled', window.batch_error)
        self.assertIsNone(app.motion_coordinator.owner)
