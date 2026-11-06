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

    def test_stage_limits_bound_commands_but_record_tracking_error(self):
        limits = policy(load_tester_config(), PREFIX + '05')
        arguments = {k: limits[k] for k in ('max_command_rate_turns_s', 'tracking_guard_deg', 'travel_guard_deg', 'tracking_error_action')}
        result = step(5, 5, 100, 0, .1, 2.3, **arguments)
        self.assertAlmostEqual(result['command_target_turns'], limits['max_command_rate_turns_s'] * .1)
        lagged = step(5, 0, 0, 0, .1, 2.3, **arguments)
        self.assertTrue(lagged["tracking_threshold_exceeded"])
        self.assertEqual(lagged["tracking_error_action"], "record")
        with self.assertRaisesRegex(RuntimeError, "Rotary travel guard"):
            step(5, 11.1, 0, 0, .1, 2.3, **arguments)
        with self.assertRaises(ValueError):
            ramp(0, 5, 5, 0, acceleration=0)


    def test_recorded_r9_lag_exceeds_threshold_without_bypassing_travel_or_rate(self):
        limits = policy(load_tester_config(), PREFIX + '05')
        arguments = {k: limits[k] for k in (
            'max_command_rate_turns_s', 'tracking_guard_deg', 'travel_guard_deg', 'tracking_error_action')}
        # Last saved R9 point; following updates had just crossed 1.5 degrees.
        for desired, measured in [(3.675, 2.1965529713), (4.22, 2.55), (-4.22, -2.55)]:
            for gain in (0., .3):
                row = step(desired, measured, 10., 0., .1, 2.3, gain=gain, **arguments)
                self.assertEqual(row['tracking_threshold_exceeded'], abs(desired-measured)>1.5)
                self.assertLessEqual(abs(row['command_target_turns']), limits['max_command_rate_turns_s']*.1)
                self.assertLessEqual(abs(row['requested_correction_turns']), .1/2.3)
        with self.assertRaisesRegex(RuntimeError, 'Rotary tracking guard'):
            step(4.22, 2.55, 10., 0., .1, 2.3, **{**arguments, 'tracking_error_action':'stop'})
        for measured in (11.01, -11.01):
            with self.assertRaisesRegex(RuntimeError, 'Rotary travel guard'):
                step(0., measured, 0., 0., .1, 2.3, **arguments)



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

    def test_fifteen_degree_pair_records_endpoint_error_past_old_cutoff(self):
        app, window = self.setup_stage('10')
        submit = app._submit_position
        def endpoint_offset(token, target):
            submit(token, target)
            if abs(self.angle) > 14.9:
                self.angle += .2 if self.angle > 0 else -.2
        app._submit_position = endpoint_offset
        window.start()
        self.assertTrue(window.active, window.status.options)
        self.pump(lambda: (window.tick(), not window.active)[1], 85)
        self.assertEqual(window.batch_status, 'completed', window.batch_error)
        self.assertEqual(len(window.records), 2)
        for record in window.records:
            metadata = json.loads(Path(record['metadata_file']).read_text())
            self.assertEqual(metadata['completed_cycles'], 1)
            samples = metadata['rotary_feedback_pilot']['samples']
            self.assertGreater(max(abs(row['measured_angle_deg']) for row in samples), 15.1)
            self.assertEqual(metadata['rotary_feedback_pilot']['travel_guard_deg'], 16.)
            self.assertTrue(metadata['csv_recorder']['worker_complete'])

    def test_wide_commissioning_uses_selected_guard_in_all_acquisition_paths(self):
        from unittest.mock import Mock
        app, window = self.setup_stage('10')
        app.run_motion_mapping_context = {'feedback_condition_key': PREFIX+'10'}
        app.run_rotary_targeted = True
        token = object()
        app.acquisition_error = None
        app.last_rotary_logged_key = None
        app.angle_filter.get_smoothed_value = Mock(return_value=None)
        app.motion_coordinator.assert_active = Mock()
        snapshot = app.get_feedback()
        from dataclasses import replace
        app.get_feedback = Mock(return_value=replace(snapshot,
            current_state=int(self.module.AXIS_STATE_CLOSED_LOOP_CONTROL),
            active_errors=0,disarm_reason=app.expected_disarm_reason))
        self.assertEqual(app._rotary_acquisition_travel_limit(), 16.)
        for angle in (15.11, 15.7, -15.7):
            self.angle = angle
            app._test_feedback()
            self.assertAlmostEqual(app.check_rotary_acquisition(token)[1]['angle_deg'], angle)
            app.acquisition_error = None
            app.log_strain_data(0., 0)
            self.assertIsNone(app.acquisition_error)
        for angle in (16.01, -16.01):
            self.angle = angle
            with self.assertRaisesRegex(Exception, 'limit \+/-16'):
                app._test_feedback()
            with self.assertRaisesRegex(RuntimeError, 'limit \+/-16'):
                app.check_rotary_acquisition(token)
        app.run_motion_mapping_context = None
        self.assertEqual(app._rotary_acquisition_travel_limit(), 15.1)
        self.angle = 15.11
        with self.assertRaisesRegex(Exception, 'limit \+/-15.1'):
            app._test_feedback()

    def test_r9_observed_lag_is_recorded_and_control_continues(self):
        # Reproduce the recorded R9 trajectory lag in the real GUI worker.
        app, window = self.setup_stage('05')
        # Match the lab's 250 ms-derived 1.5 degree comparison threshold.
        app.system_config['reference']['feedback_stale_after_ms'] = 250
        submit = app._submit_position
        def lagged(token, target):
            submit(token, target)
            if app.run_motion_model is not None:
                row = app.run_metadata['rotary_feedback_pilot']['samples'][-1]
                if row['phase'] == 'moving_to_initial_max' and row['desired_angle_deg'] > 2.:
                    self.angle = row['desired_angle_deg'] - 2.
        app._submit_position = lagged
        window.start()
        def recorded_lag():
            window.tick()
            samples = (app.run_metadata or {}).get('rotary_feedback_pilot', {}).get('samples', [])
            return sum(row['tracking_threshold_exceeded'] for row in samples) >= 3
        self.assertTrue(window.active, window.status.options)
        self.pump(recorded_lag, 12)
        self.assertTrue(app.strain_test_active)
        self.assertIsNone(app.acquisition_error)
        window.stop()
        self.pump(lambda: (window.tick(), not window.active)[1], 12)
        self.assertEqual(window.batch_status, 'cancelled', window.batch_error)
        metadata = json.loads(Path(window.records[0]['metadata_file']).read_text())
        samples = metadata['rotary_feedback_pilot']['samples']
        self.assertGreaterEqual(sum(row['tracking_threshold_exceeded'] for row in samples), 3)
        self.assertTrue(metadata['csv_recorder']['worker_complete'])
        self.assertIsNone(app.motion_coordinator.owner)

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
