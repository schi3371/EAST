"""Combined feedback sequence keeps per-condition control and stage review boundaries."""
from pathlib import Path
import json
import unittest
from unittest.mock import Mock

from east_core import load_tester_config
from east_feedback_commissioning import CONDITIONS, PREFIX, SEQUENCE_PRESET, policy
from east_protocol_batch import make_protocol_plan
from east_wide_candidate import curve
import test_motion_model as harness
from test_three_speed_protocol import window_for


def sequence(config):
    return make_protocol_plan(
        config, SEQUENCE_PRESET,
        dict(operator='SC', fixture_id='0', calibration_id='0', afo_id=''),
        dict(cycles=1, minimum_angle_deg=5, maximum_angle_deg=5, acceleration_deg_s2=1),
        'setup', '', 'manifest.json', 'now', batch_id='test-batch')


class PlanTests(unittest.TestCase):
    def test_twenty_two_unique_runs_preserve_each_condition_and_pair(self):
        config = load_tester_config()
        plan = sequence(config)
        self.assertEqual(len(plan), 22)
        self.assertEqual(len({r.parameters.file_prefix for r in plan}), 22)
        for i, suffix in enumerate(CONDITIONS):
            pair = plan[2*i:2*i+2]
            self.assertEqual([r.feedback_pilot for r in pair], [False, True])
            self.assertEqual([r.index for r in pair], [2*i+1, 2*i+2])
            for request in pair:
                context = request.context()
                self.assertEqual(context['protocol_key'], SEQUENCE_PRESET)
                self.assertEqual(context['feedback_condition_key'], PREFIX + suffix)
                self.assertEqual(request.preset_metadata['preset_key'], SEQUENCE_PRESET)
                settings = policy(config, context['feedback_condition_key'])
                self.assertEqual(settings['speed_deg_s'], request.parameters.commanded_afo_speed_deg_s)
                self.assertEqual(settings['half_range_deg'], request.parameters.max_angle_deg)


class WindowTests(unittest.TestCase):
    setUp = harness.ApplicationTests.setUp
    tearDown = harness.ApplicationTests.tearDown
    pump = harness.ApplicationTests.pump
    ready = harness.ApplicationTests.ready
    prepare = harness.ApplicationTests.prepare

    def setup_sequence(self):
        app = self.prepare()
        app.loaded_preset_key = SEQUENCE_PRESET
        app.cycles_input.value = '1'
        app.min_angle_input.value = app.max_angle_input.value = '5'
        app.acceleration_input.value = '1'
        return app, window_for(app, self.module.ctk, SEQUENCE_PRESET)[0]

    def test_pause_occurs_between_pairs_and_continue_is_recorded(self):
        app, window = self.setup_sequence()
        plan = sequence(app.system_config)
        window.requests = plan
        window.records = [{}, {}]
        window.pause_stages_snapshot = True
        window.active = True
        window.save = Mock()
        app.start_strain_test = Mock()
        window.next_run()
        self.assertEqual(window.awaiting_stage_review, 3)
        app.start_strain_test.assert_not_called()
        window.continue_stage()
        self.assertEqual(window.stage_approved_run, 3)
        self.assertIsNone(window.awaiting_stage_review)
        self.assertEqual(window.stage_reviews[0]['next_run'], 3)
        window.save.assert_called_once_with('next')
        window.active = False

    def test_stage_five_selection_dispatches_condition_five_and_stop_saves_range(self):
        app, window = self.setup_sequence()
        app.rotary_sensor.config['stale_after_ms'] = 1000
        dispatch = app.__class__._submit_position.__get__(app)
        def follow(token, target):
            dispatch(token, target)
            if app.run_motion_model is not None:
                row = app.run_metadata['rotary_feedback_pilot']['samples'][-1]
                self.angle = curve(target-app.run_motion_model.origin_turns, row['direction'])
        app._submit_position = follow
        window.fields['start_run'].value = window.fields['end_run'].value = '5'
        window.start()
        self.assertTrue(window.active, window.status.options)
        self.assertEqual([r.index for r in window.requests], [9, 10])
        self.pump(lambda: (window.tick(), len((app.run_metadata or {}).get(
            'rotary_feedback_pilot', {}).get('samples', [])) >= 5)[1], 12)
        self.assertEqual(app.run_metadata['rotary_feedback_pilot']['commissioning_policy']['speed_deg_s'], 5)
        self.assertEqual(app.run_metadata['protocol_batch']['feedback_condition_key'], PREFIX+'05')
        window.stop()
        self.pump(lambda: (window.tick(), not window.active)[1], 12)
        self.assertEqual(window.batch_status, 'cancelled', window.batch_error)
        meta = json.loads(Path(window.manifest_path).read_text())
        self.assertEqual(meta['selected_start_run'], 9)
        self.assertEqual(meta['selected_end_run'], 10)
        self.assertTrue(meta['pause_between_stages'])
        self.assertEqual(meta['runs'][10]['status'], 'outside_selected_run_range')

    def test_stop_while_paused_never_launches_successor(self):
        app, window = self.setup_sequence()
        window.requests = sequence(app.system_config)
        window.records = [{}, {}]
        window.pause_stages_snapshot = True
        window.active = True
        window.save = Mock()
        app.start_strain_test = Mock()
        window.next_run()
        window.cancelled.set()
        window.continue_stage()
        self.assertIsNone(window.stage_approved_run)
        window.save.assert_not_called()
        app.start_strain_test.assert_not_called()
        window.active = False
