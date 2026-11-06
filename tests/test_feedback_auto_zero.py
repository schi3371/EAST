"""A saved feedback zero miss homes and advances across a stage boundary."""
from dataclasses import replace
from pathlib import Path
import json
import unittest
from unittest.mock import patch

from east_feedback_commissioning import SEQUENCE_PRESET
from east_wide_candidate import curve
import test_motion_model as harness
from test_three_speed_protocol import window_for


class RecoveryTests(unittest.TestCase):
    setUp = harness.ApplicationTests.setUp
    tearDown = harness.ApplicationTests.tearDown
    pump = harness.ApplicationTests.pump
    ready = harness.ApplicationTests.ready
    prepare = harness.ApplicationTests.prepare

    def test_feedback_return_miss_homes_then_starts_next_stage(self):
        app = self.prepare()
        app.loaded_preset_key = SEQUENCE_PRESET
        app.cycles_input.value = '1'
        app.min_angle_input.value = app.max_angle_input.value = '5'
        app.acceleration_input.value = '1'
        app.rotary_sensor.config['stale_after_ms'] = 1000
        app.start_rotary_homing = app.__class__.start_rotary_homing.__get__(app)
        reader = app.rotary_sensor.reader
        raw_sample = reader._sample
        reader._sample = lambda channel, value: raw_sample(
            None, (self.angle-app.rotary_sensor.calibration.intercept)/app.rotary_sensor.calibration.slope)
        dispatch = app.__class__._submit_position.__get__(app)
        launched = []
        start = app.start_strain_test
        def track_start(*args, **kwargs):
            accepted = start(*args, **kwargs)
            if accepted:
                launched.append(kwargs['mapping_request'].index)
            return accepted
        app.start_strain_test = track_start
        def follow(token, target):
            old_motor = app.get_feedback().position_turns
            old_angle = self.angle
            dispatch(token, target)
            if token.owner == 'rotary-home':
                self.angle = old_angle + (target-old_motor)*2.055
            elif app.run_motion_model is not None:
                row = app.run_metadata['rotary_feedback_pilot']['samples'][-1]
                self.angle = curve(target-app.run_motion_model.origin_turns, row['direction'])
                if (app.motion_mapping.request.index == 2
                        and row['phase'] == 'returning_to_machine_zero'
                        and abs(row['desired_angle_deg']) < .05):
                    self.angle = -.2593
        app._submit_position = follow
        window, module = window_for(app, self.module.ctk, SEQUENCE_PRESET)
        builder = module.make_protocol_plan
        def short_plan(*args, **kwargs):
            return [replace(request,index=i,total=4)
                    for i,request in enumerate(builder(*args,**kwargs)[8:12],1)]
        window.fields['end_run'].value = '2'
        with patch.object(module, 'make_protocol_plan', side_effect=short_plan):
            window.start()
            self.assertTrue(window.active, window.status.options)
            self.pump(lambda: (window.tick(), launched == [1, 2, 3] or not window.active)[1], 85)
            self.assertEqual(launched, [1, 2, 3], window.batch_error or str(window.status.options))
            self.assertIsNone(window.awaiting_stage_review)
            recovered = window.records[1]
            self.assertTrue(recovered['acquisition_complete'])
            self.assertFalse(recovered['model_zero_return_passed'])
            original = json.loads(Path(recovered['metadata_file']).read_text())
            self.assertEqual(original['run_status'],'error')
            self.assertAlmostEqual(original['zero_return_check']['unassisted_angle_deg'],-.2593)
            home = json.loads(Path(recovered['zero_recovery']['evidence_file']).read_text())
            self.assertEqual(home['outcome'],'sensor_zero_reached')
            self.assertTrue(home['zero_reference_written'])
            window.stop()
            self.pump(lambda: (window.tick(), not window.active)[1], 15)
        self.assertEqual(launched,[1,2,3])
        self.assertIsNone(app.motion_coordinator.owner)
