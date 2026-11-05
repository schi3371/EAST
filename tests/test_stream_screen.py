"""Prospective model screen plan, inverses and real fake-device startup."""
import math
import unittest

from east_core import load_tester_config
from east_protocol_batch import make_protocol_plan
from east_stream_screen import BLOCKS, MODELS, MOTOR_BRACKET, SCREEN_PRESET, StreamScreenModel
import test_motion_model as harness
from test_three_speed_protocol import window_for


class ScreenPlanTests(unittest.TestCase):
    def test_all_seven_methods_per_block_and_identical_trajectory_settings(self):
        settings = dict(cycles=1, minimum_angle_deg=5, maximum_angle_deg=5,
                        acceleration_deg_s2=5)
        identity = dict(operator='test', fixture_id='0', calibration_id='cal', afo_id='')
        plan = make_protocol_plan(load_tester_config(), SCREEN_PRESET, identity, settings,
                                  'setup', '', 'manifest.json', 'now')
        self.assertEqual(len(plan), 21)
        for index, order in enumerate(BLOCKS):
            block = plan[index * 7:(index + 1) * 7]
            self.assertEqual(set(order), set(MODELS) | {'feedback'})
            self.assertEqual([r.preset_metadata['screen_method'] for r in block], list(order))
            self.assertEqual(sum(r.feedback_pilot for r in block), 1)
            self.assertTrue(all(r.parameters.commanded_afo_speed_deg_s == .5
                                and r.parameters.cycles == 1 for r in block))

    def test_each_frozen_branch_is_monotonic_and_covers_the_screen(self):
        for variant in MODELS:
            model = StreamScreenModel.anchored(variant, 10., .03)
            self.assertAlmostEqual(model.target(.03, 1), 10.)
            for direction in (-1, 1):
                with self.subTest(variant=variant, direction=direction):
                    positions = [model.origin_turns + MOTOR_BRACKET * i / 100 for i in range(-99, 100)]
                    self.assertTrue(all(model.slope(p, direction) > 0 for p in positions))
                    self.assertLess(model.target(-5, direction), model.target(5, direction))
                    self.assertTrue(math.isfinite(model.target(0, direction)))


class ScreenIntegrationTests(unittest.TestCase):
    setUp = harness.ApplicationTests.setUp
    tearDown = harness.ApplicationTests.tearDown
    pump = harness.ApplicationTests.pump
    ready = harness.ApplicationTests.ready
    prepare = harness.ApplicationTests.prepare

    def test_quintic_starts_through_screen_gui_and_records_fixed_anchor(self):
        app = self.prepare()
        # Use the real dispatcher. The harness wrapper expects endpoint-only segment metadata.
        app._submit_position = app.__class__._submit_position.__get__(app)
        app.loaded_preset_key = SCREEN_PRESET
        app.cycles_input.value = '1'
        app.min_angle_input.value = app.max_angle_input.value = '5'
        app.acceleration_input.value = '5'
        window, _ = window_for(app, self.module.ctk, SCREEN_PRESET)
        self.assertEqual(window.fields['end_run'].get(), '7')
        window.fields['start_run'].value = window.fields['end_run'].value = '6'
        window.start()
        self.assertTrue(window.active, window.status.options)
        self.pump(lambda: (window.tick(), len(
            (app.run_metadata or {}).get('rotary_feedback_pilot', {}).get('samples', [])) >= 5)[1], 12)
        meta = app.run_metadata['experimental_motion_model']
        self.assertEqual(meta['model_variant'], 'quintic')
        self.assertIn('origin fixed', meta['zero_alignment'])
        self.assertFalse(app.run_metadata['rotary_feedback_pilot']['enabled'])
        window.stop()
        self.pump(lambda: (window.tick(), not window.active)[1], 10)
        self.assertEqual(window.batch_status, 'cancelled')
