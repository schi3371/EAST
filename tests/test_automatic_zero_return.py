"""Routine loaded returns must close the outer zero loop, including post-idle drift."""
import json
from pathlib import Path
import unittest
from unittest.mock import Mock
import test_motion_model as harness


class AutomaticReturnTests(unittest.TestCase):
    setUp = harness.ApplicationTests.setUp
    tearDown = harness.ApplicationTests.tearDown
    pump = harness.ApplicationTests.pump
    ready = harness.ApplicationTests.ready
    prepare = harness.ApplicationTests.prepare

    def execute_return(self, *, second_idle_drift=False, stop_on_correction=False, stuck=False):
        app = self.prepare()
        app.rotary_sensor.config['stale_after_ms'] = 1000
        app.run_metadata = {'rotary_sensor': {}}
        app.acquisition_error = None
        app.operator_stop_requested = False
        app.test_stop_event.clear()
        app.strain_test_active = True
        app.run_motion_mapping_context = None
        app.run_rotary_targeted = False
        app.run_parameters = self.parameters = __import__('types').SimpleNamespace(
            commanded_afo_speed_deg_s=1., commanded_afo_acceleration_deg_s2=5.)
        app.last_rotary_logged_key = None
        token = app.motion_coordinator.acquire('test-zero-return')
        self.token = token
        self.angle = 2.26
        self.zero_reference = app.reference_manager.metadata_snapshot()
        self.tare = app.empty_machine_tare
        reader = app.rotary_sensor.reader
        original_sample = reader._sample
        reader._sample = lambda ch, v: original_sample(None,
            (self.angle-app.rotary_sensor.calibration.intercept)/app.rotary_sensor.calibration.slope)
        original_submit = app.__class__._submit_position.__get__(app)
        self.correction_commands = []
        def submit(token, target):
            before = app.get_feedback().position_turns
            original_submit(token, target)
            self.correction_commands.append(target)
            if stop_on_correction:
                app.operator_stop_requested = True
                app.test_stop_event.set()
            if not stuck:
                self.angle += (target-before)*2.055
        app._submit_position = submit
        original_idle = app.safe_idle_motor
        self.idle_count = 0
        def idle(*args, **kwargs):
            result = original_idle(*args, **kwargs)
            self.idle_count += 1
            if second_idle_drift and self.idle_count == 1:
                self.angle = -.18
            return result
        app.safe_idle_motor = idle
        return app

    def run_in_worker(self, app):
        import threading
        self.failure = None
        def worker():
            try:
                app.enter_closed_loop(self.token)
                app.complete_rotary_zero_return(self.token)
            except Exception as exc:
                self.failure = exc
        thread = threading.Thread(target=worker)
        thread.start()
        self.pump(lambda: not thread.is_alive(), 35)
        thread.join()
        return self.failure

    def cleanup(self, app):
        import threading
        thread = threading.Thread(target=lambda: app.safe_idle_motor('test cleanup', token=self.token))
        thread.start()
        thread.join(3)
        app.motion_coordinator.release(self.token)
        app.strain_test_active = False

    def test_loaded_return_corrects_and_retries_if_idle_drift_recurs(self):
        app = self.execute_return(second_idle_drift=True)
        try:
            error = self.run_in_worker(app)
            self.assertIsNone(error, str(error))
            record = app.run_metadata['automatic_zero_return']
            self.assertEqual(record['status'], 'completed')
            self.assertGreaterEqual(len(record['attempts']), 2)
            self.assertGreater(len(self.correction_commands), 1)
            self.assertEqual(record['method'], 'direct_rotary_feedback')
            self.assertGreater(record['corrections'][0]['before_deg'], 2.)
            self.assertAlmostEqual(record['corrections'][0]['nominal_step_deg'], -.5)
            self.assertTrue(app.run_metadata['zero_return_check']['passed'])
            self.assertNotIn('unassisted_angle_deg', app.run_metadata['zero_return_check'])
            self.assertTrue(record['final_observation']['passed'])
            self.assertLessEqual(abs(self.angle), .1)
            self.assertTrue(record['idle_confirmed'])
            self.assertEqual(app.reference_manager.metadata_snapshot(), self.zero_reference)
            self.assertIs(app.empty_machine_tare, self.tare)
        finally:
            self.cleanup(app)

    def test_stop_during_correction_prevents_completion_and_further_commands(self):
        app = self.execute_return(stop_on_correction=True)
        try:
            self.assertIsNotNone(self.run_in_worker(app))
            self.assertEqual(app.run_metadata['automatic_zero_return']['status'], 'failed')
            self.assertEqual(len(self.correction_commands), 1)
        finally:
            self.cleanup(app)

    def test_nonresponsive_fixture_does_not_report_success(self):
        app = self.execute_return(stuck=True)
        try:
            error = self.run_in_worker(app)
            self.assertIsNotNone(error)
            self.assertIn('progress', str(error))
            self.assertEqual(app.run_metadata['automatic_zero_return']['status'], 'failed')
        finally:
            self.cleanup(app)
