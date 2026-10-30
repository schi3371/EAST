"""Offline motion-window tests: real GUI execution/services with fake devices and view."""
import copy
import csv
import importlib.util
import json
from pathlib import Path
import sys
import threading
import time
import unittest
import unicodedata  # Keep named Unicode decoding loaded outside patched module dictionaries.
from unittest.mock import patch

import test_engine_gui_integration as harness
from east_core import load_tester_config, CSV_COLUMNS, ROTARY_RUN_COLUMNS, MOTOR_TIMING_COLUMNS
from east_motion_mapping import make_plan, completed_run, ready_for_next


class Text(harness.Field):
    def index(self, *args):
        return '1.0'


def window_for(app, ctk):
    path = Path(__file__).resolve().parents[1] / 'east_motion_mapping_gui.py'
    spec = importlib.util.spec_from_file_location('offline_mapping_window', path)
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {'customtkinter': ctk}), \
         patch.object(ctk, 'CTkEntry', harness.Field, create=True), \
         patch.object(ctk, 'CTkTextbox', Text, create=True):
        spec.loader.exec_module(module)
        window = module.MotionMappingWindow(app.master, app)
    app.motion_mapping = window
    for name, value in {'range':'5', 'cycles':'1', 'slow':'0.5', 'fast':'5', 'acceleration':'100'}.items():
        window.fields[name].set(value)
    return window


class PlanTests(unittest.TestCase):
    def test_plan_freezes_identity_and_rejects_invalid_motion(self):
        config = load_tester_config()
        identity = dict(operator='TEST', fixture_id='0', calibration_id='TEST-CAL')
        settings = dict(range='10', cycles='3', slow='0.5', fast='5', acceleration='100')
        plan = make_plan(config, identity, settings, 'sequence', 'batch.json')
        self.assertEqual([r.run_id for r in plan], ['S1','M1','S2','M2','S3','M3'])
        self.assertEqual([r.parameters.commanded_afo_speed_deg_s for r in plan], [.5,5,.5,5,.5,5])
        self.assertEqual(len({r.parameters.file_prefix for r in plan}), 6)
        self.assertTrue(all(r.parameters.fixture_id=='0' and r.parameters.test_type=='empty_machine_baseline' for r in plan))
        self.assertTrue(all(r.preset_metadata['actual_commanded_values']['cycles']==3 for r in plan))
        identity['fixture_id'] = 'CHANGED'
        self.assertEqual(plan[0].parameters.fixture_id, '0')
        for bad in ({'range':'16'}, {'cycles':'0'}, {'fast':'nan'}, {'slow':'nan'},
                    {'acceleration':'101'}, {'fast':'21'}, {'slow':'5'}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                make_plan(config, identity, {**settings, **bad}, 'sequence', 'batch.json')


class MotionWindowIntegration(unittest.TestCase):
    setUp = harness.FullApplicationIntegration.setUp
    tearDown = harness.FullApplicationIntegration.tearDown
    pump = harness.FullApplicationIntegration.pump
    ready = harness.FullApplicationIntegration.ready

    def prepare_window(self):
        # Fake services run under an unconstrained CI/desktop scheduler. Test gates and
        # disk outcomes here; production 150/250 ms limits are unchanged in the config.
        self.app.system_config['reference'].update(
            maximum_feedback_capture_ms=1000, feedback_stale_after_ms=2000)
        self.ready()
        self.pump(lambda: ready_for_next(self.app))
        return window_for(self.app, self.module.ctk)

    def pump_window(self, window, condition, seconds=30):
        self.pump(lambda: (window.tick(), condition())[1], seconds)

    def test_six_runs_save_normal_data_and_batch_metadata(self):
        window = self.prepare_window()
        window.start('sequence')
        self.assertTrue(window.active)
        self.assertEqual(self.app.operator_input.cget('state'), 'disabled')
        self.assertFalse(self.app.start_strain_test())  # Parent Start cannot interleave.
        self.pump_window(window, lambda: not window.active)
        self.assertEqual(window.batch_status, 'completed', window.status.options)
        manifest = json.loads(window.manifest_path.read_text())
        self.assertEqual(manifest['outcome'], 'completed')
        self.assertEqual(len(manifest['runs']), 6)
        for record in manifest['runs']:
            meta = json.loads(Path(record['metadata_file']).read_text())
            self.assertEqual(meta['motion_modelling'], record['context'])
            self.assertEqual(meta['completed_cycles'], 1)
            self.assertEqual(meta['run_status'], 'completed')
            self.assertTrue(meta['calibration']['empty_machine_tare'])
            with open(record['strain_file'], newline='') as handle:
                rows = list(csv.reader(handle))
            self.assertEqual(rows[0], CSV_COLUMNS+ROTARY_RUN_COLUMNS+MOTOR_TIMING_COLUMNS)
            self.assertGreater(len(rows), 1)
            self.assertTrue(Path(record['rotary_file']).exists())
        window.tick()
        self.assertEqual(self.app.operator_input.cget('state'), 'normal')
        # The inactive tool must not re-enable entries during an ordinary run.
        self.app.start_strain_test()
        window.tick()
        self.assertEqual(self.app.operator_input.cget('state'), 'disabled')
        self.pump(lambda: ready_for_next(self.app), 12)
        self.assertNotIn('motion_modelling', self.app.run_metadata)

    def test_stop_between_runs_cancels_sequence_before_motion(self):
        window = self.prepare_window()
        window.start('sequence')
        window.stop()  # During the initial manifest save, before the first admission.
        self.pump_window(window, lambda: not window.active)
        self.assertEqual(window.batch_status, 'cancelled')
        self.assertEqual(window.records, [])
        self.assertIsNone(self.app.strain_thread)
        self.assertEqual(json.loads(window.manifest_path.read_text())['outcome'], 'cancelled')

    def test_fault_during_initialization_cancels_remaining_runs(self):
        window = self.prepare_window()
        self.app._ensure_load_cell = lambda: (_ for _ in ()).throw(OSError('SIMULATED bridge failure'))
        window.start('sequence')
        self.pump_window(window, lambda: not window.active)
        self.assertNotEqual(window.batch_status, 'completed')
        self.assertEqual(len(window.records), 1)
        self.assertFalse(self.app.reference_manager.verified)
        self.assertIsNone(self.app.strain_thread)

    def test_stop_during_motion_keeps_first_run_and_cancels_remainder(self):
        window = self.prepare_window()
        window.start('sequence')
        self.pump_window(window, lambda: self.app.strain_thread is not None)
        window.stop()
        self.pump_window(window, lambda: not window.active)
        self.assertNotEqual(window.batch_status, 'completed')
        self.assertEqual(len(window.records), 1)
        self.assertNotEqual(self.app.run_metadata['run_status'], 'completed')
        self.assertTrue(self.app.motion_coordinator.motor_idle_confirmed)
        self.assertIsNone(self.app.motion_coordinator.owner)
        self.assertTrue(Path(window.records[0]['metadata_file']).exists())

    def test_recorder_save_failure_prevents_admission(self):
        window = self.prepare_window()
        with patch.dict(window.save.__globals__, {'write_json_atomic': lambda *args: (_ for _ in ()).throw(OSError('SIMULATED disk full'))}):
            window.start('sequence')
            self.pump_window(window, lambda: not window.active)
        self.assertIsNone(self.app.strain_thread)
        self.assertIn('save failed', window.status.options['text'])

    def test_completion_gate_blocks_partial_data_and_live_workers(self):
        window = self.prepare_window()
        window.start('single')
        self.pump_window(window, lambda: not window.active)
        request = window.requests[0]
        self.assertEqual(completed_run(self.app, request)[0], 'success')
        saved = copy.deepcopy(self.app.run_metadata)
        mutations = [lambda m: m.update(completed_cycles=0),
            lambda m: m.update(outcome_pending_cleanup=True),
            lambda m: m['csv_recorder'].update(rows_persisted=0),
            lambda m: m['rotary_sensor']['recording_statistics'].update(invalid_samples=1),
            lambda m: m.update(motion_modelling={})]
        for mutate in mutations:
            self.app.run_metadata = copy.deepcopy(saved)
            mutate(self.app.run_metadata)
            self.assertEqual(completed_run(self.app, request)[0], 'failure')
        self.app.run_metadata = saved
        self.app.run_finalized = False
        self.assertEqual(completed_run(self.app, request)[0], 'failure')
        self.app.run_finalized = True
        self.app.motion_coordinator.begin_async_stop()
        self.assertEqual(completed_run(self.app, request)[0], 'failure')
        self.app.motion_coordinator.confirm_idle()
        self.app.motion_coordinator.complete_async_stop()
        release = threading.Event()
        self.app.cleanup_thread = threading.Thread(target=lambda: release.wait(2))
        self.app.cleanup_thread.start()
        try:
            self.assertEqual(completed_run(self.app, request)[0], 'pending')
        finally:
            release.set()
            self.app.cleanup_thread.join(2)

    def test_prior_completed_stop_does_not_cancel_new_run_with_stale_warning(self):
        window = self.prepare_window()
        self.app.stop_logging()
        self.pump(lambda: ready_for_next(self.app))
        self.app.stop_started_monotonic = time.monotonic()-10
        self.app.stop_diagnostic_recorded = False
        window.start('single')
        self.pump_window(window, lambda: not window.active)
        self.assertEqual(window.batch_status, 'completed', window.status.options)
        self.assertFalse(window.cancelled.is_set())

    def test_real_cleanup_warning_cancels_remaining_sequence(self):
        window = self.prepare_window()
        window.start('sequence')
        self.pump_window(window, lambda: self.app.strain_thread is not None)
        self.app.update_terminal('Stop/cleanup is still running; SIMULATED delayed cleanup.\n')
        self.assertTrue(window.cancelled.is_set())
        self.pump_window(window, lambda: not window.active)
        self.assertNotEqual(window.batch_status, 'completed')
        self.assertEqual(len(window.records), 1)

    def test_child_stop_remains_application_stop_when_batch_is_inactive(self):
        window = self.prepare_window()
        self.app.start_strain_test()
        self.pump(lambda: self.app.strain_thread is not None)
        self.assertFalse(window.active)
        window.stop()
        self.pump(lambda: ready_for_next(self.app), 12)
        self.assertNotEqual(self.app.run_metadata['run_status'], 'completed')

    def test_terminal_mirrors_on_ui_thread_and_close_requests_stop(self):
        window = self.prepare_window()
        self.app.update_terminal('SIMULATED terminal line\n')
        self.assertIn('SIMULATED terminal', window.terminal.value)
        window.start('sequence')
        window.close()
        self.assertFalse(window.closed)
        self.assertTrue(window.cancelled.is_set())
        self.pump_window(window, lambda: not window.active)
        window.close()
        self.assertTrue(window.closed)


if __name__ == '__main__':
    unittest.main()
