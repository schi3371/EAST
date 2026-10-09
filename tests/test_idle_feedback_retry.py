"""Exercise late-read handling without a GUI, USB devices or motor commands."""
import ast
import copy
from dataclasses import replace
from pathlib import Path
import queue
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch
import json

from east_core import load_tester_config
from east_odrive import ODriveAdapter
from east_reference import (FeedbackAcquisitionTimeout, FeedbackError, FeedbackSnapshot,
                            IdleFeedbackRetry, MotionCoordinator, MotionConflictError, MotionState)
from test_odrive_adapter import Device

ROOT = Path(__file__).resolve().parents[1]


def sample(at=10, duration=0.04, **kwargs):
    fields = dict(captured_monotonic_s=at, captured_at='test', position_turns=4.5,
                  velocity_turns_s=0, active_errors=0, current_state=1,
                  disarm_reason=0, system_uptime=100 + at,
                  capture_started_monotonic_s=at-duration, capture_duration_s=duration)
    fields.update(kwargs)
    return FeedbackSnapshot(**fields)


def gui_method(name):
    tree = ast.parse((ROOT/'Ortho-Sim.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MyInterface')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    ns = dict(time=time, IdleFeedbackRetry=IdleFeedbackRetry,
              FeedbackAcquisitionTimeout=FeedbackAcquisitionTimeout, FeedbackError=FeedbackError,
              MotionConflictError=MotionConflictError, MotionState=MotionState,
              AMBER='amber', GREEN='green', RED='red')
    exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])), str(ROOT/'Ortho-Sim.py'), 'exec'), ns)
    return ns[name]


class IdleFeedbackRetryTests(unittest.TestCase):
    def setUp(self):
        self.config = load_tester_config()
        self.previous = sample()
        self.late = sample(10.266, .266)
        self.retry = IdleFeedbackRetry(self.config)

    def test_late_sample_is_typed_diagnostic_not_valid_measurement(self):
        with self.assertRaises(FeedbackAcquisitionTimeout) as error:
            self.late.validate(max_capture_duration_s=.150)
        self.assertIs(error.exception.snapshot, self.late)
        self.assertIn('0.266 s', str(error.exception))

    def test_idle_delay_then_fresh_confirmation(self):
        self.assertTrue(self.retry.reject_and_retry(FeedbackAcquisitionTimeout(self.late), self.previous, None, None))
        self.retry.accept_fresh(self.previous, sample(10.30))
        self.assertEqual(self.retry.rejected, 0)

    def test_two_retries_then_stop(self):
        for at in (10.266, 10.55):
            self.assertTrue(self.retry.reject_and_retry(FeedbackAcquisitionTimeout(sample(at,.266)), self.previous, None, None))
        self.assertFalse(self.retry.reject_and_retry(FeedbackAcquisitionTimeout(sample(10.85,.266)), self.previous, None, None))

    def test_total_retry_window_is_bounded(self):
        self.assertFalse(self.retry.reject_and_retry(FeedbackAcquisitionTimeout(sample(11.01,.266)), self.previous, None, None))

    def test_powered_or_motion_owner_never_retries(self):
        for state, owner in ((8,None), (None,'manual-step'), (None,'strain-test')):
            with self.subTest(state=state, owner=owner):
                self.assertFalse(self.retry.reject_and_retry(FeedbackAcquisitionTimeout(self.late), self.previous,state,owner))
        for owner in ('rotary-static-capture','rotary-sensor-only-capture'):
            self.assertTrue(IdleFeedbackRetry(self.config).reject_and_retry(FeedbackAcquisitionTimeout(self.late),self.previous,None,owner))

    def test_missing_or_changed_evidence_never_retries(self):
        for changes in ({'active_errors':1}, {'current_state':8}, {'disarm_reason':1},
                        {'system_uptime':None}, {'system_uptime':1}, {'system_uptime':110},
                        {'position_turns':4.55}, {'velocity_turns_s':1},
                        {'position_turns':float('nan')}, {'captured_monotonic_s':9}):
            with self.subTest(changes=changes):
                self.assertFalse(IdleFeedbackRetry(self.config).reject_and_retry(
                    FeedbackAcquisitionTimeout(replace(self.late,**changes)),self.previous,None,None))
        self.assertFalse(self.retry.reject_and_retry(FeedbackError('read failed'),self.previous,None,None))
        self.assertFalse(self.retry.reject_and_retry(FeedbackAcquisitionTimeout(self.late),None,None,None))

    def test_changed_fresh_sample_cannot_complete_retry(self):
        self.retry.reject_and_retry(FeedbackAcquisitionTimeout(self.late),self.previous,None,None)
        with self.assertRaises(FeedbackError):
            self.retry.accept_fresh(self.previous,sample(10.3,position_turns=4.6))

    def test_config_bounds_and_old_limits(self):
        self.assertEqual(self.config['reference']['maximum_feedback_capture_ms'],150)
        self.assertEqual(self.config['reference']['feedback_stale_after_ms'],250)
        for key, value in (('idle_feedback_retry_count',4),('idle_feedback_retry_count',True),
                           ('idle_feedback_retry_count',1.5),('idle_feedback_retry_window_ms',250),
                           ('idle_feedback_retry_window_ms',2001),('idle_feedback_retry_window_ms',float('nan'))):
            with self.subTest(key=key,value=value), tempfile.TemporaryDirectory() as folder:
                config=copy.deepcopy(self.config);config['reference'][key]=value
                p=Path(folder)/'config.json';p.write_text(json.dumps(config))
                with self.assertRaises(ValueError):load_tester_config(p)

    def test_adapter_raises_typed_timeout_without_command(self):
        device=Device();adapter=ODriveAdapter(device,0,idle_state=1,closed_loop_state=8)
        with patch('east_odrive.time.monotonic',side_effect=[10,10.266]):
            with self.assertRaises(FeedbackAcquisitionTimeout) as error:
                adapter.snapshot(max_capture_duration_s=.15)
        self.assertAlmostEqual(error.exception.snapshot.capture_duration_s,.266)
        self.assertEqual(device.axis0.request_count,0)


class MonitorIntegrationTests(unittest.TestCase):
    def run_monitor(self, observations, expected=None, owner=None):
        coordinator=MotionCoordinator()
        if owner:coordinator.acquire(owner)
        app=SimpleNamespace(system_config=load_tester_config(),feedback_lock=threading.RLock(),
                            latest_feedback=sample(), expected_axis_state=expected,
                            expected_disarm_reason=None, feedback_retry_pending=False,
                            motion_coordinator=coordinator, monitor_stop_event=Mock(),
                            odrive_adapter=Mock(), state_store=Mock(), reference_manager=Mock(),
                            test_stop_event=threading.Event(), neutral_stop_event=threading.Event(),
                            continuous_stop_event=threading.Event(), ui_message_queue=queue.Queue(),
                            _mark_control_health=Mock(),monitor_error=None)
        app.monitor_stop_event.is_set.side_effect=[False]*len(observations)+[True]
        app.odrive_adapter.snapshot.side_effect=observations
        app.odrive_adapter.request_idle.return_value=SimpleNamespace(confirmed=True)
        gui_method('_feedback_monitor_loop')(app)
        return app

    def test_idle_delay_preserves_zero_publishes_only_fresh_checkpoint(self):
        late=sample(10.266,.266);fresh=sample(10.3)
        app=self.run_monitor([FeedbackAcquisitionTimeout(late),fresh])
        self.assertIs(app.latest_feedback,fresh)
        app.reference_manager.invalidate.assert_not_called()
        app.odrive_adapter.request_idle.assert_not_called()
        self.assertFalse(app.feedback_retry_pending)
        self.assertIsNone(app.monitor_error)
        app.state_store.append_event.assert_called_once()
        checkpoints=app.reference_manager.write_checkpoint.call_args_list
        self.assertEqual(len(checkpoints),1)
        self.assertIs(checkpoints[0].args[1],fresh)
        messages=list(app.ui_message_queue.queue)
        self.assertIn(('status','FEEDBACK DELAY / RETRYING','amber'),messages)
        self.assertIn(('status','IDLE / FEEDBACK RESTORED','green'),messages)

    def test_powered_latency_faults_immediately(self):
        for expected,owner in ((8,None),(None,'manual-step')):
            with self.subTest(expected=expected,owner=owner):
                app=self.run_monitor([FeedbackAcquisitionTimeout(sample(10.266,.266))],expected,owner)
                app.reference_manager.invalidate.assert_called_once()
                app.odrive_adapter.request_idle.assert_called_once()
                self.assertTrue(app.test_stop_event.is_set())
                self.assertEqual(app.state_store.append_event.call_count,0)

    def test_repeated_latency_faults_without_publishing_late_samples(self):
        app=self.run_monitor([FeedbackAcquisitionTimeout(sample(at,.266)) for at in (10.266,10.55,10.85)])
        app.reference_manager.invalidate.assert_called_once()
        self.assertEqual(app.state_store.append_event.call_count,2)
        app.reference_manager.write_checkpoint.assert_not_called()
        self.assertEqual(app.latest_feedback.captured_monotonic_s,10)

    def test_changed_fresh_feedback_faults_without_publishing(self):
        app=self.run_monitor([FeedbackAcquisitionTimeout(sample(10.266,.266)),sample(10.3,system_uptime=1)])
        app.reference_manager.invalidate.assert_called_once()
        self.assertEqual(app.latest_feedback.captured_monotonic_s,10)

    def test_other_feedback_failure_is_not_retried(self):
        app=self.run_monitor([FeedbackError('USB read failed')])
        app.reference_manager.invalidate.assert_called_once()
        app.state_store.append_event.assert_not_called()

    def test_motor_enable_and_position_submission_blocked_during_retry(self):
        app=SimpleNamespace(feedback_retry_pending=True,odrive_adapter=Mock(),motion_coordinator=Mock())
        for name,args in (('enter_closed_loop',(None,)),('_submit_position',(None,4.5))):
            with self.subTest(name=name),self.assertRaises(FeedbackError):gui_method(name)(app,*args)
        self.assertTrue(gui_method('_motion_cancelled')(app,None))
        self.assertEqual(app.odrive_adapter.mock_calls,[])
        self.assertEqual(app.motion_coordinator.mock_calls,[])


if __name__=='__main__':unittest.main()
