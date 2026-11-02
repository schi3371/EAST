"""Audited cross-component failures: no physical devices or unbounded waits."""
import unittest,json,threading
from pathlib import Path
from unittest.mock import Mock,patch
from dataclasses import replace
from types import SimpleNamespace
import test_motion_model as models
import test_rotary_homing as homes
from test_three_speed_protocol import window_for
from east_motion_mapping import ready_for_next

class HandoffTests(unittest.TestCase):
    setUp=models.ApplicationTests.setUp
    tearDown=models.ApplicationTests.tearDown
    pump=models.ApplicationTests.pump
    ready=models.ApplicationTests.ready
    prepare=models.ApplicationTests.prepare
    def setup_batch(self,angle=.075):
        a=self.prepare();self.angle=angle
        a.system_config['motion']['experimental_model']['enabled']=False
        a.rotary_sensor.config['stale_after_ms']=1000
        a.start_rotary_homing=a.__class__.start_rotary_homing.__get__(a)
        a.loaded_preset_key='model_comparison'
        a.cycles_input.value='3';a.min_angle_input.value='15';a.max_angle_input.value='15';a.acceleration_input.value='100'
        p0=a.get_feedback().position_turns
        a.reference_manager.mapping=replace(a.reference_manager.mapping,neutral_position_turns=p0+.3/2.055)
        raw=a.__class__._submit_position.__get__(a);self.targets=[]
        def submit(token,target):
            before=a.get_feedback().position_turns
            raw(token,target)
            self.angle+=(target-before)*2.055
            if token.owner=='strain-test':self.targets.append(target)
        a._submit_position=submit
        w,module=window_for(a,self.module.ctk,'model_comparison');builder=module.make_protocol_plan
        def short(*args,**kwargs):
            r=builder(*args,**kwargs)[0]
            return [replace(r,total=1)]
        self.plan_patch=patch.object(module,'make_protocol_plan',side_effect=short);self.plan_patch.start();self.addCleanup(self.plan_patch.stop)
        return a,w,p0
    def test_physical_zero_overrides_legacy_motor_angle_and_prepares_reference(self):
        a,w,p0=self.setup_batch()
        self.assertTrue(a.rotary_zero_ready())
        tare=a.empty_machine_tare
        w.start();self.assertTrue(w.active,w.status.options)
        self.pump(lambda:(w.tick(),not w.active)[1],15)
        self.assertEqual(w.batch_status,'completed',w.batch_error)
        prep=w.preparation_records[0]
        self.assertTrue(prep['verification_only']);self.assertEqual(prep['status'],'completed')
        m=json.loads(Path(w.records[0]['metadata_file']).read_text())
        self.assertAlmostEqual(self.targets[0],p0+(5-.075)/2.055,places=5)
        self.assertAlmostEqual(m['original_run_anchor']['rotary_angle_deg'],.075,places=5)
        self.assertTrue(m['zero_return_check']['passed'])
        self.assertEqual(m['motor_feedback_statistics']['scope'],'run_window')
        self.assertEqual(m['motor_feedback_connection_statistics']['scope'],'connection_lifetime')
        self.assertIs(a.empty_machine_tare,tare)
    def test_out_of_band_pose_is_prepared_before_original_motion(self):
        a,w,p0=self.setup_batch(.27)
        w.start();self.assertTrue(w.active,w.status.options)
        self.pump(lambda:(w.tick(),not w.active)[1],18)
        self.assertEqual(w.batch_status,'completed',w.batch_error)
        self.assertFalse(w.preparation_records[0]['verification_only'])
        self.assertEqual(w.preparation_records[0]['status'],'completed')
    def test_stop_during_preparation_never_dispatches_test(self):
        a,w,p0=self.setup_batch()
        w.start();self.pump(lambda:(w.tick(),w.preparation_future is not None)[1],8)
        w.stop();self.pump(lambda:(w.tick(),not w.active)[1],10)
        self.assertEqual(w.batch_status,'cancelled',w.batch_error)
        self.assertEqual(w.records,[]);self.assertEqual(self.targets,[])
        self.assertEqual(w.preparation_records[0]['status'],'cancelled')
        self.assertIsNone(a.motion_coordinator.owner)
    def test_scheduler_exception_finalises_error_manifest_without_dispatch(self):
        a,w,p0=self.setup_batch()
        w.start()
        w.identity.configure=Mock(side_effect=RuntimeError('simulated UI failure'))
        self.pump(lambda:(w.tick(),not w.active)[1],10)
        body=json.loads(w.manifest_path.read_text())
        self.assertEqual(body['outcome'],'error');self.assertIn('simulated UI failure',body['error'])
        self.assertEqual(self.targets,[]);self.assertIsNone(a.motion_coordinator.owner)
    def test_terminal_warning_text_does_not_cancel_healthy_batch(self):
        a,w,p0=self.setup_batch()
        w.start();w.append_terminal('An old warning: Stop/cleanup is still running')
        self.assertFalse(w.cancelled.is_set())
        w.stop();self.pump(lambda:(w.tick(),not w.active)[1],10)
    def test_main_window_stop_cancels_before_next_run(self):
        a,w,p0=self.setup_batch()
        w.start();a.stop_logging()
        self.pump(lambda:(w.tick(),not w.active)[1],10)
        self.assertEqual(w.batch_status,'cancelled',w.batch_error)
        self.assertEqual(self.targets,[])

    def test_live_recorder_prevents_new_motion_admission(self):
        a=self.prepare();self.pump(lambda:ready_for_next(a))
        real=a.rotary_sensor.recording
        a.rotary_sensor.recording=SimpleNamespace(thread=SimpleNamespace(is_alive=lambda:True))
        self.assertFalse(ready_for_next(a))
        a.rotary_sensor.recording=real

class HomingSettlingTests(unittest.TestCase):
    setUp=homes.ApplicationTests.setUp
    tearDown=homes.ApplicationTests.tearDown
    pump=homes.ApplicationTests.pump
    ready=homes.ApplicationTests.ready
    prepare=homes.ApplicationTests.prepare
    run_home=homes.ApplicationTests.run_home
    def test_release_velocity_transient_requires_new_stationary_dwell(self):
        window=self.prepare(-.02);a=self.app
        idle=a.safe_idle_motor;feedback=a.get_feedback;left=[0]
        def after_idle(*args,**kwargs):
            result=idle(*args,**kwargs);left[0]=4;return result
        def read():
            snap=feedback()
            if left[0]:left[0]-=1;return replace(snap,velocity_turns_s=.1)
            return snap
        a.safe_idle_motor=after_idle;a.get_feedback=read
        evidence=self.run_home(window)
        self.assertEqual(evidence['outcome'],'sensor_zero_reached',evidence.get('error'))
        self.assertGreater(evidence['stationary_observation_resets'],0)
        self.assertGreaterEqual(evidence['final_observation']['duration_s'],.5)
