"""Offline policy and actual application-service tests. No physical devices."""
import json
import math
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest
import unicodedata
from unittest.mock import patch

import test_engine_gui_integration as harness
from east_rotary_homing import HomingPolicy, home
from east_motion_mapping import ready_for_next
from east_reference import FeedbackSnapshot, wait_for_settle


class PolicyTests(unittest.TestCase):
    def simulated(self, start=-9.7, factor=1.0, policy=None):
        state = {'angle':start, 'steps':[]}
        def observe(duration):
            a = state['angle']
            return dict(mean_deg=a,min_deg=a-.001,max_deg=a+.001,samples=20)
        def move(delta):
            state['steps'].append(delta)
            state['angle'] += delta*factor
        result = home(policy or HomingPolicy(),observe,move,lambda:None)
        return state,result

    def test_converges_from_both_sides_with_bounded_corrections(self):
        for initial in (-9.7,9.7,0):
            state,result = self.simulated(initial,1.15)
            self.assertLessEqual(abs(state['angle'])+.001,.1)
            self.assertTrue(all(abs(v)<=.5 for v in state['steps']))
            self.assertLessEqual(sum(map(abs,state['steps'])),20)
            self.assertFalse(result['zero_reference_written'])

    def test_rejects_wrong_direction_stuck_and_limits(self):
        for factor, policy in [(-1,HomingPolicy()),(0,HomingPolicy()),
                (1,HomingPolicy(maximum_corrections=1)),(1,HomingPolicy(maximum_travel_deg=.2))]:
            with self.subTest(factor=factor,policy=policy), self.assertRaises(RuntimeError):
                self.simulated(-9.7,factor,policy)

    def test_noise_is_not_accepted_from_mean_alone(self):
        with self.assertRaisesRegex(RuntimeError,'noise'):
            home(HomingPolicy(),lambda d:dict(mean_deg=0,min_deg=-.2,max_deg=.2,samples=20),
                 lambda d:self.fail('No correction should chase zero-mean noise'),lambda:None)

    def test_cancel_timeout_and_insufficient_samples_prevent_movement(self):
        obs=lambda d:dict(mean_deg=-1,min_deg=-1,max_deg=-1,samples=20)
        def stopped():raise RuntimeError('Stopped')
        with self.assertRaisesRegex(RuntimeError,'Stopped'):
            home(HomingPolicy(),obs,lambda d:self.fail('Moved after Stop'),stopped)
        ticks=iter((0,121))
        with self.assertRaisesRegex(RuntimeError,'timed out'):
            home(HomingPolicy(),obs,lambda d:self.fail('Moved after deadline'),lambda:None,lambda:next(ticks))
        with self.assertRaisesRegex(RuntimeError,'Insufficient'):
            home(HomingPolicy(),lambda d:{**obs(d),'samples':1},lambda d:self.fail('Moved'),lambda:None)

    def test_settle_uses_time_after_snapshot_provider_returns(self):
        now=[0.0]
        def provider():
            now[0]+=.001  # A monitor publishes a newer snapshot during the provider call.
            return FeedbackSnapshot(now[0],'test',0,0,0,1)
        result=wait_for_settle(provider,0,.01,.01,.01,1,.25,lambda:False,
            clock=lambda:now[0],sleeper=lambda d:now.__setitem__(0,now[0]+d))
        self.assertEqual(result.current_state,1)

    def test_unvalidated_tolerances_are_rejected(self):
        for tolerance in (0.01,0.05,0.2,-1,float('nan'),float('inf'),.6):
            with self.assertRaises(ValueError):HomingPolicy(tolerance_deg=tolerance).validate()


class ApplicationTests(unittest.TestCase):
    setUp = harness.FullApplicationIntegration.setUp
    tearDown = harness.FullApplicationIntegration.tearDown
    pump = harness.FullApplicationIntegration.pump
    ready = harness.FullApplicationIntegration.ready

    def prepare(self, angle=-1, invalidate=True):
        # Test-only scheduler allowances; production timing limits stay unchanged.
        self.app.system_config['reference'].update(maximum_feedback_capture_ms=1000,feedback_stale_after_ms=2000)
        self.ready()
        self.pump(lambda:ready_for_next(self.app))
        self.sensorstop.set();self.sensorthread.join(1)
        self.sensorstop=threading.Event()
        self.physical_offset=angle
        self.origin=self.device.axis0.pos_vel_mapper.pos_rel
        if invalidate:
            self.app.reference_manager.invalidate_local('SIMULATED lost session reference',fault=False)
        reader=self.app.rotary_sensor.reader
        self.dynamic_reader=reader
        self.sensor_fault=None
        # Override pump's fake static injection by transforming all fake callbacks
        # into readings derived from this simulated fixture position.
        original=reader._sample
        def sample(_channel,_ignored):
            physical=self.physical_offset+(self.device.axis0.pos_vel_mapper.pos_rel-self.origin)*2.055
            profile=self.app.rotary_sensor.calibration
            ratio=(physical-profile.intercept)/profile.slope
            original(None,ratio)
        reader._sample=sample
        def events():
            while not self.sensorstop.is_set():
                reader._sample(None,0)
                self.sensorstop.wait(.004)
        self.sensorthread=threading.Thread(target=events,daemon=True);self.sensorthread.start()
        self.pump(lambda:abs(self.app.rotary_sensor.measurement()['angle_deg']-angle)<.01)
        self.position_targets=[]
        original_submit=self.app._submit_position
        def submit(token,target):
            self.position_targets.append(target)
            return original_submit(token,target)
        self.app._submit_position=submit
        window=SimpleNamespace(closed=False)
        self.app.rotary_homing=window
        return window

    def run_home(self, window):
        future=self.app.start_rotary_homing(window,HomingPolicy())
        self.assertIsNotNone(future)
        self.pump(future.done,20)
        return future.result()['evidence']

    def test_unreferenced_fixture_converges_and_writes_rotary_machine_zero(self):
        window=self.prepare()
        before=self.app.reference_manager.metadata_snapshot()
        evidence=self.run_home(window)
        self.assertEqual(evidence['outcome'],'sensor_zero_reached',evidence)
        self.assertLessEqual(abs(evidence['final_observation']['mean_deg']),.1)
        self.assertTrue(evidence['zero_reference_written'])
        self.assertTrue(self.app.rotary_reference_verified())
        self.assertNotEqual(self.app.reference_manager.metadata_snapshot()['record']['reference_id'],before['record']['reference_id'])
        self.assertTrue(evidence['idle_confirmed'])
        self.assertIsNone(self.app.motion_coordinator.owner)
        self.assertGreater(evidence['rotary_recording_statistics']['delivered_samples'],0)
        self.assertTrue(Path(evidence['raw_rotary_file']).exists())

    def test_existing_verified_zero_is_refreshed_from_rotary_on_success(self):
        window=self.prepare(-.02,invalidate=False)
        before=self.app.reference_manager.metadata_snapshot()
        evidence=self.run_home(window)
        self.assertEqual(evidence['outcome'],'sensor_zero_reached')
        self.assertTrue(self.app.reference_manager.verified)
        self.assertNotEqual(self.app.reference_manager.metadata_snapshot()['session_mapping']['reference_id'],before['session_mapping']['reference_id'])
        self.assertTrue(evidence['zero_reference_written'])
        self.assertTrue(self.app.rotary_reference_verified())

    def test_stop_prevents_success_and_retires_ownership(self):
        window=self.prepare(-3)
        future=self.app.start_rotary_homing(window,HomingPolicy())
        self.pump(lambda:len(self.position_targets)>0,8)
        self.app.stop_logging()
        self.pump(future.done,10)
        evidence=future.result()['evidence']
        self.assertIn(evidence['outcome'],('cancelled','failed'))  # Stop may race an independently latched sensor/feedback fault.
        self.pump(lambda:self.app.motion_coordinator.owner is None,5)
        self.assertFalse(self.app.reference_manager.verified)

    def test_idle_drift_rejects_success(self):
        window=self.prepare(-.02)
        idle=self.app.safe_idle_motor
        def drifting_idle(*args,**kwargs):
            result=idle(*args,**kwargs)
            self.physical_offset=.4
            return result
        self.app.safe_idle_motor=drifting_idle
        evidence=self.run_home(window)
        self.assertEqual(evidence['outcome'],'failed')
        self.assertIn('stopping band',evidence['error'])

    def test_sensor_loss_during_homing_stops(self):
        window=self.prepare(-3)
        future=self.app.start_rotary_homing(window,HomingPolicy())
        self.pump(lambda:len(self.position_targets)>0,8)
        self.dynamic_reader.error='SIMULATED rotary failure'
        self.pump(future.done,10)
        evidence=future.result()['evidence']
        self.assertEqual(evidence['outcome'],'failed')
        self.assertTrue(evidence['idle_confirmed'])

    def test_failed_evidence_write_prevents_motor_enable(self):
        window=self.prepare(-1)
        with patch.object(self.module,'write_json_atomic',side_effect=OSError('SIMULATED disk full')):
            future=self.app.start_rotary_homing(window,HomingPolicy())
            self.pump(future.done,10)
            with self.assertRaises(OSError):future.result()
        self.assertFalse(self.position_targets)
        self.assertIsNone(self.app.motion_coordinator.owner)

    def test_failed_final_evidence_write_invalidates_zero(self):
        window=self.prepare(-.02)
        original=self.module.write_json_atomic
        def save(path,body):
            if body.get('zero_reference_written'):
                raise OSError('SIMULATED final evidence write failed')
            return original(path,body)
        with patch.object(self.module,'write_json_atomic',side_effect=save):
            future=self.app.start_rotary_homing(window,HomingPolicy())
            self.pump(future.done,10)
            with self.assertRaisesRegex(OSError,'final evidence'):
                future.result()
        self.assertFalse(self.app.rotary_reference_verified())
        self.assertIsNone(self.app.motion_coordinator.owner)

    def test_cancelled_home_does_not_continue_into_test(self):
        self.prepare(-3)
        self.app.homing_acknowledged.value=True
        self.app.start_strain_test()
        self.pump(lambda:len(self.position_targets)>0,8)
        self.app.stop_logging()
        self.pump(lambda:not self.app.rotary_homing.active,10)
        self.assertIsNone(self.app.strain_thread)
        self.assertFalse(self.app.rotary_reference_verified())

    def test_stop_after_home_completion_cancels_pending_test_start(self):
        self.prepare(-.02)
        self.app.homing_acknowledged.value=True
        self.app.start_strain_test()
        state=self.app.rotary_homing
        with patch.object(self.app,'_poll_rotary_homing'):
            self.pump(state.future.done,10)
            self.assertEqual(state.future.result()['evidence']['outcome'],'sensor_zero_reached')
            self.app.stop_logging()
            self.pump(lambda:self.app.stop_worker is not None and not self.app.stop_worker.is_alive(),5)
        self.app._poll_rotary_homing()
        self.assertFalse(state.active)
        self.assertIsNone(self.app.strain_thread)

    def test_main_action_recovers_feedback_and_homes_with_afo_without_retare(self):
        window=self.prepare(-1)
        tare=self.app.empty_machine_tare
        self.app.homing_condition.value='AFO fitted'
        self.app.homing_acknowledged.value=True
        self.app._stop_feedback_monitor()
        self.app._latch_feedback_fault('SIMULATED stale feedback',self.app.odrive_connection_generation,self.app.odrive_adapter)
        self.app.open_rotary_homing()
        self.pump(lambda:not self.app.rotary_homing.active,20)
        self.assertTrue(self.app.rotary_reference_verified(),self.app.terminal.value)
        self.assertIs(self.app.empty_machine_tare,tare)
        self.assertIsNone(self.app.recovery_window)
        root=Path(self.app.system_config['logging']['output_directory'])
        meta=json.loads(next(root.glob('rotary_home_*.json')).read_text())
        self.assertEqual(meta['fixture_condition'],'AFO fitted')
        self.assertTrue(meta['zero_reference_written'])

    def test_start_prepares_zero_automatically_then_starts_a_new_test(self):
        self.prepare(-1)
        self.app.homing_acknowledged.value=True
        self.app.start_strain_test()
        self.assertTrue(self.app.rotary_homing.active)
        self.pump(lambda:self.app.run_metadata is not None and self.app.run_metadata.get('run_status')=='completed',25)
        self.assertTrue(self.app.rotary_reference_verified())
        self.assertTrue(self.app.run_metadata['rotary_zero_reference']['method'].startswith('rotary_sensor_homing:'))
        self.assertIsNone(self.app.recovery_window)

    def test_unhealthy_feedback_is_blocked_before_admission(self):
        window=self.prepare()
        self.app.monitor_error='SIMULATED stale feedback'
        with self.assertRaisesRegex(ValueError,'healthy ODrive'):
            self.app.start_rotary_homing(window,HomingPolicy())
        self.assertFalse(self.position_targets)


if __name__=='__main__':unittest.main()
