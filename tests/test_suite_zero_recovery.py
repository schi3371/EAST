"""Separate zero-recovery admission, failure preservation and no experiment replay."""
import copy
import json
import threading
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
import unittest
from east_motion_mapping import zero_recovery_candidate
import test_remaining_protocol as plan_harness
import test_motion_model as model_harness
from test_three_speed_protocol import window_for

class AdmissionTests(unittest.TestCase):
    def test_only_durable_full_acquisitions_with_zero_failure_qualify(self):
        request=plan_harness.PlanTests().plan()[0]
        m=dict(protocol_batch=request.context(),run_status='error',completed_cycles=3,
            outcome_pending_cleanup=False,sample_count=10,error='Model return missed rotary zero: -0.1059 degrees',
            zero_return_check=dict(passed=False,unassisted_angle_deg=-.1059),
            idle_confirmation=dict(confirmed=True,motion_owner=None,monitor_error=None),
            csv_recorder=dict(worker_complete=True,pending_rows=0,error=None,rows_persisted=10),
            rotary_sensor=dict(recording_statistics=dict(delivered_samples=20,invalid_samples=0,recording_error=None)))
        a=SimpleNamespace(run_metadata=m,run_finalized=True,strain_test_active=False,metadata_workers=[],shutdown_requested=False,
            engine=SimpleNamespace(busy=None,cancel_event=threading.Event()),
            motion_coordinator=SimpleNamespace(owner=None,stopping=False,shutting_down=False,motor_idle_confirmed=True),
            monitor_error=None,feedback_retry_pending=False,operator_stop_requested=False,acquisition_error=None)
        self.assertTrue(zero_recovery_candidate(a,request))
        for mutate in [lambda x:x.update(completed_cycles=2),lambda x:x['csv_recorder'].update(pending_rows=1),
                lambda x:x['csv_recorder'].update(rows_persisted=9),lambda x:x['rotary_sensor']['recording_statistics'].update(invalid_samples=1),
                lambda x:x.update(error='ODrive feedback is stale'),
                lambda x:x['idle_confirmation'].update(confirmed=False),lambda x:x.update(outcome_pending_cleanup=True)]:
            a.run_metadata=copy.deepcopy(m);mutate(a.run_metadata)
            self.assertFalse(zero_recovery_candidate(a,request))
        for angle in (-.2593, -.346, -.8):
            a.run_metadata = copy.deepcopy(m)
            a.run_metadata['zero_return_check']['unassisted_angle_deg'] = angle
            self.assertTrue(zero_recovery_candidate(a, request))
        a.run_metadata=m;a.engine.cancel_event.set();self.assertFalse(zero_recovery_candidate(a,request));a.engine.cancel_event.clear()
        a.acquisition_error='failed';self.assertFalse(zero_recovery_candidate(a,request))

class RecoveryTests(unittest.TestCase):
    setUp=model_harness.ApplicationTests.setUp
    tearDown=model_harness.ApplicationTests.tearDown
    pump=model_harness.ApplicationTests.pump
    ready=model_harness.ApplicationTests.ready
    prepare=model_harness.ApplicationTests.prepare

    def run_case(self,error,stop_recovery=False):
        a=self.prepare();a.loaded_preset_key='remaining_motion_data'
        # This multi-run fake-device test exercises recovery/ownership, not callback scheduling.
        # Dedicated stale-sample tests retain the production 100 ms threshold.
        a.rotary_sensor.config['stale_after_ms']=1000
        a.cycles_input.value='3';a.min_angle_input.value='10';a.max_angle_input.value='10';a.acceleration_input.value='100'
        a.start_rotary_homing=a.__class__.start_rotary_homing.__get__(a)
        reader=a.rotary_sensor.reader;raw=reader._sample
        reader._sample=lambda ch,v:raw(None,(self.angle-a.rotary_sensor.calibration.intercept)/a.rotary_sensor.calibration.slope)
        original=a._submit_position
        def submit(token,target):
            if token.owner=='rotary-home':
                before=a.get_feedback().position_turns;angle=self.angle
                a.__class__._submit_position(a,token,target)
                self.angle=angle+(target-before)*2.055
            else:
                original(token,target)
                if a.motion_mapping.request.index==1 and a.motion_phase=='returning_to_machine_zero':self.angle=error
        a._submit_position=submit
        w,module=window_for(a,self.module.ctk,'remaining_motion_data');builder=module.make_protocol_plan
        def short_plan(*args,**kwargs):
            return [replace(r,total=3,parameters=replace(r.parameters,cycles=1)) for r in builder(*args,**kwargs)[:3]]
        with patch.object(module,'make_protocol_plan',side_effect=short_plan):
            w.start();self.assertTrue(w.active,w.status.options)
            if stop_recovery:
                self.pump(lambda:(w.tick(),w.recovery_future is not None)[1],30)
                w.stop()
            try:
                self.pump(lambda:(w.tick(),not w.active)[1],15 if stop_recovery else 45)
            except AssertionError:
                print('RECOVERY DEBUG',dict(active=w.active,pending=w.pending_zero_recovery,
                    future_done=w.recovery_future.done() if w.recovery_future else None,
                    owner=a.motion_coordinator.owner,idle=a.motion_coordinator.motor_idle_confirmed,
                    stopping=a.motion_coordinator.stopping,busy=a.engine.busy,monitor=a.monitor_error,
                    threads={n:getattr(a,n).is_alive() if getattr(a,n,None) is not None else None
                        for n in ['stop_worker','cleanup_thread','strain_thread','data_collection_thread']},
                    save_action=w.save_action,save_done=w.save_future.done() if w.save_future else None))
                raise
        return w,json.loads(w.manifest_path.read_text())

    def test_small_miss_is_preserved_then_separate_home_allows_next_run(self):
        w,body=self.run_case(-.1059)
        self.assertEqual(body['outcome'],'completed_with_zero_recovery',body.get('error'))
        self.assertEqual([r['status'] for r in body['runs']],['acquired_zero_recovered','completed','completed'])
        first=json.loads(Path(body['runs'][0]['metadata_file']).read_text())
        self.assertEqual(first['run_status'],'error')
        self.assertEqual(first['zero_return_check']['unassisted_angle_deg'],-.1059)
        self.assertEqual(first['completed_cycles'],1)
        self.assertEqual(len(first['experimental_motion_model']['segments']),4)
        self.assertEqual(len(body['reference_history']),1)
        self.assertNotEqual(body['initial_reference']['record']['reference_id'],body['reference']['record']['reference_id'])
        self.assertTrue(Path(body['reference_history'][0]['evidence_file']).exists())
        evidence=json.loads(Path(body['reference_history'][0]['evidence_file']).read_text())
        self.assertEqual(evidence['purpose'],'separate_suite_zero_recovery')
        self.assertEqual(evidence['policy']['maximum_travel_deg'],20.)
        self.assertFalse(body['reference_history'][0]['reference_equivalence_assumed'])
        self.app._tare_worker.assert_not_called()

    def test_larger_miss_homes_and_continues_without_repeating_sweeps(self):
        w,body=self.run_case(-.3)
        self.assertEqual(body['outcome'],'completed_with_zero_recovery',body.get('error'))
        self.assertEqual([r['status'] for r in body['runs']],['acquired_zero_recovered','completed','completed'])
        self.assertEqual(len(body['reference_history']),1)

    def test_operator_start_four_creates_skipped_rows_without_certifying_prior_data(self):
        a=self.prepare();a.loaded_preset_key='remaining_motion_data'
        a.cycles_input.value='3';a.min_angle_input.value='10';a.max_angle_input.value='10';a.acceleration_input.value='100'
        a.start_strain_test=Mock(return_value=False)
        w,_=window_for(a,self.module.ctk,'remaining_motion_data')
        w.fields['start_run'].value='4'
        w.start();self.pump(lambda:(w.tick(),not w.active)[1],8)
        body=json.loads(w.manifest_path.read_text())
        self.assertEqual(body['selected_start_run'],4)
        self.assertEqual(body['selected_run_count'],7)
        self.assertEqual([r['status'] for r in body['runs'][:3]],['skipped_by_operator_before_start']*3)
        self.assertEqual(a.start_strain_test.call_args.kwargs['mapping_request'].index,4)
        self.assertTrue(all(r['metadata_file'] is None for r in body['runs'][:3]))

    def test_duplicate_stop_reconfirms_idle_after_first_acknowledgement(self):
        a=self.prepare();token=a.motion_coordinator.acquire('idle-race')
        entered=threading.Event();release=threading.Event();self.addCleanup(release.set)
        original=a.safe_idle_motor;calls=[]
        def idle(*args,**kwargs):
            result=original(*args,**kwargs);calls.append(result)
            if len(calls)==1:entered.set();release.wait(2)
            return result
        a.safe_idle_motor=idle
        a._request_coordinated_stop('first Stop')
        self.pump(entered.is_set,5)
        self.assertTrue(a.motion_coordinator.motor_idle_confirmed)
        a._request_coordinated_stop('second Stop')
        self.assertFalse(a.motion_coordinator.motor_idle_confirmed)
        release.set();self.pump(lambda:not a.stop_worker.is_alive(),5)
        self.assertGreaterEqual(len(calls),2)
        self.assertTrue(a.motion_coordinator.motor_idle_confirmed)
        a._retire_stop_worker();a.motion_coordinator.release(token)

    def test_stop_cancels_recovery_and_prevents_successors(self):
        w,body=self.run_case(-.1059,stop_recovery=True)
        self.assertEqual(body['outcome'],'cancelled',body.get('error'))
        self.assertEqual([r['status'] for r in body['runs'][1:]],['skipped','skipped'])
        self.assertIsNone(self.app.motion_coordinator.owner)
        self.assertFalse(self.app.rotary_homing.active)

if __name__=='__main__':unittest.main()
