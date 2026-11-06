"""Complete high-speed matrix, scoped limits, real GUI recording and continuation."""
from collections import Counter
from copy import deepcopy
import json
from pathlib import Path
import unittest

from east_core import load_tester_config, validate_test_parameters
from east_full_model_suite import PRESET, SPEEDS, rows, make_plan, condition_key, validate_models
from east_feedback_commissioning import policy
from east_stream_screen import MODELS, _coefficients, _evaluate
import test_motion_model as harness
from test_three_speed_protocol import window_for


class PlanTests(unittest.TestCase):
    def plan(self):
        return make_plan(load_tester_config(), dict(operator='SC',fixture_id='0',calibration_id='0'),
            dict(cycles=1,minimum_angle_deg=15,maximum_angle_deg=15,acceleration_deg_s2=100),
            'setup','','manifest.json','now')

    def test_all_models_all_speeds_three_repeats_full_range(self):
        plan=self.plan()
        self.assertEqual(len(plan),147)
        counts=Counter((r.parameters.commanded_afo_speed_deg_s,r.preset_metadata['screen_method']) for r in plan)
        self.assertEqual(set(counts), {(s,m) for s in SPEEDS for m in (*MODELS,'feedback')})
        self.assertEqual(set(counts.values()),{3})
        self.assertTrue(all(r.parameters.cycles==1 and r.parameters.max_angle_deg==15
                            and r.parameters.min_angle_deg==15 and r.parameters.commanded_afo_acceleration_deg_s2==100
                            and r.parameters.test_type=='empty_machine_baseline' for r in plan))
        self.assertEqual([r.index for r in plan],list(range(1,148)))
        self.assertEqual(len({r.parameters.file_prefix for r in plan}),147)
        self.assertEqual([r.model_variant for r in plan[126:133]],list(MODELS)+['candidate'])

    def test_only_saved_healthy_model_failures_may_skip_to_next_condition(self):
        from types import SimpleNamespace
        import threading
        from east_motion_mapping import zero_recovery_candidate
        request=self.plan()[126]
        metadata=dict(protocol_batch=request.context(),run_status='error',completed_cycles=0,
            error='Rotary pilot leg timed out',sample_count=10,outcome_pending_cleanup=False,
            idle_confirmation=dict(confirmed=True,motion_owner=None,monitor_error=None),
            csv_recorder=dict(worker_complete=True,pending_rows=0,error=None,rows_persisted=10),
            rotary_sensor=dict(recording_statistics=dict(delivered_samples=20,invalid_samples=0,recording_error=None)))
        app=SimpleNamespace(run_metadata=metadata,run_finalized=True,strain_test_active=False,metadata_workers=[],shutdown_requested=False,
            engine=SimpleNamespace(busy=None,cancel_event=threading.Event()),
            motion_coordinator=SimpleNamespace(owner=None,stopping=False,shutting_down=False,motor_idle_confirmed=True),
            monitor_error=None,feedback_retry_pending=False,operator_stop_requested=False,acquisition_error=None)
        self.assertTrue(zero_recovery_candidate(app,request))
        for error in ('ODrive feedback is stale', 'Rotary travel guard exceeded', 'disk failure'):
            metadata['error']=error
            self.assertFalse(zero_recovery_candidate(app,request))
        metadata['error']='Rotary pilot leg timed out'
        app.operator_stop_requested=True
        self.assertFalse(zero_recovery_candidate(app,request))
        app.operator_stop_requested=False
        metadata['csv_recorder']['pending_rows']=1
        self.assertFalse(zero_recovery_candidate(app,request))

    def test_high_speed_limits_are_scoped_and_model_branches_cover_endpoints(self):
        config=load_tester_config();snapshot=deepcopy(config)
        validate_models()
        for speed in SPEEDS:
            limits=policy(config,condition_key(speed))
            self.assertEqual(limits['tracking_error_action'],'record')
            self.assertEqual(limits['travel_guard_deg'],16)
            self.assertEqual(limits['reference_period_s'],.04)
            self.assertEqual(limits['maximum_leg_timeout_s'],90)
            self.assertLessEqual(limits['motor_speed_nominal_deg_s'],50)
            self.assertLessEqual(limits['motor_acceleration_nominal_deg_s2'],100)
        self.assertEqual(config,snapshot)
        self.assertEqual(config['motion']['maximum_speed_deg_s'],20)
        self.assertEqual(config['motion']['maximum_motion_timeout_s'],60)
        values=dict(file_prefix='test',operator='SC',fixture_id='0',calibration_id='0',afo_id='AFO',
            test_type='afo_test',cycles=1,speed_deg_s=50,acceleration_deg_s2=100,min_angle_deg=15,max_angle_deg=15)
        with self.assertRaisesRegex(ValueError,'Speed'):
            validate_test_parameters(values,config)


class WindowTests(unittest.TestCase):
    setUp=harness.ApplicationTests.setUp
    tearDown=harness.ApplicationTests.tearDown
    pump=harness.ApplicationTests.pump
    ready=harness.ApplicationTests.ready
    prepare=harness.ApplicationTests.prepare

    def prepare_suite(self):
        app=self.prepare();app.loaded_preset_key=PRESET
        app.cycles_input.value='1';app.min_angle_input.value=app.max_angle_input.value='15'
        app.acceleration_input.value='100'
        app.rotary_sensor.config['stale_after_ms']=1000
        reader=app.rotary_sensor.reader;raw=reader._sample
        reader._sample=lambda ch,v:raw(None,(self.angle-app.rotary_sensor.calibration.intercept)/app.rotary_sensor.calibration.slope)
        dispatch=app.__class__._submit_position.__get__(app)
        def follow(token,target):
            before=app.get_feedback().position_turns
            dispatch(token,target)
            if token.owner == 'rotary-home':
                self.angle += (target-before)*2.055
                return
            if app.run_motion_model is not None:
                row=app.run_metadata['rotary_feedback_pilot']['samples'][-1]
                self.angle=_evaluate(_coefficients(app.run_model_variant,row['direction']),target-app.run_motion_model.origin_turns)
                if (getattr(self,'inject_zero_miss',False) and app.run_model_variant=='quintic'
                        and row['phase']=='returning_to_machine_zero' and abs(row['desired_angle_deg'])<.03):
                    self.angle=-.26
                if (getattr(self,'inject_motion_timeout',False) and not getattr(self,'timeout_injected',False)
                        and app.motion_mapping.request.index==127 and row['desired_angle_deg']>.2):
                    self.timeout_injected=True
                    raise RuntimeError('Rotary pilot leg timed out')
        app._submit_position=follow
        window,module=window_for(app,self.module.ctk,PRESET)
        return app,window

    def test_fast_block_runs_every_method_and_saves_each_without_clicks(self):
        app,window=self.prepare_suite()
        self.assertEqual(window.fields['end_run'].get(),'')
        window.fields['start_run'].value='127';window.fields['end_run'].value='133'
        self.inject_zero_miss=True
        app.start_rotary_homing=app.__class__.start_rotary_homing.__get__(app)
        window.start();self.assertTrue(window.active,window.status.options)
        self.pump(lambda:(window.tick(),not window.active)[1],110)
        self.assertEqual(window.batch_status,'completed_with_zero_recovery',window.batch_error)
        recovered=window.records[5]
        self.assertTrue(recovered['acquisition_complete'])
        original=json.loads(Path(recovered['metadata_file']).read_text())
        self.assertFalse(original['zero_return_check']['passed'])
        self.assertAlmostEqual(original['zero_return_check']['unassisted_angle_deg'],-.26,places=3)
        self.assertTrue(recovered['zero_recovery'])
        self.assertEqual(len(window.records),7)
        for record in window.records:
            meta=json.loads(Path(record['metadata_file']).read_text())
            stream=meta['rotary_feedback_pilot']
            self.assertEqual(meta['completed_cycles'],1)
            self.assertEqual(stream['period_s'],.04)
            self.assertEqual(stream['commissioning_policy']['speed_deg_s'],50)
            self.assertEqual(len(stream['legs']),4)
            self.assertTrue(meta['csv_recorder']['worker_complete'])
            self.assertEqual(meta['sample_count'],meta['csv_recorder']['rows_persisted'])
            self.assertEqual(meta['experimental_motion_model']['target_limit_deg'],15)
            self.assertEqual(meta['protocol_batch']['protocol_key'],PRESET)
        self.assertIsNone(app.motion_coordinator.owner)

    def test_incomplete_model_run_is_saved_homed_and_skipped_without_replay(self):
        app,window=self.prepare_suite()
        self.inject_motion_timeout=True
        app.start_rotary_homing=app.__class__.start_rotary_homing.__get__(app)
        window.fields['start_run'].value='127';window.fields['end_run'].value='128'
        window.start();self.assertTrue(window.active,window.status.options)
        self.pump(lambda:(window.tick(),not window.active)[1],45)
        self.assertEqual(window.batch_status,'completed_with_failed_runs',window.batch_error)
        self.assertEqual(len(window.records),2)
        first=window.records[0]
        self.assertFalse(first['acquisition_complete'])
        self.assertTrue(first['failed_run_skipped'])
        metadata=json.loads(Path(first['metadata_file']).read_text())
        self.assertEqual(metadata['run_status'],'error')
        self.assertEqual(metadata['completed_cycles'],0)
        self.assertTrue(metadata['error'].startswith('Rotary pilot leg timed out'))
        self.assertEqual(window.records[1]['run_status'],'completed')
        manifest=json.loads(window.manifest_path.read_text())
        selected=[r for r in manifest['runs'] if r['status'] not in ('skipped_by_operator_before_start','outside_selected_run_range')]
        self.assertEqual([r['status'] for r in selected],['failed_run_recovered_skipped','completed'])
        self.assertIsNone(app.motion_coordinator.owner)

    def test_restart_refuses_to_discard_an_unfinished_homing_transaction(self):
        from concurrent.futures import Future
        app,window=self.prepare_suite()
        pending=Future();window.recovery_future=pending
        window.start()
        self.assertFalse(window.active)
        self.assertIs(window.recovery_future,pending)
        self.assertIn('previous zero transaction',window.status.options['text'])
        self.assertIsNone(app.motion_coordinator.owner)

    def test_recovery_rejects_success_evidence_from_another_batch(self):
        from concurrent.futures import Future
        from unittest.mock import Mock
        app,window=self.prepare_suite()
        request=make_plan(app.system_config,dict(operator='SC',fixture_id='0',calibration_id='0'),
            dict(cycles=1,minimum_angle_deg=15,maximum_angle_deg=15,acceleration_deg_s2=100),
            'setup','','manifest.json','now')[126]
        window.request=request;window.records=[{}];window.pending_zero_recovery=True
        future=Future();future.set_result(dict(evidence=dict(outcome='sensor_zero_reached',
            idle_confirmed=True,zero_reference_written=True,parent_batch_id='old-batch',parent_run_id=request.run_id)))
        window.recovery_future=future;window.finish=Mock()
        window.next_run()
        self.assertEqual(window.finish.call_args.args[0],'error')
        self.assertIn('another batch/run',window.finish.call_args.args[1])
        self.assertIsNone(app.motion_coordinator.owner)

    def test_restart_same_window_discards_failed_recovery_and_launches_fresh_home(self):
        from concurrent.futures import Future
        from types import SimpleNamespace
        app,window=self.prepare_suite()
        self.inject_zero_miss=True
        app.start_rotary_homing=app.__class__.start_rotary_homing.__get__(app)
        actual_recovery=app.start_suite_zero_recovery
        failed=Future()
        def fail_first(owner):
            app.rotary_homing=SimpleNamespace(active=False,closed=False,
                evidence_path='simulated_first_home.json')
            failed.set_result(dict(message='SIMULATED homing failure',evidence=dict(
                outcome='failed',error='SIMULATED ODrive feedback is stale',idle_confirmed=True,
                zero_reference_written=False,parent_batch_id=owner.request.batch_id,
                parent_run_id=owner.request.run_id)))
            return failed
        app.start_suite_zero_recovery=fail_first
        window.fields['start_run'].value=window.fields['end_run'].value='132'
        window.start();self.assertTrue(window.active,window.status.options)
        self.pump(lambda:(window.tick(),not window.active)[1],35)
        self.assertEqual(window.batch_status,'error')
        self.assertIn('SIMULATED ODrive feedback is stale',window.batch_error)
        self.assertIs(window.recovery_future,failed)
        old_manifest=window.manifest_path
        old_body=json.loads(old_manifest.read_text())
        old_id=window.requests[0].batch_id
        old_metadata=Path(window.records[0]['metadata_file']).read_text()
        # Stand in for the operator's successful manual home between batches.
        self.angle=0.
        self.pump(lambda:abs(app.rotary_sensor.measurement()['angle_deg'])<.01)
        app.start_suite_zero_recovery=actual_recovery
        window.fields['end_run'].value='133'
        window.start();self.assertTrue(window.active,window.status.options)
        self.assertIsNone(window.recovery_future)
        self.assertFalse(window.pending_zero_recovery)
        self.assertNotEqual(window.requests[0].batch_id,old_id)
        self.pump(lambda:(window.tick(),not window.active)[1],45)
        self.assertEqual(window.batch_status,'completed_with_zero_recovery',window.batch_error)
        self.assertEqual(len(window.records),2)
        attempt=window.records[0]['zero_recovery_attempt']
        self.assertEqual(attempt['parent_batch_id'],window.requests[0].batch_id)
        self.assertTrue(attempt['zero_reference_written'])
        self.assertEqual(window.records[1]['run_status'],'completed')
        self.assertEqual(json.loads(old_manifest.read_text()),old_body)
        self.assertEqual(Path(old_body['runs'][131]['metadata_file']).read_text(),old_metadata)
        self.assertIsNone(app.motion_coordinator.owner)

    def test_stop_during_fast_run_cancels_remaining(self):
        app,window=self.prepare_suite()
        window.fields['start_run'].value='131';window.fields['end_run'].value='133'
        window.start();self.assertTrue(window.active,window.status.options)
        self.pump(lambda:(window.tick(),len((app.run_metadata or {}).get('rotary_feedback_pilot',{}).get('samples',[]))>=5)[1],12)
        window.stop();self.pump(lambda:(window.tick(),not window.active)[1],12)
        self.assertEqual(window.batch_status,'cancelled',window.batch_error)
        self.assertEqual(len(window.records),1)
        self.assertIsNone(app.motion_coordinator.owner)
