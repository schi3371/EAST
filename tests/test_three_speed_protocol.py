"""Offline plans and real GUI/service execution with fake devices. Never opens hardware."""
import copy
import csv
import importlib.util
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
import unicodedata
from unittest.mock import Mock,patch

import test_engine_gui_integration as harness
from east_core import load_tester_config,CSV_COLUMNS,ROTARY_RUN_COLUMNS,MOTOR_TIMING_COLUMNS
from east_protocol_batch import make_protocol_plan,SPEEDS
from east_motion_mapping import completed_run,ready_for_next

class Text(harness.Field):
    def index(self,*args):return '1.0'

def window_for(app,ctk,key):
    path=Path(__file__).resolve().parents[1]/'east_protocol_batch_gui.py'
    spec=importlib.util.spec_from_file_location('offline_protocol_window',path)
    module=importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,{'customtkinter':ctk}),patch.object(ctk,'CTkEntry',harness.Field,create=True),patch.object(ctk,'CTkTextbox',Text,create=True):
        spec.loader.exec_module(module)
        window=module.ThreeSpeedProtocolWindow(app.master,app,key)
    app.motion_mapping=window
    return window,module

class PlanTests(unittest.TestCase):
    def setUp(self):
        self.config=load_tester_config()
        self.identity=dict(file_prefix='SpecimenRun',operator='SC',afo_id='AFO-42',fixture_id='0',calibration_id='CAL')
        self.settings=dict(cycles='3',minimum_angle_deg='10',maximum_angle_deg='10',acceleration_deg_s2='100',speed_deg_s='.5')
    def plan(self,key='three_speed_afo',**changes):
        return make_protocol_plan(self.config,key,self.identity,{**self.settings,**changes},'MOUNT-42','','batch.json','now')
    def test_explicit_plan_defaults_provenance_and_frozen_identity(self):
        p=self.plan();self.assertEqual([r.parameters.commanded_afo_speed_deg_s for r in p],list(SPEEDS))
        self.assertTrue(all(r.parameters.cycles==3 and r.parameters.afo_id=='AFO-42' for r in p))
        self.assertTrue(all(not r.preset_metadata['preset_modified'] for r in p))
        self.assertEqual(len({r.parameters.file_prefix for r in p}),3)
        self.assertTrue(all(r.context()['mounting_setup_id']=='MOUNT-42' for r in p))
        self.assertTrue(all(r.context()['planned_speed_sequence_deg_s']==list(SPEEDS) for r in p))
        self.assertNotIn('speed_deg_s',self.config['test_presets']['three_speed_afo'])
        self.identity['afo_id']='CHANGED';self.settings['cycles']='1'
        self.assertTrue(all(r.parameters.afo_id=='AFO-42' and r.parameters.cycles==3 for r in p))
    def test_common_modifications_and_baseline_identity(self):
        p=self.plan(cycles='2',minimum_angle_deg='8',maximum_angle_deg='9',acceleration_deg_s2='50')
        self.assertTrue(all(r.parameters.cycles==2 and r.parameters.min_angle_deg==8 and r.parameters.max_angle_deg==9 and r.preset_metadata['preset_modified'] for r in p))
        b=self.plan('three_speed_empty_baseline')
        self.assertTrue(all(r.parameters.test_type=='empty_machine_baseline' and r.parameters.afo_id=='EMPTY_MACHINE_BASELINE' for r in b))
        self.assertNotEqual(b[0].batch_id,p[0].batch_id)
        self.assertTrue(all(not r.context()['automatic_baseline_correction'] for r in b))
    def test_invalid_parameters_and_reserved_loaded_identity(self):
        for changes in ({'cycles':'0'},{'minimum_angle_deg':'16'},{'acceleration_deg_s2':'101'}):
            with self.assertRaises(ValueError):self.plan(**changes)
        self.identity['afo_id']='EMPTY_MACHINE_BASELINE'
        with self.assertRaises(ValueError):self.plan()
    def test_completion_gate_waits_for_finalization_and_rejects_partial_data(self):
        request=self.plan()[0]
        m=dict(protocol_batch=request.context(),run_status='completed',completed_cycles=3,sample_count=10,
               idle_confirmation=dict(confirmed=True,motion_owner=None,monitor_error=None),
               csv_recorder=dict(worker_complete=True,pending_rows=0,error=None,rows_persisted=10),
               rotary_sensor=dict(recording_statistics=dict(delivered_samples=10,invalid_samples=0,recording_error=None)))
        app=SimpleNamespace(run_metadata=m,strain_test_active=False,run_finalized=True,engine=SimpleNamespace(busy=None),
            shutdown_requested=False,motion_coordinator=SimpleNamespace(shutting_down=False,stopping=False,owner=None,motor_idle_confirmed=True),metadata_workers=[])
        self.assertEqual(completed_run(app,request)[0],'success')
        app.run_finalized=False;self.assertEqual(completed_run(app,request)[0],'failure');app.run_finalized=True
        app.cleanup_thread=SimpleNamespace(is_alive=lambda:True)
        self.assertEqual(completed_run(app,request)[0],'pending');app.cleanup_thread=None
        for mutate in [lambda d:d['idle_confirmation'].update(confirmed=False),lambda d:d['csv_recorder'].update(rows_persisted=9),lambda d:d.update(completed_cycles=2),lambda d:d.update(outcome_pending_cleanup=True)]:
            app.run_metadata=copy.deepcopy(m);mutate(app.run_metadata)
            self.assertEqual(completed_run(app,request)[0],'failure')

class ApplicationTests(unittest.TestCase):
    setUp=harness.FullApplicationIntegration.setUp
    tearDown=harness.FullApplicationIntegration.tearDown
    pump=harness.FullApplicationIntegration.pump
    ready=harness.FullApplicationIntegration.ready

    def prepare(self,key='three_speed_afo',cycles=3):
        self.app.system_config['reference'].update(maximum_feedback_capture_ms=1000,feedback_stale_after_ms=2000)
        self.ready();self.pump(lambda:ready_for_next(self.app))
        a=self.app;a.loaded_preset_key=key
        a.afo_id_input.value='AFO-42'
        for field,value in [('cycles_input',str(cycles)),('min_angle_input','10'),('max_angle_input','10'),('acceleration_input','100'),('speed_input','.5')]:getattr(a,field).value=value
        a.show_resizable_confirmation=Mock(return_value=True)
        a.start_rotary_homing=Mock(side_effect=AssertionError('A batch must not re-home between healthy runs'))
        a._tare_worker=Mock(side_effect=AssertionError('Never tare with the AFO fitted'))
        return window_for(a,self.module.ctk,key)
    def pump_window(self,w,condition,seconds=40):
        self.pump(lambda:(w.tick(),condition())[1],seconds)
    def manifest(self,w):return json.loads(w.manifest_path.read_text())
    def test_loaded_success_three_cycles_each_one_confirmation_and_full_files(self):
        w,_=self.prepare();tare=self.app.empty_machine_tare;ref=self.app.reference_manager.record.reference_id
        w.start();self.assertTrue(w.active)
        self.app.set_test_inputs_state('normal');self.assertEqual(self.app.afo_id_input.cget('state'),'disabled')
        self.assertFalse(self.app.start_strain_test())
        self.pump_window(w,lambda:not w.active)
        self.assertEqual(w.batch_status,'completed',w.status.options)
        self.assertEqual(self.app.show_resizable_confirmation.call_count,1)
        self.assertIs(self.app.empty_machine_tare,tare)
        self.assertEqual(self.app.reference_manager.record.reference_id,ref)
        self.app._tare_worker.assert_not_called();self.app.start_rotary_homing.assert_not_called()
        body=self.manifest(w);self.assertEqual(body['total_numbered_cycles'],9)
        self.assertEqual(body['planned_speed_sequence_deg_s'],list(SPEEDS));self.assertEqual(body['additional_inter_run_rest_s'],0)
        self.assertEqual([r['status'] for r in body['runs']],['completed']*3)
        for i,row in enumerate(body['runs']):
            meta=json.loads(Path(row['metadata_file']).read_text())
            self.assertEqual(meta['test_parameters']['commanded_afo_speed_deg_s'],SPEEDS[i])
            self.assertEqual(meta['completed_cycles'],3);self.assertEqual(meta['test_parameters']['afo_id'],'AFO-42')
            self.assertEqual(meta['protocol_batch']['mounting_setup_id'],body['mounting_setup_id'])
            self.assertEqual(meta['protocol_batch_files']['metadata_file'],row['metadata_file'])
            self.assertNotIn('motion_modelling',meta)
            with open(row['strain_file'],newline='') as f:
                reader=csv.reader(f);self.assertEqual(next(reader),CSV_COLUMNS+ROTARY_RUN_COLUMNS+MOTOR_TIMING_COLUMNS);self.assertGreater(sum(1 for _ in reader),0)
            self.assertTrue(Path(row['rotary_file']).exists())
    def test_empty_baseline_is_separate_reserved_identity(self):
        w,_=self.prepare('three_speed_empty_baseline');w.start();self.pump_window(w,lambda:not w.active)
        body=self.manifest(w);self.assertEqual(body['outcome'],'completed',w.status.options)
        self.assertEqual(body['test_type'],'empty_machine_baseline')
        for row in body['runs']:
            m=json.loads(Path(row['metadata_file']).read_text())
            self.assertEqual(m['test_parameters']['afo_id'],'EMPTY_MACHINE_BASELINE')
            self.assertEqual(m['test_parameters']['test_type'],'empty_machine_baseline')
            self.assertFalse(m['baseline_matching']['automatic_subtraction_applied'])
    def test_stop_during_run_skips_remaining(self):
        w,_=self.prepare(cycles=1);w.start()
        self.pump_window(w,lambda:self.app.strain_thread is not None)
        w.stop();self.pump_window(w,lambda:not w.active)
        b=self.manifest(w);self.assertNotEqual(b['outcome'],'completed')
        self.assertEqual([r['status'] for r in b['runs'][1:]],['skipped','skipped'])
        self.assertIsNone(self.app.motion_coordinator.owner)
    def test_stop_between_runs_preserves_first_and_skips_two(self):
        w,_=self.prepare(cycles=1);w.start()
        self.pump_window(w,lambda:len(w.records)==1 and w.save_future is not None)
        w.stop();self.pump_window(w,lambda:not w.active)
        self.assertEqual([r['status'] for r in self.manifest(w)['runs']],['completed','skipped','skipped'])
    def test_initialization_failure_skips_remaining(self):
        w,_=self.prepare(cycles=1)
        self.app._ensure_load_cell=lambda:(_ for _ in ()).throw(OSError('SIMULATED Bridge unavailable'))
        w.start();self.pump_window(w,lambda:not w.active)
        self.assertEqual([r['status'] for r in self.manifest(w)['runs']],['failed','skipped','skipped'])
    def test_recording_failure_cancels_remaining(self):
        w,_=self.prepare(cycles=1);w.start()
        self.pump_window(w,lambda:self.app.csv_recorder is not None)
        self.app.csv_recorder.error=OSError('SIMULATED disk recording failure')
        self.app._persistence_fault('recording_health',self.app.csv_recorder.error)
        self.pump_window(w,lambda:not w.active)
        self.assertEqual([r['status'] for r in self.manifest(w)['runs']],['failed','skipped','skipped'])
    def test_manifest_failure_before_movement(self):
        w,module=self.prepare(cycles=1)
        with patch.object(module,'write_json_atomic',side_effect=OSError('SIMULATED manifest disk full')):
            w.start();self.pump_window(w,lambda:not w.active)
        self.assertIsNone(self.app.strain_thread);self.assertIn('save failed',w.status.options['text'])
    def test_manifest_failure_after_first_run_never_starts_second(self):
        w,module=self.prepare(cycles=1);original=module.write_json_atomic;calls=[]
        def save(path,body):
            calls.append(1)
            if len(calls)>1:raise OSError('SIMULATED next manifest save failure')
            return original(path,body)
        with patch.object(module,'write_json_atomic',side_effect=save):
            w.start();self.pump_window(w,lambda:not w.active)
        self.assertEqual(len(w.records),1)
        self.assertEqual(self.app.run_parameters.commanded_afo_speed_deg_s,.5)
    def test_setup_change_blocks_continuation(self):
        w,_=self.prepare(cycles=1);w.start()
        self.pump_window(w,lambda:len(w.records)==1 and w.save_future is not None)
        self.app.empty_machine_tare=None
        self.pump_window(w,lambda:not w.active)
        self.assertEqual([r['status'] for r in self.manifest(w)['runs']],['completed','skipped','skipped'])
        self.assertIn('tare',w.batch_error.lower())
    def test_application_shutdown_saves_cancelled_manifest_before_exit(self):
        w,_=self.prepare(cycles=1);w.start()
        self.pump_window(w,lambda:len(w.records)==1 and w.save_future is not None)
        self.module.CTkMessagebox=lambda **kw:harness.Field(value='Yes')
        self.app._poll_shutdown_ready=Mock()  # Keep the fake view alive to observe persistence.
        self.app.on_close()
        self.assertTrue(self.app.engine.closing)
        self.pump_window(w,lambda:not w.active)
        body=self.manifest(w)
        self.assertEqual(body['outcome'],'cancelled')
        self.assertEqual([r['status'] for r in body['runs']],['completed','skipped','skipped'])
        self.assertIsNone(self.app.motion_coordinator.owner)

    def test_change_during_confirmation_rejected_before_movement(self):
        w,_=self.prepare(cycles=1)
        def confirm(**kwargs):
            self.app.empty_machine_tare=None
            return True
        self.app.show_resizable_confirmation=confirm
        w.start()
        self.assertFalse(w.active)
        self.assertIsNone(self.app.strain_thread)
        self.assertIn('changed',w.status.options['text'])

    def test_load_sequence_disables_scalar_speed_and_switch_back_preserves_single(self):
        self.ready()
        self.module.CTkMessagebox=lambda **kw:harness.Field(value='Load Preset')
        self.app.protocol_var.value='Three-Speed AFO Protocol'
        self.app.load_selected_preset()
        self.assertEqual(self.app.loaded_preset_key,'three_speed_afo')
        self.assertEqual(float(self.app.speed_input.get()),.5)
        self.assertEqual(self.app.speed_input.cget('state'),'disabled')
        self.app.protocol_var.value='Standard AFO Test'
        self.app.load_selected_preset()
        self.assertEqual(float(self.app.speed_input.get()),5)
        self.assertEqual(self.app.speed_input.cget('state'),'normal')

    def test_close_requests_safe_stop_then_allows_close(self):
        w,_=self.prepare(cycles=1);w.start();w.close()
        self.assertFalse(w.closed)
        self.pump_window(w,lambda:not w.active)
        w.close();self.assertTrue(w.closed)
        self.assertTrue(all(r['status']=='skipped' for r in self.manifest(w)['runs']))

if __name__=='__main__':unittest.main()
