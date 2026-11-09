"""Offline staged acquisition, measured endpoint control and homing edge tests."""
import json
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock,patch
from east_core import load_tester_config,validate_test_parameters
from east_protocol_batch import make_protocol_plan
from east_rotary_positioning import reach
from east_rotary_homing import home,HomingPolicy
import test_engine_gui_integration as harness
from test_three_speed_protocol import window_for

class PlanTests(unittest.TestCase):
    def plan(self):
        return make_protocol_plan(load_tester_config(),'remaining_motion_data',
            dict(file_prefix='Validation',operator='SC',fixture_id='0',calibration_id='0',afo_id='ignored'),
            dict(cycles='3',minimum_angle_deg='10',maximum_angle_deg='10',acceleration_deg_s2='100',speed_deg_s='.5'),
            'SAME_SETUP','','manifest.json','now')
    def test_all_remaining_runs_have_fixed_methods_and_provenance(self):
        p=self.plan();self.assertEqual(len(p),10)
        self.assertEqual([r.command_model_enabled for r in p],[True]*6+[False]*4)
        self.assertEqual([r.parameters.commanded_afo_speed_deg_s for r in p[:9]],[.5,5,20]*3)
        self.assertTrue(all(r.parameters.cycles==3 and r.parameters.afo_id=='EMPTY_MACHINE_BASELINE' for r in p))
        self.assertTrue(all(r.parameters.max_angle_deg==15 and r.rotary_targeted for r in p[6:9]))
        self.assertTrue(p[-1].stationary_holds)
        self.assertEqual({r.mounting_setup_id for r in p},{'SAME_SETUP'})
        self.assertTrue(all(not r.context()['independent_remount_trials'] for r in p))
    def test_wide_slow_plan_and_standard_sweep_both_fit_the_budget(self):
        c=load_tester_config();p=self.plan()[6].parameters
        self.assertEqual(c['motion']['maximum_motion_timeout_s'],70)
        values=dict(file_prefix='test',operator='SC',fixture_id='0',calibration_id='0',afo_id='EMPTY_MACHINE_BASELINE',test_type='empty_machine_baseline',cycles=3,speed_deg_s=.5,acceleration_deg_s2=100,min_angle_deg=15,max_angle_deg=15)
        self.assertEqual(validate_test_parameters(values,c).max_angle_deg,15)
        self.assertEqual(validate_test_parameters(values,c,segmented_rotary_motion=True).max_angle_deg,15)

class PositioningTests(unittest.TestCase):
    def test_logged_in_band_endpoints_do_not_chase_homing_centre(self):
        for target,low,mean,high in [(-2.,-1.98657934,-1.95716443,-1.93556553),
                                   (-5.,-5.07381927,-5.04971023,-5.02208186)]:
            with self.subTest(target=target):
                def no_move(delta):self.fail('Already qualified endpoint commanded a correction')
                result=reach(target,lambda:mean,no_move,no_move,
                    lambda duration:dict(min_deg=low,mean_deg=mean,max_deg=high,samples=30,duration_s=duration),lambda:None)
                self.assertEqual(result['corrections'],[])
                self.assertEqual(result['centring_mean_limit_deg'],.1)

    def test_wide_endpoints_use_real_measurement_and_slow_correction(self):
        state={'angle':0.};bulk=[];fine=[]
        def check():
            if abs(state['angle'])>15.1:raise RuntimeError('overtravel')
        def move(delta,container):container.append(delta);state['angle']+=delta*1.15
        def observe(duration):
            a=state['angle'];return dict(mean_deg=a,min_deg=a-.001,max_deg=a+.001,samples=30,duration_s=duration)
        for target in [15.,-15.,0.]:
            result=reach(target,lambda:state['angle'],lambda d:move(d,bulk),lambda d:move(d,fine),observe,check)
            observation=result['final_observation']
            self.assertGreaterEqual(observation['min_deg'],-.1)
            self.assertLessEqual(observation['max_deg'],.1)
        self.assertTrue(all(abs(x)<=.5 for x in fine))
        self.assertLessEqual(max(map(abs,bulk)),24.1)
    def test_edge_noise_is_centred_without_widening_acceptance(self):
        state={'a':.09,'moves':0}
        def observe(duration):
            a=state['a'];return dict(mean_deg=a,min_deg=a-.025,max_deg=a+.025,samples=30)
        def move(d):state['a']+=d;state['moves']+=1
        result=home(HomingPolicy(),observe,move,lambda:None)
        self.assertGreater(state['moves'],0)
        self.assertLessEqual(abs(state['a']),.04)
        self.assertLessEqual(result['final_observation']['max_deg'],.1)
    def test_noise_stall_and_stop_are_not_bypassed(self):
        with self.assertRaisesRegex(RuntimeError,'noise'):
            home(HomingPolicy(),lambda d:dict(mean_deg=0,min_deg=-.2,max_deg=.2,samples=20),lambda d:self.fail('noise chasing'),lambda:None)
        with self.assertRaisesRegex(RuntimeError,'Stopped'):
            reach(15,lambda:0,lambda d:self.fail('moved'),lambda d:self.fail('moved'),Mock(),lambda:(_ for _ in ()).throw(RuntimeError('Stopped')))
        with self.assertRaises(ValueError):reach(16,lambda:0,Mock(),Mock(),Mock(),Mock())

class ApplicationTests(unittest.TestCase):
    setUp=harness.FullApplicationIntegration.setUp
    tearDown=harness.FullApplicationIntegration.tearDown
    pump=harness.FullApplicationIntegration.pump
    ready=harness.FullApplicationIntegration.ready
    def test_bulk_feedback_checks_measured_limit_without_load_cell_callback(self):
        self.ready()
        self.app.run_rotary_targeted=True
        self.app.rotary_sensor.measurement=Mock(return_value={"angle_deg":15.11})
        with self.assertRaisesRegex(self.module.RotaryError,"exceeded"):
            self.app._test_feedback()

    def test_full_ten_run_suite_records_methods_holds_and_files(self):
        from east_motion_mapping import ready_for_next
        from east_motion_model import curve
        self.app.system_config['reference'].update(maximum_feedback_capture_ms=1000,feedback_stale_after_ms=2000)
        self.ready();self.pump(lambda:ready_for_next(self.app));a=self.app
        a.loaded_preset_key='remaining_motion_data'
        for key,v in [('cycles_input','3'),('min_angle_input','10'),('max_angle_input','10'),('acceleration_input','100'),('speed_input','.5')]:getattr(a,key).value=v
        a.system_config['motion']['experimental_model']['enabled']=False
        angle=[0.];old_measure=a.rotary_sensor.measurement
        def measure(*args,**kw):
            value=dict(old_measure(*args,**kw));value['angle_deg']=angle[0];return value
        a.rotary_sensor.measurement=measure
        reader=a.rotary_sensor.reader;raw_sample=reader._sample
        reader._sample=lambda ch,v:raw_sample(None,(angle[0]-a.rotary_sensor.calibration.intercept)/a.rotary_sensor.calibration.slope)
        old_submit=a._submit_position;plant={'metadata':None,'origin':0}
        def submit(token,target):
            before=a.get_feedback().position_turns
            if plant['metadata'] is not a.run_metadata:
                plant['metadata']=a.run_metadata;plant['origin']=before-angle[0]/2.055
            old_submit(token,target)
            a.latest_feedback=a._read_adapter_feedback()
            if a.run_motion_model is not None:
                d=a.run_metadata['experimental_motion_model']['segments'][-1]['direction']
                angle[0]=curve(target-a.run_motion_model.origin_turns,d)
            else:angle[0]=(target-plant['origin'])*2.055
        a._submit_position=submit
        def hold(duration,token):
            a.check_rotary_acquisition(token);time.sleep(.004)
            return dict(mean_deg=angle[0],min_deg=angle[0]-.001,max_deg=angle[0]+.001,sd_deg=.001,samples=30,duration_s=duration)
        a.observe_rotary_hold=hold
        a.show_resizable_confirmation=Mock(return_value=True)
        w,_=window_for(a,self.module.ctk,'remaining_motion_data')
        with patch.object(self.module,'wait_for_settle',return_value=None):
            w.start();self.assertTrue(w.active,w.status.options)
            self.pump(lambda:(w.tick(),not w.active)[1],60)
        self.assertEqual(w.batch_status,'completed',w.batch_error)
        body=json.loads(w.manifest_path.read_text());self.assertEqual(len(body['runs']),10)
        self.assertEqual(body['stationary_target_holds'],30)
        self.assertEqual(a.show_resizable_confirmation.call_count,1)
        results=[json.loads(__import__('pathlib').Path(row['metadata_file']).read_text()) for row in body['runs']]
        self.assertTrue(all(r['run_status']=='completed' for r in results))
        self.assertEqual([r['experimental_motion_model']['enabled'] for r in results],[True]*6+[False]*4)
        self.assertEqual(len(results[-1]['rotary_targeted_acquisition']['holds']),30)
        self.assertEqual(results[-1]['rotary_targeted_acquisition']['zero_stability']['duration_s'],30)
        self.assertTrue(all(r['csv_recorder']['rows_persisted']>0 for r in results))

if __name__=='__main__':unittest.main()
