"""Offline inverse-model and fake-device integration checks. Never discovers hardware."""
import math
import unittest
from unittest.mock import Mock
from east_motion_model import RunModel, curve, inverse, enabled, validate_range, MODEL_ID
import test_engine_gui_integration as h
from east_motion_mapping import ready_for_next

class ModelTests(unittest.TestCase):
    def test_both_branches_round_trip_and_monotonicity(self):
        for d in (-1,1):
            last=-math.inf
            for i in range(101):
                x=-4.38+8.76*i/100
                y=curve(x,d)
                self.assertGreater(y,last);last=y
                self.assertAlmostEqual(inverse(y,d),x,places=10)
    def test_anchor_translates_encoder_origin_without_erasing_direction(self):
        m=RunModel.anchored(100,.04)
        self.assertAlmostEqual(curve(100-m.origin_turns,1),.04)
        self.assertNotAlmostEqual(m.target(0,1),m.target(0,-1),places=2)
        for d in (-1,1):
            self.assertAlmostEqual(curve(m.target(9,d)-m.origin_turns,d),9)
            self.assertAlmostEqual(curve(m.target(-9,d)-m.origin_turns,d),-9)
    def test_no_extrapolation_nan_or_bad_direction(self):
        for x in (math.nan,math.inf,4.39,-4.39):
            with self.assertRaises(ValueError):curve(x,1)
        for y in (10,15,math.nan,math.inf):
            with self.assertRaises(ValueError):inverse(y,1)
        for d in (0,2):
            with self.assertRaises(ValueError):inverse(0,d)
        with self.assertRaises(ValueError):RunModel.anchored(0,.11)
        with self.assertRaises(ValueError):RunModel.anchored(math.nan,0)
        with self.assertRaises(ValueError):RunModel.anchored(0,0).target(9.1,1)
        with self.assertRaises(ValueError):validate_range(10,10)
        with self.assertRaises(ValueError):enabled({'motion':{'experimental_model':{'enabled':'false'}}})
        with self.assertRaises(ValueError):enabled({'motion':{'experimental_model':{'enabled':True,'model_id':'unknown'}}})

class ApplicationTests(unittest.TestCase):
    setUp=h.FullApplicationIntegration.setUp
    tearDown=h.FullApplicationIntegration.tearDown
    pump=h.FullApplicationIntegration.pump
    ready=h.FullApplicationIntegration.ready

    def prepare(self):
        self.ready();self.pump(lambda:ready_for_next(self.app))
        a=self.app
        a.system_config['reference'].update(maximum_feedback_capture_ms=1000,feedback_stale_after_ms=2000)
        a.system_config['motion']['experimental_model']['enabled']=True
        self.angle=0.;original=a.rotary_sensor.measurement
        def measurement(*args,**kwargs):
            v=dict(original(*args,**kwargs));v['angle_deg']=self.angle;return v
        a.rotary_sensor.measurement=measurement
        original_submit=a._submit_position
        self.commands=[]
        def submit(token,target):
            original_submit(token,target)
            model=getattr(a,'run_motion_model',None)
            if model is not None:
                segment=a.run_metadata['experimental_motion_model']['segments'][-1]
                self.angle=curve(target-model.origin_turns,segment['direction'])
                self.commands.append((target,self.angle,segment['phase']))
        a._submit_position=submit
        a.start_rotary_homing=Mock(side_effect=AssertionError('No hidden re-home'))
        a._tare_worker=Mock(side_effect=AssertionError('No hidden re-tare'))
        a.min_angle_input.value='9';a.max_angle_input.value='9'
        return a

    def test_complete_normal_run_targets_and_zero_sensor_check(self):
        a=self.prepare();tare=a.empty_machine_tare;reference=a.reference_manager.record.reference_id
        self.assertTrue(a.start_strain_test())
        self.pump(lambda:a.run_finalized and a.run_metadata is not None and not a.strain_test_active,15)
        self.assertEqual(a.run_metadata['run_status'],'completed',a.run_metadata.get('error'))
        model=a.run_metadata['experimental_motion_model']
        self.assertEqual(model['model_id'],MODEL_ID);self.assertTrue(model['enabled'])
        self.assertEqual([round(v[1],6) for v in self.commands],[9,-9,9,0])
        self.assertEqual(len(model['segments']),4)
        self.assertIs(a.empty_machine_tare,tare);self.assertEqual(a.reference_manager.record.reference_id,reference)
        a.start_rotary_homing.assert_not_called();a._tare_worker.assert_not_called()
        self.assertTrue(a.rotary_zero_ready())
        self.assertNotAlmostEqual(self.commands[-1][0],a.reference_manager.require_verified().neutral_position_turns,places=2)
        self.assertNotAlmostEqual(abs(self.commands[0][0]-model['initial_alignment']['anchor_motor_turns']),9/2.055,places=2)

    def test_model_default_load_and_range_block_before_motion(self):
        a=self.prepare();self.module.CTkMessagebox=lambda **kw:h.Field(value='Load Preset')
        a.protocol_var.value='Three-Speed AFO Protocol';a.load_selected_preset()
        self.assertEqual(float(a.min_angle_input.get()),9)
        self.assertEqual(float(a.max_angle_input.get()),9)
        a.loaded_preset_key='custom';a.min_angle_input.value='10'
        self.assertFalse(a.start_strain_test())
        self.assertIsNone(a.strain_thread)

    def test_return_miss_fails_normal_record_and_prevents_success(self):
        a=self.prepare();original=a._submit_position
        def wrong_return(token,target):
            original(token,target)
            if a.motion_phase=='returning_to_machine_zero':self.angle=.25
        a._submit_position=wrong_return
        a.start_strain_test()
        self.pump(lambda:a.run_finalized and a.run_metadata is not None and not a.strain_test_active,15)
        self.assertEqual(a.run_metadata['run_status'],'error')
        self.assertIn('missed rotary zero',a.run_metadata['error'])

    def test_unknown_initial_zero_branch_refines_against_fixed_plant(self):
        a=self.prepare()
        zero=a.latest_feedback.position_turns
        true_origin=zero-inverse(0,-1)  # Physical zero approached from decreasing direction.
        raw_submit=a._submit_position
        def physical_submit(token,target):
            raw_submit(token,target)
            segment=a.run_metadata['experimental_motion_model']['segments'][-1]
            self.angle=curve(target-true_origin,segment['direction'])
        a._submit_position=physical_submit
        a.start_strain_test()
        self.pump(lambda:a.run_finalized and a.run_metadata is not None and not a.strain_test_active,15)
        self.assertEqual(a.run_metadata['run_status'],'completed',a.run_metadata.get('error'))
        self.assertAlmostEqual(a.run_motion_model.origin_turns,true_origin,places=8)
        self.assertAlmostEqual(self.angle,0,places=8)
        self.assertTrue(a.run_metadata['experimental_motion_model']['numbered_cycle_origin_locked_after_initial_positioning'])

    def test_three_speed_model_batch_uses_normal_recorders(self):
        from test_three_speed_protocol import window_for
        a=self.prepare();a.loaded_preset_key='three_speed_empty_baseline';a.acceleration_input.value='100'
        w,_=window_for(a,self.module.ctk,'three_speed_empty_baseline')
        w.start()
        self.pump(lambda:(w.tick(),not w.active)[1],20)
        self.assertEqual(w.batch_status,'completed',w.batch_error)
        self.assertEqual(len(w.records),3)
        self.assertEqual(a.run_metadata['experimental_motion_model']['segments'][0]['nominal_speed_deg_s'],20)
        self.assertTrue(a.run_metadata['experimental_motion_model']['segments'][0]['outside_training_speed_range'])

    def test_model_motor_profile_preserves_legacy_global_ceilings(self):
        a=self.prepare()
        a.engine.submit("simulated profile bounds",lambda:a.configure_trajectory(20,100,turns_per_degree=.7)).result(2)
        self.assertAlmostEqual(a.commanded_odrive_velocity,20/2.055)
        self.assertAlmostEqual(a.commanded_odrive_acceleration,100/2.055)

    def test_stop_blocks_model_command(self):
        a=self.prepare();a.run_motion_model=RunModel.anchored(0,0);a.test_stop_event.set()
        a.motion_coordinator.assert_active=Mock()
        with self.assertRaises(self.module.TestStopped):a.command_test_angle(9,0,'moving_to_max',object())
        self.assertEqual(self.commands,[])

if __name__=='__main__':unittest.main()
