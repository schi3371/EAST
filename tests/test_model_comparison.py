"""Frozen candidate, explicit plan and GUI admission. No physical devices."""
import unittest
from unittest.mock import Mock
from pathlib import Path
import json
from east_core import load_tester_config
from east_protocol_batch import make_protocol_plan
from east_wide_candidate import WideRunModel,curve,inverse,MODEL_ID
import test_motion_model as harness
from test_three_speed_protocol import window_for
class CandidateTests(unittest.TestCase):
    def test_inverse_monotonic_and_origin_alignment(self):
        for d in [-1,1]:
            values=[curve(x/10,d) for x in range(-80,81)]
            self.assertTrue(all(a<b for a,b in zip(values,values[1:])))
            for angle in [-15,-10,0,10,15]:self.assertAlmostEqual(curve(inverse(angle,d),d),angle,places=9)
        m=WideRunModel.observed(3.,10.,1)
        self.assertAlmostEqual(m.target(10,1),3.)
        self.assertEqual(m.metadata()['model_id'],MODEL_ID)
        with self.assertRaises(ValueError):m.target(15.1,1)
    def plan(self):
        return make_protocol_plan(load_tester_config(),'model_comparison',
            dict(file_prefix='x',operator='SC',fixture_id='0',calibration_id='0',afo_id='ignored'),
            dict(cycles='3',minimum_angle_deg='15',maximum_angle_deg='15',acceleration_deg_s2='100',speed_deg_s='.5'),
            'SETUP','','out.json','now')
    def test_balanced_frozen_plan(self):
        p=self.plan();self.assertEqual(len(p),22)
        self.assertEqual([r.parameters.max_angle_deg for r in p[:4]],[5,10,5,10])
        self.assertTrue(all(r.parameters.cycles==1 for r in p[:4]))
        self.assertTrue(all(r.parameters.cycles==3 and not r.rotary_targeted and r.command_model_enabled==(r.model_variant!='original') for r in p[4:]))
        for speed in [.5,5,20]:
            for model in ['original','candidate','current']:
                self.assertEqual(sum(r.parameters.commanded_afo_speed_deg_s==speed and r.model_variant==model for r in p[4:]),2)
        self.assertEqual([r.model_variant for r in p[4:13]],['original','current','candidate']*3)
        self.assertEqual([r.model_variant for r in p[13:]],['candidate','current','original']*3)
class GuiTests(unittest.TestCase):
    setUp=harness.ApplicationTests.setUp
    tearDown=harness.ApplicationTests.tearDown
    pump=harness.ApplicationTests.pump
    ready=harness.ApplicationTests.ready
    prepare=harness.ApplicationTests.prepare
    def test_comparison_records_angle_beyond_nominal_target_without_fault(self):
        a=self.app
        a.run_segmented_comparison=True;a.run_rotary_targeted=False
        a.rotary_sensor.measurement=Mock(return_value={'angle_deg':15.5})
        evidence=object();a.get_feedback=Mock(return_value=evidence)
        self.assertIs(a._test_feedback(),evidence)
        # Sensor validity remains authoritative; no extrapolated angle is silently accepted.
        from east_rotary_sensor import RotaryError
        a.rotary_sensor.measurement=Mock(side_effect=RotaryError('outside_checked_angle_range'))
        with self.assertRaises(RotaryError):a._test_feedback()

    def test_plan_admitted_and_candidate_runs_through_normal_recorders(self):
        a=self.prepare();a.loaded_preset_key='model_comparison'
        a.cycles_input.value='3';a.min_angle_input.value='15';a.max_angle_input.value='15';a.acceleration_input.value='100'
        # Fake plant deliberately follows the active frozen model. This checks
        # wiring/recording, not predictive accuracy against independent mechanics.
        raw=a.__class__._submit_position.__get__(a)
        def submit(token,target):
            raw(token,target)
            model=a.run_motion_model
            if model is None:
                self.angle=(target-a.reference_manager.require_verified().neutral_position_turns)*2.055
            else:
                seg=a.run_metadata['experimental_motion_model']['segments'][-1]
                from east_motion_model import curve as current_curve
                self.angle=(curve if a.run_model_variant=='candidate' else current_curve)(target-model.origin_turns,seg['direction'])
        a._submit_position=submit
        w,_=window_for(a,self.module.ctk,'model_comparison')
        w.start();self.assertTrue(w.active,w.status.options)
        self.pump(lambda:(w.tick(),not w.active)[1],120)
        self.assertEqual(w.batch_status,'completed',w.batch_error)
        self.assertEqual(len(w.records),22)
        self.assertTrue(w.records,w.batch_error)
        m=json.loads(Path(w.records[2]['metadata_file']).read_text())
        self.assertEqual(m['run_status'],'completed',m.get('error'))
        self.assertEqual(m['experimental_motion_model']['model_id'],MODEL_ID)
        self.assertFalse(m['rotary_targeted_acquisition']['enabled'])
        self.assertEqual(sum(r['outcome']=='success' for r in w.records),22)
        body=json.loads(w.manifest_path.read_text())
        self.assertEqual(body['stationary_target_holds'],0)
        self.assertEqual(body['total_numbered_cycles'],58)
        self.assertEqual([r.context()['model_variant'] for r in w.requests[4:13]],['original','current','candidate']*3)
        self.assertTrue(any(json.loads(Path(r['metadata_file']).read_text())['experimental_motion_model']['segments'][1].get('submoves',0)>1 for r in w.records[4:] if json.loads(Path(r['metadata_file']).read_text())['experimental_motion_model'].get('enabled')))
