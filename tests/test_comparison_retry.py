"""Reject late bundles; never extend freshness or replay an acquired experiment."""
import unittest,threading
from unittest.mock import Mock,patch
from types import SimpleNamespace
from dataclasses import replace
from east_execution import FeedbackMonitor
from east_reference import FeedbackAcquisitionTimeout
from east_core import load_tester_config
from test_idle_feedback_retry import sample
class RetryTests(unittest.TestCase):
    def run_case(self,events,allowed=True):
        clock=[10.];stop=threading.Event();published=[];faults=[];delay=[]
        initial=sample(10.,.02,current_state=8)
        events=iter(events)
        def read(**kwargs):
            at,item=next(events);clock[0]=at
            if isinstance(item,Exception):raise item
            return item
        adapter=SimpleNamespace(snapshot=Mock(side_effect=read))
        def publish(s):published.append(s);stop.set()
        def fault(s):faults.append(s);stop.set()
        monitor=FeedbackMonitor(lambda:adapter,load_tester_config(),stop,initial,lambda:(8,0),lambda:'strain-test',
            publish,Mock(),Mock(),Mock(),Mock(),fault,allow_motion_retry=lambda:allowed,on_motion_delay=delay.append)
        with patch('east_execution.time.monotonic',side_effect=lambda:clock[0]):monitor.run()
        return monitor,adapter,published,faults,delay
    def late(self,at=10.156,**kwargs):return FeedbackAcquisitionTimeout(sample(at,.156,current_state=8,**kwargs))
    def test_single_rejection_then_fresh_success(self):
        good=sample(10.20,.02,current_state=8)
        m,a,p,f,d=self.run_case([(10.18,self.late()),(10.21,good)])
        self.assertEqual(p,[good]);self.assertFalse(f);self.assertEqual(len(d),1)
        self.assertEqual(len(m.statistics()['comparison_rejected_bundles']),1)
        self.assertIsNone(m.fault_snapshot)
    def test_second_late_bundle_faults(self):
        m,a,p,f,d=self.run_case([(10.18,self.late()),(10.22,self.late(10.21))])
        self.assertFalse(p);self.assertTrue(f);self.assertEqual(a.snapshot.call_count,2)
        self.assertEqual(m.fault_snapshot.captured_monotonic_s,10.21)
    def test_prior_feedback_age_deadline_is_not_extended(self):
        for events in [[(10.26,self.late())],[(10.18,self.late()),(10.29,sample(10.28,.02,current_state=8))]]:
            m,a,p,f,d=self.run_case(events);self.assertFalse(p);self.assertTrue(f)
    def test_noncomparison_and_controller_fault_never_retry(self):
        for allowed,late in [(False,self.late()),(True,self.late(active_errors=1))]:
            m,a,p,f,d=self.run_case([(10.18,late)],allowed)
            self.assertFalse(p);self.assertTrue(f);self.assertEqual(a.snapshot.call_count,1)

import test_motion_model as mh
from test_three_speed_protocol import window_for
from pathlib import Path
import json
class ComparisonRecoveryTests(unittest.TestCase):
    setUp=mh.ApplicationTests.setUp
    tearDown=mh.ApplicationTests.tearDown
    pump=mh.ApplicationTests.pump
    ready=mh.ApplicationTests.ready
    prepare=mh.ApplicationTests.prepare
    def run_recovery_case(self,original=False):
        a=self.prepare();a.loaded_preset_key='model_comparison'
        a.cycles_input.value='3';a.min_angle_input.value='15';a.max_angle_input.value='15';a.acceleration_input.value='100'
        a.start_rotary_homing=a.__class__.start_rotary_homing.__get__(a)
        reader=a.rotary_sensor.reader;raw_sample=reader._sample
        reader._sample=lambda ch,v:raw_sample(None,(self.angle-a.rotary_sensor.calibration.intercept)/a.rotary_sensor.calibration.slope)
        raw_submit=a.__class__._submit_position.__get__(a)
        def submit(token,target):
            before=a.get_feedback().position_turns;angle=self.angle
            raw_submit(token,target)
            if token.owner=='rotary-home':self.angle=angle+(target-before)*2.055
            else:
                model=a.run_motion_model
                if model is None:
                    self.angle=angle+(target-before)*2.055
                    if a.motion_mapping.request.index==1 and a.motion_phase=='returning_to_machine_zero':self.angle=.271
                    return
                seg=a.run_metadata['experimental_motion_model']['segments'][-1]
                from east_motion_model import curve as current
                from east_wide_candidate import curve as candidate
                self.angle=(candidate if a.run_model_variant=='candidate' else current)(target-model.origin_turns,seg['direction'])
                if a.motion_mapping.request.index==1 and a.motion_phase=='returning_to_machine_zero':self.angle=-.704
        a._submit_position=submit
        w,module=window_for(a,self.module.ctk,'model_comparison');builder=module.make_protocol_plan
        def short(*args,**kwargs):
            plan=builder(*args,**kwargs)
            return [replace(r,index=i,total=2,parameters=replace(r.parameters,cycles=1,min_angle_deg=5,max_angle_deg=5,commanded_afo_speed_deg_s=5)) for i,r in enumerate((plan[4:6] if original else plan[5:7]),1)]
        tare=a.empty_machine_tare
        with patch.object(module,'make_protocol_plan',side_effect=short):
            w.start();self.assertTrue(w.active,w.status.options)
            self.pump(lambda:(w.tick(),not w.active)[1],40)
        self.assertEqual(w.batch_status,'completed_with_zero_recovery',w.batch_error)
        self.assertEqual(len(w.records),2)
        m=json.loads(Path(w.records[0]['metadata_file']).read_text())
        self.assertEqual(m['run_status'],'error');self.assertEqual(m['completed_cycles'],1)
        self.assertAlmostEqual(m['zero_return_check']['unassisted_angle_deg'],.271 if original else -.704)
        if not original:self.assertEqual(len(m['experimental_motion_model']['segments']),4)
        self.assertEqual(sum(h.get('kind')=='post_run_zero_recovery' for h in w.reference_history),1)
        self.assertEqual(len(w.preparation_records),2)
        self.assertIs(a.empty_machine_tare,tare)
        self.assertEqual(w.records[1]['run_status'],'completed')

    def test_saved_comparison_zero_miss_recovers_separately_without_repeating(self):self.run_recovery_case()
    def test_original_motor_zero_with_rotary_offset_is_recovered(self):self.run_recovery_case(original=True)
