"""Pilot kernel/plan/actual Stop path. No physical devices."""
import unittest,math,json
from pathlib import Path
from unittest.mock import Mock
from east_rotary_feedback import step,ramp,MAX_COMMAND_RATE_TURNS_S
from east_core import load_tester_config
from east_protocol_batch import make_protocol_plan
from east_wide_candidate import inverse,curve
import test_motion_model as harness
from test_three_speed_protocol import window_for
class KernelTests(unittest.TestCase):
    def test_rate_correction_and_input_guards(self):
        row=step(.5,0,1,0,.1,2.3)
        self.assertLessEqual(abs(row['command_target_turns']),MAX_COMMAND_RATE_TURNS_S*.1)
        self.assertLessEqual(abs(row['requested_correction_turns']),.1/2.3)
        self.assertEqual(step(.5,0,1,0,.1,2.3,gain=0)['requested_correction_turns'],0)
        for measured in [6.1,-6.1,float('nan')]:
            with self.assertRaises((ValueError,RuntimeError)):step(0,measured,0,0,.1,2.3)
        with self.assertRaises(RuntimeError):step(0,1.1,0,0,.1,2.3)
    def test_ramp_reaches_target_and_respects_rate(self):
        for a,b in [(0,5),(5,-5),(-5,5),(5,0)]:
            duration=ramp(a,b,.5,0)[1];values=[ramp(a,b,.5,i*.1)[0] for i in range(math.ceil(duration/.1)+2)]
            self.assertAlmostEqual(values[0],a);self.assertAlmostEqual(values[-1],b)
            self.assertLessEqual(max(abs(y-x) for x,y in zip(values,values[1:])),.05000001)
    def test_proportional_correction_reduces_static_plant_offset(self):
        errors=[]
        for gain in [0,.3]:
            ff=inverse(0,1);command=ff
            for _ in range(50):
                measured=curve(command,1)+.08
                command=step(0,measured,ff,command,.1,2.36,gain=gain)['command_target_turns']
            errors.append(abs(curve(command,1)+.08))
        self.assertLess(errors[1],errors[0])

    def test_fixed_two_run_plan(self):
        p=make_protocol_plan(load_tester_config(),'rotary_feedback_pilot',dict(operator='SC',fixture_id='0',calibration_id='0',afo_id='x'),
            dict(cycles=1,minimum_angle_deg=5,maximum_angle_deg=5,acceleration_deg_s2=5),'setup','','p.json','now')
        self.assertEqual([r.feedback_pilot for r in p],[False,True])
        self.assertTrue(all(r.parameters.max_angle_deg==5 and r.parameters.cycles==1 and r.parameters.commanded_afo_speed_deg_s==.5 for r in p))
class IntegrationTests(unittest.TestCase):
    setUp=harness.ApplicationTests.setUp
    tearDown=harness.ApplicationTests.tearDown
    pump=harness.ApplicationTests.pump
    ready=harness.ApplicationTests.ready
    prepare=harness.ApplicationTests.prepare
    def execute_stop(self,start_run,complete=False):
        a=self.prepare();a.loaded_preset_key='rotary_feedback_pilot'
        if complete:self.angle=.075  # Identical desired trajectory despite in-band starting offset.
        a.cycles_input.value='1';a.min_angle_input.value='5';a.max_angle_input.value='5';a.acceleration_input.value='5'
        raw=a.__class__._submit_position.__get__(a)
        def submit(token,target):
            raw(token,target)
            model=a.run_motion_model
            if model is not None:
                row=a.run_metadata['rotary_feedback_pilot']['samples'][-1]
                self.angle=curve(target-model.origin_turns,row['direction'])
        a._submit_position=submit
        w,_=window_for(a,self.module.ctk,'rotary_feedback_pilot')
        w.fields['start_run'].value=str(start_run)
        w.start();self.assertTrue(w.active,w.status.options)
        if complete:
            # Long fake-device traversal: isolate host scheduling from the controller test.
            a.rotary_sensor.config['stale_after_ms']=1000
            self.pump(lambda:(w.tick(),not w.active)[1],100)
            self.assertEqual(w.batch_status,'completed',w.batch_error)
            m=json.loads(Path(w.records[0]['metadata_file']).read_text())
            self.assertEqual(m['completed_cycles'],1)
            self.assertTrue(m['rotary_feedback_pilot']['enabled'])
            self.assertAlmostEqual(m['rotary_feedback_pilot']['samples'][0]['desired_angle_deg'],0.,places=5)
            self.assertGreater(len(m['rotary_feedback_pilot']['samples']),600)
            self.assertIsNone(a.motion_coordinator.owner)
            return
        self.pump(lambda:(w.tick(),len((a.run_metadata or {}).get('rotary_feedback_pilot',{}).get('samples',[]))>=5)[1],8)
        w.stop();self.pump(lambda:(w.tick(),not w.active)[1],10)
        self.assertEqual(w.batch_status,'cancelled',w.batch_error)
        self.assertEqual(len(w.records),1)
        m=json.loads(Path(w.records[0]['metadata_file']).read_text())
        self.assertEqual(m['rotary_feedback_pilot']['enabled'],start_run==2)
        self.assertGreaterEqual(len(m['rotary_feedback_pilot']['samples']),5)
        self.assertIsNone(a.motion_coordinator.owner)
        self.assertTrue(m['csv_recorder']['worker_complete'])

    def test_stop_transport_cancellation_is_not_upgraded_to_fault(self):
        a=self.app
        a._command_rotary_feedback_pilot=Mock(side_effect=RuntimeError('Pending motion cancelled by Stop'))
        a.operator_stop_requested=True;a.test_stop_event.set();a.acquisition_error=None
        with self.assertRaises(self.module.TestStopped):a.command_rotary_feedback_pilot(0,'x',None)
        a.monitor_error='actual feedback fault'
        with self.assertRaisesRegex(RuntimeError,'Pending motion'):a.command_rotary_feedback_pilot(0,'x',None)

    def test_streamed_reference_stop(self):self.execute_stop(1)
    def test_feedback_commands_recorded_and_stop_cancels(self):self.execute_stop(2)

    def test_complete_feedback_pilot_records_all_legs_and_idle(self):self.execute_stop(2,complete=True)
