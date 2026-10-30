"""Offline contract/fault tests. Only fakes can be opened by this test suite."""
import copy
import csv
import json
import os
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "analysis" / "motion_mapping_cli"))
import run_motion_mapping as runner

REPO = Path(os.environ.get('EAST_TEST_REPO', str(Path(__file__).resolve().parents[1]))).resolve()
MODS = runner.load_east(REPO)
core, ref, exe, motor, rotary, runtime = MODS
sys.path.insert(0, str(REPO/'tests'))
from test_odrive_adapter import Device


class Controller:
    def __init__(self, axis):
        self.axis = axis
        self.config = axis.controller.config
        self.value = axis.pos_vel_mapper.pos_rel
    @property
    def input_pos(self):
        return self.value
    @input_pos.setter
    def input_pos(self, value):
        self.value = value
        self.axis.pos_vel_mapper.pos_rel = value
        self.axis.writes.append(value)
        if getattr(self.axis, 'hook', None):
            self.axis.hook(value)


class Sensor:
    fail_samples = False
    fail_finalization = False
    def __init__(self, config, repo):
        self.count = 0
        self.active = False
        self.recording = None
    def connect(self): pass
    def measurement(self, require=False):
        self.count += 1
        if self.active and self.fail_samples:
            raise RuntimeError('synthetic rotary loss')
        now = time.monotonic()
        return dict(valid=True, angle_deg=0, sample_age_s=0, achieved_rate_hz=125,
            sample=(now,'fake',.3977,self.count,'fake-id',int(now*1e9),False), metadata={})
    def start_recording(self, path, started):
        self.active = True
        Path(path).write_text('synthetic_rotary\n0\n')
    def finish_recording(self):
        if self.active and self.fail_finalization:
            raise RuntimeError('synthetic rotary flush failure')
        self.active = False
        return {'delivered_samples':self.count,'invalid_samples':0,'synthetic':True}
    def metadata(self): return {'synthetic': True}
    def shutdown(self): self.active = False


class RunnerTests(unittest.TestCase):
    def setUp(self):
        self.config = core.load_tester_config(REPO/'tester_config.json')
        self.args = runner.arguments(['--repo',str(REPO),'--operator','TEST','--fixture-id','FIX','--cycles','1'])
        Sensor.fail_samples = Sensor.fail_finalization = False
    def test_exact_gui_phase_order_and_profiles(self):
        p=runner.build_plan(self.args,self.config,core)[0][1]
        phases=list(runner.targets(p,self.config))
        self.assertEqual([x[1] for x in phases],['moving_to_initial_max','moving_to_min','moving_to_max','returning_to_machine_zero'])
        self.assertEqual([x[2] for x in phases],[10,-10,10,0])
        self.assertEqual(phases[-1][3:],(2,5))
        self.assertEqual(self.config['motion']['afo_degrees_per_odrive_turn'],2.055)
    def test_plan_and_timeout(self):
        plan=runner.build_plan(self.args,self.config,core)
        self.assertEqual([x[0] for x in plan],['S1','M1','S2','M2','S3','M3'])
        self.assertAlmostEqual(core.motion_timeout_seconds(20,.5,100,self.config),45.005)
    def test_invalid_settings_rejected_before_hardware(self):
        for changes in [dict(range_deg=16),dict(slow_speed=float('nan')),dict(fast_speed=21),dict(cycles=0),dict(acceleration=0)]:
            a=copy.copy(self.args)
            for k,v in changes.items():setattr(a,k,v)
            with self.assertRaises(ValueError): runner.build_plan(a,self.config,core)
    def test_default_preview_never_imports_hardware(self):
        with patch.dict(sys.modules,{'odrive':None,'Phidget22':None}):
            self.assertEqual(runner.main(['--repo',str(REPO)]),0)
    def test_changed_source_refused(self):
        with tempfile.TemporaryDirectory() as d:
            with self.assertRaisesRegex(RuntimeError,'differs'):
                runner.load_east(Path(d))

    def test_windows_crlf_checkout_is_accepted_without_importing_hardware(self):
        manifest=json.loads((REPO/'analysis/motion_mapping_cli/source_manifest.json').read_text())
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            for name in manifest:
                p=root/name
                p.parent.mkdir(parents=True,exist_ok=True)
                p.write_bytes((REPO/name).read_bytes().replace(b'\r\n',b'\n').replace(b'\n',b'\r\n'))
            runner.verify_reviewed_sources(root,manifest)

    def test_real_content_change_still_rejected_in_windows_checkout(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            p=root/'source.py'
            import hashlib
            original=b'conversion = 2.055\n'
            manifest={'source.py':hashlib.sha256(original).hexdigest()}
            p.write_bytes(b'conversion = 2.455\r\n')
            with self.assertRaisesRegex(RuntimeError,'content change'):
                runner.verify_reviewed_sources(root,manifest)

    def test_config_and_indentation_changes_are_not_normalized_away(self):
        import hashlib
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            for original,changed in [(b'{"limit": 15}\n',b'{"limit": 30}\r\n'),
                                     (b'if ready:\n    move()\n',b'if ready:\r\nmove()\r\n')]:
                (root/'source').write_bytes(changed)
                with self.assertRaisesRegex(RuntimeError,'content change'):
                    runner.verify_reviewed_sources(root,{'source':hashlib.sha256(original).hexdigest()})

    def launcher(self):
        import importlib.util
        spec=importlib.util.spec_from_file_location('mapping_play_launcher',REPO/'Testing Scripts'/'run-motion-mapping.py')
        module=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_play_defaults_execute_fixture_zero_without_prompt(self):
        launch=self.launcher()
        with patch.object(runner,'main',return_value=0) as entry, patch.object(launch.time,'sleep'):
            self.assertEqual(launch.main([]),0)
        args=entry.call_args.args[0]
        self.assertEqual(args[args.index('--fixture-id')+1],'0')
        self.assertEqual(args[args.index('--operator')+1],'SC')
        self.assertIn('--execute',args)
        self.assertIn('--confirmed-empty-at-90',args)
        self.assertEqual(Path(args[args.index('--repo')+1]),REPO)

    def test_play_preview_never_enables_execution(self):
        launch=self.launcher()
        with patch.object(runner,'main',return_value=0) as entry, patch.object(launch.time,'sleep') as sleep:
            launch.main(['--preview'])
        self.assertNotIn('--execute',entry.call_args.args[0])
        sleep.assert_not_called()

    def test_explicit_setup_confirmation_does_not_require_stdin(self):
        with tempfile.TemporaryDirectory() as d, patch.object(sys,'stdin',SimpleNamespace(isatty=lambda:False)), patch('builtins.input',side_effect=AssertionError('Unexpected prompt')), patch.object(runner,'Runner') as backend:
            backend.return_value.session={'outcome':'completed'}
            self.assertEqual(runner.main(['--repo',str(REPO),'--execute','--operator','SC','--confirmed-empty-at-90','--output',str(Path(d)/'run')]),0)
            self.assertEqual(backend.call_args.args[0].fixture_id,'0')
            backend.return_value.run.assert_called_once()

    def run_fake(self, fault=None):
        with tempfile.TemporaryDirectory() as d:
            folder=Path(d)/'data';folder.mkdir()
            config=copy.deepcopy(self.config)
            config['reference']['settle_dwell_ms']=1
            config['reference']['post_idle_observation_ms']=1
            plan=runner.build_plan(self.args,config,core)
            device=Device()
            device.axis0.writes=[]
            device.axis0.controller=Controller(device.axis0)
            stop=threading.Event()
            r=runner.Runner(self.args,MODS,config,plan,folder,stop)
            if fault=='cancel':
                # Initial arm holds the current nonzero position; cancel on first actual move.
                device.axis0.hook=lambda target: r.request_stop() if target!=4.5 else None
            if fault=='rotary': Sensor.fail_samples=True
            if fault=='flush': Sensor.fail_finalization=True
            driver=SimpleNamespace(find_any=lambda **kw:device,enums=SimpleNamespace(
                AXIS_STATE_IDLE=1,AXIS_STATE_CLOSED_LOOP_CONTROL=8,
                CONTROL_MODE_POSITION_CONTROL=3,INPUT_MODE_TRAP_TRAJ=5))
            errors=[]
            def work():
                try:r.run()
                except BaseException as e:errors.append(e)
            with patch.dict(sys.modules,{'odrive':driver}),patch.object(rotary,'RotarySensorService',Sensor),patch.object(ref,'runtime_state_directory',lambda:Path(d)/'state'):
                t=threading.Thread(target=work);t.start();t.join(10)
                self.assertFalse(t.is_alive(),'Fake runner did not retire')
            self.assertEqual(device.axis0.current_state,1)
            self.assertFalse(r.lock.held)
            session=json.loads((folder/'session.json').read_text())
            if fault:
                self.assertTrue(errors)
                self.assertNotEqual(session['outcome'],'completed')
                self.assertLessEqual(len(session['runs']),1)
                if fault=='cancel':
                    self.assertNotIn(4.5-10/2.055,device.axis0.writes) # no subsequent endpoint/return
            else:
                self.assertEqual(errors,[])
                self.assertEqual(session['outcome'],'completed')
                self.assertEqual(len(session['runs']),6)
                self.assertTrue(all(x['outcome']=='completed' for x in session['runs']))
                self.assertEqual(device.axis0.pos_vel_mapper.pos_rel,4.5)
                for label,_ in plan:
                    with (folder/f'{label}_motor_samples.csv').open() as h:
                        rows=list(csv.DictReader(h))
                    self.assertTrue(rows)
                    self.assertTrue(all(float(x['zero_motor_turns'])==4.5 for x in rows))
            return session
    def test_complete_six_run_series_nonzero_reference(self): self.run_fake()
    def test_cancel_prevents_next_target_and_preserves_abort(self): self.run_fake('cancel')
    def test_sensor_loss_stops_series(self): self.run_fake('rotary')
    def test_flush_failure_never_called_completed(self): self.run_fake('flush')
    def test_idle_unconfirmed_uses_shared_result_detail(self):
        with tempfile.TemporaryDirectory() as d:
            r=runner.Runner(self.args,MODS,self.config,[],Path(d),threading.Event())
            r.adapter=SimpleNamespace(request_idle=lambda _:motor.IdleResult(True,False,0,None,'test idle failure'))
            with self.assertRaisesRegex(RuntimeError,'test idle failure'):r.idle()

if __name__=='__main__':unittest.main()
