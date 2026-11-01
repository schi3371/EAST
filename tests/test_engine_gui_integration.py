"""Full MyInterface import, fake view/devices, real execution/disk services.

No method extraction and no hardware libraries are required. The view is a
headless fake: Windows display scaling remains a physical bench check.
"""
import importlib.util
import os
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch, Mock

from east_rotary_sensor import RotaryReader
from test_odrive_adapter import Device

class Field:
    def __init__(self,*args,value='',**kwargs):self.value=value;self.options={'state':'normal',**kwargs}
    def get(self):return self.value
    def set(self,value):self.value=value
    def delete(self,*args):self.value=''
    def insert(self,*args):self.value=str(args[-1])
    def configure(self,**kwargs):self.options.update(kwargs)
    def cget(self,key):return self.options.get(key)
    def __getattr__(self,name):
        if name in ('winfo_x','winfo_y'):return lambda:0
        if name in ('winfo_width','winfo_screenwidth'):return lambda:900
        if name in ('winfo_height','winfo_screenheight'):return lambda:700
        return lambda *args,**kwargs:None
class Controller:
    def __init__(self,axis,original):self.axis=axis;self.config=original.config;self._input=axis.pos_vel_mapper.pos_rel
    @property
    def input_pos(self):return self._input
    @input_pos.setter
    def input_pos(self,value):self._input=value;self.axis.pos_vel_mapper.pos_rel=value;self.axis.pos_vel_mapper.vel=0
class Bridge:
    def getVoltageRatio(self):return .00001
    def getDeviceSerialNumber(self):return 513296
    def getDataInterval(self):return 8
    def getAttached(self):return True
    def close(self):pass

def load_gui():
    ctk=ModuleType('customtkinter')
    for name in ['CTkFrame','CTkToplevel','CTkLabel','CTkCheckBox','CTkButton','StringVar','BooleanVar']:setattr(ctk,name,Field)
    ctk.END='end';ctk.set_default_color_theme=lambda *args:None;ctk.set_appearance_mode=lambda *args:None
    message=ModuleType('CTkMessagebox');message.CTkMessagebox=Field
    pil=ModuleType('PIL');pil.Image=SimpleNamespace()
    odrive=ModuleType('odrive');odrive.find_any=lambda *args,**kwargs:(_ for _ in ()).throw(AssertionError('Physical discovery prohibited'))
    enums=ModuleType('odrive.enums')
    for name,value in [('AXIS_STATE_IDLE',1),('AXIS_STATE_CLOSED_LOOP_CONTROL',8),('CONTROL_MODE_POSITION_CONTROL',3),('INPUT_MODE_TRAP_TRAJ',5)]:setattr(enums,name,value)
    phidget=ModuleType('Phidget22.Devices.VoltageRatioInput');phidget.VoltageRatioInput=lambda:(_ for _ in ()).throw(AssertionError('Physical Phidget opening prohibited'))
    tk=ModuleType('tkinter');tk.TclError=type('TclError',(Exception,),{});tk.TkVersion=8.6
    modules={'tkinter':tk,'customtkinter':ctk,'CTkMessagebox':message,'PIL':pil,'odrive':odrive,'odrive.enums':enums,
             'Phidget22':ModuleType('Phidget22'),'Phidget22.Devices':ModuleType('Phidget22.Devices'),
             'Phidget22.Devices.VoltageRatioInput':phidget}
    # OutputTabs is irrelevant to the fake view; no real Tk object is created.
    tabs=ModuleType('east_gui_tabs');tabs.OutputTabs=Field;modules['east_gui_tabs']=tabs
    path=Path(__file__).resolve().parents[1]/'Ortho-Sim.py'
    spec=importlib.util.spec_from_file_location('east_headless_full_gui',path)
    module=importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules,modules):spec.loader.exec_module(module)
    return module

class FullApplicationIntegration(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory();self.addCleanup(self.folder.cleanup)
        self.environment=patch.dict(os.environ,{'EAST_STATE_DIR':str(Path(self.folder.name)/'state')});self.environment.start();self.addCleanup(self.environment.stop)
        self.module=load_gui()
        def view(app):
            names=('file_name_input cycles_input speed_input acceleration_input min_angle_input max_angle_input '
                'operator_input afo_id_input fixture_id_input calibration_id_input protocol_menu '
                'load_preset_button reset_fields_button clear_session_tare_button reference_action_button '
                'manual_mode_toggle left_arrow right_arrow step_angle_input manual_step_units_menu '
                'mode_toggle neutral_button tare_button reference_status_label tare_status_label status_label '
                'terminal isolated_exit_button output_tabs homing_acknowledged homing_condition homing_live_label').split()
            for name in names:setattr(app,name,Field())
            app.buttons=[Field(),Field()];app.continuous_mode=Field(value=False)
            app.output_tabs.value='Session Terminal'
        with patch.object(self.module.MyInterface,'setup_ui',view):
            self.app=self.module.MyInterface(Field())
        self.app.system_config['motion']['experimental_model']['enabled']=False  # Legacy regression fixture; sensor is fixed at zero.
        self.app.create_plot_window=lambda:None;self.app.update_plot=lambda:None
        self.app.system_config['logging']['output_directory']=str(Path(self.folder.name)/'logs')
        self.app.system_config['load_cell']['tare_samples']=2
        self.device=Device();self.device.axis0.controller=Controller(self.device.axis0,self.device.axis0.controller)
        self.module.MyInterface.find_odrive_with_timeout=staticmethod(lambda *args:self.device)
        self.app.connect_rotary_sensor=lambda:None
        self.bridge=Bridge();self.app._ensure_load_cell=lambda:self.bridge;self.app.voltage_ratio_input=self.bridge
        for name,value in {'operator_input':'SIMULATED','fixture_id_input':'SIM-FIXTURE','calibration_id_input':'SIM-LOAD',
            'file_name_input':'SIMULATED','afo_id_input':'EMPTY','cycles_input':'1','speed_input':'1',
            'acceleration_input':'5','min_angle_input':'5','max_angle_input':'5'}.items():getattr(self.app,name).value=value
        self.app.homing_condition.value='Empty fixture'
        self.app.homing_acknowledged.value=True
        self.sensorstop=threading.Event();self.sensorthread=None

    def tearDown(self):
        self.sensorstop.set()
        if self.sensorthread:self.sensorthread.join(1)
        self.app.test_stop_event.set()
        for name in ['strain_thread','data_collection_thread','cleanup_thread','stop_worker']:
            thread=getattr(self.app,name,None)
            if thread:thread.join(3)
        self.app._stop_feedback_monitor()
        if self.app.transport_owner:self.assertTrue(self.app.transport_owner.close(1))
        self.assertTrue(self.app.engine.close(1));self.app.process_lock.release()

    def pump(self,condition,seconds=8):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            # Simulated driver batches, as host callbacks can arrive together.
            # Times are real host arrivals; tied callbacks are never separated.
            reader = self.app.rotary_sensor.reader
            if reader is not None and reader.accepting and not self.sensorstop.is_set():
                reader._sample(None,.39772982090527514)
                reader._sample(None,.39772982090527514)
            self.app._drain_ui_queues()
            if condition():return
            time.sleep(.005)
        self.fail('UI/service operation timed out: '+self.app.terminal.value)

    def ready(self):
        self.app.connect_system()
        self.pump(lambda:self.app.engine.busy is None and self.app.monitor_thread is not None)
        reader=RotaryReader(750256,'Voltage ratio');reader.accepting=True
        reader.metadata={'interval_ms':8,'requested_interval_ms':8,'hub_serial':750256,'input_mode':'Voltage ratio'}
        self.app.rotary_sensor.reader=reader
        def events():
            while not self.sensorstop.is_set():reader._sample(None,.39772982090527514);self.sensorstop.wait(.004)
        self.sensorthread=threading.Thread(target=events,daemon=True);self.sensorthread.start()
        self.pump(lambda:self.app.rotary_sensor.measurement()['valid'])
        def simulated_zero():
            value=self.app.rotary_sensor.measurement(require=True)
            angle=value['angle_deg']
            self.app.reference_manager.establish_from_rotary_zero(
                self.app.get_feedback(),self.app.hardware_fingerprint,'SIMULATED','SIM-FIXTURE',
                self.app.rotary_sensor.calibration.sha256,reader.connection_id,
                dict(mean_deg=angle,min_deg=angle,max_deg=angle,samples=60,duration_s=.5))
            self.app.unsafe_exit_recovered_in_session=True
        self.app.engine.submit('SIMULATED rotary zero evidence',simulated_zero).result(2)
        self.app.engine.submit('simulated empty tare',lambda:self.app._tare_worker('SIMULATED','SIM-FIXTURE','SIM-LOAD')).result(2)
        self.app.show_resizable_confirmation=lambda **kwargs:True

    def test_successful_complete_run_through_real_services(self):
        self.ready();self.app.start_strain_test()
        self.pump(lambda:self.app.run_finalized and self.app.run_metadata is not None and not self.app.strain_test_active,10)
        self.assertEqual(self.app.run_metadata['run_status'],'completed', self.app.run_metadata.get('error'))
        self.assertIsNone(self.app.motion_coordinator.owner)
        self.assertFalse(self.app.csv_recorder.thread.is_alive())
        self.assertGreater(self.app.csv_recorder.written_rows,0)
        self.assertTrue(Path(self.app.metadata_file_name).exists())

    def test_initialization_failure_before_metadata_restores_editing_but_blocks_motion(self):
        self.ready()
        self.app._ensure_load_cell=lambda:(_ for _ in ()).throw(OSError('SIMULATED unavailable Bridge'))
        self.app.start_strain_test()
        self.pump(lambda:not self.app.strain_test_active and self.app.cleanup_thread is not None and not self.app.cleanup_thread.is_alive())
        self.assertIsNone(self.app.metadata_file_name)
        self.assertTrue(self.app.run_finalized)
        self.pump(lambda:self.app.speed_input.cget('state')=='normal')
        self.assertFalse(self.app.reference_manager.verified)
        self.assertEqual(self.app.buttons[1].cget('state'),'disabled')

    def test_stopped_monitor_recovery_restarts_monitor_and_requires_reference(self):
        self.ready();self.app._stop_feedback_monitor()
        self.assertTrue(self.app._latch_feedback_fault(
            'SIMULATED failure', self.app.odrive_connection_generation, self.app.odrive_adapter))
        self.assertFalse(self.app.engine.evidence['feedback_health']['healthy'])
        self.app.connect_system()
        self.pump(lambda:self.app.engine.busy is None and self.app.monitor_thread is not None and self.app.monitor_thread.is_alive())
        self.assertIsNone(self.app.monitor_error)
        self.assertIsNone(self.app.feedback_fault_reason)
        self.assertTrue(self.app.engine.evidence['feedback_health']['healthy'])
        self.assertTrue(self.app.engine.evidence['feedback_recovery']['validated'])
        self.assertFalse(self.app.reference_manager.verified)
        self.assertIsNotNone(self.app.empty_machine_tare)
        self.assertTrue(self.app.motion_coordinator.motor_idle_confirmed)
        self.assertFalse(self.app.engine.evidence['reference_continuity']['proven'])

    def test_lost_transport_is_retired_before_replacement_connection_and_frame_not_restored(self):
        self.ready()
        self.app._stop_feedback_monitor()
        old_owner=self.app.transport_owner
        old_generation=self.app.odrive_connection_generation
        class LostAxis:
            @property
            def requested_state(self):raise OSError('SIMULATED Fibre object lost')
            @requested_state.setter
            def requested_state(self,value):raise OSError('SIMULATED Fibre object lost')
        self.app.odrive_adapter.axis=LostAxis()
        replacement=Device();replacement.axis0.pos_vel_mapper.pos_rel=50.0
        replacement.axis0.controller=Controller(replacement.axis0,replacement.axis0.controller)
        self.module.MyInterface.find_odrive_with_timeout=staticmethod(lambda *args:replacement)
        self.app.monitor_error='SIMULATED USB loss'
        self.app.connect_system()
        self.pump(lambda:self.app.engine.busy is None and self.app.monitor_thread is not None and self.app.monitor_thread.is_alive())
        self.assertFalse(old_owner.thread.is_alive())
        self.assertGreater(self.app.odrive_connection_generation,old_generation)
        self.assertIs(self.app.odrive_adapter.device,replacement)
        self.assertFalse(self.app.reference_manager.verified)
        self.assertIsNone(self.app.reference_manager.mapping)
        self.assertIsNotNone(self.app.empty_machine_tare)
        self.assertFalse(self.app.strain_test_active)

    def test_blocked_transport_quit_shows_direct_isolation_action_and_dirty_exit(self):
        self.app.connect_system()
        self.pump(lambda:self.app.engine.busy is None and self.app.monitor_thread is not None)
        entered,release=threading.Event(),threading.Event()
        blocked=self.app.transport_owner.submit('SIMULATED blocked Fibre',lambda:(entered.set(),release.wait(2)))
        self.assertTrue(entered.wait(1))
        self.module.CTkMessagebox=lambda **kwargs:Field(value='Yes')
        self.app._force_process_exit=Mock()
        self.app._schedule_isolated_exit_deadline=lambda:None
        try:
            started=time.monotonic();self.app.on_close()
            self.assertLess(time.monotonic()-started,.1)
            self.assertTrue(self.app.motion_coordinator.shutting_down)
            self.app.shutdown_started_monotonic=time.monotonic()-4
            self.app._poll_shutdown_ready()
            dialog=self.app.shutdown_dialog
            self.assertIsNotNone(dialog)
            self.assertEqual(dialog.exit.cget('state'),'disabled')
            dialog.estop.set(True);dialog._confirm()
            self.assertFalse(self.app.isolated_exit_requested)
            dialog.isolation.set(True);dialog._confirm()
            self.assertTrue(self.app.isolated_exit_requested)
            self.pump(lambda:self.app._force_process_exit.called)
            self.assertTrue(self.app.state_store.unsafe_exit_present())
        finally:
            release.set();blocked.result(1)

    def test_idle_observation_survives_late_followup_read_and_separate_checkpoint(self):
        self.ready()
        publication_armed=threading.Event();publication_entered=threading.Event()
        release_publication=threading.Event();publication_finished=threading.Event()
        original=self.app._record_feedback_observation
        def gated_publication(*args):
            if publication_armed.is_set() and not publication_finished.is_set():
                publication_entered.set()
                self.assertTrue(release_publication.wait(1))
            accepted=original(*args)
            if publication_entered.is_set():publication_finished.set()
            return accepted
        self.app._record_feedback_observation=gated_publication
        publication_armed.set()
        self.assertTrue(publication_entered.wait(1))
        try:
            self.app._read_adapter_feedback=lambda **kwargs:(_ for _ in ()).throw(RuntimeError('SIMULATED late diagnostic'))
            result=self.app.engine.submit('idle observation',lambda:self.app.safe_idle_motor('SIMULATED Stop')).result(3)
            release_publication.set()
            self.assertTrue(publication_finished.wait(1))
            self.assertTrue(result.confirmed)
            self.assertTrue(self.app.motion_coordinator.motor_idle_confirmed)
            self.assertIn('SIMULATED late diagnostic',self.app.monitor_error)
            self.assertFalse(self.app.engine.evidence['feedback_health']['healthy'])
            self.assertIn('SIMULATED late diagnostic',self.app.engine.evidence['feedback_health']['reason'])
            self.assertGreaterEqual(
                self.app.engine.evidence['feedback_latest']['recorded_at'],result.observed_monotonic_s)
            self.assertTrue(self.app.engine.evidence['persistence_health']['healthy'])
            self.assertEqual(self.app.last_idle_observation['observed_at'],result.observed_at)
            self.assertFalse(self.app.reference_manager.verified)
            self.assertEqual(self.app.reference_manager.confidence.value,'fault')
            self.assertFalse(self.app._transport_motion_allowed())
            command=Mock();self.app.odrive_adapter.request_closed_loop_holding_current=command
            with self.assertRaisesRegex(self.module.FeedbackError,'Recover the stopped feedback monitor'):
                self.app.enter_closed_loop(None)
            command.assert_not_called()
        finally:
            release_publication.set()

    def test_failed_feedback_recovery_preserves_fault_latch(self):
        self.ready();self.app._stop_feedback_monitor()
        self.assertTrue(self.app._latch_feedback_fault(
            'SIMULATED feedback fault', self.app.odrive_connection_generation, self.app.odrive_adapter))
        self.app.odrive_adapter.request_idle=Mock(side_effect=RuntimeError('SIMULATED recovery read failed'))
        self.module.MyInterface.find_odrive_with_timeout=staticmethod(lambda *args:None)
        self.app.connect_system()
        self.pump(lambda:self.app.engine.busy is None)
        self.assertEqual(self.app.feedback_fault_reason,'SIMULATED feedback fault')
        self.assertFalse(self.app.engine.evidence['feedback_health']['healthy'])
        self.assertFalse(self.app.reference_manager.verified)

    def test_retired_connection_cannot_publish_feedback(self):
        self.ready()
        old_adapter=self.app.odrive_adapter;old_generation=self.app.odrive_connection_generation
        current=self.app.get_feedback();before=dict(self.app.engine.evidence['feedback_latest'])
        self.app.odrive_connection_generation+=1
        self.assertFalse(self.app._record_feedback_observation(current,old_generation,old_adapter))
        self.assertEqual(self.app.engine.evidence['feedback_latest'],before)
