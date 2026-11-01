"""Three-speed AFO/baseline panel using the shared recorded-batch scheduler."""
from copy import deepcopy
from datetime import datetime
from pathlib import Path
import threading
import sys
import uuid
import customtkinter as ctk
from east_core import APP_DIR, write_json_atomic
from east_batch_gui import RecordedBatchWindow
from east_motion_mapping import ready_for_next
from east_protocol_batch import make_protocol_plan


class ThreeSpeedProtocolWindow(RecordedBatchWindow):
    track_admission_failures=True

    def __init__(self,master,app,preset_key):
        self.app,self.closed,self.active=app,False,False
        self.preset_key=preset_key
        self.cancelled=threading.Event()
        self.request=None;self.requests=[];self.records=[]
        self.save_future=None;self.save_action=None;self.manifest_path=None
        self.release_pending=False;self.awaiting_run=False
        self.window=ctk.CTkToplevel(master)
        self.window.title('EAST — Three-Speed Protocol')
        self.window.geometry('800x680');self.window.minsize(660,560)
        self.window.transient(master);self.window.protocol('WM_DELETE_WINDOW',self.close)
        self.window.bind('<Escape>',lambda e:self.stop())
        panel=ctk.CTkFrame(self.window);panel.pack(fill='both',expand=True,padx=12,pady=12)
        panel.grid_columnconfigure((0,1),weight=1);panel.grid_rowconfigure(7,weight=1)
        preset=app.system_config['test_presets'][preset_key]
        ctk.CTkLabel(panel,text=preset['display_name'],font=('Arial',19,'bold')).grid(row=0,column=0,columnspan=2,pady=6)
        empty=preset['test_type']=='empty_machine_baseline'
        ctk.CTkLabel(panel,text=('EMPTY MACHINE — run before mounting the AFO.\n' if empty else 'AFO FITTED — one mounting shared by all three speed conditions.\n')+
            'Three complete runs: 0.5 → 5 → 20°/s. Main ROM, acceleration and cycles apply to EACH speed.\n'
            'Settings are provisional. No added inter-run rest beyond normal settling, idle and file saving.',
            wraplength=720,justify='left').grid(row=1,column=0,columnspan=2,sticky='ew',pady=6)
        self.identity=ctk.CTkLabel(panel,text='',wraplength=720)
        self.identity.grid(row=2,column=0,columnspan=2,sticky='ew')
        self.fields={}
        for row,key,label,value in [(3,'mounting_setup_id','Mounting/setup ID (reuse if not remounted)',('setup_' if empty else 'mount_')+uuid.uuid4().hex[:12]),
                (4,'baseline_batch_id','Baseline batch ID (optional, unverified link only)','')]:
            ctk.CTkLabel(panel,text=label).grid(row=row,column=0,sticky='w',padx=4)
            entry=ctk.CTkEntry(panel);entry.insert(0,value);entry.grid(row=row,column=1,sticky='ew',pady=4)
            self.fields[key]=entry
        self.readonly_fields={'baseline_batch_id'} if empty else set()
        if empty:self.fields['baseline_batch_id'].configure(state='disabled')
        self.status=ctk.CTkLabel(panel,text='Ready. Common cycles apply to EACH speed; the full total is shown before Run.',wraplength=720)
        self.status.grid(row=5,column=0,columnspan=2,sticky='ew',pady=6)
        self.run_button=ctk.CTkButton(panel,text='Run Three-Speed Sequence',command=self.start)
        self.run_button.grid(row=6,column=0,padx=4,pady=6,sticky='ew')
        self.run_buttons=[self.run_button]
        self.stop_button=ctk.CTkButton(panel,text='Stop Sequence',command=self.stop,fg_color='#b91c1c')
        self.stop_button.grid(row=6,column=1,padx=4,pady=6,sticky='ew')
        self.terminal=ctk.CTkTextbox(panel,wrap='word');self.terminal.grid(row=7,column=0,columnspan=2,sticky='nsew')
        self.terminal.configure(state='disabled')
        self.output=ctk.CTkLabel(panel,text='Separate normal strain + rotary + metadata files per speed.',wraplength=720)
        self.output.grid(row=8,column=0,columnspan=2,sticky='ew',pady=5)
        self.window.after(200,self.tick)

    def current_identity(self):
        a=self.app
        return dict(file_prefix=a.file_name_input.get().strip(),operator=a.operator_input.get().strip(),
            afo_id=a.afo_id_input.get().strip(),fixture_id=a.fixture_id_input.get().strip(),
            calibration_id=a.calibration_id_input.get().strip())

    def start(self):
        if self.active or self.save_future is not None:return
        a=self.app
        try:
            if a.loaded_preset_key!=self.preset_key:
                raise ValueError('The active preset changed; reopen the sequence from Start')
            if not ready_for_next(a) or a.rotary_homing_active() or a.monitor_error or a.feedback_retry_pending:
                raise ValueError('Finish current work and resolve feedback/cleanup before starting')
            if a.manual_mode.get() or not a.rotary_zero_ready() or a.empty_machine_tare is None:
                raise ValueError('Home to zero, capture valid unloaded tare, and turn Manual Mode off first')
            if a.rotary_calibration is not None and not a.rotary_calibration.closed:
                raise ValueError('Close Rotary Sensor Calibration before starting')
            identity=self.current_identity();settings=a._motion_field_values()
            batch_id=uuid.uuid4().hex
            base=Path(sys.executable).resolve().parent if getattr(sys,'frozen',False) else APP_DIR
            path=base/a.system_config['logging']['output_directory']/('three_speed_'+batch_id+'.json')
            requests=make_protocol_plan(a.system_config,self.preset_key,identity,settings,
                self.fields['mounting_setup_id'].get(),self.fields['baseline_batch_id'].get(),path,a.preset_loaded_at,batch_id=batch_id)
            from east_core import validate_empty_machine_tare
            for request in requests:
                valid,reason=validate_empty_machine_tare(a.empty_machine_tare,request.parameters,a.system_config)
                if not valid:raise ValueError(reason)
            prep_reference=deepcopy(a.reference_manager.metadata_snapshot())
            prep_tare=a.empty_machine_tare.to_dict()
            prep_sensor=(a.rotary_sensor.reader.connection_id,a.rotary_sensor.calibration.sha256)
            prep_connection=a.odrive_connection_generation
            prep_preset=deepcopy(a.system_config['test_presets'][self.preset_key])
            p=requests[0].parameters
            summary='\n'.join(f'Run {r.index}: {r.parameters.commanded_afo_speed_deg_s:g}°/s — {r.parameters.cycles} cycles' for r in requests)
            empty=p.test_type=='empty_machine_baseline'
            if not a.show_resizable_confirmation(title='Confirm Three-Speed Protocol',message=(
                ('EMPTY MACHINE — remove AFO and removable loads.' if empty else f'AFO: {p.afo_id} — same mounting for all speeds.')+
                f'\nOperator: {p.operator} | Fixture: {p.fixture_id} | Calibration: {p.calibration_id}'
                f'\nMounting/setup ID: {requests[0].mounting_setup_id}\n\n'+summary+
                f'\nTotal: 3 runs, {3*p.cycles} numbered cycles; each run also positions initially and returns to zero.'
                f'\nROM: −{p.min_angle_deg:g}° to +{p.max_angle_deg:g}°; acceleration {p.commanded_afo_acceleration_deg_s2:g}°/s².'
                '\nNo added rest interval. Settings provisional; not scientifically validated.'
                '\n\nFixture clear, enclosure protection in use, E-stop accessible. Stop cancels all remaining runs.'),confirm_text='Run Sequence'):
                return
            # Freeze only if modal nested actions did not change any preparation.
            if identity!=self.current_identity() or settings!=a._motion_field_values() or not ready_for_next(a) or a.monitor_error:
                raise ValueError('Preparation/settings changed during confirmation; review and start again')
            if (a.reference_manager.metadata_snapshot()!=prep_reference or a.empty_machine_tare is None
                    or a.empty_machine_tare.to_dict()!=prep_tare or a.odrive_connection_generation!=prep_connection
                    or (a.rotary_sensor.reader.connection_id,a.rotary_sensor.calibration.sha256)!=prep_sensor
                    or a.loaded_preset_key!=self.preset_key
                    or a.system_config['test_presets'][self.preset_key]!=prep_preset):
                raise ValueError('Preparation changed during confirmation; review and start again')
            self.preset_snapshot=prep_preset
            self.requests,self.records=requests,[]
            self.manifest_path=path;self.identity_snapshot=deepcopy(identity);self.settings_snapshot=deepcopy(settings)
            self.reference_snapshot=deepcopy(a.reference_manager.metadata_snapshot())
            self.tare_snapshot=a.empty_machine_tare.to_dict()
            self.sensor_identity=(a.rotary_sensor.reader.connection_id,a.rotary_sensor.calibration.sha256)
            self.connection_generation=a.odrive_connection_generation
            self.started_at=datetime.now().astimezone().isoformat()
            self.batch_status='running';self.batch_error=None
            self.request=None;self.awaiting_run=False;self.cancelled.clear()
            self.active=True;self.release_pending=True
            a.set_test_inputs_state('disabled');a._refresh_motion_controls()
            self.output.configure(text=f'Batch manifest: {path}')
            self.save('next')
        except Exception as exc:
            self.status.configure(text=f'Cannot start sequence: {exc}')
            a.update_terminal(f'Three-speed sequence: {exc}\n')
            if self.active:self.cancelled.set();self.active=False;a.stop_logging()

    def check_setup_continuity(self):
        a=self.app
        reason=None
        if a.loaded_preset_key!=self.preset_key or a.system_config['test_presets'][self.preset_key]!=self.preset_snapshot:
            reason='Frozen protocol definition changed'
        elif self.current_identity()!=self.identity_snapshot or a._motion_field_values()!=self.settings_snapshot:
            reason='Frozen identifiers/settings changed'
        elif a.empty_machine_tare is None or a.empty_machine_tare.to_dict()!=self.tare_snapshot:
            reason='Unloaded tare changed or became invalid; no loaded tare was attempted'
        elif a.reference_manager.metadata_snapshot()!=self.reference_snapshot:
            reason='Zero reference or setup changed; baseline equivalence is not assumed'
        elif a.odrive_connection_generation!=self.connection_generation:
            reason='Controller connection changed'
        elif a.rotary_sensor.reader is None or (a.rotary_sensor.reader.connection_id,a.rotary_sensor.calibration.sha256)!=self.sensor_identity:
            reason='Rotary connection/calibration changed'
        if reason:
            self.finish('error',reason);return False
        return True

    def save(self,action):
        rows=[]
        finished=self.batch_status!='running'
        for i,request in enumerate(self.requests):
            row=dict(context=request.context(),parameters=request.parameters.__dict__,status='skipped' if finished else 'pending',
                strain_file=None,rotary_file=None,metadata_file=None,started_at=None,completed_at=None,
                reason=self.batch_error if finished else None)
            if i<len(self.records):
                record=deepcopy(self.records[i]);row.update(record)
                row['status']='completed' if record['outcome']=='success' else ('cancelled' if record.get('run_status')=='aborted' else 'failed')
            if i and row.get('started_at') and rows[-1].get('completed_at'):
                row['observed_gap_since_previous_run_s']=(datetime.fromisoformat(row['started_at'])-datetime.fromisoformat(rows[-1]['completed_at'])).total_seconds()
            rows.append(row)
        body=dict(schema='east-three-speed-batch-1',batch_id=self.requests[0].batch_id,
            mounting_setup_id=self.requests[0].mounting_setup_id,test_type=self.requests[0].parameters.test_type,
            identity=self.identity_snapshot,common_settings=self.settings_snapshot,
            protocol=deepcopy(self.requests[0].preset_metadata),planned_speed_sequence_deg_s=list(self.requests[0].speed_sequence),
            cycles_per_speed=self.requests[0].parameters.cycles,total_numbered_cycles=3*self.requests[0].parameters.cycles,
            additional_inter_run_rest_s=0.0,inter_run_behaviour='Normal zero return, settling, confirmed idle, recorder finalization and cleanup; no extra conditioning/rest',
            reference=self.reference_snapshot,unloaded_tare=self.tare_snapshot,started_at=self.started_at,
            updated_at=datetime.now().astimezone().isoformat(),completed_at=datetime.now().astimezone().isoformat() if finished else None,
            runs=rows,outcome=self.batch_status,error=self.batch_error,independent_remount_trials=False,
            automatic_baseline_correction=False,baseline_reference_equivalence_assumed=False)
        self.save_action=action
        self.save_future=self.submit_manifest('save three-speed batch manifest',lambda:write_json_atomic(self.manifest_path,body))
