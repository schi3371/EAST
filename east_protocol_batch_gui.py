"""Three-speed AFO/baseline panel using the shared recorded-batch scheduler."""
from copy import deepcopy
from datetime import datetime
from pathlib import Path
import threading
import sys
import uuid
import customtkinter as ctk
from east_core import APP_DIR, write_json_atomic
from east_motion_model import enabled as motion_model_enabled, validate_range as validate_model_range, provenance as model_provenance
from east_batch_gui import RecordedBatchWindow
from east_motion_mapping import ready_for_next
from east_protocol_batch import make_protocol_plan
from east_stream_screen import SCREEN_PRESET
from east_smooth_motion import PRESET as SMOOTH_PAIR
from east_full_model_suite import PRESET as FULL_COMPARISON
from east_feedback_commissioning import CONDITIONS, SEQUENCE_PRESET, condition_for


class ThreeSpeedProtocolWindow(RecordedBatchWindow):
    track_admission_failures=True

    def __init__(self,master,app,preset_key):
        self.app,self.closed,self.active=app,False,False
        self.preset_key=preset_key
        self.smooth_pair = preset_key == SMOOTH_PAIR
        self.full_comparison = preset_key == FULL_COMPARISON
        self.screen_suite = preset_key in (SCREEN_PRESET, FULL_COMPARISON)
        self.commissioning = condition_for(preset_key)
        self.feedback_sequence = preset_key == SEQUENCE_PRESET
        self.stage_approved_run = None
        self.awaiting_stage_review = None
        self.stage_reviews = []
        self.pilot_suite=app.system_config['test_presets'][preset_key].get('protocol_kind')=='rotary_feedback_pilot'
        self.comparison_suite=app.system_config['test_presets'][preset_key].get('protocol_kind') in ('model_comparison','rotary_feedback_pilot')
        self.remaining_suite=app.system_config['test_presets'][preset_key].get('protocol_kind') in ('remaining_motion_data','model_comparison','rotary_feedback_pilot')
        self.pending_zero_recovery=False;self.recovery_future=None;self.reference_history=[]
        self.pending_zero_preparation=False;self.preparation_future=None;self.prepared_for_run=None;self.preparation_records=[]
        self.cancelled=threading.Event()
        self.request=None;self.requests=[];self.records=[]
        self.save_future=None;self.save_action=None;self.manifest_path=None
        self.release_pending=False;self.awaiting_run=False
        self.window=ctk.CTkToplevel(master)
        self.window.title('EAST — Full Model × Speed Comparison' if self.full_comparison else 'EAST — Rotary Feedback Pilot' if self.pilot_suite else 'EAST — Model Comparison' if self.comparison_suite else 'EAST — Remaining Motion Data' if self.remaining_suite else 'EAST — Three-Speed Protocol')
        self.window.geometry('800x680');self.window.minsize(660,560)
        self.window.transient(master);self.window.protocol('WM_DELETE_WINDOW',self.close)
        self.window.bind('<Escape>',lambda e:self.stop())
        panel=ctk.CTkFrame(self.window);panel.pack(fill='both',expand=True,padx=12,pady=12)
        panel.grid_columnconfigure((0,1),weight=1);panel.grid_rowconfigure(7,weight=1)
        preset=app.system_config['test_presets'][preset_key]
        ctk.CTkLabel(panel,text=preset['display_name'],font=('Arial',19,'bold')).grid(row=0,column=0,columnspan=2,pady=6)
        empty=preset['test_type']=='empty_machine_baseline'
        ctk.CTkLabel(panel,text=('EMPTY MACHINE — run before mounting the AFO.\n' if empty else 'AFO FITTED — one mounting shared by all three speed conditions.\n')+
            ('One sequence of 11 stages. Each stage runs feedforward then rotary feedback. '
             'Choose a stage range and pause for review between stages.\n'
             if self.feedback_sequence else 'Two matched runs, rotary feedback off then on. Empty ±10° at 5°/s, three cycles each.\n' if self.smooth_pair else f"Two matched runs: feedforward then feedback. Empty ±{self.commissioning.half_range_deg:g}°, "
             f"{self.commissioning.speed_deg_s:g}°/s, reference acceleration "
             f"{self.commissioning.acceleration_deg_s2:g}°/s². Review this pair before selecting another stage.\n"
             if self.commissioning else '147 runs: seven methods × seven speeds × three repeats. Empty ±15°, acceleration 100°/s². Includes 30, 40 and 50°/s. Start/end run allow resuming.\n' if self.full_comparison else '21 runs in three blocks of seven methods. Start with runs 1–7. Empty ±5° at 0.5°/s.\n' if self.screen_suite else '2 runs: wider cubic feedforward then continuous rotary-feedback PILOT. Empty +/-5 at 0.5 degrees/s only.\n' if self.pilot_suite else '22 runs: Original/Candidate +/-5 and +/-10 checks, then 18 Original/Current/Candidate +/-15 runs.\n' if self.comparison_suite else '10 runs: two Updated +/-10 repeat sets, rotary-supervised +/-15 identification at all three speeds, then stationary holds.\n' if self.remaining_suite else 'Three complete runs: 0.5 → 5 → 20°/s. Main ROM, acceleration and cycles apply to EACH speed.\n')+
            'Settings are provisional. No added inter-run rest beyond normal settling, idle and file saving.',
            wraplength=720,justify='left').grid(row=1,column=0,columnspan=2,sticky='ew',pady=6)
        self.identity=ctk.CTkLabel(panel,text='',wraplength=720)
        self.identity.grid(row=2,column=0,columnspan=2,sticky='ew')
        self.fields={}
        for row,key,label,value in [(3,'mounting_setup_id','Mounting/setup ID (reuse if not remounted)',('setup_' if empty else 'mount_')+uuid.uuid4().hex[:12]),
                ((4,'start_run','Start at stage' if self.feedback_sequence else 'Start at run (new batch; earlier runs skipped)','1') if self.remaining_suite else (4,'baseline_batch_id','Baseline batch ID (optional, unverified link only)',''))]:
            ctk.CTkLabel(panel,text=label).grid(row=row,column=0,sticky='w',padx=4)
            entry=ctk.CTkEntry(panel);entry.insert(0,value);entry.grid(row=row,column=1,sticky='ew',pady=4)
            self.fields[key]=entry
        if self.comparison_suite:
            ctk.CTkLabel(panel, text="Stop after stage (blank = last)" if self.feedback_sequence else "Stop after run (blank = last)").grid(row=9, column=0, sticky="w", padx=4)
            entry = ctk.CTkEntry(panel)
            entry.grid(row=9, column=1, sticky="ew", pady=4)
            if self.screen_suite and not self.full_comparison:
                entry.insert(0, '7')
            self.fields["end_run"] = entry
        if self.feedback_sequence:
            self.pause_between_stages = ctk.BooleanVar(value=False)
            ctk.CTkCheckBox(panel, text="Pause for review after each stage",
                           variable=self.pause_between_stages, command=self.set_stage_pause_choice).grid(row=10, column=0, sticky="w")
            self.continue_stage_button = ctk.CTkButton(
                panel, text="Continue to next stage", command=self.continue_stage,
                state="disabled")
            self.continue_stage_button.grid(row=10, column=1, sticky="ew", pady=4)
        self.readonly_fields={'baseline_batch_id'} if empty and not self.remaining_suite else set()
        if empty and not self.remaining_suite:self.fields['baseline_batch_id'].configure(state='disabled')
        self.status=ctk.CTkLabel(panel,text=('Ready. Load the fixed preset, confirm the empty fixture, then run all stages.' if self.remaining_suite else 'Ready. Common cycles apply to EACH speed; the full total is shown before Run.'),wraplength=720)
        self.status.grid(row=5,column=0,columnspan=2,sticky='ew',pady=6)
        self.run_button=ctk.CTkButton(panel,text='Run All Model × Speed Tests' if self.full_comparison else 'Run Feedback Sequence' if self.feedback_sequence else 'Run Smoothed Pair' if self.smooth_pair else 'Run Feedback Pilot' if self.pilot_suite else 'Run Model Comparison' if self.comparison_suite else 'Run Remaining Data Suite' if self.remaining_suite else 'Run Three-Speed Sequence',command=self.start)
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

    def confirmation_message(self, requests):
        a = self.app
        p=requests[0].parameters
        def stage_label(r):
            if self.feedback_sequence:
                stage = r.preset_metadata["feedback_stage"]
                return f"Stage {stage}, " + ("rotary feedback" if r.feedback_pilot else "feedforward")
            if self.smooth_pair:
                return 'Smoothed motion, rotary feedback ' + ('on' if r.feedback_pilot else 'off')
            if self.screen_suite:
                return f"Block {r.repeat}, {r.model_variant} " + ("with feedback" if r.feedback_pilot else "feedforward")
            return {'updated_10_repeat':f'Updated +/-10 repeat {r.repeat}', 'wide_15_identification':'Original + rotary +/-15 identification', 'stationary_direction_holds':'Stationary directional holds'}.get(r.stage,f'{r.stage}: {getattr(r,"model_variant","current")}')
        summary='\n'.join(f'Run {r.index}: {stage_label(r)} — {r.parameters.commanded_afo_speed_deg_s:g}°/s, +/-{r.parameters.max_angle_deg:g}°, {"30 holds + 30s zero recording" if r.stationary_holds else str(r.parameters.cycles)+" cycles"}' for r in requests)
        empty=p.test_type=='empty_machine_baseline'
        return (
            ('EMPTY MACHINE — remove AFO and removable loads.' if empty else f'AFO: {p.afo_id} — same mounting for all speeds.')+
                f'\nOperator: {p.operator} | Fixture: {p.fixture_id} | Calibration: {p.calibration_id}'
                f'\nMounting/setup ID: {requests[0].mounting_setup_id}\n\n'+summary+
                f'\nTotal: {len(requests)} runs, {sum(r.parameters.cycles for r in requests if not r.stationary_holds)} numbered sweep cycles'+ (' plus 30 stationary target holds.' if any(r.stationary_holds for r in requests) else '.')+
                '\nNormal return/settling/recording between runs. One mounting; no independent remount claims.'
                + (' Each comparison run receives a separately recorded rotary-zero check and fresh motor reference before dispatch; zeroing movement occurs only if needed.' if self.comparison_suite and not self.pilot_suite else '')
                + ('\nROM and acceleration follow the fixed settings for each stage.' if self.remaining_suite else f'\nROM: −{p.min_angle_deg:g}° to +{p.max_angle_deg:g}°; acceleration {p.commanded_afo_acceleration_deg_s2:g}°/s².')
                + ('\nMethods follow the explicit staged plan.' if self.remaining_suite else ('\nExperimental cubic targets; extrapolation allowed to machine limit; speed varies.' if motion_model_enabled(a.system_config) else '\nLegacy fixed conversion.'))+
                ('\nEMPTY FIXTURE ONLY. The full selection reaches the maximum displayed range. '
                 + ('The sequence pauses after each matched pair.' if self.pause_between_stages.get()
                    else 'Automatic continuation through the selected stage range is enabled.')
                 if self.feedback_sequence else '\nEMPTY FIXTURE ONLY. This batch runs only the selected matched pair. Review tracking, command timing and reversals before another stage.' if self.commissioning else '\nEMPTY FIXTURE ONLY. Full model × speed comparison through 50°/s; ±15° reference, ±16° measured-travel guard. Requested speed is not assumed achieved. Tracking errors are saved; zero misses automatically home and continue. Allow approximately 90–120 minutes for all 147 runs.' if self.full_comparison else '\nEMPTY FIXTURE ONLY. Fixed ±5° at 0.5°/s. Automatic homing follows a fully saved model zero miss.' if self.screen_suite else '\nEMPTY FIXTURE ONLY. Confirm clearance through both endpoints and stay with the machine.' if self.smooth_pair else '\nEMPTY FIXTURE ONLY. Continuous pilot: +/-5 at 0.5 degrees/s, no automatic escalation. First run is the feedforward reference; second is the feedback pilot.' if self.pilot_suite else '\nEMPTY FIXTURE ONLY. Original 2.055, current and wider cubic. Original angles are nominal; measured travel supervised. No endpoint corrections. Slow moves are split into <=30-second legs; analyse stop/transition phases separately. Confirm +/-15 clearance. The full 22-run sequence takes about 60–90 minutes.' if self.comparison_suite else '\nEMPTY FIXTURE ONLY. Confirm physical clearance through +/-15 degrees plus 0.1-degree tolerance. Methods change automatically by stage. Wide endpoints use rotary feedback: these are identification data, not a feed-forward accuracy test. Stationary holds last 2 seconds, approached from both directions. Allow roughly 25–35 minutes; Stop cancels the remaining suite.' if self.remaining_suite else '')+
                ('\nAfter a fully saved run misses its model zero, EAST automatically homes using '
                 'the rotary sensor and continues after zero verifies. The unassisted error and '
                 'homing result are saved separately. Stop and other faults halt the batch.'
                 if self.app.system_config['test_presets'][self.preset_key].get('allow_bounded_zero_recovery') else '')+
                '\nNo added rest interval. Settings provisional; not scientifically validated.'
                '\n\nFixture clear, enclosure protection in use, E-stop accessible. Stop cancels all remaining runs.'
        )

    def start(self):
        if self.active or self.save_future is not None:return
        a=self.app
        try:
            if a.loaded_preset_key!=self.preset_key:
                raise ValueError('The active preset changed; reopen the sequence from Start')
            if any(future is not None and not future.done() for future in (self.recovery_future,self.preparation_future)):
                raise ValueError('Wait for the previous zero transaction to finish before starting a new batch')
            if not ready_for_next(a) or a.rotary_homing_active() or a.monitor_error or a.feedback_retry_pending:
                raise ValueError('Finish current work and resolve feedback/cleanup before starting')
            if a.manual_mode.get() or not a.rotary_reference_verified() or (not self.comparison_suite and not a.rotary_zero_ready()) or a.empty_machine_tare is None:
                raise ValueError('Home to zero, capture valid unloaded tare, and turn Manual Mode off first')
            if a.rotary_calibration is not None and not a.rotary_calibration.closed:
                raise ValueError('Close Rotary Sensor Calibration before starting')
            identity=self.current_identity();settings=a._motion_field_values()
            batch_id=uuid.uuid4().hex
            base=Path(sys.executable).resolve().parent if getattr(sys,'frozen',False) else APP_DIR
            path=base/a.system_config['logging']['output_directory']/(('motion_suite_' if self.remaining_suite else 'three_speed_')+batch_id+'.json')
            requests=make_protocol_plan(a.system_config,self.preset_key,identity,settings,
                self.fields['mounting_setup_id'].get(),'' if self.remaining_suite else self.fields['baseline_batch_id'].get(),path,a.preset_loaded_at,batch_id=batch_id)
            self.all_requests=requests
            start_input = int(self.fields["start_run"].get()) if self.remaining_suite else 1
            end_input = self.fields["end_run"].get().strip() if self.comparison_suite else ""
            selection_count = len(CONDITIONS) if self.feedback_sequence else len(requests)
            end_selection = int(end_input) if end_input else selection_count
            if not 1 <= start_input <= end_selection <= selection_count:
                raise ValueError("Choose a valid inclusive start/end range")
            start_at = 2 * (start_input - 1) + 1 if self.feedback_sequence else start_input
            end_at = 2 * end_selection if self.feedback_sequence else end_selection
            self.pause_stages_snapshot = bool(self.feedback_sequence and self.pause_between_stages.get())
            requests = requests[start_at - 1:end_at]
            self.selected_start_run=start_at
            from east_core import validate_empty_machine_tare
            for request in requests:
                valid,reason=validate_empty_machine_tare(a.empty_machine_tare,request.parameters,a.system_config)
                if not valid:raise ValueError(reason)
            prep_model=deepcopy(a.system_config["motion"].get("experimental_model", {}))
            for request in requests:
                if (request.command_model_enabled if request.command_model_enabled is not None else motion_model_enabled(a.system_config)):
                    validate_model_range(request.parameters.min_angle_deg, request.parameters.max_angle_deg)
            prep_reference=deepcopy(a.reference_manager.metadata_snapshot())
            prep_tare=a.empty_machine_tare.to_dict()
            prep_sensor=(a.rotary_sensor.reader.connection_id,a.rotary_sensor.calibration.sha256)
            prep_connection=a.odrive_connection_generation
            prep_preset=deepcopy(a.system_config['test_presets'][self.preset_key])
            if not a.show_resizable_confirmation(
                title='Confirm Remaining Motion Data' if self.remaining_suite else 'Confirm Three-Speed Protocol',
                message=self.confirmation_message(requests), confirm_text='Run Sequence',
            ):
                return
            # Freeze only if modal nested actions did not change any preparation.
            if self.remaining_suite and int(self.fields["start_run"].get())!=start_input:
                raise ValueError("Starting run changed during confirmation; review again")
            if self.comparison_suite and self.fields["end_run"].get().strip() != end_input:
                raise ValueError("Ending run changed during confirmation; review again")
            if self.feedback_sequence and self.pause_between_stages.get() != self.pause_stages_snapshot:
                raise ValueError("Stage pause setting changed during confirmation")
            if identity!=self.current_identity() or settings!=a._motion_field_values() or not ready_for_next(a) or a.monitor_error:
                raise ValueError('Preparation/settings changed during confirmation; review and start again')
            if (a.reference_manager.metadata_snapshot()!=prep_reference or a.empty_machine_tare is None
                    or a.empty_machine_tare.to_dict()!=prep_tare or a.odrive_connection_generation!=prep_connection
                    or (a.rotary_sensor.reader.connection_id,a.rotary_sensor.calibration.sha256)!=prep_sensor
                    or a.system_config["motion"].get("experimental_model", {})!=prep_model
                    or a.loaded_preset_key!=self.preset_key
                    or a.system_config['test_presets'][self.preset_key]!=prep_preset):
                raise ValueError('Preparation changed during confirmation; review and start again')
            self.model_snapshot=prep_model
            self.preset_snapshot=prep_preset
            self.requests,self.records=requests,[]
            self.manifest_path=path;self.identity_snapshot=deepcopy(identity);self.settings_snapshot=deepcopy(settings)
            self.reference_snapshot=deepcopy(a.reference_manager.metadata_snapshot())
            self.initial_reference_snapshot=deepcopy(self.reference_snapshot)
            self.tare_snapshot=a.empty_machine_tare.to_dict()
            self.sensor_identity=(a.rotary_sensor.reader.connection_id,a.rotary_sensor.calibration.sha256)
            self.connection_generation=a.odrive_connection_generation
            self.started_at=datetime.now().astimezone().isoformat()
            self.batch_status='running';self.batch_error=None
            self.request=None;self.awaiting_run=False;self.cancelled.clear()
            a.operator_stop_requested=False;a.acquisition_error=None
            # A restarted batch must never consume the previous batch's completed/failed homing future.
            self.pending_zero_recovery=False;self.recovery_future=None
            self.pending_zero_preparation=False;self.preparation_future=None;self.prepared_for_run=None;self.preparation_records=[];self.reference_history=[]
            self.stage_approved_run = None
            self.awaiting_stage_review = None
            self.stage_reviews = []
            self.active=True;self.release_pending=True
            a.set_test_inputs_state('disabled');a._refresh_motion_controls()
            self.output.configure(text=f'Batch manifest: {path}')
            self.save('next')
        except Exception as exc:
            self.status.configure(text=f'Cannot start sequence: {exc}')
            a.update_terminal(f'Three-speed sequence: {exc}\n')
            if self.active:self.cancelled.set();self.active=False;a.stop_logging()

    def handle_failed_run(self, reason):
        from east_motion_mapping import zero_recovery_candidate
        if (not self.preset_snapshot.get('allow_bounded_zero_recovery')
                or not zero_recovery_candidate(self.app,self.request)):
            return False
        self.pending_zero_recovery=True
        self.status.configure(text='Run data saved. Automatically homing before continuing; incomplete attempts remain labelled failed.')
        self.save('next')
        return True

    def set_stage_pause_choice(self):
        if self.active:
            self.pause_between_stages.set(self.pause_stages_snapshot)
            self.app.update_terminal("Stage pause mode is fixed for the active sequence.\n")

    def continue_stage(self):
        if (not self.active or self.cancelled.is_set()
                or self.awaiting_stage_review is None or self.save_future is not None):
            return
        self.stage_approved_run = self.awaiting_stage_review
        self.stage_reviews.append(dict(next_run=self.stage_approved_run,
                                       approved_at=datetime.now().astimezone().isoformat()))
        self.awaiting_stage_review = None
        self.continue_stage_button.configure(state="disabled")
        self.save("next")

    def next_run(self):
        if (self.feedback_sequence and not self.cancelled.is_set() and not self.pending_zero_recovery
                and self.pause_stages_snapshot and 0 < len(self.records) < len(self.requests)):
            request = self.requests[len(self.records)]
            previous = self.requests[len(self.records) - 1]
            if (request.feedback_condition_key != previous.feedback_condition_key
                    and self.stage_approved_run != request.index):
                self.awaiting_stage_review = request.index
                self.status.configure(text=f"Stage complete. Review the saved pair, then continue to stage "
                                           f"{request.preset_metadata['feedback_stage']}.")
                self.continue_stage_button.configure(state="normal")
                return
        if not self.pending_zero_recovery:
            if self.comparison_suite and not self.pilot_suite and self._prepare_comparison_next_run():return
            return super().next_run()
        from east_motion_mapping import ready_for_next,quiescent
        from east_rotary_homing import HomingPolicy
        a=self.app
        if self.cancelled.is_set():
            if quiescent(a) and (self.recovery_future is None or self.recovery_future.done()):
                self._record_aborted_zero_transaction('post_run_zero_recovery',self.recovery_future,self.records[-1].setdefault('zero_recovery_attempt',{}))
                self.finish('error' if a.monitor_error or a.acquisition_error else 'cancelled',a.monitor_error or a.acquisition_error or 'Stop cancelled separate zero recovery')
            return
        try:
            if self.recovery_future is None:
                if not ready_for_next(a):return
                if not self.check_setup_continuity():return
                self.recovery_future=a.start_suite_zero_recovery(self)
                self.records[-1]["zero_recovery_attempt"] = {
                    "evidence_file": getattr(a.rotary_homing, "evidence_path", None),
                    "policy": HomingPolicy().__dict__,
                    "mode": "automatic_rotary_homing_after_saved_zero_miss",
                }
                self.status.configure(text='Automatic rotary homing in progress; next run starts after zero verifies.')
                return
            if not self.recovery_future.done() or not quiescent(a):return
            result=self.recovery_future.result();e=result['evidence']
            attempt=self.records[-1].setdefault('zero_recovery_attempt',{})
            attempt.update(outcome=e.get('outcome'),error=e.get('error'),
                idle_confirmed=e.get('idle_confirmed'),zero_reference_written=e.get('zero_reference_written'),
                parent_batch_id=e.get('parent_batch_id'),parent_run_id=e.get('parent_run_id'))
            if e.get('parent_batch_id') != self.request.batch_id or e.get('parent_run_id') != self.request.run_id:
                raise RuntimeError('Homing result belongs to another batch/run; refusing stale recovery evidence')
            a.rotary_homing.active=False
            if e.get('outcome')!='sensor_zero_reached' or not e.get('idle_confirmed') or not e.get('zero_reference_written'):
                detail=e.get('error') or f"outcome={e.get('outcome')}; idle={e.get('idle_confirmed')}; saved zero={e.get('zero_reference_written')}"
                raise RuntimeError(f"Automatic homing failed: {detail}. Evidence: {attempt.get('evidence_file')}")
            if (self.current_identity()!=self.identity_snapshot or a._motion_field_values()!=self.settings_snapshot
                    or a.empty_machine_tare is None or a.empty_machine_tare.to_dict()!=self.tare_snapshot
                    or a.odrive_connection_generation!=self.connection_generation
                    or (a.rotary_sensor.reader.connection_id,a.rotary_sensor.calibration.sha256)!=self.sensor_identity
                    or a.system_config['test_presets'][self.preset_key]!=self.preset_snapshot
                    or a.system_config['motion'].get('experimental_model',{})!=self.model_snapshot):
                raise RuntimeError('Setup changed during separate zero recovery')
            new_reference=deepcopy(a.reference_manager.metadata_snapshot())
            entry=dict(kind='post_run_zero_recovery',after_run=self.request.run_id,old_reference=self.reference_snapshot,new_reference=new_reference,
                evidence_file=getattr(a.rotary_homing,'evidence_path',None),
                evidence=result['message'],final_observation=e['final_observation'],reference_equivalence_assumed=False)
            self.reference_history.append(entry);self.reference_snapshot=new_reference
            self.records[-1]['zero_recovery']=entry
            complete = (a.run_metadata or {}).get('completed_cycles') == self.request.parameters.cycles
            self.records[-1]['model_zero_return_passed']=(a.run_metadata or {}).get('zero_return_check',{}).get('passed')
            self.records[-1]['acquisition_complete']=complete
            self.records[-1]['failed_run_skipped']=not complete
            self.pending_zero_recovery=False;self.recovery_future=None
            self.save('next')
        except Exception as exc:
            self.pending_zero_recovery=False
            self.finish('error',f'Separate zero recovery stopped: {exc}')

    def _prepare_comparison_next_run(self):
        """One durable zero transaction per open-loop run, not an encoder-angle guess."""
        from east_motion_mapping import ready_for_next,quiescent
        a=self.app
        if self.cancelled.is_set():
            if quiescent(a) and (self.preparation_future is None or self.preparation_future.done()):
                if self.preparation_records:self._record_aborted_zero_transaction('pre_run_zero_preparation',self.preparation_future,self.preparation_records[-1])
                self.pending_zero_preparation=False
                self.finish('error' if a.monitor_error or a.acquisition_error else 'cancelled',a.monitor_error or a.acquisition_error or 'Stop during zero preparation')
            return True
        if len(self.records)>=len(self.requests):return False
        request=self.requests[len(self.records)]
        try:
            if not self.pending_zero_preparation:
                if not ready_for_next(a):return True
                if not self.check_setup_continuity():return True
                if self.prepared_for_run==request.run_id:
                    if a.rotary_zero_ready(rotary_only=True):return False
                    raise RuntimeError('Rotary zero moved outside the band after preparation; inspect fixture settling before retrying')
                self.pending_zero_preparation=True;self.preparation_future=None
                self.preparation_records.append(dict(before_run=request.run_id,status='requested',old_reference=deepcopy(self.reference_snapshot)))
                self.status.configure(text=f'Preparing measured zero before {request.run_id}; saving preparation intent first.')
                self.save('next')
                return True
            if self.preparation_future is None:
                if not ready_for_next(a):return True
                if not self.check_setup_continuity():return True
                self.preparation_future=a.start_comparison_zero_preparation(self)
                record=self.preparation_records[-1]
                record.update(status='running',evidence_file=a.rotary_homing.evidence_path,
                    verification_only=a.rotary_homing.verify_only)
                return True
            if not self.preparation_future.done() or not quiescent(a):return True
            result=self.preparation_future.result();e=result['evidence'];a.rotary_homing.active=False
            if e.get('outcome')!='sensor_zero_reached' or not e.get('idle_confirmed') or not e.get('zero_reference_written'):
                raise RuntimeError('Zero preparation did not finish with saved evidence and confirmed idle')
            # Only the explicitly owned zero transaction may change the reference.
            if (self.current_identity()!=self.identity_snapshot or a._motion_field_values()!=self.settings_snapshot
                    or a.empty_machine_tare is None or a.empty_machine_tare.to_dict()!=self.tare_snapshot
                    or a.odrive_connection_generation!=self.connection_generation
                    or (a.rotary_sensor.reader.connection_id,a.rotary_sensor.calibration.sha256)!=self.sensor_identity
                    or a.system_config['test_presets'][self.preset_key]!=self.preset_snapshot
                    or a.system_config['motion'].get('experimental_model',{})!=self.model_snapshot):
                raise RuntimeError('Setup changed during zero preparation')
            new=deepcopy(a.reference_manager.metadata_snapshot());record=self.preparation_records[-1]
            record.update(status='completed',new_reference=new,final_observation=e['final_observation'],
                reference_equivalence_assumed=False)
            self.reference_history.append(dict(kind='pre_run_zero_preparation',**record))
            self.reference_snapshot=new;self.prepared_for_run=request.run_id
            self.pending_zero_preparation=False;self.preparation_future=None
            self.save('next')
            return True
        except Exception as exc:
            if self.preparation_records:self.preparation_records[-1].update(status='failed',error=str(exc))
            self.pending_zero_preparation=False
            self.finish('error',f'Before {request.run_id}: zero preparation failed: {exc}')
            return True

    def _record_aborted_zero_transaction(self,kind,future,record):
        a=self.app
        record.update(status='error' if a.monitor_error or a.acquisition_error else 'cancelled',
            error=a.monitor_error or a.acquisition_error or 'Operator Stop before next-run dispatch')
        if future is not None and future.done():
            try:
                evidence=future.result()['evidence']
                record.update(zero_reference_written=evidence.get('zero_reference_written',False),
                    transaction_outcome=evidence.get('outcome'))
            except Exception as exc:record['transaction_error']=str(exc)
        new=deepcopy(a.reference_manager.metadata_snapshot())
        if new!=self.reference_snapshot:
            self.reference_history.append(dict(kind=kind+'_aborted',old_reference=self.reference_snapshot,
                new_reference=new,reference_equivalence_assumed=False,
                evidence_file=record.get('evidence_file'),transaction_status=record['status']))
            self.reference_snapshot=new
        record['actual_reference_after']=new

    def abort_preparation_for_batch_error(self):
        if self.pending_zero_preparation and self.preparation_records:
            self._record_aborted_zero_transaction('pre_run_zero_preparation',self.preparation_future,self.preparation_records[-1])
            self.pending_zero_preparation=False

    def finish(self,status,error):
        state=getattr(self.app,'rotary_homing',None)
        if getattr(state,'managed_by_batch',False):state.active=False
        if status=='completed' and any(r.get('failed_run_skipped') for r in self.records):
            status='completed_with_failed_runs'
        if status=='completed' and any(r.get('zero_recovery') for r in self.records):
            status='completed_with_zero_recovery'
        return super().finish(status,error)

    def check_setup_continuity(self):
        a=self.app
        reason=None
        if a.system_config["motion"].get("experimental_model", {})!=self.model_snapshot:
            reason="Frozen motion model configuration changed"
        elif a.loaded_preset_key!=self.preset_key or a.system_config['test_presets'][self.preset_key]!=self.preset_snapshot:
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
                if record.get('zero_recovery'):
                    row['status']='failed_run_recovered_skipped' if record.get('failed_run_skipped') else 'acquired_zero_recovered'
            if i and row.get('started_at') and rows[-1].get('completed_at'):
                row['observed_gap_since_previous_run_s']=(datetime.fromisoformat(row['started_at'])-datetime.fromisoformat(rows[-1]['completed_at'])).total_seconds()
            rows.append(row)
        skipped=[dict(context=r.context(),parameters=r.parameters.__dict__,status='skipped_by_operator_before_start',
            reason='Operator selected a later starting run in a NEW batch; prior experimental data are not imported or certified',
            metadata_file=None,strain_file=None,rotary_file=None) for r in self.all_requests[:self.selected_start_run-1]]
        after = [dict(context=r.context(), parameters=r.parameters.__dict__,
                      status='outside_selected_run_range', metadata_file=None,
                      strain_file=None, rotary_file=None)
                 for r in self.all_requests[self.selected_start_run - 1 + len(self.requests):]]
        rows = skipped + rows + after
        body=dict(schema='east-three-speed-batch-1',batch_id=self.requests[0].batch_id,
            mounting_setup_id=self.requests[0].mounting_setup_id,test_type=self.requests[0].parameters.test_type,
            identity=self.identity_snapshot,common_settings=self.settings_snapshot,
            pause_between_stages=getattr(self, 'pause_stages_snapshot', False),
            stage_reviews=self.stage_reviews,
            selected_start_run=self.selected_start_run,selected_end_run=self.selected_start_run + len(self.requests) - 1,selected_run_count=len(self.requests),total_plan_runs=len(self.all_requests),
            protocol=deepcopy(self.requests[0].preset_metadata),planned_speed_sequence_deg_s=list(self.requests[0].speed_sequence),
            cycles_per_speed=self.requests[0].parameters.cycles,total_numbered_cycles=sum(r.parameters.cycles for r in self.requests if not r.stationary_holds),
            stationary_target_holds=30 if any(r.stationary_holds for r in self.requests) else 0,initial_zero_stability_s=30 if any(r.stationary_holds for r in self.requests) else 0,
            additional_inter_run_rest_s=0.0,inter_run_behaviour='Normal zero return, settling, confirmed idle, recorder finalization and cleanup; no extra conditioning/rest',
            reference=self.reference_snapshot,initial_reference=self.initial_reference_snapshot,reference_history=self.reference_history,zero_preparations=self.preparation_records,
            unloaded_tare=self.tare_snapshot,started_at=self.started_at,
            updated_at=datetime.now().astimezone().isoformat(),completed_at=datetime.now().astimezone().isoformat() if finished else None,
            runs=rows,outcome=self.batch_status,error=self.batch_error,independent_remount_trials=False,
            automatic_baseline_correction=False,baseline_reference_equivalence_assumed=False,
            experimental_motion_model={"mode":"mixed_by_plan","coefficients":model_provenance()} if self.remaining_suite else (model_provenance() if self.model_snapshot.get("enabled",False) else {"enabled":False}))
        self.save_action=action
        self.save_future=self.submit_manifest('save three-speed batch manifest',lambda:write_json_atomic(self.manifest_path,body))
