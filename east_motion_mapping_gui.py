"""Modeless motion experiment window. No hardware connection/control implementation."""
from copy import deepcopy
from datetime import datetime
from pathlib import Path
import threading
import time
import customtkinter as ctk
from east_core import write_json_atomic
from east_motion_mapping import make_plan, ready_for_next, completed_run, quiescent


from east_batch_gui import RecordedBatchWindow


class MotionMappingWindow(RecordedBatchWindow):
    def __init__(self, master, app):
        self.app, self.closed, self.active = app, False, False
        self.cancelled = threading.Event()
        self.request = None
        self.requests = []
        self.records = []
        self.save_future = None
        self.save_action = None
        self.manifest_path = None
        self.release_pending = False
        self.awaiting_run = False
        self.window = ctk.CTkToplevel(master)
        self.window.title('EAST — Motion Modelling')
        width = min(820, max(640, self.window.winfo_screenwidth()-80))
        height = min(760, max(580, self.window.winfo_screenheight()-100))
        self.window.geometry(f'{width}x{height}')
        self.window.minsize(min(680, width), min(600, height))
        self.window.transient(master)
        self.window.protocol('WM_DELETE_WINDOW', self.close)
        self.window.bind('<Escape>', lambda event: self.stop())
        panel = ctk.CTkFrame(self.window)
        panel.pack(fill='both', expand=True, padx=12, pady=12)
        panel.grid_columnconfigure((0, 1, 2), weight=1)
        panel.grid_rowconfigure(10, weight=1)
        ctk.CTkLabel(panel, text='Empty-machine nonlinear motion experiment',
                     font=('Arial', 20, 'bold')).grid(row=0, column=0, columnspan=3, pady=6)
        ctk.CTkLabel(panel, text='Uses the existing connection, rotary-verified zero, unloaded tare and full normal recording.\n'
            'Home and capture unloaded tare in the main window. Keep the AFO absent. This tool does not redefine zero or reconnect.',
            wraplength=730, justify='left').grid(row=1, column=0, columnspan=3, sticky='ew', pady=5)
        self.identity = ctk.CTkLabel(panel, text='', wraplength=730)
        self.identity.grid(row=2, column=0, columnspan=3, sticky='ew')
        self.fields = {}
        for row, (key, label, value) in enumerate([
            ('range', 'Nominal range each side (deg)', '10'), ('acceleration', 'Acceleration (deg/s²)', '100'),
            ('cycles', 'Cycles per run', '3'), ('slow', 'Slow speed (deg/s)', '0.5'),
            ('fast', 'Operating speed (deg/s)', '5')], 3):
            ctk.CTkLabel(panel, text=label).grid(row=row, column=0, sticky='w', padx=6)
            entry = ctk.CTkEntry(panel)
            entry.insert(0, value)
            entry.grid(row=row, column=1, columnspan=2, sticky='ew', padx=6, pady=3)
            self.fields[key] = entry
        self.status = ctk.CTkLabel(panel, text='Ready to prepare a single slow run or the six-run sequence', wraplength=730)
        self.status.grid(row=8, column=0, columnspan=3, pady=5, sticky='ew')
        self.single_button = ctk.CTkButton(panel, text='Run One Slow Test', command=lambda:self.start('single'))
        self.sequence_button = ctk.CTkButton(panel, text='Run Six-Test Sequence', command=lambda:self.start('sequence'))
        self.run_buttons = [self.single_button, self.sequence_button]
        self.stop_button = ctk.CTkButton(panel, text='Stop', command=self.stop, fg_color='#dc2626')
        for col, button in enumerate((self.single_button, self.sequence_button, self.stop_button)):
            button.grid(row=9, column=col, padx=4, pady=6, sticky='ew')
        self.terminal = ctk.CTkTextbox(panel, wrap='word')
        self.terminal.grid(row=10, column=0, columnspan=3, sticky='nsew', pady=4)
        self.terminal.configure(state='disabled')
        self.output = ctk.CTkLabel(panel, text='Full strain CSV + raw rotary CSV + metadata JSON per run', wraplength=730)
        self.output.grid(row=11, column=0, columnspan=3, sticky='ew')
        self.window.after(200, self.tick)



    def start(self, mode):
        if self.active or self.save_future is not None:
            return
        app = self.app
        try:
            calibration = app.rotary_calibration
            if calibration is not None and not calibration.closed:
                raise ValueError('Close Rotary Sensor Calibration before motion modelling')
            recovery = app.recovery_window
            if recovery is not None and recovery.winfo_exists():
                raise ValueError('Close Machine Zero setup before motion modelling')
            if not ready_for_next(app) or app.monitor_error or app.feedback_retry_pending:
                raise ValueError('Finish the current operation and resolve feedback/cleanup before starting')
            if not app.reference_manager.verified or app.empty_machine_tare is None:
                raise ValueError('Use Home / Return to Zero and capture unloaded tare first')
            if app.manual_mode.get():
                raise ValueError('Turn Manual Mode off before a normal test')
            app.rotary_sensor.measurement(require=True)
            identity = dict(operator=app.operator_input.get(),fixture_id=app.fixture_id_input.get(),
                            calibration_id=app.calibration_id_input.get())
            settings = {k:v.get() for k,v in self.fields.items()}
            from east_core import APP_DIR
            import sys
            base = Path(sys.executable).resolve().parent if getattr(sys,'frozen',False) else APP_DIR
            root = base/app.system_config['logging']['output_directory']
            path = root/('motion_modelling_'+datetime.now().strftime('%Y%m%dT%H%M%S_%f')+'.json')
            requests = make_plan(app.system_config, identity, settings, mode, path)
            summary = '\n'.join(f'{r.run_id}: {r.parameters.commanded_afo_speed_deg_s:g} deg/s, '
                f'+/-{r.parameters.max_angle_deg:g} nominal deg, {r.parameters.cycles} cycles' for r in requests)
            if not app.show_resizable_confirmation(title='Confirm Motion Modelling',
                message='EMPTY MACHINE ONLY — AFO absent, fixture clear, enclosure protection in use and E-stop accessible.\n\n'
                +summary+'\n\nEach run uses the normal force/motor/rotary recorder. Stop cancels the remaining sequence.',
                confirm_text='Run Test' if mode=='single' else 'Run Sequence'):
                return
            self.requests, self.records = requests, []
            self.manifest_path = path
            self.cancelled.clear()
            self.active = True
            self.awaiting_run = False
            self.release_pending = True
            self.request = None
            self.identity_snapshot = identity
            self.reference_snapshot = app.reference_manager.metadata_snapshot()
            self.batch_status = 'running'
            self.batch_error = None
            app.set_test_inputs_state('disabled')
            app._refresh_motion_controls()
            self.output.configure(text=f'Batch manifest: {path}')
            self.save('next')
        except Exception as exc:
            self.status.configure(text=f'Cannot start: {exc}')
            app.update_terminal(f'Motion modelling: {exc}\n')
            if self.active:
                self.cancelled.set()
                self.active = False
                app.stop_logging()

    def save(self, action):
        # Small sidecar writes use the existing lifecycle worker, never block Tk.
        body = dict(schema='east-gui-motion-modelling-1',purpose='nonlinear_motion_identification',
            batch_id=self.requests[0].batch_id,physical_condition='empty_machine',
            identity=self.identity_snapshot, reference=self.reference_snapshot,
            plan=[dict(context=r.context(),parameters=r.parameters.__dict__) for r in self.requests],
            runs=deepcopy(self.records),outcome=self.batch_status,error=self.batch_error,
            updated_at=datetime.now().astimezone().isoformat(),
            idle_confirmed=bool(self.app.motion_coordinator.motor_idle_confirmed))
        self.save_action = action
        self.save_future = self.submit_manifest('save motion modelling manifest',
                                                 lambda:write_json_atomic(self.manifest_path,body))
