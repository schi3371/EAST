"""Shared UI batch scheduling. All motor execution stays in MyInterface."""
import time
import threading
from concurrent.futures import Future
from east_motion_mapping import ready_for_next, completed_run, quiescent


class RecordedBatchWindow:
    def accepts(self, request):
        return self.active and not self.cancelled.is_set() and request is self.request

    def append_terminal(self, message):
        if self.closed:
            return
        self.terminal.configure(state='normal')
        self.terminal.insert('end', message)
        # Bounded view; the main terminal and saved experimental files remain authoritative.
        if int(self.terminal.index('end-1c').split('.')[0]) > 1200:
            self.terminal.delete('1.0', '300.0')
        self.terminal.see('end')
        self.terminal.configure(state='disabled')
        # Terminal text is presentation, not a control signal. Cancellation is
        # driven by explicit Stop/health/lifecycle state in tick()/next_run().

    def next_run(self):
        if self.cancelled.is_set():
            if not quiescent(self.app):
                return
            self.finish('cancelled', 'Stop, fault or cleanup warning cancelled the remaining sequence')
            return
        if len(self.records) == len(self.requests):
            self.finish('completed', None)
            return
        if not ready_for_next(self.app):
            return
        if not self.check_setup_continuity():
            return
        self.request = self.requests[len(self.records)]
        self.status.configure(text=f'Starting {self.request.run_id} ({self.request.index}/{self.request.total})')
        self.launched_at = time.monotonic()
        if not self.app.start_strain_test(mapping_request=self.request):
            if getattr(self,'track_admission_failures',False):
                self.records.append(dict(context=self.request.context(),outcome='failure',
                    error='Normal GUI admission blocked this run',strain_file=None,rotary_file=None,metadata_file=None))
            self.finish('error', 'Normal GUI admission blocked this run; see Session Terminal')
            return
        self.awaiting_run = True

    def finish(self, status, error):
        self.batch_status, self.batch_error = status, error
        self.awaiting_run = False
        # Keep active ownership until the final sidecar is durably saved.
        self.save('finish')

    def stop(self):
        self.cancelled.set()
        if self.active:
            self.status.configure(text='Stop requested — remaining runs cancelled; waiting for normal cleanup')
        # Escape/Stop in this window remains a real application Stop even when
        # the batch is inactive and a normal main-window test is running.
        self.app.stop_logging()

    def tick(self):
        if self.closed:
            return
        app = self.app
        if getattr(self,'failure_finalizing',False):
            self._finish_failed_batch_when_quiescent()
            self.window.after(200,self.tick)
            return
        try:
            self.identity.configure(text=f'Operator: {app.operator_input.get()} | Fixture: {app.fixture_id_input.get()} | '
                                     f'Load calibration: {app.calibration_id_input.get()}')
            if self.active:
                if (getattr(app,'operator_stop_requested',False) and app.test_stop_event.is_set()
                        and not app.monitor_error and not getattr(app,'acquisition_error',None)):
                    self.cancelled.set()
                for v in self.fields.values():v.configure(state='disabled')
                for button in self.run_buttons:button.configure(state='disabled')
                if app.shutdown_requested or app.monitor_error or app.feedback_retry_pending:
                    if not self.cancelled.is_set():
                        self.stop()
                if self.save_future is not None:
                    if self.save_future.done():
                        future, action = self.save_future, self.save_action
                        self.save_future = None
                        try:
                            future.result()
                        except Exception as exc:
                            self.cancelled.set()
                            if app.shutdown_requested and not getattr(self,'_shutdown_manifest_attempted',False):
                                self._shutdown_manifest_attempted=True
                                self.finish('cancelled','Application shutdown interrupted manifest persistence')
                            else:
                                self.active = False
                                self.batch_status='error'
                                self.batch_error=f'Manifest save failed: {exc}'
                                self.status.configure(text=f'Batch manifest save failed; sequence stopped: {exc}')
                                app.update_terminal(f'Batch manifest save failed: {exc}\n')
                        else:
                            if action == 'finish':
                                self.active = False
                                self.request = None
                                self.status.configure(text=f'{self.batch_status.upper()}: {self.batch_error or "all runs saved"}')
                            else:
                                self.next_run()
                elif getattr(self,'awaiting_run',False):
                    state, reason = completed_run(app,self.request)
                    if state != 'pending':
                        self.records.append(dict(context=self.request.context(), outcome=state,error=reason,
                            strain_file=app.strain_file_name,rotary_file=app.rotary_samples_file_name,
                            metadata_file=app.metadata_file_name,
                            started_at=(app.run_metadata or {}).get('started_at'),
                            completed_at=(app.run_metadata or {}).get('completed_at'),
                            completed_cycles=(app.run_metadata or {}).get('completed_cycles'),
                            run_status=(app.run_metadata or {}).get('run_status')))
                        self.awaiting_run = False
                        if state == 'failure' and not self.cancelled.is_set() and self.handle_failed_run(reason):
                            pass
                        elif state == 'failure' or self.cancelled.is_set():
                            self.finish('cancelled' if self.cancelled.is_set() and not app.monitor_error and not getattr(app,'acquisition_error',None) else 'error',reason or 'Stop requested')
                        else:
                            self.save('next')
                    else:
                        s = app.latest_feedback
                        rotary = app.rotary_sensor.measurement()
                        angle = rotary.get('angle_deg')
                        engineering = getattr(app, "show_engineering", None)
                        motor_detail = (
                            f'Motor: {s.position_turns if s else "unavailable"} turns | '
                            if engineering is not None and engineering.get() else ''
                        )
                        self.status.configure(text=f'{self.request.run_id} ({self.request.index}/{self.request.total}) | '
                            f'{self.request.parameters.commanded_afo_speed_deg_s:g}°/s | '
                            f'cycle {app.current_cycle}/{self.request.parameters.cycles} | {app.motion_phase} | '
                            f'{time.monotonic()-self.launched_at:.0f} s elapsed\n'
                            f'{motor_detail}Rotary: {angle} deg | '
                            f'{rotary.get("reason", "")}')
                else:
                    # Subclasses own preparation/recovery cancellation as well as
                    # run dispatch. Do not bypass their evidence finalisation.
                    self.next_run()
            else:
                for button in self.run_buttons:button.configure(state='normal')
                for key,v in self.fields.items():
                    v.configure(state='disabled' if key in getattr(self,'readonly_fields',()) else 'normal')
                if self.release_pending and quiescent(app):
                    self.release_pending = False
                    app.set_test_inputs_state('normal')
                    app._refresh_motion_controls()
        except Exception as exc:
            self.cancelled.set()
            self.failure_finalizing=True;self.fatal_save_started=False;self.fatal_finished=False
            self.batch_status='error';self.batch_error=f'Batch orchestration error: {exc}'
            app.update_terminal(self.batch_error+'; stopping and preserving the batch manifest.\n')
            app.stop_logging()
        self.window.after(200,self.tick)

    def _finish_failed_batch_when_quiescent(self):
        """A UI/scheduler exception must not abandon a running manifest/owner."""
        app=self.app
        if getattr(self,'fatal_finished',False):return
        future=self.save_future
        if future is not None:
            if not future.done():return
            try:future.result()
            except Exception as exc:
                self.batch_error+=f'; manifest write failed: {exc}'
            self.save_future=None
            if self.fatal_save_started:
                self.fatal_finished=True;self.active=False;self.request=None
                if quiescent(app):
                    self.release_pending=False;app.set_test_inputs_state('normal');app._refresh_motion_controls()
                app.update_terminal(self.batch_error+'\n')
                try:self.status.configure(text='ERROR: '+self.batch_error)
                except Exception:pass  # The original failure may be a destroyed widget.
                return
        if not quiescent(app):return
        if self.awaiting_run and self.request is not None:
            m=app.run_metadata or {}
            self.records.append(dict(context=self.request.context(),outcome='failure',error=self.batch_error,
                strain_file=app.strain_file_name,rotary_file=app.rotary_samples_file_name,metadata_file=app.metadata_file_name,
                started_at=m.get('started_at'),completed_at=m.get('completed_at'),
                completed_cycles=m.get('completed_cycles'),run_status=m.get('run_status','error')))
            self.awaiting_run=False
        try:
            hook=getattr(self,'abort_preparation_for_batch_error',None)
            if hook is not None:hook()
            self.fatal_save_started=True;self.finish('error',self.batch_error)
        except Exception as exc:
            self.batch_error+=f'; final manifest could not be written: {exc}'
            self.fatal_finished=True;self.active=False
            app.update_terminal(self.batch_error+'; raw run files remain authoritative.\n')
            if quiescent(app):
                self.release_pending=False;app.set_test_inputs_state('normal');app._refresh_motion_controls()

    def close(self):
        if self.active or self.save_future is not None:
            self.stop()
            self.status.configure(text='Wait for Stop/recording cleanup and batch saving before closing this window')
            return
        self.closed = True
        self.window.destroy()

    def handle_failed_run(self, reason):
        return False

    def check_setup_continuity(self):
        return True

    def submit_manifest(self, name, callback):
        if not self.app.engine.closing:
            return self.app.engine.submit(name,callback)
        # Shutdown closes lifecycle admission. Required final sidecar writes still
        # run off Tk in the app's tracked metadata-worker set; no hardware calls.
        future=Future()
        def persist():
            try:
                future.set_result(callback())
            except Exception as exc:
                future.set_exception(exc)
        worker=threading.Thread(target=persist,name='east-batch-final-manifest',daemon=True)
        self.app.metadata_workers.append(worker)
        worker.start()
        return future
