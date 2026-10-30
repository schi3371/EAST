"""Modeless provisional homing controls; the app owns all motor execution."""
import customtkinter as ctk
from east_rotary_homing import HomingPolicy


class RotaryHomingWindow:
    def __init__(self, master, app):
        self.app, self.closed, self.active = app, False, False
        self.future = None
        self.window = ctk.CTkToplevel(master)
        self.window.title('EAST — Provisional Rotary Homing')
        self.window.geometry('660x490')
        self.window.minsize(600, 450)
        self.window.transient(master)
        self.window.protocol('WM_DELETE_WINDOW', self.close)
        self.window.bind('<Escape>', lambda e: app.stop_logging())
        frame = ctk.CTkFrame(self.window)
        frame.pack(fill='both', expand=True, padx=14, pady=14)
        ctk.CTkLabel(frame, text='Provisional Rotary Homing', font=('Arial', 20, 'bold')).pack(pady=8)
        ctk.CTkLabel(frame, text='Empty fixture only. Uses the accepted rotary calibration to approach sensor 0°.\n'
            'Does not replace, save or approve machine zero. No load-cell tare is required.\n'
            'Recover ODrive feedback first. Keep enclosure protection and E-stop in use.',
            wraplength=590, justify='left').pack(pady=8)
        self.live = ctk.CTkLabel(frame, text='Reading sensor…', wraplength=590)
        self.live.pack(pady=6)
        ctk.CTkLabel(frame, text='Sensor stopping band ±degrees (provisional; not absolute accuracy)').pack()
        self.tolerance = ctk.CTkEntry(frame)
        self.tolerance.insert(0, '0.10')
        self.tolerance.pack(pady=5)
        self.ack = ctk.BooleanVar(value=False)
        ctk.CTkCheckBox(frame, text='Fixture empty; sensor mounting unchanged; clear for movement', variable=self.ack).pack(pady=8)
        self.run_button = ctk.CTkButton(frame, text='Run Provisional Homing', command=self.start)
        self.run_button.pack(pady=5)
        ctk.CTkButton(frame, text='Stop', command=app.stop_logging, fg_color='#b91c1c').pack(pady=5)
        self.status = ctk.CTkLabel(frame, text='Original Machine Zero Setup remains available in the main window.',
                                   wraplength=590)
        self.status.pack(pady=8)
        self.window.after(200, self.tick)

    def start(self):
        if self.active:
            return
        try:
            policy = HomingPolicy(tolerance_deg=float(self.tolerance.get())).validate()
            if not self.ack.get():
                raise ValueError('Confirm the empty fixture, unchanged mounting and clear mechanism')
            future = self.app.start_rotary_homing(self, policy)
            if future is not None:
                self.future = future
                self.active = True
                self.ack.set(False)
                self.status.configure(text='Running — progress and file paths are in Session Terminal.')
        except Exception as exc:
            self.status.configure(text=f'Cannot start: {exc}')

    def tick(self):
        if self.closed:
            return
        value = self.app.rotary_sensor.measurement()
        text = f'Accepted measured angle: {value["angle_deg"]:+.3f}°' if value['valid'] else f'Sensor unavailable: {value["reason"]}'
        self.live.configure(text=text)
        if self.future is not None and self.future.done():
            future, self.future = self.future, None
            self.active = False
            try:
                result = future.result()
                self.status.configure(text=result['message'])
            except Exception as exc:
                self.status.configure(text=f'Homing failed: {exc}; see Session Terminal.')
            self.app.set_test_inputs_state('normal')
            self.app._refresh_motion_controls()
        self.run_button.configure(state='disabled' if self.active else 'normal')
        self.tolerance.configure(state='disabled' if self.active else 'normal')
        self.window.after(200, self.tick)

    def close(self):
        if self.active:
            self.app.stop_logging()
            self.status.configure(text='Stop requested. Wait for cleanup before closing.')
            return
        self.closed = True
        self.window.destroy()
