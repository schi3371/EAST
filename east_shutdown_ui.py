"""Visible blocked-shutdown action, constructed only by the GUI thread.

Factory injection allows offline tests to exercise the same widget construction
and acknowledgement gate without importing Tk or running physical hardware.
"""
class BlockedShutdownDialog:
    def __init__(self, factory, master, reason, on_isolated):
        self.window = factory.CTkToplevel(master)
        self.window.title('EAST — Shutdown blocked')
        self.window.transient(master)
        self.window.geometry('620x330')
        self.window.resizable(True, True)
        self.window.grid_columnconfigure(0, weight=1)
        self.window.grid_rowconfigure(0, weight=1)
        self.reason = factory.CTkLabel(self.window, text=(reason + '\n\n'
            'A hardware call or cleanup may still be blocked. New motion is locked. '
            'Use the physical E-stop and isolate motor power before exiting. '
            'This is an operator acknowledgement, not electronic proof of idle. '
            'The run remains incomplete and buffered data may be lost.'),
            wraplength=550, justify='left')
        self.reason.grid(row=0, column=0, sticky='nsew', padx=16, pady=12)
        self.estop = factory.BooleanVar(value=False)
        self.isolation = factory.BooleanVar(value=False)
        factory.CTkCheckBox(self.window, text='I have engaged the physical E-stop',
                          variable=self.estop, command=self._refresh).grid(row=1, column=0, sticky='w', padx=16, pady=6)
        factory.CTkCheckBox(self.window, text='I have isolated and verified the motor supply is off',
                          variable=self.isolation, command=self._refresh).grid(row=2, column=0, sticky='w', padx=16, pady=6)
        # Always in the dialog itself, below the text; never a scrollable panel.
        self.exit = factory.CTkButton(self.window, text='Exit After Motor Power Is Isolated',
                                     command=self._confirm, state='disabled', fg_color='#b91c1c')
        self.exit.grid(row=3, column=0, sticky='ew', padx=16, pady=16)
        self.on_isolated = on_isolated
        self.window.protocol('WM_DELETE_WINDOW', self.window.withdraw)
        self.window.bind('<Configure>', self._resize)
        self.window.lift()

    def _resize(self, event):
        if event.widget is self.window:
            self.reason.configure(wraplength=max(180, event.width-50))

    def _refresh(self):
        self.exit.configure(state='normal' if self.estop.get() and self.isolation.get() else 'disabled')

    def _confirm(self):
        if self.estop.get() and self.isolation.get():
            self.exit.configure(state='disabled')
            self.on_isolated()

    def show(self):
        self.window.deiconify()
        self.window.lift()
