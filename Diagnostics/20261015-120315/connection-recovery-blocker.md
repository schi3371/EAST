# EAST stationary connection-recovery check — blocked 2026-10-15

## Scope and starting state

- Stage started: 2026-10-15 12:03:15 AEDT
- Stage stopped: 2026-10-15 12:07:12 AEDT
- Elapsed before stop: 236.759 s
- Repository HEAD: `07ee7f9f245bd1c2ca3215d67d4375a0e6a28b02`
- GUI starting state: connected and idle, status **CONNECTED / MACHINE-ZERO SETUP REQUIRED**
- Operator direction: fixture stationary; no motion, reference/tare, error clearing,
  configuration change, enclosure-interlock action, or motor-power interruption allowed
- Preserved precheck JSON: `precheck_stationary_20261015T115719.json`
- Precheck SHA-256: `4F399D95D9A28682E09A3C6E9C479A738E9687125C97E651AEABAB37EBD7177D`

## Step 1 outcome — BLOCKED before Disconnect

The operator reported that no Disconnect button was visible. Source inspection of the
current committed GUI confirmed the visible main action definitions are Connect, Start,
and Stop (`Ortho-Sim.py` lines 503–507), and the File menu exposes only Help (lines
4872–4877). A `disconnect_odrive()` method exists at line 2168, but no GUI command or
menu item is wired to it.

The verification required the normal GUI Disconnect action. Invoking the method through
an interpreter/automation back door would not test that operator workflow and was not
authorized. Adding a button would be a production-code patch during diagnostic work and
was explicitly out of scope. Therefore Disconnect was not attempted.

- Idle confirmation during Disconnect: **NOT TESTED**
- Disconnect worker retirement: **NOT TESTED**
- ODrive transport retirement: **NOT TESTED**
- Hardware-resource release: **NOT TESTED**
- Post-Disconnect GUI responsiveness: **NOT TESTED**
- Cleanup unresolved: no cleanup was initiated; the blocker is the missing GUI action

The existing GUI processes remained running as expected. EAST was left connected in its
pre-stage stationary idle state; no alternate disconnect path was invoked.

## Remaining requested steps

Because Step 1 could not start, the stop condition was applied before physical USB work:

- Connect with ODrive absent: **NOT TESTED**
- USB reconnect and fresh Connect: **NOT TESTED**
- Communication loss while connected: **NOT TESTED**
- Recovery after communication loss: **NOT TESTED**
- Connection generation change, retired-publication rejection, feedback-health recovery,
  and worker retirement on real hardware: **NOT TESTED**
- Physical ODrive USB unplug/replug actions: none performed

No source, tests, configuration, calibration, timing limits, or original recordings were
changed. The offline suite was not run. This finding does not resolve the reported
freezing problem.
