# EAST planned ODrive communication-loss recovery — 2026-10-15

## Scope

This check corrected the earlier procedure to use only the existing visible Connect
action. The missing manual Disconnect control remains documented separately in
`Diagnostics/20261015-120315/connection-recovery-blocker.md`; no control was added and
`disconnect_odrive()` was not invoked indirectly.

The operator kept the fixture stationary and changed only the ODrive USB connection.
Bridge and rotary connections remained unchanged. No enclosure, motor-supply, E-stop,
sensor-coupling, motion, machine-zero, tare, error-clear, configuration-save, Quit, or
source/configuration action was performed.

## Fresh precheck

Visible Connect completed, followed by a stationary diagnostic save at 12:10:54 AEDT.
A separately preserved runtime checkpoint at 12:11:40.849 provided the required fresh
controller evidence:

- controller state: 1 (IDLE)
- active errors: 0
- motion state: idle
- position/feedback capture time: 12:11:40.849 AEDT
- queue wait / lock wait / complete capture: 0 / 0 / 0 ms
- engine state: ready
- rotary: connected, valid, ready, 124.57 Hz
- reference confidence: `recovery_required`; session mapping absent

Evidence:

- `precheck_stationary_20261015T121054.json`, SHA-256
  `C2B499006B440A09813CE8CC6970542AB42A7DB3DF65CE5D0FCFAEE720A3CE50`
- `precheck_runtime_checkpoint.json`, SHA-256
  `84FD7405625FA6C604D0D0F9EA2D0A312E8C79CFEA93D4D2441C77849AFAFCB5`

## Communication-loss detection

The operator unplugged only ODrive USB after the precheck. Physical unplug time was not
machine timestamped; it occurred after the instruction at 12:11:50.531 and before the
operator's reply. Therefore exact physical-to-detection latency is unavailable and is
not invented.

Recorded sequence:

1. 12:12:26.096 — a 218 ms feedback capture was rejected as delayed, retry 1, with
   motion blocked pending fresh feedback.
2. The GUI displayed a transient “Fresh idle feedback restored; machine zero retained.”
   This preceded final link-loss detection and is not treated as post-loss idle proof.
3. The monitor then failed because the unplugged Fibre interface no longer exposed
   `pos_vel_mapper`.
4. Stop latched and requested idle confirmation.
5. Idle confirmation failed because the interface no longer exposed `active_errors`.
   The GUI displayed **Motor idle NOT CONFIRMED**.
6. 12:12:40.108 — the fault checkpoint was persisted, 14.012 s after the first rejected
   capture: feedback absent, motion state `fault`, reference confidence `fault`, session
   mapping absent, and no fresh idle observation.

The GUI remained responsive. Historical pre-unplug IDLE evidence was not represented as
fresh confirmation. Fault evidence: `communication_loss_fault_checkpoint.json`,
SHA-256 `957FCED831025CB9A92A2CC9FEC99824F9159D2646607998E9CDC742776CFC7E`.

## Visible Connect while ODrive remained absent

Before Connect, Ctrl+Shift+D completed at 12:14:11, demonstrating that the lifecycle
queue could accept and finish a diagnostic save. Visible Connect was then pressed once;
no competing retry was started.

The retained adapter failed fresh idle validation:

> Retained connection could not supply healthy feedback: idle confirmation failed:
> 'EmptyInterface' object has no attribute 'active_errors'. Rediscovering while motion
> stays blocked.

Rediscovery then ended with the actionable failure:

> Connect/recover failed: Recovery discovery failed or was cancelled; reference remains
> invalid.

Controls became usable again, and tab switching/resizing remained responsive. Evidence:

- `pre_absent_connect_stationary_20261015T121411.json`, SHA-256
  `323C7B679040A044FCD198886A08A3822EE8438544F5419734B855361F76AF55`
- `absent_connect_failure_stationary_20261015T121553.json`, SHA-256
  `9669A01F95045AEBCA228ACC4E4A7F6946ED13C2FD62F962A4E5D1651ECD4D6E`

This was an in-session recovery attempt with ODrive absent, not a fresh application
launch with ODrive absent.

## USB reconnect and visible Connect recovery

The operator reconnected only ODrive USB. After the prior operation had completed,
visible Connect was pressed once. The retained adapter was not reused:

> Retained connection could not supply healthy feedback: Connection retired or
> replaced. Rediscovering while motion stays blocked.

Recovery then completed:

> Monitoring recovered; coordinate continuity not proven. Verify physical machine zero.
> No test was restarted.

Rotary reconnected/validated at a displayed 125.6 delivered Hz. Controls and tab
switching remained responsive, and the operator observed that no test or positioning
operation resumed automatically.

Final status: **MONITOR RESTORED / MACHINE-ZERO SETUP REQUIRED**.

Fresh checkpoint evidence after recovery:

- controller state: 1 (IDLE)
- active errors: 0
- motion state: idle
- engine state: ready
- reference confidence: `recovery_required`
- reference reason: monitoring recovered; coordinate continuity not proven
- session mapping: absent
- latest capture / queue wait / lock wait in final GUI JSON: 0 / 0 / 0 ms
- rotary: connected, valid, ready; same rotary connection identity; approximately
  124.57 Hz in the saved JSON

The old transport must return successfully from `close()` before the replacement path
can proceed. Recovery reaching rediscovery therefore demonstrates retirement of the old
transport worker rather than concurrent reuse. The successful final diagnostic and
continuing new feedback show that the replacement monitor is active. No stale old
monitor fault was observed overwriting recovered feedback health.

The GUI diagnostic schema does not persist the numeric ODrive connection generation.
From the executed code path, the initial GUI connection was generation 1 and the one
successful replacement incremented it once to generation 2. The failed absent-device
rediscovery did not increment because no device was found. This 1 → 2 value is therefore
an implementation-path inference, not a directly serialized diagnostic field.

Validated recovery clears the feedback-fault latch before starting the replacement
monitor. The ready lifecycle, fresh error-free IDLE checkpoint, continued new samples,
and absence of a recurring stale fault are external evidence of that result; the latch
value itself is not serialized in the GUI diagnostic schema.

Final evidence:

- `recovered_stationary_20261015T121851.json`, SHA-256
  `15BF2208B29BE1380E6B9BC63A3E0DD41C29D3814D7F91D6D6BCD592D5CAE4D3`
- `recovered_runtime_checkpoint.json`, SHA-256
  `5E9CAF72C027F338AEFB15D162C88B7991DD575BE59A2003A992EEAB6C445663`

## Result and remaining scope

Planned connected USB-loss detection, one absent-device recovery attempt, USB reconnect,
and visible Connect recovery: **COMPLETED**. Recovery was fail-closed, did not reuse stale
idle evidence, retired/replaced the old transport, restored fresh monitoring, and did
not restore unproven machine coordinates or restart an operation.

Still pending separately:

- manual Disconnect (not exposed in the operator interface)
- fresh-launch Connect with ODrive absent
- explicit Stop/Escape
- Quit and blocked-shutdown/isolation workflow
- live populated plotting and force acquisition
- powered motion and motion-time communication failure

No source, tests, configuration, calibration, timing limits, or original recordings were
changed. The offline suite was not run. This stationary recovery success does not resolve
the reported freezing problem.
