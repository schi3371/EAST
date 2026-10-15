# EAST ordinary stationary operator checks — 2026-10-15

## Scope and evidence

The fixture remained stationary. No machine zero, tare, arm, jog, test, controller-error
clear, controller configuration save, production timing change, source change, or offline
test run was performed. The operator performed visible GUI actions and reported physical
observations; process and diagnostic checks below are automated observations.

Evidence in this folder:

- `after_idle_stop_runtime_checkpoint.json`: later routine monitor checkpoint after Stop.
  The Stop-specific checkpoint had already been replaced, so this is not a precise
  Stop-completion timestamp.
- `post_idle_stop_escape_stationary_20261015T124410.json`: diagnostic saved after idle
  Stop and Escape.
- `pre_quit_runtime_checkpoint.json`: checkpoint immediately before ordinary Quit.
- `ordinary_quit_final_checkpoint.json`: final checkpoint written during ordinary Quit.

Additional original GUI captures remain at their original application-state paths:

- `C:\Users\schi3371\AppData\Local\EAST\stationary_20261015T125231.json`, SHA-256
  `3F24FAB4B3E102A2D0602DAD7EA6F901E0305A169FD5C33D3C7618F4C8A67EFC`
- `C:\Users\schi3371\AppData\Local\EAST\stationary_20261015T125523.json`, SHA-256
  `67D6F4C04D1F1AA622CC08FDA34CD721BCC99B253A6AAC8EC69EFD057683C4D8`

## Idle Stop and Escape

The operator exercised visible Stop while the controller was already idle, waited for
completion, and then exercised Escape. Each action displayed Stop latched followed by
Motor idle confirmed. The operator reported that controls remained usable. Later
evidence retained controller state 1 (IDLE), zero active errors, no session mapping, and
`recovery_required` reference confidence. No worker remained visibly unresolved. These
are **idle-state input and cleanup checks only**; stopping performance during powered
motion remains NOT TESTED.

A 203 ms rejected feedback capture visible in the terminal occurred at 12:24:19.359,
before this stage's evidence-folder start at 12:24:48.861. It is not attributed to Stop
or Escape.

## Ordinary Quit

The operator used the main-window close route and confirmed ordinary Quit. The window
closed normally without a blocked dialog, exception, or observed delay. Automated
checks found both prior EAST process IDs absent and no remaining `python`/`pythonw`
process. The later fresh-launch process also exited with code 0 after its ordinary Quit,
with no Python worker remaining.

The final checkpoint contained fresh controller IDLE feedback, zero active errors, a
16 ms capture, zero queue/lock wait, and no mapping. It recorded `clean_shutdown: false`,
`continuity_checkpoint: dirty`, motion state `fault`, and reference confidence
`recovery_required`, because machine coordinates were not verified and continuity was
not eligible to be saved. This is preserved as an observed outcome; it was not
accompanied by a surviving worker or blocked-shutdown UI. The old
`power_isolated_exit.marker` predates this stage and was not created or used here.

Blocked shutdown and the power-isolated exit route were not deliberately invoked and
remain NOT TESTED.

## Fresh launch with ODrive absent

Only after clean process exit, the operator unplugged ODrive USB while leaving Bridge
and rotary connections unchanged. EAST was launched fresh and visible Connect was
pressed once. It returned `Connection timed out: ODrive 3943355F3231 was not found`.
No competing discovery was started. The load-cell interface connected and rotary
progressed to ready; the operator reported responsive controls, switching, and resizing.
The saved diagnostic had no motor samples, engine state `connecting`, cumulative GUI
maximum drain/gap 47/687 ms, and valid rotary feedback at 124.63 Hz. Ordinary closure
then completed; the process exited with code 0 and no Python worker remained.

## Restored lab setup

After the absent-device application exited, the operator reconnected ODrive USB,
relaunched EAST, and pressed Connect once. Observed terminal output reported the expected
ODrive S1 serial `3943355F3231`, axis active errors 0, no errors cleared or controller
configuration saved, load-cell connection, accepted rotary calibration, and motion
blocked pending verified machine zero. No operation resumed automatically.

The final diagnostic had engine state `ready`, 2,048 rolling motor observations over
48.203 s, and 42.47 Hz achieved motor rate. Motor gaps were 16/47/63/94 ms
median/p95/p99/maximum. Capture durations were 0/16/47/79 ms; maximum transport queue
wait was 16 ms and maximum adapter lock wait was 0 ms. Rotary was valid and within fit
range at 124.57 Hz. The displayed 115.4 Hz was an earlier settling estimate; 124.57 Hz
is the later saved value. Requested rotary interval remained 8 ms; achieved rate is
reported separately.

EAST was left connected and idle with machine-zero setup required. Force acquisition,
sensor recording, populated live plotting, and powered motion were not active.

## Communication-loss timing review

The earlier evidence records a rejected 218 ms capture at 12:12:26.096 and durable fault
checkpoint completion at 12:12:40.108: an end-to-end interval of 14.012 s. It cannot be
validly partitioned from the retained evidence:

- The first capture exceeded the configured 150 ms evaluation limit.
- Monitor poll wait is 8 ms. A qualifying late idle sample may be retried up to twice,
  with successive evidence inside a 1,000 ms consistency window. The record identifies
  retry 1 but does not timestamp the transient fresh sample or later `pos_vel_mapper`
  failure.
- Coordinated Stop called `request_idle()` with a configured 1.0 s deadline. Fibre
  property calls expose no cancellable per-call deadline; elapsed checks reject only
  after a call returns. Idle confirmation failed on missing `active_errors`, but its
  internal observation timestamps are null.
- No separate timestamps were retained for the fault callback, stop-worker dispatch,
  failed reads, idle-result return, checkpoint submission, or durable write.

Thus 14.012 s contains an unknown combination of retry scheduling, later transport
failure, stop dispatch, idle confirmation, and persistence. Only the 218 ms first
capture and final envelope are measured. No fault was rerun solely to fill this gap.

## Result and remaining work

Completed: idle Stop, idle Escape, ordinary Quit, fresh-launch absent-ODrive Connect
failure, ordinary closure after that failure, and final connected-idle restoration.
Manual Disconnect remains an interface limitation, not a prerequisite for the completed
visible-Connect recovery workflow.

Still NOT TESTED: powered Stop, live force acquisition/populated plotting, powered
motion, motion-time communications loss, deliberate blocked shutdown and power-isolated
recovery, physical machine-zero setup, empty tare, and short/1°/s/5°/s motion runs.
Stationary success does not resolve the reported freezing problem.
