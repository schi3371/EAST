# Changelog

## Unreleased

- Increased the maximum permitted ODrive feedback-capture time to 150 ms after observed 109 ms hardware reads.
- Converted unavailable non-finite ODrive diagnostics to JSON `null` so metadata creation cannot fail on `pos_abs = NaN`.
- Increased the feedback freshness window to 250 ms and require it to exceed the capture-duration limit.
- Replaced the mixed PyQtGraph/Tk live plot with a Tk-native canvas after `QApplication.processEvents()` caused a fatal GIL crash during motion.

## 1.2.1-safety-hardening

- Serialized the final ownership check with every position/state command so a queued worker cannot write a target after Stop is latched.
- Kept motion ownership until the owning worker finishes cleanup; idle confirmation no longer re-enables a replacement worker early.
- Added fail-closed checks for relative, non-circular ODrive setpoints and the required position/velocity mapper scale at connection and arming.
- Required the expected closed-loop state and unchanged disarm reason throughout powered settle dwell; feedback snapshots now record acquisition duration.
- Reconciled Stop and acquisition/flush errors after the acquisition thread exits, and report idle as confirmed only when it was observed.
- Hardened runtime-state schema validation and quarantine of structurally invalid JSON.
- Bound optional recovery/continuity to the full controller identity, reference generation, conversion, and RS485 mode; measured-angle recovery now retains the original physical reference.
- Changed recovery jog accounting from signed displacement to cumulative path length and added post-idle observation to manual neutral return.
- Kept continuity, phase recovery, measured-angle recovery, and watchdog disabled pending bench verification.

## 1.2.0-neutral-recovery - 2026-09-18

- Removed the unsafe assumption that ODrive relative position `0.0` is physical 90 degree neutral.
- Added a persistent, operator-attributed physical-neutral record and a current-session position mapping; normal tests/manual motion are blocked until verified.
- Added a guided recovery dialog with bounded slow jog and a no-motion `Set Neutral` action.
- Added per-user runtime state outside the repository/executable, atomic fsync/replace writes, corrupt-state preservation, ordered generations, audit events, and an exclusive hardware process lock.
- Added serialized ODrive access, continuous finite/fresh feedback monitoring, cancellation generations, single motion ownership, position/velocity settle dwell, and explicit idle confirmation.
- Successful tests now require verified-neutral return, settling, confirmed idle, and post-idle observation. Stop, Escape, faults, and communication errors request idle without automatic return.
- Added neutral-reference snapshots to run metadata and made CSV/metadata write failures visible as run errors.
- Added a read-only live/mock reference inspection tool. Standalone motion diagnostics now require explicit unreferenced authorization and invalidate normal-test trust.
- Implemented optional controller-session continuity, encoder-phase recovery, measured-angle recovery, and watchdog lifecycles; all remain disabled by default pending bench verification.
- Added hardware-independent reference, race, stale/NaN, persistence, phase ambiguity, lock, adapter, and inspection tests plus a bench checklist.
- Updated the GUI preview, Windows builder, README, and verification protocol for the recovery workflow.

- Increased both commanded angle-magnitude limits from 12 deg to 15 deg.
- Kept the provisional maximum commanded speed at 20 deg/s and marked the unchanged 2.055 deg/turn conversion as accepted.
- Added a 5 deg minimum calculated constant-speed span check using commanded speed, editable acceleration, and total ROM.
- Added persistent GUI labels and a live commanded-parameter summary.
- Expanded CSV traceability to include commanded acceleration, angle limits, cycles, and per-move calculated constant-speed span.
- Restored editable acceleration consistently in the GUI and guarded diagnostic scripts.

## 1.1.0 - 2026-08-10

- Defined GUI speed as commanded AFO angular speed in degrees per second.
- Replaced conflicting motion factors with one provisional, configurable degree/turn conversion.
- Applied commanded speed to the ODrive trapezoidal trajectory velocity field.
- Added bounded validation for cycles, speed, acceleration, and angle magnitudes.
- Added interruptible motion timeouts, ODrive error checks, immediate idle stop, and guaranteed idle cleanup.
- Added bounded manual movement and an Escape-key software stop.
- Removed duplicate acquisition threads and duplicate-sample suppression.
- Added monotonic elapsed time, motion phase, commands, raw sensor values, physical units, and ODrive state to CSV output.
- Added operator, AFO, fixture, and calibration identifiers plus per-run JSON metadata.
- Added unique filenames, software/Git traceability, run outcome, and completion summaries.
- Replaced unsafe legacy testing scripts with guarded command-line diagnostics.
- Added offline conversion/calculation tests and a speed-verification analysis tool.
- Documented provisional constants and the required physical verification protocol.
