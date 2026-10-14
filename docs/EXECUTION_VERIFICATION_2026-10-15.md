# Offline execution verification — 15 October 2026

**Physical Windows behaviour: NOT TESTED.** No physical hardware was automatically
opened or operated. This report does not conclude that the original freeze is fixed.

Implementation checkpoint: `3486fe0`; diagnostic baseline: `4b4f160`.
Reviewed starting HEAD: `f6dfdb9`, with the incoming partial working changes retained.
The remote was fetched before delivery; it had not advanced past the baseline.

## Evidence inspected

The original 14 October metadata was read without changing it:

| Record | Observed stored outcome |
|---|---|
| `RS_0_20261014T200736_923698+1100` | Error: 0.266 s-old motor feedback; 251 recorded samples; zero completed cycles |
| `RS_0_20261014T200838_489473+1100` | Completed; 17,381 strain samples; three completed cycles |
| `RS_0_20261014T201139_552211+1100` | Still marked started; no final sample/cycle counts |

These support the handover's concern about timing and finalisation, not a diagnosis
of Windows CPU saturation or USB failure. Earlier `bf0fbb3` remains a comparison
baseline; no motor conversion or timing-limit correction was inferred from it.

## Verification performed

- Full root suite: **240 passed** on Python 3.14 and **240 passed** on the repository
  Python 3.11 environment, run against the committed application checkout.
- Compilation of changed application/service/diagnostic modules: passed.
- Complete branch whitespace/diff checks: passed.
- Simulated soak: **300 operations**, 240 normal completions and 60 Stop sequences;
  900 rows persisted, no reported errors, no remaining workers. The final run's
  maximum observed transport/recording pending queues were one/one; bounds are
  also exercised by deliberate backlog tests. These are simulated operations,
  not independent physical reliability trials.
- Headless motor + Bridge CSV + raw rotary CSV smoke: simulated, no acquisition or
  recording errors. Actual simulated rotary rate was reported below 100 Hz rather
  than relabelled as 125 Hz. Diagnostic mode retains those events; production
  startup/freshness/rate protections are unchanged.

The new tests use actual transport, lifecycle, acquisition, checkpoint and CSV
services. Full-application integration imports MyInterface intact with a fake view
and fake devices. It exercises a completed test, early initialisation failure,
stopped-monitor recovery, lost-object replacement, blocked-transport Quit and
acknowledged dirty exit. Both installed Python environments ran these tests.
Legacy GUI contract regressions were updated for the extracted service boundaries;
source-string assertions alone are not the verification basis.

A native Mac GUI smoke attempt exited with code 134 without diagnostic output.
Native rendering and Windows scaling are unverified. The shutdown dialog's actual
construction/acknowledgement gate is exercised with fake widgets, and the Windows
checklist explicitly requires checking visibility at laptop scaling.

## Complete-change review

The review checked lock ordering, queued cancellation, surviving worker ownership,
reference/tare invalidation, file durability, Stop escalation, idle provenance,
shutdown admission and GUI-thread I/O. It found and corrected:

1. Cached idle could outlive successful arming and falsely fail an immediate settle
   check. The actual closed-loop observation is now published before settling;
   older monitor bundles cannot replace it.
2. Motion admission could block behind an already executing command. Admission
   now rejects promptly while that command retains its worker/ownership.
3. A lost Fibre object could be retried indefinitely. Its dispatcher must exit
   before rediscovery gets a new generation; the frame remains unverified.
4. An expired calibration recording deadline could strand its passive guard.
   Ownership remains until the writer exits, then retires without accepting the
   incomplete point.
5. Engine completion could erase a newly submitted operation's busy state. Busy
   retirement is generation-checked, and stale completion events cannot reopen
   fields during newer service work.
6. A later measurement/disk failure could obscure actual idle evidence. Idle,
   feedback health and persistence outcomes now remain separately recorded.

Callback and transitive-helper review found no hardware reads, durability waits
or worker joins in the relevant GUI callback paths. Plot draining is bounded and
redraws are batched. Widget states change only when necessary. Native Tk redraw
cost and real Windows transport/storage response still require bench evidence.

## Preserved and limited

The 2.055 conversion, torque source, accepted ratio calibration, original streams,
filename behaviour, limits and physical safety semantics are unchanged. The
accepted calibration file was compared byte-for-byte with the original checkout.
The two incoming untracked P3022 bench-report files and original experimental data
were excluded from commits.

Fibre calls remain uninterruptible through this API. A blocked call retains its
transport; the responsive shutdown dialog is a route to acknowledged physical
isolation and bounded best-effort exit, not electronic protection. Automatic
continuity/phase recovery/watchdog configuration remains disabled. Recovery of an
unproven frame requires physical zero verification and matching tare. No test is
automatically resumed. Motion scripts retain their existing dedicated owned
workers; hardware dispatch, acquisition, recording and lifecycle submission are
now service boundaries.

If outcome persistence failed after cleanup, Connect retries the incomplete/error
outcome off Tk after confirming the previous run no longer owns workers. It does
not reclassify a failed run as completed or restart it. Storage that remains blocked
continues to block finalisation/movement admission where required.

Reproduce offline checks from the repository root:

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -q
.\.venv\Scripts\python.exe analysis\simulate_east_soak.py --iterations 300 --output simulated-soak.json
.\.venv\Scripts\python.exe analysis\diagnose_east_timing.py --duration 10 --output simulated-timing.json
```

Hardware procedures and exact Windows pull/restart/diagnostic commands are in
[the bench checklist](bench-tests/EAST_EXECUTION_BENCH_TEST_2026-10-15.md).
Detailed machine-readable offline results are in
[the verification JSON](verification/east_execution_offline_20261015.json).
