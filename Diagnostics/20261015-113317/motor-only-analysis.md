# EAST stationary motor-only diagnostic — 2026-10-15 11:33 AEDT

## Operator prerequisites

The operator reported that the fixture was stationary and that all other ODrive,
Phidget, and hardware applications were closed before this diagnostic began.

## Scope

- Repository HEAD: `07ee7f9f245bd1c2ca3215d67d4375a0e6a28b02`
- Python: 3.10.21
- Mode: real hardware, read-only, motor-only
- Requested duration: 30 s
- Requested interval: 8 ms (125 Hz requested schedule)
- Capture evaluation limit: 150 ms
- Bridge/Phidget opened: no
- Rotary sensor opened: no
- Motion, arming, homing, reboot, error clearing, or controller configuration save: none requested
- Production timing limits changed: no

Command:

```powershell
.\.venv\Scripts\python.exe analysis\diagnose_east_timing.py --real-hardware --duration 30 --interval-ms 8 --capture-limit-ms 150 --output "Diagnostics\20261015-113317\motor-150.json"
```

## Recorded result

- Raw result: `motor-150.json`
- Raw-result SHA-256: `B36563EB8036E38541FD08CF263052D2E58600557A5641656FDA9067F3F2D4F8`
- Successful samples: 2,165
- Diagnostic acquisition exceptions: 0
- Achieved rate: 72.1718 Hz (57.74% of the requested 125 Hz schedule)
- Sample gaps: median 16 ms; p95 31 ms; p99 47 ms; maximum 94 ms
- Complete snapshot duration: median 0 ms; p95 16 ms; p99 47 ms; maximum 63 ms
- Adapter lock wait: median/p95/p99/maximum 0 ms

Property-read maxima:

| Property | p95 | p99 | Maximum |
|---|---:|---:|---:|
| Position | 15 ms | 16 ms | 62 ms |
| Velocity | 15 ms | 16 ms | 16 ms |
| Active errors read | 0 ms | 16 ms | 47 ms |
| Current state read | 15 ms | 16 ms | 16 ms |
| Disarm reason read | 15 ms | 16 ms | 63 ms |
| Uptime read | 15 ms | 16 ms | 47 ms |

## Interpretation

The diagnostic completed without an acquisition exception, and every accepted
snapshot stayed below the 150 ms diagnostic evaluation limit. No adapter lock
contention was observed. The requested 8 ms schedule did not produce 125 Hz: the
observed median gap was 16 ms and the achieved rate was 72.17 Hz. Timing is visibly
quantized near the Windows clock interval, with occasional longer property reads and
a 94 ms maximum sample gap. This motor-only result is evidence for stationary
read-only operation only; it does not validate motion, Stop, reconnect, shutdown, the
GUI, Bridge/rotary coexistence, or resolution of the reported freezing problem.

The JSON's `error_count` counts acquisition/setup exceptions. It does not provide a
separate aggregate of the controller's active-error values, so zero diagnostic errors
must not be relabelled as proof that every sampled controller error field was zero.

## Remaining bench stages

- Stationary ODrive + Bridge + rotary recording: **NOT TESTED**
- GUI Terminal and Plot comparison: **NOT TESTED**
- Connection/recovery, Stop, Quit, and isolation-exit workflow: **NOT TESTED**
- Physical machine zero, empty tare, and all motion runs: **NOT TESTED**
