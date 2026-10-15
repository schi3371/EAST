# EAST stationary ODrive + Bridge + rotary diagnostic — 2026-10-15 11:40 AEDT

## Operator prerequisites and scope

Immediately before interface opening, the operator physically confirmed that the
fixture remained stationary and EAST and all other hardware applications remained
closed.

- Repository HEAD: `07ee7f9f245bd1c2ca3215d67d4375a0e6a28b02`
- Python: 3.10.21
- Mode: real hardware, read-only ODrive + Bridge + raw rotary recording
- Requested duration: 30 s
- Requested interval: 8 ms (125 Hz requested motor/Bridge schedule)
- Capture evaluation limit: 150 ms
- Motion, arming, homing, reboot, error clearing, or configuration save: none requested
- Application configuration, calibration, production limits, and prior evidence: unchanged

Command:

```powershell
.\.venv\Scripts\python.exe analysis\diagnose_east_timing.py --real-hardware --duration 30 --interval-ms 8 --capture-limit-ms 150 --with-sensor --sensor-recording "Diagnostics\20261015-114046\bridge.csv" --rotary-recording "Diagnostics\20261015-114046\rotary.csv" --output "Diagnostics\20261015-114046\motor-sensors-150.json"
```

## Evidence integrity

| File | SHA-256 |
|---|---|
| `motor-sensors-150.json` | `FDC41C7279B18D59E03BA5A89A21950CDC75F8DB8C7FD44DAA9C062CDCB7597D` |
| `bridge.csv` | `CA8C30F37C9CE4788CBC479E9B5AC98297DD37B67B21175F1DB7C188D2C2798D` |
| `rotary.csv` | `6F55757ADE58BE561F9C2298E3D68142696497E532305DC668624BF7411E50FF` |

Original host and sensor timestamps are retained in the CSV files. No prior file was
overwritten.

## Motor result

- Successful samples: 2,073
- Setup/acquisition exceptions: 0
- Achieved rate: 69.1035 Hz, not the requested 125 Hz
- Gaps: median 16 ms; p95 31 ms; p99 62 ms; maximum 79 ms
- Complete snapshots: median 0 ms; p95 16 ms; p99 47 ms; maximum 78 ms
- Adapter lock wait: median/p95/p99/maximum 0 ms
- Transport queue wait: unavailable in this standalone diagnostic because it uses the
  adapter directly rather than the GUI's `TransportOwner`

Property-read timing:

| Property | Median | p95 | p99 | Maximum |
|---|---:|---:|---:|---:|
| Position | 0 ms | 15 ms | 16 ms | 63 ms |
| Velocity | 0 ms | 15 ms | 16 ms | 31 ms |
| Active-errors read | 0 ms | 15 ms | 16 ms | 63 ms |
| Current-state read | 0 ms | 15 ms | 16 ms | 63 ms |
| Disarm-reason read | 0 ms | 16 ms | 16 ms | 47 ms |
| Uptime read | 0 ms | 15 ms | 16 ms | 63 ms |
| Bridge read | 0 ms | 0 ms | 0 ms | 16 ms |

The JSON's zero `error_count` means no setup/acquisition exception. It is not an
aggregate assertion that every sampled controller active-error field was zero.

## Bridge recording

- Rows written: 2,073, matching the accepted motor snapshot count
- Timestamp span: 29.968 s
- Achieved recorded rate from preserved host timestamps: 69.1404 Hz
- Host-timestamp gaps: median 15 ms; p95 32 ms; p99 63 ms; maximum 79 ms
- Equal host-monotonic timestamps: 779 adjacent pairs, reflecting Windows timestamp
  quantization rather than invented ordering; row order is retained
- Sensor/recorder errors: none reported
- Recorder completion: `finish()` returned and the diagnostic process exited

The Bridge is read once after each accepted motor snapshot in this diagnostic, so its
achieved rate follows the combined loop rather than an independent 125 Hz callback.

## Rotary recording

- Delivered events: 3,751
- Invalid events: 0
- Validity/status: all recorded rows valid and `within_fit_range`
- Accepted calibration identifier: `P3022_RATIO_20261014_V1`
- Achieved event rate: 125.0 Hz over the 30 s recording
- Maximum host-monotonic inter-event gap: 47 ms
- Events tied to the preceding host-monotonic timestamp: 1,896
- Recording error: none
- Recorder completion: final statistics were produced, the rotary interface was shut
  down by the diagnostic, and the process exited

The rotary service is callback-driven independently of the motor/Bridge loop. Its CSV
preserves higher-resolution ISO event timestamps, raw host-monotonic nanoseconds,
sample indices, connection identity, and explicit tie flags. Quantized Windows
host-monotonic values cause many ties even though callback events remain ordered and
the event-rate statistic is 125 Hz.

## Comparison with motor-only capture

Baseline: `Diagnostics/20261015-113317/motor-150.json`.

| Metric | Motor only | With Bridge + rotary | Difference |
|---|---:|---:|---:|
| Successful motor samples | 2,165 | 2,073 | -92 (-4.25%) |
| Motor achieved rate | 72.1718 Hz | 69.1035 Hz | -3.0683 Hz (-4.25%) |
| Gap median | 16 ms | 16 ms | 0 ms |
| Gap p95 | 31 ms | 31 ms | 0 ms |
| Gap p99 | 47 ms | 62 ms | +15 ms |
| Gap maximum | 94 ms | 79 ms | -15 ms |
| Snapshot p95 | 16 ms | 16 ms | 0 ms |
| Snapshot p99 | 47 ms | 47 ms | approximately 0 ms |
| Snapshot maximum | 63 ms | 78 ms | +15 ms |
| Lock-wait maximum | 0 ms | 0 ms | 0 ms |
| Acquisition exceptions | 0 | 0 | 0 |

Adding the Bridge and rotary service produced a measurable but modest 4.25% reduction
in achieved motor rate. Typical timing (median and p95) was unchanged. Tail results
were mixed: gap p99 and maximum snapshot duration increased by 15 ms, while the single
largest motor gap was 15 ms lower. All accepted combined snapshots remained below the
150 ms evaluation limit, recorders reported no persistence error, and the process
exited with no surviving diagnostic worker. This single stationary pair does not prove
that Phidgets have no material effect under repetition, the GUI, or powered motion.

## Remaining stages

- GUI Terminal/Plot stationary comparison and GUI diagnostic JSONs: **NOT TESTED**
- Connection failure and stationary reconnect/recovery: **NOT TESTED**
- Stop/Escape, Quit, and visible isolation-exit workflow: **NOT TESTED**
- Physical machine zero, empty tare, and motion runs: **NOT TESTED**

The next outstanding stage is the stationary GUI Terminal/Plot comparison. It was not
started in this session.
