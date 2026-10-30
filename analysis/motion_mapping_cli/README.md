# Run/Play motion experiment

Open `Testing Scripts/run-motion-mapping.py` and press Run/Play using the normal
EAST Python environment. No command arguments or typed confirmation are required.
The launcher is explicitly configured for **operator SC, fixture ID 0**, using the
operator-confirmed empty fixture already at physical 90 degrees, clear of tools,
with the enclosure/E-stop in normal operating condition. Close the GUI first.
There is a five-second cancellation countdown. **This script commands motion.**

Six interleaved runs: S1/M1/S2/M2/S3/M3, speeds 0.5/5 degrees/s, nominal +/-10 degrees,
100 degrees/s², three numbered min/max cycles per run. The shared existing EAST
motion conversions, ODrive adapter, trajectory mode, reference, ownership,
settling and conservative return profile are used unchanged. This is empty-machine
motor/rotary identification, with no force acquisition or tare; it is not a loaded
AFO test or automatic rotary zero recovery. The sensor currently only measures.

Ctrl+C requests Stop, cancels queued motion and stops the entire series. Use the
physical E-stop if motion persists/idle is unconfirmed. A blocked Fibre call cannot
be interrupted by Python. The worker retains ownership while cleanup retires.

Results: `EAST Logs/motion_mapping/<timestamp>/`, with separate motor and full
rotary-event CSVs and per-run/session metadata. Raw motor-read timestamps and rotary
sequences are retained; channels are not claimed synchronous. Partial/failed records
are preserved. A later GUI session still needs its normal zero/tare preparation.

For a hardware-free preview:

```powershell
.\.venv\Scripts\python.exe "Testing Scripts/run-motion-mapping.py" --preview
```

The source manifest checks compatibility against reviewed baseline 5b558be's
motion/reference/sensor/configuration files. LF/CRLF line-ending differences from Windows Git checkouts are normalised before comparison; actual content differences remain blocked. A content mismatch stops before connection;
review the changed code rather than deleting the guard. New launcher/runner/test
files do not affect that baseline check. No controller persistent configuration,
motor gains, calibration or protection settings are changed.

Offline fake-device tests cover six-run completion, the GUI phase/return order,
nonzero motor coordinates at physical zero, limits, cancellation, rotary loss,
and recording failures. They do not establish hardware accuracy/stop timing.
No hardware execution was performed while developing this script.
