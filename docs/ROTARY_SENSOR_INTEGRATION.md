# Rotary sensor measurement in EAST 1.7.0

The P3022 is an independent measurement channel in normal GUI tests. It does not
command the motor, establish machine zero, tare the load cell, correct the 2.055
conversion or change the torque geometry calculation. Normal tests require all
existing machine-zero/tare conditions and fresh, valid rotary measurements.

## Accepted calibration and provenance

The bundled profile is `calibration/rotary_P3022_20261014.json`, calibration ID
`P3022_RATIO_20261014_V1`. It matches HUB0007 serial 750256, port/channel 0,
voltage-ratio input, the existing fixture-shaft coupling and physical 90° zero.

    physical_angle_deg = 361.80006173743607 * voltage_ratio - 143.89867375834794

The first 12 positions (36 stationary captures) supply the fit, over −5.7° to
+15.4°. Equal weight is given to each position's three-capture mean. Fit RMS
residual is 0.06437°, maximum absolute residual 0.10806°. These are fitting
residuals, not quantified independent validation accuracy.

The operator reported independent remeasurement agreeing with the negative
extensions down to approximately −15.33°. Exact new angle-finder readings were
not supplied. That limitation is retained in the profile. The source dataset
keeps original angle values, derived estimates, recheck status and inclusion
flags. Sensor-derived extension values are excluded from the sensor fit.

Profile and source-data hashes are recorded for reproducibility. Changing the
sensor/coupling or physical reference requires reviewing/replacing the accepted
profile. A new fit in the calibration window does not silently replace it.
For a changed setup, mark the profile status unaccepted and restart EAST until
its calibration is resolved. The software cannot identify a passive sensor's
printed serial or detect every mechanical mounting change electronically.

## Connection and acquisition

The main Connect action attempts the ODrive, load-cell Bridge and rotary sensor.
The sensor remains independent of ODrive reconnect/zero recovery. **Reconnect
Sensor** retries only the sensor channel, without resetting machine zero or tare.
The Bridge channel is shared by tare and normal tests and remains connected
between successful tests.

The main application owns one rotary channel. The calibration window subscribes
to its events. Closing that window does not disconnect the main sensor. Explicit
Disconnect Sensor closes the channel and blocks normal Start. Connection changes
are blocked during active normal tests/capture/movement. Voltage mode remains a
diagnostic option in calibration; normal tests require voltage-ratio mode.

EAST requests an 8 ms interval (125 Hz), clamps to the attached channel's reported
minimum/maximum and reads back the actual configured interval. Trigger is zero.
Only Phidget change events count as new sensor samples, including equal-valued
events at rest. No `getVoltageRatio()` polling is used for the rotary sensor.

Before normal Start, at least 16 delivered events and an observed rate of at
least 100 Hz and 75% of the configured rate are required. Sample age must be
within 100 ms. These settings are in `tester_config.json`, not guarantees that
the Windows computer achieves the requested rate. Hardware bench verification
is still pending.

Loss, stale readings, inadequate rate, buffer overflow or invalid readings stop a
normal test through its existing cancellation/idle path. A 0.25° allowance around
the checked measurement range accommodates reference rounding/noise. This is
a measurement validity allowance, not a new physical travel limit. Existing
enclosure/E-stop, motion limits and timeouts remain active.

## Display and output

The main display shows **Measured Fixture Angle**, delivered event rate and sample
age. The Plot tab uses measured fixture angle on its x-axis. Torque remains the
existing calculation using ODrive-derived angle, explicitly identified in JSON.

Each GUI run produces:

- Existing `*_strain_data.csv`, with appended rotary ratio, angle, event timestamp,
  sequence, connection ID, age, validity/status, calibration ID and observed rate.
  The ODrive feedback timestamp is included to evaluate alignment.
- `*_rotary_samples.csv`, written by a separate worker from every delivered event
  during the recording. This is the authoritative sensor series for speed fits.
- Existing metadata JSON, now schema 3, with the full profile/hash, connection
  interval, source of plot/torque angle, rotary filename, delivered count, rate,
  invalid count and maximum sample gap.

The main load-cell loop may reuse a sensor event or miss intermediate events.
`Rotary New Sample Since Previous Row` and the event index disclose reuse; the
separate sensor file retains intermediate events. No interpolation is performed.
Timestamps describe host callback arrival, not hardware-synchronised sampling.
Invalid sensor events remain in the independent stream even when the test fails.
Existing CSV columns remain in their original order, with rotary fields appended.
The legacy hardware diagnostic CLI retains its motor-only schema; use the GUI
for the integrated verification tests.

## Windows bench verification

Use an empty machine and independently checked physical travel limits. Keep the
enclosure and physical E-stop operative. Begin with stationary and small-motion
checks before the normal speed tests.

1. Pull this version, restart source-based EAST, and close other programs using
   the HUB0007 port. Existing executables require rebuilding.
2. Press Connect. Verify hub 750256, ratio mode, requested/configured 8 ms and
   live angle/rate/age. No fixture motion should occur from connection alone.
3. Verify the fixture physically at 90° using the square/angle finder. Compare
   displayed sensor angle without taring or redefining its calibration. Establish
   normal machine zero and capture the empty-machine tare as usual.
4. In the calibration window, capture three 10-second stationary Validation holds
   at physical zero and selected independently measured positions. Check actual
   event counts, ratio/angle stability, reference agreement and absence of time
   gaps. A 125 Hz stream yields approximately 1,250 events per 10-second hold.
   Record actual counts; do not infer them from the requested interval.
5. Close/reopen the calibration window. Main angle/rate must continue, and no
   competing-channel connection error should occur.
6. Run one initial empty-machine test at ±10°, 1°/s and three software cycles.
   Inspect CSV headers and metadata and confirm both motor and measured angles
   are present. Do not expect the known motor conversion error to be corrected.
7. Repeat at 5, 10 and 20°/s. Analyse the unfiltered separate sensor stream, using
   event timestamps and the central −8° to +8° portion where available. Report
   physical ROM, fitted speed, R², direction and cycle variation, actual event
   rate and maximum gaps. Compare against existing independent checks.
8. Before formal use, carry out controlled Stop/enclosure/E-stop and sensor-loss
   tests under the approved lab procedure. Verify idle, error status, preserved
   data, disabled Start until recovery, and no automatic restart. Perform an
   initial USB-loss check at rest; arrange any motion fault test deliberately.
9. Check reconnect and power-cycle sensor zero with the fixture unchanged. This
   does not yet restore ODrive machine-zero continuity automatically.

Record observed outcomes as pass/fail/not tested. Offline tests cannot establish
Windows layout, actual hardware rate, measurement accuracy under motion or
physical stop response. Quantify independent check errors from actual angle-
finder readings when those are supplied, rather than from sensor-derived values.

## Sources

- [HUB0007 specifications](https://www.phidgets.com/?prodid=1290)
- [Phidget data interval and change trigger](https://www.phidgets.com/docs/Data_Interval/Change_Trigger)
