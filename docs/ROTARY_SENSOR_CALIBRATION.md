# Independent P3022 calibration in EAST

Version: EAST 1.5.5-idle-feedback-retry. This is a stationary, empty-fixture
measurement workflow. It does not apply a calibration to motor control or replace
angles in strain-test CSV files.

## What is measured

At each hold EAST records the P3022 reading, independently measured fixture
angle, raw ODrive turns, repeat and approach direction. It fits two relationships:

- fixture angle = slope × sensor reading + intercept;
- fixture angle = slope × ODrive position + intercept (degrees per motor turn).

Only the operator's independent physical-angle measurement supplies the reference.
EAST's displayed angle, requested angle and 2.055 conversion do not supply the
calibration labels. Existing motor conversion remains in use for motion bounds,
profiles and stationary checks. The proposed fit remains provisional.

## Input mode and supply

Use **Voltage ratio** when the sensor is powered from the same HUB0007 port whose
signal input is measuring it. The Phidget supplies power and measures the ratio
Vout/VCC. A reading of 0.5 is a dimensionless ratio, not 0.5 V. Do not divide a
VoltageRatioInput reading by 5.11 or any other supply value a second time.

Published [P3022-series specifications](https://caltsensor.com/product/miniature-non-contact-angle-sensor-p3022-series/)
list 0–5 V ratio output and 5 V ±10% supply. They support this choice for the
P3022 family; the installed PandAuto unit still needs independent validation.
The [Phidgets ratio guide](https://cdn.phidgets.com/docs/Voltage_Ratio_Input_Guide)
explains supply normalisation, and the [HUB0007 documentation](https://www.phidgets.com/?prodid=1290)
explains the USB-derived sensor supply. Voltage remains an optional diagnostic
mode. The two modes cannot be mixed within one calibration session or opened
simultaneously on the same hub port.

The optional Measured VCC field records a multimeter measurement as metadata.
It never changes ratio readings or calibrates the sensor automatically. Changes
in USB source, cable resistance, current demand or measurement conditions can
change VCC; 5.19 V versus 5.11 V alone does not identify which cause applies.
If the sensor is ratiometric, ratio mode reduces the associated scale error;
it does not remove sensor nonlinearity, mounting slip, noise or reference error.

## Preparation

1. Pull the current repository and restart EAST. Check the displayed version.
   Use the source `Ortho-Sim.py`; an older EAST.exe will not gain this feature
   until it is rebuilt.
2. Remove the AFO. Verify that the P3022 shaft is securely coupled to the moving
   member whose angle is being measured. Inspect for slip, play and cable strain.
3. Close the standalone P3022 logger and any Phidget Control Panel channel window
   using HUB0007. The load-cell Bridge is a separate device; a tare is not needed
   for this angle-only workflow.
4. Connect ODrive, enter Operator ID and Fixture ID, and verify machine zero at
   physical 90° using the engineer's square. Previously verified zero can be
   retained if its validity and physical alignment remain established.
5. Enable Manual Mode. Select **Rotary Sensor Calibration** below its movement
   controls. Enter the HUB0007 serial shown in the Phidget Control Panel; the
   current default is 750256. Leave input mode at **Voltage ratio** and select
   **Connect Sensor**. Wait for fresh raw V/V readings.
6. Record the independent measuring instrument and the actual angular graduation
   spacing. Supply voltage is optional metadata (for example 5.11 V measured in
   the current setup). Decide and record a consistent angle sign convention.

The window can show sensor readings without an ODrive connection, but paired
point capture requires a verified zero, connected ODrive and fresh stationary
feedback. Operator, fixture, zero, hub and mode identify a measurement session.
Changing them requires **New Calibration Session**; previous files are preserved.

## Establish independent angles

Use the motor as a positioning aid. For example, a requested −5° step only takes
the fixture near the intended position; it is not evidence that the actual angle
is −5°. Wait until movement has stopped, then read the actual physical angle.
If it is −4° rather than −5°, enter −4°. Exact nominal targets are unnecessary.
Do not label a point with the EAST display just because the motor moved there.

Align the protractor to consistent rigid fixture surfaces in the rotation plane:
a fixed footplate reference and the moving shank/linkage reference. Establish
90° with the square, then measure angular change from that reference. Assign
positive change to dorsiflexion and negative change to plantarflexion. The
protractor's included-angle reading depends on the chosen surfaces; do not assume
that reading minus 90 always has the required sign.
Measure the same physical joint/member at every point. Photograph the instrument
placement and record its graduation spacing and any reading ambiguity.

Make motor movements with the existing enclosure/interlock arrangement. Place or
adjust the measuring tool only after the motor is idle. Remove it before the next
movement. Do not force the motor or gearbox by hand. The software makes no
interlock changes and adds no new motor-command path.

## Point collection: three repeats in both directions

Use nominal calibration positions −10°, −5°, 0°, +5°, +10° within existing travel
limits. Approach those five positions in increasing order, then decreasing order.
Perform three such pairs of passes. This gives **30 stationary calibration holds**
(5 positions × 2 directions × 3 repeats). Return away and reapproach between passes;
repeated clicks without repositioning do not assess positioning repeatability.
Actual independently measured angles may differ from nominal targets.

For each point:

1. Position approximately with Manual Mode; use smaller bounded steps if helpful.
2. Wait for settled idle feedback and independently measure the actual angle.
3. Enter the signed measured angle, repeat number and approach direction.
4. Select **Calibration** as point purpose and a **10-second** stationary hold.
5. Confirm the empty fixture, independent measurement and stationary condition.
6. Select **Capture Stationary Point**. Do not reposition during the hold.
7. Check that the point was recorded and has sufficient samples. If excluded,
   retain it, investigate the stated reason, then collect a replacement point.

Motor commands are blocked during each capture by EAST's existing motion owner.
For Sensor + motor, Stop/Escape, cancellation, stale feedback, sensor loss,
active motor state, movement and reference changes end the hold. Partial readings remain saved with
an exclusion reason; excluded holds are not used to fit.

A **3-point exploratory check** at negative angle, physical zero and positive
angle can establish that the window works. It is not the complete calibration or
validation protocol above.

## Separate validation and fit

Select **Validation** and collect new holds at approximately −7.5°, −2.5°, +2.5°,
+7.5°, measuring the actual angles again. Use three passes in each direction if
the instrument supports these readings (24 validation holds). If its graduations
are too coarse, choose other independently readable intermediate positions and
record them; do not invent half-degree precision.

Select **Fit + Save Calibration Report**. Calibration holds alone determine the
fit; validation holds are used only to calculate prediction errors. Review:

- local sensor slope, zero reading, fit residuals and calibrated range;
- separate validation errors and flags for extrapolation;
- direction and repeat differences in the point data;
- independent angle-versus-motor-turn slope and residuals, compared with 2.055;
- reference graduation spacing and alignment uncertainty.

No numerical pass threshold is assumed, and a high R² is not proof of accuracy.
A coarse combination-square protractor can support exploratory assessment but
may not distinguish a small percentage scale error over ±10°. For example,
a 1° error over a 20° span is 5% of that span. Use a suitably verified finer
reference or the independent camera method if the required conclusion is finer
than the reference can support. Calibration should span the intended use range;
a full 360° machine rotation is neither required nor requested.

The local linear fit rejects possible sensor rollover rather than averaging
values near 0 and 1. If the operating range crosses rollover, preserve the data;
angle unwrapping or mechanical sensor orientation needs separate review. Do not
rotate the machine beyond its permitted travel to find the full electrical range.

## Output and interpretation

A new folder is created under the configured EAST log directory:

`EAST Logs/Rotary Calibration/rotary_<timestamp>/`

- `session_metadata.json`: instrument, graduation spacing, operator, fixture,
  machine zero, hub, input mode, software version and optional measured supply.
- `raw_samples.csv`: original sensor readings and paired motor feedback with
  host callback timestamps. Each row is flushed as it is recorded.
- `points.csv` / `points.json`: hold means, SD, counts, range and exclusion reasons.
- `calibration_report.json`: provisional sensor fit, independent motor-turn fit
  and separate validation errors. New points mark any previous fit as outdated.

The timestamps are host callback arrival times. This stationary workflow is not
validated for dynamic speed measurement. It does not alter load-cell calibration,
tare, machine zero, 2.055, strain-test output or the existing motion limits.

Older voltage CSVs remain usable as raw electrical-response, noise and rollover
evidence. Vout divided by contemporaneous VCC can estimate a ratio, but one
multimeter reading does not establish the supply at every sample. Do not replace
historical voltages with supposedly measured ratios or treat nominal 72°/V
estimates as independently calibrated physical angles.

Once a validated ratio-to-angle calibration is established, routine VCC changes
within the supported range should not require manually renormalising each run.
Check physical zero and reference behaviour each measurement session, and repeat
calibration/validation after sensor coupling or fixture changes, unexplained
checks failing, or other evidence of drift. The fit is not yet loaded into EAST
or persisted as an automatically trusted angle reference.

## Software verification

Offline calculation, guard and reader-addressing tests; actual GUI rendering,
mocked paired capture/export/cancellation and normal idle shutdown were checked
on macOS without opening hardware. Windows GUI, the installed HUB0007/ODrive
pair and physical measurement behaviour require the first lab bench check.

## If Connect Sensor does not produce readings

Keep the main EAST GUI and its calibration window open. They are one application,
not competing sensor sessions. The connection message names hub serial, input
mode and stage. Progress and errors also appear under **Session Terminal**.

EAST requests attachment with a four-second timeout. An eight-second GUI watchdog
reports a stalled driver/first-reading step and requests cleanup. Connect stays
disabled while that worker still owns the channel, preventing a second channel
from being opened on top of it. Disconnect requests cancellation.

Check that the serial in the window matches the connected HUB0007. Close only
separate programs using its analog port: the standalone sensor logger or a
Phidget Control Panel channel window. Keep the Bridge/ODrive USBs connected.
Check the Phidget Windows driver/library installation if attachment fails.
If it still fails, copy the **ROTARY SENSOR** lines from the Session Terminal;
these identify the failing stage and preserve the actual exception. No failure
message proves that another program is open. Do not change sensor wiring solely
to address a software connection failure.

Stationary readings now use explicit VoltageRatioInput/VoltageInput getters in
the worker at the configured interval (normally 100 ms), rather than depending
on sensor-change callbacks. Repeated unchanged readings are recorded. Host
timestamps remain acquisition-arrival timestamps, not validated dynamic timing.

The built-in analog channel can report `VoltageRatioInput_PORT` or
`VoltageInput_PORT` as its device SKU. These are valid channel identities, not
wrong hubs. EAST validates the parent hub's HUB0007 SKU, hub/channel serial,
port 0 and hub-port-device flag, and records hub and channel SKUs separately.


## Capture after opening the enclosure invalidates motor zero

Select **Sensor only** in the calibration window's capture-mode dropdown.
Enter the angle independently measured relative to physical 90 degrees, e.g.
**-10.1** for 10.1 degrees plantarflexion. Confirm the empty, independently
measured, stationary fixture and capture. This mode does not require a valid
ODrive reference or fresh ODrive feedback. It never sets zero, clears faults,
confirms motor idle or enables movement. The operator must independently confirm
that the fixture stays stationary; when fresh motor feedback exists, active
motor state or movement aborts the hold. Sensor loss, stale sensor readings,
invalid sensor values and operator cancellation still exclude the hold.

The capture reserves passive observation so other motor commands cannot be
submitted during the hold, even if a motor stop is already latched. Stop/idle
state is retained. Existing motor/reference requirements still apply before
subsequent movement. This solves recording at the current physical angle; it
is not a solution to motor reference loss when the enclosure is opened.

`capture_mode` identifies every hold. Sensor-only readings may fit the sensor
against the independent physical angle. Available raw motor turns are retained
with `motor_feedback_status=unverified_observation` and
`motor_feedback_valid=False`; absent feedback is blank and marked unavailable.
They are excluded from motor means and the motor-conversion fit. Only
**Sensor + motor** captures supply verified paired motor measurements.

Within an open session, switching to Sensor only retains the existing physical
calibration session identity even if the motor reference is invalidated. It does
not claim that the saved motor zero is currently valid. Operator, fixture, hub
and input-mode changes still require a new session. Files from older sessions
remain untouched. Restarting EAST creates a new calibration folder; retain both
folders and explicitly reconcile their physical reference/setup before combined
analysis. Do not assume the GUI automatically resumes an older session.

For physical angle sign, use dorsiflexion positive and plantarflexion negative.
Whether an included-angle protractor reads above or below 90 depends on the
chosen surfaces: assign sign by anatomical direction, not automatically by
subtracting 90 from every reading. Document those surfaces and convention.

## Idle feedback delays (1.5.5)

A single ODrive acquisition longer than 150 ms previously stopped the monitor
and invalidated machine zero, including between stationary calibration points.
Acquisition time includes waiting for the adapter I/O lock and reading several
controller properties. A delay alone does not identify a controller reset.

EAST now rejects a late sample and permits up to two consecutive retries only
when the controller is idle, no motion owner is active (or the owner is a
stationary rotary capture), position and velocity remain within the existing
stationary limits, disarm reason is unchanged, and controller uptime advances.
All retry observations must be within 1 second of the last accepted sample.
Late snapshots are diagnostic evidence only: they do not replace the feedback
cache, become calibration measurements, or refresh reference checkpoints.
Motion controls and motor enable are blocked pending fresh feedback. A fresh
sample must confirm the same conditions before the delay status clears.

The 150 ms acquisition limit and 250 ms freshness limit remain unchanged.
Stationary paired captures still reject stale feedback; a hold may need repeating.
Powered movement, missing/reset uptime, changed position/state/disarm reason,
controller errors, failed reads and exhausted retries still use the existing
fault/idle/invalidation path. This feature does not restore a zero already lost
in an older run, establish automatic continuity after reconnect, or bypass the
enclosure interlock. Preserve existing calibration files before restarting.

Bench verification: with the empty machine and enclosure closed, verify fresh
idle readings between holds, use a controlled software test to inject one idle
read delay, and confirm the rejection/restoration messages with zero retained.
Repeat the hold if sampling was interrupted. Sustained delay and powered-motion
fault behavior require controlled verification by the operator; do not disconnect
USB during powered movement as a fault-injection method.
