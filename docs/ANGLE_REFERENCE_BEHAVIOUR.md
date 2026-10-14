# ODrive turns, machine zero and rotary angle

ODrive's `axis0.pos_vel_mapper.pos_rel` is a relative motor-turn coordinate since
controller startup. It is not the physical fixture angle and zero is not
necessarily physical 90°. EAST reads this field; Set Machine Zero never writes
an encoder position or resets that counter.

If the motor coordinate at physical 90° is 7.288 turns, EAST saves that coordinate
in the controller-session mapping. Its current motor-derived angle is:

    (current_motor_turns - 7.288) * 2.055

The displayed angle at that saved position is zero while raw turns remain 7.288.
The existing conversion is approximate; calibrated rotary measurement is
independent of this motor-derived angle.

## Expected behaviour

| Event | Raw ODrive turns | EAST machine-zero behaviour |
|---|---|---|
| Stationary fixture, controller running | Normally unchanged apart from measurement noise | Existing verified mapping retained |
| Movement while controller tracks the motor | Increases/decreases | Same zero coordinate retained |
| Successful test or software idle/Stop without another fault | Does not reset merely because the motor enters idle | Mapping can remain verified |
| Set Machine Zero at physical 90° | Counter is unchanged | Current raw motor coordinate becomes the saved zero |
| Press Connect with a healthy, still-live adapter | Existing session is reused after fresh idle/identity checks | Does not redefine or restore zero; existing state retained |
| Actual USB disconnect/reconnect without controller reset | Usually retains its coordinate if the controller truly kept running | Automatic cross-disconnect recovery remains disabled |
| Restart EAST while ODrive keeps running | Restarting EAST does not itself reset controller turns | Saved evidence is loaded, but cross-session restoration remains disabled |
| Controller power cycle, firmware reboot or nRST reset | A new startup-relative origin is established | Old relative mapping cannot be used automatically |

The documented EAST door/E-stop circuit pulls the ODrive nRST line low. A
controller reset can therefore occur while the main power supply remains on.
Verify the actual lab wiring and reset behaviour during bench commissioning.
Relative feedback will generally begin near zero after startup, but initialization
or motion can occur before the application reads it. Do not assume its displayed
value must be exactly zero at connection.

Reference invalidation concerns whether the saved mapping is trustworthy. It
does not necessarily mean the controller erased its counter or EAST deleted the
saved physical definition. Current `session_continuity_enabled=false` and
`odrive_uptime_units=null` explicitly prevent automatic recovery after an actual
disconnect/restart. Controller uptime units, advancement/reset behaviour and
existing continuity checks must be verified before enabling that recovery.

## Rotary sensor

The P3022 measures current shaft orientation through its Hall sensor; it does not
need the previous run's turn count. Its ratio at physical 90° is approximately
0.3977298209 for the accepted mounting. EAST's saved profile converts that reading
to physical fixture zero:

    fixture_angle_deg = 361.80006173743607 * ratio - 143.89867375834794

The sensor does not intrinsically know what the machine calls 90°. The installed
coupling and stored calibration establish that relationship. Disconnecting or
powering the sensor off does not redefine this saved zero. When powered again,
the same physical position should produce the same calibrated angle within its
measurement uncertainty. Recheck after mounting/coupling/reference changes.
It is a single-turn angular sensor, not a retained multi-turn motor counter.

EAST currently displays/logs the rotary measurement and requires valid samples
for normal tests. It does not yet use this sensor to establish an ODrive zero,
recover that zero after reset or control motion. That next integration needs a
verified method for mapping physical fixture angle to motor turns, especially
away from zero where the four-bar relationship is nonlinear.

## Sampling timestamp correction in 1.7.1

Each delivered sensor event has a unique sequence index and an integer monotonic
nanosecond timestamp. Equal host arrival timestamps are retained and flagged;
they are not a sensor fault and are not changed to synthetic times. The raw
sensor stream, joined main CSV and stationary calibration captures include these
fields. Run metadata reports tied-event counts. A genuine backwards host clock,
stale data, sensor loss, inadequate rate or invalid readings still blocks/stops
normal tests.

For speed analysis, use integer timestamp differences and inspect arrival ties
and gaps. Do not divide adjacent angle differences by a zero time difference.
Regression can retain equal-time observations; these timestamps still describe
host delivery rather than guaranteed hardware sampling instants.

## Sources

- [ODrive Mapper API](https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html)
- [ODrive position reference frames](https://docs.odriverobotics.com/v/latest/manual/control.html#position-reference-frame)
- [P3022 series angle sensor](https://caltsensor.com/product/miniature-non-contact-angle-sensor-p3022-series/)
