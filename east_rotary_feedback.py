"""First empty-fixture pilot policy. Not a tuned/validated production controller."""
import math
PERIOD_S=.1
GAIN=.3
CORRECTION_DEG=.1
MAX_TRACKING_ERROR_DEG=1.
MAX_COMMAND_RATE_TURNS_S=.75/2.055
ACTUAL_ANGLE_LIMIT_DEG=6.
def step(desired,measured,feedforward,last_target,dt,slope,*,gain=GAIN):
    if not all(math.isfinite(v) for v in (desired,measured,feedforward,last_target,dt,slope,gain)) or dt<=0 or slope<=0 or not 0<=gain<=GAIN:
        raise ValueError('Invalid rotary feedback input')
    error=desired-measured
    if abs(measured)>ACTUAL_ANGLE_LIMIT_DEG or abs(error)>MAX_TRACKING_ERROR_DEG:
        raise RuntimeError('Rotary pilot travel/tracking guard exceeded')
    correction=max(-CORRECTION_DEG,min(CORRECTION_DEG,gain*error))/slope
    target=feedforward+correction
    limit=MAX_COMMAND_RATE_TURNS_S*min(dt,.2)
    command=max(last_target-limit,min(last_target+limit,target))
    return dict(desired_angle_deg=desired,measured_angle_deg=measured,tracking_error_deg=error,
        feedforward_target_turns=feedforward,requested_correction_turns=correction,
        command_target_turns=command,applied_correction_turns=command-feedforward,command_rate_limited=command!=target)
def ramp(start,target,speed,elapsed):
    # Smooth acceleration into and out of the requested 0.5 degree/s ramp.
    distance=abs(target-start);sign=1 if target>=start else -1;acceleration=.5
    ta=min(speed/acceleration,math.sqrt(distance/acceleration));peak=acceleration*ta
    tc=max(0.,(distance-acceleration*ta*ta)/peak) if peak else 0.;duration=2*ta+tc
    t=min(max(elapsed,0.),duration)
    if t<ta:travel=.5*acceleration*t*t
    elif t<ta+tc:travel=.5*acceleration*ta*ta+peak*(t-ta)
    else:travel=distance-.5*acceleration*(duration-t)**2
    return start+sign*travel,duration
