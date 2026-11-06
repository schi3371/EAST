"""First empty-fixture pilot policy. Not a tuned/validated production controller."""
import math
PERIOD_S=.1
GAIN=.3
CORRECTION_DEG=.1
MAX_TRACKING_ERROR_DEG=1.
MAX_COMMAND_RATE_TURNS_S=.75/2.055
ACTUAL_ANGLE_LIMIT_DEG=6.
def step(desired,measured,feedforward,last_target,dt,slope,*,gain=GAIN,
         max_command_rate_turns_s=MAX_COMMAND_RATE_TURNS_S,
         tracking_guard_deg=MAX_TRACKING_ERROR_DEG, travel_guard_deg=ACTUAL_ANGLE_LIMIT_DEG,
         tracking_error_action="stop"):
    if not all(math.isfinite(v) for v in (desired,measured,feedforward,last_target,dt,slope,gain)) or dt<=0 or slope<=0 or not 0<=gain<=GAIN:
        raise ValueError('Invalid rotary feedback input')
    limits = (max_command_rate_turns_s, tracking_guard_deg, travel_guard_deg)
    if not all(math.isfinite(value) and value > 0 for value in limits):
        raise ValueError("Invalid feedback commissioning guard")
    if tracking_error_action not in ("stop", "record"):
        raise ValueError("Invalid tracking error action")
    error=desired-measured
    if abs(measured)>travel_guard_deg:
        raise RuntimeError(f"Rotary travel guard exceeded: measured {measured:+.4f} degrees; limit +/-{travel_guard_deg:g}")
    tracking_exceeded = abs(error)>tracking_guard_deg
    if tracking_exceeded and tracking_error_action == "stop":
        raise RuntimeError(f"Rotary tracking guard exceeded: error {error:+.4f} degrees; limit +/-{tracking_guard_deg:g}")
    correction=max(-CORRECTION_DEG,min(CORRECTION_DEG,gain*error))/slope
    target=feedforward+correction
    limit=max_command_rate_turns_s*min(dt,.2)
    command=max(last_target-limit,min(last_target+limit,target))
    return dict(desired_angle_deg=desired,measured_angle_deg=measured,tracking_error_deg=error,tracking_threshold_exceeded=tracking_exceeded,
        tracking_threshold_deg=tracking_guard_deg,tracking_error_action=tracking_error_action,
        feedforward_target_turns=feedforward,requested_correction_turns=correction,
        command_target_turns=command,applied_correction_turns=command-feedforward,command_rate_limited=command!=target)
def ramp(start,target,speed,elapsed,*,acceleration=.5):
    if not all(math.isfinite(v) for v in (start, target, speed, elapsed, acceleration)) or speed <= 0 or acceleration <= 0:
        raise ValueError("Invalid reference trajectory parameters")
    distance=abs(target-start);sign=1 if target>=start else -1
    ta=min(speed/acceleration,math.sqrt(distance/acceleration));peak=acceleration*ta
    tc=max(0.,(distance-acceleration*ta*ta)/peak) if peak else 0.;duration=2*ta+tc
    t=min(max(elapsed,0.),duration)
    if t<ta:travel=.5*acceleration*t*t
    elif t<ta+tc:travel=.5*acceleration*ta*ta+peak*(t-ta)
    else:travel=distance-.5*acceleration*(duration-t)**2
    return start+sign*travel,duration
