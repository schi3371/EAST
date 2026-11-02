"""Fresh stationary observations. Timing limits supplied by existing workflow policy."""
import math,time,statistics

def collect_stationary_window(read, duration, timeout, *, position_tolerance_turns=None, clock=time.monotonic, sleep=time.sleep):
    if not math.isfinite(duration) or not math.isfinite(timeout) or duration<=0 or timeout<duration:
        raise ValueError('Invalid stationary observation duration/deadline')
    if position_tolerance_turns is not None and (not math.isfinite(position_tolerance_turns) or position_tolerance_turns<=0):
        raise ValueError('Invalid stationary position tolerance')
    deadline=clock()+timeout;quiet=None;angles={};positions=[];resets=0
    while clock()<=deadline:
        value=read()  # Caller enforces cancellation, identity, freshness and controller state.
        now=clock()
        if not value['stationary']:
            quiet=None;angles.clear();positions.clear();resets+=1
        else:
            if quiet is None:quiet=now
            motor=float(value['motor_turns'])
            if not math.isfinite(motor):raise RuntimeError('Non-finite stationary motor position')
            positions.append(motor)
            if position_tolerance_turns is not None and max(positions)-min(positions)>position_tolerance_turns:
                quiet=None;angles.clear();positions.clear();resets+=1
                sleep(.01)
                continue
            if value.get('angle_deg') is not None:
                angle=float(value['angle_deg'])
                if not math.isfinite(angle):raise RuntimeError('Non-finite stationary rotary angle')
                angles[value['sequence']]=angle
            if now-quiet>=duration:
                if value.get('angle_deg') is not None and len(angles)<5:
                    raise RuntimeError('Insufficient fresh rotary samples during stationary observation')
                result=dict(duration_s=now-quiet,stationary_resets=resets,
                    motor_min_turns=min(positions),motor_max_turns=max(positions),motor_final_turns=motor,
                    completed_at_monotonic_s=now)
                if angles:
                    data=list(angles.values());result.update(mean_deg=statistics.mean(data),min_deg=min(data),
                        max_deg=max(data),samples=len(data))
                return result
        sleep(.01)
    raise RuntimeError('Motor did not remain stationary for the required observation before the deadline')
