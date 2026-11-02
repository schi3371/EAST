"""Approximate display estimates only; never used for control or timeouts."""
import time

def clock_text(seconds):
    seconds=max(0,int(round(seconds)));hours,seconds=divmod(seconds,3600);minutes,seconds=divmod(seconds,60)
    return f'{hours:d}:{minutes:02d}:{seconds:02d}' if hours else f'{minutes:d}:{seconds:02d}'

def estimate(parameters,config,stage=''):
    if stage=='stationary_direction_holds':return None
    if stage=='rotary_feedback_pilot':return 75. # Four smooth legs + settling/saving, one cycle.
    p=parameters;speed=p.commanded_afo_speed_deg_s;acc=p.commanded_afo_acceleration_deg_s2
    back=config['motion']['neutral_return_speed_deg_s']
    travel=p.max_angle_deg/speed+2*(p.min_angle_deg+p.max_angle_deg)*p.cycles/speed+p.max_angle_deg/back
    # Nominal profiles plus an explicit approximate preparation/settling allowance.
    return 1.1*(travel+(2*p.cycles+1)*speed/acc)+8.

def timing_text(app,now=None):
    now=time.monotonic() if now is None else now
    active=getattr(app,'strain_test_active',False);m=getattr(app,'run_metadata',None) or {}
    if not active:
        if getattr(getattr(app,'rotary_homing',None),'active',False) is True:
            return 'Zero preparation/recovery in progress — completion depends on settling'
        if m.get('duration_s') is not None:return 'Last run: '+clock_text(m['duration_s'])+' | '+str(m.get('run_status','finished'))
        return 'Elapsed / estimated remaining: shown when a test starts'
    started=getattr(app,'test_started_monotonic',None)
    if started is None:return 'Preparing test — time estimate pending'
    elapsed=max(0.,now-started);context=getattr(app,'run_motion_mapping_context',None) or {}
    p=getattr(app,'run_parameters',None)
    total=estimate(p,app.system_config,context.get('stage','')) if p else None
    text='Run elapsed '+clock_text(elapsed)
    if total is None:text+=' | Remaining: variable (feedback positioning)'
    elif elapsed>total:text+=' | Estimate exceeded — completion time uncertain'
    else:text+=' | Run remaining ≈ '+clock_text(total-elapsed)
    batch=getattr(app,'motion_mapping',None)
    if batch is not None and getattr(batch,'active',False) and getattr(batch,'requests',None):
        request=getattr(batch,'request',None)
        if request in batch.requests:
            future=batch.requests[batch.requests.index(request)+1:]
            durations=[estimate(r.parameters,app.system_config,getattr(r,'stage','')) for r in future]
            if total is not None and elapsed<=total and all(v is not None for v in durations):
                text+=' | Batch remaining ≈ '+clock_text(total-elapsed+sum(durations))
    if getattr(app,'test_stop_event',None) is not None and app.test_stop_event.is_set():
        text='Run elapsed '+clock_text(elapsed)+' | Stopping/cleanup — remaining time unknown'
    return text
