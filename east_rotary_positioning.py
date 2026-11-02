"""Bounded rotary endpoint positioning, reusing official homing observation rules.

The bulk leg uses legacy motor conversion, then slow bounded corrections target
measured angle. This is identification/acquisition, not validation of feed-forward
endpoint accuracy. Callbacks retain app ownership, cancellation and recording.
"""
import math
from east_rotary_homing import HomingPolicy, home


def reach(target_deg, current, bulk_move, fine_move, observe, check):
    target=float(target_deg)
    if not math.isfinite(target) or abs(target)>15:
        raise ValueError('Rotary acquisition target must be within +/-15 degrees')
    check()
    angle=float(current())
    if not math.isfinite(angle) or abs(angle)>15.1:
        raise RuntimeError('Rotary acquisition angle exceeds +/-15 degrees plus 0.1-degree tolerance')
    error=target-angle
    if abs(error)>1.0:
        # Conservative first leg stops short, leaving room for measured approach.
        bulk_move(error*.8)
    def relative_observe(duration):
        v=observe(duration)
        return {**v,'mean_deg':v['mean_deg']-target,'min_deg':v['min_deg']-target,'max_deg':v['max_deg']-target}
    result=home(HomingPolicy(),relative_observe,fine_move,check)
    result['observation_coordinates']='error relative to target'
    relative=result['final_observation']
    result['final_measured_observation']={**relative,'mean_deg':relative['mean_deg']+target,'min_deg':relative['min_deg']+target,'max_deg':relative['max_deg']+target}
    result['target_angle_deg']=target
    return result
