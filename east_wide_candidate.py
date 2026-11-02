"""Frozen wider direction cubic, selected before prospective comparison. No hardware."""
from east_motion_model import RunModel, finite, provenance
B = (-0.12566267182143176, 2.361689928112189, 0.0053034037069941324, -0.005727128647760105)
C = (-0.15639611231389627, -0.0006496451626720298, 0.000372761734383137, 1.3446315765937928e-05)
MODEL_ID='wide-direction-cubic-20261102-v1'
def curve(x,d):
    x=finite(x)
    if d not in (-1,1) or abs(x)>8:raise ValueError('Invalid candidate coordinate/direction')
    return sum((b+d*c)*x**i for i,(b,c) in enumerate(zip(B,C)))
def inverse(y,d):
    y=finite(y);lo,hi=-8.,8.
    if not curve(lo,d)<=y<=curve(hi,d):raise ValueError('Candidate target outside inverse bracket')
    for _ in range(60):
        mid=(lo+hi)/2
        if curve(mid,d)<y:lo=mid
        else:hi=mid
    return (lo+hi)/2
class WideRunModel(RunModel):
    @classmethod
    def anchored(cls,motor_turns,rotary_angle_deg):
        motor,angle=finite(motor_turns),finite(rotary_angle_deg)
        if abs(angle)>.1:raise ValueError('Candidate requires rotary zero')
        return cls(motor-inverse(angle,1),motor,angle)
    @classmethod
    def observed(cls,motor_turns,rotary_angle_deg,direction):
        motor,angle=finite(motor_turns),finite(rotary_angle_deg)
        return cls(motor-inverse(angle,direction),motor,angle)
    def target(self,angle,direction):
        if abs(finite(angle))>15:raise ValueError('Candidate test target exceeds 15 degrees')
        return self.origin_turns+inverse(angle,direction)
    def metadata(self):
        m=super().metadata()
        m.update(model_id=MODEL_ID,b=list(B),c=list(C),fitted_motor_range_turns=[-7.534791504867035,7.1896987511329655],
            source_batch='408453cc',status='frozen_candidate_pending_prospective_validation',
            zero_alignment='same initial increasing-branch origin alignment as current model; frozen for numbered cycles')
        return m
