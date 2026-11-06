"""Prospective empty-fixture comparison; explicit high-speed scope, never normal AFO settings."""
from copy import deepcopy
import math
import uuid
from east_stream_screen import BLOCKS, MODELS, StreamScreenModel

class ComparisonMotionMiss(RuntimeError):
    """A recorded model-domain/deadline failure; not a hardware or sensing fault."""


RECOVERABLE_PREFIXES = ('Rotary pilot leg timed out', 'Measured position outside streamed model bracket',
                        'Streamed target outside frozen inverse bracket')

PRESET = 'full_model_speed_comparison'
CONDITION_PREFIX = 'model_speed_'
SPEEDS = (.5, 5., 10., 20., 30., 40., 50.)
PROFILE = dict(maximum_speed_deg_s=50., maximum_acceleration_deg_s2=100.,
               maximum_motion_timeout_s=90., reference_period_s=.04)


def rows():
    return [(speed, block, method) for speed in SPEEDS
            for block, order in enumerate(BLOCKS, 1) for method in order]


def condition_key(speed):
    if speed not in SPEEDS:
        raise ValueError('Unsupported full-comparison speed')
    return CONDITION_PREFIX + str(float(speed)).replace('.', 'p')


def condition_speed(key):
    for speed in SPEEDS:
        if key == condition_key(speed):
            return speed
    raise ValueError('Unknown full-comparison speed condition')


def profile(config):
    configured = config.get('engineering_model_comparison', {})
    if configured != PROFILE:
        raise ValueError('Reload the explicit empty-fixture engineering comparison profile')
    return dict(configured)


def validate_models():
    # Frozen branches must cover both endpoints with a single increasing inverse.
    for variant in MODELS:
        model = StreamScreenModel.anchored(variant, 0., 0., full_range=True)
        for direction in (-1, 1):
            for angle in (-15., 0., 15.):
                model.target(angle, direction)
            if min(model.slope(model.origin_turns - 8. + i*.008, direction)
                   for i in range(2001)) <= 0:
                raise ValueError('Non-monotonic full-range model: '+variant)


def make_plan(config, identity, settings, setup, baseline, path, loaded, batch_id=None):
    from east_core import validate_test_parameters, make_preset_metadata
    from east_protocol_batch import ProtocolRequest
    if (int(settings['cycles']), float(settings['minimum_angle_deg']),
            float(settings['maximum_angle_deg']), float(settings['acceleration_deg_s2'])) != (1,15.,15.,100.):
        raise ValueError('Reload Full Model × Speed Comparison: fixed one cycle, ±15°, acceleration 100°/s²')
    if not str(setup).strip() or str(baseline).strip():
        raise ValueError('Empty fixture setup ID required; no baseline link')
    limits = profile(config)
    validate_models()
    scoped = deepcopy(config)
    scoped['motion'].update({k:v for k,v in limits.items() if k!='reference_period_s'})
    batch_id = batch_id or uuid.uuid4().hex
    plan = rows(); sequence = tuple(r[0] for r in plan); result=[]
    for index,(speed,block,method) in enumerate(plan,1):
        feedback = method=='feedback'; variant = 'candidate' if feedback else method
        params=validate_test_parameters(dict(file_prefix=f'FullCompare_B{batch_id[:8]}_R{index}_{method}',
            operator=identity['operator'],fixture_id=identity['fixture_id'],calibration_id=identity['calibration_id'],
            afo_id='EMPTY_MACHINE_BASELINE',test_type='empty_machine_baseline',cycles=1,
            speed_deg_s=speed,acceleration_deg_s2=100.,min_angle_deg=15.,max_angle_deg=15.),scoped)
        meta=make_preset_metadata(PRESET,config['test_presets'][PRESET],params,loaded,False)
        meta.update(screen_method=method, screen_block=block, engineering_profile=limits,
                    comparison_scope='common streamed trajectory; empty fixture; achieved speed measured separately')
        result.append(ProtocolRequest(batch_id,f'R{index} {speed:g} dps B{block} {method}',index,len(plan),params,
            meta,str(path),str(setup).strip(),sequence,None,command_model_enabled=True,
            stage='rotary_feedback_pilot',repeat=block,model_variant=variant,feedback_pilot=feedback,
            feedback_condition_key=condition_key(speed)))
    return result
