"""Frozen, explicit three-speed plans. Reuses normal test validation/provenance."""
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime
import uuid
from typing import Optional
from east_core import validate_test_parameters, make_preset_metadata, preset_values_modified, sanitise_identifier

from east_core import (AFO_MATERIALS, AFO_RETEST, AFO_RETEST_SPEEDS, FITTED_SPEED_SCREEN, FITTED_SPEED_SCREEN_RANGES, FITTED_SPEED_SCREEN_SPEEDS,
                       SLOW_REPEATABILITY_PRESETS, SLOW_REPEATABILITY_SPEEDS)
from east_motion_model import selected_variant
from east_stream_screen import BLOCKS, SCREEN_PRESET
from east_feedback_commissioning import CONDITIONS, PREFIX, SEQUENCE_PRESET, condition_for

SPEEDS = (0.5, 5.0, 20.0)
MATERIALS = {
    "N12F": ("Nylon 12 / FDM", "EPIC LAB"),
    "CN12F": ("Carbon-fibre Nylon 12 / FDM", "objective3D"),
    "N11H": ("Nylon 11 / Stratasys H350", "objective3D"),
    "N11H-S": ("Grey Nylon 11, marked N11H-S (unconfirmed)", None),
    "N11H-P": ("Grey Nylon 11, marked N11H-P (unconfirmed)", None),
    "N11H-UNCLEAR": ("Grey Nylon 11, marking unclear (unconfirmed)", None),
    "WSLS": ("White Nylon 12 SLS (code unconfirmed)", None),
    "N12HP": ("Nylon 12 / HP Jet Fusion", "GoProto"),
    "N11S": ("Nylon 11 / 3D Systems SLS", "GoProto"),
    "N12S": ("Nylon 12 / 3D Systems SLS", "objective3D"),
    "N12E": ("Nylon 12 / EOS SLS", "3D Printing Studio"),
    "PAX": ("PAx / SLS 380", "GoProto"),
    "DLP": ("Digital light processing", "objective3D"),
}

DUPLICATE_MATERIAL_SETS = {"CN12F", "N12S", "DLP", "WSLS"}

@dataclass(frozen=True)
class ProtocolRequest:
    batch_id: str
    run_id: str
    index: int
    total: int
    parameters: object
    preset_metadata: dict
    manifest_path: str
    mounting_setup_id: str
    speed_sequence: tuple
    baseline_batch_id: Optional[str]
    metadata_key: str = 'protocol_batch'
    command_model_enabled: Optional[bool] = None
    rotary_targeted: bool = False
    stage: str = ''
    repeat: int = 0
    stationary_holds: bool = False
    model_variant: str = 'current'
    feedback_pilot: bool = False
    feedback_condition_key: Optional[str] = None

    def context(self):
        return dict(purpose='three_speed_empty_baseline' if self.parameters.test_type=='empty_machine_baseline' else 'three_speed_afo_testing',
            batch_id=self.batch_id,mounting_setup_id=self.mounting_setup_id,run_id=self.run_id,
            run_index=self.index,total_runs=self.total,planned_speed_deg_s=self.parameters.commanded_afo_speed_deg_s,
            planned_speed_sequence_deg_s=list(self.speed_sequence),protocol_key=self.preset_metadata['preset_key'],
            protocol_version=self.preset_metadata['preset_version'],batch_manifest=self.manifest_path,
            additional_inter_run_rest_s=0.0,baseline_batch_id=self.baseline_batch_id,
            baseline_link_status=('preset_reference' if self.preset_metadata.get('empty_reference_name')
                                  else 'operator_supplied_unverified' if self.baseline_batch_id else 'not_linked'),
            same_mounting_across_speed_conditions=True,independent_remount_trials=False,
            automatic_baseline_correction=False,settings_status='provisional_not_scientifically_validated',
            command_model_enabled=self.command_model_enabled,rotary_targeted=self.rotary_targeted,
            stage=self.stage,repeat=self.repeat,stationary_holds=self.stationary_holds,model_variant=self.model_variant,feedback_pilot=self.feedback_pilot,
            endpoint_feedback_is_not_feedforward_validation=self.rotary_targeted,
            **{key: self.preset_metadata[key] for key in
               ('material_code', 'specimen_number', 'material_print_method', 'print_bureau',
                'specimen_scale_percent', 'sample_source', 'specimen_set', 'empty_reference_name') if key in self.preset_metadata},
            **({"feedback_condition_key": self.feedback_condition_key}
               if self.feedback_condition_key is not None else {}))


def make_protocol_plan(config,preset_key,identity,settings,mounting_setup_id,baseline_batch_id,manifest_path,loaded_at,batch_id=None,
                       *, empty_calibration=False):
    if empty_calibration and preset_key != FITTED_SPEED_SCREEN:
        raise ValueError('Empty fixture calibration is part of the five-speed study')
    from east_full_model_suite import PRESET, make_plan
    if preset_key == PRESET:
        return make_plan(config,identity,settings,mounting_setup_id,baseline_batch_id,manifest_path,loaded_at,batch_id)
    if config['test_presets'][preset_key].get('protocol_kind') in ('model_comparison','rotary_feedback_pilot'):
        return make_comparison_plan(config,preset_key,identity,settings,mounting_setup_id,baseline_batch_id,manifest_path,loaded_at,batch_id)
    if config['test_presets'][preset_key].get('protocol_kind')=='remaining_motion_data':
        return make_remaining_plan(config,preset_key,identity,settings,mounting_setup_id,baseline_batch_id,manifest_path,loaded_at,batch_id)
    preset=deepcopy(config['test_presets'][preset_key])
    if preset_key == FITTED_SPEED_SCREEN:
        speeds = FITTED_SPEED_SCREEN_SPEEDS
        if preset['test_type'] != 'afo_test':
            raise ValueError('The five-speed study preset must declare fitted AFO mode')
        if empty_calibration:
            preset['test_type'] = 'empty_machine_baseline'
    elif preset_key in (AFO_RETEST, AFO_MATERIALS):
        speeds = AFO_RETEST_SPEEDS
        if any(float(settings[key]) != preset[key] for key in
               ('cycles', 'minimum_angle_deg', 'maximum_angle_deg', 'acceleration_deg_s2')):
            raise ValueError(f"Reload {preset['display_name']} for {preset['cycles']} cycles at ±10° and 100°/s²")
        baseline_batch_id = preset['empty_reference_batch_id']
    else:
        speeds = SLOW_REPEATABILITY_SPEEDS if preset_key in SLOW_REPEATABILITY_PRESETS else SPEEDS
    if preset.get('execution_kind') != 'speed_sequence' or tuple(preset['speeds_deg_s']) != speeds:
        raise ValueError('Select a supported explicit speed protocol')
    if preset_key == AFO_MATERIALS:
        code, separator, number = identity.get('afo_id', '').rpartition('-')
        specimen_set = None
        if code.endswith(('-A', '-B')):
            code, _, specimen_set = code.rpartition('-')
        if code in DUPLICATE_MATERIAL_SETS and specimen_set is None:
            raise ValueError('Choose Set A or Set B for this print code')
        if code not in MATERIALS or not separator or number not in ('01', '02', '03', '04', '05'):
            raise ValueError('Select the print code and physical specimen number')
        selected_speed = float(identity['materials_speed'])
        if selected_speed not in speeds:
            raise ValueError('Select the agreed materials-test speed')
        speeds = (selected_speed,)
        if not identity.get('use_short_plate_reference'):
            baseline_batch_id = ''
    if preset_key in SLOW_REPEATABILITY_PRESETS:
        angles = float(settings['minimum_angle_deg']), float(settings['maximum_angle_deg'])
        if angles not in ((5., 5.), (10., 10.)):
            raise ValueError('This pilot requires symmetric ±5° or ±10°')
    mounting_setup_id=str(mounting_setup_id).strip()
    if not mounting_setup_id:
        raise ValueError('Enter a mounting/setup ID; reuse it if the setup has not changed')
    if preset['test_type']=='afo_test' and str(identity['afo_id']).strip() in ('','EMPTY_MACHINE_BASELINE'):
        raise ValueError('Enter the actual AFO ID for a loaded sequence')
    baseline_batch_id=str(baseline_batch_id).strip() or None
    if preset['test_type']=='empty_machine_baseline' and baseline_batch_id:
        raise ValueError('An empty-machine batch cannot link to another baseline batch')
    batch_id=batch_id or uuid.uuid4().hex
    prefix=sanitise_identifier(identity['file_prefix'] or 'MotionSuite')[:24]
    modified=preset_values_modified(settings,preset)
    requests=[]
    if preset_key == FITTED_SPEED_SCREEN:
        if str(settings['cycles']).strip() != str(preset['cycles']):
            raise ValueError(f"Reload Five-Speed AFO Study for {preset['cycles']} cycles per speed and range")
        conditions = [(speed, angle, angle) for angle in FITTED_SPEED_SCREEN_RANGES for speed in speeds]
    else:
        conditions = [(speed, settings['minimum_angle_deg'], settings['maximum_angle_deg']) for speed in speeds]
    planned_speeds = tuple(speed for speed, _, _ in conditions)
    for index, (speed, minimum, maximum) in enumerate(conditions, 1):
        label = f'R{index}_{str(speed).replace(".", "p")}dps'
        if preset_key == FITTED_SPEED_SCREEN:
            label += f'_ROM{maximum:g}'
        parameters=validate_test_parameters(dict(file_prefix=f'{prefix}_B{batch_id[:8]}_{label}',
            operator=identity['operator'],fixture_id=identity['fixture_id'],calibration_id=identity['calibration_id'],
            afo_id=identity['afo_id'],test_type=preset['test_type'],cycles=settings['cycles'],speed_deg_s=speed,
            acceleration_deg_s2=settings['acceleration_deg_s2'],min_angle_deg=minimum,
            max_angle_deg=maximum),config)
        metadata=make_preset_metadata(preset_key,preset,parameters,loaded_at,modified)
        metadata.update(execution_kind='speed_sequence',planned_speed_sequence_deg_s=list(planned_speeds),
                        settings_status='provisional_not_scientifically_validated')
        if preset_key == AFO_MATERIALS:
            material, bureau = MATERIALS[code]
            metadata.update(material_code=code, specimen_number=int(number), material_print_method=material,
                            print_bureau=bureau, specimen_scale_percent=75, specimen_set=specimen_set,
                            sample_source="EPIC Lab")
        if preset_key in (AFO_RETEST, AFO_MATERIALS) and baseline_batch_id:
            metadata['empty_reference_name'] = preset['empty_reference_name']
        if preset_key == FITTED_SPEED_SCREEN:
            metadata['study_mode'] = 'empty_fixture_calibration' if empty_calibration else 'fitted_speed_screen'
            metadata['planned_half_ranges_deg'] = list(FITTED_SPEED_SCREEN_RANGES)
        requests.append(ProtocolRequest(batch_id,label,index,len(conditions),parameters,metadata,str(manifest_path),
                                       mounting_setup_id,planned_speeds,baseline_batch_id,
                                       model_variant=selected_variant(config)))
    return requests


def make_remaining_plan(config,preset_key,identity,settings,mounting_setup_id,baseline_batch_id,manifest_path,loaded_at,batch_id=None):
    preset=deepcopy(config['test_presets'][preset_key])
    if (int(settings['cycles'])!=3 or float(settings['minimum_angle_deg'])!=10
            or float(settings['maximum_angle_deg'])!=10 or float(settings['acceleration_deg_s2'])!=100):
        raise ValueError('Reload Remaining Motion Data preset: its staged plan uses fixed settings')
    if not str(mounting_setup_id).strip() or str(baseline_batch_id).strip():
        raise ValueError('Empty-machine suite requires a setup ID and no baseline link')
    batch_id=batch_id or uuid.uuid4().hex
    plan=[('updated_10_repeat',repeat,sp,10,True,False,False) for repeat in (2,3) for sp in SPEEDS]
    plan += [('wide_15_identification',1,sp,15,False,True,False) for sp in SPEEDS]
    plan += [('stationary_direction_holds',1,5.,12,False,True,True)]
    sequence=tuple(row[2] for row in plan);requests=[]
    for i,(stage,repeat,speed,rom,use_model,rotary,holds) in enumerate(plan,1):
        p=validate_test_parameters(dict(file_prefix=f"{sanitise_identifier(identity['file_prefix'] or 'MotionSuite')[:24]}_B{batch_id[:8]}_R{i}",
            operator=identity['operator'],fixture_id=identity['fixture_id'],calibration_id=identity['calibration_id'],
            afo_id='EMPTY_MACHINE_BASELINE',test_type='empty_machine_baseline',cycles=3,speed_deg_s=speed,
            acceleration_deg_s2=100,min_angle_deg=rom,max_angle_deg=rom),config,segmented_rotary_motion=rotary)
        meta=make_preset_metadata(preset_key,preset,p,loaded_at,False)
        meta.update(stage=stage,repeat=repeat,execution_kind='staged_validation_suite',settings_status='provisional_not_scientifically_validated')
        requests.append(ProtocolRequest(batch_id,f'R{i} {stage} {speed:g} deg/s',i,len(plan),p,meta,str(manifest_path),
            str(mounting_setup_id).strip(),sequence,None,command_model_enabled=use_model,rotary_targeted=rotary,
            stage=stage,repeat=repeat,stationary_holds=holds))
    return requests

def make_comparison_plan(config,key,identity,settings,setup,baseline,path,loaded,batch_id=None):
    pilot=config['test_presets'][key].get('protocol_kind')=='rotary_feedback_pilot'
    if pilot:
        return make_feedback_plan(config,key,identity,settings,setup,baseline,path,loaded,batch_id)
    from dataclasses import replace
    if (int(settings['cycles'])!=3 or float(settings['minimum_angle_deg'])!=15 or
            float(settings['maximum_angle_deg'])!=15 or float(settings['acceleration_deg_s2'])!=100):
        raise ValueError('Reload Model Comparison preset: fixed 3 cycles, +/-15, acceleration 100')
    if not str(setup).strip() or str(baseline).strip():raise ValueError('Empty fixture setup ID required; no baseline link')
    batch_id=batch_id or uuid.uuid4().hex
    plan=[('commissioning',0,5.,rom,model,1) for model in ('original','candidate') for rom in (5.,10.)]
    for repeat,speeds,models in [(1,SPEEDS,('original','current','candidate')),(2,tuple(reversed(SPEEDS)),('candidate','current','original'))]:
        plan += [('model_comparison',repeat,sp,15.,model,3) for sp in speeds for model in models]
    requests=[];sequence=tuple(z[2] for z in plan)
    for i,(stage,repeat,sp,rom,model,cycles) in enumerate(plan,1):
        params=validate_test_parameters(dict(file_prefix=f"Compare_B{batch_id[:8]}_R{i}_{model}",operator=identity['operator'],
            fixture_id=identity['fixture_id'],calibration_id=identity['calibration_id'],afo_id='EMPTY_MACHINE_BASELINE',
            test_type='empty_machine_baseline',cycles=cycles,speed_deg_s=sp,acceleration_deg_s2=100,min_angle_deg=rom,max_angle_deg=rom),config,segmented_rotary_motion=True)
        meta=make_preset_metadata(key,config['test_presets'][key],params,loaded,False)
        meta.update(stage=stage,repeat=repeat,model_variant=model,execution_kind='staged_model_comparison',
            candidate_model_id='wide-direction-cubic-20261102-v1',slow_sweeps_segmented=True)
        requests.append(ProtocolRequest(batch_id,f'R{i} {stage} {model} {sp:g} deg/s',i,len(plan),params,meta,str(path),
            str(setup).strip(),sequence,None,command_model_enabled=model!='original',stage=stage,repeat=repeat,model_variant=model))
    return requests

def make_feedback_plan(config,key,identity,settings,setup,baseline,path,loaded,batch_id=None):
    if key == SEQUENCE_PRESET:
        return make_feedback_sequence(config, identity, settings, setup, baseline, path, loaded, batch_id)
    from east_smooth_motion import PRESET as SMOOTH_PAIR, runnable_study_conditions
    smooth = key == SMOOTH_PAIR
    condition = condition_for(key)
    cycles = 3 if smooth else 1
    half_range = 10. if smooth else (condition.half_range_deg if condition else 5.)
    speed = 5. if smooth else (condition.speed_deg_s if condition else .5)
    preset = config["test_presets"][key]
    acceleration = preset["acceleration_deg_s2"] if smooth else (condition.acceleration_deg_s2 if condition else 5.)
    if (int(settings['cycles']) != cycles or float(settings['minimum_angle_deg']) != half_range
            or float(settings['maximum_angle_deg']) != half_range
            or float(settings['acceleration_deg_s2']) != acceleration):
        raise ValueError('Reload the selected fixed feedback preset')
    if not str(setup).strip() or str(baseline).strip():raise ValueError('Empty fixture setup ID required')
    batch_id = batch_id or uuid.uuid4().hex
    requests = []
    screen = key == SCREEN_PRESET
    plan = ([(block, method) for block, order in enumerate(BLOCKS, 1) for method in order]
            if screen else [(1, "candidate"), (1, "feedback")])
    rows = [(block, method, speed, acceleration) for block, method in plan]
    if smooth:
        rows = [(stage, method, stage_speed, stage_acceleration)
                for stage, (stage_speed, stage_acceleration) in enumerate(runnable_study_conditions(config["motion"]), 1)
                for method in ("candidate", "feedback")]
    sequence = tuple(row[2] for row in rows)
    for i, (block, method, speed, acceleration) in enumerate(rows, 1):
        feedback = method == "feedback"
        variant = "candidate" if feedback else method
        prefix = "SmoothPair" if smooth else ("StreamScreen" if screen else "Pilot")
        p=validate_test_parameters(dict(file_prefix=f'{prefix}_B{batch_id[:8]}_R{i}',operator=identity['operator'],fixture_id=identity['fixture_id'],
            calibration_id=identity['calibration_id'],afo_id='EMPTY_MACHINE_BASELINE',test_type='empty_machine_baseline',cycles=cycles,
            speed_deg_s=speed,acceleration_deg_s2=acceleration,min_angle_deg=half_range,max_angle_deg=half_range),config)
        meta=make_preset_metadata(key,config['test_presets'][key],p,loaded,False)
        if screen:
            meta.update(screen_method=method, screen_block=block)
        label = f'R{i} {variant} ' + ('rotary feedback' if feedback else 'feedforward')
        requests.append(ProtocolRequest(batch_id, label, i, len(rows), p, meta,
            str(path), str(setup).strip(), sequence, None, command_model_enabled=True,
            stage='rotary_feedback_pilot', repeat=block, model_variant=variant, feedback_pilot=feedback))
    return requests



def make_feedback_sequence(config, identity, settings, setup, baseline, path, loaded, batch_id=None):
    """Compose reviewed pairs into one batch while preserving each condition's identity."""
    from dataclasses import replace

    first = next(iter(CONDITIONS.values()))
    expected = (1, first.half_range_deg, first.half_range_deg, first.acceleration_deg_s2)
    actual = (int(settings['cycles']), float(settings['minimum_angle_deg']),
              float(settings['maximum_angle_deg']), float(settings['acceleration_deg_s2']))
    if actual != expected:
        raise ValueError("Reload the Feedback Commissioning Sequence preset")
    batch_id = batch_id or uuid.uuid4().hex
    requests = []
    sequence = tuple(condition.speed_deg_s for condition in CONDITIONS.values() for _ in range(2))
    total = len(sequence)
    preset = config['test_presets'][SEQUENCE_PRESET]
    for stage, condition in CONDITIONS.items():
        condition_key = PREFIX + stage
        values = dict(cycles=1, minimum_angle_deg=condition.half_range_deg,
                      maximum_angle_deg=condition.half_range_deg,
                      acceleration_deg_s2=condition.acceleration_deg_s2)
        pair = make_feedback_plan(config, condition_key, identity, values, setup,
                                  baseline, path, loaded, batch_id)
        for request in pair:
            index = len(requests) + 1
            parameters = replace(
                request.parameters,
                file_prefix=f"FeedbackSuite_B{batch_id[:8]}_R{index}_S{stage}")
            metadata = make_preset_metadata(SEQUENCE_PRESET, preset, parameters, loaded, False)
            metadata.update(feedback_stage=int(stage), feedback_condition_key=condition_key,
                            condition_preset=deepcopy(request.preset_metadata))
            requests.append(replace(
                request, index=index, total=total, parameters=parameters,
                run_id=f"Stage {int(stage)} R{request.index} " + (
                    "rotary feedback" if request.feedback_pilot else "feedforward"),
                preset_metadata=metadata, speed_sequence=sequence,
                feedback_condition_key=condition_key))
    return requests
