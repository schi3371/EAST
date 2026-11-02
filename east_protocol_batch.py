"""Frozen, explicit three-speed plans. Reuses normal test validation/provenance."""
from copy import deepcopy
from dataclasses import dataclass
from datetime import datetime
import uuid
from typing import Optional
from east_core import validate_test_parameters, make_preset_metadata, preset_values_modified, sanitise_identifier

SPEEDS = (0.5, 5.0, 20.0)

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

    def context(self):
        return dict(purpose='three_speed_empty_baseline' if self.parameters.test_type=='empty_machine_baseline' else 'three_speed_afo_testing',
            batch_id=self.batch_id,mounting_setup_id=self.mounting_setup_id,run_id=self.run_id,
            run_index=self.index,total_runs=self.total,planned_speed_deg_s=self.parameters.commanded_afo_speed_deg_s,
            planned_speed_sequence_deg_s=list(self.speed_sequence),protocol_key=self.preset_metadata['preset_key'],
            protocol_version=self.preset_metadata['preset_version'],batch_manifest=self.manifest_path,
            additional_inter_run_rest_s=0.0,baseline_batch_id=self.baseline_batch_id,
            baseline_link_status='operator_supplied_unverified' if self.baseline_batch_id else 'not_linked',
            same_mounting_across_speed_conditions=True,independent_remount_trials=False,
            automatic_baseline_correction=False,settings_status='provisional_not_scientifically_validated',
            command_model_enabled=self.command_model_enabled,rotary_targeted=self.rotary_targeted,
            stage=self.stage,repeat=self.repeat,stationary_holds=self.stationary_holds,model_variant=self.model_variant,
            endpoint_feedback_is_not_feedforward_validation=self.rotary_targeted)


def make_protocol_plan(config,preset_key,identity,settings,mounting_setup_id,baseline_batch_id,manifest_path,loaded_at,batch_id=None):
    if config['test_presets'][preset_key].get('protocol_kind')=='model_comparison':
        return make_comparison_plan(config,preset_key,identity,settings,mounting_setup_id,baseline_batch_id,manifest_path,loaded_at,batch_id)
    if config['test_presets'][preset_key].get('protocol_kind')=='remaining_motion_data':
        return make_remaining_plan(config,preset_key,identity,settings,mounting_setup_id,baseline_batch_id,manifest_path,loaded_at,batch_id)
    preset=deepcopy(config['test_presets'][preset_key])
    if preset.get('execution_kind')!='speed_sequence' or tuple(preset['speeds_deg_s'])!=SPEEDS:
        raise ValueError('Select a supported explicit three-speed protocol')
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
    for index,speed in enumerate(SPEEDS,1):
        label=f'R{index}_{str(speed).replace(".","p")}dps'
        parameters=validate_test_parameters(dict(file_prefix=f'{prefix}_B{batch_id[:8]}_{label}',
            operator=identity['operator'],fixture_id=identity['fixture_id'],calibration_id=identity['calibration_id'],
            afo_id=identity['afo_id'],test_type=preset['test_type'],cycles=settings['cycles'],speed_deg_s=speed,
            acceleration_deg_s2=settings['acceleration_deg_s2'],min_angle_deg=settings['minimum_angle_deg'],
            max_angle_deg=settings['maximum_angle_deg']),config)
        metadata=make_preset_metadata(preset_key,preset,parameters,loaded_at,modified)
        metadata.update(execution_kind='speed_sequence',planned_speed_sequence_deg_s=list(SPEEDS),
                        settings_status='provisional_not_scientifically_validated')
        requests.append(ProtocolRequest(batch_id,label,index,3,parameters,metadata,str(manifest_path),
                                       mounting_setup_id,SPEEDS,baseline_batch_id))
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
    from dataclasses import replace
    if (int(settings['cycles'])!=3 or float(settings['minimum_angle_deg'])!=15 or
            float(settings['maximum_angle_deg'])!=15 or float(settings['acceleration_deg_s2'])!=100):
        raise ValueError('Reload Model Comparison preset: fixed 3 cycles, +/-15, acceleration 100')
    if not str(setup).strip() or str(baseline).strip():raise ValueError('Empty fixture setup ID required; no baseline link')
    batch_id=batch_id or uuid.uuid4().hex
    plan=[('commissioning',0,5.,5.,'candidate',1),('commissioning',0,5.,10.,'candidate',1)]
    for repeat,speeds,models in [(1,SPEEDS,('current','candidate')),(2,tuple(reversed(SPEEDS)),('candidate','current'))]:
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
            str(setup).strip(),sequence,None,command_model_enabled=True,stage=stage,repeat=repeat,model_variant=model))
    return requests
