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

    def context(self):
        return dict(purpose='three_speed_empty_baseline' if self.parameters.test_type=='empty_machine_baseline' else 'three_speed_afo_testing',
            batch_id=self.batch_id,mounting_setup_id=self.mounting_setup_id,run_id=self.run_id,
            run_index=self.index,total_runs=self.total,planned_speed_deg_s=self.parameters.commanded_afo_speed_deg_s,
            planned_speed_sequence_deg_s=list(self.speed_sequence),protocol_key=self.preset_metadata['preset_key'],
            protocol_version=self.preset_metadata['preset_version'],batch_manifest=self.manifest_path,
            additional_inter_run_rest_s=0.0,baseline_batch_id=self.baseline_batch_id,
            baseline_link_status='operator_supplied_unverified' if self.baseline_batch_id else 'not_linked',
            same_mounting_across_speed_conditions=True,independent_remount_trials=False,
            automatic_baseline_correction=False,settings_status='provisional_not_scientifically_validated')


def make_protocol_plan(config,preset_key,identity,settings,mounting_setup_id,baseline_batch_id,manifest_path,loaded_at,batch_id=None):
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
    prefix=sanitise_identifier(identity['file_prefix'])[:24]
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
