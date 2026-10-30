"""Hardware-free plans and completion gates for the GUI motion experiment."""
from dataclasses import dataclass
from datetime import datetime
import uuid
from east_core import validate_test_parameters, make_preset_metadata, preset_motion_values


@dataclass(frozen=True)
class MappingRequest:
    batch_id: str
    run_id: str
    index: int
    total: int
    parameters: object
    preset_metadata: dict
    manifest_path: str

    def context(self):
        return dict(purpose='nonlinear_motion_identification', physical_condition='empty_machine',
                    batch_id=self.batch_id, run_id=self.run_id, run_index=self.index,
                    total_runs=self.total, batch_manifest=self.manifest_path)


def make_plan(config, identity, settings, mode, manifest_path):
    """Every entry is validated through exactly the normal GUI parameter validator."""
    if mode not in ('single', 'sequence'):
        raise ValueError('Unknown motion modelling plan')
    import math
    slow, fast = float(settings['slow']), float(settings['fast'])
    if not all(math.isfinite(v) for v in (slow, fast)):
        raise ValueError('Speeds must be finite')
    if not slow < fast:
        raise ValueError('Slow speed must be below the operating speed')
    batch_id = uuid.uuid4().hex
    order = [('S1', slow)] if mode == 'single' else [
        item for i in range(1, 4) for item in ((f'S{i}', slow), (f'M{i}', fast))]
    requests = []
    for i, (label, speed) in enumerate(order, 1):
        p = validate_test_parameters(dict(file_prefix=f'MM_{batch_id[:8]}_{label}',
            operator=identity['operator'], fixture_id=identity['fixture_id'],
            calibration_id=identity['calibration_id'], afo_id='EMPTY_MACHINE_BASELINE',
            test_type='empty_machine_baseline', cycles=settings['cycles'], speed_deg_s=speed,
            acceleration_deg_s2=settings['acceleration'], min_angle_deg=settings['range'],
            max_angle_deg=settings['range']), config)
        preset = config['test_presets']['standard_empty_machine_baseline']
        meta = make_preset_metadata('standard_empty_machine_baseline', preset, p,
                                    datetime.now().astimezone().isoformat(),
                                    preset_motion_values(preset) != {
                                        'cycles': p.cycles, 'minimum_angle_deg': p.min_angle_deg,
                                        'maximum_angle_deg': p.max_angle_deg,
                                        'speed_deg_s': p.commanded_afo_speed_deg_s,
                                        'acceleration_deg_s2': p.commanded_afo_acceleration_deg_s2})
        requests.append(MappingRequest(batch_id, label, i, len(order), p, meta, str(manifest_path)))
    return requests


def workers_finished(app):
    for name in ('strain_thread', 'data_collection_thread', 'cleanup_thread',
                 'stop_worker', 'failure_metadata_worker', 'disconnect_worker',
                 'manual_step_thread', 'neutral_thread', 'continuous_thread', 'recovery_thread'):
        worker = getattr(app, name, None)
        if worker is not None and worker.is_alive():
            return False
    return not any(w.is_alive() for w in app.metadata_workers)


def quiescent(app):
    return not app.strain_test_active and app.engine.busy is None and workers_finished(app)


def ready_for_next(app):
    return (quiescent(app) and app.run_finalized
            and not app.shutdown_requested and not app.motion_coordinator.shutting_down
            and not app.motion_coordinator.stopping and app.motion_coordinator.owner is None
            and app.motion_coordinator.motor_idle_confirmed)


def completed_run(app, request):
    """Return pending, success, or failure. Never accept old metadata/partial cleanup."""
    if not quiescent(app):
        return 'pending', None
    metadata = app.run_metadata
    if not metadata or metadata.get('motion_modelling') != request.context():
        return 'failure', 'Run initialization ended without matching metadata'
    if not ready_for_next(app):
        return 'failure', 'Run cleanup, metadata finalization or confirmed idle is unresolved'
    if metadata.get('run_status') != 'completed':
        return 'failure', metadata.get('error') or 'Run did not complete'
    if metadata.get('outcome_pending_cleanup'):
        return 'failure', 'Run still has unresolved cleanup'
    idle = metadata.get('idle_confirmation', {})
    recording = metadata.get('csv_recorder', {})
    rotary = metadata.get('rotary_sensor', {}).get('recording_statistics', {})
    if (not idle.get('confirmed') or idle.get('motion_owner') is not None or idle.get('monitor_error')
            or not recording.get('worker_complete') or recording.get('pending_rows')
            or recording.get('error') or not metadata.get('sample_count')
            or recording.get('rows_persisted') != metadata.get('sample_count')
            or metadata.get('completed_cycles') != request.parameters.cycles
            or not rotary.get('delivered_samples') or rotary.get('invalid_samples')
            or rotary.get('recording_error')):
        return 'failure', 'Idle, cycle count or data finalization did not pass the batch gate'
    return 'success', None
