#!/usr/bin/env python3
"""EAST empty-machine identification. Default is a hardware-free plan preview."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys
import threading
import time
from datetime import datetime

HERE = Path(__file__).resolve().parent


def arguments(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--repo', type=Path, default=Path.cwd(), help='EAST repository directory')
    p.add_argument('--execute', action='store_true', help='Connect real hardware after interactive confirmation')
    p.add_argument('--operator', default='')
    p.add_argument('--fixture-id', default='0')
    p.add_argument('--confirmed-empty-at-90', action='store_true',
                   help='Use the operator-confirmed empty/90-degree setup; no typed prompt')
    p.add_argument('--range-deg', type=float, default=10.0, help='Nominal magnitude on each side')
    p.add_argument('--cycles', type=int, default=3)
    p.add_argument('--slow-speed', type=float, default=0.5)
    p.add_argument('--fast-speed', type=float, default=5.0)
    p.add_argument('--acceleration', type=float, default=100.0)
    p.add_argument('--output', type=Path, help='New session directory (must not exist)')
    return p.parse_args(argv)


def verify_reviewed_sources(repo, manifest):
    """Ignore only Git's LF/CRLF checkout conversion, never code/config changes."""
    for name, digest in manifest.items():
        path = repo / name
        if not path.is_file():
            raise RuntimeError(f'{name} differs from the reviewed baseline: file missing')
        raw = path.read_bytes()
        normalized = raw.replace(b'\r\n', b'\n')
        if hashlib.sha256(normalized).hexdigest() != digest:
            raise RuntimeError(
                f'{name} differs from the reviewed baseline after Windows line-ending '
                'normalization. This is a content change, not just LF/CRLF. '
                'Keep your local changes and inspect the Git diff before updating the runner.')


def load_east(repo):
    repo = repo.resolve()
    manifest = json.loads((HERE / 'source_manifest.json').read_text())
    verify_reviewed_sources(repo, manifest)
    sys.path.insert(0, str(repo))
    import east_core as core
    import east_reference as reference
    import east_execution as execution
    import east_odrive as motor
    import east_rotary_sensor as rotary
    import east_runtime as runtime
    # Do not accidentally use modules previously imported from another checkout.
    for module in (core, reference, execution, motor, rotary, runtime):
        if Path(module.__file__).resolve().parent != repo:
            raise RuntimeError(f'Wrong checkout imported: {module.__file__}')
    return core, reference, execution, motor, rotary, runtime


def build_plan(args, config, core):
    if not (math.isfinite(args.slow_speed) and math.isfinite(args.fast_speed)
            and args.slow_speed < args.fast_speed):
        raise ValueError('Slow speed must be finite and below fast speed')
    plan = []
    for i in range(1, 4):
        for label, speed in [('S', args.slow_speed), ('M', args.fast_speed)]:
            params = core.validate_test_parameters(dict(
                file_prefix=f'MAP_{label}{i}', operator=args.operator or 'PREVIEW',
                afo_id='EMPTY_MACHINE', fixture_id=args.fixture_id or 'PREVIEW',
                calibration_id='NOT_APPLICABLE_MOTOR_ROTARY_ONLY', test_type='custom',
                cycles=args.cycles, speed_deg_s=speed, acceleration_deg_s2=args.acceleration,
                min_angle_deg=args.range_deg, max_angle_deg=args.range_deg), config)
            plan.append((f'{label}{i}', params))
    return plan


def targets(parameters, config):
    """Exactly the GUI strain_test_control phase/cycle/return ordering, in nominal degrees."""
    speed = parameters.commanded_afo_speed_deg_s
    accel = parameters.commanded_afo_acceleration_deg_s2
    yield 0, 'moving_to_initial_max', parameters.max_angle_deg, speed, accel
    for cycle in range(1, parameters.cycles + 1):
        yield cycle, 'moving_to_min', -parameters.min_angle_deg, speed, accel
        yield cycle, 'moving_to_max', parameters.max_angle_deg, speed, accel
    m = config['motion']
    yield parameters.cycles, 'returning_to_machine_zero', 0.0, m['neutral_return_speed_deg_s'], m['neutral_return_acceleration_deg_s2']


class Runner:
    """All hardware access runs on this worker or the existing TransportOwner."""
    def __init__(self, args, modules, config, plan, folder, stop):
        self.args, self.modules, self.config = args, modules, config
        self.core, self.ref, self.exe, self.motor, self.rotary_module, self.runtime = modules
        self.plan, self.folder, self.stop = plan, folder, stop
        self.adapter = self.transport = self.sensor = self.manager = None
        self.recorder = None
        self.lock = self.token = None
        self.coordinator = self.ref.MotionCoordinator()
        self.latest = None
        self.last_uptime = None
        self.errors = []
        self.completed = []
        self.phase, self.cycle, self.target = 'stationary', 0, None
        self.started = None
        self.sampling_stop = threading.Event()
        self.sampler = None
        self.sampling_error = None
        self.session = {'schema': 'east-motion-mapping-1', 'physical_condition': 'empty_machine',
                        'force_acquisition': False, 'tare': 'not captured; motor/rotary identification only',
                        'runs': self.completed, 'outcome': 'preparing', 'errors': self.errors,
                        'setup_confirmation': 'operator-confirmed empty fixture at physical 90 degrees',
                        'confirmation_mode': 'explicit launcher/flag' if args.confirmed_empty_at_90 else 'interactive',
                        'config': config, 'operator': args.operator, 'fixture_id': args.fixture_id}

    def cancelled(self):
        if self.stop.is_set():
            return True
        if self.token is not None:
            try:
                self.coordinator.assert_active(self.token)
            except Exception:
                return True
        return False

    def request_stop(self):
        # Nonblocking main-thread admission cancellation and priority idle dispatch.
        self.stop.set()
        self.coordinator.request_stop()
        if self.transport:
            self.transport.cancel_motion()
            if self.adapter:
                self.transport.submit('operator-stop-idle', lambda: self.adapter.request_idle(
                    self.config['reference']['idle_confirmation_timeout_s']), priority=0)

    def save(self):
        self.core.write_json_atomic(self.folder / 'session.json', self.session)

    def snapshot(self, expected_state=None, expected_disarm=None):
        if self.cancelled():
            raise self.ref.MotionConflictError('Operator stop requested')
        cfg = self.config['reference']
        s = self.adapter.snapshot(max_capture_duration_s=cfg['maximum_feedback_capture_ms']/1000,
                                  lock_timeout_s=cfg['maximum_feedback_capture_ms']/1000)
        s.validate(time.monotonic(), cfg['feedback_stale_after_ms']/1000,
                   cfg['maximum_feedback_capture_ms']/1000)
        if self.cancelled():
            raise self.ref.MotionConflictError('Stop requested during feedback capture')
        if s.active_errors:
            raise RuntimeError(f'ODrive errors: {s.active_errors}')
        if expected_state is not None and s.current_state != int(expected_state):
            raise RuntimeError(f'Unexpected axis state {s.current_state}')
        if expected_disarm is not None and s.disarm_reason != expected_disarm:
            raise RuntimeError('ODrive disarm reason changed')
        if self.last_uptime is not None and s.system_uptime is not None and s.system_uptime < self.last_uptime:
            raise RuntimeError('Possible controller reset; reference invalid')
        self.last_uptime = s.system_uptime
        r = self.sensor.measurement(require=True)
        self.latest = s
        if self.manager and self.manager.verified:
            angle = self.manager.angle_from_position(s.position_turns)
            if abs(angle) > self.config['motion']['maximum_afo_angle_deg']:
                raise RuntimeError('Motor-derived travel exceeded configured bound')
        if self.recorder:
            if self.recorder.error:
                raise RuntimeError(f'Motor recorder failed: {self.recorder.error}')
            sample = r['sample']
            self.recorder.append([s.captured_at, time.monotonic()-self.started,
                s.captured_monotonic_s, s.position_started_monotonic_s, s.position_finished_monotonic_s,
                s.velocity_started_monotonic_s, s.velocity_finished_monotonic_s,
                s.position_turns, s.velocity_turns_s, self.manager.mapping.neutral_position_turns,
                self.manager.angle_from_position(s.position_turns), self.phase, self.cycle,
                self.target, s.current_state, s.active_errors, s.disarm_reason, s.system_uptime,
                s.capture_duration_s, s.transport_queue_wait_s, s.lock_wait_s,
                sample[0], sample[3], sample[4], sample[5], r['angle_deg'], r['sample_age_s'], r['achieved_rate_hz']])
        return s

    def cached(self, expected_state, expected_disarm=None):
        if self.cancelled() or self.sampling_error:
            raise RuntimeError(self.sampling_error or 'Stop requested')
        s = self.latest
        if s is None:
            raise RuntimeError('No cached motor feedback')
        s.validate(time.monotonic(), self.config['reference']['feedback_stale_after_ms']/1000)
        if s.active_errors or s.current_state != int(expected_state):
            raise RuntimeError('Unexpected motor state/error in cached feedback')
        if expected_disarm is not None and s.disarm_reason != expected_disarm:
            raise RuntimeError('Motor disarm reason changed')
        return s

    def collect(self):
        # Match GUI's cached-feedback boundary: wait_for_settle must not perform
        # blocking reads because it validates the cached sample against its own clock.
        while not self.sampling_stop.is_set():
            try:
                s = self.snapshot()
                if s.current_state not in (int(self.enums.AXIS_STATE_IDLE), int(self.enums.AXIS_STATE_CLOSED_LOOP_CONTROL)):
                    raise RuntimeError('Motor left allowed operating states')
            except Exception as exc:
                if not self.sampling_stop.is_set():
                    self.sampling_error = str(exc)
                    self.request_stop()
                return
            self.sampling_stop.wait(self.config['reference']['feedback_poll_interval_ms']/1000)

    def configure(self, speed, accel):
        c = self.core
        v = c.afo_speed_to_odrive_turns_s(speed, self.config)
        self.adapter.configure_trajectory(v, c.afo_acceleration_to_odrive_turns_s2(accel, self.config),
            v*self.config['motion']['controller_velocity_safety_multiplier'],
            self.enums.CONTROL_MODE_POSITION_CONTROL, self.enums.INPUT_MODE_TRAP_TRAJ)

    def idle(self):
        result = self.adapter.request_idle(self.config['reference']['idle_confirmation_timeout_s'])
        if not result.confirmed:
            raise RuntimeError(f'MOTOR IDLE UNCONFIRMED: {result.detail}; use physical E-stop')
        self.coordinator.confirm_motor_idle()
        if result.active_errors:
            raise RuntimeError(f'Controller errors at idle: {result.active_errors}')
        return result

    def move(self, angle, speed, accel, armed):
        self.coordinator.assert_active(self.token)
        target = self.manager.target_for_angle(angle)
        self.target = target
        before = self.cached(self.enums.AXIS_STATE_CLOSED_LOOP_CONTROL, armed.disarm_reason)
        distance = abs(self.core.odrive_turns_to_afo_degrees(target-before.position_turns, self.config))
        cfg = self.config['reference']
        self.manager.write_checkpoint(self.ref.MotionState.MOVING, before, False,
                                      {'source': 'motion_mapping_cli', 'phase': self.phase, 'target_turns': target})
        self.coordinator.submit_command(self.token, lambda: self.adapter.command_position(target, cancelled=self.cancelled))
        self.ref.wait_for_settle(
            lambda: self.cached(self.enums.AXIS_STATE_CLOSED_LOOP_CONTROL, armed.disarm_reason),
            target_turns=target,
            tolerance_turns=self.core.afo_degrees_to_odrive_turns(self.config['motion']['position_tolerance_deg'], self.config),
            velocity_limit_turns_s=self.core.afo_speed_to_odrive_turns_s(cfg['settle_velocity_limit_deg_s'], self.config),
            dwell_s=cfg['settle_dwell_ms']/1000,
            timeout_s=self.core.motion_timeout_seconds(distance, speed, accel, self.config),
            stale_after_s=cfg['feedback_stale_after_ms']/1000,
            cancelled=self.cancelled, expected_state=int(self.enums.AXIS_STATE_CLOSED_LOOP_CONTROL),
            expected_disarm_reason=armed.disarm_reason)

    def one_run(self, label, p):
        info = {'id': label, 'outcome': 'in_progress', 'completed_cycles': 0,
                'nominal_speed_deg_s': p.commanded_afo_speed_deg_s,
                'acceleration_deg_s2': p.commanded_afo_acceleration_deg_s2,
                'min_angle_magnitude_deg': p.min_angle_deg, 'max_angle_magnitude_deg': p.max_angle_deg,
                'cycles_requested': p.cycles, 'physical_condition': 'empty_machine',
                'reference': self.manager.metadata_snapshot()}
        self.completed.append(info)
        self.save()
        error = None
        self.started = time.monotonic()
        self.phase, self.cycle, self.target = 'pre_run_idle', 0, None
        motor_path, rotary_path = self.folder/f'{label}_motor_samples.csv', self.folder/f'{label}_rotary_samples.csv'
        info['files'] = [motor_path.name, rotary_path.name, f'{label}_metadata.json']
        try:
            s = self.snapshot(self.enums.AXIS_STATE_IDLE)
            if abs(self.manager.angle_from_position(s.position_turns)) > self.config['motion']['position_tolerance_deg']:
                raise RuntimeError('Not at verified zero before run; no automatic off-zero start')
            with motor_path.open('x', newline='', encoding='utf8') as h:
                csv.writer(h).writerow(['motor_time_iso','host_elapsed_s','motor_feedback_time_s',
                    'position_read_start_s','position_read_end_s','velocity_read_start_s','velocity_read_end_s',
                    'motor_position_turns','motor_velocity_turns_s','zero_motor_turns','nominal_motor_angle_deg',
                    'phase','cycle','target_motor_turns','axis_state','active_errors','disarm_reason','uptime',
                    'capture_duration_s','queue_wait_s','lock_wait_s','rotary_time_s','rotary_index','rotary_connection',
                    'rotary_time_ns','rotary_angle_deg','rotary_age_s','rotary_rate_hz'])
            self.recorder = self.runtime.CSVRecorder(motor_path, self.config['acquisition']['csv_flush_rows'],
                                                      lambda exc: self.errors.append(str(exc)))
            self.recorder.start()
            self.sensor.start_recording(rotary_path, self.started)
            self.token = self.coordinator.acquire(f'mapping-{label}')
            self.configure(p.commanded_afo_speed_deg_s, p.commanded_afo_acceleration_deg_s2)
            self.manager.write_checkpoint(self.ref.MotionState.ARMING, s, False, {'run': label})
            cfg = self.config['reference']
            armed = self.adapter.enter_closed_loop_holding_current(cancelled=self.cancelled,
                expected_mapper_scale=cfg['required_pos_vel_mapper_scale'],
                mapper_scale_tolerance=cfg['pos_vel_mapper_scale_tolerance'])
            self.latest = armed
            self.sampling_error = None
            self.sampling_stop.clear()
            self.sampler = threading.Thread(target=self.collect, name='mapping-feedback', daemon=False)
            self.sampler.start()
            previous_profile = (p.commanded_afo_speed_deg_s, p.commanded_afo_acceleration_deg_s2)
            for cycle, phase, angle, speed, accel in targets(p, self.config):
                if self.cancelled():
                    raise self.ref.MotionConflictError('Stopped before next phase')
                self.phase, self.cycle = phase, cycle
                if (speed, accel) != previous_profile:
                    self.configure(speed, accel)
                    previous_profile = speed, accel
                self.move(angle, speed, accel, armed)
                if phase == 'moving_to_max':
                    info['completed_cycles'] = cycle
            self.idle()
            self.phase = 'post_idle_observation'
            end = time.monotonic()+cfg['post_idle_observation_ms']/1000
            while time.monotonic() < end:
                observed = self.snapshot(self.enums.AXIS_STATE_IDLE)
                if abs(self.manager.angle_from_position(observed.position_turns)) > self.config['motion']['position_tolerance_deg']:
                    raise RuntimeError('Zero drift after idle')
                time.sleep(.02)
            if self.cancelled():
                raise self.ref.MotionConflictError('Stopped during final observation')
        except BaseException as exc:
            error = str(exc)
            self.stop.set()
            self.coordinator.request_stop()
            self.transport.cancel_motion()
            self.manager.invalidate_local(error, fault=True)
        finally:
            # Request/confirm idle before waiting for data finalization, on success or failure.
            try:
                self.idle()
                info['idle_confirmed'] = True
            except Exception as exc:
                error = f'{error or ""}; {exc}'
                info['idle_confirmed'] = False
            self.sampling_stop.set()
            if self.sampler:
                # Keep hardware lock and process alive until its owner actually exits.
                self.sampler.join(2)
                if self.sampler.is_alive():
                    error = f'{error or ""}; feedback worker still retiring'
                    print('Waiting for feedback worker; secure motor if idle unconfirmed.', flush=True)
                    while self.sampler.is_alive():
                        self.sampler.join(.2)
                self.sampler = None
            if self.sampling_error:
                error = f'{error or ""}; acquisition: {self.sampling_error}'
            try:
                info['rotary_statistics'] = self.sensor.finish_recording()
            except Exception as exc:
                error = f'{error or ""}; rotary finalization: {exc}'
            if self.recorder:
                try:
                    self.recorder.finish()
                    if self.recorder.thread.is_alive() or self.recorder.error or self.recorder.unsaved_rows():
                        raise RuntimeError('Motor data not fully persisted')
                except Exception as exc:
                    error = f'{error or ""}; motor finalization: {exc}'
                    self.core.write_json_atomic(self.folder/f'{label}_unsaved_buffer.json',
                        {'uncertain_buffer': self.recorder.unsaved_rows(), 'error': str(exc)})
                    while self.recorder.thread.is_alive():
                        self.recorder.thread.join(.2)
                self.recorder = None
            if self.stop.is_set() and not error:
                error = 'Stop requested during cleanup'
            info['outcome'] = ('aborted' if self.stop.is_set() and error and 'Stop' in error else 'error') if error else 'completed'
            info['error'] = error
            info['elapsed_s'] = time.monotonic()-self.started
            self.core.write_json_atomic(self.folder/f'{label}_metadata.json', info)
            self.save()
            if self.token and not error:
                self.coordinator.release(self.token)
                self.token = None
        if error:
            raise RuntimeError(error)

    def run(self):
        # Hardware packages are deliberately imported only in --execute worker.
        import odrive
        from odrive import enums
        self.enums = enums
        store = self.ref.AtomicStateStore(self.ref.runtime_state_directory())
        self.lock = self.ref.ProcessLock(store.lock_path)
        try:
            self.lock.acquire()  # same lock as GUI; cannot run together
            store.mark_unsafe_exit()
            self.session['software'] = self.core.git_revision(self.args.repo)
            self.session['runner_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
            self.session['source_manifest'] = json.loads((HERE/'source_manifest.json').read_text())
            self.manager = self.ref.ReferenceManager(self.config, store)
            self.manager.invalidate_local('CLI requires fresh operator-confirmed physical zero')
            h = self.config['hardware']
            device = odrive.find_any(serial_number=h['odrive_serial_number'], timeout=h['odrive_connection_timeout_s'])
            if device is None:
                raise RuntimeError('ODrive not found')
            self.transport = self.exe.TransportOwner(1, lambda: not self.stop.is_set())
            self.adapter = self.motor.ODriveAdapter(device, h['odrive_axis'], enums.AXIS_STATE_IDLE,
                                                    enums.AXIS_STATE_CLOSED_LOOP_CONTROL, transport=self.transport)
            self.adapter.validate_motion_capabilities(
                expected_mapper_scale=self.config['reference']['required_pos_vel_mapper_scale'],
                scale_tolerance=self.config['reference']['pos_vel_mapper_scale_tolerance'])
            self.idle()
            self.sensor = self.rotary_module.RotarySensorService(self.config['rotary_sensor'], self.args.repo)
            self.sensor.connect()
            deadline = time.monotonic()+8
            while not self.sensor.measurement()['valid']:
                if self.stop.is_set() or time.monotonic()>deadline:
                    raise RuntimeError('Rotary not ready: '+self.sensor.measurement()['reason'])
                time.sleep(.05)
            initial = self.snapshot(enums.AXIS_STATE_IDLE)
            self.session['initial_rotary'] = self.sensor.measurement()
            print('No motion commanded. Current rotary angle:', self.sensor.measurement()['angle_deg'], flush=True)
            print('Confirm empty fixture physically aligned at 90 degrees, square/tools removed, enclosure closed, E-stop accessible.', flush=True)
            initial = self.snapshot(enums.AXIS_STATE_IDLE)
            self.manager.establish_at_physical_neutral(initial, self.adapter.fingerprint(),
                self.args.operator, self.args.fixture_id, True, method='cli_operator_confirmed_physical_90_deg')
            self.session['reference'] = self.manager.metadata_snapshot()
            self.session['rotary'] = self.sensor.metadata()
            self.session['outcome'] = 'running'
            self.save()
            for label, p in self.plan:
                print(f'{label}: {p.commanded_afo_speed_deg_s:g} deg/s, {p.cycles} cycles', flush=True)
                self.one_run(label, p)
                print(f'{label}: completed and persisted', flush=True)
            self.session['outcome'] = 'completed'
        except BaseException as exc:
            self.errors.append(str(exc))
            self.session['outcome'] = 'aborted' if self.stop.is_set() else 'error'
            self.stop.set()
            if self.transport:
                self.transport.cancel_motion()
            if self.manager:
                self.manager.invalidate_local(str(exc), fault=True)
            raise
        finally:
            idle_ok = False
            if self.adapter:
                try:
                    self.idle()
                    idle_ok = True
                except Exception as exc:
                    self.errors.append(str(exc))
            if self.sensor:
                try:
                    self.sensor.finish_recording()
                    self.sensor.shutdown()
                except Exception as exc:
                    self.errors.append('Sensor cleanup: '+str(exc))
                    self.session['outcome'] = 'error'
            if self.manager:
                self.manager.invalidate_local('CLI finished; physical reference setup required for a new application session')
                try:
                    self.manager.write_checkpoint(self.ref.MotionState.IDLE if idle_ok else self.ref.MotionState.FAULT,
                        self.latest, False, {'source':'motion_mapping_cli','automatic_reference_restoration':False})
                except Exception as exc:
                    self.errors.append('Checkpoint: '+str(exc))
                    self.session['outcome'] = 'error'
            if not idle_ok and self.adapter:
                self.session['outcome'] = 'error'
            self.session['idle_confirmed_on_exit'] = idle_ok
            # Leave unsafe-exit marker: this diagnostic never certifies cross-session continuity.
            try:
                self.save()
            finally:
                if self.sensor:
                    for obj in (getattr(self.sensor, 'reader', None), getattr(self.sensor, 'recording', None)):
                        thread = getattr(obj, 'thread', None)
                        if thread is not None and thread.is_alive():
                            print('Waiting for sensor/data worker to retire; hardware lock retained.', flush=True)
                            while thread.is_alive():
                                thread.join(.2)
                if self.transport:
                    if not self.transport.close(timeout_s=2):
                        print('Waiting for transport to retire; hardware lock retained. Secure motor if needed.', flush=True)
                        while self.transport.thread.is_alive():
                            self.transport.thread.join(.2)
                if self.lock and self.lock.held:
                    self.lock.release()


def main(argv=None):
    args = arguments(argv)
    args.repo = args.repo.resolve()
    modules = load_east(args.repo)
    core = modules[0]
    config = core.load_tester_config(args.repo/'tester_config.json')
    plan = build_plan(args, config, core)
    print('REVIEWED EAST motor control: existing conversion, POSITION_CONTROL / TRAP_TRAJ, GUI phase order.')
    for label, p in plan:
        print(f'{label}: +/-{p.max_angle_deg:g} nominal deg; {p.commanded_afo_speed_deg_s:g} deg/s; '
              f'{p.commanded_afo_acceleration_deg_s2:g} deg/s^2; {p.cycles} cycles')
    if not args.execute:
        print('PLAN ONLY: no hardware imported/opened and no motion. Add --execute --operator ID --fixture-id ID in the lab.')
        return 0
    if not args.operator.strip() or not args.fixture_id.strip():
        raise ValueError('--execute requires operator and fixture ID')
    if not args.confirmed_empty_at_90 and not sys.stdin.isatty():
        raise ValueError('An interactive terminal or explicit --confirmed-empty-at-90 is required')
    print('Close EAST/other diagnostics. Empty fixture must already be physically at 90 degrees, tools removed, enclosure closed, E-stop accessible.')
    if not args.confirmed_empty_at_90 and input('Type EMPTY AT 90 to connect, establish the physical reference and run this series: ').strip() != 'EMPTY AT 90':
        raise ValueError('Physical setup not confirmed; nothing connected')
    folder = args.output or args.repo/'EAST Logs'/'motion_mapping'/datetime.now().strftime('%Y%m%dT%H%M%S_%f')
    folder = folder.resolve()
    folder.mkdir(parents=True, exist_ok=False)
    stop = threading.Event()
    runner = Runner(args, modules, config, plan, folder, stop)
    failure = []
    def work():
        try:
            runner.run()
        except BaseException as exc:
            failure.append(exc)
    worker = threading.Thread(target=work, name='mapping-experiment', daemon=False)
    worker.start()
    while worker.is_alive():
        try:
            worker.join(.2)
        except KeyboardInterrupt:
            runner.request_stop()
            print('\nSTOP requested. If motion persists or idle is unconfirmed, use physical E-stop. '
                  'Waiting for hardware worker; a blocked Fibre call cannot be interrupted by Python.', flush=True)
    print('Results:', folder)
    if failure:
        print('SERIES STOPPED:', failure[0], file=sys.stderr)
        return 1
    return 0 if runner.session['outcome'] == 'completed' else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError, FileNotFoundError) as exc:
        raise SystemExit(str(exc))
