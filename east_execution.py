"""Headless supervised execution boundaries. No Tk or hardware-package imports.

Blocking Fibre cannot be interrupted: one executing request retains its worker,
connection and identity. Stop cancels pending writes and is next to dispatch.
"""
from concurrent.futures import Future
from dataclasses import dataclass, field, replace
from enum import Enum
from functools import wraps
import itertools
import queue
import threading
import time


class Lifecycle(str, Enum):
    DISCONNECTED = 'disconnected'
    CONNECTING = 'connecting'
    READY = 'ready'
    MOVING = 'moving'
    STOPPING = 'stopping'
    FAULT = 'fault'
    CLOSING = 'closing'


class DispatchRejected(RuntimeError):
    pass


@dataclass
class Request:
    name: str
    connection: int
    operation: int
    motion: bool
    callback: object
    guard: object
    future: Future = field(default_factory=Future)
    submitted: float = field(default_factory=time.monotonic)


class TransportOwner:
    """One bounded, prioritised dispatcher per controller connection.

    Future timeouts only bound the caller, never release executing transport.
    Epoch cancellation and caller guards are checked again on the owner thread.
    """
    def __init__(self, connection, motion_allowed=lambda: True, max_pending=64):
        self.connection = connection
        self.operation = 0
        self.motion_allowed = motion_allowed
        self.pending = queue.PriorityQueue(maxsize=max_pending)
        self.lock = threading.RLock()
        self.sequence = itertools.count()
        self.closing = False
        self.executing = None
        self.last_queue_wait_s = 0.0
        self.thread = threading.Thread(target=self._run, name=f'east-transport-{connection}', daemon=True)
        self.thread.start()

    def cancel_motion(self):
        with self.lock:
            self.operation += 1
            # Retire pending motion immediately, retaining read/Stop ordering.
            retained = []
            while True:
                try: item = self.pending.get_nowait()
                except queue.Empty: break
                request = item[2]
                if request.motion:
                    if not request.future.done():
                        request.future.set_exception(DispatchRejected('Pending motion cancelled by Stop'))
                else: retained.append(item)
                self.pending.task_done()
            for item in retained: self.pending.put_nowait(item)

    def submit(self, name, callback, *, motion=False, guard=lambda: True, priority=10,
               connection=None, operation=None):
        with self.lock:
            req = Request(name, self.connection if connection is None else connection,
                          self.operation if operation is None else operation, motion, callback, guard)
            if self.closing or req.connection != self.connection:
                req.future.set_exception(DispatchRejected('Connection retired or replaced'))
                return req.future
            if motion and (req.operation != self.operation or not self.motion_allowed() or not guard()):
                req.future.set_exception(DispatchRejected('Motion admission cancelled'))
                return req.future
            try: self.pending.put_nowait((priority, next(self.sequence), req))
            except queue.Full: req.future.set_exception(DispatchRejected('Transport queue is full'))
            return req.future

    def call(self, name, callback, **kwargs):
        if threading.current_thread() is self.thread:
            return callback()
        # GUI callbacks must submit lifecycle work to TestEngine first.
        if threading.current_thread() is threading.main_thread():
            raise RuntimeError('Hardware wait on GUI thread is prohibited')
        return self.submit(name, callback, **kwargs).result()

    def close(self, timeout_s=0.0):
        with self.lock:
            self.closing = True
            self.cancel_motion()
        if timeout_s and threading.current_thread() is not self.thread:
            self.thread.join(timeout_s)
        return not self.thread.is_alive()

    def _run(self):
        while True:
            with self.lock:
                if self.closing and self.pending.empty(): return
            try: _, _, req = self.pending.get(timeout=.05)
            except queue.Empty: continue
            try:
                if not req.future.set_running_or_notify_cancel(): continue
                with self.lock:
                    if self.closing or req.connection != self.connection:
                        raise DispatchRejected('Retired controller request')
                    if req.motion and (req.operation != self.operation or not self.motion_allowed() or not req.guard()):
                        raise DispatchRejected('Motion cancelled before transport dispatch')
                    self.executing = req
                    self.last_queue_wait_s = time.monotonic()-req.submitted
                # Never hold a lifecycle/coordinator lock across Fibre.
                req.future.set_result(req.callback())
            except Exception as exc:
                req.future.set_exception(exc)
            finally:
                with self.lock: self.executing = None
                self.pending.task_done()


def transported(*, motion=False, priority=10):
    """Route an adapter operation through its supervised transport owner."""
    def decorate(method):
        @wraps(method)
        def wrapped(self, *args, **kwargs):
            owner = getattr(self, 'transport', None)
            if owner is None: return method(self, *args, **kwargs)
            cancelled = kwargs.get('cancelled', lambda: False)
            submitted = time.monotonic()
            def execute():
                dispatched = time.monotonic()
                result = method(self, *args, **kwargs)
                if method.__name__ == "snapshot":
                    queue_wait = dispatched-submitted
                    result = replace(result, transport_queue_wait_s=queue_wait,
                        capture_started_monotonic_s=submitted,
                        capture_duration_s=result.capture_duration_s+queue_wait)
                    result.validate(max_capture_duration_s=kwargs.get("max_capture_duration_s"))
                return result
            return owner.call(method.__name__, execute,
                              motion=motion, guard=lambda: not cancelled(), priority=priority)
        return wrapped
    return decorate


class TestEngine:
    """Single background lifecycle worker; Stop bypasses it via TransportOwner.

    submit never waits. Events contain plain results for the GUI to consume.
    State, feedback, idle and persistence are independent evidence domains.
    """
    def __init__(self, events=None):
        self.events = events if events is not None else queue.Queue()
        self.pending = queue.Queue(maxsize=8)
        self.lock = threading.RLock()
        self.lifecycle = Lifecycle.DISCONNECTED
        self.busy = None
        self.closing = False
        self.generation = 0
        self.evidence = {}
        self.cancel_event = threading.Event()
        self.thread = threading.Thread(target=self._run, name='east-test-engine', daemon=True)
        self.thread.start()

    def transition(self, state):
        with self.lock:
            if self.closing and state != Lifecycle.CLOSING: return
            self.lifecycle = Lifecycle(state)
        self.events.put(('engine_state', self.lifecycle.value))

    def submit(self, name, callback):
        future = Future()
        with self.lock:
            if self.closing or self.busy is not None:
                future.set_exception(DispatchRejected('Lifecycle operation already running or closing'))
                return future
            self.cancel_event.clear()
            self.busy = name
            self.generation += 1
            self.pending.put_nowait((name, self.generation, callback, future))
        return future

    def record(self, domain, observation):
        with self.lock: self.evidence[domain] = observation

    def cancel_current(self):
        self.cancel_event.set()

    def begin_close(self):
        self.cancel_event.set()
        with self.lock:
            self.closing = True
            self.lifecycle = Lifecycle.CLOSING
        self.events.put(('engine_state', Lifecycle.CLOSING.value))

    def close(self, timeout_s=0.0):
        self.begin_close()
        if timeout_s and threading.current_thread() is not self.thread: self.thread.join(timeout_s)
        return not self.thread.is_alive()

    def _run(self):
        while True:
            with self.lock:
                if self.closing and self.busy is None: return
            try: name, generation, callback, future = self.pending.get(timeout=.05)
            except queue.Empty: continue
            try:
                if not future.set_running_or_notify_cancel():
                    continue
                if self.closing: raise DispatchRejected('Operation cancelled by shutdown')
                result = callback()
                with self.lock: self.busy = None
                future.set_result(result)
                self.events.put(('engine_done', name, generation, None))
            except Exception as exc:
                with self.lock: self.busy = None
                future.set_exception(exc)
                self.transition(Lifecycle.FAULT)
                self.events.put(('engine_done', name, generation, str(exc)))
            finally:
                with self.lock:
                    if self.generation == generation:
                        self.busy = None
                self.pending.task_done()


class AcquisitionLoop:
    """Deadline-based polling; no invented catch-up samples or disk operations."""
    def __init__(self, read, consume, stop, interval_s):
        self.read, self.consume, self.stop, self.interval_s = read, consume, stop, interval_s
        self.overruns = 0
        self.maximum_lateness_s = 0.0
        self.read_brackets = None

    def run(self):
        deadline = time.monotonic()
        while not self.stop.is_set():
            started = time.monotonic()
            value = self.read()
            finished = time.monotonic()
            self.read_brackets = (started, finished)
            self.consume(value)
            deadline += self.interval_s
            lateness = time.monotonic() - deadline
            if lateness > 0:
                self.overruns += max(1, int(lateness / self.interval_s))
                self.maximum_lateness_s = max(self.maximum_lateness_s, lateness)
                deadline = time.monotonic() + self.interval_s
            self.stop.wait(max(0.0, deadline-time.monotonic()))


@dataclass(frozen=True)
class RecoveryEvidence:
    idle: object
    feedback: object
    fingerprint: object
    continuity_proven: bool
    reason: str


class RetainedConnectionRecovery:
    """Stationary recovery; never replaces a live transport or restores a test.

    Timing/uptime units are unvalidated by default. Fresh raw turns are not proof
    of coordinate continuity. Operator reference recovery remains required then.
    """
    def run(self, adapter, retire_monitor, config, previous=None, fingerprint=None, on_idle=lambda result: None):
        retire_monitor()  # Must reject a surviving monitor/writer before reading.
        idle = adapter.request_idle(config['reference']['idle_confirmation_timeout_s'])
        if not idle.confirmed: raise RuntimeError(idle.detail)
        on_idle(idle)
        feedback = adapter.snapshot(max_capture_duration_s=config['reference']['maximum_feedback_capture_ms']/1000)
        feedback.validate(time.monotonic(), config['reference']['feedback_stale_after_ms']/1000)
        live = adapter.fingerprint()
        if feedback.active_errors or feedback.current_state != adapter.idle_state:
            raise RuntimeError('Recovery requires fresh error-free controller idle')
        factor = config['motion']['afo_degrees_per_odrive_turn']
        if abs(feedback.velocity_turns_s*factor) > config['reference']['stationary_velocity_limit_deg_s']:
            raise RuntimeError('Recovery requires stationary feedback')
        # Deliberately conservative: only the existing validated continuity path
        # may restore a mapping. This service does not infer from equal turns.
        return RecoveryEvidence(idle, feedback, live, False,
                                'Monitoring recovered; coordinate continuity not proven. Verify physical machine zero.')


class FeedbackMonitor:
    """Independent cached-feedback publisher. Disk submission never waits on fsync."""
    def __init__(self, adapter_provider, config, stop, initial, expectations, owner,
                 publish, checkpoint, control_health, on_delay, on_restored, on_fault):
        self.adapter_provider, self.config, self.stop = adapter_provider, config, stop
        self.initial, self.expectations, self.owner = initial, expectations, owner
        self.publish, self.checkpoint, self.control_health = publish, checkpoint, control_health
        self.on_delay, self.on_restored, self.on_fault = on_delay, on_restored, on_fault
        self.error = None
        self.samples = 0
        self.first_position_at = self.last_position_at = None
        self.maximum_gap_s = 0.0

    def run(self):
        from east_reference import IdleFeedbackRetry, FeedbackAcquisitionTimeout, FeedbackError
        cfg = self.config['reference']
        interval = cfg['feedback_poll_interval_ms']/1000
        checkpoint_interval = cfg['checkpoint_interval_ms']/1000
        next_checkpoint = time.monotonic()
        retry = IdleFeedbackRetry(self.config)
        last_good = self.initial
        delayed = False
        try:
            while not self.stop.is_set():
                adapter = self.adapter_provider()
                if adapter is None: return
                expected_state, expected_disarm = self.expectations()
                try:
                    snapshot = adapter.snapshot(include_phase=bool(cfg['phase_recovery_enabled']),
                        include_uptime=expected_state is None,
                        max_capture_duration_s=cfg['maximum_feedback_capture_ms']/1000)
                except FeedbackAcquisitionTimeout as exc:
                    if not retry.reject_and_retry(exc,last_good,expected_state,self.owner()): raise
                    delayed = True
                    self.on_delay(exc,retry.rejected)
                    self.stop.wait(interval)
                    continue
                retry.accept_fresh(last_good,snapshot)
                if snapshot.active_errors: raise FeedbackError(f'ODrive active errors: {snapshot.active_errors}')
                # A state request may complete while this bundle is returning.
                # Compare only to the expectation that existed when capture began.
                # Do not publish an old powered-state bundle as fresh idle evidence.
                if self.expectations() != (expected_state, expected_disarm):
                    self.stop.wait(interval)
                    continue
                if expected_state is not None and snapshot.current_state != expected_state:
                    raise FeedbackError(f'ODrive left expected powered state {expected_state} (reported {snapshot.current_state})')
                if expected_state is not None and expected_disarm is not None and snapshot.disarm_reason != expected_disarm:
                    raise FeedbackError(f'ODrive disarm reason changed during powered motion ({expected_disarm} -> {snapshot.disarm_reason})')
                self.publish(snapshot)
                last_good = snapshot
                self.samples += 1
                if self.last_position_at is not None:
                    self.maximum_gap_s=max(self.maximum_gap_s,snapshot.captured_monotonic_s-self.last_position_at)
                self.first_position_at = self.first_position_at or snapshot.captured_monotonic_s
                self.last_position_at = snapshot.captured_monotonic_s
                if delayed:
                    delayed=False
                    self.on_restored()
                if expected_state is None: self.control_health()
                if time.monotonic() >= next_checkpoint:
                    self.checkpoint(snapshot)
                    next_checkpoint=time.monotonic()+checkpoint_interval
                # Actual period includes transport duration and this wait.
                self.stop.wait(interval)
        except Exception as exc:
            self.error = str(exc)
            self.on_fault('Feedback monitor failure: '+str(exc))

    def statistics(self):
        duration = (self.last_position_at-self.first_position_at
                    if self.last_position_at is not None and self.first_position_at is not None else 0)
        return {'samples':self.samples,'achieved_rate_hz':(self.samples-1)/duration if duration>0 else 0,
                'maximum_gap_s':self.maximum_gap_s,'error':self.error,
                'timestamp_basis':'host position-read completion; original times preserved'}
