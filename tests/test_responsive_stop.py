"""Offline regression tests for EAST's nonblocking stop and persistence path."""

import ast
import csv
from datetime import datetime
import json
import os
from pathlib import Path
import queue
import tempfile
import threading
import time
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from east_core import (CSV_COLUMNS, ROTARY_RUN_COLUMNS, MOTOR_TIMING_COLUMNS, load_tester_config,
                       reconcile_run_outcome, write_json_atomic)
from east_odrive import ODriveAdapter
from east_reference import (AtomicStateStore, FeedbackAcquisitionTimeout, FeedbackError,
                            IdleFeedbackRetry, MotionConflictError, MotionCoordinator,
                            MotionState, ReferenceConfidence)
from east_runtime import CheckpointWriter
from east_execution import AcquisitionLoop, Lifecycle, FeedbackMonitor
from test_odrive_adapter import Device
from test_idle_feedback_retry import sample
from test_strain_sequence import TestStopped


SOURCE = Path(__file__).resolve().parents[1] / "Ortho-Sim.py"


def method(name):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "MyInterface")
    function = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == name)
    namespace = dict(FeedbackMonitor=FeedbackMonitor, AcquisitionLoop=AcquisitionLoop, Lifecycle=Lifecycle, threading=threading, time=time, queue=queue, datetime=datetime,
                     Path=Path, write_json_atomic=write_json_atomic, os=os, csv=csv,
                     RED="red", AMBER="amber", GREEN="green",
                     APP_VERSION="test", ReferenceConfidence=ReferenceConfidence,
                     CSV_COLUMNS=CSV_COLUMNS, ROTARY_RUN_COLUMNS=ROTARY_RUN_COLUMNS, MOTOR_TIMING_COLUMNS=MOTOR_TIMING_COLUMNS,
                     FeedbackAcquisitionTimeout=FeedbackAcquisitionTimeout,
                     FeedbackError=FeedbackError, IdleFeedbackRetry=IdleFeedbackRetry,
                     MotionState=MotionState, TestStopped=TestStopped,
                     reconcile_run_outcome=reconcile_run_outcome)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])),
                 str(SOURCE), "exec"), namespace)
    return namespace[name]


def bind(app, *names):
    # New explicit session domains; fixtures must not fabricate them via Mock.
    for name, value in {
        "engine": SimpleNamespace(busy=None, closing=False, close=Mock(return_value=True), cancel_current=Mock(), transition=Mock(), record=Mock()),
        "transport_owner": None, "csv_recorder": None,
        "ui_timing": {"drains": 0, "maximum_drain_s": 0.0, "maximum_gap_s": 0.0},
        "last_ui_drain": time.monotonic(),
    }.items():
        if not hasattr(app, name): setattr(app, name, value)
    for name in names:
        setattr(app, name, MethodType(method(name), app))


class CheckpointWriterTests(unittest.TestCase):
    def test_required_idle_checkpoint_follows_pending_moving_checkpoint(self):
        entered = threading.Event()
        release = threading.Event()
        calls = []

        def write(state, snapshot, clean_shutdown, extra):
            if snapshot == "moving":
                entered.set()
                release.wait(1.0)
            calls.append((state, snapshot))

        writer = CheckpointWriter(SimpleNamespace(write_checkpoint=write), Mock())
        writer.start()
        writer.submit(MotionState.MOVING, "moving", {})
        self.assertTrue(entered.wait(1.0))
        required = threading.Thread(target=lambda: writer.write_and_wait(
            MotionState.IDLE, "idle", clean_shutdown=False))
        required.start()
        deadline = time.monotonic() + 1.0
        while writer.pending.empty() and time.monotonic() < deadline:
            time.sleep(0.001)
        self.assertTrue(required.is_alive())
        release.set()
        required.join(1.0)
        self.assertFalse(required.is_alive())
        self.assertTrue(writer.stop(timeout_s=1.0))
        self.assertEqual(calls, [(MotionState.MOVING, "moving"), (MotionState.IDLE, "idle")])

    def test_feedback_monitor_continues_during_blocked_checkpoint_write(self):
        entered = threading.Event()
        release = threading.Event()
        captured = []

        def write(*args, **kwargs):
            entered.set()
            release.wait(1.0)

        manager = SimpleNamespace(write_checkpoint=write,
                                  store=SimpleNamespace(append_event=Mock()))
        writer = CheckpointWriter(manager, Mock())
        writer.start()

        class ThreePolls:
            def is_set(self):
                return len(captured) >= 3

            def wait(self, interval):
                entered.wait(1.0)

        def capture(**kwargs):
            captured.append(time.monotonic())
            return sample(at=captured[-1])

        app = SimpleNamespace(system_config=load_tester_config(), feedback_lock=threading.RLock(),
                              stop_worker_lock=threading.Lock(),
                              latest_feedback=sample(at=time.monotonic()),
                              expected_axis_state=None, expected_disarm_reason=None,
                              feedback_retry_pending=False, motion_coordinator=MotionCoordinator(),
                              monitor_stop_event=ThreePolls(), odrive_adapter=SimpleNamespace(snapshot=capture),
                              checkpoint_writer=writer, ui_message_queue=queue.Queue(),
                              odrive_connection_generation=1,
                              _record_feedback_observation=Mock(return_value=True),
                              _latch_feedback_fault=Mock(return_value=True),
                              _mark_control_health=Mock(), _request_coordinated_stop=Mock())
        bind(app, "_feedback_monitor_loop")
        started = time.monotonic()
        app._feedback_monitor_loop(1, app.odrive_adapter)
        self.assertLess(time.monotonic() - started, 0.1)
        self.assertEqual(len(captured), 3)
        app._request_coordinated_stop.assert_not_called()
        release.set()
        self.assertTrue(writer.stop(timeout_s=1.0))

    def test_slow_disk_does_not_delay_submission_and_preserves_order(self):
        entered = threading.Event()
        release = threading.Event()
        calls = []

        def write(state, snapshot, clean_shutdown, extra):
            entered.set()
            release.wait(1.0)
            calls.append(snapshot)

        manager = SimpleNamespace(write_checkpoint=write,
                                  store=SimpleNamespace(append_event=lambda *a: calls.append(a)))
        errors = []
        writer = CheckpointWriter(manager, errors.append)
        writer.start()
        writer.submit("moving", 1, {})
        self.assertTrue(entered.wait(1.0))
        started = time.monotonic()
        writer.submit("moving", 2, {})
        writer.submit_event("event", {})
        self.assertLess(time.monotonic() - started, 0.05)
        release.set()
        self.assertTrue(writer.stop(timeout_s=1.0))
        self.assertEqual(calls, [1, 2, ("event", {})])
        self.assertEqual(errors, [])

    def test_persistence_failure_is_reported(self):
        failed = threading.Event()

        def write(*args, **kwargs):
            raise OSError("disk unavailable")

        writer = CheckpointWriter(SimpleNamespace(write_checkpoint=write),
                                  lambda error: failed.set())
        writer.start()
        writer.submit("moving", 1, {})
        self.assertTrue(failed.wait(1.0))
        with self.assertRaisesRegex(RuntimeError, "persistence failed"):
            writer.submit("moving", 2, {})
        self.assertFalse(writer.stop(timeout_s=1.0))


class TransportTests(unittest.TestCase):
    def test_queued_target_is_cancelled_after_transport_lock_wait(self):
        device = Device()
        adapter = ODriveAdapter(device, 0, idle_state=1, closed_loop_state=8)
        coordinator = MotionCoordinator()
        token = coordinator.acquire("test")
        occupied = threading.Event()
        release = threading.Event()
        outcomes = []

        def hold_transport():
            with adapter.io_lock:
                occupied.set()
                release.wait(1.0)

        blocker = threading.Thread(target=hold_transport)
        blocker.start()
        self.assertTrue(occupied.wait(1.0))

        def command():
            try:
                coordinator.submit_command(token, lambda: adapter.command_position(
                    42.0, cancelled=lambda: coordinator.stopping))
            except Exception as exc:
                outcomes.append(exc)

        pending = threading.Thread(target=command)
        pending.start()
        time.sleep(0.02)
        coordinator.request_stop()
        release.set()
        pending.join(1.0)
        blocker.join(1.0)
        self.assertEqual(device.axis0.controller.input_pos, -999.0)
        self.assertEqual(len(outcomes), 1)
        self.assertIn("cancelled", str(outcomes[0]))
        self.assertTrue(adapter.request_idle().confirmed)
        self.assertEqual(device.axis0.controller.input_pos, -999.0)

    def test_queued_arming_is_cancelled_after_transport_lock_wait(self):
        device = Device()
        adapter = ODriveAdapter(device, 0, idle_state=1, closed_loop_state=8)
        occupied = threading.Event()
        release = threading.Event()
        cancelled = threading.Event()
        errors = []

        def hold_transport():
            with adapter.io_lock:
                occupied.set()
                release.wait(1.0)

        blocker = threading.Thread(target=hold_transport)
        blocker.start()
        self.assertTrue(occupied.wait(1.0))

        def arm():
            try:
                adapter.request_closed_loop_holding_current(cancelled=cancelled.is_set)
            except Exception as exc:
                errors.append(exc)

        pending = threading.Thread(target=arm)
        pending.start()
        time.sleep(0.02)
        cancelled.set()
        release.set()
        pending.join(1.0)
        blocker.join(1.0)
        self.assertEqual(device.axis0.request_count, 0)
        self.assertEqual(device.axis0.controller.input_pos, -999.0)
        self.assertEqual(len(errors), 1)
    def test_idle_lock_wait_is_bounded_without_sending_command(self):
        device = Device()
        adapter = ODriveAdapter(device, 0, idle_state=1, closed_loop_state=8)
        held = threading.Event()
        release = threading.Event()

        def occupy():
            with adapter.io_lock:
                held.set()
                release.wait(1.0)

        blocker = threading.Thread(target=occupy)
        blocker.start()
        self.assertTrue(held.wait(1.0))
        try:
            started = time.monotonic()
            result = adapter.request_idle(0.05)
            self.assertLess(time.monotonic() - started, 0.15)
            self.assertFalse(result.confirmed)
            self.assertFalse(result.requested)
            self.assertEqual(device.axis0.request_count, 0)
            with self.assertRaisesRegex(Exception, "I/O lock unavailable"):
                adapter.snapshot(lock_timeout_s=0.02)
        finally:
            release.set()
            blocker.join(1.0)

    def test_blocked_transport_remains_a_live_worker_past_deadline(self):
        entered = threading.Event()
        release = threading.Event()

        class BlockingAxis:
            @property
            def requested_state(self):
                return 8

            @requested_state.setter
            def requested_state(self, state):
                entered.set()
                release.wait(1.0)

        adapter = ODriveAdapter(SimpleNamespace(axis0=BlockingAxis()), 0, 1, 8)
        results = []
        worker = threading.Thread(target=lambda: results.append(adapter.request_idle(0.05)))
        worker.start()
        self.assertTrue(entered.wait(1.0))
        time.sleep(0.08)
        self.assertTrue(worker.is_alive())
        self.assertEqual(results, [])
        release.set()
        worker.join(1.0)
        self.assertFalse(results[0].confirmed)
        self.assertIn("exceeded deadline", results[0].detail)


class GuiWorkerTests(unittest.TestCase):
    def test_required_checkpoints_use_ordered_writer_when_running(self):
        app = SimpleNamespace(checkpoint_writer=Mock(), reference_manager=Mock())
        bind(app, "_write_runtime_checkpoint")
        app._write_runtime_checkpoint(MotionState.IDLE, "snapshot", False, {"reason": "stop"})
        app.checkpoint_writer.write_and_wait.assert_called_once_with(
            MotionState.IDLE, "snapshot", False, {"reason": "stop"})
        app.reference_manager.write_checkpoint.assert_not_called()

    def app(self, idle):
        coordinator = MotionCoordinator()
        token = coordinator.acquire("strain-test")
        app = SimpleNamespace(
            motion_coordinator=coordinator, run_motion_token=token,
            odrive_adapter=object(),
            test_stop_event=threading.Event(), neutral_stop_event=threading.Event(),
            continuous_stop_event=threading.Event(), stop_worker_lock=threading.Lock(),
            stop_worker=None, stop_token=None, stop_result=None,
            stop_operation=None, stop_sequence=0, odrive_connection_generation=1,
            stop_started_monotonic=None, stop_diagnostic_recorded=False,
            strain_test_active=False, run_metadata=None, run_finalized=True,
            acquisition_error=None, monitor_error=None,
            failure_metadata_worker=None, ui_message_queue=queue.Queue(),
            metadata_workers=[],
            shutdown_requested=False,
            safe_idle_motor=idle, rotary_calibration=None, operator_stop_requested=False,
            reference_manager=Mock(),
            update_terminal=Mock(),
        )
        bind(app, "_request_coordinated_stop", "_coordinated_stop_worker",
             "_retire_stop_worker", "stop_logging")
        return app

    def test_stop_callback_returns_while_idle_is_stalled_and_blocks_restart(self):
        entered = threading.Event()
        release = threading.Event()
        app = self.app(None)

        def slow_idle(*args, **kwargs):
            entered.set()
            release.wait(1.0)
            app.motion_coordinator.confirm_motor_idle()
            return SimpleNamespace(confirmed=True)

        app.safe_idle_motor = slow_idle
        started = time.monotonic()
        app.stop_logging()
        self.assertLess(time.monotonic() - started, 0.05)
        self.assertTrue(entered.wait(1.0))
        self.assertTrue(app.stop_worker.is_alive())
        self.assertTrue(app.test_stop_event.is_set())
        with self.assertRaises(MotionConflictError):
            app.motion_coordinator.assert_active(app.run_motion_token)
        with self.assertRaises(MotionConflictError):
            app.motion_coordinator.acquire("manual")
        first_worker = app.stop_worker
        app.stop_logging()
        self.assertIs(app.stop_worker, first_worker)
        release.set()
        first_worker.join(1.0)
        app._retire_stop_worker()
        self.assertEqual(app.motion_coordinator.owner, "strain-test")
        app.motion_coordinator.release(app.run_motion_token)
        self.assertEqual(app.motion_coordinator.acquire("manual").owner, "manual")

    def test_fresh_fault_after_completed_ownerless_stop_gets_new_idle_evidence(self):
        app = self.app(None)
        app.motion_coordinator.release(app.run_motion_token)
        calls = []

        def idle(reason, invalidate_reference=False, token=None):
            calls.append((reason, invalidate_reference))
            app.motion_coordinator.confirm_motor_idle()
            return SimpleNamespace(confirmed=True)

        app.safe_idle_motor = idle
        app._request_coordinated_stop("ordinary stop")
        first = app.stop_worker
        first.join(1.0)
        app._retire_stop_worker()
        self.assertTrue(app.stop_result.confirmed)
        app._request_coordinated_stop("new feedback fault", fault=True)
        second = app.stop_worker
        second.join(1.0)
        app._retire_stop_worker()
        self.assertIsNot(first, second)
        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[-1], ("new feedback fault", True))
        self.assertEqual(app.monitor_error, "new feedback fault")

    def test_ownerless_stop_retains_reservation_until_worker_has_exited(self):
        entered = threading.Event()
        release = threading.Event()

        class BlockingQueue(queue.Queue):
            def put(self, item, *args, **kwargs):
                if item[0] == "status" and threading.current_thread().name == "east-idle-stop":
                    entered.set()
                    release.wait(1.0)
                return super().put(item, *args, **kwargs)

        app = self.app(None)
        app.motion_coordinator.release(app.run_motion_token)
        app.ui_message_queue = BlockingQueue()

        def idle(*args, **kwargs):
            app.motion_coordinator.confirm_motor_idle()
            return SimpleNamespace(confirmed=True)

        app.safe_idle_motor = idle
        app._request_coordinated_stop("ordinary stop")
        try:
            self.assertTrue(entered.wait(1.0))
            self.assertTrue(app.motion_coordinator.motor_idle_confirmed)
            app._retire_stop_worker()
            with self.assertRaises(MotionConflictError):
                app.motion_coordinator.acquire("manual")
        finally:
            release.set()
            app.stop_worker.join(1.0)
        app._retire_stop_worker()
        self.assertEqual(app.motion_coordinator.acquire("manual").owner, "manual")

    def test_fault_escalates_live_stop_and_rechecks_idle(self):
        app = self.app(None)
        app.motion_coordinator.release(app.run_motion_token)
        entered = threading.Event()
        release = threading.Event()
        calls = []

        def idle(reason, invalidate_reference=False, token=None):
            calls.append((reason, invalidate_reference))
            if len(calls) == 1:
                entered.set()
                release.wait(1.0)
            app.motion_coordinator.confirm_motor_idle()
            return SimpleNamespace(confirmed=True)

        app.safe_idle_motor = idle
        app._request_coordinated_stop("operator stop")
        worker = app.stop_worker
        self.assertTrue(entered.wait(1.0))
        started = time.monotonic()
        app._request_coordinated_stop("stale feedback", fault=True)
        self.assertLess(time.monotonic() - started, 0.05)
        self.assertIs(worker, app.stop_worker)
        self.assertIsNone(app.stop_result)
        release.set()
        worker.join(1.0)
        self.assertEqual(len(calls), 2)
        self.assertTrue(calls[-1][1])
        self.assertIn("stale feedback", calls[-1][0])
        self.assertTrue(app.stop_result.confirmed)

    def test_fault_escalation_updates_existing_incomplete_run_outcome(self):
        entered = threading.Event()
        release = threading.Event()
        app = self.app(None)

        def idle(*args, **kwargs):
            entered.set()
            release.wait(1.0)
            app.motion_coordinator.confirm_motor_idle()
            return SimpleNamespace(confirmed=True)

        app.safe_idle_motor = idle
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "run_metadata.json"
            path.write_text(json.dumps({"run_status": "started"}))
            app.strain_test_active = True
            app.run_metadata = {"run_status": "started"}
            app.metadata_file_name = str(path)
            app.finalize_lock = threading.Lock()
            app.run_finalized = False
            app.sample_count = 3
            app.acquisition_overruns = 0
            app.maximum_acquisition_lateness_s = 0.0
            bind(app, "_persist_interim_outcome")
            app._request_coordinated_stop("operator stop")
            self.assertTrue(entered.wait(1.0))
            app._request_coordinated_stop("feedback stale", fault=True)
            release.set()
            app.stop_worker.join(1.0)
            for worker in app.metadata_workers:
                worker.join(1.0)
            saved = json.loads(path.read_text())
            self.assertEqual(saved["run_status"], "error")
            self.assertIn("feedback stale", saved["error"])
            self.assertFalse(saved["idle_confirmation"]["confirmed"])
            app.reference_manager.invalidate_local.assert_called_with("feedback stale", fault=True)

    def test_isolated_exit_requires_explicit_power_acknowledgement(self):
        app = self.app(None)
        app.isolated_exit_requested = False
        app.shutdown_requested = True
        app.isolated_exit_offered = True
        app.show_resizable_confirmation = Mock(return_value=False)
        app._isolated_exit_worker = Mock()
        app._force_process_exit = Mock()
        app._schedule_isolated_exit_deadline = Mock()
        app.set_status = Mock()
        app.isolated_exit_button = Mock()
        app.master = SimpleNamespace(after=Mock())
        bind(app, "exit_after_motor_power_is_isolated")
        app.exit_after_motor_power_is_isolated()
        self.assertFalse(app.isolated_exit_requested)
        app._request_coordinated_stop = Mock()
        app._request_coordinated_stop.assert_not_called()
        prompt = app.show_resizable_confirmation.call_args.kwargs["message"]
        self.assertTrue(app.show_resizable_confirmation.call_args.kwargs["dangerous"])
        self.assertIn("physical E-stop", prompt)
        self.assertIn("motor power", prompt)
        self.assertIn("buffered data may be lost", prompt)
        app.show_resizable_confirmation.return_value = True
        app.exit_after_motor_power_is_isolated()
        self.assertTrue(app.isolated_exit_requested)
        self.assertTrue(app.motion_coordinator.shutting_down)
        app._request_coordinated_stop.assert_called_once_with(
            "power-isolated exit; idle unconfirmed", fault=True)
        app._schedule_isolated_exit_deadline.assert_called_once()

    def test_disconnect_callback_does_not_release_adapter_during_stalled_stop(self):
        release = threading.Event()
        app = self.app(None)
        app.motion_coordinator.release(app.run_motion_token)
        app.odrive_adapter = object()
        app.disconnect_worker = None

        def slow_idle(*args, **kwargs):
            release.wait(1.0)
            return SimpleNamespace(confirmed=False)

        app.safe_idle_motor = slow_idle
        app._disconnect_after_idle_worker = lambda: app.stop_worker.join()
        bind(app, "disconnect_odrive")
        started = time.monotonic()
        app.disconnect_odrive()
        self.assertLess(time.monotonic() - started, 0.05)
        self.assertTrue(app.stop_worker.is_alive())
        self.assertIsNotNone(app.odrive_adapter)
        self.assertTrue(app.disconnect_worker.is_alive())
        release.set()
        app.stop_worker.join(1.0)
        app.disconnect_worker.join(1.0)

    def test_rotary_fault_poll_requests_async_stop_without_motor_io(self):
        app = SimpleNamespace(rotary_sensor=SimpleNamespace(
                                  reader=None, measurement=lambda: {"valid": False, "reason": "stale"}),
                              last_rotary_notice="stale", strain_test_active=True,
                              test_stop_event=threading.Event(),
                              _request_coordinated_stop=Mock(), safe_idle_motor=Mock(),
                              _refresh_motion_controls=Mock())
        bind(app, "_poll_rotary_sensor")
        started = time.monotonic()
        app._poll_rotary_sensor()
        self.assertLess(time.monotonic() - started, 0.05)
        app._request_coordinated_stop.assert_called_once_with("Rotary sensor: stale", fault=True)
        app.safe_idle_motor.assert_not_called()

    def test_fault_persists_incomplete_metadata_before_stalled_idle_finishes(self):
        release = threading.Event()
        def stalled_idle(*args, **kwargs):
            release.wait(1.0)
            return SimpleNamespace(confirmed=False)

        app = self.app(stalled_idle)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "run_metadata.json"
            path.write_text(json.dumps({"run_status": "started"}))
            app.strain_test_active = True
            app.run_finalized = False
            app.run_metadata = {"run_status": "started"}
            app.metadata_file_name = str(path)
            app.finalize_lock = threading.Lock()
            app.sample_count = 200
            app.acquisition_overruns = 3
            app.maximum_acquisition_lateness_s = 0.4
            bind(app, "_persist_interim_outcome")
            app._request_coordinated_stop("feedback stale", fault=True)
            app.failure_metadata_worker.join(1.0)
            saved = json.loads(path.read_text())
            self.assertEqual(saved["run_status"], "error")
            self.assertTrue(saved["outcome_pending_cleanup"])
            self.assertFalse(saved["idle_confirmation"]["confirmed"])
            self.assertEqual(saved["sample_count"], 200)
            self.assertTrue(app.stop_worker.is_alive())
            release.set()
            app.stop_worker.join(1.0)

    def test_queue_batches_are_bounded_and_plot_redraws_once(self):
        app = SimpleNamespace(ui_message_queue=queue.Queue(), plot_data_queue=queue.Queue(),
                              update_terminal=Mock(), update_plot_data=Mock(), update_plot=Mock(),
                              _retire_stop_worker=Mock(),
                              _poll_rotary_sensor=Mock(), _diagnose_stalled_workers=Mock(),
                              update_manual_position_display=Mock(),
                              master=SimpleNamespace(after=Mock()))
        for _ in range(300):
            app.ui_message_queue.put(("terminal", "event"))
            app.plot_data_queue.put((1, 2))
        bind(app, "_drain_ui_queues")
        app._drain_ui_queues()
        self.assertEqual(app.ui_message_queue.qsize(), 200)
        self.assertEqual(app.plot_data_queue.qsize(), 100)
        self.assertEqual(app.update_plot_data.call_count, 200)
        app.update_plot.assert_called_once()

    def test_slow_csv_flush_records_overrun_without_immediate_catchup(self):
        class ControlledClock:
            def __init__(self): self.now = 0.0
            def monotonic(self): return self.now
            def advance(self, seconds): self.now += seconds

        class ControlledStop:
            def __init__(self, clock): self.clock, self.stopped, self.waits = clock, False, []
            def is_set(self): return self.stopped
            def set(self): self.stopped = True
            def wait(self, seconds):
                if self.stopped: return True
                self.waits.append(seconds)
                self.clock.advance(seconds)
                return False

        clock = ControlledClock()
        stop = ControlledStop(clock)
        sample_times = []

        def log(*args):
            sample_times.append(clock.monotonic())
            if len(sample_times) == 1:
                clock.advance(0.05)
            else:
                stop.set()

        app = SimpleNamespace(system_config=load_tester_config(), test_stop_event=stop,
                              voltage_ratio_input=SimpleNamespace(getVoltageRatio=lambda: 0.1),
                              log_strain_data=log, current_cycle=1, acquisition_overruns=0,
                              maximum_acquisition_lateness_s=0.0, strain_data_buffer=[],
                              acquisition_done_event=threading.Event(),
                              update_terminal=Mock(), _request_coordinated_stop=Mock())
        bind(app, "continuous_strain_read")
        with patch("east_execution.time.monotonic", side_effect=clock.monotonic):
            app.continuous_strain_read()
        self.assertEqual(len(sample_times), 2)
        self.assertEqual(sample_times, [0.0, 0.058])
        self.assertEqual(app.acquisition_overruns, 5)
        self.assertAlmostEqual(app.maximum_acquisition_lateness_s, 0.042)
        self.assertEqual(stop.waits, [0.008])
        app._request_coordinated_stop.assert_not_called()

    def test_finalization_does_not_request_live_odrive_diagnostics(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "run_metadata.json"
            app = SimpleNamespace(finalize_lock=threading.Lock(), run_finalized=False,
                                  run_metadata={"run_status": "started"}, metadata_file_name=str(path),
                                  sample_count=200, completed_cycles=0, acquisition_overruns=4,
                                  maximum_acquisition_lateness_s=0.406, test_started_monotonic=time.monotonic(),
                                  reference_manager=SimpleNamespace(metadata_snapshot=lambda: {}),
                                  motion_coordinator=SimpleNamespace(owner="strain-test", motor_idle_confirmed=False),
                                  monitor_error="feedback stale", odrive_configuration_snapshot=Mock(side_effect=RuntimeError("hung")),
                                  set_status=Mock(), update_terminal=Mock(), ui_message_queue=queue.Queue())
            bind(app, "finalize_run")
            app.finalize_run("error", "feedback stale", idle_confirmed=False)
            saved = json.loads(path.read_text())
            self.assertEqual(saved["run_status"], "error")
            self.assertFalse(saved["idle_confirmation"]["confirmed"])
            self.assertEqual(saved["acquisition_overruns"], 4)
            app.odrive_configuration_snapshot.assert_not_called()

    def test_shutdown_latch_blocks_new_motion_and_reconnect(self):
        app = self.app(None)
        app.motion_coordinator.begin_shutdown()
        bind(app, "connect_system", "start_strain_test")
        app.connect_system()
        app.start_strain_test()
        self.assertIn("shutdown", app.update_terminal.call_args.args[0])
        with self.assertRaises(MotionConflictError):
            app.motion_coordinator.acquire("manual-step")

    def test_early_initialization_failure_restores_form_without_enabling_motion(self):
        coordinator = MotionCoordinator()
        token = coordinator.acquire("strain-test")
        coordinator.request_stop()
        app = SimpleNamespace(
            system_config=load_tester_config(), stop_worker=None, data_collection_thread=None,
            rotary_sensor=SimpleNamespace(finish_recording=lambda: {}),
            run_metadata=None, metadata_file_name=None, run_finalized=True,
            run_motion_token=token, motion_coordinator=coordinator,
            metadata_workers=[],
            strain_test_active=True, ui_message_queue=queue.Queue(),
            update_terminal=Mock(),
        )
        bind(app, "_cleanup_failed_start")
        app._cleanup_failed_start("configuration failed")
        self.assertFalse(app.strain_test_active)
        self.assertEqual(coordinator.owner, "strain-test")
        self.assertEqual(app.ui_message_queue.get_nowait(), ("inputs", "normal"))
        self.assertEqual(app.ui_message_queue.get_nowait(), ("run_buttons", "normal"))
        with self.assertRaises(MotionConflictError):
            coordinator.acquire("manual-step")

    def test_ordinary_quit_offers_isolated_exit_when_idle_unavailable(self):
        coordinator = MotionCoordinator()
        coordinator.begin_shutdown()
        app = SimpleNamespace(motion_coordinator=coordinator, isolated_exit_requested=False,
                              shutdown_started_monotonic=time.monotonic(), odrive_adapter=object(),
                              metadata_workers=[],
                              _retire_stop_worker=Mock(),
                              _offer_isolated_exit=Mock(), set_status=Mock(),
                              master=SimpleNamespace(after=Mock()))
        bind(app, "_poll_shutdown_ready")
        app._poll_shutdown_ready()
        app._offer_isolated_exit.assert_called_once_with("Idle not confirmed")
        app.master.after.assert_not_called()

    def test_power_isolated_exit_keeps_incomplete_metadata_with_surviving_worker(self):
        with tempfile.TemporaryDirectory() as folder:
            store = AtomicStateStore(Path(folder))
            store.mark_unsafe_exit()
            path = Path(folder) / "run_metadata.json"
            path.write_text(json.dumps({"run_status": "started"}))
            strain_path = Path(folder) / "run_strain_data.csv"
            strain_path.write_text("original saved row\n")
            coordinator = MotionCoordinator()
            token = coordinator.acquire("strain-test")
            coordinator.begin_shutdown()
            release = threading.Event()
            worker = threading.Thread(target=lambda: release.wait(1.0), daemon=True)
            worker.start()
            app = SimpleNamespace(
                state_store=store, reference_manager=SimpleNamespace(
                    mapping="prior", confidence=ReferenceConfidence.VERIFIED, reason="prior"),
                metadata_file_name=str(path), run_metadata={"run_status": "started"},
                run_finalized=False, strain_test_active=True,
                sample_count=200, acquisition_overruns=2,
                maximum_acquisition_lateness_s=0.4, monitor_error="stale",
                motion_coordinator=coordinator, stop_operation=None,
                metadata_workers=[], isolated_exit_requested=True,
                finalize_lock=threading.Lock(), strain_file_name=str(strain_path),
                strain_data_buffer=[["buffered row"]],
                acquisition_done_event=threading.Event(), checkpoint_writer=None,
                ui_message_queue=queue.Queue(), _force_process_exit=Mock(),
            )
            bind(app, "_persist_interim_outcome", "_isolated_exit_worker")
            app.acquisition_done_event.set()
            try:
                app._isolated_exit_worker()
                saved = json.loads(path.read_text())
                self.assertEqual(saved["run_status"], "error")
                self.assertTrue(saved["outcome_pending_cleanup"])
                self.assertFalse(saved["idle_confirmation"]["confirmed"])
                self.assertTrue(worker.is_alive())
                self.assertTrue(store.unsafe_exit_present())
                self.assertEqual(strain_path.read_text(), "original saved row\n")
                self.assertIn("buffered row", (Path(folder) / "run_strain_data_unsaved_buffer.csv").read_text())
                self.assertEqual(app.reference_manager.confidence, ReferenceConfidence.FAULT)
                app._force_process_exit.assert_called_once()
            finally:
                release.set()
                worker.join(1.0)

    def test_normal_shutdown_clears_marker_and_releases_lock(self):
        with tempfile.TemporaryDirectory() as folder:
            store = AtomicStateStore(Path(folder))
            store.mark_unsafe_exit()
            coordinator = MotionCoordinator()
            coordinator.begin_shutdown()
            app = SimpleNamespace(
                state_store=store, motion_coordinator=coordinator,
                isolated_exit_requested=False, odrive_adapter=None,
                isolated_exit_offered=False,
                metadata_workers=[], _stop_watchdog=Mock(), _stop_feedback_monitor=Mock(),
                watchdog_thread=None, monitor_thread=None, checkpoint_writer=None,
                rotary_calibration=None, rotary_sensor=Mock(), voltage_ratio_input=None,
                previous_unsafe_exit=False, unsafe_exit_recovered_in_session=False,
                process_lock=Mock(), ui_message_queue=queue.Queue(), odrive_controller=object(),
            )
            bind(app, "_shutdown_cleanup_worker")
            app._shutdown_cleanup_worker()
            self.assertFalse(store.unsafe_exit_present())
            app.process_lock.release.assert_called_once()
            self.assertEqual(app.ui_message_queue.get_nowait(), ("shutdown_ready",))

    def test_connected_shutdown_blocks_motion_during_delayed_cleanup(self):
        with tempfile.TemporaryDirectory() as folder:
            store = AtomicStateStore(Path(folder))
            store.mark_unsafe_exit()
            coordinator = MotionCoordinator()
            coordinator.begin_shutdown()
            coordinator.confirm_motor_idle()
            entered = threading.Event()
            release = threading.Event()

            def read_feedback():
                entered.set()
                release.wait(1.0)
                return "fresh idle"

            app = SimpleNamespace(
                state_store=store, motion_coordinator=coordinator,
                isolated_exit_requested=False, isolated_exit_offered=False,
                odrive_adapter=object(), metadata_workers=[],
                _stop_watchdog=Mock(), _stop_feedback_monitor=Mock(),
                watchdog_thread=None, monitor_thread=None, checkpoint_writer=None,
                _read_adapter_feedback=read_feedback,
                reference_manager=SimpleNamespace(verified=True), monitor_error=None,
                previous_unsafe_exit=False, unsafe_exit_recovered_in_session=False,
                rotary_calibration=None, rotary_sensor=Mock(), voltage_ratio_input=None,
                process_lock=Mock(), ui_message_queue=queue.Queue(), odrive_controller=object(),
            )
            app.engine = SimpleNamespace(close=Mock(return_value=True))
            app.transport_owner = None
            worker_method = method("_shutdown_cleanup_worker")
            persist = Mock(return_value=True)
            worker_method.__globals__["persist_idle_continuity_checkpoint"] = persist
            worker_method.__globals__["AXIS_STATE_IDLE"] = 1
            worker = threading.Thread(target=lambda: worker_method(app))
            worker.start()
            try:
                self.assertTrue(entered.wait(1.0))
                with self.assertRaises(MotionConflictError):
                    coordinator.acquire("manual")
                with self.assertRaises(MotionConflictError):
                    coordinator.reserve_observation("calibration")
            finally:
                release.set()
                worker.join(1.0)
            self.assertFalse(worker.is_alive())
            persist.assert_called_once()
            self.assertFalse(store.unsafe_exit_present())
            app.process_lock.release.assert_called_once()
            self.assertEqual(app.ui_message_queue.get_nowait(), ("shutdown_ready",))

    def test_run_finalizes_error_but_keeps_owner_while_stop_worker_is_stalled(self):
        release = threading.Event()
        coordinator = MotionCoordinator()
        token = coordinator.acquire("strain-test")
        worker = threading.Thread(target=lambda: release.wait(3.0), daemon=True)
        worker.start()
        coordinator.begin_async_stop()
        app = SimpleNamespace(
            run_motion_token=token, motion_coordinator=coordinator,
            test_stop_event=threading.Event(), operator_stop_requested=False,
            acquisition_error="feedback stale", completed_cycles=0,
            system_config=load_tester_config(), data_collection_thread=None,
            failure_metadata_worker=None, stop_worker=worker, stop_result=None,
            metadata_workers=[],
            run_metadata={"rotary_sensor": {}},
            rotary_sensor=SimpleNamespace(finish_recording=lambda: {}),
            strain_test_active=True, ui_message_queue=queue.Queue(),
            _retire_stop_worker=Mock(),
            _request_coordinated_stop=Mock(), update_terminal=Mock(), finalize_run=Mock(),
        )
        bind(app, "strain_test_control", "_release_run_ownership_when_workers_exit")
        app.test_stop_event.set()
        try:
            app.strain_test_control()
            self.assertEqual(app.finalize_run.call_args.args[0], "error")
            self.assertFalse(app.finalize_run.call_args.kwargs["idle_confirmed"])
            self.assertEqual(coordinator.owner, "strain-test")
            self.assertTrue(app.cleanup_thread.is_alive())
            with self.assertRaises(MotionConflictError):
                coordinator.acquire("another")
        finally:
            release.set()
            worker.join(1.0)
            app.cleanup_thread.join(1.0)


if __name__ == "__main__":
    unittest.main()
