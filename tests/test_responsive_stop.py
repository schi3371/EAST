"""Offline regression tests for EAST's nonblocking stop and persistence path."""

import ast
from datetime import datetime
import json
from pathlib import Path
import queue
import tempfile
import threading
import time
from types import MethodType, SimpleNamespace
import unittest
from unittest.mock import Mock

from east_core import load_tester_config, reconcile_run_outcome, write_json_atomic
from east_odrive import ODriveAdapter
from east_reference import (FeedbackAcquisitionTimeout, FeedbackError, IdleFeedbackRetry,
                            MotionConflictError, MotionCoordinator, MotionState)
from east_runtime import CheckpointWriter
from test_odrive_adapter import Device
from test_idle_feedback_retry import sample
from test_strain_sequence import TestStopped


SOURCE = Path(__file__).resolve().parents[1] / "Ortho-Sim.py"


def method(name):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "MyInterface")
    function = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == name)
    namespace = dict(threading=threading, time=time, queue=queue, datetime=datetime,
                     Path=Path, write_json_atomic=write_json_atomic,
                     RED="red", AMBER="amber", GREEN="green",
                     FeedbackAcquisitionTimeout=FeedbackAcquisitionTimeout,
                     FeedbackError=FeedbackError, IdleFeedbackRetry=IdleFeedbackRetry,
                     MotionState=MotionState, TestStopped=TestStopped,
                     reconcile_run_outcome=reconcile_run_outcome)
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])),
                 str(SOURCE), "exec"), namespace)
    return namespace[name]


def bind(app, *names):
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
                              _mark_control_health=Mock(), _request_coordinated_stop=Mock())
        bind(app, "_feedback_monitor_loop")
        started = time.monotonic()
        app._feedback_monitor_loop()
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
            stop_started_monotonic=None, stop_diagnostic_recorded=False,
            strain_test_active=False, run_metadata=None, run_finalized=True,
            acquisition_error=None, monitor_error=None,
            failure_metadata_worker=None, ui_message_queue=queue.Queue(),
            safe_idle_motor=idle, rotary_calibration=None, operator_stop_requested=False,
            update_terminal=Mock(),
        )
        bind(app, "_request_coordinated_stop", "_coordinated_stop_worker", "stop_logging")
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
        self.assertEqual(app.motion_coordinator.owner, "strain-test")
        app.motion_coordinator.release(app.run_motion_token)
        self.assertEqual(app.motion_coordinator.acquire("manual").owner, "manual")

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
        stop = threading.Event()
        sample_times = []

        def log(*args):
            sample_times.append(time.monotonic())
            if len(sample_times) == 1:
                time.sleep(0.05)
            else:
                stop.set()

        app = SimpleNamespace(system_config=load_tester_config(), test_stop_event=stop,
                              voltage_ratio_input=SimpleNamespace(getVoltageRatio=lambda: 0.1),
                              log_strain_data=log, current_cycle=1, acquisition_overruns=0,
                              maximum_acquisition_lateness_s=0.0, strain_data_buffer=[],
                              acquisition_done_event=threading.Event(),
                              update_terminal=Mock(), _request_coordinated_stop=Mock())
        bind(app, "continuous_strain_read")
        app.continuous_strain_read()
        self.assertEqual(len(sample_times), 2)
        self.assertGreaterEqual(app.acquisition_overruns, 4)
        self.assertGreater(app.maximum_acquisition_lateness_s, 0.04)
        self.assertGreater(sample_times[1] - sample_times[0], 0.055)
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
            run_metadata={"rotary_sensor": {}},
            rotary_sensor=SimpleNamespace(finish_recording=lambda: {}),
            strain_test_active=True, ui_message_queue=queue.Queue(),
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
