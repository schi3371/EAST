"""Behavioural tests across actual transport, lifecycle, acquisition and disk boundaries."""
import csv
from pathlib import Path
import queue
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from east_execution import (TransportOwner, TestEngine, Lifecycle, DispatchRejected,
                            RetainedConnectionRecovery, AcquisitionLoop)
from east_odrive import ODriveAdapter
from east_reference import AtomicStateStore, FeedbackAcquisitionTimeout, MotionCoordinator, MotionConflictError
from east_runtime import CSVRecorder, CheckpointWriter
from east_core import load_tester_config
from east_shutdown_ui import BlockedShutdownDialog
from test_odrive_adapter import Device


class Services(unittest.TestCase):
    def setUp(self):
        self.engine = TestEngine()
        self.coordinator = MotionCoordinator()
        self.owner = TransportOwner(1, lambda: not self.coordinator.stopping and not self.coordinator.shutting_down)
        self.device = Device()
        self.adapter = ODriveAdapter(self.device, 0, 1, 8, transport=self.owner)
        self.releases = []

    def tearDown(self):
        for release in self.releases: release.set()
        self.engine.close(2)
        self.owner.close(2)
        self.assertFalse(self.engine.thread.is_alive())
        self.assertFalse(self.owner.thread.is_alive())

    def test_pending_motion_cancelled_stop_is_next_executed_request(self):
        entered, release = threading.Event(), threading.Event()
        self.releases.append(release)
        first = self.owner.submit('blocked read', lambda: (entered.set(), release.wait(2)))
        self.assertTrue(entered.wait(1))
        calls=[]
        motion = self.owner.submit('old target', lambda: calls.append('target'), motion=True, priority=5)
        read = self.owner.submit('poll', lambda: calls.append('read'))
        self.coordinator.request_stop()
        self.owner.cancel_motion()
        stop = self.owner.submit('idle', lambda: calls.append('idle'), priority=0)
        release.set();first.result(1);stop.result(1);read.result(1)
        with self.assertRaises(DispatchRejected):motion.result(1)
        self.assertEqual(calls,['idle','read'])

    def test_ownership_guard_rechecked_at_dispatch(self):
        entered, release = threading.Event(), threading.Event();self.releases.append(release)
        blocked = self.owner.submit('blocked', lambda: (entered.set(),release.wait(2)))
        self.assertTrue(entered.wait(1))
        allowed=[True];calls=[]
        future=self.owner.submit('arm',lambda: calls.append(1),motion=True,guard=lambda:allowed[0])
        allowed[0]=False;release.set();blocked.result(1)
        with self.assertRaises(DispatchRejected):future.result(1)
        self.assertEqual(calls,[])

    def test_old_connection_and_operation_never_dispatch(self):
        for arguments in ({'connection':0},{'operation':-1,'motion':True}):
            with self.assertRaises(DispatchRejected):self.owner.submit('stale',lambda: self.fail('dispatched'),**arguments).result(1)

    def test_gui_hardware_wait_is_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'GUI thread'):self.adapter.snapshot()
        snapshot = self.engine.submit('read',self.adapter.snapshot).result(1)
        self.assertEqual(snapshot.position_turns,4.5)

    def test_blocked_transport_quit_retains_owner_but_closes_admission(self):
        entered,release=threading.Event(),threading.Event();self.releases.append(release)
        future=self.owner.submit('Fibre call',lambda:(entered.set(),release.wait(2)))
        self.assertTrue(entered.wait(1));self.engine.begin_close();self.coordinator.begin_shutdown()
        started=time.monotonic();self.assertFalse(self.owner.close(.02))
        self.assertLess(time.monotonic()-started,.1);self.assertIsNotNone(self.owner.executing)
        with self.assertRaises(DispatchRejected):self.owner.submit('new motion',lambda:None,motion=True).result(1)
        with self.assertRaises(DispatchRejected):self.engine.submit('new run',lambda:None).result(1)
        release.set();future.result(1)

    def test_queue_wait_counts_toward_capture_budget(self):
        entered,release=threading.Event(),threading.Event();self.releases.append(release)
        blocker=self.owner.submit('slow auxiliary request',lambda:(entered.set(),release.wait(2)))
        self.assertTrue(entered.wait(1))
        future=self.engine.submit('measure',lambda:self.adapter.snapshot(max_capture_duration_s=.015))
        time.sleep(.03);release.set();blocker.result(1)
        with self.assertRaises(FeedbackAcquisitionTimeout) as exc:future.result(1)
        self.assertGreater(exc.exception.snapshot.transport_queue_wait_s,.02)

    def test_idle_evidence_does_not_read_failing_position_or_uptime(self):
        self.device.axis0.pos_vel_mapper = SimpleNamespace(pos_rel=float('nan'),vel=float('nan'))
        result=self.engine.submit('idle',self.adapter.request_idle).result(1)
        self.assertTrue(result.confirmed);self.assertIsNotNone(result.observed_at)
        with self.assertRaises(Exception):self.engine.submit('subsequent diagnostic',self.adapter.snapshot).result(1)
        self.assertTrue(result.confirmed)

    def test_recovery_retires_monitor_before_read_and_never_claims_continuity(self):
        retired=[]
        recovery=RetainedConnectionRecovery()
        result=self.engine.submit('recover',lambda:recovery.run(self.adapter,lambda:retired.append(True),load_tester_config())).result(1)
        self.assertEqual(retired,[True]);self.assertTrue(result.idle.confirmed)
        self.assertFalse(result.continuity_proven)
        self.assertEqual(self.device.axis0.current_state,1)

    def test_recovery_rejects_surviving_monitor_and_error_or_reset_does_not_restore_frame(self):
        def retire():raise RuntimeError('monitor still executing')
        with self.assertRaisesRegex(RuntimeError,'still executing'):
            self.engine.submit('recover',lambda:RetainedConnectionRecovery().run(self.adapter,retire,load_tester_config())).result(1)
        self.assertEqual(self.device.axis0.request_count,0)
        self.device.system_stats.uptime=0
        result=self.engine.submit('reset',lambda:RetainedConnectionRecovery().run(self.adapter,lambda:None,load_tester_config())).result(1)
        self.assertFalse(result.continuity_proven)
        self.device.axis0.active_errors=4
        with self.assertRaisesRegex(RuntimeError,'error-free'):
            self.engine.submit('error',lambda:RetainedConnectionRecovery().run(self.adapter,lambda:None,load_tester_config())).result(1)

    def test_failed_initialization_clears_engine_admission(self):
        def fail():raise OSError('before metadata creation')
        with self.assertRaises(OSError):self.engine.submit('initialize',fail).result(1)
        deadline=time.monotonic()+1
        while self.engine.busy is not None and time.monotonic()<deadline:time.sleep(.001)
        self.assertIsNone(self.engine.busy)
        self.assertEqual(self.engine.submit('recover',lambda:'ready').result(1),'ready')

    def test_motion_admission_never_waits_on_an_old_transport_dispatch(self):
        token=self.coordinator.acquire('old run')
        entered,release=threading.Event(),threading.Event();self.releases.append(release)
        worker=threading.Thread(target=lambda:self.coordinator.submit_command(
            token,lambda:self.owner.submit('blocked command',lambda:(entered.set(),release.wait(1)),motion=True).result(2)))
        worker.start()
        try:
            self.assertTrue(entered.wait(1))
            started=time.monotonic()
            with self.assertRaises(MotionConflictError):self.coordinator.acquire('manual')
            with self.assertRaises(MotionConflictError):self.coordinator.reserve_observation('capture')
            self.assertLess(time.monotonic()-started,.05)
        finally:release.set();worker.join(1)
        self.coordinator.release(token)

    def test_communication_lock_contention_has_bounded_rejection(self):
        self.adapter.io_lock.acquire()
        try:
            future = self.engine.submit('contended read',lambda:self.adapter.snapshot(max_capture_duration_s=.02))
            with self.assertRaisesRegex(Exception,'lock unavailable'):future.result(1)
            self.assertGreaterEqual(self.adapter.last_lock_wait_s,.015)
        finally:self.adapter.io_lock.release()

    def test_recovery_keeps_idle_observation_when_followup_measurement_fails(self):
        self.device.axis0.pos_vel_mapper.pos_rel = float('nan')
        observed=[]
        with self.assertRaises(Exception):
            self.engine.submit('recover',lambda:RetainedConnectionRecovery().run(
                self.adapter,lambda:None,load_tester_config(),on_idle=observed.append)).result(1)
        self.assertEqual(len(observed),1)
        self.assertTrue(observed[0].confirmed)
        self.assertIsNotNone(observed[0].observed_monotonic_s)

    def test_cancel_lifecycle_work_prevents_new_motion_on_shutdown(self):
        self.engine.cancel_current()
        self.assertTrue(self.engine.cancel_event.is_set())
        self.coordinator.begin_shutdown()
        with self.assertRaises(DispatchRejected):
            self.owner.submit('arm after Quit',lambda:self.fail('armed'),motion=True).result(1)

    def test_sequential_future_followups_do_not_erase_new_operation_admission(self):
        for iteration in range(20):
            self.assertEqual(self.engine.submit('first',lambda:iteration).result(1),iteration)
            gate=threading.Event();release=threading.Event();self.releases.append(release)
            second=self.engine.submit('second',lambda:(gate.set(),release.wait(1)))
            self.assertTrue(gate.wait(1));self.assertEqual(self.engine.busy,'second')
            release.set();second.result(1)


class RecordingServices(unittest.TestCase):
    def test_backlog_failure_explicit_and_buffer_retained_ordered(self):
        with tempfile.TemporaryDirectory() as folder:
            recorder=CSVRecorder(Path(folder)/'data.csv',1,lambda exc:None,max_pending_rows=2)
            # Producer can fill a bounded queue before writer starts.
            recorder.append([1]);recorder.append([2])
            with self.assertRaisesRegex(RuntimeError,'backlog'):recorder.append([3])
            self.assertEqual(recorder.unsaved_rows(),[(1,),(2,)])
            recorder.start();self.assertEqual(recorder.finish(1),2)
            with open(recorder.path,newline='') as handle:self.assertEqual(list(csv.reader(handle)),[['1'],['2']])

    def test_disk_delay_does_not_block_acquisition_and_finish_has_deadline(self):
        entered,release=threading.Event(),threading.Event()
        with tempfile.TemporaryDirectory() as folder:
            recorder=CSVRecorder(Path(folder)/'data.csv',1,lambda exc:None)
            recorder.start()
            def sync(fd):entered.set();release.wait(1)
            with patch('east_runtime.os.fsync',side_effect=sync):
                try:
                    recorder.append([0]);self.assertTrue(entered.wait(1))
                    stop=threading.Event();count=[]
                    def consume(value):
                        recorder.append([value]);count.append(value)
                        if len(count)==5:stop.set()
                    AcquisitionLoop(lambda:len(count)+1,consume,stop,.001).run()
                    self.assertEqual(len(count),5)
                    with self.assertRaises(TimeoutError):recorder.finish(.01)
                    self.assertTrue(recorder.thread.is_alive())
                    self.assertEqual(recorder.unsaved_rows(),[(i,) for i in range(6)])
                finally:release.set()
                self.assertEqual(recorder.finish(1),6)

    def test_disk_failure_preserves_uncertain_batch(self):
        failed=[]
        with tempfile.TemporaryDirectory() as folder:
            recorder=CSVRecorder(Path(folder)/'data.csv',1,failed.append)
            with patch('east_runtime.os.fsync',side_effect=OSError('disk failed')):
                recorder.start();recorder.append([1])
                with self.assertRaisesRegex(RuntimeError,'disk failed'):recorder.finish(1)
            self.assertEqual(recorder.unsaved_rows(),[(1,)])
            self.assertEqual(recorder.written_rows,0)
            self.assertEqual(len(failed),1)

    def test_required_checkpoint_deadline_keeps_blocked_writer_owned(self):
        entered,release=threading.Event(),threading.Event()
        def write(*args,**kwargs):entered.set();release.wait(1)
        writer=CheckpointWriter(SimpleNamespace(write_checkpoint=write),lambda exc:None)
        writer.start()
        try:
            with self.assertRaises(TimeoutError):writer.write_and_wait('arming',None,timeout_s=.02)
            self.assertTrue(entered.is_set());self.assertTrue(writer.thread.is_alive())
            self.assertFalse(writer.stop())
        finally:release.set();writer.stop(1)
        self.assertFalse(writer.thread.is_alive())

    def test_dirty_session_marker_survives_new_store(self):
        with tempfile.TemporaryDirectory() as folder:
            store=AtomicStateStore(Path(folder));store.mark_unsafe_exit()
            self.assertTrue(AtomicStateStore(Path(folder)).unsafe_exit_present())
            store.clear_unsafe_exit_after_recovery()
            self.assertFalse(store.unsafe_exit_present())


class FakeWidget:
    def __init__(self,master=None,**kwargs):self.master=master;self.options=kwargs;self.placement=None
    def grid(self,**kwargs):self.placement=kwargs
    def configure(self,**kwargs):self.options.update(kwargs)
    def __getattr__(self,name):return lambda *a,**k:None
class FakeVar:
    def __init__(self,value=False):self.value=value
    def get(self):return self.value
    def set(self,value):self.value=value
class ShutdownPresentation(unittest.TestCase):
    def test_visible_isolation_controls_and_both_acknowledgements_required(self):
        factory=SimpleNamespace(CTkToplevel=FakeWidget,CTkLabel=FakeWidget,CTkCheckBox=FakeWidget,
                                CTkButton=FakeWidget,BooleanVar=FakeVar)
        calls=[];dialog=BlockedShutdownDialog(factory,FakeWidget(),'Fibre blocked',lambda:calls.append(1))
        self.assertIs(dialog.exit.master,dialog.window)
        self.assertEqual(dialog.exit.placement['row'],3)
        self.assertEqual(dialog.exit.options['state'],'disabled')
        dialog._confirm();self.assertEqual(calls,[])
        dialog.estop.set(True);dialog._refresh();dialog._confirm();self.assertEqual(calls,[])
        dialog.isolation.set(True);dialog._refresh();self.assertEqual(dialog.exit.options['state'],'normal')
        dialog._confirm();self.assertEqual(calls,[1])


class SoakBoundary(unittest.TestCase):
    def test_repeated_start_stop_recover_cleans_workers_and_bounded_queues(self):
        from analysis.simulate_east_soak import run
        result=run(20)
        self.assertEqual(result['iterations'],20)
        self.assertEqual(result['rows_persisted'],60)
        self.assertEqual(result['remaining_workers'],[])
        self.assertEqual(result['errors'],[])
