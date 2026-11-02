"""Persistence backlog regression checks; no hardware or actual slow disk needed."""
import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from east_runtime import CheckpointWriter

class BacklogTests(unittest.TestCase):
    def writer(self):
        self.entered=threading.Event();self.release=threading.Event();self.calls=[]
        def write(state,snapshot,**kwargs):
            if snapshot=='inflight':
                self.entered.set();self.release.wait(2)
            self.calls.append((state,snapshot))
        manager=SimpleNamespace(write_checkpoint=write,store=SimpleNamespace(append_event=lambda n,d:self.calls.append(('event',n))))
        w=CheckpointWriter(manager,Mock());w.start()
        self.addCleanup(lambda:(self.release.set(),w.stop(2)))
        w.submit('moving','inflight',{'motion_owner':'test'})
        self.assertTrue(self.entered.wait(1))
        return w

    def test_many_periodic_snapshots_keep_latest_without_queue_backlog(self):
        w=self.writer()
        for i in range(100):w.submit('moving',i,{'motion_owner':'test'})
        self.assertEqual(w.pending.qsize(),1)
        self.assertEqual(w.coalesced_snapshots,99)
        self.release.set();self.assertTrue(w.stop(2))
        self.assertEqual(self.calls,[('moving','inflight'),('moving',99)])

    def test_required_checkpoint_and_events_are_ordered_barriers(self):
        w=self.writer();w.submit('moving','before',{})
        result=[]
        def required():
            w.write_and_wait('idle','required',timeout_s=1)
            result.append(True)
        t=threading.Thread(target=required);t.start()
        deadline=time.monotonic()+1
        while w.pending.qsize()<2 and time.monotonic()<deadline:time.sleep(.001)
        self.assertEqual(w.pending.qsize(),2)
        w.submit('moving','after',{});w.submit('moving','latest',{})
        w.submit_event('audit',{})
        w.submit('moving','after-event',{})
        self.release.set();t.join(2)
        self.assertEqual(result,[True]);self.assertTrue(w.stop(2))
        self.assertEqual(self.calls,[('moving','inflight'),('moving','before'),('idle','required'),('moving','latest'),('event','audit'),('moving','after-event')])

    def test_state_changes_faults_and_different_owners_never_coalesce(self):
        w=self.writer()
        for state,owner in [('moving','a'),('moving','b'),('idle','b'),('fault','b'),('fault','b')]:
            w.submit(state,owner,{'motion_owner':owner})
        self.assertEqual(w.pending.qsize(),5)
        self.release.set();self.assertTrue(w.stop(2))
        self.assertEqual(len(self.calls),6)

    def test_coalescing_does_not_hide_stalled_disk(self):
        w=self.writer();w.submit('moving','pending',{})
        w.last_progress_monotonic_s=time.monotonic()-3.1
        with self.assertRaisesRegex(RuntimeError,'no progress'):w.submit('moving','latest',{})
        self.assertFalse(w.stop())
        self.assertTrue(w.thread.is_alive())
        self.release.set();self.assertTrue(w.stop(2))

    def test_full_nonreplaceable_audit_queue_still_stops_submission(self):
        w=self.writer()
        for i in range(8):w.submit_event(str(i),{})
        with self.assertRaisesRegex(RuntimeError,'checkpoint persistence backlog'):
            w.submit('moving','latest',{})
        with self.assertRaisesRegex(RuntimeError,'audit event backlog'):
            w.submit_event('overflow',{})
        self.release.set();self.assertTrue(w.stop(2))

    def test_checkpoint_fault_is_identified_as_persistence_not_odrive(self):
        from east_execution import FeedbackMonitor
        from east_core import load_tester_config
        from test_idle_feedback_retry import sample
        fault=Mock();initial=sample(at=time.monotonic())
        monitor=FeedbackMonitor(lambda:SimpleNamespace(snapshot=lambda **kw:sample(at=time.monotonic())),
            load_tester_config(),threading.Event(),initial,lambda:(None,None),lambda:None,
            Mock(),Mock(side_effect=RuntimeError('disk backlog')),Mock(),Mock(),Mock(),fault)
        monitor.run()
        self.assertEqual(fault.call_args.args[0],'Checkpoint persistence failure: disk backlog')

if __name__=='__main__':unittest.main()
