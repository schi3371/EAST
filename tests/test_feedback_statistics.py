"""Run-local telemetry must not include preceding/following runs on one connection."""
import threading,time,unittest
from unittest.mock import Mock
from east_core import load_tester_config
from east_execution import FeedbackMonitor
from test_idle_feedback_retry import sample

class StatisticsTests(unittest.TestCase):
    def test_run_windows_are_isolated_and_frozen_while_monitor_continues(self):
        stop=threading.Event();windows=[];frozen=[];count=[0]
        monitor=None
        def read(**kwargs):
            i=count[0];count[0]+=1
            if i==1:windows.append(monitor.begin_run_statistics())
            if i==3:
                frozen.append(monitor.finish_run_statistics(windows[0]))
                windows.append(monitor.begin_run_statistics())
            if i==4:frozen.append(monitor.finish_run_statistics(windows[1]))
            if i==5:stop.set()
            return sample(time.monotonic(),duration=.001)
        adapter=Mock();adapter.snapshot.side_effect=read
        monitor=FeedbackMonitor(lambda:adapter,load_tester_config(),stop,sample(time.monotonic()),
            lambda:(None,None),lambda:None,lambda s:None,lambda s:None,lambda:None,
            lambda *args:None,lambda:None,self.fail)
        monitor.run()
        self.assertIsNone(monitor.error)
        self.assertEqual(monitor.statistics()['samples'],6)
        self.assertEqual([v['samples'] for v in frozen],[2,1])
        self.assertEqual([v['scope'] for v in frozen],['run_window','run_window'])
        self.assertEqual(monitor.statistics()['scope'],'connection_lifetime')
        monitor.error='later unrelated fault'
        self.assertEqual(monitor.finish_run_statistics(windows[0]),frozen[0])
        self.assertEqual(monitor.finish_run_statistics(windows[1]),frozen[1])
        self.assertGreater(frozen[1]['started_at_monotonic_s'],frozen[0]['started_at_monotonic_s'])
