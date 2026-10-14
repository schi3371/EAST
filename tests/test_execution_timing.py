import time
import unittest
from types import SimpleNamespace
from east_odrive import ODriveAdapter
from east_reference import FeedbackAcquisitionTimeout

class SlowMapper:
    def __init__(self, delay=0): self.delay = delay
    @property
    def pos_rel(self):
        time.sleep(self.delay)
        return 2.0
    vel = 0.0

class SlowAxis:
    current_state = 1
    disarm_reason = 0
    def __init__(self, critical=0, auxiliary=0):
        self.pos_vel_mapper = SlowMapper(critical)
        self.auxiliary = auxiliary
    @property
    def active_errors(self):
        time.sleep(self.auxiliary)
        return 0

def adapter(critical=0, auxiliary=0):
    device = SimpleNamespace(axis0=SlowAxis(critical, auxiliary), system_stats=SimpleNamespace(uptime=1))
    return ODriveAdapter(device, 0, 1, 8)

class TimingTests(unittest.TestCase):
    def test_early_position_never_receives_later_diagnostic_timestamp(self):
        s = adapter(auxiliary=.04).snapshot()
        self.assertGreater(time.monotonic()-s.captured_monotonic_s, .035)
        self.assertEqual(s.captured_monotonic_s, s.position_finished_monotonic_s)
        self.assertGreater(s.capture_duration_s, .035)
        self.assertGreater(s.property_brackets_monotonic_s['active_errors'][1]-s.property_brackets_monotonic_s['active_errors'][0], .035)

    def test_delayed_critical_read_rejected(self):
        with self.assertRaises(FeedbackAcquisitionTimeout):
            adapter(critical=.04).snapshot(max_capture_duration_s=.02)

    def test_delayed_auxiliary_read_rejected_with_evidence(self):
        with self.assertRaises(FeedbackAcquisitionTimeout) as error:
            adapter(auxiliary=.04).snapshot(max_capture_duration_s=.02)
        self.assertLess(error.exception.snapshot.position_finished_monotonic_s,
                        error.exception.snapshot.property_brackets_monotonic_s['active_errors'][1])
