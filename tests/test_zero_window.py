"""Stationary window policy and explicit affine coordinate transforms."""
import unittest
from east_zero_window import collect_stationary_window
from east_motion_coordinates import LinearPoseAnchor
class WindowTests(unittest.TestCase):
    def run_window(self,moving=lambda t:t<.2,sequence=lambda t:int(t*100),position=lambda t:0):
        clock=[0.]
        def read():return dict(stationary=not moving(clock[0]),motor_turns=position(clock[0]),angle_deg=.02,sequence=sequence(clock[0]))
        return collect_stationary_window(read,.5,1.,position_tolerance_turns=.02,clock=lambda:clock[0],sleep=lambda dt:clock.__setitem__(0,clock[0]+dt))
    def test_transient_velocity_waits_then_requires_full_new_window(self):
        r=self.run_window();self.assertGreater(r['stationary_resets'],0)
        self.assertGreaterEqual(r['completed_at_monotonic_s'],.7)
        self.assertGreaterEqual(r['duration_s'],.5);self.assertGreaterEqual(r['samples'],5)
    def test_persistent_motion_and_frozen_samples_are_rejected(self):
        with self.assertRaisesRegex(RuntimeError,'stationary'):self.run_window(moving=lambda t:True)
        with self.assertRaisesRegex(RuntimeError,'fresh'):self.run_window(sequence=lambda t:1)
    def test_position_drift_cannot_pass_using_zero_velocity_alone(self):
        with self.assertRaisesRegex(RuntimeError,'stationary'):
            self.run_window(moving=lambda t:False,position=lambda t:t)
    def test_original_accounts_measured_offset_and_keeps_fixed_slope(self):
        a=LinearPoseAnchor(12.,.075,2.055)
        self.assertAlmostEqual(a.target(.075),12.)
        self.assertAlmostEqual(a.target(15)-a.target(-15),30/2.055)
        self.assertAlmostEqual(a.target(0),12-.075/2.055)
        with self.assertRaises(ValueError):LinearPoseAnchor(0,.2,2.055)
