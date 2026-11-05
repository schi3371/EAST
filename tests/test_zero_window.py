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

class ZeroAcceptanceTests(unittest.TestCase):
    def observe(self, angle_at, stationary_at=lambda tick: True):
        tick = [0]

        def read():
            return dict(stationary=stationary_at(tick[0]), motor_turns=0,
                        angle_deg=angle_at(tick[0]), sequence=tick[0])

        return collect_stationary_window(
            read, .5, 1.5,
            accept_window=lambda result: result['min_deg'] >= -.1 and result['max_deg'] <= .1,
            clock=lambda: tick[0] / 100,
            sleep=lambda delay: tick.__setitem__(0, tick[0] + 1),
        )

    def test_near_boundary_excursion_can_clear_in_a_later_full_window(self):
        result = self.observe(lambda tick: -.107 if tick == 43 else -.084)
        self.assertGreaterEqual(result['completed_at_monotonic_s'], .94)
        self.assertGreaterEqual(result['duration_s'], .5)
        self.assertEqual(result['min_deg'], -.084)
        self.assertEqual(result['first_rejected_window']['min_deg'], -.107)

    def test_sustained_miss_is_returned_as_rejected_evidence(self):
        result = self.observe(lambda tick: -.263)
        self.assertEqual(result['mean_deg'], -.263)
        self.assertEqual(result['completed_at_monotonic_s'], 1.5)

    def test_motion_after_rejected_window_prevents_reusing_stale_evidence(self):
        with self.assertRaisesRegex(RuntimeError, 'stationary'):
            self.observe(lambda tick: -.263, stationary_at=lambda tick: tick < 120)


    def test_idle_settling_grace_and_late_stable_window(self):
        tick = [0]

        def read():
            angle = -.14 if tick[0] < 220 else -.04
            return dict(stationary=True, motor_turns=0, angle_deg=angle, sequence=tick[0])

        result = collect_stationary_window(
            read, .5, 5., settle_s=.5,
            accept_window=lambda window: window['min_deg'] >= -.1 and window['max_deg'] <= .1,
            clock=lambda: tick[0] / 100,
            sleep=lambda delay: tick.__setitem__(0, tick[0] + 1),
        )
        self.assertEqual(result['settle_grace_s'], .5)
        self.assertGreaterEqual(result['observation_elapsed_s'], 2.7)
        self.assertEqual(result['min_deg'], -.04)
        self.assertEqual(result['first_rejected_window']['min_deg'], -.14)

    def test_sensor_fault_during_grace_stops_observation(self):
        def stale_read():
            raise RuntimeError('rotary sample is stale')

        with self.assertRaisesRegex(RuntimeError, 'stale'):
            collect_stationary_window(stale_read, .5, 5., settle_s=.5)
