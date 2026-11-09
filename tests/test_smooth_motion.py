import math
import unittest

from east_smooth_motion import FilteredPositionSweep, Sweep
from east_stream_screen import StreamScreenModel


class SmoothMotionTests(unittest.TestCase):
    def setUp(self):
        self.model = StreamScreenModel.anchored("candidate", .2, 0., full_range=True)

    def test_reference_reaches_endpoints_and_respects_speed_and_acceleration(self):
        for start, stop in ((0., 5.), (15., -15.)):
            for speed in (.5, 5., 40., 50.):
                sweep = Sweep.plan(start, stop, speed, 100.)
                self.assertEqual(sweep.at(0.), (start, 0.))
                self.assertEqual(sweep.at(sweep.duration), (stop, 0.))
                dt = sweep.duration / 3000
                last = 0.
                for i in range(1, 3001):
                    angle, velocity = sweep.at(dt * i)
                    self.assertLessEqual(abs(angle), max(abs(start), abs(stop)) + 1e-12)
                    self.assertLessEqual(abs(velocity), speed)
                    self.assertLessEqual(abs(velocity - last) / dt, 100. + 1e-9)
                    last = velocity

    def test_short_leg_has_a_finite_peak_at_the_midpoint(self):
        sweep = Sweep.plan(0., 5., 50., 100.)
        self.assertAlmostEqual(sweep.cruise, 0.)
        angle, speed = sweep.at(sweep.duration / 2)
        self.assertAlmostEqual(angle, 2.5)
        self.assertAlmostEqual(speed, math.sqrt(5. * 100. / 1.5))

    def test_direction_change_starts_at_previous_branch_without_command_jump(self):
        start = self.model.target(15., 1)
        sweep = Sweep.plan(15., -15., .5, 100.)
        planner = FilteredPositionSweep(sweep, self.model, start)
        self.assertEqual(planner.command(0.)["target_motor_turns"], start)
        row = planner.command(.05)
        self.assertLess(abs(row["target_motor_turns"] - start), .02)
        self.assertLess(row["nominal_motor_speed_turns_s"], 0.)

    def test_feedback_is_timestamp_aligned_and_drops_stale_samples(self):
        sweep = Sweep.plan(0., 5., 5., 100.)
        planner = FilteredPositionSweep(sweep, self.model, .2, feedback=True)
        planner.command(0., measured=0., sample_time=0.)
        delayed_angle = sweep.at(.09)[0]
        row = planner.command(.15, measured=delayed_angle, sample_time=.09)
        self.assertEqual(row["timestamp_aligned_error_deg"], 0.)
        self.assertEqual(row["rotary_trim_deg"], 0.)
        tied = planner.command(.16, measured=delayed_angle + .02, sample_time=.09)
        self.assertFalse(tied["rotary_filter_updated"])
        self.assertEqual(tied["filtered_error_deg"], row["filtered_error_deg"])
        with self.assertRaisesRegex(ValueError, "out of order"):
            planner.command(.17, measured=delayed_angle, sample_time=.08)
        with self.assertRaisesRegex(ValueError, "stale"):
            planner.command(.25, measured=delayed_angle, sample_time=.09)
        next_row = planner.command(.26, measured=sweep.at(.25)[0], sample_time=.25)
        self.assertTrue(next_row["rotary_filter_updated"])

    def test_trim_stays_bounded_and_returns_to_same_endpoint_at_full_range(self):
        for start, target in ((-15., 15.), (15., -15.)):
            for error in (-1., 1.):
                sweep = Sweep.plan(start, target, 50., 100.)
                motor = self.model.target(start, -sweep.direction)
                planner = FilteredPositionSweep(sweep, self.model, motor, feedback=True)
                previous = 0.
                for i in range(math.ceil(sweep.duration / .04) + 1):
                    t = i * .04
                    angle = sweep.at(t)[0]
                    row = planner.command(t, measured=angle - error, sample_time=t)
                    trim = row["rotary_trim_deg"]
                    self.assertLessEqual(abs(trim), .1)
                    self.assertLessEqual(abs(trim - previous), .008 + 1e-12)
                    self.assertLessEqual(abs(angle + trim), 15. + 1e-12)
                    previous = trim
                self.assertEqual(trim, 0.)
                self.assertEqual(row["target_motor_turns"], self.model.target(target, sweep.direction))


if __name__ == "__main__":
    unittest.main()
