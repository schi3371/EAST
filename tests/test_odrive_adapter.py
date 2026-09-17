import unittest
from types import SimpleNamespace

from east_odrive import ODriveAdapter, UnsupportedCoordinateModeError


class Axis:
    def __init__(self):
        self.active_errors = 0
        self.current_state = 1
        self.disarm_reason = 0
        self.pos_vel_mapper = SimpleNamespace(
            pos_rel=4.5,
            vel=0.0,
            pos_abs=None,
            config=SimpleNamespace(scale=1.0, offset_valid=False),
        )
        self.commutation_mapper = SimpleNamespace(
            config=SimpleNamespace(scale=1.0, offset_valid=False)
        )
        self.controller = SimpleNamespace(
            input_pos=-999.0,
            config=SimpleNamespace(
                absolute_setpoints=False,
                circular_setpoints=False,
                circular_setpoint_range=1.0,
                input_mode=0,
                control_mode=0,
                vel_limit=1.0,
            ),
        )
        self.trap_traj = SimpleNamespace(
            config=SimpleNamespace(vel_limit=1.0, accel_limit=1.0, decel_limit=1.0)
        )
        self.config = SimpleNamespace(
            load_encoder=0,
            commutation_encoder=0,
            motor=SimpleNamespace(motor_type=0),
            watchdog_timeout=0.0,
            enable_watchdog=False,
        )
        self.watchdog_feeds = 0
        self.request_count = 0

    @property
    def requested_state(self):
        return self.current_state

    @requested_state.setter
    def requested_state(self, value):
        self.request_count += 1
        self.current_state = int(value)

    def watchdog_feed(self):
        self.watchdog_feeds += 1


class Device:
    def __init__(self):
        self.axis0 = Axis()
        self.serial_number = "MOCK"
        self.fw_version_major = 0
        self.fw_version_minor = 6
        self.fw_version_revision = 10
        self.system_stats = SimpleNamespace(uptime=100)
        self.rs485_encoder_group0 = SimpleNamespace(
            raw=0.2,
            status=0,
            active_errors=0,
            config=SimpleNamespace(mode=3),
        )


class ODriveAdapterTests(unittest.TestCase):
    def setUp(self):
        self.device = Device()
        self.adapter = ODriveAdapter(self.device, 0, idle_state=1, closed_loop_state=8)

    def test_read_only_report_performs_no_state_request(self):
        report = self.adapter.read_only_report(include_phase=True)
        self.assertEqual(self.device.axis0.request_count, 0)
        self.assertEqual(report["snapshot"]["raw_phase"], 0.2)
        self.assertEqual(report["fingerprint"]["details"]["rs485_encoder_protocol_mode"], 3)
        self.assertIn("pos_abs", report["capabilities"])
        self.assertIn("watchdog", report)

    def test_closed_loop_loads_current_position_before_enable(self):
        self.adapter.enter_closed_loop_holding_current()
        self.assertEqual(self.device.axis0.controller.input_pos, 4.5)
        self.assertEqual(self.device.axis0.current_state, 8)

    def test_pre_cancelled_worker_never_arms(self):
        with self.assertRaises(RuntimeError):
            self.adapter.enter_closed_loop_holding_current(cancelled=lambda: True)
        self.assertEqual(self.device.axis0.request_count, 0)

    def test_supported_relative_coordinate_mode_is_required(self):
        report = self.adapter.validate_motion_capabilities(1.0)
        self.assertFalse(report["absolute_setpoints"])

        self.device.axis0.controller.config.absolute_setpoints = True
        with self.assertRaises(UnsupportedCoordinateModeError):
            self.adapter.validate_motion_capabilities(1.0)
        self.device.axis0.controller.config.absolute_setpoints = False

        self.device.axis0.controller.config.circular_setpoints = True
        with self.assertRaises(UnsupportedCoordinateModeError):
            self.adapter.validate_motion_capabilities(1.0)
        self.device.axis0.controller.config.circular_setpoints = False

        self.device.axis0.pos_vel_mapper.config.scale = 2.0
        with self.assertRaises(UnsupportedCoordinateModeError):
            self.adapter.validate_motion_capabilities(1.0)

    def test_unknown_coordinate_mode_is_rejected(self):
        del self.device.axis0.controller.config.absolute_setpoints
        with self.assertRaises(UnsupportedCoordinateModeError):
            self.adapter.validate_motion_capabilities(1.0)

    def test_idle_is_confirmed(self):
        self.device.axis0.current_state = 8
        result = self.adapter.request_idle()
        self.assertTrue(result.requested)
        self.assertTrue(result.confirmed)
        self.assertEqual(self.device.axis0.current_state, 1)

    def test_watchdog_configuration_and_feed(self):
        self.adapter.configure_watchdog(True, 1.25)
        self.adapter.feed_watchdog()
        self.assertTrue(self.device.axis0.config.enable_watchdog)
        self.assertEqual(self.device.axis0.config.watchdog_timeout, 1.25)
        self.assertEqual(self.device.axis0.watchdog_feeds, 1)


if __name__ == "__main__":
    unittest.main()
