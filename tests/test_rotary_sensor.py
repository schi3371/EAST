"""Offline tests of delivered-event acquisition and normal-test integration."""
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import queue
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from east_core import CSV_COLUMNS, ROTARY_RUN_COLUMNS, load_tester_config
from east_rotary_sensor import RotaryCalibration, RotaryError, RotaryReader, RotarySensorService
from test_strain_sequence import gui_method

ROOT = Path(__file__).resolve().parents[1]


def configured_reader(mode="Voltage ratio"):
    reader = RotaryReader(750256, mode)
    reader.metadata = {"hub_serial":750256,"hub_port":0,"channel":0,
        "input_mode":mode,"interval_ms":8,"connection_id":reader.connection_id}
    reader.accepting = True
    return reader


def deliver(reader, values, start=10.0, interval=0.008):
    with patch("east_rotary_sensor.time.monotonic_ns", side_effect=[round((start+i*interval)*1_000_000_000) for i in range(len(values))]):
        for value in values:
            reader._sample(None, value)


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.config = load_tester_config()["rotary_sensor"]
        self.profile = RotaryCalibration(ROOT/self.config["calibration_profile"], self.config)

    def test_physical_zero_and_independent_positive_point(self):
        angle, valid, _ = self.profile.evaluate(0.39772982090527514)
        self.assertTrue(valid)
        self.assertAlmostEqual(angle, 0)
        self.assertAlmostEqual(self.profile.evaluate(0.42753978)[0], 10.785245, places=5)

    def test_checked_extension_is_distinguished_from_fit_range(self):
        angle, valid, status = self.profile.evaluate(0.36049571333333336)
        self.assertAlmostEqual(angle, -13.471302, places=5)
        self.assertTrue(valid)
        self.assertEqual(status, "user_checked_extension")
        for ratio in (math.nan, math.inf, -0.01, 1.01, 0.0, 1.0):
            self.assertFalse(self.profile.evaluate(ratio)[1])

    def test_wrong_sensor_identity_or_unaccepted_profile_cannot_load(self):
        with self.assertRaisesRegex(RotaryError, "hub_serial"):
            RotaryCalibration(ROOT/self.config["calibration_profile"], {**self.config,"hub_serial":123})
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp)/"draft.json"
            p.write_text(json.dumps({**self.profile.profile,"status":"draft"}))
            with self.assertRaisesRegex(RotaryError,"not accepted"):
                RotaryCalibration(p,self.config)

    def test_sensor_derived_extensions_remain_excluded_from_sensor_fit(self):
        source = ROOT/self.profile.profile["source_data"]
        with source.open() as handle:
            rows = list(csv.DictReader(handle))
        fitted = [r for r in rows if r["Included_in_sensor_calibration_fit"]=="True"]
        self.assertEqual(len(fitted),36)
        self.assertEqual(len(rows),54)
        self.assertTrue(all("Sensor-derived" in r["Angle_source"] for r in rows[36:]))
        means = [(sum(float(r["Sensor_mean"]) for r in fitted[i:i+3])/3,
                  float(fitted[i]["Angle_deg"])) for i in range(0,36,3)]
        xm=sum(x for x,y in means)/12
        ym=sum(y for x,y in means)/12
        slope=sum((x-xm)*(y-ym) for x,y in means)/sum((x-xm)**2 for x,y in means)
        self.assertAlmostEqual(self.profile.slope,slope,places=7)
        self.assertAlmostEqual(self.profile.intercept,ym-slope*xm,places=7)


class DeliveredReaderTests(unittest.TestCase):
    def channel(self):
        channel=Mock()
        channel.getDeviceSKU.return_value="VoltageRatioInput_PORT"
        channel.getHub.return_value.getDeviceSKU.return_value="HUB0007_0"
        channel.getHub.return_value.getDeviceSerialNumber.return_value=750256
        channel.getDeviceSerialNumber.return_value=750256
        channel.getHubPort.return_value=0
        channel.getIsHubPortDevice.return_value=True
        channel.getMinDataInterval.return_value=1
        channel.getMaxDataInterval.return_value=60000
        channel.getDataInterval.return_value=8
        return channel

    def run_reader(self, channel, mode="Voltage ratio", values=(0.4,0.4,0.4)):
        reader=RotaryReader(750256,mode,channel_factory=lambda:channel)
        values=iter(values)
        def advance(_delay):
            try:
                reader._sample(channel,next(values))
                return False
            except StopIteration:
                return True
        reader.stop.wait=Mock(side_effect=advance)
        reader._run()
        return reader

    def test_exact_address_callback_sampling_and_single_channel_close(self):
        for mode in ("Voltage ratio","Voltage"):
            channel=self.channel()
            reader=self.run_reader(channel,mode)
            channel.setDeviceSerialNumber.assert_called_once_with(750256)
            channel.setHubPort.assert_called_once_with(0)
            channel.setChannel.assert_called_once_with(0)
            channel.setIsHubPortDevice.assert_called_once_with(True)
            channel.setDataInterval.assert_called_once_with(8)
            if mode=="Voltage ratio":
                channel.setOnVoltageRatioChangeHandler.assert_called_once()
                channel.setVoltageRatioChangeTrigger.assert_called_once_with(0.0)
            else:
                channel.setOnVoltageChangeHandler.assert_called_once()
                channel.setVoltageChangeTrigger.assert_called_once_with(0.0)
            channel.getVoltageRatio.assert_not_called()
            channel.getVoltage.assert_not_called()
            self.assertEqual(reader.samples,3)
            channel.close.assert_called_once()

    def test_hardware_interval_is_read_back_and_clamped(self):
        channel=self.channel()
        channel.getMinDataInterval.return_value=16
        channel.getDataInterval.return_value=16
        reader=self.run_reader(channel)
        channel.setDataInterval.assert_called_once_with(16)
        self.assertEqual(reader.metadata["requested_interval_ms"],8)
        self.assertEqual(reader.metadata["interval_ms"],16)

    def test_wrong_parent_or_address_prevents_sample_acceptance(self):
        for mismatch in ("sku","parent_serial","serial","port","port_device"):
            channel=self.channel()
            if mismatch=="sku": channel.getHub.return_value.getDeviceSKU.return_value="HUB0000"
            if mismatch=="parent_serial": channel.getHub.return_value.getDeviceSerialNumber.return_value=123
            if mismatch=="serial": channel.getDeviceSerialNumber.return_value=123
            if mismatch=="port": channel.getHubPort.return_value=1
            if mismatch=="port_device": channel.getIsHubPortDevice.return_value=False
            reader=self.run_reader(channel)
            self.assertIsNotNone(reader.error)
            self.assertIsNone(reader.metadata)
            self.assertEqual(reader.samples,0)
            channel.close.assert_called_once()

    def test_attachment_failure_and_cancellation_close_channel(self):
        channel=self.channel()
        channel.openWaitForAttachment.side_effect=RuntimeError("driver attachment failed")
        reader=self.run_reader(channel)
        self.assertIn("driver attachment failed",reader.error)
        channel.close.assert_called_once()
        channel=self.channel()
        reader=RotaryReader(750256,"Voltage ratio",channel_factory=lambda:channel)
        channel.openWaitForAttachment.side_effect=lambda _:reader.stop.set()
        reader._run()
        channel.setDataInterval.assert_not_called()
        self.assertIsNone(reader.metadata)
        channel.close.assert_called_once()

    def test_no_initial_event_times_out_without_polling_cached_value(self):
        channel=self.channel()
        reader=RotaryReader(750256,"Voltage ratio",channel_factory=lambda:channel)
        reader.stop.wait=Mock(return_value=False)
        with patch("east_rotary_sensor.time.monotonic",side_effect=[0,4]):
            reader._run()
        self.assertIn("No delivered",reader.error)
        channel.getVoltageRatio.assert_not_called()

    def test_invalid_sample_is_preserved_and_fault_latched(self):
        reader=configured_reader()
        deliver(reader,[1.01])
        self.assertEqual(reader.latest[2],1.01)
        self.assertTrue(reader.stop.is_set())
        self.assertIn("Invalid",reader.error)
        deliver(reader,[0.4],start=11)
        self.assertEqual(reader.samples,1)

    def test_event_buffers_cannot_silently_drop_samples(self):
        reader=configured_reader()
        reader.events=queue.Queue(maxsize=1)
        deliver(reader,[0.4,0.4])
        self.assertTrue(reader.stop.is_set())
        self.assertIn("overflow",reader.error)

    def test_subscription_close_does_not_disconnect_shared_sensor(self):
        reader=configured_reader()
        subscription=reader.subscribe()
        subscription.unsubscribe()
        deliver(reader,[0.4])
        self.assertFalse(reader.stop.is_set())
        self.assertFalse(any(kind=="sample" for kind,_ in subscription.events.queue))

    def test_equal_host_timestamps_retain_distinct_events_without_fault(self):
        reader=configured_reader()
        with patch("east_rotary_sensor.time.monotonic_ns",side_effect=[10_000_000_000,10_000_000_000,10_008_000_000]):
            for value in (0.4,0.4001,0.4002): reader._sample(None,value)
        samples=[p for kind,p in reader.events.queue if kind=="sample"]
        self.assertEqual([p[3] for p in samples],[1,2,3])
        self.assertEqual(samples[0][5],samples[1][5])
        self.assertTrue(samples[1][6])
        self.assertFalse(reader.stop.is_set())
        self.assertIsNone(reader.error)
        self.assertEqual(reader.snapshot()[1]["timestamp_tie_count"],1)

    def test_nanosecond_timestamps_survive_float_precision_loss(self):
        reader=configured_reader()
        with patch("east_rotary_sensor.time.monotonic_ns",side_effect=[10**18,10**18+1]):
            reader._sample(None,0.4)
            reader._sample(None,0.4)
        self.assertFalse(reader.stop.is_set())
        self.assertEqual(reader.latest[5],10**18+1)
        self.assertFalse(reader.latest[6])
        self.assertGreater(reader.snapshot()[4],0)

    def test_actual_backwards_host_clock_is_still_a_fault(self):
        reader=configured_reader()
        with patch("east_rotary_sensor.time.monotonic_ns",side_effect=[10_000_000_000,9_999_999_999]):
            reader._sample(None,0.4)
            reader._sample(None,0.4)
        self.assertTrue(reader.stop.is_set())
        self.assertIn("clock moved backwards",reader.error)


class SharedServiceTests(unittest.TestCase):
    def setUp(self):
        self.config=load_tester_config()["rotary_sensor"]
        self.service=RotarySensorService(self.config,ROOT)
        self.reader=configured_reader()
        self.service.reader=self.reader
        deliver(self.reader,[0.39774]*16)
        self.now=10.12

    def test_fresh_samples_are_ready_and_stale_samples_block(self):
        self.assertTrue(self.service.measurement(now=self.now)["valid"])
        self.assertAlmostEqual(self.service.measurement(now=self.now)["achieved_rate_hz"],125)
        with self.assertRaisesRegex(RotaryError,"stale"):
            self.service.measurement(now=self.now+0.101,require=True)

    def test_detachment_sampling_mode_and_insufficient_rate_block_tests(self):
        self.reader._failure("detached")
        with self.assertRaisesRegex(RotaryError,"detached"):
            self.service.measurement(now=self.now,require=True)
        self.reader=configured_reader("Voltage")
        self.service.reader=self.reader
        deliver(self.reader,[2.0]*16)
        with self.assertRaisesRegex(RotaryError,"voltage-ratio"):
            self.service.measurement(now=self.now,require=True)
        self.reader=configured_reader()
        self.service.reader=self.reader
        deliver(self.reader,[0.39774]*16,interval=0.016)
        with self.assertRaisesRegex(RotaryError,"rate is too low"):
            self.service.measurement(now=10.24,require=True)

    def test_configured_slow_rate_and_missing_profile_cannot_appear_ready(self):
        self.reader.metadata["interval_ms"]=16
        with self.assertRaisesRegex(RotaryError,"rate is too low"):
            self.service.measurement(now=self.now,require=True)
        missing=RotarySensorService({**self.config,"calibration_profile":"not_present.json"},ROOT)
        with self.assertRaises(RotaryError): missing.measurement(require=True)

    def test_normal_reconnect_reuses_reader_without_zeroing(self):
        self.reader.thread=Mock()
        self.reader.thread.is_alive.return_value=True
        self.assertIs(self.service.connect(),self.reader)
        self.assertEqual(self.reader.samples,16)
        with self.assertRaisesRegex(RotaryError,"changing mode"):
            self.service.connect("Voltage")

    def test_recording_retains_every_delivered_event_and_reports_true_rate(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"rotary.csv"
            with patch("east_rotary_sensor.time.monotonic",return_value=self.now):
                self.service.start_recording(path,self.now)
            # No main GUI/load-cell consumption occurs between these delivered events.
            deliver(self.reader,[0.4]*5,start=10.128)
            statistics=self.service.finish_recording()
            with path.open() as handle:
                rows=list(csv.DictReader(handle))
            self.assertEqual(len(rows),5)
            self.assertEqual(len({r["Sensor Sample Index"] for r in rows}),5)
            self.assertEqual(statistics["delivered_samples"],5)
            self.assertAlmostEqual(statistics["achieved_event_rate_hz"],125)
            self.assertEqual(statistics["invalid_samples"],0)
            self.assertEqual(rows[0]["Rotary Calibration ID"],"P3022_RATIO_20261014_V1")
            self.assertFalse(self.reader.stop.is_set())

    def test_failed_recording_open_does_not_leak_subscription(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"existing.csv"
            path.touch()
            with patch("east_rotary_sensor.time.monotonic",return_value=self.now), self.assertRaises(FileExistsError):
                self.service.start_recording(path,self.now)
            self.assertEqual(self.reader.listeners,[])
            self.assertIsNone(self.service.recording)

    def test_final_sensor_loss_cannot_be_reported_as_successful_recording(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"rotary.csv"
            with patch("east_rotary_sensor.time.monotonic",return_value=self.now):
                self.service.start_recording(path,self.now)
            deliver(self.reader,[0.4]*3,start=10.128)
            self.reader._failure("detached at final endpoint")
            with self.assertRaisesRegex(RotaryError,"detached"):
                self.service.finish_recording()
            self.assertIsNone(self.service.recording)
            self.assertEqual(self.service.last_recording_statistics["delivered_samples"],3)
            with path.open() as handle:
                self.assertEqual(len(list(csv.DictReader(handle))),3)

    def test_bad_recorded_sample_is_retained_and_cannot_pass_validation(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"rotary.csv"
            with patch("east_rotary_sensor.time.monotonic",return_value=self.now):
                self.service.start_recording(path,self.now)
            deliver(self.reader,[0.4,0.0,0.4],start=10.128)
            with self.assertRaisesRegex(RotaryError,"invalid"):
                self.service.finish_recording()
            with path.open() as handle:
                rows=list(csv.DictReader(handle))
            self.assertEqual(len(rows),3)
            self.assertEqual(rows[1]["Rotary Sample Valid"],"False")
            self.assertEqual(rows[1]["Rotary Voltage Ratio (V/V)"],"0.0")
            self.assertEqual(self.service.last_recording_statistics["invalid_samples"],1)

    def test_recording_preserves_timestamp_ties_and_counts_them(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/"rotary.csv"
            with patch("east_rotary_sensor.time.monotonic",return_value=self.now):
                self.service.start_recording(path,self.now)
            with patch("east_rotary_sensor.time.monotonic_ns",side_effect=[10_128_000_000,10_128_000_000,10_136_000_000]):
                for value in (0.4,0.4001,0.4002): self.reader._sample(None,value)
            statistics=self.service.finish_recording()
            with path.open() as handle: rows=list(csv.DictReader(handle))
            self.assertEqual(len(rows),3)
            self.assertEqual(rows[0]["Sensor Host Monotonic Time (ns)"],rows[1]["Sensor Host Monotonic Time (ns)"])
            self.assertNotEqual(rows[0]["Sensor Sample Index"],rows[1]["Sensor Sample Index"])
            self.assertEqual(rows[1]["Sensor Timestamp Tied With Previous Event"],"True")
            self.assertEqual(statistics["tied_timestamp_events"],1)

    def test_startup_burst_with_no_elapsed_time_is_not_a_false_rate_pass(self):
        reader=configured_reader()
        self.service.reader=reader
        with patch("east_rotary_sensor.time.monotonic_ns",return_value=10_000_000_000):
            for _ in range(16): reader._sample(None,0.39774)
        self.assertFalse(self.service.measurement(now=10)["valid"])
        self.assertEqual(reader.snapshot()[4],0)
        self.assertFalse(reader.stop.is_set())


class ConnectRetryTests(unittest.TestCase):
    def app(self):
        from east_reference import FeedbackSnapshot
        app=Mock()
        app.system_config=load_tester_config()
        app.monitor_error=None
        app.feedback_retry_pending=False
        app.monitor_thread.is_alive.return_value=True
        previous=FeedbackSnapshot(time.monotonic()-0.02,"time",7.288,0,0,1,system_uptime=1000)
        current=FeedbackSnapshot(time.monotonic(),"time",7.288,0,0,1,system_uptime=1000.02)
        app.get_feedback.return_value=previous
        app._read_adapter_feedback.return_value=current
        return app,current

    def reuse(self,app):
        return gui_method("_reuse_live_odrive_connection",time=time,AXIS_STATE_IDLE=1)(app)

    def test_healthy_live_connection_reuse_does_not_restore_or_redefine_zero(self):
        app,current=self.app()
        self.assertTrue(self.reuse(app))
        app.reference_manager.establish_at_physical_neutral.assert_not_called()
        app.reference_manager.invalidate.assert_not_called()
        app.safe_idle_motor.assert_not_called()

    def test_reset_fault_motion_and_missing_uptime_do_not_qualify_for_reuse(self):
        from dataclasses import replace
        for fields in ({"system_uptime":0},{"system_uptime":None},{"active_errors":1},
                       {"current_state":8},{"position_turns":0},{"velocity_turns_s":1}):
            with self.subTest(fields=fields):
                app,current=self.app()
                app._read_adapter_feedback.return_value=replace(current,**fields)
                self.assertFalse(self.reuse(app))

    def test_connect_retry_connects_all_devices_without_disconnecting_live_odrive(self):
        app=Mock()
        for name in ("strain_thread","data_collection_thread","neutral_thread","manual_step_thread",
                     "continuous_thread","recovery_thread","stop_worker","disconnect_worker",
                     "cleanup_thread"): setattr(app,name,None)
        app.strain_test_active=False
        app.motion_coordinator.owner=None
        app.startup_block_reason=None
        app.process_lock.held=True
        app._reuse_live_odrive_connection.return_value=True
        gui_method("connect_system")(app)
        app.connect_rotary_sensor.assert_called_once()
        app._ensure_load_cell.assert_called_once()
        app.safe_idle_motor.assert_not_called()
        app._stop_feedback_monitor.assert_not_called()
        app.rotary_calibration.suspend_positioning.assert_not_called()


class NormalGuiIntegrationTests(unittest.TestCase):
    def test_missing_rotary_blocks_start_before_motor_ownership_or_motion(self):
        app=Mock()
        app.odrive_controller=object()
        app.reference_manager.verified=True
        app.reference_manager.angle_from_position.return_value=0
        app.system_config=load_tester_config()
        app.stop_worker=None
        app.run_finalized=True
        app.run_metadata=None
        app.disconnect_worker=None
        app.rotary_sensor.measurement.side_effect=RotaryError("disconnected")
        gui_method("start_strain_test", plot_window_open=False,RotaryError=RotaryError)(app)
        app.motion_coordinator.acquire.assert_not_called()
        app.enter_closed_loop.assert_not_called()
        self.assertIn("Test blocked",app.update_terminal.call_args.args[0])

    def test_joined_rows_identify_reused_samples_without_inventing_samples(self):
        app=Mock()
        app.system_config=load_tester_config()
        app.empty_machine_tare=SimpleNamespace(offset_v_per_v=0)
        app.get_feedback.return_value=SimpleNamespace(position_turns=3,velocity_turns_s=0,
            active_errors=0,captured_monotonic_s=time.monotonic())
        app.reference_manager.angle_from_position.return_value=0
        params=SimpleNamespace(commanded_afo_speed_deg_s=5,commanded_afo_acceleration_deg_s2=100,
            min_angle_deg=10,max_angle_deg=10,cycles=3,file_prefix="test",operator="SC",afo_id="EMPTY",
            fixture_id="fixture",calibration_id="load",test_type="empty_machine_baseline")
        app.run_parameters=params
        app.run_metadata={"protocol":{"preset_display_name":"Custom","preset_version":None,"preset_modified":False}}
        stamp=time.monotonic_ns()
        sample=(stamp/1e9,"sample-time",0.41310826,42,"connection-a",stamp,False)
        app.rotary_sensor.measurement.return_value={"sample":sample,"angle_deg":5.5639,
            "sample_age_s":0.001,"valid":True,"sample_status":"within_fit_range","achieved_rate_hz":125}
        app.rotary_sensor.calibration.profile={"calibration_id":"rotary"}
        app.test_started_monotonic=time.monotonic()
        app.sample_count=1
        app.last_rotary_logged_key=None
        app.commanded_odrive_velocity=1
        app.current_nominal_distance_deg=20
        app.current_expected_constant_speed_span_deg=19
        app.strain_data_buffer=[]
        for name in ("angle_filter","weight_filter","torque_filter"):
            getattr(app,name).get_smoothed_value.return_value=0
        method=gui_method("log_strain_data",datetime=datetime,time=time,plot_window_open=False)
        method(app,0.00001,1)
        method(app,0.00001,1)
        names=CSV_COLUMNS+ROTARY_RUN_COLUMNS
        first,second=[dict(zip(names,row)) for row in app.strain_data_buffer]
        self.assertEqual(len(app.strain_data_buffer[0]),len(names))
        self.assertTrue(first["Rotary New Sample Since Previous Row"])
        self.assertFalse(second["Rotary New Sample Since Previous Row"])
        self.assertEqual(first["Rotary Sample Index"],second["Rotary Sample Index"])
        self.assertEqual(first["Measured Fixture Angle (deg)"],"5.56390000")
        self.assertEqual(first["ODrive-Derived AFO Angle (deg)"],"0.000000")

    def test_rotary_failure_during_motion_uses_existing_idle_cleanup(self):
        app=Mock()
        app.system_config=load_tester_config()
        app.run_motion_token="token"
        app.test_stop_event=threading.Event()
        app.operator_stop_requested=False
        app.acquisition_error=None
        app.completed_cycles=0
        app.run_metadata={"rotary_sensor":{}}
        app.rotary_sensor.measurement.side_effect=RotaryError("stale")
        app.rotary_sensor.finish_recording.return_value={}
        app.data_collection_thread.is_alive.return_value=False
        app.stop_worker=None
        app.failure_metadata_worker=None
        app.motion_coordinator.motor_idle_confirmed=True
        def request_stop(*args, **kwargs):
            app.stop_worker=Mock()
            app.stop_worker.is_alive.return_value=False
            app.stop_result=SimpleNamespace(confirmed=True)
        app._request_coordinated_stop.side_effect=request_stop
        gui_method("strain_test_control")(app)
        app.enter_closed_loop.assert_not_called()
        app._request_coordinated_stop.assert_called_once()
        self.assertEqual(app.finalize_run.call_args.args[0],"error")
        app.motion_coordinator.release.assert_called_once_with("token")


if __name__=="__main__":
    unittest.main()
