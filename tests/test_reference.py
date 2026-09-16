import json
import math
import subprocess
import sys
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path

from east_core import load_tester_config
from east_reference import (
    AtomicStateStore,
    FeedbackError,
    FeedbackSnapshot,
    HardwareFingerprint,
    MotionConflictError,
    MotionCoordinator,
    MotionState,
    ProcessLock,
    ReferenceConfidence,
    ReferenceError,
    ReferenceManager,
    StateCorruptError,
    evaluate_continuity,
    resolve_phase_candidates,
    wait_for_settle,
)


ROOT = Path(__file__).resolve().parents[1]


def snapshot(position=0.0, velocity=0.0, captured=1.0, errors=0, uptime=100.0, raw_phase=None):
    return FeedbackSnapshot(
        captured_monotonic_s=captured,
        captured_at="2026-09-17T12:00:00+10:00",
        position_turns=position,
        velocity_turns_s=velocity,
        active_errors=errors,
        current_state=1,
        system_uptime=uptime,
        raw_phase=raw_phase,
    )


def fingerprint(serial="123", digest_value=None):
    built = HardwareFingerprint.build(serial, 0, "0.6.10", {"scale": 1.0})
    if digest_value is None:
        return built
    return HardwareFingerprint(serial, 0, "0.6.10", digest_value, {"scale": 1.0})


class FakeClock:
    def __init__(self):
        self.value = 0.0

    def now(self):
        return self.value

    def sleep(self, duration):
        self.value += duration


class ReferenceTests(unittest.TestCase):
    def setUp(self):
        self.config = load_tester_config(ROOT / "tester_config.json")

    def test_first_launch_requires_recovery(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = ReferenceManager(self.config, AtomicStateStore(Path(directory)))
            self.assertFalse(manager.verified)
            self.assertEqual(manager.confidence, ReferenceConfidence.UNKNOWN)

    def test_nonzero_session_position_can_be_physical_neutral(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = ReferenceManager(self.config, AtomicStateStore(Path(directory)))
            manager.establish_at_physical_neutral(
                snapshot(position=42.25, captured=10.0),
                fingerprint(),
                "SC",
                "FIX-1",
                True,
            )
            self.assertAlmostEqual(manager.target_for_angle(0), 42.25)
            self.assertAlmostEqual(manager.angle_from_position(42.25), 0.0)
            self.assertAlmostEqual(
                manager.angle_from_position(manager.target_for_angle(5.0)), 5.0
            )
            lower = manager.target_for_angle(-15.0)
            upper = manager.target_for_angle(15.0)
            self.assertLess(lower, 42.25)
            self.assertGreater(upper, 42.25)

    def test_reference_survives_but_mapping_does_not_auto_restore(self):
        with tempfile.TemporaryDirectory() as directory:
            store = AtomicStateStore(Path(directory))
            first = ReferenceManager(self.config, store)
            first.establish_at_physical_neutral(
                snapshot(position=3.0), fingerprint(), "SC", "FIX-1", True
            )
            second = ReferenceManager(self.config, AtomicStateStore(Path(directory)))
            self.assertIsNotNone(second.record)
            self.assertFalse(second.verified)
            self.assertEqual(second.confidence, ReferenceConfidence.RECOVERY_REQUIRED)

    def test_persistence_failure_does_not_enable_reference(self):
        class FailingStore(AtomicStateStore):
            def save_reference(self, record):
                raise OSError("disk full")

        with tempfile.TemporaryDirectory() as directory:
            manager = ReferenceManager(self.config, FailingStore(Path(directory)))
            with self.assertRaises(OSError):
                manager.establish_at_physical_neutral(
                    snapshot(position=2.0), fingerprint(), "SC", "FIX-1", True
                )
            self.assertFalse(manager.verified)

    def test_checkpoint_failure_rolls_back_in_memory_verification(self):
        class FailingCheckpointStore(AtomicStateStore):
            def save_checkpoint(self, payload):
                raise OSError("read-only state directory")

        with tempfile.TemporaryDirectory() as directory:
            manager = ReferenceManager(
                self.config, FailingCheckpointStore(Path(directory))
            )
            with self.assertRaises(OSError):
                manager.establish_at_physical_neutral(
                    snapshot(position=2.0), fingerprint(), "SC", "FIX-1", True
                )
            self.assertFalse(manager.verified)
            self.assertEqual(manager.confidence, ReferenceConfidence.FAULT)

    def test_feedback_rejects_nan_inf_and_stale(self):
        for value in (math.nan, math.inf, -math.inf):
            with self.assertRaises(FeedbackError):
                snapshot(position=value).validate()
        with self.assertRaises(FeedbackError):
            snapshot(captured=1.0).validate(now_monotonic_s=2.0, max_age_s=0.1)

    def test_motion_coordinator_cancels_stale_workers(self):
        coordinator = MotionCoordinator()
        token = coordinator.acquire("test")
        with self.assertRaises(MotionConflictError):
            coordinator.acquire("manual")
        coordinator.request_stop()
        with self.assertRaises(MotionConflictError):
            coordinator.assert_active(token)
        coordinator.confirm_idle()
        coordinator.acquire("manual")

    def test_wait_for_settle_requires_low_velocity_and_dwell(self):
        clock = FakeClock()

        def provider():
            velocity = 1.0 if clock.value < 0.1 else 0.01
            return snapshot(position=2.0, velocity=velocity, captured=clock.value)

        result = wait_for_settle(
            provider,
            target_turns=2.0,
            tolerance_turns=0.01,
            velocity_limit_turns_s=0.02,
            dwell_s=0.2,
            timeout_s=1.0,
            stale_after_s=0.1,
            cancelled=lambda: False,
            clock=clock.now,
            sleeper=clock.sleep,
        )
        self.assertLessEqual(abs(result.velocity_turns_s), 0.02)
        self.assertGreaterEqual(clock.value, 0.3)

    def test_error_at_target_is_not_accepted_as_arrival(self):
        clock = FakeClock()
        with self.assertRaises(FeedbackError):
            wait_for_settle(
                lambda: snapshot(position=1.0, velocity=0.0, captured=clock.value, errors=7),
                1.0,
                0.01,
                0.02,
                0.1,
                1.0,
                0.1,
                lambda: False,
                clock.now,
                clock.sleep,
            )

    def test_corrupt_state_is_preserved(self):
        with tempfile.TemporaryDirectory() as directory:
            store = AtomicStateStore(Path(directory))
            store.reference_path.write_text("{not-json", encoding="utf-8")
            with self.assertRaises(StateCorruptError):
                store.load_reference()
            self.assertFalse(store.reference_path.exists())
            self.assertEqual(len(list(Path(directory).glob("neutral_reference.json.corrupt-*"))), 1)

    def test_out_of_order_checkpoint_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            store = AtomicStateStore(Path(directory))
            store.save_checkpoint({"generation": 4})
            with self.assertRaises(ReferenceError):
                store.save_checkpoint({"generation": 3})

    def test_motion_checkpoint_records_unclean_moving_state(self):
        with tempfile.TemporaryDirectory() as directory:
            store = AtomicStateStore(Path(directory))
            manager = ReferenceManager(self.config, store)
            manager.write_checkpoint(MotionState.MOVING, snapshot(), False)
            body = store.load_checkpoint()
            self.assertEqual(body["motion_state"], "moving")
            self.assertFalse(body["clean_shutdown"])

    def test_duplicate_process_lock_is_blocked(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "hardware.lock"
            first = ProcessLock(path)
            second = ProcessLock(path)
            first.acquire()
            try:
                with self.assertRaises(MotionConflictError):
                    second.acquire()
            finally:
                first.release()

    def test_continuity_is_disabled_and_reboot_is_rejected(self):
        mapping = {
            "reference_id": "r1",
            "reference_generation": 1,
            "neutral_position_turns": 5.0,
            "verified_at": "2026-09-17T12:00:00+00:00",
            "verification_method": "physical",
            "hardware_fingerprint": fingerprint().to_dict(),
        }
        checkpoint = {
            "clean_shutdown": True,
            "written_at": datetime.now(timezone.utc).isoformat(),
            "session_mapping": mapping,
            "feedback": snapshot(uptime=100.0).to_dict(),
        }
        valid, reason, _ = evaluate_continuity(
            checkpoint, snapshot(uptime=101.0), fingerprint(), self.config
        )
        self.assertFalse(valid)
        self.assertIn("disabled", reason)

        enabled = json.loads(json.dumps(self.config))
        enabled["reference"]["session_continuity_enabled"] = True
        enabled["reference"]["odrive_uptime_units"] = "seconds"
        valid, reason, _ = evaluate_continuity(
            checkpoint, snapshot(uptime=1.0), fingerprint(), enabled
        )
        self.assertFalse(valid)
        self.assertIn("did not advance", reason)

    def test_configuration_change_rejects_continuity(self):
        enabled = json.loads(json.dumps(self.config))
        enabled["reference"]["session_continuity_enabled"] = True
        enabled["reference"]["odrive_uptime_units"] = "seconds"
        prior = fingerprint()
        mapping = {
            "reference_id": "r1",
            "reference_generation": 1,
            "neutral_position_turns": 0.0,
            "verified_at": "now",
            "verification_method": "physical",
            "hardware_fingerprint": prior.to_dict(),
        }
        checkpoint = {
            "clean_shutdown": True,
            "written_at": datetime.now(timezone.utc).isoformat(),
            "session_mapping": mapping,
            "feedback": snapshot(uptime=10).to_dict(),
        }
        valid, reason, _ = evaluate_continuity(
            checkpoint, snapshot(uptime=11), fingerprint(digest_value="changed"), enabled
        )
        self.assertFalse(valid)
        self.assertIn("changed", reason)

    def test_explicit_continuity_accepts_only_matching_time_evidence(self):
        enabled = json.loads(json.dumps(self.config))
        enabled["reference"]["session_continuity_enabled"] = True
        enabled["reference"]["odrive_uptime_units"] = "seconds"
        prior = fingerprint()
        base_wall = 1_800_000_000.0
        checkpoint = {
            "clean_shutdown": True,
            "written_at": datetime.fromtimestamp(base_wall, timezone.utc).isoformat(),
            "session_mapping": {
                "reference_id": "r1",
                "reference_generation": 1,
                "neutral_position_turns": 12.0,
                "verified_at": "now",
                "verification_method": "physical",
                "hardware_fingerprint": prior.to_dict(),
            },
            "feedback": snapshot(uptime=50.0).to_dict(),
        }
        valid, reason, mapping = evaluate_continuity(
            checkpoint,
            snapshot(uptime=51.0),
            prior,
            enabled,
            now_wall_s=base_wall + 1.0,
        )
        self.assertTrue(valid, reason)
        self.assertEqual(mapping.neutral_position_turns, 12.0)

        valid, reason, _ = evaluate_continuity(
            checkpoint,
            snapshot(uptime=51.0),
            prior,
            enabled,
            now_wall_s=base_wall - 1.0,
        )
        self.assertFalse(valid)
        self.assertIn("backwards", reason)

    def test_phase_candidate_resolution_zero_multiple_and_unique(self):
        multiple = resolve_phase_candidates(0.1, 0.2, 1.0, 1.0, 0.0, 2.0, 1)
        self.assertGreater(len(multiple), 1)
        unique = resolve_phase_candidates(0.1, 0.2, 1.0, 1.0, 0.0, 0.2, 1)
        self.assertEqual(unique, [0.1])
        none = resolve_phase_candidates(0.1, 0.6, 1.0, 1.0, 0.0, 0.1, 1)
        self.assertEqual(none, [])

    def test_phase_recovery_requires_confirmed_support_and_unique_candidate(self):
        with tempfile.TemporaryDirectory() as directory:
            disabled = ReferenceManager(self.config, AtomicStateStore(Path(directory)))
            ok, reason = disabled.recover_from_phase(
                snapshot(raw_phase=0.2), fingerprint()
            )
            self.assertFalse(ok)
            self.assertIn("disabled", reason)

        enabled = json.loads(json.dumps(self.config))
        enabled["reference"].update({
            "phase_recovery_enabled": True,
            "phase_units": "turn_fraction",
            "phase_period": 1.0,
            "controller_turns_per_phase_period": 20.0,
            "phase_sign": 1,
            "phase_uncertainty_turns": 0.01,
        })
        with tempfile.TemporaryDirectory() as directory:
            store = AtomicStateStore(Path(directory))
            first = ReferenceManager(enabled, store)
            first.establish_at_physical_neutral(
                snapshot(position=42.0, raw_phase=0.2),
                fingerprint(),
                "SC",
                "FIX-1",
                True,
            )
            second = ReferenceManager(enabled, AtomicStateStore(Path(directory)))
            ok, reason = second.recover_from_phase(
                snapshot(position=46.0, raw_phase=0.4), fingerprint()
            )
            self.assertTrue(ok, reason)
            self.assertAlmostEqual(second.require_verified().neutral_position_turns, 42.0)

    def test_mock_inspection_has_no_hardware_writes(self):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "Testing Scripts" / "inspect_reference.py"),
                    "--mock",
                    "--state-dir",
                    directory,
                ],
                check=True,
                capture_output=True,
                text=True,
            )
            payload = json.loads(result.stdout)
            self.assertEqual(payload["mode"], "mock")
            self.assertFalse(payload["hardware_writes_performed"])


if __name__ == "__main__":
    unittest.main()
