"""Persistent neutral-reference and motion-safety primitives for EAST.

This module deliberately has no GUI, ODrive, or Phidget imports so its safety
logic can be exercised on development machines without tester hardware.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import threading
import time
import uuid
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, Callable, Dict, Iterable, List, Optional, Tuple


STATE_SCHEMA_VERSION = 1


class ReferenceError(RuntimeError):
    """Base class for reference and runtime-state failures."""


class ReferenceRequiredError(ReferenceError):
    """Raised when a motion requires a verified neutral reference."""


class FeedbackError(ReferenceError):
    """Raised when controller feedback is missing, stale, or non-finite."""


class MotionConflictError(ReferenceError):
    """Raised when another owner already controls motion."""


class StateCorruptError(ReferenceError):
    """Raised after an invalid state file has been preserved for inspection."""


class ReferenceConfidence(str, Enum):
    UNKNOWN = "unknown"
    RECOVERY_REQUIRED = "recovery_required"
    VERIFIED = "verified"
    FAULT = "fault"


class MotionState(str, Enum):
    IDLE = "idle"
    ARMING = "arming"
    MOVING = "moving"
    STOPPING = "stopping"
    FAULT = "fault"


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


def runtime_state_directory(app_name: str = "EAST") -> Path:
    """Return a per-user state directory outside source and frozen bundles."""
    override = os.environ.get("EAST_STATE_DIR")
    if override:
        return Path(override).expanduser().resolve()
    if os.name == "nt":
        root = Path(os.environ.get("LOCALAPPDATA", Path.home() / "AppData" / "Local"))
        return root / app_name
    if sys_platform() == "darwin":
        return Path.home() / "Library" / "Application Support" / app_name
    root = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local" / "state"))
    return root / app_name.lower()


def sys_platform() -> str:
    # Kept behind a function to make state-path tests deterministic.
    import sys

    return sys.platform


def _finite(value: Any, label: str) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise FeedbackError(f"{label} is not numeric") from exc
    if not math.isfinite(number):
        raise FeedbackError(f"{label} is not finite")
    return number


@dataclass(frozen=True)
class FeedbackSnapshot:
    captured_monotonic_s: float
    captured_at: str
    position_turns: float
    velocity_turns_s: float
    active_errors: int
    current_state: int
    disarm_reason: int = 0
    system_uptime: Optional[float] = None
    raw_phase: Optional[float] = None
    capture_started_monotonic_s: Optional[float] = None
    capture_duration_s: Optional[float] = None

    def validate(
        self,
        now_monotonic_s: Optional[float] = None,
        max_age_s: Optional[float] = None,
        max_capture_duration_s: Optional[float] = None,
    ) -> None:
        _finite(self.captured_monotonic_s, "feedback capture time")
        _finite(self.position_turns, "ODrive position")
        _finite(self.velocity_turns_s, "ODrive velocity")
        if self.system_uptime is not None:
            _finite(self.system_uptime, "ODrive uptime")
        if self.raw_phase is not None:
            _finite(self.raw_phase, "encoder raw phase")
        if self.capture_started_monotonic_s is not None:
            _finite(self.capture_started_monotonic_s, "feedback capture start time")
        if self.capture_duration_s is not None:
            duration = _finite(self.capture_duration_s, "feedback capture duration")
            if duration < 0:
                raise FeedbackError("feedback capture duration is negative")
            if max_capture_duration_s is not None and duration > max_capture_duration_s:
                raise FeedbackError(
                    f"ODrive feedback acquisition took too long ({duration:.3f} s)"
                )
        if max_age_s is not None:
            now = time.monotonic() if now_monotonic_s is None else now_monotonic_s
            age = _finite(now, "current monotonic time") - self.captured_monotonic_s
            if age < -0.001 or age > max_age_s:
                raise FeedbackError(f"ODrive feedback is stale ({age:.3f} s old)")

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "FeedbackSnapshot":
        return cls(**payload)


@dataclass(frozen=True)
class HardwareFingerprint:
    odrive_serial: str
    axis: int
    firmware: str
    configuration_digest: str
    details: Dict[str, Any]

    @classmethod
    def build(
        cls,
        odrive_serial: Any,
        axis: int,
        firmware: str,
        details: Dict[str, Any],
    ) -> "HardwareFingerprint":
        encoded = json.dumps(details, sort_keys=True, separators=(",", ":"), default=str)
        digest = hashlib.sha256(encoded.encode("utf-8")).hexdigest()
        return cls(str(odrive_serial), int(axis), str(firmware), digest, dict(details))

    def stable_identity(self) -> Tuple[str, int, str, str]:
        return self.odrive_serial, self.axis, self.firmware, self.configuration_digest

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "HardwareFingerprint":
        fingerprint = cls(**payload)
        if not fingerprint.odrive_serial or not fingerprint.firmware:
            raise ValueError("hardware fingerprint identity is incomplete")
        if int(fingerprint.axis) < 0 or not fingerprint.configuration_digest:
            raise ValueError("hardware fingerprint axis/configuration is invalid")
        if not isinstance(fingerprint.details, dict):
            raise ValueError("hardware fingerprint details must be an object")
        rebuilt = cls.build(
            fingerprint.odrive_serial,
            fingerprint.axis,
            fingerprint.firmware,
            fingerprint.details,
        )
        if rebuilt.configuration_digest != fingerprint.configuration_digest:
            raise ValueError("hardware fingerprint digest does not match its details")
        return fingerprint


def hardware_fingerprints_match(
    saved: Dict[str, Any], current: HardwareFingerprint
) -> Tuple[bool, str]:
    """Compare every identity field that binds a reference to one controller frame."""
    try:
        prior = HardwareFingerprint.from_dict(saved)
    except (KeyError, TypeError, ValueError) as exc:
        return False, f"saved hardware fingerprint is invalid: {exc}"
    if prior.odrive_serial != current.odrive_serial:
        return False, "ODrive serial number changed"
    if prior.axis != current.axis:
        return False, "ODrive axis changed"
    if prior.firmware != current.firmware:
        return False, "ODrive firmware changed"
    if prior.configuration_digest != current.configuration_digest:
        return False, "frame-relevant ODrive configuration changed"
    return True, "hardware identity and frame configuration match"


@dataclass(frozen=True)
class ReferenceRecord:
    reference_id: str
    generation: int
    established_at: str
    established_by: str
    fixture_id: str
    physical_definition: str
    afo_degrees_per_odrive_turn: float
    hardware_fingerprint: Dict[str, Any]
    phase_at_neutral: Optional[float] = None
    phase_units: Optional[str] = None
    phase_uncertainty_turns: Optional[float] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema_version": STATE_SCHEMA_VERSION,
            **asdict(self),
        }

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "ReferenceRecord":
        values = dict(payload)
        if int(values.pop("schema_version", -1)) != STATE_SCHEMA_VERSION:
            raise ValueError("unsupported reference-state schema")
        record = cls(**values)
        if not record.reference_id or record.generation < 1:
            raise ValueError("invalid reference identity or generation")
        if _finite(record.afo_degrees_per_odrive_turn, "reference conversion factor") <= 0:
            raise ValueError("reference conversion factor must be positive")
        HardwareFingerprint.from_dict(record.hardware_fingerprint)
        if record.phase_at_neutral is not None:
            _finite(record.phase_at_neutral, "reference phase")
        if record.phase_uncertainty_turns is not None:
            if _finite(record.phase_uncertainty_turns, "reference phase uncertainty") < 0:
                raise ValueError("reference phase uncertainty must not be negative")
        return record


@dataclass(frozen=True)
class SessionMapping:
    reference_id: str
    reference_generation: int
    neutral_position_turns: float
    verified_at: str
    verification_method: str
    hardware_fingerprint: Dict[str, Any]

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, payload: Dict[str, Any]) -> "SessionMapping":
        mapping = cls(**payload)
        if not mapping.reference_id or int(mapping.reference_generation) < 1:
            raise ValueError("invalid session-mapping reference identity")
        _finite(mapping.neutral_position_turns, "session neutral position")
        HardwareFingerprint.from_dict(mapping.hardware_fingerprint)
        return mapping


@dataclass(frozen=True)
class MotionToken:
    owner: str
    generation: int


class ProcessLock:
    """An OS-held, non-blocking single-process lock for hardware ownership."""

    def __init__(self, path: Path):
        self.path = Path(path)
        self._handle = None
        self._held = False

    @property
    def held(self) -> bool:
        return self._held

    def acquire(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        handle = self.path.open("a+b")
        try:
            if os.name == "nt":
                import msvcrt

                handle.seek(0)
                if handle.read(1) == b"":
                    handle.seek(0)
                    handle.write(b"0")
                    handle.flush()
                handle.seek(0)
                msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
            else:
                import fcntl

                fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (OSError, BlockingIOError) as exc:
            handle.close()
            raise MotionConflictError(
                f"Another EAST process owns the tester lock: {self.path}"
            ) from exc
        self._handle = handle
        self._held = True

    def release(self) -> None:
        if not self._held or self._handle is None:
            return
        try:
            if os.name == "nt":
                import msvcrt

                self._handle.seek(0)
                msvcrt.locking(self._handle.fileno(), msvcrt.LK_UNLCK, 1)
            else:
                import fcntl

                fcntl.flock(self._handle.fileno(), fcntl.LOCK_UN)
        finally:
            self._handle.close()
            self._handle = None
            self._held = False

    def __enter__(self) -> "ProcessLock":
        self.acquire()
        return self

    def __exit__(self, *_args: Any) -> None:
        self.release()


class AtomicStateStore:
    """Atomic, generation-ordered persistence for safety-critical runtime state."""

    def __init__(self, directory: Path):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.reference_path = self.directory / "neutral_reference.json"
        self.checkpoint_path = self.directory / "runtime_checkpoint.json"
        self.event_path = self.directory / "reference_events.jsonl"
        self.lock_path = self.directory / "hardware.lock"
        self._lock = threading.RLock()
        self._last_generations: Dict[Path, int] = {}

    def _preserve_corrupt(self, path: Path) -> Path:
        suffix = datetime.now().strftime("%Y%m%dT%H%M%S%f")
        preserved = path.with_name(f"{path.name}.corrupt-{suffix}")
        os.replace(path, preserved)
        return preserved

    def _read(self, path: Path) -> Optional[Dict[str, Any]]:
        with self._lock:
            if not path.exists():
                return None
            try:
                with path.open("r", encoding="utf-8") as handle:
                    payload = json.load(handle)
                if not isinstance(payload, dict):
                    raise ValueError("state payload is not an object")
                return payload
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                try:
                    preserved = self._preserve_corrupt(path)
                except OSError:
                    preserved = path
                raise StateCorruptError(
                    f"Invalid runtime state preserved at {preserved}: {exc}"
                ) from exc

    def _atomic_write(self, path: Path, payload: Dict[str, Any]) -> None:
        generation = int(payload.get("generation", 0))
        with self._lock:
            current = self._last_generations.get(path)
            if current is None and path.exists():
                existing = self._read(path)
                current = int(existing.get("generation", 0)) if existing else -1
            if current is None:
                current = -1
            if generation <= current:
                raise ReferenceError(
                    f"Refusing out-of-order state write ({generation} <= {current})"
                )
            temporary = path.with_name(f".{path.name}.{uuid.uuid4().hex}.tmp")
            try:
                with temporary.open("x", encoding="utf-8") as handle:
                    json.dump(payload, handle, indent=2, sort_keys=True, allow_nan=False)
                    handle.write("\n")
                    handle.flush()
                    os.fsync(handle.fileno())
                os.replace(temporary, path)
                try:
                    directory_fd = os.open(str(path.parent), os.O_RDONLY)
                    try:
                        os.fsync(directory_fd)
                    finally:
                        os.close(directory_fd)
                except OSError:
                    # Windows does not support fsync on directory handles.
                    pass
                self._last_generations[path] = generation
            finally:
                try:
                    temporary.unlink()
                except FileNotFoundError:
                    pass

    def load_reference(self) -> Optional[ReferenceRecord]:
        payload = self._read(self.reference_path)
        if payload is None:
            return None
        try:
            record = ReferenceRecord.from_dict(payload)
        except (KeyError, TypeError, ValueError, FeedbackError) as exc:
            try:
                preserved = self._preserve_corrupt(self.reference_path)
            except OSError:
                preserved = self.reference_path
            raise StateCorruptError(
                f"Invalid neutral reference preserved at {preserved}: {exc}"
            ) from exc
        self._last_generations[self.reference_path] = record.generation
        return record

    def save_reference(self, record: ReferenceRecord) -> None:
        self._atomic_write(self.reference_path, record.to_dict())

    def load_checkpoint(self) -> Optional[Dict[str, Any]]:
        payload = self._read(self.checkpoint_path)
        if payload is None:
            return None
        try:
            if int(payload.get("schema_version", -1)) != STATE_SCHEMA_VERSION:
                raise ValueError("unsupported runtime-checkpoint schema")
            generation = int(payload["generation"])
            if generation < 1:
                raise ValueError("checkpoint generation must be positive")
            datetime.fromisoformat(str(payload["written_at"]))
            MotionState(str(payload["motion_state"]))
            if not isinstance(payload["clean_shutdown"], bool):
                raise ValueError("clean_shutdown must be boolean")
            ReferenceConfidence(str(payload["reference_confidence"]))
            reference_id = payload.get("reference_id")
            if reference_id is not None and not isinstance(reference_id, str):
                raise ValueError("reference_id must be text or null")
            mapping = payload.get("session_mapping")
            if mapping is not None:
                if not isinstance(mapping, dict):
                    raise ValueError("session_mapping must be an object or null")
                SessionMapping.from_dict(mapping)
            feedback = payload.get("feedback")
            if feedback is not None:
                if not isinstance(feedback, dict):
                    raise ValueError("feedback must be an object or null")
                FeedbackSnapshot.from_dict(feedback).validate()
        except (KeyError, TypeError, ValueError, FeedbackError) as exc:
            try:
                preserved = self._preserve_corrupt(self.checkpoint_path)
            except OSError:
                preserved = self.checkpoint_path
            raise StateCorruptError(
                f"Invalid runtime checkpoint preserved at {preserved}: {exc}"
            ) from exc
        self._last_generations[self.checkpoint_path] = generation
        return payload

    def save_checkpoint(self, payload: Dict[str, Any]) -> None:
        body = {"schema_version": STATE_SCHEMA_VERSION, **payload}
        self._atomic_write(self.checkpoint_path, body)

    def append_event(self, event: str, details: Optional[Dict[str, Any]] = None) -> None:
        body = {
            "at": utc_now_iso(),
            "event": event,
            "details": details or {},
        }
        line = json.dumps(body, sort_keys=True, allow_nan=False) + "\n"
        with self._lock:
            with self.event_path.open("a", encoding="utf-8") as handle:
                handle.write(line)
                handle.flush()
                os.fsync(handle.fileno())


class MotionCoordinator:
    """Serializes motion ownership, command submission, and stop ordering."""

    def __init__(self):
        self._lock = threading.RLock()
        self._command_gate = threading.Lock()
        self._generation = 0
        self._owner_token: Optional[MotionToken] = None
        self._stopping = False
        self._motor_idle_confirmed = True

    @property
    def owner(self) -> Optional[str]:
        with self._lock:
            return self._owner_token.owner if self._owner_token else None

    @property
    def motor_idle_confirmed(self) -> bool:
        with self._lock:
            return self._motor_idle_confirmed

    @property
    def active_token(self) -> Optional[MotionToken]:
        with self._lock:
            return self._owner_token

    def acquire(self, owner: str) -> MotionToken:
        with self._command_gate:
            with self._lock:
                if self._owner_token is not None:
                    raise MotionConflictError(
                        f"Motion is already owned by {self._owner_token.owner}"
                    )
                if self._stopping or not self._motor_idle_confirmed:
                    raise MotionConflictError(
                        "Motion is stopping; wait for confirmed idle and worker exit"
                    )
                self._generation += 1
                token = MotionToken(owner, self._generation)
                self._owner_token = token
                return token

    def request_stop(self) -> int:
        # Taking the same gate as submit_command guarantees that every command
        # already in progress completes before stop is latched, and no command
        # can be submitted after the latch.
        with self._command_gate:
            with self._lock:
                self._stopping = True
                self._motor_idle_confirmed = False
                return self._generation

    def confirm_motor_idle(self) -> None:
        with self._lock:
            self._motor_idle_confirmed = True
            if self._owner_token is None:
                self._stopping = False

    def confirm_idle(self) -> None:
        """Compatibility alias; idle confirmation deliberately keeps ownership."""
        self.confirm_motor_idle()

    def assert_active(self, token: MotionToken) -> None:
        with self._lock:
            if (
                self._stopping
                or self._owner_token != token
            ):
                raise MotionConflictError(f"Stale or cancelled motion owner: {token.owner}")

    def submit_command(self, token: MotionToken, command: Callable[[], Any]) -> Any:
        """Run one target write only if ownership is still valid at submission."""
        with self._command_gate:
            self.assert_active(token)
            return command()

    def release(self, token: MotionToken) -> None:
        with self._command_gate:
            with self._lock:
                if self._owner_token == token:
                    self._owner_token = None
                    if self._motor_idle_confirmed:
                        self._stopping = False


class ReferenceManager:
    """Owns the durable physical reference and the current-session mapping."""

    def __init__(self, config: Dict[str, Any], store: AtomicStateStore):
        self.config = config
        self.store = store
        self._lock = threading.RLock()
        self.record: Optional[ReferenceRecord] = None
        self.mapping: Optional[SessionMapping] = None
        self.confidence = ReferenceConfidence.UNKNOWN
        self.reason = "No neutral reference has been loaded"
        self._generation = 0
        self._checkpoint_generation = 0
        try:
            self.record = self.store.load_reference()
        except StateCorruptError as exc:
            self.confidence = ReferenceConfidence.FAULT
            self.reason = str(exc)
        else:
            if self.record is None:
                self.reason = "No neutral reference has been established"
            else:
                self._generation = self.record.generation
                self.confidence = ReferenceConfidence.RECOVERY_REQUIRED
                self.reason = "Physical neutral must be verified for this controller session"
        try:
            checkpoint = self.store.load_checkpoint()
            if checkpoint:
                self._checkpoint_generation = int(checkpoint.get("generation", 0))
        except StateCorruptError as exc:
            self.mapping = None
            self.confidence = ReferenceConfidence.FAULT
            self.reason = str(exc)

    @property
    def verified(self) -> bool:
        return self.confidence == ReferenceConfidence.VERIFIED and self.mapping is not None

    def require_verified(self) -> SessionMapping:
        if not self.verified or self.mapping is None:
            raise ReferenceRequiredError(self.reason)
        return self.mapping

    def establish_at_physical_neutral(
        self,
        snapshot: FeedbackSnapshot,
        fingerprint: HardwareFingerprint,
        operator: str,
        fixture_id: str,
        acknowledgement: bool,
        method: str = "operator_confirmed_physical_90_deg",
    ) -> SessionMapping:
        if not acknowledgement:
            raise ReferenceError("Physical-neutral acknowledgement is required")
        snapshot.validate()
        if snapshot.active_errors:
            raise ReferenceError(
                f"ODrive has active errors ({snapshot.active_errors}); neutral cannot be set"
            )
        reference_cfg = self.config["reference"]
        max_velocity = float(reference_cfg["stationary_velocity_limit_deg_s"])
        degrees_per_turn = float(self.config["motion"]["afo_degrees_per_odrive_turn"])
        if abs(snapshot.velocity_turns_s * degrees_per_turn) > max_velocity:
            raise ReferenceError("Motor must be stationary before setting neutral")
        operator = str(operator).strip()
        fixture_id = str(fixture_id).strip()
        if not operator:
            raise ReferenceError("Operator ID is required to set neutral")
        if not fixture_id:
            raise ReferenceError("Fixture ID is required to set neutral")

        self._generation += 1
        phase_value = snapshot.raw_phase if reference_cfg["phase_recovery_enabled"] else None
        phase_units = reference_cfg.get("phase_units") if phase_value is not None else None
        record = ReferenceRecord(
            reference_id=str(uuid.uuid4()),
            generation=self._generation,
            established_at=utc_now_iso(),
            established_by=operator,
            fixture_id=fixture_id,
            physical_definition="Mounted AFO/fixture physically aligned at 90 degrees",
            afo_degrees_per_odrive_turn=degrees_per_turn,
            hardware_fingerprint=fingerprint.to_dict(),
            phase_at_neutral=phase_value,
            phase_units=phase_units,
            phase_uncertainty_turns=reference_cfg.get("phase_uncertainty_turns"),
        )
        # Persistence is deliberately completed before this mapping can enable motion.
        self.store.save_reference(record)
        mapping = SessionMapping(
            reference_id=record.reference_id,
            reference_generation=record.generation,
            neutral_position_turns=snapshot.position_turns,
            verified_at=utc_now_iso(),
            verification_method=method,
            hardware_fingerprint=fingerprint.to_dict(),
        )
        self.record = record
        self.mapping = mapping
        self.confidence = ReferenceConfidence.VERIFIED
        self.reason = "Physical 90 degree neutral verified for this controller session"
        try:
            self.store.append_event("neutral_verified", self.metadata_snapshot())
            self.write_checkpoint(MotionState.IDLE, snapshot, clean_shutdown=False)
        except Exception as exc:
            self.mapping = None
            self.confidence = ReferenceConfidence.FAULT
            self.reason = f"Neutral was not enabled because runtime-state persistence failed: {exc}"
            raise
        return mapping

    def verify_measured_displacement(
        self,
        snapshot: FeedbackSnapshot,
        fingerprint: HardwareFingerprint,
        measured_angle_deg: float,
        uncertainty_deg: float,
        operator: str,
        fixture_id: str,
        acknowledgement: bool,
    ) -> SessionMapping:
        cfg = self.config["reference"]
        if not cfg["assisted_measured_angle_enabled"] or not cfg["phase_recovery_enabled"]:
            raise ReferenceError(
                "Measured-angle recovery requires both assisted and confirmed phase recovery settings"
            )
        record = self.record
        if record is None:
            raise ReferenceRequiredError(
                "Measured-angle recovery requires an existing physical-neutral reference"
            )
        snapshot.validate()
        if snapshot.active_errors:
            raise ReferenceError(
                f"ODrive has active errors ({snapshot.active_errors}); recovery is blocked"
            )
        matches, reason = hardware_fingerprints_match(
            record.hardware_fingerprint, fingerprint
        )
        if not matches:
            raise ReferenceError(reason)
        if snapshot.raw_phase is None or record.phase_at_neutral is None:
            raise ReferenceError("Measured-angle recovery requires finite raw phase feedback")
        uncertainty = abs(_finite(uncertainty_deg, "angle uncertainty"))
        if uncertainty > float(cfg["maximum_assisted_uncertainty_deg"]):
            raise ReferenceError("Measured-angle uncertainty exceeds the configured limit")
        angle = _finite(measured_angle_deg, "measured angle")
        maximum = float(self.config["motion"]["maximum_afo_angle_deg"])
        if abs(angle) > maximum:
            raise ReferenceError("Measured angle is outside the permitted AFO range")
        period = _finite(cfg.get("phase_period"), "phase period")
        turns_per_period = _finite(
            cfg.get("controller_turns_per_phase_period"), "turns per phase period"
        )
        sign = int(cfg.get("phase_sign"))
        if period <= 0 or turns_per_period <= 0 or sign not in (-1, 1):
            raise ReferenceError("Confirmed phase recovery scale/sign is invalid")
        if not acknowledgement:
            raise ReferenceError("Measured-angle recovery acknowledgement is required")
        if not str(operator).strip():
            raise ReferenceError("Operator ID is required for measured-angle recovery")
        if str(fixture_id).strip() != record.fixture_id:
            raise ReferenceError("Fixture ID does not match the saved neutral reference")

        degrees_per_turn = float(self.config["motion"]["afo_degrees_per_odrive_turn"])
        if abs(snapshot.velocity_turns_s * degrees_per_turn) > float(
            cfg["stationary_velocity_limit_deg_s"]
        ):
            raise ReferenceError("Motor must be stationary for measured-angle recovery")
        inferred_neutral = snapshot.position_turns - angle / degrees_per_turn
        maximum_distance = maximum / degrees_per_turn
        candidates = resolve_phase_candidates(
            snapshot.raw_phase,
            record.phase_at_neutral,
            period,
            turns_per_period,
            snapshot.position_turns,
            maximum_distance,
            sign,
            float(record.phase_uncertainty_turns or 0.0),
        )
        allowed_error_turns = (
            uncertainty / degrees_per_turn + float(record.phase_uncertainty_turns or 0.0)
        )
        accepted = [
            candidate
            for candidate in candidates
            if abs(candidate - inferred_neutral) <= allowed_error_turns
        ]
        if len(accepted) != 1:
            raise ReferenceError(
                "Measured angle and saved phase do not identify exactly one neutral "
                f"candidate ({len(accepted)} matched)"
            )
        mapping = SessionMapping(
            reference_id=record.reference_id,
            reference_generation=record.generation,
            neutral_position_turns=accepted[0],
            verified_at=utc_now_iso(),
            verification_method="operator_measured_angle_and_saved_phase_recovery",
            hardware_fingerprint=fingerprint.to_dict(),
        )
        self.mapping = mapping
        self.confidence = ReferenceConfidence.VERIFIED
        self.reason = "Neutral mapping recovered from measured angle and saved phase"
        try:
            self.store.append_event(
                "neutral_measured_angle_recovered",
                {
                    **self.metadata_snapshot(),
                    "measured_angle_deg": angle,
                    "uncertainty_deg": uncertainty,
                    "operator": str(operator).strip(),
                },
            )
            self.write_checkpoint(MotionState.IDLE, snapshot, clean_shutdown=False)
        except Exception as exc:
            self.mapping = None
            self.confidence = ReferenceConfidence.FAULT
            self.reason = f"Measured-angle recovery persistence failed: {exc}"
            raise
        return mapping

    def recover_from_phase(
        self,
        snapshot: FeedbackSnapshot,
        fingerprint: HardwareFingerprint,
    ) -> Tuple[bool, str]:
        cfg = self.config["reference"]
        record = self.record
        if not cfg["phase_recovery_enabled"]:
            return False, "phase recovery is disabled"
        if record is None or record.phase_at_neutral is None or snapshot.raw_phase is None:
            return False, "saved and current phase evidence is incomplete"
        try:
            snapshot.validate()
        except FeedbackError as exc:
            return False, f"phase-recovery feedback is invalid: {exc}"
        if snapshot.active_errors:
            return False, f"ODrive has active errors ({snapshot.active_errors})"
        degrees_per_turn = float(self.config["motion"]["afo_degrees_per_odrive_turn"])
        if abs(snapshot.velocity_turns_s * degrees_per_turn) > float(
            cfg["stationary_velocity_limit_deg_s"]
        ):
            return False, "motor is not stationary enough for phase recovery"
        matches, reason = hardware_fingerprints_match(
            record.hardware_fingerprint, fingerprint
        )
        if not matches:
            return False, reason
        configured_conversion = degrees_per_turn
        if not math.isclose(
            record.afo_degrees_per_odrive_turn,
            configured_conversion,
            rel_tol=0.0,
            abs_tol=1e-12,
        ):
            return False, "motion conversion changed since neutral was established"
        period = cfg.get("phase_period")
        turns_per_period = cfg.get("controller_turns_per_phase_period")
        sign = cfg.get("phase_sign")
        if period is None or turns_per_period is None or sign is None:
            return False, "phase units/scale/sign are not confirmed"
        maximum_distance = (
            float(self.config["motion"]["maximum_afo_angle_deg"])
            / float(self.config["motion"]["afo_degrees_per_odrive_turn"])
        )
        candidates = resolve_phase_candidates(
            snapshot.raw_phase,
            record.phase_at_neutral,
            float(period),
            float(turns_per_period),
            snapshot.position_turns,
            maximum_distance,
            int(sign),
            float(record.phase_uncertainty_turns or 0.0),
        )
        if not candidates:
            return False, "phase recovery found no neutral candidate within safety bounds"
        if len(candidates) != 1:
            return False, f"phase recovery is ambiguous ({len(candidates)} candidates)"
        mapping = SessionMapping(
            reference_id=record.reference_id,
            reference_generation=record.generation,
            neutral_position_turns=candidates[0],
            verified_at=utc_now_iso(),
            verification_method="confirmed_encoder_phase_recovery",
            hardware_fingerprint=fingerprint.to_dict(),
        )
        self.mapping = mapping
        self.confidence = ReferenceConfidence.VERIFIED
        self.reason = "Neutral mapping recovered from confirmed encoder phase model"
        try:
            self.store.append_event("neutral_phase_recovered", self.metadata_snapshot())
            self.write_checkpoint(MotionState.IDLE, snapshot, clean_shutdown=False)
        except Exception as exc:
            self.mapping = None
            self.confidence = ReferenceConfidence.FAULT
            self.reason = f"Phase recovery was not enabled because persistence failed: {exc}"
            raise
        return True, self.reason

    def invalidate(self, reason: str, fault: bool = False) -> None:
        self.mapping = None
        self.confidence = ReferenceConfidence.FAULT if fault else ReferenceConfidence.RECOVERY_REQUIRED
        self.reason = str(reason)
        self.store.append_event("reference_invalidated", {"reason": self.reason})

    def angle_from_position(self, position_turns: float) -> float:
        mapping = self.require_verified()
        position = _finite(position_turns, "ODrive position")
        return (
            position - mapping.neutral_position_turns
        ) * float(self.config["motion"]["afo_degrees_per_odrive_turn"])

    def target_for_angle(self, angle_deg: float) -> float:
        mapping = self.require_verified()
        angle = _finite(angle_deg, "AFO target angle")
        maximum = float(self.config["motion"]["maximum_afo_angle_deg"])
        if abs(angle) > maximum:
            raise ReferenceError(f"Target exceeds the +/-{maximum:g} degree safety limit")
        return mapping.neutral_position_turns + angle / float(
            self.config["motion"]["afo_degrees_per_odrive_turn"]
        )

    def write_checkpoint(
        self,
        motion_state: MotionState,
        snapshot: Optional[FeedbackSnapshot],
        clean_shutdown: bool,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        with self._lock:
            self._checkpoint_generation += 1
            body: Dict[str, Any] = {
                "generation": self._checkpoint_generation,
                "written_at": utc_now_iso(),
                "motion_state": motion_state.value,
                "clean_shutdown": bool(clean_shutdown),
                "reference_confidence": self.confidence.value,
                "reference_reason": self.reason,
                "reference_id": self.record.reference_id if self.record else None,
                "session_mapping": self.mapping.to_dict() if self.mapping else None,
                "feedback": snapshot.to_dict() if snapshot else None,
            }
            if extra:
                body["details"] = dict(extra)
            self.store.save_checkpoint(body)

    def metadata_snapshot(self) -> Dict[str, Any]:
        return {
            "confidence": self.confidence.value,
            "reason": self.reason,
            "record": self.record.to_dict() if self.record else None,
            "session_mapping": self.mapping.to_dict() if self.mapping else None,
            "state_directory": str(self.store.directory),
        }


def resolve_phase_candidates(
    current_phase: float,
    neutral_phase: float,
    phase_period: float,
    turns_per_phase_period: float,
    current_position_turns: float,
    maximum_distance_turns: float,
    sign: int,
    uncertainty_turns: float = 0.0,
) -> List[float]:
    """Return all neutral-position candidates supported by wrapped phase data."""
    values = (
        current_phase,
        neutral_phase,
        phase_period,
        turns_per_phase_period,
        current_position_turns,
        maximum_distance_turns,
        uncertainty_turns,
    )
    if not all(math.isfinite(float(value)) for value in values):
        raise ReferenceError("Phase recovery inputs must be finite")
    if phase_period <= 0 or turns_per_phase_period <= 0 or sign not in (-1, 1):
        raise ReferenceError("Phase recovery scale, period, and sign are invalid")
    wrapped_delta = ((neutral_phase - current_phase + phase_period / 2) % phase_period) - phase_period / 2
    base_delta_turns = sign * wrapped_delta / phase_period * turns_per_phase_period
    candidates: List[float] = []
    search = int(math.ceil((maximum_distance_turns + uncertainty_turns) / turns_per_phase_period)) + 1
    for index in range(-search, search + 1):
        candidate = current_position_turns + base_delta_turns + index * turns_per_phase_period
        if abs(candidate - current_position_turns) <= maximum_distance_turns + uncertainty_turns:
            candidates.append(candidate)
    return sorted(set(round(value, 12) for value in candidates))


def evaluate_continuity(
    checkpoint: Dict[str, Any],
    current_snapshot: FeedbackSnapshot,
    fingerprint: HardwareFingerprint,
    config: Dict[str, Any],
    now_wall_s: Optional[float] = None,
    reference_record: Optional[ReferenceRecord] = None,
) -> Tuple[bool, str, Optional[SessionMapping]]:
    """Conservatively decide whether a prior relative-position frame still exists."""
    reference_cfg = config["reference"]
    if not reference_cfg["session_continuity_enabled"]:
        return False, "automatic controller-session continuity is disabled", None
    try:
        current_snapshot.validate()
    except FeedbackError as exc:
        return False, f"current feedback is invalid: {exc}", None
    units = reference_cfg.get("odrive_uptime_units")
    scale = {"seconds": 1.0, "milliseconds": 0.001}.get(units)
    if scale is None:
        return False, "ODrive uptime units are not explicitly configured", None
    if not checkpoint.get("clean_shutdown"):
        return False, "previous application shutdown was not confirmed clean", None
    prior_mapping = checkpoint.get("session_mapping")
    prior_feedback = checkpoint.get("feedback")
    if not prior_mapping or not prior_feedback:
        return False, "checkpoint has no prior session mapping or feedback", None
    try:
        FeedbackSnapshot.from_dict(prior_feedback).validate()
    except (TypeError, ValueError, FeedbackError) as exc:
        return False, f"checkpoint feedback is invalid: {exc}", None
    if reference_record is None:
        return False, "saved neutral-reference record is unavailable", None
    if checkpoint.get("reference_id") != reference_record.reference_id:
        return False, "checkpoint reference identity does not match saved reference", None
    if prior_mapping.get("reference_id") != reference_record.reference_id:
        return False, "session mapping reference identity changed", None
    if int(prior_mapping.get("reference_generation", -1)) != reference_record.generation:
        return False, "session mapping reference generation changed", None
    matches, reason = hardware_fingerprints_match(
        prior_mapping.get("hardware_fingerprint", {}), fingerprint
    )
    if not matches:
        return False, reason, None
    matches, reason = hardware_fingerprints_match(
        reference_record.hardware_fingerprint, fingerprint
    )
    if not matches:
        return False, reason, None
    configured_conversion = _finite(
        config["motion"]["afo_degrees_per_odrive_turn"], "configured conversion factor"
    )
    if not math.isclose(
        reference_record.afo_degrees_per_odrive_turn,
        configured_conversion,
        rel_tol=0.0,
        abs_tol=1e-12,
    ):
        return False, "motion conversion changed since neutral was established", None
    prior_uptime = prior_feedback.get("system_uptime")
    if prior_uptime is None or current_snapshot.system_uptime is None:
        return False, "controller uptime evidence is unavailable", None
    prior_uptime_s = _finite(prior_uptime, "prior ODrive uptime") * scale
    current_uptime_s = _finite(current_snapshot.system_uptime, "current ODrive uptime") * scale
    if current_uptime_s <= prior_uptime_s:
        return False, "controller uptime did not advance; restart/reset is possible", None
    written_at = checkpoint.get("written_at")
    try:
        prior_wall = datetime.fromisoformat(str(written_at)).timestamp()
    except (TypeError, ValueError):
        return False, "checkpoint wall-clock time is invalid", None
    current_wall = time.time() if now_wall_s is None else float(now_wall_s)
    wall_delta = current_wall - prior_wall
    uptime_delta = current_uptime_s - prior_uptime_s
    if wall_delta < 0:
        return False, "system wall clock moved backwards", None
    tolerance = float(reference_cfg["continuity_time_tolerance_s"])
    if abs(wall_delta - uptime_delta) > tolerance:
        return False, "wall time and controller uptime do not prove one continuous session", None
    try:
        mapping = SessionMapping.from_dict(prior_mapping)
    except (TypeError, ValueError) as exc:
        return False, f"saved session mapping is invalid: {exc}", None
    return True, "controller-session continuity verified", mapping


def wait_for_settle(
    snapshot_provider: Callable[[], FeedbackSnapshot],
    target_turns: float,
    tolerance_turns: float,
    velocity_limit_turns_s: float,
    dwell_s: float,
    timeout_s: float,
    stale_after_s: float,
    cancelled: Callable[[], bool],
    clock: Callable[[], float] = time.monotonic,
    sleeper: Callable[[float], None] = time.sleep,
    expected_state: Optional[int] = None,
    expected_disarm_reason: Optional[int] = None,
    progress: Optional[Callable[[], None]] = None,
) -> FeedbackSnapshot:
    """Wait for finite, fresh position and low velocity throughout a dwell."""
    start = clock()
    settled_since: Optional[float] = None
    last: Optional[FeedbackSnapshot] = None
    while clock() - start <= timeout_s:
        if cancelled():
            raise MotionConflictError("Motion was cancelled")
        now = clock()
        last = snapshot_provider()
        last.validate(now, stale_after_s)
        if last.active_errors:
            raise FeedbackError(f"ODrive active errors: {last.active_errors}")
        if expected_state is not None and last.current_state != int(expected_state):
            raise FeedbackError(
                f"ODrive left expected state {int(expected_state)} "
                f"(reported {last.current_state})"
            )
        if (
            expected_disarm_reason is not None
            and last.disarm_reason != int(expected_disarm_reason)
        ):
            raise FeedbackError(
                "ODrive disarm reason changed during powered motion "
                f"({expected_disarm_reason} -> {last.disarm_reason})"
            )
        if progress is not None:
            progress()
        position_error = abs(last.position_turns - _finite(target_turns, "target position"))
        stationary = abs(last.velocity_turns_s) <= abs(velocity_limit_turns_s)
        if position_error <= abs(tolerance_turns) and stationary:
            if settled_since is None:
                settled_since = now
            elif now - settled_since >= dwell_s:
                return last
        else:
            settled_since = None
        sleeper(min(0.01, max(0.001, dwell_s / 10.0)))
    if last is None:
        raise TimeoutError("No feedback received while waiting for target")
    raise TimeoutError("Target did not remain within position/velocity settle limits")
