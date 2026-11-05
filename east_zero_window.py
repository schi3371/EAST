"""Fresh stationary observations with caller-supplied timing and acceptance limits."""
from collections import deque
import math
import statistics
import time


ZERO_IDLE_SETTLE_S = 0.5
ZERO_IDLE_TIMEOUT_S = 5.0


def collect_stationary_window(read, duration, timeout, *, position_tolerance_turns=None,
                              accept_window=None, settle_s=0.0, clock=time.monotonic, sleep=time.sleep):
    if not math.isfinite(duration) or not math.isfinite(timeout) or duration <= 0 or timeout < duration:
        raise ValueError("Invalid stationary observation duration/deadline")
    if position_tolerance_turns is not None and (
        not math.isfinite(position_tolerance_turns) or position_tolerance_turns <= 0
    ):
        raise ValueError("Invalid stationary position tolerance")

    if not math.isfinite(settle_s) or settle_s < 0 or duration + settle_s > timeout:
        raise ValueError("Invalid settling period for stationary observation")

    started = clock()
    deadline = started + timeout
    recent = deque()
    resets = 0
    first_rejected = last_rejected = None
    while clock() <= deadline:
        value = read()  # Caller enforces cancellation, identity, freshness and controller state.
        now = clock()
        if now > deadline:
            break
        if not value["stationary"]:
            recent.clear()
            last_rejected = None
            resets += 1
            sleep(.01)
            continue

        motor = float(value["motor_turns"])
        if not math.isfinite(motor):
            raise RuntimeError("Non-finite stationary motor position")
        angle = value.get("angle_deg")
        if angle is not None:
            angle = float(angle)
            if not math.isfinite(angle):
                raise RuntimeError("Non-finite stationary rotary angle")

        if now - started < settle_s:
            sleep(.01)
            continue

        recent.append((now, motor, value.get("sequence"), angle))
        if accept_window is not None:
            # Keep the shortest observed window that still spans the full dwell.
            while len(recent) > 1 and now - recent[1][0] >= duration:
                recent.popleft()

        positions = [entry[1] for entry in recent]
        low, high = min(positions), max(positions)
        if position_tolerance_turns is not None and high - low > position_tolerance_turns:
            recent.clear()
            last_rejected = None
            resets += 1
            sleep(.01)
            continue

        elapsed = now - recent[0][0]
        if elapsed >= duration:
            angles = {entry[2]: entry[3] for entry in recent if entry[3] is not None}
            if angle is not None and len(angles) < 5:
                raise RuntimeError("Insufficient fresh rotary samples during stationary observation")
            result = dict(duration_s=elapsed, stationary_resets=resets,
                          motor_min_turns=low, motor_max_turns=high,
                          motor_final_turns=motor, completed_at_monotonic_s=now,
                          settle_grace_s=settle_s, observation_elapsed_s=now - started)
            if angles:
                data = angles.values()
                result.update(mean_deg=statistics.mean(data), min_deg=min(data),
                              max_deg=max(data), samples=len(data))
            if accept_window is None or accept_window(result):
                if first_rejected is not None:
                    result["first_rejected_window"] = first_rejected
                return result
            if first_rejected is None:
                first_rejected = result
            last_rejected = result
        sleep(.01)

    if last_rejected is not None:
        return {**last_rejected, "first_rejected_window": first_rejected}
    raise RuntimeError("Motor did not remain stationary for the required observation before the deadline")
