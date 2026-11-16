"""Analyse completed EAST strain recordings and their matched empty references."""
import csv
import json
import math
from collections import defaultdict
from pathlib import Path, PureWindowsPath
from statistics import mean, median, stdev

ANGLES = tuple(i / 4 for i in range(-32, 33))
PHASES = {
    "moving_to_max": "increasing_machine_angle",
    "moving_to_min": "decreasing_machine_angle",
}
SETTINGS = ("commanded_afo_speed_deg_s", "commanded_afo_acceleration_deg_s2",
            "min_angle_deg", "max_angle_deg")
CALIBRATION = ("mass_kg_per_voltage_ratio", "standard_gravity_m_s2", "lever_arm_m",
               "force_angle_polynomial_deg")
WINDOWS = ((1, 4), (-4, -1))
WIDE_WINDOWS = ((1, 8), (-8, -1))
STANDARD_EMPTY_BATCH = "c5aaf9052435437bb2ccb050e453b92c"
STANDARD_EMPTY_NAME = "Empty calibration · 1°/s Standard AFO Test"


def sibling(path, recorded_name):
    return path.with_name(PureWindowsPath(recorded_name).name)


def read_run(path):
    metadata = json.loads(path.read_text(encoding="utf-8-sig"))
    recorder = metadata.get("csv_recorder", {})
    if (metadata.get("run_status") != "completed" or metadata.get("outcome_pending_cleanup")
            or recorder.get("error") or not recorder.get("worker_complete")):
        raise ValueError(f"Recording is incomplete or has a recorder error: {path.name}")
    return metadata


def linked_reference(path, loaded, batch=None):
    batch = batch or (loaded.get("protocol_batch") or {}).get("baseline_batch_id")
    if not batch:
        raise ValueError("This run has no linked empty reference. Choose a matching empty recording.")
    manifests = [p for p in path.parent.glob(f"*{batch}.json")
                 if p.name.startswith(("three_speed_", "motion_suite_"))]
    if len(manifests) != 1:
        raise ValueError("The linked empty reference manifest is unavailable in this run's folder.")
    manifest = json.loads(manifests[0].read_text(encoding="utf-8-sig"))
    matches = [run for run in manifest["runs"] if run.get("status") == "completed"
               and all(run["parameters"][key] == loaded["test_parameters"][key] for key in SETTINGS)]
    if len(matches) != 1:
        raise ValueError("The linked reference needs exactly one completed matching condition.")
    return sibling(manifests[0], matches[0]["metadata_file"])


def sweep_curve(samples):
    if len(samples) < 2 or min(a for a, _ in samples) > ANGLES[0] or max(a for a, _ in samples) < ANGLES[-1]:
        return None
    bins = defaultdict(list)
    for angle, torque in samples:
        bins[round(angle * 8) / 8].append(torque)
    ordered = sorted((angle, median(values)) for angle, values in bins.items())
    if len(ordered) < 2:
        return None
    result = []
    index = 0
    for angle in ANGLES:
        while index < len(ordered) - 2 and ordered[index + 1][0] < angle:
            index += 1
        x0, y0 = ordered[index]
        x1, y1 = ordered[index + 1]
        weight = (angle - x0) / (x1 - x0)
        result.append(y0 + weight * (y1 - y0))
    return result


def read_curves(path, metadata):
    csv_path = sibling(path, metadata["csv_file"])
    sweeps = defaultdict(list)
    with csv_path.open(newline="", encoding="utf-8-sig") as stream:
        count = 0
        for count, row in enumerate(csv.DictReader(stream), 1):
            phase = row["Motion Phase"]
            if (phase not in PHASES or row["Movement Direction"] != PHASES[phase]
                    or row["Rotary Sample Valid"].lower() != "true"):
                continue
            try:
                angle = float(row["Measured Fixture Angle (deg)"])
                torque = float(row["Raw Torque (Nm)"])
            except ValueError:
                continue
            if math.isfinite(angle) and math.isfinite(torque):
                sweeps[int(row["Cycle"]), phase].append((angle, torque))
    if count != metadata["sample_count"] or count != metadata["csv_recorder"]["rows_persisted"]:
        raise ValueError(f"CSV row count does not match the saved metadata: {csv_path.name}")

    curves = {}
    excluded = []
    for cycle in range(2, metadata["completed_cycles"] + 1):
        pair = [sweep_curve(sweeps[cycle, phase]) for phase in PHASES]
        if any(trace is None for trace in pair):
            excluded.append(cycle)
        else:
            curves[cycle] = pair
    return curves, excluded


def mean_curves(curves):
    return [[mean(point) for point in zip(*direction)] for direction in zip(*curves)]


def resisting_slope(curve, window):
    samples = [(angle, torque) for angle, torque in zip(ANGLES, curve)
               if window[0] <= angle <= window[1]]
    x, y = map(mean, zip(*samples))
    return -sum((angle - x) * (torque - y) for angle, torque in samples) / sum(
        (angle - x) ** 2 for angle, _ in samples)


def zero_crossings(curve):
    """Find sign-changing roots without extrapolating the corrected sweep."""
    crossings = []
    previous = None
    for angle, torque in zip(ANGLES, curve):
        if torque == 0:
            continue
        if previous is not None:
            x0, y0 = previous
            if (y0 < 0) != (torque < 0):
                crossings.append(x0 - y0 * (angle - x0) / (torque - y0))
        previous = angle, torque
    return crossings


def analyse_stiffness(path, reference_path=None, *, standard_empty=False):
    loaded = read_run(path)
    if loaded["test_parameters"]["test_type"] != "afo_test":
        raise ValueError("Choose a completed loaded AFO run.")
    parameters = loaded["test_parameters"]
    if standard_empty and tuple(parameters.get(key) for key in SETTINGS) != (1.0, 100.0, 10.0, 10.0):
        raise ValueError("This empty calibration requires 1°/s, ±10°, and 100°/s².")
    use_linked = reference_path is None
    if standard_empty:
        reference_path = linked_reference(path, loaded, STANDARD_EMPTY_BATCH)
    else:
        reference_path = reference_path or linked_reference(path, loaded)
    reference_name = STANDARD_EMPTY_NAME if standard_empty else (
        (loaded.get("protocol") or {}).get("empty_reference_name") if use_linked else None
    )
    empty = read_run(reference_path)
    empty_parameters = empty["test_parameters"]
    # The archived EMPTYCalibration batch used the AFO mode with ID 0.
    archived_empty = (empty_parameters["afo_id"] == "0"
                      and str(empty_parameters["file_prefix"]).startswith("EMPTYCalibration_Bc5aaf905_"))
    if path.resolve() == reference_path.resolve() or (
            empty_parameters["test_type"] != "empty_machine_baseline" and not archived_empty):
        raise ValueError("Choose an empty-fixture recording.")
    if any(loaded["test_parameters"][key] != empty_parameters[key] for key in SETTINGS):
        raise ValueError("The empty reference has different speed, acceleration or angle settings.")
    for key in ("motion_method", "torque_angle_source"):
        if not loaded.get(key) or loaded[key] != empty.get(key):
            raise ValueError("The empty reference uses a different motion or torque convention.")
    rotary_hash = loaded["rotary_zero_reference"]["calibration_sha256"]
    if not rotary_hash or rotary_hash != empty["rotary_zero_reference"]["calibration_sha256"]:
        raise ValueError("The empty reference uses a different rotary calibration.")
    if any(loaded["calibration"][key] != empty["calibration"][key] for key in CALIBRATION):
        raise ValueError("The empty reference uses different torque calibration coefficients.")

    fitted, excluded = read_curves(path, loaded)
    baseline, empty_excluded = read_curves(reference_path, empty)
    if len(fitted) < 2:
        raise ValueError(f"Need at least two usable cycles after conditioning. Found {len(fitted)}.")
    if len(baseline) < 2:
        raise ValueError("The empty reference needs at least two usable cycles after conditioning.")
    empty_mean = mean_curves(baseline.values())
    corrected = {cycle: [[f - e for f, e in zip(trace, ref)]
                         for trace, ref in zip(traces, empty_mean)]
                 for cycle, traces in fitted.items()}
    primary_windows = WIDE_WINDOWS if (parameters["min_angle_deg"], parameters["max_angle_deg"]) == (10, 10) else WINDOWS

    def fit_branches(windows):
        branches = []
        for index, direction in enumerate(("Increasing", "Decreasing")):
            for window in windows:
                slopes = {cycle: resisting_slope(traces[index], window) for cycle, traces in corrected.items()}
                branches.append(dict(direction=direction, fit_window_deg=window, mean_nm_per_deg=mean(slopes.values()),
                                     sd_nm_per_deg=stdev(slopes.values()), cycle_slopes_nm_per_deg=slopes))
        return branches

    corrected_mean = mean_curves(corrected.values())
    neutral = []
    for direction, curve in zip(("Increasing", "Decreasing"), corrected_mean):
        crossings = zero_crossings(curve)
        neutral.append(dict(direction=direction, angle_deg=crossings[0] if len(crossings) == 1 else None,
                            zero_crossings_deg=crossings, resisting_torque_at_zero_nm=-curve[ANGLES.index(0)]))
    separation = [abs(a - b) for a, b in zip(*corrected_mean)]
    loop_area = sum((a + b) / 2 for a, b in zip(separation, separation[1:])) * math.radians(0.25)

    return dict(analysis_version=2, specimen_id=loaded["test_parameters"]["afo_id"],
                loaded_metadata=str(path), empty_metadata=str(reference_path),
                reference_name=reference_name,
                requested_speed_deg_s=loaded["test_parameters"][SETTINGS[0]],
                conditioning_cycle=1, cycles=list(fitted), excluded_cycles=excluded,
                empty_cycles=list(baseline), empty_excluded_cycles=empty_excluded,
                branches=fit_branches(primary_windows), primary_fit_windows_deg=primary_windows,
                near_neutral_branches=fit_branches(WINDOWS) if primary_windows == WIDE_WINDOWS else [],
                neutral_by_sweep=neutral, corrected_loop_area_j=loop_area,
                loop_area_window_deg=(ANGLES[0], ANGLES[-1]),
                angles_deg=ANGLES, fitted_torque_nm=mean_curves(fitted.values()),
                empty_torque_nm=empty_mean, corrected_torque_nm=corrected_mean)
