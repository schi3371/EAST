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
    """Locate unique sign-changing roots, including an exact grid-point zero."""
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


def loading_fit(curve, neutral, increasing):
    """OLS resisting-torque fit with the interpolated zero point included."""
    samples = [(angle, -torque) for angle, torque in zip(ANGLES, curve)
               if (angle >= neutral if increasing else angle <= neutral)]
    samples.insert(0 if increasing else len(samples), (neutral, 0.0))
    if len(set(angle for angle, _ in samples)) < 4:
        raise ValueError("Fewer than four distinct angles in the loading interval")
    x_mean, y_mean = map(mean, zip(*samples))
    slope = sum((x - x_mean) * (y - y_mean) for x, y in samples) / sum(
        (x - x_mean) ** 2 for x, _ in samples)
    return dict(stiffness_nm_per_deg=slope, intercept_nm=y_mean - slope * x_mean,
                fit_window_deg=(neutral, ANGLES[-1]) if increasing else (ANGLES[0], neutral))


def cycle_properties(traces):
    """Calculate the thesis's five properties independently for one cycle."""
    fits = []
    for index, (direction, outcome) in enumerate((("Increasing", "DF loading"), ("Decreasing", "PF loading"))):
        crossings = zero_crossings(traces[index])
        if len(crossings) != 1:
            reason = "no crossing" if not crossings else "multiple crossings"
            raise ValueError(f"{direction} sweep has {reason} within measured ±8°")
        neutral = crossings[0]
        fit = loading_fit(traces[index], neutral, index == 0)
        fits.append(dict(direction=direction, outcome=outcome, neutral_angle_deg=neutral,
                         resisting_torque_at_zero_nm=-traces[index][ANGLES.index(0)], **fit))
    difference = [a - b for a, b in zip(*traces)]
    area = abs(sum((a + b) / 2 for a, b in zip(difference, difference[1:])) * math.radians(0.25))
    return dict(loading_branches=fits, corrected_loop_area_j=area)


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
    properties = {}
    exclusion_reasons = {cycle: "Insufficient measured-angle coverage" for cycle in excluded}
    for cycle, traces in corrected.items():
        try:
            properties[cycle] = cycle_properties(traces)
        except ValueError as error:
            exclusion_reasons[cycle] = str(error)
    if len(properties) < 2:
        details = "; ".join(f"cycle {cycle}: {reason}" for cycle, reason in exclusion_reasons.items())
        raise ValueError(f"Need at least two usable cycles with unique neutral crossings. "
                         f"Found {len(properties)}. {details}")
    fitted = {cycle: fitted[cycle] for cycle in properties}
    corrected = {cycle: corrected[cycle] for cycle in properties}
    branches = []
    neutral = []
    for index, direction in enumerate(("Increasing", "Decreasing")):
        cycle_fits = {cycle: values["loading_branches"][index] for cycle, values in properties.items()}
        slopes = {cycle: fit["stiffness_nm_per_deg"] for cycle, fit in cycle_fits.items()}
        roots = {cycle: fit["neutral_angle_deg"] for cycle, fit in cycle_fits.items()}
        torques = {cycle: fit["resisting_torque_at_zero_nm"] for cycle, fit in cycle_fits.items()}
        representative_neutral = mean(roots.values())
        window = ((representative_neutral, ANGLES[-1]) if index == 0
                  else (ANGLES[0], representative_neutral))
        branches.append(dict(outcome="DF loading" if index == 0 else "PF loading",
                             direction=direction, fit_window_deg=window,
                             cycle_fit_windows_deg={cycle: fit["fit_window_deg"] for cycle, fit in cycle_fits.items()},
                             mean_nm_per_deg=mean(slopes.values()), sd_nm_per_deg=stdev(slopes.values()),
                             cycle_slopes_nm_per_deg=slopes))
        neutral.append(dict(direction=direction, angle_deg=representative_neutral,
                            sd_angle_deg=stdev(roots.values()), cycle_angles_deg=roots,
                            resisting_torque_at_zero_nm=mean(torques.values()),
                            sd_torque_at_zero_nm=stdev(torques.values())))
    areas = {cycle: values["corrected_loop_area_j"] for cycle, values in properties.items()}
    return dict(analysis_version=4, specimen_id=loaded["test_parameters"]["afo_id"],
                method="Per-cycle neutral-referenced DF/PF loading; magnitude of signed sweep-difference integral",
                loaded_metadata=str(path), empty_metadata=str(reference_path), reference_name=reference_name,
                requested_speed_deg_s=parameters[SETTINGS[0]], conditioning_cycle=1,
                cycles=list(properties), excluded_cycles=sorted(exclusion_reasons),
                excluded_cycle_reasons=exclusion_reasons,
                empty_cycles=list(baseline), empty_excluded_cycles=empty_excluded,
                branches=branches, primary_fit_windows_deg=[branch["fit_window_deg"] for branch in branches],
                neutral_by_sweep=neutral, corrected_loop_area_j=mean(areas.values()),
                corrected_loop_area_sd_j=stdev(areas.values()), cycle_properties=properties,
                loop_area_window_deg=(ANGLES[0], ANGLES[-1]), angles_deg=ANGLES,
                fitted_torque_nm=mean_curves(fitted.values()), empty_torque_nm=empty_mean,
                corrected_torque_nm=mean_curves(corrected.values()))
