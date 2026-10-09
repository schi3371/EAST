#!/usr/bin/env python3
"""Append provisional angle estimates to saved P3022 CSVs, preserving all old fields.

Defaults to a dry run. --apply saves byte-for-byte backups before atomic replacements.
The selected full-scale voltage is an assumption, never an historical VCC measurement.
"""
import argparse
import csv
import hashlib
import io
import json
import math
import os
import shutil
import tempfile
from datetime import datetime
from pathlib import Path

from p3022_quick_test import ESTIMATE_FIELDS, estimate_fields


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def prepare_file(path, full_scale_voltage):
    original = path.read_bytes()
    reader = csv.DictReader(io.StringIO(original.decode("utf-8-sig"), newline=""))
    headers = reader.fieldnames
    if not headers or len(headers) != len(set(headers)):
        raise ValueError(f"{path.name}: missing or duplicate column names")
    if not {"voltage_v", "host_elapsed_s", "data_source"}.issubset(headers):
        raise ValueError(f"{path.name}: not a supported P3022 recording")
    if any(field in headers for field in ESTIMATE_FIELDS):
        raise ValueError(f"{path.name}: estimates already present; refusing to overwrite provenance")
    rows = list(reader)
    if not rows:
        raise ValueError(f"{path.name}: no measurements")
    output = io.StringIO(newline="")
    writer = csv.DictWriter(output, fieldnames=headers + ESTIMATE_FIELDS)
    writer.writeheader()
    above_five = estimated = above_scale = 0
    for number, row in enumerate(rows, 2):
        if None in row or any(value is None for value in row.values()):
            raise ValueError(f"{path.name}:{number}: malformed CSV row")
        try:
            voltage = float(row["voltage_v"])
        except ValueError as exc:
            raise ValueError(f"{path.name}:{number}: invalid voltage") from exc
        fields = estimate_fields(voltage, full_scale_voltage)
        above_five += math.isfinite(voltage) and voltage > 5.0
        estimated += bool(fields["estimated_shaft_angle_deg"])
        above_scale += fields["angle_estimate_status"] == "uncalibrated_above_assumed_full_scale"
        writer.writerow({**row, **fields})
    updated = output.getvalue().encode("utf-8")
    # Verify every original value, including timestamps and nominal validity flags.
    checked = list(csv.DictReader(io.StringIO(updated.decode("utf-8"), newline="")))
    if len(checked) != len(rows) or not all(
        {key: new[key] for key in headers} == old for old, new in zip(rows, checked)
    ):
        raise RuntimeError(f"{path.name}: original-column preservation failed")
    return {
        "path": path, "original": original, "updated": updated,
        "record": {"filename": path.name, "rows": len(rows),
                   "above_nominal_5v": above_five, "estimated_rows": estimated,
                   "above_assumed_full_scale": above_scale,
                   "original_sha256": sha256(original), "updated_sha256": sha256(updated)},
    }


def backfill(paths, full_scale_voltage, apply=False):
    # Validate every input before backing up or writing any recording.
    plans = [prepare_file(Path(path), full_scale_voltage) for path in paths]
    if len({p["path"].resolve() for p in plans}) != len(plans):
        raise ValueError("Duplicate input recording")
    report = {
        "assumed_full_scale_voltage_v": full_scale_voltage,
        "basis": "Assumed linear transfer; not calibrated; not simultaneous historical VCC",
        "original_columns_preserved": True,
        "files": [p["record"] for p in plans],
    }
    if not apply or not plans:
        return report
    parents = {p["path"].resolve().parent for p in plans}
    if len(parents) != 1:
        raise ValueError("Apply one recording directory at a time")
    stamp = datetime.now().astimezone().strftime("%Y%m%dT%H%M%S_%f%z")
    backup_dir = parents.pop() / "originals_before_angle_estimates" / stamp
    backup_dir.mkdir(parents=True, exist_ok=False)
    report["backup_directory"] = str(backup_dir)
    report["completed_files"] = []
    manifest = backup_dir / "migration_manifest.json"
    for plan in plans:
        path = plan["path"]
        if path.read_bytes() != plan["original"]:
            raise RuntimeError(f"{path.name} changed during preparation; no replacement performed")
        shutil.copy2(path, backup_dir / path.name)
        if (backup_dir / path.name).read_bytes() != plan["original"]:
            raise RuntimeError("Backup verification failed")
    manifest.write_text(json.dumps(report, indent=2) + "\n")
    for plan in plans:
        path = plan["path"]
        if path.read_bytes() != plan["original"]:
            raise RuntimeError(f"{path.name} changed; refusing replacement")
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(dir=path.parent, prefix=".p3022_estimate_", delete=False) as handle:
                temporary = Path(handle.name)
                handle.write(plan["updated"])
                handle.flush()
                os.fsync(handle.fileno())
            os.chmod(temporary, path.stat().st_mode & 0o777)
            os.replace(temporary, path)
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()
        report["completed_files"].append(path.name)
        manifest.write_text(json.dumps(report, indent=2) + "\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    parser.add_argument("--full-scale-voltage", type=float, required=True,
                        help="Explicit assumed full-scale voltage; not measured historical VCC")
    parser.add_argument("--apply", action="store_true", help="Back up originals and append estimates in place")
    args = parser.parse_args()
    try:
        estimate_fields(0.0, args.full_scale_voltage)
        paths = sorted(args.directory.glob("p3022_*.csv"))
        if not paths:
            raise ValueError("No p3022_*.csv recordings found")
        report = backfill(paths, args.full_scale_voltage, args.apply)
    except (ValueError, OSError, RuntimeError) as exc:
        parser.exit(1, str(exc) + "\n")
    print(json.dumps(report, indent=2))
    if not args.apply:
        print("Dry run only. Add --apply to back up originals and append estimates.")


if __name__ == "__main__":
    main()
