"""RUN/PLAY starts six empty-fixture identification runs after a five-second countdown.
Operator SC, fixture 0. Requires the operator-confirmed empty fixture already at
physical 90 degrees and clear, with normal enclosure/E-stop protection. Close the
EAST GUI first. Ctrl+C stops; --preview prints the plan without opening hardware.
"""
from pathlib import Path
import sys
import time

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "analysis" / "motion_mapping_cli"))
import run_motion_mapping


def main(argv=None):
    options = list(sys.argv[1:] if argv is None else argv)
    preview = "--preview" in options
    if preview:
        options.remove("--preview")
    args = ["--repo", str(REPO), "--operator", "SC", "--fixture-id", "0"]
    if not preview:
        print("EAST MOTION EXPERIMENT: fixture 0, empty and physically at 90 degrees.", flush=True)
        print("Close the GUI. Six runs: 0.5/5 deg/s, nominal +/-10 deg, 3 cycles each.", flush=True)
        print("Press Ctrl+C to cancel/stop. Keep the physical E-stop accessible.", flush=True)
        for remaining in range(5, 0, -1):
            print(f"Connecting in {remaining}...", flush=True)
            time.sleep(1)
        args += ["--execute", "--confirmed-empty-at-90"]
    return run_motion_mapping.main(args + options)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit("Cancelled before hardware execution.")
    except (ValueError, RuntimeError, FileNotFoundError) as exc:
        raise SystemExit(str(exc))
