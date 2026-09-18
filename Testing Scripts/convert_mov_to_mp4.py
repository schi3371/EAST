"""Convert .mov videos in one folder to .mp4 using ffmpeg.

Example:
    python "Testing Scripts/convert_mov_to_mp4.py" "/Users/name/Desktop/videos"
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert .mov files in a folder to H.264 .mp4 files."
    )
    parser.add_argument("folder", type=Path, help="Folder containing .mov files")
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Replace an existing .mp4 with the same name",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    folder = args.folder.expanduser().resolve()

    if not folder.is_dir():
        print(f"Folder not found: {folder}", file=sys.stderr)
        return 1
    if shutil.which("ffmpeg") is None:
        print(
            "ffmpeg is not installed. On macOS, install Homebrew then run: brew install ffmpeg",
            file=sys.stderr,
        )
        return 1

    mov_files = sorted(
        path for path in folder.iterdir() if path.is_file() and path.suffix.lower() == ".mov"
    )
    if not mov_files:
        print(f"No .mov files found in: {folder}")
        return 0

    for source in mov_files:
        destination = source.with_suffix(".mp4")
        if destination.exists() and not args.overwrite:
            print(f"Skipping existing file: {destination.name}")
            continue

        command = [
            "ffmpeg",
            "-hide_banner",
            "-loglevel",
            "error",
            "-y" if args.overwrite else "-n",
            "-i",
            str(source),
            "-c:v",
            "libx264",
            "-crf",
            "18",
            "-preset",
            "medium",
            "-c:a",
            "aac",
            "-b:a",
            "192k",
            "-movflags",
            "+faststart",
            str(destination),
        ]
        print(f"Converting: {source.name} -> {destination.name}")
        result = subprocess.run(command, check=False)
        if result.returncode != 0:
            print(f"Conversion failed: {source.name}", file=sys.stderr)
        else:
            print(f"Created: {destination}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
