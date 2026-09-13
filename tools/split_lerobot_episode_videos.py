#!/usr/bin/env python3
"""Materialize one local/global MP4 per LeRobot episode for Hub browsing."""

from __future__ import annotations

import argparse
import json
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import pyarrow.parquet as pq


CAMERAS = ("local", "global")


def split_one(root: Path, output: Path, row: dict, camera: str) -> dict:
    prefix = f"videos/observation.images.{camera}"
    chunk = row[f"{prefix}/chunk_index"]
    file = row[f"{prefix}/file_index"]
    start = row[f"{prefix}/from_timestamp"]
    length = row["length"]
    source = root / prefix / f"chunk-{chunk:03d}" / f"file-{file:03d}.mp4"
    target = output / f"episode_{row['episode_index']:03d}" / f"{camera}.mp4"
    target.parent.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        [
            "ffmpeg", "-nostdin", "-loglevel", "error", "-y",
            "-ss", f"{start:.9f}", "-i", str(source), "-frames:v", str(length),
            "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "30",
            "-g", "2", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(target),
        ],
        check=True,
    )
    probe = subprocess.run(
        [
            "ffprobe", "-v", "error", "-select_streams", "v:0",
            "-show_entries", "stream=nb_frames", "-of", "default=nw=1:nk=1", str(target),
        ],
        check=True, capture_output=True, text=True,
    )
    actual = int(probe.stdout.strip())
    if actual != length:
        raise RuntimeError(f"{target}: expected {length} frames, got {actual}")
    return {
        "episode_index": row["episode_index"], "camera": camera,
        "frames": actual, "fps": 50, "path": str(target.relative_to(output)),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--workers", type=int, default=4)
    args = parser.parse_args()
    if args.output.exists():
        raise RuntimeError(f"Output already exists: {args.output}")
    table = pq.read_table(args.root / "meta/episodes/chunk-000/file-000.parquet")
    rows = table.to_pylist()
    jobs = [(row, camera) for row in rows for camera in CAMERAS]
    results = []
    with ThreadPoolExecutor(max_workers=args.workers) as pool:
        futures = [pool.submit(split_one, args.root, args.output, row, camera) for row, camera in jobs]
        for done, future in enumerate(as_completed(futures), 1):
            result = future.result()
            results.append(result)
            print(f"[{done:02d}/{len(jobs)}] episode {result['episode_index']:03d} {result['camera']}", flush=True)
    results.sort(key=lambda x: (x["episode_index"], x["camera"]))
    (args.output / "manifest.json").write_text(json.dumps(results, indent=2) + "\n")


if __name__ == "__main__":
    main()
