#!/usr/bin/env python3
"""Download and verify one floor-1ball model from its GitHub Release."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path
from urllib.request import urlretrieve


ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = "swaaniida/pico-piper-teleop"


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("model_id")
    parser.add_argument("--output", type=Path, default=ROOT / "checkpoints")
    args = parser.parse_args()
    metadata = ROOT / "deployment" / "models" / args.model_id / "manifest.json"
    if not metadata.is_file():
        raise SystemExit(f"unknown model_id: {args.model_id}")
    manifest = json.loads(metadata.read_text(encoding="utf-8"))
    release = manifest["release"]
    destination = args.output.expanduser().resolve() / args.model_id
    destination.mkdir(parents=True, exist_ok=True)
    archive = destination / release["asset"]
    if not archive.exists():
        url = (
            f"https://github.com/{REPOSITORY}/releases/download/"
            f"{release['tag']}/{release['asset']}"
        )
        partial = archive.with_suffix(archive.suffix + ".partial")
        print(f"Downloading {url}")
        urlretrieve(url, partial)
        partial.replace(archive)
    actual = digest(archive)
    if actual != release["asset_sha256"]:
        raise SystemExit(f"archive SHA-256 mismatch: {actual}")
    listing = subprocess.run(
        ["tar", "--zstd", "-tf", str(archive)], check=True,
        capture_output=True, text=True,
    ).stdout.splitlines()
    if any(name.startswith("/") or ".." in Path(name).parts for name in listing):
        raise SystemExit("unsafe path in release archive")
    subprocess.run(
        ["tar", "--zstd", "-xf", str(archive), "-C", str(destination)],
        check=True,
    )
    roots = [path for path in destination.iterdir() if path.is_dir() and (path / "pretrained_model").is_dir()]
    if len(roots) != 1:
        raise SystemExit(f"expected one extracted bundle, found {len(roots)}")
    print(roots[0])


if __name__ == "__main__":
    main()
