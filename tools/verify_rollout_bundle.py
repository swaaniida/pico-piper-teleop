#!/usr/bin/env python3
"""Perform non-executing structural checks on a rollout checkpoint bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


EXPECTED_IMAGES = ["observation.images.local", "observation.images.global"]
EXPECTED_STATE = [
    "joint_1_rad", "joint_2_rad", "joint_3_rad", "joint_4_rad",
    "joint_5_rad", "joint_6_rad", "gripper_mm", "base_linear_mps",
    "base_angular_rad_s",
]
EXPECTED_ACTION = EXPECTED_STATE.copy()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require_value(value: object, name: str) -> None:
    if value is None or value == "" or (isinstance(value, str) and value.startswith("REQUIRED")):
        raise RuntimeError(f"manifest field is not filled: {name}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bundle", type=Path)
    args = parser.parse_args()
    root = args.bundle.resolve()
    manifest_path = root / "manifest.json"
    checksums_path = root / "SHA256SUMS"
    model_dir = root / "pretrained_model"
    for path in (manifest_path, checksums_path, model_dir):
        if not path.exists():
            raise RuntimeError(f"missing required bundle path: {path}")

    manifest = json.loads(manifest_path.read_text())
    for key in ("model_id", "policy_type", "source_git_commit", "action_representation"):
        require_value(manifest.get(key), key)
    training = manifest.get("training_dataset", {})
    software = manifest.get("software", {})
    for key in ("repo_id", "revision", "episodes", "fps"):
        require_value(training.get(key), f"training_dataset.{key}")
    for key in ("lerobot_version", "torch_version", "torchvision_version", "cuda_runtime"):
        require_value(software.get(key), f"software.{key}")

    features = manifest.get("features", {})
    if features.get("image_keys") != EXPECTED_IMAGES:
        raise RuntimeError(f"unexpected image key/order: {features.get('image_keys')}")
    if features.get("image_shape_hwc") != [480, 640, 3]:
        raise RuntimeError(f"unexpected image shape: {features.get('image_shape_hwc')}")
    if features.get("state_names") != EXPECTED_STATE:
        raise RuntimeError("state names/order do not match the robot contract")
    if features.get("action_names") != EXPECTED_ACTION:
        raise RuntimeError("action names/order do not match the robot contract")
    if training.get("fps") != 50:
        raise RuntimeError(f"unexpected training fps: {training.get('fps')}")
    if manifest.get("action_representation") not in {"absolute", "stepwise_delta", "chunkwise_delta"}:
        raise RuntimeError("unsupported action_representation")

    config_candidates = list(model_dir.glob("*config*.json")) + list(model_dir.glob("*.safetensors"))
    if not any(path.name == "config.json" for path in config_candidates):
        raise RuntimeError("pretrained_model/config.json is missing")
    weights = list(model_dir.glob("*.safetensors")) + list(model_dir.glob("*.bin")) + list(model_dir.glob("*.pt"))
    if not weights:
        raise RuntimeError("no model weights found in pretrained_model")

    verified = 0
    for line in checksums_path.read_text().splitlines():
        if not line.strip():
            continue
        expected, relative = line.split(maxsplit=1)
        relative = relative.lstrip("* ")
        path = root / relative
        if not path.is_file():
            raise RuntimeError(f"checksummed file is missing: {relative}")
        actual = sha256(path)
        if actual != expected:
            raise RuntimeError(f"checksum mismatch: {relative}")
        verified += 1
    if verified == 0:
        raise RuntimeError("SHA256SUMS contains no files")

    print(f"BUNDLE_OK model={manifest['model_id']} files={verified} weights={len(weights)}")
    print("Structural verification only: no checkpoint was loaded and no robot command was sent.")


if __name__ == "__main__":
    main()
