#!/usr/bin/env python3
"""Convert completed PiPER-Tracer episodes to a LeRobot v3 dataset.

The raw USB recordings are read-only inputs. Camera overlays are not present in
the source JPEGs, so this export also contains clean training images.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image
from lerobot.datasets.lerobot_dataset import LeRobotDataset


TASK = (
    "Pick up the tennis ball from the floor, place it in the blue cup, "
    "and return to the locomotion init pose."
)
PHASE_IDS = {"locomotion": 0, "manipulation": 1, "transport": 2}
SOURCE_IDS = {"teleoperation": 0, "automatic": 1}


def load_complete_episodes(root: Path) -> list[Path]:
    episodes = []
    for path in sorted(p for p in root.iterdir() if p.is_dir()):
        metadata_path = path / "metadata.json"
        samples_path = path / "samples.jsonl"
        if not metadata_path.is_file() or not samples_path.is_file():
            continue
        metadata = json.loads(metadata_path.read_text())
        if metadata.get("status") == "complete":
            episodes.append(path)
    return episodes


def image_path(ep: Path, sample: dict, key: str, previous: Path | None) -> Path | None:
    ref = sample.get(key)
    if ref and ref.get("color_path"):
        candidate = ep / ref["color_path"]
        if candidate.is_file():
            return candidate
    return previous


def first_referenced_image(ep: Path, samples: list[dict], key: str, fallback_dir: str) -> Path:
    for sample in samples:
        path = image_path(ep, sample, key, None)
        if path is not None:
            return path
    fallback = next(iter(sorted((ep / fallback_dir / "color").glob("*.jpg"))), None)
    if fallback is None:
        raise RuntimeError(f"No image available for {key} in {ep.name}")
    return fallback


def read_rgb(path: Path, cache: dict[Path, np.ndarray]) -> np.ndarray:
    image = cache.get(path)
    if image is None:
        with Image.open(path) as source:
            image = np.asarray(source.convert("RGB"), dtype=np.uint8)
        if image.shape != (480, 640, 3):
            raise RuntimeError(f"Unexpected image shape {image.shape}: {path}")
        cache.clear()  # Only the most recent frame from each camera is reused.
        cache[path] = image
    return image


def f32(values: list[float]) -> np.ndarray:
    return np.asarray(values, dtype=np.float32)


def convert(source: Path, output: Path, repo_id: str, limit: int | None) -> None:
    episodes = load_complete_episodes(source)
    if limit is not None:
        episodes = episodes[:limit]
    if not episodes:
        raise RuntimeError("No completed episodes found")
    if output.exists():
        raise RuntimeError(f"Output path already exists: {output}")

    features = {
        "observation.images.local": {
            "dtype": "video", "shape": (480, 640, 3), "names": ["height", "width", "channels"]
        },
        "observation.images.global": {
            "dtype": "video", "shape": (480, 640, 3), "names": ["height", "width", "channels"]
        },
        "observation.state": {
            "dtype": "float32", "shape": (9,),
            "names": [
                "joint_1_rad", "joint_2_rad", "joint_3_rad", "joint_4_rad", "joint_5_rad",
                "joint_6_rad", "gripper_mm", "base_linear_mps", "base_angular_rad_s",
            ],
        },
        "action": {
            "dtype": "float32", "shape": (9,),
            "names": [
                "joint_1_rad", "joint_2_rad", "joint_3_rad", "joint_4_rad", "joint_5_rad",
                "joint_6_rad", "gripper_mm", "base_linear_mps", "base_angular_rad_s",
            ],
        },
        "observation.base_odometry": {
            "dtype": "float32", "shape": (6,),
            "names": ["x_m", "y_m", "orientation_z", "orientation_w", "linear_mps", "angular_rad_s"],
        },
        "observation.ee_pose_raw": {
            "dtype": "float32", "shape": (6,),
            "names": ["x_0p001", "y_0p001", "z_0p001", "roll_0p001", "pitch_0p001", "yaw_0p001"],
        },
        "observation.gripper_effort": {"dtype": "float32", "shape": (1,), "names": ["effort"]},
        "annotation.phase": {"dtype": "int64", "shape": (1,), "names": ["phase_id"]},
        "annotation.control_source": {"dtype": "int64", "shape": (1,), "names": ["source_id"]},
    }
    dataset = LeRobotDataset.create(
        repo_id=repo_id,
        fps=50,
        features=features,
        root=output,
        robot_type="piper_tracer",
        use_videos=True,
        vcodec="h264",
        streaming_encoding=True,
        encoder_threads=2,
    )

    for number, ep in enumerate(episodes, 1):
        samples = [json.loads(line) for line in (ep / "samples.jsonl").read_text().splitlines() if line]
        local = first_referenced_image(ep, samples, "wrist_camera_frame_reference", "camera")
        global_ = first_referenced_image(ep, samples, "global_camera_frame_reference", "global_camera")
        local_cache: dict[Path, np.ndarray] = {}
        global_cache: dict[Path, np.ndarray] = {}

        for sample in samples:
            local = image_path(ep, sample, "wrist_camera_frame_reference", local)
            global_ = image_path(ep, sample, "global_camera_frame_reference", global_)
            assert local is not None and global_ is not None

            status = sample["piper_status"]
            odom = sample["tracer_status"]["odometry"]
            measured_rad = [math.radians(v) for v in sample["piper_joint_deg"]]
            commanded_rad = [math.radians(v) for v in sample["commanded_joint_deg"]]
            state = measured_rad + [float(status["gripper_stroke_mm"]), odom["linear_mps"], odom["angular_rad_s"]]
            action = commanded_rad + [
                float(sample["commanded_gripper_mm"]),
                float(sample["tracer_command"]["linear_mps"]),
                float(sample["tracer_command"]["angular_rad_s"]),
            ]
            dataset.add_frame({
                "observation.images.local": read_rgb(local, local_cache),
                "observation.images.global": read_rgb(global_, global_cache),
                "observation.state": f32(state),
                "action": f32(action),
                "observation.base_odometry": f32([
                    odom["x_m"], odom["y_m"], odom["orientation_z"], odom["orientation_w"],
                    odom["linear_mps"], odom["angular_rad_s"],
                ]),
                "observation.ee_pose_raw": f32(status["end_pose_0p001"]),
                "observation.gripper_effort": f32([status["gripper_effort"]]),
                "annotation.phase": np.asarray([PHASE_IDS.get(sample.get("episode_phase"), -1)], dtype=np.int64),
                "annotation.control_source": np.asarray(
                    [SOURCE_IDS.get(sample.get("control_source"), -1)], dtype=np.int64
                ),
                "task": TASK,
            })
        dataset.save_episode(parallel_encoding=False)
        print(f"[{number:02d}/{len(episodes):02d}] {ep.name}: {len(samples)} frames", flush=True)

    dataset.finalize()
    print(f"Finalized {len(episodes)} episodes at {output}", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repo-id", default="swaaniida/floor-1ball")
    parser.add_argument("--limit", type=int)
    args = parser.parse_args()
    convert(args.source, args.output, args.repo_id, args.limit)


if __name__ == "__main__":
    main()
