"""No-motion ACT checkpoint check and hardware shadow rollout."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import time

import cv2
import numpy as np

from .hardware import CameraPair, ReadOnlyRobot
from .policy import ActRuntime
from .safety import ActionGate


def resolve_bundle(path: Path) -> Path:
    path = path.expanduser().resolve()
    if (path / "pretrained_model").is_dir():
        return path
    matches = [item for item in path.iterdir() if item.is_dir() and (item / "pretrained_model").is_dir()]
    if len(matches) != 1:
        raise RuntimeError(f"cannot resolve one checkpoint bundle below {path}")
    return matches[0]


def main() -> None:
    parser = argparse.ArgumentParser(description="PiPER–Tracer multi-model shadow rollout (never sends motion)")
    parser.add_argument("bundles", type=Path, nargs="+")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--local-camera", default="/dev/video10")
    parser.add_argument("--global-camera", default="/dev/video11")
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    runtimes = [ActRuntime(resolve_bundle(bundle), args.device) for bundle in args.bundles]
    model_ids = [runtime.manifest["model_id"] for runtime in runtimes]
    if len(set(model_ids)) != len(model_ids):
        parser.error("each bundle must have a distinct model_id")
    gate = ActionGate()
    cameras = CameraPair(args.local_camera, args.global_camera)
    robot = ReadOnlyRobot()
    stop = False

    def request_stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    output = (
        args.output.expanduser().resolve() if args.output is not None else
        (Path("log") / f"rollout-shadow-{time.strftime('%Y%m%d_%H%M%S')}").resolve()
    )
    if output.exists() and any(output.iterdir()):
        raise RuntimeError(f"output directory is not empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    log_path = output / "samples.jsonl"
    metadata_path = output / "metadata.json"
    metadata_path.write_text(json.dumps({
        "mode": "shadow_no_motion",
        "models": model_ids,
        "video_fps": 30,
        "video_codec": "MJPG",
        "image_color": "BGR in AVI; RGB supplied to policy",
    }, indent=2), encoding="utf-8")
    for runtime in runtimes:
        runtime.reset()
    started = time.monotonic()
    deadline = started + args.seconds
    tick = 0
    deadline_misses = 0
    try:
        cameras.start()
        robot.start()
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")
        local_video = cv2.VideoWriter(str(output / "local.avi"), fourcc, 30.0, (640, 480))
        global_video = cv2.VideoWriter(str(output / "global.avi"), fourcc, 30.0, (640, 480))
        if not local_video.isOpened() or not global_video.isOpened():
            raise RuntimeError("cannot create local/global MJPEG videos")
        with log_path.open("a", encoding="utf-8", buffering=1) as stream:
            print(f"SHADOW READY — {len(runtimes)} models, no motion commands; Ctrl-C stops.")
            while not stop and time.monotonic() < deadline:
                iteration = time.monotonic()
                local, global_frame = cameras.read_rgb()
                state, tracer = robot.state()
                observation_wall_ns = time.time_ns()
                observation_monotonic_ns = time.monotonic_ns()
                # Video frame N and every model result in JSONL row N use exactly
                # the same observation; timestamps retain exact timing.
                local_video.write(cv2.cvtColor(local, cv2.COLOR_RGB2BGR))
                global_video.write(cv2.cvtColor(global_frame, cv2.COLOR_RGB2BGR))
                predictions = {}
                for runtime in runtimes:
                    inference_started = time.perf_counter_ns()
                    raw = runtime.predict(local, global_frame, state)
                    inference_ms = (time.perf_counter_ns() - inference_started) / 1e6
                    safe = gate.apply(raw, state)
                    predictions[runtime.manifest["model_id"]] = {
                        "policy_action": raw.tolist(),
                        "gated_action": safe.tolist(),
                        "inference_ms": inference_ms,
                    }
                stream.write(json.dumps({
                    "observation_wall_ns": observation_wall_ns,
                    "observation_monotonic_ns": observation_monotonic_ns,
                    "logged_wall_ns": time.time_ns(), "tick": tick,
                    "video_frame_index": tick,
                    "state": state.tolist(), "models": predictions,
                    "tracer": tracer,
                }, separators=(",", ":")) + "\n")
                tick += 1
                remaining = 0.02 - (time.monotonic() - iteration)
                if remaining > 0:
                    time.sleep(remaining)
                else:
                    deadline_misses += 1
        print(f"SHADOW COMPLETE — {tick} ticks, output={output}")
    finally:
        if "local_video" in locals():
            local_video.release()
        if "global_video" in locals():
            global_video.release()
        robot.close()
        cameras.close()
        if output.exists():
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            metadata.update(
                ticks=tick,
                elapsed_s=time.monotonic() - started,
                deadline_misses=deadline_misses,
            )
            metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
