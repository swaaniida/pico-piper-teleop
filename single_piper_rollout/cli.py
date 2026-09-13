"""No-motion ACT checkpoint check and hardware shadow rollout."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import signal
import time

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
    parser = argparse.ArgumentParser(description="PiPER–Tracer ACT shadow rollout (never sends motion)")
    parser.add_argument("bundle", type=Path)
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--local-camera", default="/dev/video10")
    parser.add_argument("--global-camera", default="/dev/video11")
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--log", type=Path, default=Path("log/rollout-shadow.jsonl"))
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    runtime = ActRuntime(resolve_bundle(args.bundle), args.device)
    gate = ActionGate()
    cameras = CameraPair(args.local_camera, args.global_camera)
    robot = ReadOnlyRobot()
    stop = False

    def request_stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    args.log.parent.mkdir(parents=True, exist_ok=True)
    runtime.reset()
    started = time.monotonic()
    deadline = started + args.seconds
    tick = 0
    try:
        cameras.start()
        robot.start()
        with args.log.open("a", encoding="utf-8", buffering=1) as stream:
            print("SH READY — no robot commands are ever sent; Ctrl-C stops.")
            while not stop and time.monotonic() < deadline:
                iteration = time.monotonic()
                local, global_frame = cameras.read_rgb()
                state, tracer = robot.state()
                raw = runtime.predict(local, global_frame, state)
                safe = gate.apply(raw, state)
                stream.write(json.dumps({
                    "time_ns": time.time_ns(), "tick": tick,
                    "state": state.tolist(), "policy_action": raw.tolist(),
                    "gated_action": safe.tolist(), "tracer": tracer,
                }, separators=(",", ":")) + "\n")
                tick += 1
                remaining = 0.02 - (time.monotonic() - iteration)
                if remaining > 0:
                    time.sleep(remaining)
        print(f"SHADOW COMPLETE — {tick} ticks, log={args.log}")
    finally:
        robot.close()
        cameras.close()


if __name__ == "__main__":
    main()
