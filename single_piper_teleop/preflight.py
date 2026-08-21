#!/usr/bin/env python3
"""Read-only dependency and hardware preflight for the compact pipeline."""

import argparse
import importlib
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
PIPER_CAN = "piper_can"


def check_import(name: str) -> None:
    module = importlib.import_module(name)
    version = getattr(module, "__version__", "installed")
    print(f"OK import {name}: {version}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--software-only", action="store_true")
    args = parser.parse_args()

    for module in ("numpy", "scipy", "placo", "cv2", "piper_sdk", "xrobotoolkit_sdk"):
        check_import(module)

    urdf = ROOT / "assets" / "piper_kinematics.urdf"
    if not urdf.is_file():
        raise RuntimeError(f"missing URDF: {urdf}")
    print(f"OK URDF: {urdf}")

    service = Path("/opt/apps/roboticsservice/RoboticsServiceProcess")
    if not service.is_file():
        raise RuntimeError("XRoboToolkit PC Service is missing from /opt/apps/roboticsservice")
    print(f"OK XR service: {service}")

    if args.software_only:
        return

    can = Path(f"/sys/class/net/{PIPER_CAN}")
    if not can.exists():
        raise RuntimeError(f"{PIPER_CAN} is absent")
    result = subprocess.run(
        ["ip", "-details", "link", "show", PIPER_CAN], check=True,
        text=True, capture_output=True,
    )
    if "state UP" not in result.stdout or "bitrate 1000000" not in result.stdout:
        raise RuntimeError(f"{PIPER_CAN} is not UP at 1 Mbps")
    print(f"OK {PIPER_CAN}: UP at 1 Mbps")

    import cv2
    from .wrist_camera import find_innomaker_device

    device = find_innomaker_device()
    capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
    ok, frame = capture.read() if capture.isOpened() else (False, None)
    capture.release()
    if not ok or frame is None:
        raise RuntimeError(f"InnoMaker wrist camera produced no frame: {device}")
    print(f"OK InnoMaker wrist camera: {device} {frame.shape[1]}x{frame.shape[0]}")


if __name__ == "__main__":
    main()
