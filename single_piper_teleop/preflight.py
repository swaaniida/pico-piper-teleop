#!/usr/bin/env python3
"""Read-only dependency and hardware preflight for the compact pipeline."""

import argparse
import importlib
from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def check_import(name: str) -> None:
    module = importlib.import_module(name)
    version = getattr(module, "__version__", "installed")
    print(f"OK import {name}: {version}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--software-only", action="store_true")
    args = parser.parse_args()

    for module in ("numpy", "scipy", "placo", "pyrealsense2", "piper_sdk", "xrobotoolkit_sdk"):
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

    can = Path("/sys/class/net/can0")
    if not can.exists():
        raise RuntimeError("can0 is absent")
    result = subprocess.run(
        ["ip", "-details", "link", "show", "can0"], check=True,
        text=True, capture_output=True,
    )
    if "state UP" not in result.stdout or "bitrate 1000000" not in result.stdout:
        raise RuntimeError("can0 is not UP at 1 Mbps")
    print("OK can0: UP at 1 Mbps")

    import pyrealsense2 as rs
    devices = list(rs.context().query_devices())
    if not devices:
        raise RuntimeError("no RealSense device detected")
    print("OK RealSense:", ", ".join(device.get_info(rs.camera_info.serial_number) for device in devices))


if __name__ == "__main__":
    main()
