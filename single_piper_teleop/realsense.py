#!/usr/bin/env python3
"""Compact asynchronous RGB-D recorder for the wrist RealSense D415."""

import json
from pathlib import Path
import threading
import time
from typing import Any

import cv2
import numpy as np
import pyrealsense2 as rs


class WristRealSense:
    def __init__(self, serial: str = "", width: int = 640, height: int = 480, fps: int = 30):
        self.serial = serial
        self.width = width
        self.height = height
        self.fps = fps
        self._pipeline = rs.pipeline()
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = None
        self._latest = None
        self._error = None
        self._episode = None

    def start(self, episode: Path) -> None:
        self._episode = episode
        (episode / "camera" / "color").mkdir(parents=True, exist_ok=True)
        (episode / "camera" / "depth").mkdir(parents=True, exist_ok=True)
        config = rs.config()
        if self.serial:
            config.enable_device(self.serial)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        profile = self._pipeline.start(config)
        device = profile.get_device()
        actual_serial = device.get_info(rs.camera_info.serial_number)
        self.serial = actual_serial
        depth_scale = float(device.first_depth_sensor().get_depth_scale())
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        depth_profile = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        metadata = {
            "model": device.get_info(rs.camera_info.name),
            "serial": actual_serial,
            "width": self.width,
            "height": self.height,
            "fps": self.fps,
            "color_format": "bgr8_jpeg",
            "depth_format": "z16_png",
            "depth_scale_m_per_unit": depth_scale,
            "color_intrinsics": self._intrinsics(color_profile.get_intrinsics()),
            "depth_intrinsics": self._intrinsics(depth_profile.get_intrinsics()),
        }
        (episode / "camera" / "metadata.json").write_text(
            json.dumps(metadata, indent=2), encoding="utf-8"
        )
        # Warm auto-exposure before recording.
        for _ in range(30):
            self._pipeline.wait_for_frames(5000)
        self._stop.clear()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        print(f"Wrist RealSense connected: {metadata['model']} {actual_serial}")

    @staticmethod
    def _intrinsics(value: Any) -> dict[str, Any]:
        return {
            "width": int(value.width), "height": int(value.height),
            "fx": float(value.fx), "fy": float(value.fy),
            "ppx": float(value.ppx), "ppy": float(value.ppy),
            "model": str(value.model), "coeffs": list(value.coeffs),
        }

    def _capture_loop(self) -> None:
        align = rs.align(rs.stream.color)
        try:
            while not self._stop.is_set():
                frames = align.process(self._pipeline.wait_for_frames(1000))
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    continue
                captured_ns = time.monotonic_ns()
                color = np.asanyarray(color_frame.get_data())
                depth = np.asanyarray(depth_frame.get_data())
                frame_number = int(color_frame.get_frame_number())
                stem = f"{frame_number:010d}"
                color_rel = Path("camera") / "color" / f"{stem}.jpg"
                depth_rel = Path("camera") / "depth" / f"{stem}.png"
                if not cv2.imwrite(str(self._episode / color_rel), color, [cv2.IMWRITE_JPEG_QUALITY, 90]):
                    raise RuntimeError("failed to write RealSense color frame")
                if not cv2.imwrite(str(self._episode / depth_rel), depth, [cv2.IMWRITE_PNG_COMPRESSION, 0]):
                    raise RuntimeError("failed to write RealSense depth frame")
                record = {
                    "frame_number": frame_number,
                    "depth_frame_number": int(depth_frame.get_frame_number()),
                    "camera_timestamp_ms": float(color_frame.get_timestamp()),
                    "depth_timestamp_ms": float(depth_frame.get_timestamp()),
                    "timestamp_monotonic_ns": captured_ns,
                    "color_path": str(color_rel),
                    "depth_path": str(depth_rel),
                }
                with self._lock:
                    self._latest = record
        except Exception as exc:
            with self._lock:
                self._error = exc

    def latest(self) -> dict[str, Any] | None:
        with self._lock:
            if self._error is not None:
                raise RuntimeError(f"wrist RealSense capture failed: {self._error}")
            return dict(self._latest) if self._latest is not None else None

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        self._pipeline.stop()
        print("Wrist RealSense disconnected")
