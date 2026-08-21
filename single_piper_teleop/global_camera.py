#!/usr/bin/env python3
"""Asynchronous RGB recorder for the fixed Logitech global camera."""

import json
from pathlib import Path
import threading
import time
from typing import Any

import cv2


class GlobalWebcam:
    def __init__(
        self,
        device: str = "",
        width: int = 1280,
        height: int = 720,
        fps: int = 15,
        jpeg_quality: int = 85,
    ):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self._capture = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = None
        self._latest = None
        self._error = None
        self._episode = None

    @staticmethod
    def _auto_device() -> str:
        by_id = Path("/dev/v4l/by-id")
        if by_id.is_dir():
            matches = sorted(by_id.glob("*HD_Pro_Webcam_C920*-video-index0"))
            if matches:
                return str(matches[0])
        for name_file in sorted(Path("/sys/class/video4linux").glob("video*/name")):
            if "HD Pro Webcam C920" in name_file.read_text(encoding="utf-8").strip():
                return f"/dev/{name_file.parent.name}"
        raise RuntimeError("Logitech HD Pro Webcam C920 is absent")

    def start(self, episode: Path) -> None:
        self._episode = episode
        color_dir = episode / "global_camera" / "color"
        color_dir.mkdir(parents=True, exist_ok=True)
        self.device = self.device or self._auto_device()
        capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"cannot open global camera: {self.device}")
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        self._capture = capture

        # Warm auto-exposure and verify that this V4L2 node delivers images.
        frame = None
        for _ in range(max(5, self.fps)):
            ok, frame = capture.read()
            if not ok:
                self.stop()
                raise RuntimeError(f"global camera produced no frame: {self.device}")
        actual_height, actual_width = frame.shape[:2]
        metadata = {
            "model": "Logitech HD Pro Webcam C920",
            "device": self.device,
            "width": int(actual_width),
            "height": int(actual_height),
            "requested_fps": self.fps,
            "reported_fps": float(capture.get(cv2.CAP_PROP_FPS)),
            "color_format": "bgr8_jpeg",
            "jpeg_quality": self.jpeg_quality,
        }
        (episode / "global_camera" / "metadata.json").write_text(
            json.dumps(metadata, indent=2), encoding="utf-8"
        )
        self._latest = None
        self._error = None
        self._stop.clear()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        print(
            f"Global camera connected: C920 {actual_width}x{actual_height} "
            f"@ {self.fps}Hz ({self.device})"
        )

    def _capture_loop(self) -> None:
        frame_index = 0
        try:
            while not self._stop.is_set():
                ok, frame = self._capture.read()
                captured_ns = time.monotonic_ns()
                captured_wall_ns = time.time_ns()
                if not ok:
                    raise RuntimeError("V4L2 frame read failed")
                stem = f"{frame_index:010d}"
                color_rel = Path("global_camera") / "color" / f"{stem}.jpg"
                if not cv2.imwrite(
                    str(self._episode / color_rel),
                    frame,
                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
                ):
                    raise RuntimeError("failed to write global camera frame")
                record = {
                    "frame_number": frame_index,
                    "timestamp_monotonic_ns": captured_ns,
                    "timestamp_wall_ns": captured_wall_ns,
                    "color_path": str(color_rel),
                }
                with self._lock:
                    self._latest = record
                frame_index += 1
        except Exception as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._error = exc

    def latest(self) -> dict[str, Any] | None:
        with self._lock:
            if self._error is not None:
                raise RuntimeError(f"global camera capture failed: {self._error}")
            return dict(self._latest) if self._latest is not None else None

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        print("Global camera disconnected")
