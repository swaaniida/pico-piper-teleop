#!/usr/bin/env python3
"""Asynchronous RGB recorder for the wrist-mounted InnoMaker U20CAM."""

from __future__ import annotations

import json
from pathlib import Path
import queue
import threading
import time
from typing import Any

import cv2


INNOMAKER_MODEL = "Innomaker-U20CAM-1080p-S1"
INNOMAKER_SERIAL = "SN0001"


def find_innomaker_device() -> str:
    by_id = Path("/dev/v4l/by-id")
    if by_id.is_dir():
        matches = sorted(by_id.glob("*Innomaker*U20CAM*SN0001-video-index0"))
        if not matches:
            matches = sorted(by_id.glob("*Innomaker*U20CAM*-video-index0"))
        if matches:
            return str(matches[0])
    for name_file in sorted(Path("/sys/class/video4linux").glob("video*/name")):
        if "Innomaker-U20CAM-1080p-S1" in name_file.read_text(encoding="utf-8").strip():
            return f"/dev/{name_file.parent.name}"
    raise RuntimeError("InnoMaker U20CAM wrist camera is absent")


class WristUvcCamera:
    def __init__(
        self,
        device: str = "",
        width: int = 640,
        height: int = 480,
        fps: int = 30,
        jpeg_quality: int = 90,
    ):
        self.device = device
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self._capture = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._write_lock = threading.Lock()
        self._thread = None
        self._writer_thread = None
        # Buffer compressed frames, not raw BGR images, so a long USB stall can
        # be absorbed without consuming hundreds of megabytes of RAM.
        self._write_queue = queue.Queue(maxsize=max(30, self.fps * 10))
        self._latest = None
        self._latest_frame = None
        self._error = None
        self._episode = None
        self._recording = False
        self._frame_index = 0
        self._metadata = None

    def start(self, episode: Path) -> None:
        self._episode = episode
        color_dir = episode / "camera" / "color"
        color_dir.mkdir(parents=True, exist_ok=True)
        self.device = self.device or find_innomaker_device()
        capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not capture.isOpened():
            capture.release()
            raise RuntimeError(f"cannot open wrist camera: {self.device}")
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        self._capture = capture

        frame = None
        for _ in range(max(5, self.fps)):
            ok, frame = capture.read()
            if not ok:
                self.stop()
                raise RuntimeError(f"wrist camera produced no frame: {self.device}")
        actual_height, actual_width = frame.shape[:2]
        metadata = {
            "model": INNOMAKER_MODEL,
            "serial": INNOMAKER_SERIAL,
            "device": self.device,
            "width": int(actual_width),
            "height": int(actual_height),
            "requested_fps": self.fps,
            "reported_fps": float(capture.get(cv2.CAP_PROP_FPS)),
            "color_format": "bgr8_jpeg",
            "depth_available": False,
            "jpeg_quality": self.jpeg_quality,
        }
        self._metadata = metadata
        self._latest = None
        self._latest_frame = None
        self._error = None
        self.start_recording(episode)
        self._stop.clear()
        self._writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
        self._writer_thread.start()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()
        print(
            f"Wrist camera connected: InnoMaker U20CAM {actual_width}x{actual_height} "
            f"@ {self.fps}Hz ({self.device})"
        )

    def _capture_loop(self) -> None:
        try:
            while not self._stop.is_set():
                ok, frame = self._capture.read()
                captured_ns = time.monotonic_ns()
                captured_wall_ns = time.time_ns()
                if not ok:
                    raise RuntimeError("V4L2 frame read failed")
                with self._lock:
                    self._latest_frame = frame.copy()
                with self._write_lock:
                    with self._lock:
                        recording = self._recording
                        episode = self._episode
                        frame_index = self._frame_index
                        if recording:
                            self._frame_index += 1
                    if not recording or episode is None:
                        continue
                    stem = f"{frame_index:010d}"
                    color_rel = Path("camera") / "color" / f"{stem}.jpg"
                    ok, encoded = cv2.imencode(
                        ".jpg", frame,
                        [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
                    )
                    if not ok:
                        raise RuntimeError("failed to encode wrist camera frame")
                    record = {
                        "frame_number": frame_index,
                        "timestamp_monotonic_ns": captured_ns,
                        "timestamp_wall_ns": captured_wall_ns,
                        "color_path": str(color_rel),
                    }
                    with self._lock:
                        self._latest = record
                    try:
                        self._write_queue.put_nowait(
                            (episode / color_rel, encoded.tobytes())
                        )
                    except queue.Full as exc:
                        raise RuntimeError(
                            "wrist camera USB write queue exceeded 10 seconds"
                        ) from exc
        except Exception as exc:
            if not self._stop.is_set():
                with self._lock:
                    self._error = exc

    def _writer_loop(self) -> None:
        while True:
            item = self._write_queue.get()
            try:
                if item is None:
                    return
                path, payload = item
                try:
                    with path.open("wb") as stream:
                        stream.write(payload)
                except Exception as exc:
                    with self._lock:
                        if self._error is None:
                            self._error = RuntimeError(
                                f"failed to write wrist camera frame: {exc}"
                            )
            finally:
                self._write_queue.task_done()

    def latest(self) -> dict[str, Any] | None:
        with self._lock:
            if self._error is not None:
                raise RuntimeError(f"wrist camera capture failed: {self._error}")
            return dict(self._latest) if self._latest is not None else None

    def latest_frame(self):
        with self._lock:
            if self._error is not None:
                raise RuntimeError(f"wrist camera capture failed: {self._error}")
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def start_recording(self, episode: Path) -> None:
        color_dir = episode / "camera" / "color"
        color_dir.mkdir(parents=True, exist_ok=True)
        if self._metadata is not None:
            (episode / "camera" / "metadata.json").write_text(
                json.dumps(self._metadata, indent=2), encoding="utf-8"
            )
        with self._write_lock:
            with self._lock:
                self._episode = episode
                self._frame_index = 0
                self._latest = None
                self._recording = True

    def pause_recording(self, wait_for_writes: bool = True) -> None:
        with self._write_lock:
            with self._lock:
                self._recording = False
                self._episode = None
        if wait_for_writes:
            self._write_queue.join()

    def stop(self) -> None:
        self.pause_recording()
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._writer_thread is not None:
            self._write_queue.put(None)
            self._writer_thread.join(timeout=2.0)
            self._writer_thread = None
        if self._capture is not None:
            self._capture.release()
            self._capture = None
        with self._lock:
            self._latest_frame = None
        print("Wrist camera disconnected")
