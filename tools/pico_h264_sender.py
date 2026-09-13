#!/usr/bin/env python3
"""Send the wrist camera to XRoboToolkit's TCP H.264 receiver."""

from __future__ import annotations

import argparse
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
import time
import threading

import cv2
import numpy as np


def create_global_aid_overlay(width: int, height: int) -> str:
    """Create a transparent upward-semicircle overlay for one PICO eye."""
    overlay = np.zeros((height, width, 4), dtype=np.uint8)
    center = (round(0.52 * width), round(0.89 * height))
    radius = round(0.16 * width)
    cv2.ellipse(overlay, center, (radius, radius), 0, 180, 360,
                (40, 220, 40, 70), -1, cv2.LINE_AA)
    cv2.ellipse(overlay, center, (radius, radius), 0, 180, 360,
                (40, 230, 40, 245), 5, cv2.LINE_AA)
    cv2.line(overlay, (center[0] - radius, center[1]),
             (center[0] + radius, center[1]), (40, 230, 40, 245), 5,
             cv2.LINE_AA)
    cv2.circle(overlay, center, 7, (255, 255, 255, 255), -1, cv2.LINE_AA)
    cv2.circle(overlay, center, 7, (40, 230, 40, 255), 2, cv2.LINE_AA)
    path = Path(__file__).resolve().parents[1] / ".runtime" / "pico_global_aid.png"
    path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(path), overlay):
        raise RuntimeError(f"failed to write global aid overlay: {path}")
    return str(path)


def listen_for_toggle(port: int, switch_requested: threading.Event) -> None:
    """Accept view-toggle datagrams sent directly by the PICO application."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as toggle_socket:
        toggle_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        toggle_socket.bind(("0.0.0.0", port))
        while True:
            payload, address = toggle_socket.recvfrom(64)
            if payload.strip().lower() == b"toggle":
                print(f"View toggle received from PICO {address[0]}", flush=True)
                switch_requested.set()


def ffmpeg_command(
    device: str, input_format: str, width: int, height: int, output_eye_width: int,
    output_height: int, fps: int, bitrate: str, keyint: int,
    global_aid_overlay: str = "",
) -> list[str]:
    # Duplicate the mono view side-by-side. XRoboToolkit then gives each eye the
    # same half, preserving scale without inventing stereo disparity.
    command = [
        "ffmpeg", "-hide_banner", "-loglevel", "warning",
        "-fflags", "nobuffer", "-flags", "low_delay",
        "-f", "v4l2", "-input_format", input_format,
        "-video_size", f"{width}x{height}", "-framerate", str(fps),
        "-i", device,
    ]
    if global_aid_overlay:
        command += ["-loop", "1", "-framerate", str(fps), "-i", global_aid_overlay]
        filter_graph = (
            f"[0:v]scale={output_eye_width}:{output_height}[base];"
            "[base][1:v]overlay=0:0:shortest=1[m];"
            "[m]split=2[l][r];[l][r]hstack=inputs=2[v]"
        )
    else:
        filter_graph = (
            f"[0:v]scale={output_eye_width}:{output_height}[m];"
            "[m]split=2[l][r];[l][r]hstack=inputs=2[v]"
        )
    command += [
        "-filter_complex", filter_graph,
        "-map", "[v]", "-an", "-c:v", "libx264", "-preset", "ultrafast",
        "-tune", "zerolatency", "-profile:v", "baseline", "-level", "5.1",
        "-x264-params", "aud=1:repeat-headers=1",
        "-pix_fmt", "yuv420p", "-b:v", bitrate, "-maxrate", bitrate,
        # A one-second GOP made the PICO MediaCodec surface flash at every IDR.
        # Keep periodic recovery frames, but make them infrequent and give VBV
        # enough room for the larger IDR access unit.
        "-bufsize", "2M", "-g", str(keyint), "-keyint_min", str(keyint),
        "-sc_threshold", "0", "-bf", "0", "-f", "h264", "pipe:1",
    ]
    return command


def nal_units(stream):
    """Split Annex-B output into complete NAL units, retaining start codes."""
    buffer = bytearray()
    while chunk := stream.read(65536):
        buffer.extend(chunk)
        while True:
            starts = []
            pos = 0
            while True:
                i3 = buffer.find(b"\x00\x00\x01", pos)
                i4 = buffer.find(b"\x00\x00\x00\x01", pos)
                found = [i for i in (i3, i4) if i >= 0]
                if not found:
                    break
                i = min(found)
                if not starts or starts[-1] != i:
                    starts.append(i)
                pos = i + (4 if buffer[i:i + 4] == b"\x00\x00\x00\x01" else 3)
                if len(starts) >= 2:
                    end = starts[1]
                    yield bytes(buffer[starts[0]:end])
                    del buffer[:end]
                    break
            else:
                continue
            if len(starts) < 2:
                break


def access_units(stream):
    """Group Annex-B NAL units into frames as expected by MediaCodec."""
    unit = bytearray()
    has_aud = False
    for nalu in nal_units(stream):
        start_len = 4 if nalu.startswith(b"\x00\x00\x00\x01") else 3
        nal_type = nalu[start_len] & 0x1F if len(nalu) > start_len else -1
        # Access Unit Delimiter begins the next encoded frame. SPS/PPS emitted
        # before the first AUD are retained with the first frame.
        if nal_type == 9 and has_aud:
            yield bytes(unit)
            unit.clear()
        if nal_type == 9:
            has_aud = True
        unit.extend(nalu)


def connect(host: str, port: int) -> socket.socket:
    while True:
        try:
            sock = socket.create_connection((host, port), timeout=2)
            sock.settimeout(None)
            return sock
        except OSError:
            print(f"Waiting for XRoboToolkit Listen at {host}:{port} ...", flush=True)
            time.sleep(1)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pico", default="192.168.0.229")
    parser.add_argument("--port", type=int, default=12345)
    parser.add_argument("--device", default="/dev/video4")
    parser.add_argument("--alternate-device", default="")
    parser.add_argument("--input-format", default="mjpeg")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--output-eye-width", type=int, default=960)
    parser.add_argument("--output-height", type=int, default=540)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--bitrate", default="4M")
    parser.add_argument("--keyframe-seconds", type=float, default=1.0)
    parser.add_argument("--toggle-port", type=int, default=12346)
    args = parser.parse_args()
    keyint = max(args.fps, round(args.fps * args.keyframe_seconds))
    devices = [args.device]
    if args.alternate_device:
        devices.append(args.alternate_device)
    switch_requested = threading.Event()
    signal.signal(signal.SIGUSR1, lambda _signum, _frame: switch_requested.set())
    threading.Thread(
        target=listen_for_toggle,
        args=(args.toggle_port, switch_requested),
        daemon=True,
    ).start()
    active_device = 0
    global_aid_overlay = create_global_aid_overlay(
        args.output_eye_width, args.output_height
    )

    while True:
        sock = connect(args.pico, args.port)
        try:
            while True:
                device = devices[active_device]
                switch_requested.clear()
                print(
                    f"Streaming {'LOCAL' if active_device == 0 else 'GLOBAL'} "
                    f"({device}) to PICO {args.pico}:{args.port}", flush=True
                )
                process = subprocess.Popen(
                    ffmpeg_command(
                        device, args.input_format, args.width, args.height,
                        args.output_eye_width, args.output_height, args.fps,
                        args.bitrate, keyint,
                        global_aid_overlay=global_aid_overlay
                        if active_device == 1 else "",
                    ),
                    stdout=subprocess.PIPE,
                )
                try:
                    assert process.stdout is not None
                    for frame in access_units(process.stdout):
                        sock.sendall(struct.pack(">I", len(frame)) + frame)
                        if switch_requested.is_set():
                            break
                finally:
                    process.terminate()
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        process.kill()
                if switch_requested.is_set() and len(devices) == 2:
                    active_device = 1 - active_device
                    continue
                raise ConnectionError("video encoder stopped")
        except (BrokenPipeError, ConnectionResetError, OSError):
            print("PICO video receiver disconnected; waiting to reconnect", file=sys.stderr)
        finally:
            sock.close()
        time.sleep(0.5)


if __name__ == "__main__":
    main()
