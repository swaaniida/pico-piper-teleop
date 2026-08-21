"""ROS-independent Tracer input shaping and SocketCAN control helpers."""

from __future__ import annotations

import socket
import struct


DEADZONE = 0.15
GRIP_THRESHOLD = 0.9
INPUT_TIMEOUT_S = 0.2
DEFAULT_LINEAR_MPS = 0.15
DEFAULT_ANGULAR_RAD_S = 0.40
STATE_RESET_CAN_ID = 0x441
CONTROL_MODE_CAN_ID = 0x421


def axis(value: float) -> float:
    value = max(-1.0, min(1.0, float(value)))
    if abs(value) <= DEADZONE:
        return 0.0
    return (abs(value) - DEADZONE) / (1.0 - DEADZONE) * (1.0 if value > 0 else -1.0)


def send_hardware_reset(can_socket: socket.socket) -> None:
    """Send the official Tracer V2 'clear all errors' frame (0x441#00)."""
    frame = struct.pack("=IB3x8s", STATE_RESET_CAN_ID, 1, b"\x00".ljust(8, b"\x00"))
    can_socket.send(frame)


def enable_commanded_mode(can_socket: socket.socket) -> None:
    """Match ugv_sdk EnableCommandedMode(): 0x421, mode 0x01, DLC 8."""
    frame = struct.pack("=IB3x8s", CONTROL_MODE_CAN_ID, 8, b"\x01" + b"\x00" * 7)
    can_socket.send(frame)
