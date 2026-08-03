#!/usr/bin/env python3
"""PICO joystick teleoperation for a ROS 2 Tracer Mini base."""

import argparse
import signal
import socket
import struct
import threading
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.signals import SignalHandlerOptions
from tracer_msgs.msg import TracerStatus
import xrobotoolkit_sdk as xrt


CONTROL_HZ = 20.0
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


def stop_messages(pub, node, count: int = 10) -> None:
    if pub is None:
        return
    message = Twist()
    for _ in range(count):
        pub.publish(message)
        rclpy.spin_once(node, timeout_sec=0.01)


def send_hardware_reset(can_socket: socket.socket) -> None:
    """Send the official Tracer V2 'clear all errors' frame (0x441#00)."""
    frame = struct.pack("=IB3x8s", STATE_RESET_CAN_ID, 1, b"\x00".ljust(8, b"\x00"))
    can_socket.send(frame)


def enable_commanded_mode(can_socket: socket.socket) -> None:
    """Match ugv_sdk EnableCommandedMode(): 0x421, mode 0x01, DLC 8."""
    frame = struct.pack("=IB3x8s", CONTROL_MODE_CAN_ID, 8, b"\x01" + b"\x00" * 7)
    can_socket.send(frame)


def main() -> None:
    parser = argparse.ArgumentParser(description="PICO joystick -> Tracer Mini /cmd_vel")
    parser.add_argument("--live", action="store_true", help="publish velocity commands")
    parser.add_argument("--max-linear", type=float, default=DEFAULT_LINEAR_MPS)
    parser.add_argument("--max-angular", type=float, default=DEFAULT_ANGULAR_RAD_S)
    parser.add_argument("--can-interface", default="tracer_can")
    args = parser.parse_args()
    if args.max_linear <= 0 or args.max_angular <= 0:
        raise ValueError("speed limits must be positive")

    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())

    node = None
    pub = None
    can_socket = None
    if args.live:
        # Let this process send zero velocity before shutting ROS down.
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("pico_tracer_teleop")
        pub = node.create_publisher(Twist, "/cmd_vel", 10)
        can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        can_socket.bind((args.can_interface,))
        enable_commanded_mode(can_socket)
        tracer_state = {"vehicle": None, "error": None, "stamp": 0.0}

        def status_callback(message: TracerStatus) -> None:
            tracer_state["vehicle"] = int(message.vehicle_state)
            tracer_state["error"] = int(message.error_code)
            tracer_state["stamp"] = time.monotonic()

        node.create_subscription(TracerStatus, "/tracer_status", status_callback, 10)
        deadline = time.monotonic() + 3.0
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if pub.get_subscription_count() == 0:
            node.destroy_node()
            rclpy.shutdown()
            raise RuntimeError("no /cmd_vel subscriber")
        print("WARNING: LIVE Tracer motion. Left grip is deadman; B latches stop; X rearms.")
        for remaining in (3, 2, 1):
            print(f"Starting in {remaining}...", flush=True)
            time.sleep(1.0)

    xrt.init()
    print("READY", "LIVE" if args.live else "DRY-RUN")
    last_xr_ns = None
    last_xr_change = time.monotonic()
    last_print = 0.0
    stop_latched = False
    previous_b = False
    previous_x = False
    reset_sent_at = None
    try:
        while not stop.is_set():
            started = time.monotonic()
            xr_ns = int(xrt.get_time_stamp_ns())
            if xr_ns != last_xr_ns:
                last_xr_ns = xr_ns
                last_xr_change = started
            stale = started - last_xr_change > INPUT_TIMEOUT_S

            b_pressed = bool(xrt.get_B_button())
            x_pressed = bool(xrt.get_X_button())
            if b_pressed and not previous_b:
                print("B: emergency stop")
                break
            if bool(xrt.get_A_button()):
                print("A: normal stop")
                break

            left = xrt.get_left_axis()
            right = xrt.get_right_axis()
            grip = float(xrt.get_left_grip())
            sticks_neutral = abs(float(left[1])) <= DEADZONE and abs(float(right[0])) <= DEADZONE

            status_fresh = pub is None or (
                tracer_state["vehicle"] is not None
                and started - tracer_state["stamp"] <= INPUT_TIMEOUT_S
            )
            estop_active = pub is not None and status_fresh and tracer_state["vehicle"] == 1
            hardware_ok = pub is None or (
                status_fresh
                and tracer_state["vehicle"] == 0
                and tracer_state["error"] == 0
            )
            if estop_active:
                stop_latched = True

            if reset_sent_at is not None and tracer_state["stamp"] > reset_sent_at:
                if tracer_state["vehicle"] == 0 and tracer_state["error"] == 0:
                    enable_commanded_mode(can_socket)
                    stop_latched = False
                    reset_sent_at = None
                    print("Tracer E-stop/error cleared; teleop rearmed")
                elif started - reset_sent_at > 1.0:
                    reset_sent_at = None
                    print("Tracer E-stop/error remains active; zero velocity held", flush=True)

            if x_pressed and not previous_x:
                if not stop_latched:
                    print("X ignored: no bumper E-stop is latched")
                elif grip <= GRIP_THRESHOLD and sticks_neutral and not stale:
                    if can_socket is None:
                        print("X: hardware reset requested (dry-run)")
                    else:
                        stop_latched = True
                        stop_messages(pub, node, count=3)
                        send_hardware_reset(can_socket)
                        reset_sent_at = time.monotonic()
                        print("X: Tracer hardware E-stop/error reset requested")
                else:
                    print(
                        "X ignored: release grip, center sticks, and restore PICO input",
                        flush=True,
                    )

            enabled = grip > GRIP_THRESHOLD and not stale and not stop_latched and hardware_ok
            linear = axis(left[1]) * args.max_linear if enabled else 0.0
            angular = -axis(right[0]) * args.max_angular if enabled else 0.0

            if pub is not None:
                message = Twist()
                message.linear.x = linear
                message.angular.z = angular
                pub.publish(message)
                rclpy.spin_once(node, timeout_sec=0.0)

            if started - last_print >= 0.2:
                print(
                    f"grip={grip:.2f} enabled={enabled} stale={stale} "
                    f"latched={stop_latched} "
                    f"vehicle={tracer_state['vehicle'] if pub else 'dry'} "
                    f"error={tracer_state['error'] if pub else 'dry'} "
                    f"linear={linear:+.3f} angular={angular:+.3f}",
                    flush=True,
                )
                last_print = started
            previous_b = b_pressed
            previous_x = x_pressed
            delay = 1.0 / CONTROL_HZ - (time.monotonic() - started)
            if delay > 0:
                time.sleep(delay)
    finally:
        stop_messages(pub, node)
        xrt.close()
        if can_socket is not None:
            can_socket.close()
        if node is not None:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        print("STOPPED: zero velocity sent" if args.live else "STOPPED")


if __name__ == "__main__":
    main()
