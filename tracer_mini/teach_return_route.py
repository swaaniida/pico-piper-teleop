#!/usr/bin/env python3
"""Teach a relative Tracer route from the task finish pose back to start."""

import json
import math
from pathlib import Path
import signal
import socket
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.signals import SignalHandlerOptions
from tracer_msgs.msg import TracerStatus
import xrobotoolkit_sdk as xrt

from .pico_teleop import (
    CONTROL_HZ,
    DEADZONE,
    DEFAULT_ANGULAR_RAD_S,
    DEFAULT_LINEAR_MPS,
    GRIP_THRESHOLD,
    INPUT_TIMEOUT_S,
    axis,
    enable_commanded_mode,
    send_hardware_reset,
    stop_messages,
)


ROOT = Path(__file__).resolve().parents[1]
ROUTE_PATH = ROOT / "config" / "tracer_return_route.json"
CAN_NAME = "tracer_can"


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_odom(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main() -> None:
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("teach_tracer_return_route")
    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    state = {"odom": None, "odom_stamp": 0.0, "vehicle": None, "error": None, "status_stamp": 0.0}

    def odom_callback(message: Odometry) -> None:
        state["odom"] = message
        state["odom_stamp"] = time.monotonic()

    def status_callback(message: TracerStatus) -> None:
        state["vehicle"] = int(message.vehicle_state)
        state["error"] = int(message.error_code)
        state["status_stamp"] = time.monotonic()

    node.create_subscription(Odometry, "/odom", odom_callback, 20)
    node.create_subscription(TracerStatus, "/tracer_status", status_callback, 10)
    can_socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    can_socket.bind((CAN_NAME,))
    enable_commanded_mode(can_socket)

    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline and (
        pub.get_subscription_count() == 0 or state["odom"] is None or state["vehicle"] is None
    ):
        rclpy.spin_once(node, timeout_sec=0.05)
    if pub.get_subscription_count() == 0 or state["odom"] is None or state["vehicle"] is None:
        raise RuntimeError("Tracer /cmd_vel, /odom, or status is absent")
    if state["vehicle"] != 0 or state["error"] != 0:
        raise RuntimeError(f"Tracer is not ready: vehicle={state['vehicle']} error={state['error']}")

    first = state["odom"]
    x0 = float(first.pose.pose.position.x)
    y0 = float(first.pose.pose.position.y)
    yaw0 = yaw_from_odom(first)
    cy, sy = math.cos(yaw0), math.sin(yaw0)
    points = []

    def relative_point(message: Odometry) -> dict[str, float]:
        dx = float(message.pose.pose.position.x) - x0
        dy = float(message.pose.pose.position.y) - y0
        return {
            "x_m": cy * dx + sy * dy,
            "y_m": -sy * dx + cy * dy,
            "yaw_rad": wrap(yaw_from_odom(message) - yaw0),
            "timestamp_monotonic_ns": time.monotonic_ns(),
        }

    points.append(relative_point(first))
    xrt.init()
    print("WARNING: LIVE Tracer route teaching. Left grip is deadman; B cancels.")
    print("At the start pose, release grip, center sticks, then press X to save.")
    for remaining in (3, 2, 1):
        print(f"Starting in {remaining}...", flush=True)
        time.sleep(1.0)

    last_xr_ns = None
    last_xr_change = time.monotonic()
    previous_x = False
    previous_b = False
    stop_latched = False
    reset_sent_at = None
    saved = False
    last_print = 0.0
    try:
        while not stop.is_set():
            started = time.monotonic()
            rclpy.spin_once(node, timeout_sec=0.0)
            xr_ns = int(xrt.get_time_stamp_ns())
            if xr_ns != last_xr_ns:
                last_xr_ns = xr_ns
                last_xr_change = started
            stale = started - last_xr_change > INPUT_TIMEOUT_S
            left = xrt.get_left_axis()
            right = xrt.get_right_axis()
            grip = float(xrt.get_left_grip())
            b_pressed = bool(xrt.get_B_button())
            x_pressed = bool(xrt.get_X_button())
            sticks_neutral = abs(float(left[1])) <= DEADZONE and abs(float(right[0])) <= DEADZONE
            status_fresh = started - state["status_stamp"] <= INPUT_TIMEOUT_S
            odom_fresh = started - state["odom_stamp"] <= INPUT_TIMEOUT_S
            if status_fresh and state["vehicle"] == 1:
                stop_latched = True
            if b_pressed and not previous_b:
                print("B: route teaching cancelled")
                break

            if reset_sent_at is not None and state["status_stamp"] > reset_sent_at:
                if state["vehicle"] == 0 and state["error"] == 0:
                    enable_commanded_mode(can_socket)
                    stop_latched = False
                    reset_sent_at = None
                    print("Tracer E-stop cleared")
                elif started - reset_sent_at > 1.0:
                    reset_sent_at = None

            if x_pressed and not previous_x:
                if stop_latched:
                    if grip <= GRIP_THRESHOLD and sticks_neutral and not stale:
                        stop_messages(pub, node, count=3)
                        send_hardware_reset(can_socket)
                        reset_sent_at = time.monotonic()
                        print("X: Tracer E-stop reset requested")
                elif grip <= GRIP_THRESHOLD and sticks_neutral and not stale and odom_fresh:
                    final = relative_point(state["odom"])
                    if points[-1] != final:
                        points.append(final)
                    ROUTE_PATH.parent.mkdir(parents=True, exist_ok=True)
                    ROUTE_PATH.write_text(json.dumps({
                        "format": "tracer_relative_odom_route_v1",
                        "created_wall_ns": time.time_ns(),
                        "control_hz": CONTROL_HZ,
                        "max_linear_mps": DEFAULT_LINEAR_MPS,
                        "max_angular_rad_s": DEFAULT_ANGULAR_RAD_S,
                        "waypoints": points,
                    }, indent=2), encoding="utf-8")
                    saved = True
                    print(f"X: return route saved: {ROUTE_PATH} ({len(points)} points)")
                    break
                else:
                    print("X ignored: release left grip, center sticks, and wait for fresh input/odom")

            ready = (
                not stale and odom_fresh and status_fresh and not stop_latched
                and state["vehicle"] == 0 and state["error"] == 0
            )
            enabled = ready and grip > GRIP_THRESHOLD
            linear = axis(left[1]) * DEFAULT_LINEAR_MPS if enabled else 0.0
            angular = -axis(right[0]) * DEFAULT_ANGULAR_RAD_S if enabled else 0.0
            message = Twist()
            message.linear.x = linear
            message.angular.z = angular
            pub.publish(message)

            if started - last_print >= 0.25:
                print(
                    f"grip={grip:.2f} enabled={enabled} stale={stale} "
                    f"odom_fresh={odom_fresh} status_fresh={status_fresh} "
                    f"vehicle={state['vehicle']} error={state['error']} "
                    f"linear={linear:+.3f} angular={angular:+.3f}",
                    flush=True,
                )
                last_print = started

            point = relative_point(state["odom"])
            previous = points[-1]
            if (
                math.hypot(point["x_m"] - previous["x_m"], point["y_m"] - previous["y_m"]) >= 0.02
                or abs(wrap(point["yaw_rad"] - previous["yaw_rad"])) >= math.radians(2.0)
            ):
                points.append(point)
            previous_x = x_pressed
            previous_b = b_pressed
            delay = 1.0 / CONTROL_HZ - (time.monotonic() - started)
            if delay > 0:
                time.sleep(delay)
    finally:
        stop_messages(pub, node)
        xrt.close()
        can_socket.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("STOPPED: zero velocity sent;", "route saved" if saved else "route not saved")


if __name__ == "__main__":
    main()
