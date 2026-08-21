"""Isolated ROS 2 bridge for Tracer velocity commands and status feedback."""

from __future__ import annotations

import multiprocessing as mp
import queue
import time
from typing import Any


COMMAND_TIMEOUT_S = 0.2
PUBLISH_HZ = 50.0


def _bridge_worker(command, status, stop_event, ready_event, errors) -> None:
    node = None
    publisher = None
    try:
        import rclpy
        from geometry_msgs.msg import Twist
        from rclpy.signals import SignalHandlerOptions
        from tracer_msgs.msg import TracerStatus

        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node("pico_piper_tracer_bridge")
        publisher = node.create_publisher(Twist, "/cmd_vel", 10)

        def status_callback(message: TracerStatus) -> None:
            with status.get_lock():
                status[0] = float(message.vehicle_state)
                status[1] = float(message.error_code)
                status[2] = time.monotonic()

        node.create_subscription(TracerStatus, "/tracer_status", status_callback, 10)
        period = 1.0 / PUBLISH_HZ
        next_publish = time.monotonic()
        while not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.005)
            now = time.monotonic()
            subscriber_count = publisher.get_subscription_count()
            with status.get_lock():
                status[3] = float(subscriber_count)
                status_stamp = status[2]
            if subscriber_count > 0 and status_stamp > 0.0:
                ready_event.set()
            if now < next_publish:
                continue
            with command.get_lock():
                linear, angular, command_stamp = command[:]
            if now - command_stamp > COMMAND_TIMEOUT_S:
                linear = angular = 0.0
            message = Twist()
            message.linear.x = float(linear)
            message.angular.z = float(angular)
            publisher.publish(message)
            next_publish = max(next_publish + period, now)
    except BaseException as exc:
        errors.put(f"{type(exc).__name__}: {exc}")
    finally:
        if publisher is not None and node is not None:
            try:
                from geometry_msgs.msg import Twist
                import rclpy

                zero = Twist()
                for _ in range(10):
                    publisher.publish(zero)
                    rclpy.spin_once(node, timeout_sec=0.01)
            except Exception:
                pass
            try:
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass


class TracerRosBridge:
    """Parent-side handle for a spawned ROS 2 Tracer process."""

    def __init__(self) -> None:
        context = mp.get_context("spawn")
        now = time.monotonic()
        self._command = context.Array("d", [0.0, 0.0, now], lock=True)
        self._status = context.Array("d", [-1.0, -1.0, 0.0, 0.0], lock=True)
        self._stop = context.Event()
        self._ready = context.Event()
        self._errors = context.Queue()
        self._process = context.Process(
            target=_bridge_worker,
            args=(self._command, self._status, self._stop, self._ready, self._errors),
            name="tracer_ros_bridge",
        )

    def start(self, timeout_s: float = 3.0) -> None:
        self._process.start()
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if self._ready.wait(timeout=0.05):
                return
            error = self._get_error()
            if error is not None:
                self.close()
                raise RuntimeError(f"Tracer ROS bridge failed: {error}")
            if not self._process.is_alive():
                self.close()
                raise RuntimeError("Tracer ROS bridge exited before becoming ready")
        self.close()
        raise RuntimeError("Tracer ROS feedback or /cmd_vel subscriber is absent")

    def _get_error(self) -> str | None:
        try:
            return self._errors.get_nowait()
        except queue.Empty:
            return None

    def set_command(self, linear_mps: float, angular_rad_s: float) -> None:
        if not self._process.is_alive():
            raise RuntimeError(self._get_error() or "Tracer ROS bridge is not alive")
        with self._command.get_lock():
            self._command[:] = [float(linear_mps), float(angular_rad_s), time.monotonic()]

    def stop_motion(self) -> None:
        if self._process.is_alive():
            self.set_command(0.0, 0.0)

    def snapshot(self) -> dict[str, Any]:
        with self._status.get_lock():
            vehicle, error, stamp, subscribers = self._status[:]
        return {
            "vehicle": int(vehicle) if vehicle >= 0 else None,
            "error": int(error) if error >= 0 else None,
            "stamp": float(stamp),
            "cmd_vel_subscribers": int(subscribers),
            "bridge_alive": self._process.is_alive(),
        }

    def close(self) -> None:
        if not self._process.pid:
            return
        if self._process.is_alive():
            try:
                self.stop_motion()
                time.sleep(0.1)
            except Exception:
                pass
            self._stop.set()
            self._process.join(timeout=2.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)
