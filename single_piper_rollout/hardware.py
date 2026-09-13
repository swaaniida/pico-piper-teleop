"""Read-only hardware observation adapter used before any live rollout."""

from __future__ import annotations

import time
from typing import Any

import cv2
import numpy as np


class ReadOnlyTracer:
    """ROS subscriptions only: deliberately creates no /cmd_vel publisher."""

    def __init__(self) -> None:
        self.node = None
        self.status: dict[str, Any] = {
            "vehicle": None, "error": None, "stamp": 0.0, "odometry": None,
        }

    def start(self) -> None:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.signals import SignalHandlerOptions
        from tracer_msgs.msg import TracerStatus
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node("piper_rollout_read_only")

        def status_callback(message: TracerStatus) -> None:
            self.status.update(
                vehicle=int(message.vehicle_state),
                error=int(message.error_code), stamp=time.monotonic(),
            )

        def odom_callback(message: Odometry) -> None:
            self.status["odometry"] = {
                "linear_mps": float(message.twist.twist.linear.x),
                "angular_rad_s": float(message.twist.twist.angular.z),
                "stamp": time.monotonic(),
            }

        self.node.create_subscription(TracerStatus, "/tracer_status", status_callback, 10)
        self.node.create_subscription(Odometry, "/odom", odom_callback, 10)

    def snapshot(self) -> dict[str, Any]:
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=0.005)
        return self.status

    def close(self) -> None:
        if self.node is None:
            return
        import rclpy
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        self.node = None


class CameraPair:
    def __init__(self, local_device: str, global_device: str) -> None:
        self.devices = (local_device, global_device)
        self.captures: list[Any] = []
        self.last_frame_time = 0.0

    def start(self) -> None:
        for device in self.devices:
            capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not capture.isOpened():
                self.close()
                raise RuntimeError(f"cannot open camera: {device}")
            self.captures.append(capture)

    def read_rgb(self) -> tuple[np.ndarray, np.ndarray]:
        frames = []
        for device, capture in zip(self.devices, self.captures):
            ok, frame = capture.read()
            if not ok:
                raise RuntimeError(f"camera read failed: {device}")
            if frame.shape[:2] != (480, 640):
                raise RuntimeError(f"camera contract mismatch on {device}: {frame.shape}")
            frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        self.last_frame_time = time.monotonic()
        return frames[0], frames[1]

    def close(self) -> None:
        for capture in self.captures:
            capture.release()
        self.captures.clear()


class ReadOnlyRobot:
    def __init__(self, can_name: str = "piper_can") -> None:
        self.can_name = can_name
        self.piper = None
        self.tracer = None

    def start(self) -> None:
        from piper_sdk import C_PiperInterface_V2
        self.piper = C_PiperInterface_V2(
            can_name=self.can_name, judge_flag=True, can_auto_init=True,
            dh_is_offset=0, start_sdk_joint_limit=False,
            start_sdk_gripper_limit=False,
        )
        self.piper.ConnectPort(piper_init=False)
        self.tracer = ReadOnlyTracer()
        self.tracer.start()
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            try:
                self.state()
                return
            except RuntimeError:
                time.sleep(0.02)
        raise RuntimeError("PiPER or Tracer read-only feedback did not become ready")

    def state(self) -> tuple[np.ndarray, dict[str, Any]]:
        message = self.piper.GetArmJointMsgs()
        joint = message.joint_state
        degrees = np.array([
            joint.joint_1, joint.joint_2, joint.joint_3,
            joint.joint_4, joint.joint_5, joint.joint_6,
        ], dtype=np.float64) / 1000.0
        gripper = self.piper.GetArmGripperMsgs().gripper_state
        tracer = dict(self.tracer.snapshot())
        odometry = tracer.get("odometry")
        if message.Hz <= 0 or odometry is None:
            raise RuntimeError("PiPER or Tracer feedback is absent")
        now = time.monotonic()
        if now - float(odometry["stamp"]) > 0.2:
            raise RuntimeError("Tracer odometry is stale")
        state = np.concatenate([
            np.deg2rad(degrees),
            [float(gripper.grippers_angle) / 1000.0],
            [float(odometry["linear_mps"]), float(odometry["angular_rad_s"])],
        ])
        return state, tracer

    def close(self) -> None:
        if self.tracer is not None:
            self.tracer.close()
            self.tracer = None
        if self.piper is not None:
            self.piper.DisconnectPort()
            self.piper = None
