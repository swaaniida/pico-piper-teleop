#!/usr/bin/env python3
"""Compact right-PICO -> single PiPER teleoperation.

Functional single-arm adaptation of manipulation_pipeline-main:
XRoboToolkit -> full 6D Placo IK -> MOVE_J/JointCtrl + trigger gripper.
No LeRobot, bimanual wrapper, joint-limit rewrite, or reset. The session uses
the reference ZIP's start -> init sequence and supports multiple episodes.
"""

import argparse
import json
import math
import os
from pathlib import Path
import random
import signal
import select
import shutil
import socket
import struct
import sys
import threading
import time
from typing import Any

import numpy as np
import cv2
import placo
from scipy.spatial.transform import Rotation
import xrobotoolkit_sdk as xrt
from piper_sdk import C_PiperInterface_V2
from .async_jsonl import AsyncJsonlWriter
from .wrist_camera import WristUvcCamera
from .global_camera import GlobalWebcam
from tracer_mini.control import (
    DEADZONE as TRACER_DEADZONE,
    GRIP_THRESHOLD as TRACER_GRIP_THRESHOLD,
    INPUT_TIMEOUT_S as TRACER_INPUT_TIMEOUT_S,
    axis as tracer_axis,
    enable_commanded_mode,
    send_hardware_reset,
)
from tracer_mini.ros_bridge import TracerRosBridge


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "assets" / "piper_kinematics.urdf"
EPISODES = Path(
    os.environ.get("PIPER_EPISODES_DIR", str(ROOT / "data" / "episodes"))
).expanduser().resolve()
CAN_NAME = "piper_can"
CONTROL_HZ = 50.0
DT = 1.0 / CONTROL_HZ

# Same interaction values as the ZIP single-arm path.
POSITION_SCALE = 1.0
ROTATION_SCALE = 1.0
RIGHT_ELBOW_TRACKER_SERIAL = "PC2310MLKB041978G"
ELBOW_TRACKER_LINK = "link3"
ELBOW_TRACKER_WEIGHT = 0.05
GRIP_THRESHOLD = 0.8
GRIP_RELEASE_THRESHOLD = 0.5
GRIPPER_DEADZONE = 0.05
GRIPPER_OPEN_MM = 70.0
GRIPPER_CLOSED_MM = 0.0
MOVE_SPEED_PERCENT = 100
INIT_MOVE_SPEED_PERCENT = 50
AUTO_INIT_SPEED_PERCENT = 50
AUTO_INIT_JOINT_SPEED_DEG_S = 25.0
CUP_APPROACH_JOINT_SPEED_DEG_S = 20.0
CUP_CLEARANCE_M = 0.08
# The reference ZIP uses J3=+1.60 deg, but this arm clamps J3 at its 0 deg
# boundary. Use the measured reachable equivalent so arrival can be confirmed.
JOINTS_START_DEG = np.array([-0.51, 0.49, 0.0, 3.96, 4.83, -8.33])
JOINTS_INIT_DEG = np.array([-1.0, 23.84, -60.0, 0.0, 50.65, -1.5])
FEEDBACK_STALE_S = 0.10
MIN_EPISODE_DURATION_S = 2.0
TRACER_CAN_NAME = "tracer_can"
PIPER_SDK_FEEDBACK_IDS = tuple(range(0x2A1, 0x2A9)) + tuple(range(0x261, 0x267))
PIPER_RAW_FEEDBACK_IDS = frozenset(PIPER_SDK_FEEDBACK_IDS)
TRACER_LINEAR_MPS = 0.15
TRACER_ANGULAR_RAD_S = 0.40
R_HEADSET_WORLD = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))
PREVIEW_WINDOW = "PiPER Teleop Cameras"
PICO_VIDEO_PID_FILE = ROOT / ".runtime" / "pico_h264_sender.pid"
FLOOR_INIT_CONFIG = ROOT / "config" / "floor_pick_init.json"
CUP_DROP_CONFIG = ROOT / "config" / "cup_drop_pose.json"
# Normalized global-camera target region. This is display-only and is not
# burned into the recorded training images.
GLOBAL_AID_CENTER_XY = (0.52, .89)
GLOBAL_AID_RADIUS_X = 0.16
GLOBAL_CAMERA_HEIGHT_M = 0.355
GLOBAL_CAMERA_DOWN_DEG = 60.0
GLOBAL_CAMERA_HFOV_DEG = 106.0
_preview_window_initialized = False


def build_ball_position_plan() -> list[tuple[int, int]]:
    """Build the deterministic, mixed 100-episode lateral/distance plan."""
    lateral = (-20, -10, 0, 10, 20)
    # With the global camera at 35.5 cm and pitched 60 deg downward, these
    # distances remain inside its 1280x720 / 106-deg-HFOV image, including
    # the +/-20 cm lateral extremes. D70 at the center was reachable in
    # episode 2; the former R30/D75 corner in episode 3 was not. Keep that
    # verified distance while pulling all combined extremes inward.
    distance = (45, 50, 60, 65, 70)
    counts = (
        (2, 3, 4, 3, 2),
        (3, 5, 6, 5, 3),
        (4, 6, 8, 6, 4),
        (3, 5, 6, 5, 3),
        (2, 3, 4, 3, 2),
    )
    remaining = {
        (lateral[x], distance[y]): counts[y][x]
        for y in range(5) for x in range(5)
    }
    rng = random.Random(20260909)
    plan: list[tuple[int, int]] = []
    while remaining:
        candidates = list(remaining)
        if plan and len(candidates) > 1:
            candidates = [key for key in candidates if key != plan[-1]]
        if plan:
            strongly_mixed = [
                key for key in candidates
                if key[0] != plan[-1][0] and key[1] != plan[-1][1]
            ]
            if strongly_mixed:
                candidates = strongly_mixed
        weights = [remaining[key] for key in candidates]
        selected = rng.choices(candidates, weights=weights, k=1)[0]
        plan.append(selected)
        remaining[selected] -= 1
        if remaining[selected] == 0:
            del remaining[selected]
    if len(plan) >= 2 and plan[-1] == plan[-2]:
        for index in range(1, len(plan) - 2):
            replacement = plan[index]
            if (
                replacement != plan[-2]
                and plan[-1] != plan[index - 1]
                and plan[-1] != plan[index + 1]
            ):
                plan[index], plan[-1] = plan[-1], replacement
                break
    return plan


BALL_POSITION_PLAN = build_ball_position_plan()


def ball_target_label(index: int) -> str:
    if index >= len(BALL_POSITION_PLAN):
        return "100/100 DONE"
    lateral_cm, distance_cm = BALL_POSITION_PLAN[index]
    lateral = "C" if lateral_cm == 0 else f"{'L' if lateral_cm < 0 else 'R'}{abs(lateral_cm)}"
    return f"{index + 1:03d}/100  {lateral}  D{distance_cm}"


def completed_planned_episodes() -> int:
    completed = 0
    for metadata_path in EPISODES.glob("*/metadata.json"):
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            continue
        if metadata.get("status") == "complete" and int(metadata.get("workflow_version", 1)) >= 3:
            completed += 1
    return min(completed, len(BALL_POSITION_PLAN))


def load_floor_init() -> np.ndarray:
    if not FLOOR_INIT_CONFIG.exists():
        return JOINTS_INIT_DEG.copy()
    payload = json.loads(FLOOR_INIT_CONFIG.read_text(encoding="utf-8"))
    joints = np.asarray(payload["joint_deg"], dtype=float)
    if joints.shape != (6,) or not np.all(np.isfinite(joints)):
        raise RuntimeError(f"invalid floor init config: {FLOOR_INIT_CONFIG}")
    return joints


def load_cup_drop_pose() -> tuple[np.ndarray, np.ndarray]:
    if not CUP_DROP_CONFIG.exists():
        raise RuntimeError(f"cup drop pose is not calibrated: run ./calibrate-drop")
    payload = json.loads(CUP_DROP_CONFIG.read_text(encoding="utf-8"))
    joints = np.asarray(payload["joint_deg"], dtype=float)
    transform = np.asarray(payload["link6_transform"], dtype=float)
    if joints.shape != (6,) or not np.all(np.isfinite(joints)):
        raise RuntimeError(f"invalid cup drop joints: {CUP_DROP_CONFIG}")
    if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
        raise RuntimeError(f"invalid cup drop EE transform: {CUP_DROP_CONFIG}")
    return joints, transform


def solve_cup_clearance_pose() -> np.ndarray:
    """Find a vertical pre-drop waypoint while retaining the saved EE attitude."""
    drop_joints, target = load_cup_drop_pose()
    target = target.copy()
    target[2, 3] += CUP_CLEARANCE_M

    robot = placo.RobotWrapper(str(URDF))
    solver = placo.KinematicsSolver(robot)
    solver.dt = DT
    solver.mask_fbase(True)
    robot.state.q[:7] = np.array([0, 0, 0, 0, 0, 0, 1], dtype=float)
    robot.state.q[7:13] = np.radians(drop_joints)
    robot.update_kinematics()
    task = solver.add_frame_task("link6", target)
    task.configure("cup_clearance", "soft", 1.0)
    regularization = solver.add_joints_task()
    regularization.set_joints({
        name: robot.state.q[7 + index]
        for index, name in enumerate(JOINT_NAMES)
    })
    regularization.configure("cup_clearance_regularization", "soft", 1e-4)
    for _ in range(500):
        solver.solve(True)
        robot.update_kinematics()
    actual = robot.get_T_world_frame("link6")
    position_error = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]))
    rotation_error = float(Rotation.from_matrix(
        actual[:3, :3].T @ target[:3, :3]
    ).magnitude())
    if position_error > 0.005 or rotation_error > math.radians(2.0):
        raise RuntimeError(
            "cup clearance IK is not safely reachable: "
            f"position_error={position_error:.4f}m, rotation_error={math.degrees(rotation_error):.2f}deg"
        )
    return np.degrees(robot.state.q[7:13].copy())


def draw_global_aid(frame: np.ndarray) -> np.ndarray:
    """Draw the display-only locomotion target as an upward semicircle."""
    result = frame.copy()
    height, width = result.shape[:2]
    center = (
        round(GLOBAL_AID_CENTER_XY[0] * width),
        round(GLOBAL_AID_CENTER_XY[1] * height),
    )
    radius = round(GLOBAL_AID_RADIUS_X * width)
    overlay = result.copy()
    cv2.ellipse(overlay, center, (radius, radius), 0, 180, 360,
                (40, 220, 40), -1, cv2.LINE_AA)
    cv2.addWeighted(overlay, 0.18, result, 0.82, 0, result)
    cv2.ellipse(result, center, (radius, radius), 0, 180, 360,
                (40, 230, 40), 4, cv2.LINE_AA)
    cv2.line(result, (center[0] - radius, center[1]),
             (center[0] + radius, center[1]), (40, 230, 40), 4, cv2.LINE_AA)
    cv2.circle(result, center, 6, (255, 255, 255), -1, cv2.LINE_AA)
    cv2.circle(result, center, 6, (40, 230, 40), 2, cv2.LINE_AA)
    return result


def draw_ball_setup_target(
    frame: np.ndarray, lateral_cm: int, distance_cm: int
) -> np.ndarray:
    """Project a ground target into the fixed global camera for setup only."""
    result = frame.copy()
    height, width = result.shape[:2]
    theta = math.radians(GLOBAL_CAMERA_DOWN_DEG)
    focal_px = (width / 2.0) / math.tan(math.radians(GLOBAL_CAMERA_HFOV_DEG) / 2.0)
    lateral_m = lateral_cm / 100.0
    forward_m = distance_cm / 100.0
    # Camera coordinates: +x image-right, +y image-down, +z optical-forward.
    camera_z = forward_m * math.cos(theta) + GLOBAL_CAMERA_HEIGHT_M * math.sin(theta)
    camera_y = -forward_m * math.sin(theta) + GLOBAL_CAMERA_HEIGHT_M * math.cos(theta)
    if camera_z <= 0.0:
        return result
    raw_u = width / 2.0 + focal_px * lateral_m / camera_z
    raw_v = height / 2.0 + focal_px * camera_y / camera_z
    margin = 22
    u = int(round(np.clip(raw_u, margin, width - margin - 1)))
    v = int(round(np.clip(raw_v, margin, height - margin - 1)))
    outside = raw_u != np.clip(raw_u, margin, width - margin - 1) or raw_v != np.clip(
        raw_v, margin, height - margin - 1
    )
    color = (255, 0, 255)
    cv2.circle(result, (u, v), 13, (255, 255, 255), 4, cv2.LINE_AA)
    cv2.circle(result, (u, v), 9, color, -1, cv2.LINE_AA)
    cv2.drawMarker(result, (u, v), (0, 0, 0), cv2.MARKER_CROSS, 14, 2, cv2.LINE_AA)
    if outside:
        cv2.putText(result, "OUT", (max(4, u - 24), min(height - 5, v + 34)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)
    return result


def show_camera_preview(
    camera, global_camera, recording: bool, status_text: str,
    setup_target_cm: tuple[int, int] | None = None,
) -> bool:
    """Show wrist and global cameras together; return False on Q/Esc."""
    global _preview_window_initialized
    wrist = camera.latest_frame() if camera is not None else None
    global_frame = global_camera.latest_frame() if global_camera is not None else None
    if wrist is None and global_frame is None:
        return True

    panels = []
    for frame, label in ((wrist, "LOCAL / WRIST"), (global_frame, "GLOBAL")):
        panel = np.zeros((360, 640, 3), dtype=np.uint8)
        if frame is not None:
            frame = frame.copy()
            if label == "GLOBAL":
                if recording:
                    frame = draw_global_aid(frame)
                elif setup_target_cm is not None:
                    frame = draw_ball_setup_target(frame, *setup_target_cm)
            height, width = frame.shape[:2]
            scale = min(640.0 / width, 360.0 / height)
            resized = cv2.resize(frame, (round(width * scale), round(height * scale)))
            y = (360 - resized.shape[0]) // 2
            x = (640 - resized.shape[1]) // 2
            panel[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
        cv2.rectangle(panel, (0, 0), (220, 34), (0, 0, 0), -1)
        cv2.putText(panel, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (255, 255, 255), 2, cv2.LINE_AA)
        if recording:
            cv2.circle(panel, (615, 18), 8, (0, 0, 255), -1)
        panels.append(panel)
    if not _preview_window_initialized:
        cv2.namedWindow(PREVIEW_WINDOW, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(
            PREVIEW_WINDOW, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN
        )
        _preview_window_initialized = True
    preview = np.hstack(panels)
    text_size = cv2.getTextSize(status_text, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)[0]
    text_x = max((preview.shape[1] - text_size[0]) // 2, 0)
    cv2.rectangle(preview, (text_x - 12, 0),
                  (text_x + text_size[0] + 12, 38), (0, 0, 0), -1)
    cv2.putText(preview, status_text, (text_x, 27), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (255, 255, 255), 2, cv2.LINE_AA)
    cv2.imshow(PREVIEW_WINDOW, preview)
    key = cv2.waitKey(1) & 0xFF
    return key not in (27, ord("q"), ord("Q"))


def close_camera_preview() -> None:
    global _preview_window_initialized
    try:
        cv2.destroyWindow(PREVIEW_WINDOW)
    except cv2.error:
        pass
    _preview_window_initialized = False


class OneEuroFilter:
    """Same 1-Euro controller-pose filter used by the reference ZIP."""

    def __init__(self, dim: int, rate: float, min_cutoff: float = 1.0,
                 beta: float = 0.01, d_cutoff: float = 1.0) -> None:
        self.rate = rate
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = None
        self.dx_prev = np.zeros(dim)

    def _alpha(self, cutoff: float) -> float:
        te = 1.0 / self.rate
        tau = 1.0 / (2.0 * math.pi * cutoff)
        return 1.0 / (1.0 + tau / te)

    def filter(self, value: np.ndarray) -> np.ndarray:
        value = np.asarray(value, dtype=float)
        if self.x_prev is None:
            self.x_prev = value.copy()
            return value.copy()
        derivative_alpha = self._alpha(self.d_cutoff)
        derivative = (value - self.x_prev) * self.rate
        derivative_hat = derivative_alpha * derivative + (1.0 - derivative_alpha) * self.dx_prev
        cutoff = self.min_cutoff + self.beta * np.abs(derivative_hat)
        alpha = np.array([self._alpha(component) for component in cutoff])
        filtered = alpha * value + (1.0 - alpha) * self.x_prev
        self.x_prev = filtered.copy()
        self.dx_prev = derivative_hat.copy()
        return filtered


def get_motion_tracker_pose(serial: str) -> np.ndarray | None:
    """Return one valid tracker pose selected by serial, or None when absent."""
    try:
        count = int(xrt.num_motion_data_available())
        poses = xrt.get_motion_tracker_pose()
        serials = xrt.get_motion_tracker_serial_numbers()
    except Exception:
        return None
    for index in range(min(count, len(poses), len(serials))):
        tracker_serial = serials[index]
        if isinstance(tracker_serial, bytes):
            tracker_serial = tracker_serial.decode(errors="replace")
        if str(tracker_serial) != serial:
            continue
        pose = np.asarray(poses[index], dtype=float)
        if pose.ndim == 1 and pose.size >= 3 and np.all(np.isfinite(pose[:3])):
            return pose.copy()
    return None


def piper_joints(piper: C_PiperInterface_V2) -> tuple[np.ndarray, float, float]:
    msg = piper.GetArmJointMsgs()
    state = msg.joint_state
    joints = np.array([getattr(state, name.replace("joint", "joint_")) for name in JOINT_NAMES]) / 1000.0
    return joints.astype(float), float(msg.time_stamp), float(msg.Hz)


def configure_piper_sdk_feedback_filter(piper: C_PiperInterface_V2) -> None:
    """Keep required feedback while preventing unused 200Hz motor frames from reaching Python."""
    bus = piper.GetCanBus().bus
    if bus is None:
        raise RuntimeError("PiPER SocketCAN bus is not initialized")
    bus.set_filters([
        {"can_id": can_id, "can_mask": 0x7FF, "extended": False}
        for can_id in PIPER_SDK_FEEDBACK_IDS
    ])


class RawPiperFeedbackWatchdog:
    """Observe PiPER feedback independently of the SDK parser thread."""

    def __init__(self, can_name: str):
        self._socket = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self._socket.bind((can_name,))
        self._socket.settimeout(0.1)
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._last_ns = 0
        self._count = 0
        self._max_gap_ns = 0
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                frame = self._socket.recv(16)
            except TimeoutError:
                continue
            can_id = struct.unpack_from("=I", frame)[0] & 0x7FF
            if can_id not in PIPER_RAW_FEEDBACK_IDS:
                continue
            now_ns = time.monotonic_ns()
            with self._lock:
                if self._last_ns:
                    self._max_gap_ns = max(self._max_gap_ns, now_ns - self._last_ns)
                self._last_ns = now_ns
                self._count += 1

    def snapshot(self) -> dict[str, float | int | None]:
        now_ns = time.monotonic_ns()
        with self._lock:
            return {
                "feedback_age_ms": (now_ns - self._last_ns) / 1e6 if self._last_ns else None,
                "feedback_frames": self._count,
                "max_feedback_gap_ms": self._max_gap_ns / 1e6,
            }

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=1.0)
        self._socket.close()


def piper_snapshot(piper: C_PiperInterface_V2) -> dict[str, Any]:
    status = piper.GetArmStatus().arm_status
    error = getattr(status, "err_code", getattr(status, "_err_code", -1))
    gripper = piper.GetArmGripperMsgs().gripper_state
    end = piper.GetArmEndPoseMsgs().end_pose
    return {
        "control_mode": int(status.ctrl_mode),
        "arm_status": int(status.arm_status),
        "move_mode": int(status.mode_feed),
        "motion_status": int(status.motion_status),
        "error_code": int(error),
        "enabled": list(piper.GetArmEnableStatus()),
        "gripper_stroke_mm": float(gripper.grippers_angle) / 1000.0,
        "gripper_effort": int(gripper.grippers_effort),
        "end_pose_0p001": [
            int(end.X_axis), int(end.Y_axis), int(end.Z_axis),
            int(end.RX_axis), int(end.RY_axis), int(end.RZ_axis),
        ],
    }


class RightPicoIK:
    def __init__(self) -> None:
        self.robot = placo.RobotWrapper(str(URDF))
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.dt = DT
        self.solver.mask_fbase(True)
        self.ee_task = None
        self.manipulability_task = self.solver.add_manipulability_task("link6", "both", 1.0)
        self.manipulability_task.configure("manipulability", "soft", 1e-2)
        self.joint_task = self.solver.add_joints_task()
        self.joint_task.set_joints({name: 0.0 for name in JOINT_NAMES})
        self.joint_task.configure("joint_regularization", "soft", 1e-3)
        tracker_target = self.robot.get_T_world_frame(ELBOW_TRACKER_LINK)[:3, 3].copy()
        self.tracker_task = self.solver.add_position_task(ELBOW_TRACKER_LINK, tracker_target)
        self.tracker_task.configure("right_elbow_tracker", "soft", ELBOW_TRACKER_WEIGHT)
        self.ref_controller_pos = None
        self.ref_controller_rot = None
        self.ref_ee = None
        self.ref_tracker_pos = None
        self.ref_robot_link3_pos = None
        self.pos_filter = None
        self.rot_filter = None

    def sync(self, joints_deg: np.ndarray) -> None:
        self.robot.state.q[:7] = np.array([0, 0, 0, 0, 0, 0, 1], dtype=float)
        joints_rad = np.radians(joints_deg)
        self.robot.state.q[7:13] = joints_rad
        self.robot.update_kinematics()
        if self.ref_ee is None:
            self.tracker_task.target_world = self.robot.get_T_world_frame(ELBOW_TRACKER_LINK)[:3, 3].copy()

    @staticmethod
    def world_pose(xr_pose: np.ndarray) -> tuple[np.ndarray, Rotation]:
        pos = R_HEADSET_WORLD @ xr_pose[:3]
        controller = Rotation.from_quat(xr_pose[3:7])
        frame = Rotation.from_matrix(R_HEADSET_WORLD)
        rot = frame * controller * frame.inv()
        return pos, rot

    def activate(self, xr_pose: np.ndarray, tracker_pose: np.ndarray | None = None) -> None:
        self.ref_controller_pos, self.ref_controller_rot = self.world_pose(xr_pose)
        self.pos_filter = OneEuroFilter(3, CONTROL_HZ)
        self.pos_filter.filter(self.ref_controller_pos)
        if ROTATION_SCALE:
            self.rot_filter = OneEuroFilter(3, CONTROL_HZ)
            self.rot_filter.filter(self.ref_controller_rot.as_rotvec())
        self.ref_ee = self.robot.get_T_world_frame("link6").copy()
        self.ref_tracker_pos = None
        self.ref_robot_link3_pos = self.robot.get_T_world_frame(ELBOW_TRACKER_LINK)[:3, 3].copy()
        self.tracker_task.target_world = self.ref_robot_link3_pos.copy()
        if tracker_pose is not None:
            self.ref_tracker_pos = R_HEADSET_WORLD @ tracker_pose[:3]
        if self.ee_task is None:
            self.ee_task = self.solver.add_frame_task("link6", self.ref_ee)
            self.ee_task.configure("right_ee", "soft", 1.0)

    def deactivate(self) -> None:
        self.ref_controller_pos = None
        self.ref_controller_rot = None
        self.ref_ee = None
        self.ref_tracker_pos = None
        self.ref_robot_link3_pos = None
        self.pos_filter = None
        self.rot_filter = None

    def solve(
        self,
        xr_pose: np.ndarray,
        tracker_pose: np.ndarray | None = None,
        locked_lateral_y: float | None = None,
    ) -> np.ndarray:
        if self.ref_ee is None:
            raise RuntimeError("clutch reference is absent")
        pos, rot = self.world_pose(xr_pose)
        pos = self.pos_filter.filter(pos)
        delta_pos = (pos - self.ref_controller_pos) * POSITION_SCALE
        target = self.ref_ee.copy()
        target[:3, 3] += delta_pos
        if locked_lateral_y is not None:
            target[1, 3] = locked_lateral_y
        if ROTATION_SCALE:
            rot = Rotation.from_rotvec(self.rot_filter.filter(rot.as_rotvec()))
            delta_rotvec = (rot * self.ref_controller_rot.inv()).as_rotvec() * ROTATION_SCALE
            target[:3, :3] = Rotation.from_rotvec(delta_rotvec).as_matrix() @ self.ref_ee[:3, :3]
        self.ee_task.T_world_frame = target
        if tracker_pose is not None:
            tracker_pos = R_HEADSET_WORLD @ tracker_pose[:3]
            if self.ref_tracker_pos is None:
                self.ref_tracker_pos = tracker_pos.copy()
                self.ref_robot_link3_pos = self.robot.get_T_world_frame(ELBOW_TRACKER_LINK)[:3, 3].copy()
            tracker_delta = (tracker_pos - self.ref_tracker_pos) * POSITION_SCALE
            self.tracker_task.target_world = self.ref_robot_link3_pos + tracker_delta
        elif self.ref_tracker_pos is not None:
            # Do not retain a stale elbow target when optical tracking drops.
            self.ref_tracker_pos = None
            self.ref_robot_link3_pos = self.robot.get_T_world_frame(ELBOW_TRACKER_LINK)[:3, 3].copy()
            self.tracker_task.target_world = self.ref_robot_link3_pos.copy()
        self.solver.solve(True)
        self.robot.update_kinematics()
        return np.degrees(self.robot.state.q[7:13].copy())


def send_arm(piper: C_PiperInterface_V2, joints_deg: np.ndarray) -> None:
    piper.MotionCtrl_2(0x01, 0x01, MOVE_SPEED_PERCENT, 0x00)
    piper.JointCtrl(*(int(round(v * 1000.0)) for v in joints_deg))


def validate_joint_target(solved: np.ndarray) -> np.ndarray:
    """Reject invalid IK output without altering the ZIP-equivalent target."""
    if solved.shape != (6,) or not np.all(np.isfinite(solved)):
        raise RuntimeError("IK returned an invalid joint target")
    return solved


def move_to_pose(
    piper: C_PiperInterface_V2,
    joints_deg: np.ndarray,
    speed_percent: int = INIT_MOVE_SPEED_PERCENT,
) -> None:
    """Match PiperFollower's direct initial/rest pose command."""
    piper.MotionCtrl_2(0x01, 0x01, speed_percent, 0x00)
    piper.JointCtrl(*(int(round(v * 1000.0)) for v in joints_deg))


class EmergencyStopRequested(RuntimeError):
    pass


def wait_for_pose(
    piper: C_PiperInterface_V2,
    target: np.ndarray,
    timeout_s: float = 10.0,
    monitor_b: bool = False,
    max_joint_speed_deg_s: float | None = None,
    sample_callback: Any | None = None,
) -> None:
    """Stream a return target until feedback confirms it before torque release."""
    start, _, hz = piper_joints(piper)
    if hz <= 0:
        raise RuntimeError("PiPER feedback absent before pose move")
    move_started = time.monotonic()
    move_duration = 0.0
    if max_joint_speed_deg_s is not None:
        if max_joint_speed_deg_s <= 0:
            raise ValueError("max_joint_speed_deg_s must be positive")
        move_duration = float(np.max(np.abs(target - start))) / max_joint_speed_deg_s
        timeout_s = max(timeout_s, move_duration + 3.0)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        iteration_started = time.monotonic()
        if monitor_b and bool(xrt.get_B_button()):
            current, _, hz = piper_joints(piper)
            if hz > 0:
                send_arm(piper, current)
            raise EmergencyStopRequested("B guarded hold requested during return")
        state = piper_snapshot(piper)
        if state["error_code"] != 0 or state["arm_status"] != 0:
            raise RuntimeError(f"return aborted by PiPER state: {state}")
        current, _, hz = piper_joints(piper)
        if hz > 0 and np.max(np.abs(current - target)) <= 0.5:
            if sample_callback is not None:
                sample_callback(target)
            return
        commanded = target
        speed_percent = INIT_MOVE_SPEED_PERCENT
        if move_duration > 0.0:
            progress = min((time.monotonic() - move_started) / move_duration, 1.0)
            # Quintic minimum-jerk timing: zero velocity and acceleration at
            # both ends, avoiding the visible snap of linear interpolation.
            alpha = 10.0 * progress**3 - 15.0 * progress**4 + 6.0 * progress**5
            commanded = start + alpha * (target - start)
            speed_percent = AUTO_INIT_SPEED_PERCENT
        move_to_pose(piper, commanded, speed_percent=speed_percent)
        if sample_callback is not None:
            sample_callback(commanded)
        remaining = DT - (time.monotonic() - iteration_started)
        if remaining > 0.0:
            time.sleep(remaining)
    raise RuntimeError(f"return pose timeout; target={target.tolist()}")


def compute_gripper_target(trigger: float) -> float:
    """Map the processed trigger directly, as in the reference ZIP teleoperator."""
    return GRIPPER_OPEN_MM + trigger * (GRIPPER_CLOSED_MM - GRIPPER_OPEN_MM)


def send_gripper(piper: C_PiperInterface_V2, target_mm: float) -> None:
    target = min(max(target_mm, GRIPPER_CLOSED_MM), GRIPPER_OPEN_MM)
    piper.GripperCtrl(int(round(target * 1000.0)), 1000, 0x01, 0)


def countdown() -> None:
    print("WARNING: LIVE PiPER motion will start. Clear the workspace; keep B ready.")


def main() -> None:
    # The SDK parses ~3,000 CAN frames/s in Python. A shorter switch interval
    # prevents its receiver thread from starving the command loop.
    sys.setswitchinterval(0.001)
    parser = argparse.ArgumentParser(description="Compact right-PICO single-PiPER IK teleoperation")
    parser.add_argument("--dry-run", action="store_true", help="run XR+IK+logging without CAN commands")
    parser.add_argument("--zip-auto-pose", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--no-camera", action="store_true", help="disable InnoMaker wrist RGB recording")
    parser.add_argument("--wrist-camera-device", default="", help="V4L2 device; empty auto-detects InnoMaker")
    parser.add_argument("--wrist-camera-width", type=int, default=640)
    parser.add_argument("--wrist-camera-height", type=int, default=480)
    parser.add_argument("--wrist-camera-fps", type=int, default=30)
    parser.add_argument("--no-global-camera", action="store_true", help="disable Logitech global RGB recording")
    parser.add_argument("--global-camera-device", default="", help="V4L2 device; empty auto-detects the C920")
    parser.add_argument("--global-camera-width", type=int, default=1280)
    parser.add_argument("--global-camera-height", type=int, default=720)
    parser.add_argument("--global-camera-fps", type=int, default=15)
    parser.add_argument(
        "--right-elbow-tracker-serial",
        default=RIGHT_ELBOW_TRACKER_SERIAL,
        help="PICO Motion Tracker serial for the right elbow; empty disables it",
    )
    parser.add_argument("--with-tracer", action="store_true", help="enable left-controller Tracer teleoperation")
    parser.add_argument(
        "--calibrate-floor-init", action="store_true",
        help="adjust floor-pick init with lateral translation locked; A saves it",
    )
    parser.add_argument(
        "--calibrate-drop-pose", action="store_true",
        help="adjust the cup pre-drop pose; A saves its measured EE pose",
    )
    args = parser.parse_args()
    if args.calibrate_floor_init and args.calibrate_drop_pose:
        parser.error("select only one calibration mode")
    calibration_mode = args.calibrate_floor_init or args.calibrate_drop_pose
    stop_requested = threading.Event()
    guarded_exit_requested = threading.Event()

    def request_stop(signum, _frame) -> None:
        print(f"Stop requested by signal {signum}; applying B guarded exit...", flush=True)
        guarded_exit_requested.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    if not args.dry_run and not os.path.exists(f"/sys/class/net/{CAN_NAME}"):
        raise RuntimeError(f"{CAN_NAME} is absent")
    if args.with_tracer and not os.path.exists(f"/sys/class/net/{TRACER_CAN_NAME}"):
        raise RuntimeError(f"{TRACER_CAN_NAME} is absent")

    EPISODES.mkdir(parents=True, exist_ok=True)
    plan_path = EPISODES / "collection_plan_100.json"
    plan_path.write_text(
        json.dumps(
            {
                "seed": 20260909,
                "plan_version": 2,
                "coordinate_frame": "tracer_start_forward",
                "lateral_sign": "negative=left, positive=right",
                "global_camera_visible_only": True,
                "global_camera_model": {
                    "height_m": GLOBAL_CAMERA_HEIGHT_M,
                    "down_angle_deg": GLOBAL_CAMERA_DOWN_DEG,
                    "horizontal_fov_deg": GLOBAL_CAMERA_HFOV_DEG,
                },
                "positions_cm": [
                    {"index": index + 1, "lateral": item[0], "distance": item[1]}
                    for index, item in enumerate(BALL_POSITION_PLAN)
                ],
            },
            indent=2,
        ),
        encoding="utf-8",
    )
    collection_plan_index = completed_planned_episodes()
    episode = None
    samples_path = None
    metadata_path = None
    metadata = None
    log = None
    camera = None
    global_camera = None

    piper = None
    raw_piper_watchdog = None
    if args.dry_run:
        joints = np.zeros(6)
        feedback_stamp = 0.0
    else:
        piper = C_PiperInterface_V2(
            can_name=CAN_NAME, judge_flag=True, can_auto_init=True, dh_is_offset=0,
            start_sdk_joint_limit=False, start_sdk_gripper_limit=False,
        )
        configure_piper_sdk_feedback_filter(piper)
        # Feedback is already broadcast continuously. Avoid SDK PiperInit(),
        # which sends three unrelated limit/firmware queries on every connect.
        piper.ConnectPort(piper_init=False)
        time.sleep(1.0)
        joints, feedback_stamp, hz = piper_joints(piper)
        if hz <= 0:
            piper.DisconnectPort()
            raise RuntimeError("PiPER feedback is absent")
        raw_piper_watchdog = RawPiperFeedbackWatchdog(CAN_NAME)
        raw_piper_watchdog.start()
    session_start_joints = joints.copy()

    tracer_bridge = None
    tracer_can = None
    tracer_state = {
        "vehicle": None, "error": None, "stamp": 0.0,
        "cmd_vel_subscribers": 0, "bridge_alive": False,
    }
    tracer_latched = False
    tracer_reset_sent_at = None
    previous_y = False
    previous_a = False
    previous_b = False
    previous_view_switch = False
    pico_view_global = False
    recording = False
    episode_started_monotonic = None
    episode_phase = None
    phase_started_monotonic = None
    pending_event = None
    sample_count = 0
    manual_setup = False
    calibration_lateral_y = None
    calibration_locked_joints_deg = None
    calibration_gripper_mm = None
    last_xr_ns = None
    last_xr_change = time.monotonic()
    tracker_was_available = None
    xrt_initialized = False

    def cleanup_failed_startup() -> None:
        """Release software resources without adding any robot motion."""
        nonlocal tracer_bridge, tracer_can, raw_piper_watchdog, xrt_initialized
        nonlocal camera, global_camera
        if camera is not None:
            camera.stop()
            camera = None
        if global_camera is not None:
            global_camera.stop()
            global_camera = None
        if tracer_bridge is not None:
            tracer_bridge.close()
            tracer_bridge = None
        if tracer_can is not None:
            tracer_can.close()
            tracer_can = None
        if xrt_initialized:
            xrt.close()
            xrt_initialized = False
        if raw_piper_watchdog is not None:
            raw_piper_watchdog.stop()
            raw_piper_watchdog = None
        if piper is not None:
            piper.DisconnectPort()

    try:
        if args.with_tracer:
            tracer_bridge = TracerRosBridge()
            tracer_bridge.start()
            tracer_state = tracer_bridge.snapshot()
            tracer_can = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            tracer_can.bind((TRACER_CAN_NAME,))
            enable_commanded_mode(tracer_can)
            if tracer_state["vehicle"] != 0 or tracer_state["error"] != 0:
                raise RuntimeError(f"Tracer is not ready: {tracer_state}")
    except Exception:
        cleanup_failed_startup()
        raise

    ik = RightPicoIK()
    ik.sync(joints)
    clutch = False
    filtered_trigger = 0.0
    last_command = joints.copy()
    last_feedback_change = time.monotonic()
    unsafe_stop = False
    try:
        xrt.init()
        xrt_initialized = True
    except Exception:
        cleanup_failed_startup()
        raise
    print("XRoboToolkit initialized. Right grip=PiPER, left grip=Tracer, B=stop.")
    xr_deadline = time.monotonic() + 3.0
    xr_first = int(xrt.get_time_stamp_ns())
    xr_ready = False
    while time.monotonic() < xr_deadline:
        xr_pose = np.asarray(xrt.get_right_controller_pose(), dtype=float)
        xr_now = int(xrt.get_time_stamp_ns())
        if xr_now > xr_first and xr_pose.shape == (7,) and np.all(np.isfinite(xr_pose)) and not np.allclose(xr_pose, 0):
            xr_ready = True
            break
        time.sleep(0.05)
    if not xr_ready:
        cleanup_failed_startup()
        raise RuntimeError("PICO tracking stream is absent or stale")

    # Camera availability is a precondition for live motion. Open both shared
    # feeds before enabling PiPER so a failed fanout can never leave the arm at
    # init with torque enabled.
    preview_episode = ROOT / ".runtime" / "camera_preview"
    try:
        if not args.no_camera:
            camera = WristUvcCamera(
                device=args.wrist_camera_device, width=args.wrist_camera_width,
                height=args.wrist_camera_height, fps=args.wrist_camera_fps,
            )
            camera.start(preview_episode)
            camera.pause_recording()
        if not args.no_global_camera:
            global_camera = GlobalWebcam(
                device=args.global_camera_device, width=args.global_camera_width,
                height=args.global_camera_height, fps=args.global_camera_fps,
            )
            global_camera.start(preview_episode)
            global_camera.pause_recording()
    except Exception:
        cleanup_failed_startup()
        raise
    finally:
        shutil.rmtree(preview_episode, ignore_errors=True)

    if not args.dry_run:
        try:
            countdown()
            # Enable while holding only the measured joint pose.
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and not all(piper.GetArmEnableStatus()):
                send_arm(piper, joints)
                piper.EnableArm(7)
                time.sleep(0.02)
            if not all(piper.GetArmEnableStatus()):
                raise RuntimeError("PiPER enable timeout")
            current_gripper_mm = piper_snapshot(piper)["gripper_stroke_mm"]
            if calibration_mode:
                calibration_gripper_mm = float(current_gripper_mm)
            piper.GripperCtrl(int(round(current_gripper_mm * 1000.0)), 1000, 0x03, 0)
            # Torque-off feedback can describe a slightly sagged pose that is not
            # reachable as a commanded hold. Capture the normal return target only
            # after the arm is enabled and holding its measured pose.
            time.sleep(0.2)
            joints, feedback_stamp, hz = piper_joints(piper)
            if hz <= 0:
                raise RuntimeError("PiPER feedback absent after enable")
            session_start_joints = joints.copy()
            ik.sync(joints)
            last_command = joints.copy()
            last_feedback_change = time.monotonic()
            print("Moving to ZIP start pose...")
            wait_for_pose(piper, JOINTS_START_DEG, monitor_b=True)
            print("Moving to ZIP teleop init pose...")
            # Start folded at the locomotion-safe init. The calibrated floor
            # init is entered only after locomotion is marked complete.
            wait_for_pose(piper, JOINTS_INIT_DEG, monitor_b=True)
            joints, feedback_stamp, _ = piper_joints(piper)
            ik.sync(joints)
            if args.calibrate_floor_init:
                calibration_lateral_y = float(
                    ik.robot.get_T_world_frame("link6")[1, 3]
                )
                calibration_locked_joints_deg = joints.copy()
            last_command = joints.copy()
            last_feedback_change = time.monotonic()
        except Exception:
            # Once torque has been enabled, startup failures must use the same
            # guarded return as a normal B/Ctrl-C exit. Camera preflight runs
            # before this block, so ordinary camera failures never reach here.
            try:
                if piper is not None and any(piper.GetArmEnableStatus()):
                    print("Startup failed: returning to ZIP start before torque release...")
                    wait_for_pose(piper, JOINTS_START_DEG, monitor_b=False)
                    piper.MotionCtrl_2(
                        0x00, 0x01, INIT_MOVE_SPEED_PERCENT, 0x00
                    )
                    time.sleep(0.5)
                    piper.DisableArm(7)
                    time.sleep(1.0)
                    print("Startup return confirmed; arm torque released.")
            finally:
                cleanup_failed_startup()
            raise

    def start_episode() -> None:
        nonlocal episode, samples_path, metadata_path, metadata, log, camera, global_camera
        nonlocal recording, episode_started_monotonic, episode_phase
        nonlocal phase_started_monotonic, pending_event, sample_count, manual_setup
        nonlocal collection_plan_index
        if collection_plan_index >= len(BALL_POSITION_PLAN):
            print("COLLECTION 100/100 COMPLETE")
            return
        lateral_cm, distance_cm = BALL_POSITION_PLAN[collection_plan_index]
        episode = EPISODES / time.strftime("%Y%m%d_%H%M%S")
        suffix = 1
        while episode.exists():
            episode = EPISODES / f"{time.strftime('%Y%m%d_%H%M%S')}_{suffix:02d}"
            suffix += 1
        episode.mkdir()
        samples_path = episode / "samples.jsonl"
        metadata_path = episode / "metadata.json"
        try:
            if not args.no_camera:
                if camera is None:
                    camera = WristUvcCamera(
                        device=args.wrist_camera_device,
                        width=args.wrist_camera_width,
                        height=args.wrist_camera_height,
                        fps=args.wrist_camera_fps,
                    )
                    camera.start(episode)
                else:
                    camera.start_recording(episode)
            if not args.no_global_camera:
                if global_camera is None:
                    global_camera = GlobalWebcam(
                        device=args.global_camera_device,
                        width=args.global_camera_width,
                        height=args.global_camera_height,
                        fps=args.global_camera_fps,
                    )
                    global_camera.start(episode)
                else:
                    global_camera.start_recording(episode)
            log = AsyncJsonlWriter(samples_path)
        except Exception:
            if global_camera is not None:
                global_camera.stop()
                global_camera = None
            if camera is not None:
                camera.stop()
                camera = None
            raise
        metadata = {
            "episode_id": episode.name, "started_wall_ns": time.time_ns(),
            "episodes_root": str(EPISODES),
            "collection_plan_index": collection_plan_index + 1,
            "ball_start_target_cm": {
                "lateral": lateral_cm,
                "distance": distance_cm,
            },
            "dry_run": args.dry_run, "camera_enabled": camera is not None,
            "camera_serial": "SN0001" if camera is not None else None,
            "wrist_camera_enabled": camera is not None,
            "wrist_camera_type": "innomaker_uvc_rgb" if camera is not None else None,
            "wrist_camera_has_depth": False if camera is not None else None,
            "wrist_camera_device": camera.device if camera is not None else None,
            "wrist_camera_resolution": [args.wrist_camera_width, args.wrist_camera_height]
            if camera is not None else None,
            "wrist_camera_fps": args.wrist_camera_fps if camera is not None else None,
            "global_camera_enabled": global_camera is not None,
            "global_camera_device": global_camera.device if global_camera is not None else None,
            "global_camera_resolution": [args.global_camera_width, args.global_camera_height]
            if global_camera is not None else None,
            "global_camera_fps": args.global_camera_fps if global_camera is not None else None,
            "control_hz": CONTROL_HZ, "position_scale": POSITION_SCALE,
            "rotation_scale": ROTATION_SCALE, "grip_threshold": GRIP_THRESHOLD,
            "auto_init_joint_speed_deg_s": AUTO_INIT_JOINT_SPEED_DEG_S,
            "auto_init_speed_percent": AUTO_INIT_SPEED_PERCENT,
            "right_elbow_tracker_serial": args.right_elbow_tracker_serial or None,
            "right_elbow_tracker_link": ELBOW_TRACKER_LINK if args.right_elbow_tracker_serial else None,
            "right_elbow_tracker_weight": ELBOW_TRACKER_WEIGHT if args.right_elbow_tracker_serial else None,
            "session_start_joint_deg": session_start_joints.tolist(),
            "episode_home_joint_deg": JOINTS_INIT_DEG.tolist(),
            "manipulation_init_joint_deg": load_floor_init().tolist(),
            "cup_drop_joint_deg": load_cup_drop_pose()[0].tolist(),
            "cup_clearance_joint_deg": solve_cup_clearance_pose().tolist(),
            "cup_clearance_m": CUP_CLEARANCE_M,
            "tracer_enabled": args.with_tracer,
            "tracer_ros_bridge": "separate_process" if args.with_tracer else None,
            "tracer_max_linear_mps": TRACER_LINEAR_MPS if args.with_tracer else None,
            "tracer_max_angular_rad_s": TRACER_ANGULAR_RAD_S if args.with_tracer else None,
            "workflow_version": 4,
            "phase_sequence": ["locomotion", "manipulation", "transport"],
            "automatic_motion_recorded": True,
            "learning_schema_version": 1,
            "learning_schema": {
                "observation": {
                    "piper_joint_deg": "measured 6-DoF joint position",
                    "piper_status.gripper_stroke_mm": "measured gripper position",
                    "piper_status.gripper_effort": "measured gripper effort",
                    "piper_status.end_pose_0p001": "measured SDK end pose",
                    "tracer_status.odometry": "measured base pose and twist",
                    "wrist_camera_frame_reference": "unmodified local RGB frame",
                    "global_camera_frame_reference": "unmodified global RGB frame",
                },
                "action": {
                    "commanded_joint_deg": "absolute 6-DoF joint target",
                    "commanded_gripper_mm": "absolute gripper target",
                    "tracer_command": "base linear/angular velocity target",
                },
                "derivable_action_representations": [
                    "absolute", "stepwise_delta", "chunkwise_delta"
                ],
                "derivation_policy": (
                    "derive representations after temporal resampling; preserve these raw "
                    "measurements and commands as the immutable source"
                ),
                "model_auxiliary_only": [
                    "episode_phase", "event", "control_source", "trajectory_subphase",
                    "XR controller inputs"
                ],
            },
            "automatic_subphases": [
                "manipulation_init", "fold_after_pick", "cup_clearance",
                "cup_descent", "cup_retreat", "return_after_drop",
            ],
            "events": [],
            "status": "recording",
        }
        metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        recording = True
        episode_started_monotonic = time.monotonic()
        episode_phase = "locomotion"
        phase_started_monotonic = episode_started_monotonic
        pending_event = "episode_started"
        sample_count = 0
        manual_setup = False
        print(f"A1 START: {episode.name} | {ball_target_label(collection_plan_index)}")

    def finish_episode(
        status: str = "complete", reason: str = "A",
        returned_to_home: bool = False,
    ) -> None:
        nonlocal episode, samples_path, metadata_path, metadata, log, camera, global_camera
        nonlocal recording, episode_started_monotonic, episode_phase
        nonlocal phase_started_monotonic, pending_event, sample_count, manual_setup
        nonlocal collection_plan_index
        if not recording:
            return
        if log is not None:
            log.flush()
            log.close()
        if camera is not None:
            camera.pause_recording()
        if global_camera is not None:
            global_camera.pause_recording()
        if status == "complete":
            metadata.update({"ended_wall_ns": time.time_ns(), "status": status, "stop_reason": reason,
                             "returned_to_episode_home": returned_to_home,
                             "sample_count": sample_count})
            metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
            print(f"Episode saved: {samples_path}")
            collection_plan_index = min(
                collection_plan_index + 1, len(BALL_POSITION_PLAN)
            )
        else:
            # Failed/aborted demonstrations are not training data.
            shutil.rmtree(episode)
            print(f"Incomplete episode deleted: {episode} ({reason})")
        episode = samples_path = metadata_path = metadata = log = None
        recording = False
        episode_started_monotonic = None
        episode_phase = None
        phase_started_monotonic = None
        pending_event = None
        sample_count = 0
        manual_setup = status == "complete"

    def mark_episode_event(name: str, xr_ns: int) -> None:
        event = {
            "name": name,
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "timestamp_wall_ns": time.time_ns(),
            "timestamp_xr_ns": xr_ns,
            "sample_index": sample_count,
        }
        metadata["events"].append(event)
        metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    def record_automatic_sample(
        commanded_joints: np.ndarray,
        commanded_gripper_mm: float,
        trajectory_subphase: str,
    ) -> None:
        """Record scripted motion on the same 50 Hz timeline as teleoperation."""
        nonlocal sample_count, pending_event, joints, feedback_stamp
        if not recording or log is None or piper is None:
            return
        measured_joints, feedback_stamp, hz = piper_joints(piper)
        if hz <= 0:
            raise RuntimeError("PiPER feedback absent during automatic recording")
        joints = measured_joints.copy()
        state = piper_snapshot(piper)
        tracer_snapshot = tracer_bridge.snapshot() if tracer_bridge is not None else None
        wrist_ref = camera.latest() if camera is not None else None
        global_ref = global_camera.latest() if global_camera is not None else None
        xr_pose = np.asarray(xrt.get_right_controller_pose(), dtype=float)
        sample = {
            "timestamp_monotonic_ns": time.monotonic_ns(),
            "timestamp_wall_ns": time.time_ns(),
            "timestamp_xr_ns": int(xrt.get_time_stamp_ns()),
            "right_pose_xyzw": xr_pose.tolist(),
            "right_elbow_tracker_serial": args.right_elbow_tracker_serial or None,
            "right_elbow_tracker_pose": None,
            "right_elbow_tracker_active": False,
            "right_grip": float(xrt.get_right_grip()),
            "right_trigger": float(xrt.get_right_trigger()),
            "filtered_right_trigger": None,
            "right_axis": list(xrt.get_right_axis()),
            "left_axis": list(xrt.get_left_axis()),
            "left_grip": float(xrt.get_left_grip()),
            "recording": True,
            "episode_phase": episode_phase,
            "event": pending_event,
            "control_source": "automatic",
            "trajectory_subphase": trajectory_subphase,
            "clutch_active": False,
            "commanded_joint_deg": np.asarray(commanded_joints, dtype=float).tolist(),
            "commanded_gripper_mm": float(commanded_gripper_mm),
            "piper_joint_deg": measured_joints.tolist(),
            "piper_status": state,
            "piper_raw_can": raw_piper_watchdog.snapshot()
            if raw_piper_watchdog is not None else None,
            "tracer_command": {"linear_mps": 0.0, "angular_rad_s": 0.0},
            "tracer_status": dict(tracer_snapshot) if tracer_snapshot is not None else None,
            "dry_run": args.dry_run,
            "camera_frame_reference": wrist_ref,
            "wrist_camera_frame_reference": wrist_ref,
            "global_camera_frame_reference": global_ref,
        }
        log.write(json.dumps(sample, ensure_ascii=False) + "\n")
        sample_count += 1
        pending_event = None

    def move_piper_to_init(trajectory_subphase: str = "return_init") -> None:
        nonlocal clutch, joints, feedback_stamp, last_command, last_feedback_change
        if tracer_bridge is not None:
            tracer_bridge.stop_motion()
        if clutch:
            ik.deactivate()
            clutch = False
        if piper is not None:
            held_gripper = float(piper_snapshot(piper)["gripper_stroke_mm"])
            wait_for_pose(
                piper,
                JOINTS_INIT_DEG,
                monitor_b=True,
                max_joint_speed_deg_s=AUTO_INIT_JOINT_SPEED_DEG_S,
                sample_callback=lambda command: record_automatic_sample(
                    command, held_gripper, trajectory_subphase
                ),
            )
            joints, feedback_stamp, _ = piper_joints(piper)
            ik.sync(joints)
            last_command = joints.copy()
            last_feedback_change = time.monotonic()

    def move_piper_to_manipulation_init() -> None:
        nonlocal clutch, joints, feedback_stamp, last_command, last_feedback_change
        if tracer_bridge is not None:
            tracer_bridge.stop_motion()
        if clutch:
            ik.deactivate()
            clutch = False
        if piper is not None:
            held_gripper = float(piper_snapshot(piper)["gripper_stroke_mm"])
            wait_for_pose(
                piper,
                load_floor_init(),
                monitor_b=True,
                max_joint_speed_deg_s=AUTO_INIT_JOINT_SPEED_DEG_S,
                sample_callback=lambda command: record_automatic_sample(
                    command, held_gripper, "manipulation_init"
                ),
            )
            joints, feedback_stamp, _ = piper_joints(piper)
            ik.sync(joints)
            last_command = joints.copy()
            last_feedback_change = time.monotonic()

    def move_piper_to_cup_drop() -> None:
        """Approach the cup vertically through an EE-derived clearance pose."""
        nonlocal clutch, joints, feedback_stamp, last_command, last_feedback_change
        if tracer_bridge is not None:
            tracer_bridge.stop_motion()
        if clutch:
            ik.deactivate()
            clutch = False
        if piper is not None:
            held_gripper = float(piper_snapshot(piper)["gripper_stroke_mm"])
            clearance_joints = solve_cup_clearance_pose()
            drop_joints, _ = load_cup_drop_pose()
            wait_for_pose(
                piper, clearance_joints, monitor_b=True,
                max_joint_speed_deg_s=CUP_APPROACH_JOINT_SPEED_DEG_S,
                sample_callback=lambda command: record_automatic_sample(
                    command, held_gripper, "cup_clearance"
                ),
            )
            wait_for_pose(
                piper, drop_joints, monitor_b=True,
                max_joint_speed_deg_s=CUP_APPROACH_JOINT_SPEED_DEG_S,
                sample_callback=lambda command: record_automatic_sample(
                    command, held_gripper, "cup_descent"
                ),
            )
            joints, feedback_stamp, _ = piper_joints(piper)
            ik.sync(joints)
            last_command = joints.copy()
            last_feedback_change = time.monotonic()

    def retreat_from_cup_after_drop() -> None:
        """Open and immediately reverse the descent to the safe clearance."""
        nonlocal clutch, joints, feedback_stamp, last_command, last_feedback_change
        if tracer_bridge is not None:
            tracer_bridge.stop_motion()
        if clutch:
            ik.deactivate()
            clutch = False
        if piper is not None:
            # GripperCtrl is non-blocking. Keep its open target active while the
            # arm rises, so there is no visually aliased stationary segment.
            send_gripper(piper, GRIPPER_OPEN_MM)
            wait_for_pose(
                piper,
                solve_cup_clearance_pose(),
                monitor_b=True,
                max_joint_speed_deg_s=CUP_APPROACH_JOINT_SPEED_DEG_S,
                sample_callback=lambda command: record_automatic_sample(
                    command, GRIPPER_OPEN_MM, "cup_retreat"
                ),
            )
            joints, feedback_stamp, _ = piper_joints(piper)
            ik.sync(joints)
            last_command = joints.copy()
            last_feedback_change = time.monotonic()
    failure_reason = None
    cleanup_errors = []
    return_confirmed = False
    torque_released = False
    if calibration_mode:
        manual_setup = True
        if args.calibrate_floor_init:
            print("CALIBRATE FLOOR: J2/J3/J5 only | A=save | B=exit")
        else:
            print("CALIBRATE DROP: grip=full 6D adjust | A=save EE pose | B=exit")
    else:
        print("READY: A1 start | A2 approach | A3 pick/drop/save")
    try:
        while not stop_requested.is_set():
                started = time.monotonic()
                pose = np.asarray(xrt.get_right_controller_pose(), dtype=float)
                tracker_pose = (
                    get_motion_tracker_pose(args.right_elbow_tracker_serial)
                    if args.right_elbow_tracker_serial else None
                )
                tracker_available = tracker_pose is not None
                if tracker_available != tracker_was_available:
                    state_label = "connected" if tracker_available else "absent; controller-only"
                    print(f"Right elbow tracker {args.right_elbow_tracker_serial}: {state_label}")
                    tracker_was_available = tracker_available
                grip = float(xrt.get_right_grip())
                trigger = float(xrt.get_right_trigger())
                left_axis = list(xrt.get_left_axis())
                right_axis = list(xrt.get_right_axis())
                view_switch_pressed = bool(xrt.get_right_axis_click())
                left_grip = float(xrt.get_left_grip())
                xr_ns = int(xrt.get_time_stamp_ns())
                if xr_ns != last_xr_ns:
                    last_xr_ns = xr_ns
                    last_xr_change = started
                xr_stale = started - last_xr_change > TRACER_INPUT_TIMEOUT_S
                if pose.shape != (7,) or not np.all(np.isfinite(pose)):
                    raise RuntimeError("invalid right-controller pose")
                b_pressed = bool(xrt.get_B_button())
                a_pressed = bool(xrt.get_A_button())
                terminal_toggle = False
                if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0.0)[0]:
                    terminal_toggle = sys.stdin.readline() == "\n"
                sticks_neutral = (
                    abs(float(left_axis[1])) <= TRACER_DEADZONE
                    and abs(float(right_axis[0])) <= TRACER_DEADZONE
                )
                b_exit = b_pressed and not previous_b
                if b_exit or guarded_exit_requested.is_set():
                    if tracer_bridge is not None:
                        tracer_bridge.stop_motion()
                    if piper is not None:
                        current, _, hz = piper_joints(piper)
                        if hz > 0:
                            send_arm(piper, current)
                            joints = current
                            last_command = current.copy()
                    if clutch:
                        ik.deactivate()
                        clutch = False
                    source = "B" if b_exit else "Ctrl-C"
                    print(f"{source}: motion held; guarded ZIP-start return")
                    break

                if view_switch_pressed and not previous_view_switch:
                    try:
                        sender_pid = int(PICO_VIDEO_PID_FILE.read_text().strip())
                        os.kill(sender_pid, signal.SIGUSR1)
                        pico_view_global = not pico_view_global
                        print(f"VIEW: {'GLOBAL' if pico_view_global else 'LOCAL'}")
                    except (OSError, ValueError):
                        print("VIEW: sender unavailable")
                previous_view_switch = view_switch_pressed

                if (a_pressed and not previous_a) or terminal_toggle:
                    if args.calibrate_floor_init:
                        if grip <= GRIP_THRESHOLD:
                            measured = joints.copy()
                            ik.sync(measured)
                            ee = ik.robot.get_T_world_frame("link6")
                            payload = {
                                "joint_deg": measured.round(3).tolist(),
                                "link6_xyz_from_base_m": ee[:3, 3].round(6).tolist(),
                                "base_height_above_floor_m": 0.38,
                                "saved_wall_ns": time.time_ns(),
                                "lateral_translation_locked": True,
                            }
                            FLOOR_INIT_CONFIG.write_text(
                                json.dumps(payload, indent=2), encoding="utf-8"
                            )
                            print(f"A SAVED FLOOR INIT: {payload['joint_deg']}")
                        else:
                            print("A RELEASE RIGHT GRIP TO SAVE")
                    elif args.calibrate_drop_pose:
                        if grip <= GRIP_THRESHOLD:
                            measured = joints.copy()
                            ik.sync(measured)
                            ee = ik.robot.get_T_world_frame("link6")
                            payload = {
                                "joint_deg": measured.round(3).tolist(),
                                "link6_xyz_from_base_m": ee[:3, 3].round(6).tolist(),
                                "link6_quaternion_xyzw": Rotation.from_matrix(
                                    ee[:3, :3]
                                ).as_quat().round(8).tolist(),
                                "link6_transform": ee.round(8).tolist(),
                                "saved_wall_ns": time.time_ns(),
                                "purpose": "pre_drop_above_blue_cup",
                            }
                            CUP_DROP_CONFIG.write_text(
                                json.dumps(payload, indent=2), encoding="utf-8"
                            )
                            print(f"A SAVED CUP DROP POSE: {payload['joint_deg']}")
                        else:
                            print("A RELEASE RIGHT GRIP TO SAVE")
                    elif recording:
                        now = time.monotonic()
                        if now - phase_started_monotonic < MIN_EPISODE_DURATION_S:
                            print("A WAIT")
                            previous_a = a_pressed
                            continue
                        if episode_phase == "locomotion":
                            mark_episode_event("locomotion_complete", xr_ns)
                            print("A2 APPROACH -> MANIP INIT")
                            move_piper_to_manipulation_init()
                            episode_phase = "manipulation"
                            phase_started_monotonic = time.monotonic()
                            pending_event = "locomotion_complete"
                        elif episode_phase == "manipulation":
                            mark_episode_event("pick_complete", xr_ns)
                            episode_phase = "transport"
                            phase_started_monotonic = time.monotonic()
                            pending_event = "pick_complete"
                            print("A3 PICK -> INIT -> CUP -> DROP -> INIT")
                            move_piper_to_init("fold_after_pick")
                            move_piper_to_cup_drop()
                            if piper is not None:
                                send_gripper(piper, GRIPPER_OPEN_MM)
                            mark_episode_event(
                                "place_complete", int(xrt.get_time_stamp_ns())
                            )
                            print("DROP -> RETREAT -> INIT")
                            retreat_from_cup_after_drop()
                            move_piper_to_init("return_after_drop")
                            finish_episode(
                                reason="place_complete", returned_to_home=True
                            )
                            print("SAVED -> RESET BALL/TRACER -> A1 NEXT")
                        else:
                            raise RuntimeError(f"unexpected episode phase: {episode_phase}")
                    elif grip <= GRIP_THRESHOLD and left_grip <= TRACER_GRIP_THRESHOLD and sticks_neutral:
                        if manual_setup:
                            print("A1 RESET -> INIT")
                            move_piper_to_init()
                        start_episode()
                    else:
                        print("A RELEASE GRIPS/STICKS")
                previous_a = a_pressed

                y_pressed = bool(xrt.get_Y_button())
                tracer_command = {"linear_mps": 0.0, "angular_rad_s": 0.0}
                if tracer_bridge is not None:
                    tracer_state = tracer_bridge.snapshot()
                    if not tracer_state["bridge_alive"]:
                        raise RuntimeError("Tracer ROS bridge stopped unexpectedly")
                    status_fresh = started - tracer_state["stamp"] <= TRACER_INPUT_TIMEOUT_S
                    if status_fresh and tracer_state["vehicle"] == 1:
                        tracer_latched = True
                    if tracer_reset_sent_at is not None and tracer_state["stamp"] > tracer_reset_sent_at:
                        if tracer_state["vehicle"] == 0 and tracer_state["error"] == 0:
                            enable_commanded_mode(tracer_can)
                            tracer_latched = False
                            tracer_reset_sent_at = None
                            print("Tracer E-stop cleared")
                        elif started - tracer_reset_sent_at > 1.0:
                            tracer_reset_sent_at = None
                            print("Tracer E-stop remains active")

                    if y_pressed and not previous_y and tracer_latched:
                        if left_grip <= TRACER_GRIP_THRESHOLD and sticks_neutral and not xr_stale:
                            tracer_bridge.stop_motion()
                            send_hardware_reset(tracer_can)
                            tracer_reset_sent_at = time.monotonic()
                            print("Y: Tracer E-stop reset requested")
                        else:
                            print("Y ignored: release left grip and center sticks")

                    tracer_ready = (
                        status_fresh
                        and tracer_state["vehicle"] == 0
                        and tracer_state["error"] == 0
                        and not tracer_latched
                        and not xr_stale
                        # Tracer is available in both pre-recording SET BALL
                        # states, including the first one where manual_setup is
                        # still false. PiPER remains locked there because its
                        # clutch condition still requires recording/manual_setup.
                        and (recording or manual_setup or episode_phase is None)
                        and not calibration_mode
                        and episode_phase != "manipulation"
                    )
                    if left_grip > TRACER_GRIP_THRESHOLD and tracer_ready:
                        tracer_command["linear_mps"] = tracer_axis(left_axis[1]) * TRACER_LINEAR_MPS
                        tracer_command["angular_rad_s"] = -tracer_axis(right_axis[0]) * TRACER_ANGULAR_RAD_S
                    tracer_bridge.set_command(
                        tracer_command["linear_mps"], tracer_command["angular_rad_s"]
                    )
                previous_y = y_pressed
                previous_b = b_pressed

                state = None
                if piper is not None:
                    state = piper_snapshot(piper)
                    if state["error_code"] != 0 or state["arm_status"] != 0:
                        unsafe_stop = True
                        raise RuntimeError(f"unsafe PiPER state: {state}")
                    if len(state["enabled"]) != 6 or not all(state["enabled"]):
                        unsafe_stop = True
                        raise RuntimeError(f"PiPER motor enable lost: {state['enabled']}")
                    joints, stamp, hz = piper_joints(piper)
                    if hz <= 0:
                        unsafe_stop = True
                        raise RuntimeError("PiPER feedback absent")
                    if stamp > feedback_stamp:
                        feedback_stamp = stamp
                        last_feedback_change = time.monotonic()
                    elif time.monotonic() - last_feedback_change > FEEDBACK_STALE_S:
                        unsafe_stop = True
                        raw_can = raw_piper_watchdog.snapshot()
                        raw_age = raw_can["feedback_age_ms"]
                        if raw_age is not None and raw_age <= FEEDBACK_STALE_S * 1000:
                            raise RuntimeError(
                                f"stale SDK parser while raw PiPER CAN is fresh: {raw_can}"
                            )
                        raise RuntimeError(f"stale raw PiPER CAN feedback: {raw_can}")
                    ik.sync(joints)

                grip_active = grip > (GRIP_RELEASE_THRESHOLD if clutch else GRIP_THRESHOLD)
                piper_phase_ready = (
                    not recording
                    or episode_phase in ("manipulation", "transport")
                )
                next_clutch = (
                    grip_active and (recording or manual_setup)
                    and piper_phase_ready and not xr_stale
                )
                if next_clutch and not clutch:
                    ik.activate(pose, tracker_pose)
                    last_command = joints.copy()
                    print(f"CLUTCH ON (elbow tracker={'ON' if tracker_available else 'OFF'})")
                elif clutch and not next_clutch:
                    ik.deactivate()
                    print("CLUTCH OFF")
                clutch = next_clutch

                target = last_command.copy()
                trigger_input = min(max(trigger, 0.0), 1.0)
                if trigger_input <= GRIPPER_DEADZONE:
                    trigger_input = 0.0
                elif trigger_input >= 1.0 - GRIPPER_DEADZONE:
                    trigger_input = 1.0
                # The ZIP path sends the trigger-mapped target directly. The
                # previous extra low-pass and 2 mm/cycle limiter could leave a
                # short full pull above the physical 0 mm closed endpoint.
                filtered_trigger = trigger_input
                gripper_target = compute_gripper_target(filtered_trigger)
                if clutch:
                    solved = ik.solve(
                        pose, tracker_pose,
                        locked_lateral_y=calibration_lateral_y
                        if args.calibrate_floor_init else None,
                    )
                    target = validate_joint_target(solved)
                    if args.calibrate_floor_init and calibration_locked_joints_deg is not None:
                        # Calibration deliberately exposes only J2/J3/J5. Restore
                        # J1/J4/J6 every cycle so neither base yaw nor wrist roll
                        # can be introduced as an IK compensation.
                        target[[0, 3, 5]] = calibration_locked_joints_deg[[0, 3, 5]]
                    if piper is not None:
                        send_arm(piper, target)
                    last_command = target.copy()
                if piper is not None and (recording or manual_setup):
                    if calibration_mode and calibration_gripper_mm is not None:
                        gripper_target = calibration_gripper_mm
                    send_gripper(piper, gripper_target)

                wrist_camera_ref = camera.latest() if camera is not None else None
                global_camera_ref = global_camera.latest() if global_camera is not None else None
                if recording:
                    collection_label = ball_target_label(collection_plan_index)
                    preview_status = {
                        "locomotion": "MOVE  A2",
                        "manipulation": "PICK  A3",
                        "transport": "AUTO DROP",
                    }.get(episode_phase, "REC")
                    preview_status = f"{collection_label} | {preview_status}"
                elif args.calibrate_floor_init:
                    preview_status = "FLOOR INIT | A SAVE"
                elif args.calibrate_drop_pose:
                    preview_status = "CUP POSE | A SAVE"
                elif manual_setup:
                    preview_status = (
                        f"{ball_target_label(collection_plan_index)} | SET BALL  A1"
                    )
                else:
                    preview_status = (
                        f"{ball_target_label(collection_plan_index)} | SET BALL  A1"
                    )
                setup_target_cm = None
                if (
                    not recording and not calibration_mode
                    and collection_plan_index < len(BALL_POSITION_PLAN)
                ):
                    setup_target_cm = BALL_POSITION_PLAN[collection_plan_index]
                if not show_camera_preview(
                    camera, global_camera, recording, preview_status,
                    setup_target_cm=setup_target_cm,
                ):
                    print("Camera preview: Q/Esc requested safe session exit")
                    stop_requested.set()
                sample = {
                    "timestamp_monotonic_ns": time.monotonic_ns(),
                    "timestamp_wall_ns": time.time_ns(),
                    "timestamp_xr_ns": xr_ns,
                    "right_pose_xyzw": pose.tolist(),
                    "right_elbow_tracker_serial": args.right_elbow_tracker_serial or None,
                    "right_elbow_tracker_pose": tracker_pose.tolist() if tracker_pose is not None else None,
                    "right_elbow_tracker_active": bool(clutch and ik.ref_tracker_pos is not None),
                    "right_grip": grip,
                    "right_trigger": trigger,
                    "filtered_right_trigger": filtered_trigger,
                    "right_axis": right_axis,
                    "left_axis": left_axis,
                    "left_grip": left_grip,
                    "recording": recording,
                    "episode_phase": episode_phase,
                    "event": pending_event,
                    "control_source": "teleoperation",
                    "trajectory_subphase": None,
                    "clutch_active": clutch,
                    "commanded_joint_deg": target.tolist(),
                    "commanded_gripper_mm": gripper_target,
                    "piper_joint_deg": joints.tolist(),
                    "piper_status": state,
                    "piper_raw_can": raw_piper_watchdog.snapshot()
                    if raw_piper_watchdog is not None else None,
                    "tracer_command": tracer_command,
                    "tracer_status": dict(tracer_state) if tracer_bridge is not None else None,
                    "dry_run": args.dry_run,
                    # Keep camera_frame_reference as the wrist-camera compatibility alias.
                    "camera_frame_reference": wrist_camera_ref,
                    "wrist_camera_frame_reference": wrist_camera_ref,
                    "global_camera_frame_reference": global_camera_ref,
                }
                if recording and log is not None:
                    log.write(json.dumps(sample, ensure_ascii=False) + "\n")
                    sample_count += 1
                    pending_event = None
                elapsed = time.monotonic() - started
                if elapsed < DT:
                    time.sleep(DT - elapsed)
    except EmergencyStopRequested as exc:
        failure_reason = f"{type(exc).__name__}: {exc}"
        # B/Ctrl-C during an automatic move is still a normal guarded exit:
        # hold immediately, then let finally return to ZIP start and release.
        print(f"B: motion held; guarded ZIP-start return ({exc})")
    except Exception as exc:
        failure_reason = f"{type(exc).__name__}: {exc}"
        # Unknown failures must not introduce additional robot motion.
        unsafe_stop = True
        raise
    finally:
        # Stop producing episode files immediately, but do not wait for USB.
        # Robot return and torque release take priority over draining queues.
        if recording:
            try:
                if camera is not None:
                    camera.pause_recording(wait_for_writes=False)
                if global_camera is not None:
                    global_camera.pause_recording(wait_for_writes=False)
            except Exception as exc:
                cleanup_errors.append(f"record_pause: {type(exc).__name__}: {exc}")
        try:
            if tracer_bridge is not None:
                tracer_bridge.stop_motion()
        except Exception as exc:
            cleanup_errors.append(f"tracer_hold: {type(exc).__name__}: {exc}")
        if piper is not None and not unsafe_stop:
            try:
                print("Returning to ZIP start pose before storage cleanup...")
                wait_for_pose(piper, JOINTS_START_DEG, monitor_b=False)
                return_confirmed = True
                piper.MotionCtrl_2(0x00, 0x01, INIT_MOVE_SPEED_PERCENT, 0x00)
                time.sleep(0.5)
                piper.DisableArm(7)
                time.sleep(1.0)
                torque_released = not any(piper.GetArmEnableStatus())
                print("Return confirmed; arm torque released. Finishing storage cleanup...")
            except Exception as exc:
                unsafe_stop = True
                cleanup_errors.append(f"robot: {type(exc).__name__}: {exc}")

        if recording:
            try:
                finish_episode(
                    status="incomplete", reason=failure_reason or "session exit"
                )
            except Exception as exc:
                # The arm has already returned and released torque. Preserve a
                # partially written episode for manual recovery, but continue
                # releasing all remaining software/device resources.
                cleanup_errors.append(
                    f"episode_cleanup: {type(exc).__name__}: {exc}"
                )
        try:
            if tracer_bridge is not None:
                tracer_bridge.close()
        except Exception as exc:
            cleanup_errors.append(f"tracer_stop: {type(exc).__name__}: {exc}")
        try:
            if camera is not None:
                camera.stop()
        except Exception as exc:
            cleanup_errors.append(f"camera: {type(exc).__name__}: {exc}")
        try:
            if global_camera is not None:
                global_camera.stop()
        except Exception as exc:
            cleanup_errors.append(f"global_camera: {type(exc).__name__}: {exc}")
        try:
            xrt.close()
        except Exception as exc:
            cleanup_errors.append(f"xr: {type(exc).__name__}: {exc}")
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass
        if raw_piper_watchdog is not None:
            raw_piper_watchdog.stop()
        if tracer_can is not None:
            tracer_can.close()
        if piper is not None:
            piper.DisconnectPort()


if __name__ == "__main__":
    main()
