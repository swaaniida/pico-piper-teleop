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
import placo
from scipy.spatial.transform import Rotation
import xrobotoolkit_sdk as xrt
from piper_sdk import C_PiperInterface_V2
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
EPISODES = ROOT / "data" / "episodes"
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
# The reference ZIP uses J3=+1.60 deg, but this arm clamps J3 at its 0 deg
# boundary. Use the measured reachable equivalent so arrival can be confirmed.
JOINTS_START_DEG = np.array([-0.51, 0.49, 0.0, 3.96, 4.83, -8.33])
JOINTS_INIT_DEG = np.array([-1.86, 23.84, -60.0, 1.60, 50.65, 0.0])
FEEDBACK_STALE_S = 0.10
MIN_EPISODE_DURATION_S = 2.0
TRACER_CAN_NAME = "tracer_can"
PIPER_SDK_FEEDBACK_IDS = tuple(range(0x2A1, 0x2A9)) + tuple(range(0x261, 0x267))
PIPER_RAW_FEEDBACK_IDS = frozenset(PIPER_SDK_FEEDBACK_IDS)
TRACER_LINEAR_MPS = 0.15
TRACER_ANGULAR_RAD_S = 0.40
R_HEADSET_WORLD = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 7))


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

    def solve(self, xr_pose: np.ndarray, tracker_pose: np.ndarray | None = None) -> np.ndarray:
        if self.ref_ee is None:
            raise RuntimeError("clutch reference is absent")
        pos, rot = self.world_pose(xr_pose)
        pos = self.pos_filter.filter(pos)
        delta_pos = (pos - self.ref_controller_pos) * POSITION_SCALE
        target = self.ref_ee.copy()
        target[:3, 3] += delta_pos
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


def move_to_pose(piper: C_PiperInterface_V2, joints_deg: np.ndarray) -> None:
    """Match PiperFollower's direct initial/rest pose command."""
    piper.MotionCtrl_2(0x01, 0x01, INIT_MOVE_SPEED_PERCENT, 0x00)
    piper.JointCtrl(*(int(round(v * 1000.0)) for v in joints_deg))


class EmergencyStopRequested(RuntimeError):
    pass


def wait_for_pose(
    piper: C_PiperInterface_V2,
    target: np.ndarray,
    timeout_s: float = 10.0,
    monitor_b: bool = False,
) -> None:
    """Stream a return target until feedback confirms it before torque release."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
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
            return
        move_to_pose(piper, target)
        time.sleep(0.02)
    raise RuntimeError(f"return pose timeout; target={target.tolist()}")


def compute_gripper_target(trigger: float) -> float:
    """Map the processed trigger directly, as in the reference ZIP teleoperator."""
    return GRIPPER_OPEN_MM + trigger * (GRIPPER_CLOSED_MM - GRIPPER_OPEN_MM)


def send_gripper(piper: C_PiperInterface_V2, target_mm: float) -> None:
    target = min(max(target_mm, GRIPPER_CLOSED_MM), GRIPPER_OPEN_MM)
    piper.GripperCtrl(int(round(target * 1000.0)), 1000, 0x01, 0)


def countdown() -> None:
    print("WARNING: LIVE PiPER motion will start. Clear the workspace; keep B ready.")
    for remaining in (3, 2, 1):
        print(f"Starting in {remaining}...", flush=True)
        time.sleep(1.0)


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
    args = parser.parse_args()
    stop_requested = threading.Event()

    def request_stop(signum, _frame) -> None:
        print(f"Stop requested by signal {signum}; completing safe cleanup...", flush=True)
        stop_requested.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    if not args.dry_run and not os.path.exists(f"/sys/class/net/{CAN_NAME}"):
        raise RuntimeError(f"{CAN_NAME} is absent")
    if args.with_tracer and not os.path.exists(f"/sys/class/net/{TRACER_CAN_NAME}"):
        raise RuntimeError(f"{TRACER_CAN_NAME} is absent")

    EPISODES.mkdir(parents=True, exist_ok=True)
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
    previous_x = False
    previous_a = False
    previous_b = False
    recording = False
    episode_started_monotonic = None
    last_xr_ns = None
    last_xr_change = time.monotonic()
    tracker_was_available = None
    xrt_initialized = False

    def cleanup_failed_startup() -> None:
        """Release software resources without adding any robot motion."""
        nonlocal tracer_bridge, tracer_can, raw_piper_watchdog, xrt_initialized
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
            wait_for_pose(piper, JOINTS_INIT_DEG, monitor_b=True)
            joints, feedback_stamp, _ = piper_joints(piper)
            ik.sync(joints)
            last_command = joints.copy()
            last_feedback_change = time.monotonic()
        except Exception:
            cleanup_failed_startup()
            raise

    def start_episode() -> None:
        nonlocal episode, samples_path, metadata_path, metadata, log, camera, global_camera, recording, episode_started_monotonic
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
                camera = WristUvcCamera(
                    device=args.wrist_camera_device,
                    width=args.wrist_camera_width,
                    height=args.wrist_camera_height,
                    fps=args.wrist_camera_fps,
                )
                camera.start(episode)
            if not args.no_global_camera:
                global_camera = GlobalWebcam(
                    device=args.global_camera_device,
                    width=args.global_camera_width,
                    height=args.global_camera_height,
                    fps=args.global_camera_fps,
                )
                global_camera.start(episode)
            log = samples_path.open("w", encoding="utf-8")
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
            "right_elbow_tracker_serial": args.right_elbow_tracker_serial or None,
            "right_elbow_tracker_link": ELBOW_TRACKER_LINK if args.right_elbow_tracker_serial else None,
            "right_elbow_tracker_weight": ELBOW_TRACKER_WEIGHT if args.right_elbow_tracker_serial else None,
            "session_start_joint_deg": session_start_joints.tolist(),
            "episode_home_joint_deg": JOINTS_INIT_DEG.tolist(),
            "tracer_enabled": args.with_tracer,
            "tracer_ros_bridge": "separate_process" if args.with_tracer else None,
            "tracer_max_linear_mps": TRACER_LINEAR_MPS if args.with_tracer else None,
            "tracer_max_angular_rad_s": TRACER_ANGULAR_RAD_S if args.with_tracer else None,
            "status": "recording",
        }
        metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        recording = True
        episode_started_monotonic = time.monotonic()
        print(f"A: episode started: {episode.name}")

    def finish_episode(status: str = "complete", reason: str = "A") -> None:
        nonlocal episode, samples_path, metadata_path, metadata, log, camera, global_camera, recording, episode_started_monotonic
        if not recording:
            return
        if log is not None:
            log.flush()
            log.close()
        if camera is not None:
            camera.stop()
            camera = None
        if global_camera is not None:
            global_camera.stop()
            global_camera = None
        if status == "complete":
            metadata.update({"ended_wall_ns": time.time_ns(), "status": status, "stop_reason": reason,
                             "returned_to_episode_home": True})
            metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
            print(f"Episode saved: {samples_path}")
        else:
            # Failed/aborted demonstrations are not training data.
            shutil.rmtree(episode)
            print(f"Incomplete episode deleted: {episode} ({reason})")
        episode = samples_path = metadata_path = metadata = log = None
        recording = False
        episode_started_monotonic = None
    failure_reason = None
    cleanup_errors = []
    return_confirmed = False
    torque_released = False
    print("READY AT ZIP INIT — A/Enter toggles an episode; B exits safely.")
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
                if b_pressed and not previous_b:
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
                    print("B: motion held; ending session through guarded ZIP-start return")
                    break

                if (a_pressed and not previous_a) or terminal_toggle:
                    if recording:
                        if time.monotonic() - episode_started_monotonic < MIN_EPISODE_DURATION_S:
                            print("Episode stop ignored during 2s A-button debounce")
                            previous_a = a_pressed
                            continue
                        if tracer_bridge is not None:
                            tracer_bridge.stop_motion()
                        if clutch:
                            ik.deactivate()
                            clutch = False
                        if piper is not None:
                            print("A: returning to ZIP init before ending episode...")
                            wait_for_pose(piper, JOINTS_INIT_DEG, monitor_b=True)
                            joints, feedback_stamp, _ = piper_joints(piper)
                            ik.sync(joints)
                            last_command = joints.copy()
                            last_feedback_change = time.monotonic()
                        finish_episode()
                        print("READY — press A for the next episode; B exits.")
                    elif grip <= GRIP_THRESHOLD and left_grip <= TRACER_GRIP_THRESHOLD and sticks_neutral:
                        start_episode()
                    else:
                        print("A ignored: release both grips and center sticks")
                previous_a = a_pressed

                x_pressed = bool(xrt.get_X_button())
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

                    if x_pressed and not previous_x and tracer_latched:
                        if left_grip <= TRACER_GRIP_THRESHOLD and sticks_neutral and not xr_stale:
                            tracer_bridge.stop_motion()
                            send_hardware_reset(tracer_can)
                            tracer_reset_sent_at = time.monotonic()
                            print("X: Tracer E-stop reset requested")
                        else:
                            print("X ignored: release left grip and center sticks")

                    tracer_ready = (
                        status_fresh
                        and tracer_state["vehicle"] == 0
                        and tracer_state["error"] == 0
                        and not tracer_latched
                        and not xr_stale
                        and recording
                    )
                    if left_grip > TRACER_GRIP_THRESHOLD and tracer_ready:
                        tracer_command["linear_mps"] = tracer_axis(left_axis[1]) * TRACER_LINEAR_MPS
                        tracer_command["angular_rad_s"] = -tracer_axis(right_axis[0]) * TRACER_ANGULAR_RAD_S
                    tracer_bridge.set_command(
                        tracer_command["linear_mps"], tracer_command["angular_rad_s"]
                    )
                previous_x = x_pressed
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
                next_clutch = grip_active and recording and not xr_stale
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
                    solved = ik.solve(pose, tracker_pose)
                    target = validate_joint_target(solved)
                    if piper is not None:
                        send_arm(piper, target)
                    last_command = target.copy()
                if piper is not None and recording:
                    send_gripper(piper, gripper_target)

                wrist_camera_ref = camera.latest() if camera is not None else None
                global_camera_ref = global_camera.latest() if global_camera is not None else None
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
                elapsed = time.monotonic() - started
                if elapsed < DT:
                    time.sleep(DT - elapsed)
    except Exception as exc:
        failure_reason = f"{type(exc).__name__}: {exc}"
        if isinstance(exc, EmergencyStopRequested):
            unsafe_stop = True
        raise
    finally:
        if recording:
            finish_episode(status="incomplete", reason=failure_reason or "session exit")
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
        if raw_piper_watchdog is not None:
            raw_piper_watchdog.stop()
        if tracer_can is not None:
            tracer_can.close()
        if piper is not None:
            try:
                # B/Ctrl-C normal session exit: ZIP start, confirm, standby, release.
                # Error/stale feedback: never add motion or release supporting torque.
                if not unsafe_stop:
                    print("Returning to ZIP start pose before torque release...")
                    wait_for_pose(piper, JOINTS_START_DEG, monitor_b=False)
                    return_confirmed = True
                    piper.MotionCtrl_2(0x00, 0x01, INIT_MOVE_SPEED_PERCENT, 0x00)
                    time.sleep(0.5)
                    piper.DisableArm(7)
                    time.sleep(1.0)
                    torque_released = not any(piper.GetArmEnableStatus())
                    print("Return confirmed; arm torque released.")
            except Exception as exc:
                if isinstance(exc, EmergencyStopRequested):
                    unsafe_stop = True
                cleanup_errors.append(f"robot: {type(exc).__name__}: {exc}")
            finally:
                piper.DisconnectPort()


if __name__ == "__main__":
    main()
