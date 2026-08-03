#!/usr/bin/env python3
"""Compact right-PICO -> single PiPER teleoperation.

Functional single-arm adaptation of manipulation_pipeline-main:
XRoboToolkit -> full 6D Placo IK -> MOVE_J/JointCtrl + trigger gripper.
No LeRobot, bimanual wrapper, automatic pose move, limit rewrite, reset,
joint-limit rewrite or reset. A normal shutdown returns to the captured
enabled session pose before releasing torque.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import threading
import time
from typing import Any

import numpy as np
import placo
from scipy.spatial.transform import Rotation
import xrobotoolkit_sdk as xrt
from piper_sdk import C_PiperInterface_V2
from .realsense import WristRealSense


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "assets" / "piper_kinematics.urdf"
EPISODES = ROOT / "data" / "episodes"
CAN_NAME = "can0"
CONTROL_HZ = 50.0
DT = 1.0 / CONTROL_HZ

# Same interaction values as the ZIP single-arm path.
POSITION_SCALE = 1.0
ROTATION_SCALE = 1.0
GRIP_THRESHOLD = 0.9
GRIPPER_DEADZONE = 0.1
GRIPPER_OPEN_DEG = 101.4
GRIPPER_CLOSED_DEG = 0.0
MOVE_SPEED_PERCENT = 100
INIT_MOVE_SPEED_PERCENT = 50
JOINTS_START_DEG = np.array([-0.51, 0.49, 1.60, 3.96, 4.83, -8.33])
JOINTS_INIT_DEG = np.array([-1.86, 23.84, -60.0, 1.60, 50.65, 0.0])
POSE_MOVE_WAIT_S = 3.0
FEEDBACK_STALE_S = 0.10
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


def piper_joints(piper: C_PiperInterface_V2) -> tuple[np.ndarray, float, float]:
    msg = piper.GetArmJointMsgs()
    state = msg.joint_state
    joints = np.array([getattr(state, name.replace("joint", "joint_")) for name in JOINT_NAMES]) / 1000.0
    return joints.astype(float), float(msg.time_stamp), float(msg.Hz)


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
        "gripper_angle_deg": float(gripper.grippers_angle) / 1000.0,
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
        self.joint_task = self.solver.add_joints_task()
        self.joint_task.configure("joint_regularization", "soft", 1e-3)
        self.ref_controller_pos = None
        self.ref_controller_rot = None
        self.ref_ee = None
        self.pos_filter = None
        self.rot_filter = None

    def sync(self, joints_deg: np.ndarray) -> None:
        self.robot.state.q[:7] = np.array([0, 0, 0, 0, 0, 0, 1], dtype=float)
        joints_rad = np.radians(joints_deg)
        self.robot.state.q[7:13] = joints_rad
        self.joint_task.set_joints(dict(zip(JOINT_NAMES, joints_rad)))
        self.robot.update_kinematics()

    @staticmethod
    def world_pose(xr_pose: np.ndarray) -> tuple[np.ndarray, Rotation]:
        pos = R_HEADSET_WORLD @ xr_pose[:3]
        controller = Rotation.from_quat(xr_pose[3:7])
        frame = Rotation.from_matrix(R_HEADSET_WORLD)
        rot = frame * controller * frame.inv()
        return pos, rot

    def activate(self, xr_pose: np.ndarray) -> None:
        self.ref_controller_pos, self.ref_controller_rot = self.world_pose(xr_pose)
        self.pos_filter = OneEuroFilter(3, CONTROL_HZ)
        self.rot_filter = OneEuroFilter(3, CONTROL_HZ)
        self.pos_filter.filter(self.ref_controller_pos)
        self.rot_filter.filter(self.ref_controller_rot.as_rotvec())
        self.ref_ee = self.robot.get_T_world_frame("link6").copy()
        if self.ee_task is None:
            self.ee_task = self.solver.add_frame_task("link6", self.ref_ee)
            self.ee_task.configure("right_ee", "soft", 1.0)

    def deactivate(self) -> None:
        self.ref_controller_pos = None
        self.ref_controller_rot = None
        self.ref_ee = None
        self.pos_filter = None
        self.rot_filter = None

    def solve(self, xr_pose: np.ndarray) -> np.ndarray:
        if self.ref_ee is None:
            raise RuntimeError("clutch reference is absent")
        pos, rot = self.world_pose(xr_pose)
        pos = self.pos_filter.filter(pos)
        rot = Rotation.from_rotvec(self.rot_filter.filter(rot.as_rotvec()))
        delta_pos = (pos - self.ref_controller_pos) * POSITION_SCALE
        delta_rotvec = (rot * self.ref_controller_rot.inv()).as_rotvec() * ROTATION_SCALE
        delta_rot = Rotation.from_rotvec(delta_rotvec)
        target = self.ref_ee.copy()
        target[:3, 3] += delta_pos
        target[:3, :3] = delta_rot.as_matrix() @ self.ref_ee[:3, :3]
        self.ee_task.T_world_frame = target
        self.solver.solve(True)
        self.robot.update_kinematics()
        return np.degrees(self.robot.state.q[7:13].copy())


def send_arm(piper: C_PiperInterface_V2, joints_deg: np.ndarray) -> None:
    piper.MotionCtrl_2(0x01, 0x01, MOVE_SPEED_PERCENT, 0x00)
    piper.JointCtrl(*(int(round(v * 1000.0)) for v in joints_deg))


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
            piper.EmergencyStop(1)
            raise EmergencyStopRequested("B emergency stop requested during return")
        state = piper_snapshot(piper)
        if state["error_code"] != 0 or state["arm_status"] != 0:
            raise RuntimeError(f"return aborted by PiPER state: {state}")
        current, _, hz = piper_joints(piper)
        if hz > 0 and np.max(np.abs(current - target)) <= 0.5:
            return
        move_to_pose(piper, target)
        time.sleep(0.02)
    raise RuntimeError(f"return pose timeout; target={target.tolist()}")


def send_gripper(piper: C_PiperInterface_V2, trigger: float) -> float:
    trigger = 0.0 if trigger < GRIPPER_DEADZONE else min(max(trigger, 0.0), 1.0)
    target = GRIPPER_OPEN_DEG + trigger * (GRIPPER_CLOSED_DEG - GRIPPER_OPEN_DEG)
    piper.GripperCtrl(int(round(target * 1000.0)), 1000, 0x01, 0)
    return target


def countdown() -> None:
    print("WARNING: LIVE PiPER motion will start. Clear the workspace; keep B ready.")
    for remaining in (3, 2, 1):
        print(f"Starting in {remaining}...", flush=True)
        time.sleep(1.0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Compact right-PICO single-PiPER IK teleoperation")
    parser.add_argument("--dry-run", action="store_true", help="run XR+IK+logging without CAN commands")
    parser.add_argument("--zip-auto-pose", action="store_true", help="enable the ZIP's unvalidated connect/exit pose moves")
    parser.add_argument("--no-camera", action="store_true", help="disable wrist RealSense RGB-D recording")
    parser.add_argument("--camera-serial", default="816612060658")
    args = parser.parse_args()
    stop_requested = threading.Event()

    def request_stop(signum, _frame) -> None:
        print(f"Stop requested by signal {signum}; completing safe cleanup...", flush=True)
        stop_requested.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    if not args.dry_run and not os.path.exists(f"/sys/class/net/{CAN_NAME}"):
        raise RuntimeError("can0 is absent")

    EPISODES.mkdir(parents=True, exist_ok=True)
    episode = EPISODES / time.strftime("%Y%m%d_%H%M%S")
    episode.mkdir()
    samples_path = episode / "samples.jsonl"
    camera = None
    if not args.no_camera:
        camera = WristRealSense(serial=args.camera_serial)
        camera.start(episode)

    piper = None
    if args.dry_run:
        joints = np.zeros(6)
        feedback_stamp = 0.0
    else:
        piper = C_PiperInterface_V2(
            can_name=CAN_NAME, judge_flag=True, can_auto_init=True, dh_is_offset=0,
            start_sdk_joint_limit=False, start_sdk_gripper_limit=False,
        )
        piper.ConnectPort()
        time.sleep(1.0)
        joints, feedback_stamp, hz = piper_joints(piper)
        if hz <= 0:
            piper.DisconnectPort()
            if camera is not None:
                camera.stop()
            raise RuntimeError("PiPER feedback is absent")
    session_start_joints = joints.copy()

    ik = RightPicoIK()
    ik.sync(joints)
    clutch = False
    last_command = joints.copy()
    last_feedback_change = time.monotonic()
    unsafe_stop = False
    xrt.init()
    print("XRoboToolkit initialized. Right grip=clutch, trigger=gripper, B=stop.")
    if not args.dry_run:
        countdown()
        # Enable while holding only the measured joint pose.
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not all(piper.GetArmEnableStatus()):
            send_arm(piper, joints)
            piper.EnableArm(7)
            time.sleep(0.02)
        if not all(piper.GetArmEnableStatus()):
            raise RuntimeError("PiPER enable timeout")
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
        if args.zip_auto_pose:
            print("Moving to ZIP start pose...")
            move_to_pose(piper, JOINTS_START_DEG)
            time.sleep(POSE_MOVE_WAIT_S)
            print("Moving to ZIP teleop init pose...")
            move_to_pose(piper, JOINTS_INIT_DEG)
            time.sleep(POSE_MOVE_WAIT_S)
            joints, feedback_stamp, _ = piper_joints(piper)
            ik.sync(joints)
            last_command = joints.copy()

    metadata_path = episode / "metadata.json"
    metadata = {
        "episode_id": episode.name,
        "started_wall_ns": time.time_ns(),
        "dry_run": args.dry_run,
        "camera_enabled": camera is not None,
        "camera_serial": args.camera_serial if camera is not None else None,
        "control_hz": CONTROL_HZ,
        "position_scale": POSITION_SCALE,
        "rotation_scale": ROTATION_SCALE,
        "grip_threshold": GRIP_THRESHOLD,
        "zip_auto_pose": args.zip_auto_pose,
        "session_start_joint_deg": session_start_joints.tolist(),
        "status": "recording",
    }
    metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")
    failure_reason = None
    cleanup_errors = []
    return_confirmed = False
    torque_released = False
    print("READY")
    try:
        with samples_path.open("w", encoding="utf-8") as log:
            while not stop_requested.is_set():
                started = time.monotonic()
                pose = np.asarray(xrt.get_right_controller_pose(), dtype=float)
                grip = float(xrt.get_right_grip())
                trigger = float(xrt.get_right_trigger())
                xr_ns = int(xrt.get_time_stamp_ns())
                if pose.shape != (7,) or not np.all(np.isfinite(pose)):
                    raise RuntimeError("invalid right-controller pose")
                if bool(xrt.get_B_button()):
                    if piper is not None:
                        piper.EmergencyStop(1)
                    unsafe_stop = True
                    raise RuntimeError("B emergency stop requested")

                if bool(xrt.get_A_button()):
                    print("A: normal stop requested")
                    break

                state = None
                if piper is not None:
                    state = piper_snapshot(piper)
                    if state["error_code"] != 0 or state["arm_status"] != 0:
                        unsafe_stop = True
                        raise RuntimeError(f"unsafe PiPER state: {state}")
                    joints, stamp, hz = piper_joints(piper)
                    if hz <= 0:
                        unsafe_stop = True
                        raise RuntimeError("PiPER feedback absent")
                    if stamp > feedback_stamp:
                        feedback_stamp = stamp
                        last_feedback_change = time.monotonic()
                    elif time.monotonic() - last_feedback_change > FEEDBACK_STALE_S:
                        unsafe_stop = True
                        raise RuntimeError("stale PiPER feedback")
                    ik.sync(joints)

                next_clutch = grip > GRIP_THRESHOLD
                if next_clutch and not clutch:
                    ik.activate(pose)
                    last_command = joints.copy()
                    print("CLUTCH ON")
                elif clutch and not next_clutch:
                    ik.deactivate()
                    print("CLUTCH OFF")
                clutch = next_clutch

                target = last_command.copy()
                gripper_target = GRIPPER_OPEN_DEG + trigger * (GRIPPER_CLOSED_DEG - GRIPPER_OPEN_DEG)
                if clutch:
                    solved = ik.solve(pose)
                    target = solved
                    if piper is not None:
                        send_arm(piper, target)
                    last_command = target.copy()
                if piper is not None:
                    gripper_target = send_gripper(piper, trigger)

                sample = {
                    "timestamp_monotonic_ns": time.monotonic_ns(),
                    "timestamp_wall_ns": time.time_ns(),
                    "timestamp_xr_ns": xr_ns,
                    "right_pose_xyzw": pose.tolist(),
                    "right_grip": grip,
                    "right_trigger": trigger,
                    "right_axis": list(xrt.get_right_axis()),
                    "clutch_active": clutch,
                    "commanded_joint_deg": target.tolist(),
                    "commanded_gripper_deg": gripper_target,
                    "piper_joint_deg": joints.tolist(),
                    "piper_status": state,
                    "dry_run": args.dry_run,
                    "camera_frame_reference": camera.latest() if camera is not None else None,
                }
                log.write(json.dumps(sample, ensure_ascii=False) + "\n")
                elapsed = time.monotonic() - started
                if elapsed < DT:
                    time.sleep(DT - elapsed)
    except Exception as exc:
        failure_reason = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        print("Episode saved:", samples_path)
        try:
            if camera is not None:
                camera.stop()
        except Exception as exc:
            cleanup_errors.append(f"camera: {type(exc).__name__}: {exc}")
        try:
            xrt.close()
        except Exception as exc:
            cleanup_errors.append(f"xr: {type(exc).__name__}: {exc}")
        if piper is not None:
            try:
                # Normal exit: return first, confirm arrival, enter standby, then release torque.
                # B/error/stale feedback: never add motion or release supporting torque here.
                if not unsafe_stop:
                    return_target = JOINTS_START_DEG if args.zip_auto_pose else session_start_joints
                    label = "ZIP start pose" if args.zip_auto_pose else "session start pose"
                    print(f"Returning to {label} before torque release...")
                    wait_for_pose(piper, return_target, monitor_b=True)
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
        metadata.update({
            "ended_wall_ns": time.time_ns(),
            "status": "complete" if failure_reason is None and not cleanup_errors else "incomplete",
            "stop_reason": failure_reason or "requested",
            "unsafe_stop": unsafe_stop,
            "return_confirmed": return_confirmed,
            "torque_released": torque_released,
            "cleanup_errors": cleanup_errors,
        })
        metadata_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
