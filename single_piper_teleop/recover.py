#!/usr/bin/env python3

"""Guarded, low-speed staged return to the known pre-teleop joint pose."""

import math
import os
import time
from typing import Any
import argparse

from piper_sdk import C_PiperInterface_V2


CAN_NAME = "piper_can"
MAX_WAYPOINT_DELTA = 5_000
WAYPOINT_TOLERANCE = 400
FINAL_TOLERANCE = 400
MOVE_SPEED_PERCENT = 50  # match the reference ZIP follower start/return speed
COMMAND_HZ = 100.0
WAYPOINT_TIMEOUT_S = 20.0
FEEDBACK_STALE_S = 0.15


def joints(piper: C_PiperInterface_V2) -> tuple[int, ...]:
    state = piper.GetArmJointMsgs().joint_state
    return tuple(int(getattr(state, f"joint_{i}")) for i in range(1, 7))


def status(piper: C_PiperInterface_V2) -> tuple[int, int, int]:
    state = piper.GetArmStatus().arm_status
    error: Any = getattr(state, "err_code", getattr(state, "_err_code", -1))
    return int(state.ctrl_mode), int(state.arm_status), int(error)


def enabled(piper: C_PiperInterface_V2) -> bool:
    values = piper.GetArmEnableStatus()
    return isinstance(values, (tuple, list)) and len(values) == 6 and all(values)


def require_ok(piper: C_PiperInterface_V2, last_stamp: float, last_change: float) -> tuple[float, float]:
    _, arm_state, error = status(piper)
    if error != 0 or arm_state != 0:
        raise RuntimeError(f"unsafe PiPER status: arm=0x{arm_state:02X}, error={error}")
    stamp = float(piper.GetArmJointMsgs().time_stamp)
    now = time.monotonic()
    if stamp > last_stamp:
        return stamp, now
    if now - last_change > FEEDBACK_STALE_S:
        raise RuntimeError("stale joint feedback")
    return last_stamp, last_change


def send(piper: C_PiperInterface_V2, target: tuple[int, ...], mode: int) -> None:
    piper.MotionCtrl_2(mode, 0x01, MOVE_SPEED_PERCENT, 0x00)
    piper.JointCtrl(*target)


def main() -> None:
    parser = argparse.ArgumentParser(description="Guarded return to an episode session-start pose")
    parser.add_argument(
        "--target", nargs=6, required=True, type=float,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="session_start_joint_deg from the episode metadata",
    )
    parser.add_argument(
        "--keep-enabled", action="store_true",
        help="hold the reached target instead of entering STANDBY and releasing torque",
    )
    args = parser.parse_args()
    target_final = tuple(int(round(value * 1000.0)) for value in args.target)
    if not os.path.exists(f"/sys/class/net/{CAN_NAME}"):
        raise RuntimeError(f"{CAN_NAME} is absent")

    piper = C_PiperInterface_V2(
        can_name=CAN_NAME,
        judge_flag=True,
        can_auto_init=True,
        dh_is_offset=0,
        start_sdk_joint_limit=False,
        start_sdk_gripper_limit=False,
    )
    piper.ConnectPort(piper_init=False)
    try:
        time.sleep(1.0)
        feedback = piper.GetArmJointMsgs()
        if not piper.get_connect_status() or float(feedback.Hz) <= 0:
            raise RuntimeError("PiPER joint feedback is absent")
        start = joints(piper)
        mode, arm_state, error = status(piper)
        if arm_state != 0 or error != 0:
            raise RuntimeError(f"unsafe initial status: {piper.GetArmStatus()}")
        largest = max(abs(target_final[i] - start[i]) for i in range(6))
        count = max(1, math.ceil(largest / MAX_WAYPOINT_DELTA))
        waypoints = [
            tuple(round(start[j] + (target_final[j] - start[j]) * step / count) for j in range(6))
            for step in range(1, count + 1)
        ]
        print("Start (0.001 deg):", start)
        print("Target (0.001 deg):", target_final)
        print(f"Waypoints: {count}; max step <= 5 deg; speed={MOVE_SPEED_PERCENT}%")
        print("No reset, recovery, configuration, Cartesian, or gripper command.")
        print("WARNING: live staged return will start; keep the workspace clear.")
        for remaining in (3, 2, 1):
            print(f"Starting in {remaining}...", flush=True)
            time.sleep(1.0)

        last_stamp = float(feedback.time_stamp)
        last_change = time.monotonic()
        # Enter CAN joint mode while holding the measured pose.
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            last_stamp, last_change = require_ok(piper, last_stamp, last_change)
            send(piper, start, 0x01)
            time.sleep(1.0 / COMMAND_HZ)
        enable_deadline = time.monotonic() + 3.0
        while time.monotonic() < enable_deadline and not enabled(piper):
            last_stamp, last_change = require_ok(piper, last_stamp, last_change)
            send(piper, start, 0x01)
            piper.EnableArm(7)
            time.sleep(1.0 / COMMAND_HZ)
        if not enabled(piper):
            raise RuntimeError("EnableArm timed out; no return waypoint sent")

        for index, target in enumerate(waypoints, 1):
            deadline = time.monotonic() + WAYPOINT_TIMEOUT_S
            while True:
                last_stamp, last_change = require_ok(piper, last_stamp, last_change)
                current = joints(piper)
                error_to_target = max(abs(target[j] - current[j]) for j in range(6))
                if error_to_target <= WAYPOINT_TOLERANCE:
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"waypoint {index} timeout; current={current}, target={target}")
                send(piper, target, 0x01)
                time.sleep(1.0 / COMMAND_HZ)
            print(f"Waypoint {index}/{count} reached:", current)

        result = joints(piper)
        final_error = tuple(result[i] - target_final[i] for i in range(6))
        if any(abs(value) > FINAL_TOLERANCE for value in final_error):
            raise RuntimeError(f"final tolerance exceeded: {final_error}")

        if args.keep_enabled:
            print("Result (0.001 deg):", result)
            print("Target reached; torque remains enabled.")
            return

        # Hold the achieved target while returning control mode to STANDBY.
        deadline = time.monotonic() + 0.5
        while time.monotonic() < deadline:
            last_stamp, last_change = require_ok(piper, last_stamp, last_change)
            send(piper, target_final, 0x00)
            time.sleep(1.0 / COMMAND_HZ)
        piper.DisableArm(7)
        time.sleep(1.0)
        print("Result (0.001 deg):", joints(piper))
        print("Final enabled:", piper.GetArmEnableStatus())
        print("Final status:", piper.GetArmStatus())
    finally:
        piper.DisconnectPort()


if __name__ == "__main__":
    main()
