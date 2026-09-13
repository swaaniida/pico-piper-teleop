"""Policy-independent final action gate."""

from __future__ import annotations

from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Limits:
    # Conservative limits; remain inside PiPER's mechanical joint envelope.
    joint_min_rad: tuple[float, ...] = tuple(np.deg2rad([-150, 0, -170, -100, -70, -120]))
    joint_max_rad: tuple[float, ...] = tuple(np.deg2rad([150, 190, 0, 100, 70, 120]))
    max_joint_step_rad: float = float(np.deg2rad(1.0))
    gripper_min_mm: float = 0.0
    gripper_max_mm: float = 70.0
    max_gripper_step_mm: float = 2.0
    linear_max_mps: float = 0.15
    angular_max_rad_s: float = 0.40


class ActionGate:
    def __init__(self, limits: Limits = Limits()) -> None:
        self.limits = limits

    def apply(self, action: np.ndarray, measured_state: np.ndarray) -> np.ndarray:
        if action.shape != (9,) or measured_state.shape != (9,):
            raise RuntimeError("action/state contract must be 9D")
        if not np.all(np.isfinite(action)) or not np.all(np.isfinite(measured_state)):
            raise RuntimeError("non-finite action/state")
        low = np.asarray(self.limits.joint_min_rad)
        high = np.asarray(self.limits.joint_max_rad)
        if np.any(action[:6] < low) or np.any(action[:6] > high):
            raise RuntimeError("policy joint target is outside the operational envelope")
        safe = action.copy()
        delta = np.clip(safe[:6] - measured_state[:6], -self.limits.max_joint_step_rad, self.limits.max_joint_step_rad)
        safe[:6] = measured_state[:6] + delta
        safe[6] = np.clip(safe[6], self.limits.gripper_min_mm, self.limits.gripper_max_mm)
        safe[6] = measured_state[6] + np.clip(
            safe[6] - measured_state[6], -self.limits.max_gripper_step_mm, self.limits.max_gripper_step_mm
        )
        safe[7] = np.clip(safe[7], -self.limits.linear_max_mps, self.limits.linear_max_mps)
        safe[8] = np.clip(safe[8], -self.limits.angular_max_rad_s, self.limits.angular_max_rad_s)
        return safe
