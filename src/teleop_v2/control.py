"""Calibration, retargeting and command constraints without vendor SDK dependencies.
The guard checks joint-space limits only. It is not a collision or balance solver;
hardware adapters must retain platform-specific protection and ownership checks.
"""

from __future__ import annotations

import math
import numpy as np

from .kinematics import KinematicModel, pose_matrix
from .types import InputFrame, RobotProfile, finite_vector


def rotation_step(vector: np.ndarray) -> np.ndarray:
    """Exponential map of a rotation-vector increment."""
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3)
    x, y, z = vector / angle
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


class Retargeter:
    """Map human movement through relative anchoring.

    Releasing enable/clutch or losing tracking discards the previous anchor.
    The next enabled packet starts at the measured robot pose, preventing a jump
    when the operator moves while control is paused. Integration uses real elapsed
    time and is never allowed to catch up a backlog of device packets.
    """

    def __init__(self, profile: RobotProfile, model: KinematicModel, scale: float = 1.0):
        if not math.isfinite(scale) or not 0 < scale <= 3:
            raise ValueError("Retargeting scale must be in (0, 3]")
        self.profile, self.model, self.scale = profile, model, scale
        self.reset()

    def reset(self) -> None:
        self.anchors: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        self.targets: dict[str, np.ndarray] = {}

    @staticmethod
    def _device_pose(frame: InputFrame, name: str) -> np.ndarray:
        matrix = pose_matrix(frame.poses[name])
        if frame.frame_id == "webxr":

            # WebXR right/up/backward -> robot forward/left/up, a proper rotation.
            basis = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], dtype=float)
            matrix[:3, 3] = basis @ matrix[:3, 3]
            matrix[:3, :3] = basis @ matrix[:3, :3] @ basis.T
        return matrix

    def map(self, frame: InputFrame, measured: np.ndarray, dt: float) -> tuple[dict, np.ndarray]:
        unknown = (set(frame.poses) | set(frame.twists)) - set(self.profile.chains)
        if unknown:
            raise ValueError(f"Unknown task chains: {sorted(unknown)}")
        if set(frame.poses) & set(frame.twists):
            raise ValueError("A task cannot use tracked pose and twist simultaneously")
        if set(frame.joints) - set(self.profile.joint_names):
            raise ValueError("Input contains unknown joint names")
        if set(frame.grasp) - set(self.profile.effectors):
            raise ValueError("Input contains unknown end effectors")
        requested = measured.copy()
        active = set(frame.poses) | set(frame.twists)
        for stale in set(self.targets) - active:
            self.targets.pop(stale, None)
            self.anchors.pop(stale, None)
        for name in frame.poses:
            device = self._device_pose(frame, name)
            if name not in self.anchors:
                self.anchors[name] = (device.copy(), self.model.forward(measured, name))
            device_zero, robot_zero = self.anchors[name]
            goal = robot_zero.copy()
            goal[:3, 3] += self.scale * (device[:3, 3] - device_zero[:3, 3])
            goal[:3, :3] = device[:3, :3] @ device_zero[:3, :3].T @ robot_zero[:3, :3]
            self.targets[name] = goal
        for name, values in frame.twists.items():
            twist = finite_vector(values, 6, "twist")

            # Bound intent magnitudes so device axis conventions cannot imply huge speeds.
            for part, maximum in ((slice(0, 3), 0.25), (slice(3, 6), 0.8)):
                norm = np.linalg.norm(twist[part])
                if norm > maximum:
                    twist[part] *= maximum / norm
            goal = self.targets.setdefault(name, self.model.forward(measured, name)).copy()
            goal[:3, 3] += twist[:3] * dt * self.scale
            goal[:3, :3] = rotation_step(twist[3:] * dt) @ goal[:3, :3]
            self.targets[name] = goal

        # A coordinate has one semantic owner; direct joints must not overwrite IK.
        ik_joints = {s["joint"] for name in active for s in self.profile.chains[name]["segments"]}
        if set(frame.joints) & ik_joints:
            raise ValueError("Direct joint targets conflict with an active IK task")
        for name, value in frame.joints.items():
            requested[self.profile.index(name)] = value
        for name, amount in frame.grasp.items():
            effector = self.profile.effectors[name]
            values = np.asarray(effector["open"]) * (1 - amount) + np.asarray(effector["closed"]) * amount
            if set(effector["joints"]) & set(frame.joints):
                raise ValueError("Grasp and direct hand-joint targets conflict")
            for joint, value in zip(effector["joints"], values):
                requested[self.profile.index(joint)] = value
        return {name: self.targets[name].copy() for name in active}, requested


class JointGuard:
    """Bound joint rates using actual elapsed time.

    A discrete braking envelope reserves one full step of stopping distance near
    position limits. An infeasible velocity interval raises instead of silently
    issuing a target that violates the declared acceleration limit.
    """

    def __init__(self, profile: RobotProfile, groups: list[str] | None = None):
        self.profile = profile
        if groups is not None and set(groups) - {j.group for j in profile.joints}:
            raise ValueError("Control groups must name groups declared in the robot profile")
        self.lower = np.array([j.lower for j in profile.joints])
        self.upper = np.array([j.upper for j in profile.joints])
        self.vmax = np.array([j.velocity for j in profile.joints])
        self.amax = np.array([j.acceleration for j in profile.joints])
        self.mask = np.zeros(profile.dimension, dtype=bool)
        self.mask[profile.indices(groups)] = True
        if not self.mask.any():
            raise ValueError("At least one valid control group is required")
        self.reset(profile.initial)

    def reset(self, measured: np.ndarray) -> None:
        self.position = finite_vector(measured, self.profile.dimension)
        self.velocity = np.zeros(self.profile.dimension)

    def step(self, target: np.ndarray, measured: np.ndarray, dt: float) -> tuple[np.ndarray, dict]:
        target = finite_vector(target, self.profile.dimension, "requested command")
        measured = finite_vector(measured, self.profile.dimension, "measured state")
        if not math.isfinite(dt) or not 0 < dt <= 0.1:
            raise ValueError("Control interval outside (0, 0.1] seconds")
        if np.any(measured < self.lower - 1e-4) or np.any(measured > self.upper + 1e-4):
            raise ValueError("Feedback outside profile limits; validate the hardware model")
        wanted = np.clip(target, self.lower, self.upper)
        wanted[~self.mask] = measured[~self.mask]
        velocity = (wanted - self.position) / dt
        low = np.maximum(-self.vmax, self.velocity - self.amax * dt)
        high = np.minimum(self.vmax, self.velocity + self.amax * dt)
        low = np.maximum(low, (self.lower - self.position) / dt)
        high = np.minimum(high, (self.upper - self.position) / dt)

        # Reserve dispatch travel plus continuous stopping distance: v*dt + v**2/(2*a) <= distance.
        safe_up = np.sqrt((self.amax * dt) ** 2 + 2 * self.amax * np.maximum(0, self.upper-self.position))
        safe_down = np.sqrt((self.amax * dt) ** 2 + 2 * self.amax * np.maximum(0, self.position-self.lower))
        high = np.minimum(high, safe_up - self.amax * dt)
        low = np.maximum(low, -safe_down + self.amax * dt)
        if np.any((low > high + 1e-10) & self.mask):
            raise ValueError("No feasible bounded joint step")
        velocity = np.clip(velocity, low, high)
        result = self.position + velocity * dt
        result[~self.mask] = measured[~self.mask]
        velocity[~self.mask] = 0
        self.position, self.velocity = result.copy(), velocity.copy()
        return result, {"modified": bool(np.max(np.abs(result-target)) > 1e-9),
                        "controlled_joints": int(self.mask.sum())}
