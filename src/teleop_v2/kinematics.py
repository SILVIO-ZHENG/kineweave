"""Dimension-independent forward kinematics and bounded damped-least-squares IK.
This module operates only on named model joints. It neither imports a robot SDK
nor sends commands. IK is geometric: balance, contacts and collision acceptance
belong to explicit robot controllers and the robot-side execution boundary.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any

import numpy as np

from .types import Pose, RobotProfile, finite_vector


def _skew(value: np.ndarray) -> np.ndarray:
    x, y, z = value
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)


def _rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """Rodrigues rotation around a normalized axis."""
    skew = _skew(axis)
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def _rpy_matrix(rpy: Any) -> np.ndarray:
    """URDF fixed-axis RPY convention: Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    roll, pitch, yaw = finite_vector(rpy, 3, "rpy")
    return _rotation(np.array([0., 0., 1.]), yaw) @ _rotation(np.array([0., 1., 0.]), pitch) @ _rotation(np.array([1., 0., 0.]), roll)


def _matrix_rpy(rotation: np.ndarray) -> np.ndarray:
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1, 1)))
    if abs(math.cos(pitch)) > 1e-8:
        return np.array([math.atan2(rotation[2, 1], rotation[2, 2]), pitch, math.atan2(rotation[1, 0], rotation[0, 0])])
    return np.array([math.atan2(-rotation[1, 2], rotation[1, 1]), pitch, 0.0])


def pose_matrix(pose: Pose) -> np.ndarray:
    """Convert an XYZW pose into a homogeneous matrix."""
    x, y, z, w = np.array(pose.quaternion) / np.linalg.norm(pose.quaternion)
    result = np.eye(4)
    result[:3, :3] = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])
    result[:3, 3] = pose.position
    return result


def _validate_transform(value: Any) -> np.ndarray:
    result = np.asarray(value, dtype=float)
    if result.shape != (4, 4) or not np.isfinite(result).all():
        raise ValueError("Task transform must be a finite 4x4 matrix")
    if not np.allclose(result[3], [0, 0, 0, 1], atol=1e-8):
        raise ValueError("Invalid homogeneous transform bottom row")
    rotation = result[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6) or abs(np.linalg.det(rotation) - 1) > 1e-6:
        raise ValueError("Task orientation must be a proper rotation")
    return result.copy()


def matrix_pose(matrix: np.ndarray) -> Pose:
    """Stable XYZW conversion, including rotations near pi."""
    transform = _validate_transform(matrix)
    rotation = transform[:3, :3]
    trace = np.trace(rotation)
    if trace > 0:
        scale = math.sqrt(1 + trace) * 2
        q = np.array([(rotation[2, 1]-rotation[1, 2])/scale, (rotation[0, 2]-rotation[2, 0])/scale,
                      (rotation[1, 0]-rotation[0, 1])/scale, scale/4])
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            scale = math.sqrt(max(0, 1 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2])) * 2
            q = np.array([scale/4, (rotation[0, 1]+rotation[1, 0])/scale,
                          (rotation[0, 2]+rotation[2, 0])/scale, (rotation[2, 1]-rotation[1, 2])/scale])
        elif index == 1:
            scale = math.sqrt(max(0, 1 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2])) * 2
            q = np.array([(rotation[0, 1]+rotation[1, 0])/scale, scale/4,
                          (rotation[1, 2]+rotation[2, 1])/scale, (rotation[0, 2]-rotation[2, 0])/scale])
        else:
            scale = math.sqrt(max(0, 1 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1])) * 2
            q = np.array([(rotation[0, 2]+rotation[2, 0])/scale, (rotation[1, 2]+rotation[2, 1])/scale,
                          scale/4, (rotation[1, 0]-rotation[0, 1])/scale])
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q *= -1
    return Pose(tuple(float(x) for x in transform[:3, 3]), tuple(float(x) for x in q))


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """SO(3) logarithm without singular division near pi."""
    transform = np.eye(4)
    transform[:3, :3] = rotation
    quaternion = np.array(matrix_pose(transform).quaternion)
    norm = np.linalg.norm(quaternion[:3])
    if norm < 1e-10:
        return 2 * quaternion[:3]
    return quaternion[:3] * (2 * math.atan2(norm, quaternion[3]) / norm)


@dataclass(frozen=True)
class IKResult:
    """Best bounded solution and per-task errors; an unreachable target remains a failure.
    Errors contain position error in metres and orientation error in radians.
    """

    position: np.ndarray
    success: bool
    errors: dict[str, dict[str, float]]
    iterations: int


class KinematicModel:
    """Compile serial chains with arbitrary joint counts. Multiple tasks are solved
    jointly, so a shared waist coordinate is not independently overwritten.
    The default solver is intentionally dependency-light and can be replaced by
    a constrained Pinocchio/QP backend behind the same public methods.
    """

    def __init__(self, profile: RobotProfile) -> None:
        self.profile = profile
        self._lower = np.array([j.lower for j in profile.joints])
        self._upper = np.array([j.upper for j in profile.joints])
        self._chains = {}
        for name, chain in profile.chains.items():
            base = self._fixed(chain.get("base_xyz", [0, 0, 0]), chain.get("base_rpy", [0, 0, 0]))
            tool = self._fixed(chain.get("tool_xyz", [0, 0, 0]), chain.get("tool_rpy", [0, 0, 0]))
            segments = []
            seen = set()
            for segment in chain.get("segments", []):
                index = profile.index(segment["joint"])
                if index in seen:
                    raise ValueError(f"Joint occurs twice in chain {name}: {segment['joint']}")
                seen.add(index)
                kind = segment.get("type", "revolute")
                if kind not in {"revolute", "prismatic"}:
                    raise ValueError(f"Unsupported segment type {kind}; fixed origins belong in xyz/rpy")
                expected_unit = "m" if kind == "prismatic" else "rad"
                if profile.joints[index].unit != expected_unit:
                    raise ValueError(f"Joint unit does not match its kinematic type: {segment['joint']}")
                axis = finite_vector(segment.get("axis", [0, 0, 1]), 3, "axis")
                norm = np.linalg.norm(axis)
                if norm < 1e-12:
                    raise ValueError("Joint axis must be nonzero")
                segments.append((index, self._fixed(segment.get("xyz", [0, 0, 0]), segment.get("rpy", [0, 0, 0])), axis/norm, kind))
            if not segments:
                raise ValueError(f"Kinematic chain {name} must contain an actuated joint")
            self._chains[name] = (base, segments, tool)

    @staticmethod
    def _fixed(xyz: Any, rpy: Any) -> np.ndarray:
        result = np.eye(4)
        result[:3, 3] = finite_vector(xyz, 3, "xyz")
        result[:3, :3] = _rpy_matrix(rpy)
        return result

    def _evaluate(self, q: np.ndarray, chain: str) -> tuple[np.ndarray, list, list]:
        if chain not in self._chains:
            raise ValueError(f"Unknown kinematic task: {chain}")
        base, segments, tool = self._chains[chain]
        current = base.copy()
        points = [current[:3, 3].tolist()]
        axes = []
        for index, origin, axis, kind in segments:
            current = current @ origin
            axes.append((index, current[:3, 3].copy(), current[:3, :3] @ axis, kind))
            motion = np.eye(4)
            if kind == "prismatic":
                motion[:3, 3] = axis * q[index]
            else:
                motion[:3, :3] = _rotation(axis, q[index])
            current = current @ motion
            points.append(current[:3, 3].tolist())
        current = current @ tool
        points.append(current[:3, 3].tolist())
        return current, axes, points

    def forward(self, q: Any, chain: str) -> np.ndarray:
        """End-effector pose in the robot base frame."""
        return self._evaluate(finite_vector(q, self.profile.dimension, "joint position"), chain)[0]

    def points(self, q: Any) -> dict[str, list[list[float]]]:
        """Lightweight preview points, not collision meshes."""
        position = finite_vector(q, self.profile.dimension, "joint position")
        return {name: self._evaluate(position, name)[2] for name in self._chains}

    def jacobian(self, q: Any, chain: str) -> np.ndarray:
        """Base-frame geometric Jacobian, with one column per profile coordinate.
        Inactive joints have zero columns, preserving explicit joint ordering.
        """
        position = finite_vector(q, self.profile.dimension, "joint position")
        end, axes, _ = self._evaluate(position, chain)
        jacobian = np.zeros((6, self.profile.dimension))
        for index, origin, axis, kind in axes:
            if kind == "prismatic":
                jacobian[:3, index] = axis
            else:
                jacobian[:3, index] = np.cross(axis, end[:3, 3] - origin)
                jacobian[3:, index] = axis
        return jacobian

    def solve(
        self,
        q: Any,
        targets: dict[str, np.ndarray],
        *,
        max_iterations: int = 60,
        position_tolerance: float = 0.003,
        orientation_tolerance: float = 0.04,
        orientation_weight: float = 0.25,
        damping: float = 0.04,
        max_step: float = 0.15,
    ) -> IKResult:
        """Solve all requested tasks jointly using damped least squares and bounded
        backtracking. Joint limits apply on every trial; no future command is
        emitted here. Return the best finite solution even when convergence fails.
        The runtime still applies velocity/acceleration limits before dispatch.
        """
        seed = finite_vector(q, self.profile.dimension, "IK seed")
        if max_iterations <= 0 or max_iterations > 1000:
            raise ValueError("max_iterations must be between 1 and 1000")
        settings = (position_tolerance, orientation_tolerance, orientation_weight, damping, max_step)
        if not all(math.isfinite(x) and x > 0 for x in settings):
            raise ValueError("IK tolerances, weights, damping and step must be positive and finite")
        desired = {name: _validate_transform(value) for name, value in targets.items()}
        if set(desired) - set(self._chains):
            raise ValueError(f"Unknown IK tasks: {sorted(set(desired) - set(self._chains))}")
        if not desired:
            return IKResult(seed, True, {}, 0)

        # Clamp only active task coordinates; unrelated measured joints stay intact.
        active = sorted({item[0] for name in desired for item in self._chains[name][1]})
        current = seed.copy()
        current[active] = np.clip(current[active], self._lower[active], self._upper[active])

        def evaluate(position: np.ndarray, with_jacobian: bool = False):
            residuals, jacobians, errors = [], [], {}
            for name, target in desired.items():
                actual = self.forward(position, name)
                translation = target[:3, 3] - actual[:3, 3]
                orientation = rotation_vector(target[:3, :3] @ actual[:3, :3].T)
                errors[name] = {"position": float(np.linalg.norm(translation)), "orientation": float(np.linalg.norm(orientation))}
                residuals.extend((translation, orientation * orientation_weight))
                if with_jacobian:
                    jacobian = self.jacobian(position, name)[:, active]
                    jacobian[3:] *= orientation_weight
                    jacobians.append(jacobian)
            error = np.concatenate(residuals)
            return error, np.vstack(jacobians) if with_jacobian else None, errors

        iterations = 0
        for iterations in range(max_iterations + 1):
            residual, jacobian, errors = evaluate(current, True)
            success = all(e["position"] <= position_tolerance and e["orientation"] <= orientation_tolerance for e in errors.values())
            if success or iterations == max_iterations:
                return IKResult(current, success, errors, iterations)
            step = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping*damping*np.eye(len(residual)), residual)
            largest = float(np.max(np.abs(step)))
            if largest > max_step:
                step *= max_step/largest
            improved = False
            baseline = float(residual @ residual)
            for scale in (1.0, 0.5, 0.25, 0.1, 0.03):
                trial = current.copy()
                trial[active] = np.clip(current[active] + scale*step, self._lower[active], self._upper[active])
                trial_error, _, _ = evaluate(trial)
                if float(trial_error @ trial_error) < baseline - 1e-14:
                    current = trial
                    improved = True
                    break
            if not improved:
                return IKResult(current, False, errors, iterations + 1)
        raise AssertionError("IK iteration bound must return a result")
