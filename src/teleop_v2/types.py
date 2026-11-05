"""Shared contracts whose meaning is independent of robot SDKs and input devices.
Positions use metres, rotations use radians, and quaternions use XYZW order.
Joint vectors always follow ``RobotProfile.joint_names``; dimensions are dynamic.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any, Protocol
import math

import numpy as np


def finite_vector(value: Any, length: int, name: str = "vector") -> np.ndarray:
    """Reject broadcasting and non-finite coordinates."""
    result = np.asarray(value, dtype=np.float64)
    if result.shape != (length,) or not np.isfinite(result).all():
        raise ValueError(f"{name} must be a finite vector of length {length}")
    return result.copy()


@dataclass(frozen=True)
class JointSpec:
    """One recorded coordinate and its control group."""

    name: str
    lower: float
    upper: float
    velocity: float
    acceleration: float
    initial: float = 0.0
    unit: str = "rad"
    group: str = "arm"

    def __post_init__(self) -> None:
        values = (self.lower, self.upper, self.velocity, self.acceleration, self.initial)
        if not self.name or not all(math.isfinite(x) for x in values):
            raise ValueError("Joint names and finite limits are required")
        if self.lower >= self.upper or self.velocity <= 0 or self.acceleration <= 0:
            raise ValueError(f"Invalid limits for {self.name}")
        if not self.lower <= self.initial <= self.upper:
            raise ValueError(f"Initial position outside limits: {self.name}")
        if self.unit not in {"rad", "m"}:
            raise ValueError(f"Unsupported joint unit: {self.unit}")


@dataclass(frozen=True)
class RobotProfile:
    """Robot layout, kinematic chains and effector mapping.

    ``chains`` is a mapping of task names to serial-chain descriptions. Each chain
    contains ``base_xyz``, ``base_rpy``, ``segments`` and optional ``tool_xyz``.
    Each segment contains a named joint, fixed ``xyz/rpy``, axis and joint type.
    ``effectors`` maps a task (e.g. left) to joint names, open and closed values.
    Demonstration geometry must be explicitly replaced before hardware operation.
    """

    id: str
    display_name: str
    joints: tuple[JointSpec, ...]
    chains: dict[str, dict[str, Any]] = field(default_factory=dict)
    effectors: dict[str, dict[str, Any]] = field(default_factory=dict)
    metadata: dict[str, Any] = field(default_factory=dict)
    schema_version: str = "teleop.robot/v1"

    def __post_init__(self) -> None:
        if self.schema_version != "teleop.robot/v1" or not self.id or not self.joints:
            raise ValueError("Unsupported or empty robot profile")
        if len(set(self.joint_names)) != self.dimension:
            raise ValueError("Joint names must be unique")
        for chain in self.chains.values():
            for segment in chain.get("segments", []):
                if segment["joint"] not in self.joint_names:
                    raise ValueError(f"Unknown kinematic joint: {segment['joint']}")
        for effector in self.effectors.values():
            names = effector.get("joints", [])
            if not names or any(name not in self.joint_names for name in names):
                raise ValueError("Effector joints must belong to this robot")
            for endpoint in ("open", "closed"):
                values = finite_vector(effector[endpoint], len(names), endpoint)
                for name, value in zip(names, values):
                    spec = self.joints[self.index(name)]
                    if not spec.lower <= value <= spec.upper:
                        raise ValueError(f"Effector {endpoint} outside limits: {name}")

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(j.name for j in self.joints)

    @property
    def dimension(self) -> int:
        return len(self.joints)

    @property
    def initial(self) -> np.ndarray:
        return np.array([j.initial for j in self.joints], dtype=float)

    def index(self, name: str) -> int:
        return self.joint_names.index(name)

    def indices(self, groups: list[str] | tuple[str, ...] | None = None) -> list[int]:
        return [i for i, j in enumerate(self.joints) if groups is None or j.group in groups]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> RobotProfile:
        copied = dict(data)
        copied["joints"] = tuple(JointSpec(**item) for item in copied["joints"])
        return cls(**copied)


@dataclass(frozen=True)
class Pose:
    """Cartesian pose; quaternion order is XYZW."""

    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)

    def __post_init__(self) -> None:
        finite_vector(self.position, 3, "position")
        q = finite_vector(self.quaternion, 4, "quaternion")
        if abs(np.linalg.norm(q) - 1.0) > 0.02:
            raise ValueError("Pose quaternion must be normalized")

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> Pose:
        if not isinstance(data, dict):
            raise ValueError("Each pose must be an object")
        return cls(tuple(data["position"]), tuple(data.get("quaternion", (0, 0, 0, 1))))


@dataclass(frozen=True)
class InputFrame:
    """Device-independent input packet.

    ``twists`` uses [vx, vy, vz, wx, wy, wz] in robot-base coordinates.
    ``poses`` uses absolute tracked poses in the declared frame; the runtime
    applies relative calibration. ``grasp`` ranges from 0=open to 1=closed.
    A source must send a fresh sequence even when its pose does not change.
    """

    source_id: str
    sequence: int
    enabled: bool = False
    clutch: bool = False
    frame_id: str = "robot_base"
    poses: dict[str, Pose] = field(default_factory=dict)
    twists: dict[str, list[float]] = field(default_factory=dict)
    joints: dict[str, float] = field(default_factory=dict)
    grasp: dict[str, float] = field(default_factory=dict)
    valid: bool = True
    confidence: float = 1.0
    sent_wall_ns: int = 0
    raw: dict[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        for name in ("poses", "twists", "joints", "grasp", "raw"):
            if not isinstance(getattr(self, name), dict):
                raise ValueError(f"{name} must be an object")
        if not self.source_id or len(self.source_id) > 128 or type(self.sequence) is not int:
            raise ValueError("A bounded source_id and integer sequence are required")
        if self.sequence < 0 or self.frame_id not in {"robot_base", "webxr"}:
            raise ValueError("Invalid sequence or coordinate frame")
        if not math.isfinite(self.confidence) or not 0 <= self.confidence <= 1:
            raise ValueError("Confidence must be in [0, 1]")
        for key in ("enabled", "clutch", "valid"):
            if type(getattr(self, key)) is not bool:
                raise ValueError(f"{key} must be a boolean")
        for value in self.twists.values():
            finite_vector(value, 6, "twist")
        if not all(math.isfinite(v) for v in self.joints.values()):
            raise ValueError("Joint targets must be finite")
        if not all(math.isfinite(v) and 0 <= v <= 1 for v in self.grasp.values()):
            raise ValueError("Grasp must be finite and in [0, 1]")

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> InputFrame:
        if not isinstance(data, dict):
            raise ValueError("Input must be an object")
        for name in ("poses", "twists", "joints", "grasp", "raw"):
            if name in data and not isinstance(data[name], dict):
                raise ValueError(f"{name} must be an object")
        copied = dict(data)
        copied["poses"] = {name: Pose.from_dict(pose) for name, pose in copied.get("poses", {}).items()}
        return cls(**copied)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class RobotState:
    position: np.ndarray
    velocity: np.ndarray
    timestamp_ns: int
    valid: bool = True
    metadata: dict[str, Any] = field(default_factory=dict)

    def validate(self, profile: RobotProfile) -> None:
        self.position = finite_vector(self.position, profile.dimension, "measured position")
        self.velocity = finite_vector(self.velocity, profile.dimension, "measured velocity")
        if not self.valid or self.timestamp_ns <= 0:
            raise ValueError("Robot feedback is invalid")


class RobotDriver(Protocol):
    """SDK boundary: write returns delivery evidence, never synthetic measured feedback.
    ``hold`` must preserve the platform's own balance/gravity support semantics.
    """

    profile: RobotProfile
    hardware: bool
    writable: bool

    def connect(self) -> None: ...
    def read(self) -> RobotState: ...
    def write(self, target: np.ndarray) -> dict[str, Any]: ...
    def hold(self) -> None: ...
    def close(self) -> None: ...
