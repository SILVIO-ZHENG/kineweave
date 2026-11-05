"""Robot drivers for simulation, the existing G2 GDK adapter, and G1 SDK2 DDS.
Imports of hardware SDKs are deliberately lazy: listing profiles and running the
simulator never initialize a robot network. The drivers expose named profiles,
not a hard-coded global action dimension. A DDS publication is delivery evidence,
not an acknowledgement that a physical joint reached the commanded position.
"""

from __future__ import annotations

import importlib
import json
import math
from pathlib import Path
import sys
import threading
import time
from typing import Any, Callable

import numpy as np

from .types import RobotProfile, RobotState, finite_vector


class DriverError(RuntimeError):
    """Connection, feedback or dispatch failed."""


def _jsonable(value: Any) -> Any:
    """Convert NumPy values in SDK receipts."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    return value


def _target(profile: RobotProfile, value: Any) -> np.ndarray:
    """Enforce limits at the driver boundary too."""
    q = finite_vector(value, profile.dimension, "driver target")
    lower = np.array([j.lower for j in profile.joints])
    upper = np.array([j.upper for j in profile.joints])
    if np.any(q < lower - 1e-8) or np.any(q > upper + 1e-8):
        raise ValueError("Driver target exceeds the declared robot limits")
    return q


def _configuration(value: str | Path | dict | None) -> dict[str, Any]:
    if value is None:
        return {}
    if isinstance(value, dict):
        return dict(value)
    with Path(value).open(encoding="utf-8") as stream:
        loaded = json.load(stream)
    if not isinstance(loaded, dict):
        raise ValueError("Hardware configuration must be a JSON object")
    return loaded


def _hardware_validated(profile: RobotProfile) -> bool:
    """Demo models cannot enter execution via an alias flag."""
    declared = (profile.metadata.get("hardware_validated") is True
                or profile.metadata.get("hardware_verified") is True)
    return declared and profile.metadata.get("simulation_only") is not True


class SimulationDriver:
    """Lightweight integrated joint simulator for any declared dimension. A write
    changes the target; only elapsed simulated motion changes measured state.
    This is a kinematic preview, not a contact, gravity or balance simulator.
    """

    hardware = False
    writable = True

    def __init__(self, profile: RobotProfile, *, clock=None) -> None:
        self.profile = profile
        self._clock = clock or time.monotonic_ns
        self._q = profile.initial
        self._v = np.zeros(profile.dimension)
        self._goal = self._q.copy()
        self._connected = False
        self._last_ns = 0
        self._sequence = 0

    def connect(self) -> None:
        if self._connected:
            raise DriverError("Simulation is already connected")
        self._connected = True
        self._last_ns = self._clock()

    def _advance(self) -> None:
        if not self._connected:
            raise DriverError("Simulation is not connected")
        now = self._clock()
        # Do not catch up a long stale trajectory after suspension.
        remaining = min(max((now - self._last_ns) / 1e9, 0.0), 0.2)
        self._last_ns = now
        vmax = np.array([j.velocity for j in self.profile.joints])
        amax = np.array([j.acceleration for j in self.profile.joints])
        while remaining > 1e-10:
            dt = min(remaining, 0.005)
            error = self._goal - self._q
            desired = np.sign(error) * np.minimum(vmax, np.sqrt(2 * amax * np.abs(error)))
            self._v += np.clip(desired - self._v, -amax * dt, amax * dt)
            movement = self._v * dt
            reached = (movement * error >= 0) & (np.abs(movement) >= np.abs(error))
            self._q += movement
            self._q[reached] = self._goal[reached]
            self._v[reached] = 0
            remaining -= dt

    def read(self) -> RobotState:
        self._advance()
        return RobotState(self._q.copy(), self._v.copy(), self._last_ns,
                          metadata={"simulated": True, "wall_time_ns": time.time_ns()})

    def write(self, target: np.ndarray) -> dict[str, Any]:
        self._advance()
        self._goal = _target(self.profile, target)
        self._sequence += 1
        return {"accepted": True, "physical_accepted": False, "published": False,
                "simulated": True, "sequence": self._sequence, "command": self._goal.tolist(),
                "wall_time_ns": time.time_ns()}

    def hold(self) -> None:
        if self._connected:
            self._advance()
            self._goal = self._q.copy()
            self._v[:] = 0

    def close(self) -> None:
        self.hold()
        self._connected = False


_G2_NAMES = (tuple(f"left_arm_joint{i}" for i in range(1, 8)) + ("left_effector_position",)
             + tuple(f"right_arm_joint{i}" for i in range(1, 8)) + ("right_effector_position",)
             + tuple(f"head_joint{i}" for i in range(1, 4))
             + tuple(f"waist_joint{i}" for i in range(1, 6)))


def _nexus_types(nexus_root: str | Path | None):
    """Load only the existing execution adapter, without importing a learner.
    ``nexus_root`` may point at VLA_RL_RLT or its thor_project directory.
    """
    if nexus_root is not None:
        root = Path(nexus_root).expanduser().resolve()
        candidates = (root / "thor_project" / "src", root / "src")
        source = next((p for p in candidates if (p / "nexus_deploy").is_dir()), None)
        if source is None:
            raise DriverError("nexus_root must contain thor_project/src/nexus_deploy or src/nexus_deploy")
        if str(source) not in sys.path:
            sys.path.insert(0, str(source))
    try:
        module = importlib.import_module("nexus_deploy.robot.g2a.gdk_adapter")
    except ImportError as error:
        raise DriverError("Install the G2 Nexus dependencies and provide --nexus-root in the GDK environment") from error
    return module.G2AAdapter, module.AdapterConfig


class G2NexusDriver:
    """Thin bridge preserving the existing G2 SDK, rate limiter and collision checks.
    Targets are physical measured-coordinate targets: 0 m means closed and
    0.025 m means open for Dahuan. No model-action gripper inversion is applied.
    """

    hardware = True

    def __init__(self, profile: RobotProfile, *, execute: bool = False,
                 nexus_root: str | Path | None = None, safety_config: str | Path | None = None,
                 hardware_config: str | Path | dict | None = None, control_hz: float = 50.0,
                 **unused) -> None:
        self.profile = profile
        self.writable = bool(execute)
        self.nexus_root = nexus_root
        self.safety_config = Path(safety_config).resolve() if safety_config else None
        self.config = _configuration(hardware_config)
        self.control_hz = float(control_hz)
        self._adapter = None
        self._previous: tuple[np.ndarray, int] | None = None
        mapping = self.config.get("joint_map", {name: name for name in _G2_NAMES})
        if set(mapping) != set(_G2_NAMES) or set(mapping.values()) != set(profile.joint_names):
            raise ValueError("G2 joint_map must bijectively map all 24 canonical names onto this profile")
        self._order = np.array([profile.index(mapping[name]) for name in _G2_NAMES], dtype=int)
        if not math.isfinite(self.control_hz) or not 1 <= self.control_hz <= 100:
            raise ValueError("G2 control_hz must be in [1, 100]")
        if execute:
            if not _hardware_validated(profile):
                raise ValueError("G2 execution requires a site-verified robot profile")
            if self.safety_config is None or not self.safety_config.is_file():
                raise ValueError("G2 execution requires the calibrated Nexus safety configuration")
            if self.config.get("site_verified") is not True:
                raise ValueError("G2 execution requires hardware_config.site_verified=true")

    def connect(self) -> None:
        if self._adapter is not None:
            raise DriverError("G2 is already connected")
        adapter_type, config_type = _nexus_types(self.nexus_root)
        config = config_type(hardware_mode="execute" if self.writable else "read-only",
                             effector_profile=self.config.get("effector_profile", "dahuan"),
                             safety_config_path=self.safety_config,
                             servo_control_period_s=1.0 / self.control_hz,
                             safety_enforcement_enabled=True)
        adapter = adapter_type(config)
        try:
            adapter.connect()
            self._adapter = adapter
            self.read().validate(self.profile)
            if self.writable:
                # The existing SDK anchors control at measured pose.
                adapter.prepare_execution()
        except Exception:
            adapter.disconnect()
            self._adapter = None
            raise

    def _require(self):
        if self._adapter is None:
            raise DriverError("G2 is not connected")
        return self._adapter

    def read(self) -> RobotState:
        canonical = finite_vector(self._require().read_state(), 24, "G2 feedback")
        q = np.empty(self.profile.dimension)
        q[self._order] = canonical
        now = time.monotonic_ns()
        velocity = np.zeros_like(q)
        if self._previous is not None:
            previous, then = self._previous
            dt = (now - then) / 1e9
            if dt > 1e-6:
                velocity = (q - previous) / dt
        self._previous = (q.copy(), now)
        return RobotState(q, velocity, now, metadata={"wall_time_ns": time.time_ns(),
                                                    "velocity_source": "finite_difference"})

    def write(self, target: np.ndarray) -> dict[str, Any]:
        adapter = self._require()
        if not self.writable:
            raise DriverError("G2 is connected read-only")
        q = _target(self.profile, target)
        adapter.send_action(q[self._order].astype(np.float32))
        native = adapter.dispatch_receipt()
        if native is None:
            return {"accepted": False, "published": False, "physical_accepted": False,
                    "simulated": False, "reason": "No accepted dispatch in this send call",
                    "wall_time_ns": time.time_ns()}
        command = np.empty(self.profile.dimension)
        command[self._order] = finite_vector(native["action"], 24, "G2 dispatched action")
        return {"accepted": native.get("physical_accepted") is True,
                "published": True, "physical_accepted": native.get("physical_accepted") is True,
                "simulated": False, "command": command.tolist(), "sequence": native["sequence"],
                "wall_time_ns": time.time_ns(), "sdk_receipt": _jsonable(native)}

    def hold(self) -> None:
        if self._adapter is None or not self.writable:
            return
        # Retire pending motion and anchor at fresh feedback.
        self._adapter.retire_unissued_motion(reason="teleop_hold")
        measured = finite_vector(self._adapter.read_state(), 24, "G2 hold feedback")
        self._adapter.reanchor_measured_hold(measured)
        self._adapter.send_action(measured.astype(np.float32))

    def capture_cameras(self) -> dict[str, dict[str, Any]]:
        """Capture cameras separately from normal joint reads."""
        images, frame_times = self._require().capture_images()
        result = {}
        offset = self._require().camera_wall_to_vendor_offset_ns
        for name, payload in images.items():
            data = payload.get("data")
            if isinstance(data, (bytes, bytearray)):
                native_time = int(frame_times[name])
                result[name] = {"data": bytes(data),
                                "timestamp_ns": native_time + int(offset) if offset is not None else native_time,
                                "source_timestamp_ns": native_time,
                                "clock": "host_unix_estimate" if offset is not None else "vendor",
                                "wall_minus_vendor_ns": offset,
                                "encoding": payload.get("encoding"),
                                "color_format": payload.get("color_format"),
                                "width": payload.get("width"), "height": payload.get("height")}
        return result

    def close(self) -> None:
        adapter, self._adapter = self._adapter, None
        if adapter is not None:
            adapter.disconnect()


_G1_BODY_NAMES = (
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", "left_knee_joint",
    "left_ankle_pitch_joint", "left_ankle_roll_joint", "right_hip_pitch_joint", "right_hip_roll_joint",
    "right_hip_yaw_joint", "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint", "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint", "left_shoulder_yaw_joint", "left_elbow_joint", "left_wrist_roll_joint",
    "left_wrist_pitch_joint", "left_wrist_yaw_joint", "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint", "right_shoulder_yaw_joint", "right_elbow_joint",
    "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
)


def _unitree_sdk():
    """Import official DDS types only on an explicit hardware connection."""
    try:
        channel = importlib.import_module("unitree_sdk2py.core.channel")
        defaults = importlib.import_module("unitree_sdk2py.idl.default")
        messages = importlib.import_module("unitree_sdk2py.idl.unitree_hg.msg.dds_")
        crc = importlib.import_module("unitree_sdk2py.utils.crc")
    except ImportError as error:
        raise DriverError("Install unitree_sdk2_python and CycloneDDS in the robot-side environment") from error
    return channel, defaults, messages, crc.CRC


class G1UnitreeDriver:
    """G1 body and optional Dex3 DDS driver. Arm SDK writes address upper-body
    joints only; no lowcmd publisher, locomotion switcher or leg controller is
    created. A reviewed motor map and gains are required for physical execution.
    Body and hand feedback carry their actual receive times, never a fake fresh
    timestamp on a repeated stale state.
    """

    hardware = True

    def __init__(self, profile: RobotProfile, *, execute: bool = False, interface: str | None = None,
                 hardware_config: str | Path | dict | None = None, feedback_timeout: float = 0.3,
                 connect_timeout: float = 5.0, **unused) -> None:
        self.profile = profile
        self.writable = bool(execute)
        self.interface = interface
        self.config = _configuration(hardware_config)
        self.feedback_timeout = float(feedback_timeout)
        self.connect_timeout = float(connect_timeout)
        if not 0.02 <= self.feedback_timeout <= 2 or not 0.1 <= self.connect_timeout <= 30:
            raise ValueError("G1 feedback/connect timeouts are outside supported bounds")
        self._lock = threading.RLock()
        self._body = None
        self._body_at = 0
        self._hands: dict[str, tuple[Any, int]] = {}
        self._subscribers = []
        self._publishers: dict[str, Any] = {}
        self._connected = False
        self._sequence = 0
        self._last_target = None
        self._owns_arm_sdk = False
        self._last_observed = None
        self._body_map = self.config.get("body_motor_map", {
            name: i for i, name in enumerate(_G1_BODY_NAMES) if name in profile.joint_names
        })

        # Left and right Dex3 index/middle motor orders differ; never infer by sorting names.
        default_hands = {}
        for side, fingers in (("left", ("thumb_0", "thumb_1", "thumb_2", "middle_0", "middle_1", "index_0", "index_1")),
                              ("right", ("thumb_0", "thumb_1", "thumb_2", "index_0", "index_1", "middle_0", "middle_1"))):
            names = [f"{side}_hand_{finger}_joint" for finger in fingers]
            if all(name in profile.joint_names for name in names):
                default_hands[side] = {name: motor for motor, name in enumerate(names)}
        self._hand_maps = self.config.get("hand_motor_map", default_hands)
        mapped = list(self._body_map)
        for side, mapping in self._hand_maps.items():
            if side not in {"left", "right"} or len(mapping) != 7:
                raise ValueError("Each Dex3 map must specify seven joints for left or right")
            self._validate_map(mapping, 7, "Dex3")
            mapped.extend(mapping)
        self._validate_map(self._body_map, 29, "G1 body")
        if len(mapped) != len(set(mapped)) or set(mapped) != set(profile.joint_names):
            raise ValueError("G1 body/hand motor maps must cover each profile coordinate exactly once; "
                             "43D requires explicit left/right Dex3 maps")
        self._controlled_names = self.config.get("controlled_joints", [
            name for name, index in self._body_map.items() if 15 <= index <= 28
        ])
        if len(self._controlled_names) != len(set(self._controlled_names)):
            raise ValueError("controlled_joints must be unique")
        for name in self._controlled_names:
            if name not in profile.joint_names:
                raise ValueError(f"Unknown controlled joint: {name}")
            if name in self._body_map and self._body_map[name] < 15:
                raise ValueError("This driver does not accept leg or waist motion targets")
        if execute:
            self._validate_execution()

    @staticmethod
    def _validate_map(mapping: dict, size: int, label: str) -> None:
        indices = list(mapping.values())
        if (any(type(i) is not int or not 0 <= i < size for i in indices)
                or len(indices) != len(set(indices))):
            raise ValueError(f"{label} motor indices must be unique integers in [0,{size})")

    def _validate_execution(self) -> None:
        if not _hardware_validated(self.profile) or self.config.get("site_verified") is not True:
            raise ValueError("G1 execution requires a site-verified robot profile and hardware configuration")
        if self.config.get("firmware_watchdog_verified") is not True:
            raise ValueError("Confirm the robot-side command timeout/watchdog before enabling G1 execution")
        if not self.interface:
            raise ValueError("G1 execution requires an explicit network interface")
        if not self._controlled_names:
            raise ValueError("At least one reviewed controlled joint is required")
        if set(self._body_map.values()) != set(range(29)):
            raise ValueError("G1 arm_sdk execution requires the complete reviewed 29-motor body map")
        gains = self.config.get("gains", {})

        # arm_sdk ownership covers the upper body; waist targets remain measured and use reviewed gains.
        published = [name for name, i in self._body_map.items() if i >= 12]
        published += [name for side in self._hand_maps.values() for name in side
                      if any(joint in self._controlled_names for joint in side)]
        for name in published:
            item = gains.get(name)
            if (not isinstance(item, dict) or any(key not in item for key in ("kp", "kd"))
                    or any(not isinstance(item[key], (int, float)) or not math.isfinite(item[key])
                           or item[key] <= 0 for key in ("kp", "kd"))):
                raise ValueError(f"Explicit positive finite kp/kd required: {name}")
        if self.config.get("mode_pr") not in (0, 1):
            raise ValueError("Explicit mode_pr 0/1 required for the reviewed firmware")

    def connect(self) -> None:
        if self._connected or self._subscribers:
            raise DriverError("G1 is already connected")
        channel, defaults, messages, crc_type = _unitree_sdk()
        self._defaults, self._messages, self._crc = defaults, messages, crc_type()
        if self.interface:
            channel.ChannelFactoryInitialize(0, self.interface)
        else:
            channel.ChannelFactoryInitialize(0)
        try:
            body_sub = channel.ChannelSubscriber("rt/lowstate", messages.LowState_)
            self._subscribers.append(body_sub)
            body_sub.Init(self._on_body, 10)
            for side in self._hand_maps:
                sub = channel.ChannelSubscriber(f"rt/dex3/{side}/state", messages.HandState_)
                self._subscribers.append(sub)
                sub.Init(lambda msg, s=side: self._on_hand(s, msg), 10)
            deadline = time.monotonic() + self.connect_timeout
            while True:
                with self._lock:
                    ready = self._body is not None and all(side in self._hands for side in self._hand_maps)
                if ready:
                    break
                if time.monotonic() >= deadline:
                    raise DriverError("Timed out waiting for real G1 body/Dex3 feedback")
                time.sleep(0.01)
            self._connected = True
            self.read().validate(self.profile)
            # Read-only connections never create command publishers.
            if self.writable:
                pub = channel.ChannelPublisher("rt/arm_sdk", messages.LowCmd_)
                pub.Init()
                self._publishers["body"] = pub
                for side, mapping in self._hand_maps.items():
                    if any(name in self._controlled_names for name in mapping):
                        pub = channel.ChannelPublisher(f"rt/dex3/{side}/cmd", messages.HandCmd_)
                        pub.Init()
                        self._publishers[side] = pub
        except Exception:
            self.close()
            raise

    def _on_body(self, message) -> None:
        with self._lock:
            self._body = message
            self._body_at = time.monotonic_ns()

    def _on_hand(self, side: str, message) -> None:
        with self._lock:
            self._hands[side] = (message, time.monotonic_ns())

    def read(self) -> RobotState:
        if not self._connected:
            raise DriverError("G1 is not connected")
        now = time.monotonic_ns()
        with self._lock:
            times = [self._body_at] + [self._hands[side][1] for side in self._hand_maps]
            if self._body is None or any((now - stamp) / 1e9 > self.feedback_timeout for stamp in times):
                raise DriverError("G1 body or Dex3 feedback is stale")
            q, dq = np.empty(self.profile.dimension), np.empty(self.profile.dimension)
            for name, motor in self._body_map.items():
                state = self._body.motor_state[motor]
                q[self.profile.index(name)], dq[self.profile.index(name)] = state.q, state.dq
            for side, mapping in self._hand_maps.items():
                message = self._hands[side][0]
                for name, motor in mapping.items():
                    state = message.motor_state[motor]
                    q[self.profile.index(name)], dq[self.profile.index(name)] = state.q, state.dq
            state = RobotState(q, dq, min(times), metadata={"wall_time_ns": time.time_ns(),
                               "body_received_ns": self._body_at,
                               "hand_received_ns": {side: self._hands[side][1] for side in self._hand_maps},
                               "feedback_clock": "host_receive_monotonic"})
        state.validate(self.profile)
        self._last_observed = state.position.copy()
        return state

    def _body_command(self, target: np.ndarray, measured: np.ndarray, weight: float):
        command = self._defaults.unitree_hg_msg_dds__LowCmd_()
        command.mode_pr = int(self.config["mode_pr"])
        with self._lock:
            command.mode_machine = int(getattr(self._body, "mode_machine", 0))
        command.motor_cmd[29].q = float(weight)
        for name, motor in self._body_map.items():
            if motor < 12:
                continue
            joint = command.motor_cmd[motor]
            joint.q = float(target[self.profile.index(name)] if name in self._controlled_names
                            else measured[self.profile.index(name)])
            joint.dq = 0.0
            joint.tau = 0.0
            joint.kp = float(self.config["gains"][name]["kp"])
            joint.kd = float(self.config["gains"][name]["kd"])
        command.crc = self._crc.Crc(command)
        return command

    def _hand_command(self, side: str, target: np.ndarray, measured: np.ndarray):
        command = self._defaults.unitree_hg_msg_dds__HandCmd_()
        for name, motor in self._hand_maps[side].items():
            joint = command.motor_cmd[motor]
            joint.mode = motor | (1 << 4)
            joint.q = float(target[self.profile.index(name)] if name in self._controlled_names
                            else measured[self.profile.index(name)])
            joint.dq, joint.tau = 0.0, 0.0

            # Uncontrolled fingers hold measured targets and require reviewed gains too.
            gain = self.config["gains"].get(name)
            if gain is None:
                raise DriverError(f"Missing reviewed hand gains: {name}")
            joint.kp, joint.kd = float(gain["kp"]), float(gain["kd"])
        return command

    def write(self, target: np.ndarray) -> dict[str, Any]:
        if not self._connected or not self.writable:
            raise DriverError("G1 is disconnected or read-only")
        q = _target(self.profile, target)
        previous_observation = None if self._last_observed is None else self._last_observed.copy()
        measured = self.read().position
        controlled = set(self._controlled_names)
        for i, name in enumerate(self.profile.joint_names):
            if name not in controlled:

                # Validate against the last public observation, then refresh to avoid callback-race false alarms.
                baseline = measured[i] if previous_observation is None else previous_observation[i]
                if abs(q[i] - baseline) > 1e-5:
                    raise ValueError(f"This G1 driver does not control {name}; preserve fresh measured coordinates")
                q[i] = measured[i]
        command = self._body_command(q, measured, 1.0)
        result = self._publishers["body"].Write(command)
        if result is False:
            raise DriverError("G1 arm SDK publication failed")
        self._owns_arm_sdk = True
        published = ["rt/arm_sdk"]
        for side in self._hand_maps:
            if side in self._publishers:
                result = self._publishers[side].Write(self._hand_command(side, q, measured))
                if result is False:
                    raise DriverError(f"Dex3 {side} command publication failed after arm publication")
                published.append(f"rt/dex3/{side}/cmd")
        self._last_target = q.copy()
        self._sequence += 1
        return {"accepted": True, "published": True, "physical_accepted": None,
                "simulated": False, "sequence": self._sequence, "command": q.tolist(),
                "topics": published, "wall_time_ns": time.time_ns(),
                "evidence": "DDS publication only; use measured feedback to assess execution"}

    def hold(self) -> None:
        if self._connected and self.writable and self._owns_arm_sdk:

            # Keep arm SDK ownership at measured upper-body pose; do not disable balance control.
            measured = self.read().position
            self.write(measured)

    def close(self) -> None:
        self.last_close_error = None
        try:
            if self._connected and self.writable and self._owns_arm_sdk and "body" in self._publishers:

                # Ramp down arm_sdk weight; stale feedback falls back to the reviewed firmware watchdog.
                for weight in (0.8, 0.6, 0.4, 0.2, 0.0):
                    measured = self.read().position
                    command = self._body_command(measured, measured, weight)
                    self._publishers["body"].Write(command)
                    if weight:
                        time.sleep(0.02)
        except Exception as error:
            self.last_close_error = str(error)
        finally:
            for endpoint in list(self._publishers.values()) + self._subscribers:
                close = getattr(endpoint, "Close", None)
                if callable(close):
                    try:
                        close()
                    except Exception as error:
                        self.last_close_error = str(error)
            self._publishers.clear()
            self._subscribers.clear()
            self._connected = False
            self._owns_arm_sdk = False


class G2CameraPump:
    """Separate GDK camera worker retaining native clock evidence. Start only after
    the driver is connected, and close before disconnecting that driver. The
    Nexus adapter uses a separate Camera object, so no Robot joint read is added
    to the control thread. JPEG encoding, if needed, also remains in this worker.
    """

    def __init__(self, driver: G2NexusDriver, callback: Callable[[str, bytes, int], None], *,
                 fps: float = 15, on_error: Callable[[Exception], None] | None = None,
                 on_metadata: Callable[[dict], None] | None = None) -> None:
        if not math.isfinite(fps) or not 0 < fps <= 60:
            raise ValueError("G2 camera FPS must be in (0, 60]")
        self.driver, self.callback, self.fps = driver, callback, float(fps)
        self.on_error, self.on_metadata = on_error, on_metadata
        self.error: Exception | None = None
        self.frames = 0
        self.latest_metadata: dict[str, dict] = {}
        self._last_source_times: dict[str, int] = {}
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> G2CameraPump:
        if self._thread is not None:
            raise DriverError("G2 camera worker has already been started")
        self._thread = threading.Thread(target=self._run, name="g2-gdk-cameras", daemon=True)
        self._thread.start()
        return self

    @staticmethod
    def _jpeg(frame: dict) -> bytes:
        data = frame["data"]
        if frame.get("encoding") == "jpeg":
            if not data.startswith(b"\xff\xd8"):
                raise DriverError("GDK declared JPEG but payload has no JPEG signature")
            return data
        if frame.get("encoding") != "raw":
            raise DriverError(f"Unsupported GDK camera encoding: {frame.get('encoding')}")
        try:
            import cv2
        except ImportError as error:
            raise DriverError("Raw GDK camera encoding needs the optional OpenCV camera dependency") from error
        width, height = int(frame["width"]), int(frame["height"])
        if width <= 0 or height <= 0 or width * height > 32_000_000:
            raise DriverError("Invalid GDK camera dimensions")
        pixels = np.frombuffer(data, dtype=np.uint8)
        if len(pixels) != width * height * 3:
            raise DriverError("Raw GDK camera payload does not match RGB/BGR dimensions")
        image = pixels.reshape(height, width, 3)
        if frame.get("color_format") == "rgb":
            image = image[:, :, ::-1]
        elif frame.get("color_format") != "bgr":
            raise DriverError("Only explicit RGB/BGR raw frames are supported")
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            raise DriverError("Cannot encode the raw GDK camera frame")
        return encoded.tobytes()

    def _run(self) -> None:
        try:
            while not self._stop.is_set():
                started = time.monotonic()
                for name, frame in self.driver.capture_cameras().items():
                    if self._stop.is_set():
                        break
                    if frame.get("clock") != "host_unix_estimate":
                        raise DriverError("Cannot record GDK camera before the vendor-to-host clock bridge exists")
                    source_time = int(frame["source_timestamp_ns"])
                    previous = self._last_source_times.get(name)
                    if previous is not None:
                        if source_time < previous:
                            raise DriverError(f"GDK camera {name} source clock moved backwards")
                        if source_time == previous:
                            continue
                    jpeg = self._jpeg(frame)
                    metadata = {key: value for key, value in frame.items() if key != "data"}
                    metadata["name"] = name
                    metadata["receive_wall_time_ns"] = time.time_ns()
                    self.latest_metadata[name] = metadata
                    if self.on_metadata:
                        self.on_metadata(metadata)
                    self.callback(name, jpeg, int(frame["timestamp_ns"]))
                    self._last_source_times[name] = source_time
                    self.frames += 1
                self._stop.wait(max(0.0, 1.0 / self.fps - (time.monotonic() - started)))
        except Exception as error:
            self.error = error
            if self.on_error:
                try:
                    self.on_error(error)
                except Exception:
                    # Preserve the original failure even if reporting fails.
                    pass

    def check(self) -> None:
        if self.error is not None:
            raise DriverError(f"G2 camera worker failed: {self.error}") from self.error

    def close(self, timeout: float = 3.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
            if self._thread.is_alive():
                raise DriverError("GDK camera worker did not stop; do not disconnect SDK concurrently")
        self.check()


def create_driver(kind: str, profile: RobotProfile, **options):
    """Construct a driver by name. Construction itself performs no hardware I/O.
    External robots can use ``module:factory``; the explicitly selected callable
    receives ``profile, **options`` and returns a RobotDriver. It must not perform
    hardware I/O until connect, and must respect execute=False.
    """
    drivers = {"sim": SimulationDriver, "g2": G2NexusDriver, "g1": G1UnitreeDriver}
    if ":" in kind:
        module_name, factory_name = kind.split(":", 1)
        if not module_name or not factory_name or not factory_name.isidentifier():
            raise ValueError("External driver must use module:factory syntax")
        factory = getattr(importlib.import_module(module_name), factory_name)
        if not callable(factory):
            raise ValueError("External driver factory is not callable")
        driver = factory(profile, **options)
        methods = ("connect", "read", "write", "hold", "close")
        if (not all(callable(getattr(driver, name, None)) for name in methods)
                or not isinstance(getattr(driver, "profile", None), RobotProfile)
                or driver.profile.to_dict() != profile.to_dict()
                or type(getattr(driver, "hardware", None)) is not bool
                or type(getattr(driver, "writable", None)) is not bool):
            raise ValueError("External driver does not implement the RobotDriver contract")
        if driver.hardware and driver.writable and not options.get("execute", False):
            raise ValueError("External hardware driver ignored execute=False")
        if driver.hardware and driver.writable and not _hardware_validated(profile):
            raise ValueError("External hardware execution requires a site-validated robot profile")
        return driver
    if kind not in drivers:
        raise ValueError(f"Unknown driver: {kind}; choose sim, g2 or g1")
    if kind == "sim":
        return SimulationDriver(profile)
    return drivers[kind](profile, **options)
