"""Focused control-path checks using an in-memory driver. These tests exercise the
public runtime boundary without importing vendor SDKs or publishing commands.
"""

import time

import numpy as np
import pytest

from teleop_v2.kinematics import IKResult, matrix_pose
from teleop_v2.profiles import load_profile
from teleop_v2.recording import inspect_episode, read_episode
from teleop_v2.runtime import TeleopRuntime
from teleop_v2.types import InputFrame, Pose, RobotState


class MemoryDriver:
    """Keep dispatched commands separate from measured feedback."""

    hardware = False
    writable = True

    def __init__(self, profile):
        self.profile = profile
        self.position = profile.initial
        self.commands = []
        self.holds = 0
        self.connected = False

    def connect(self):
        self.connected = True

    def read(self):
        assert self.connected
        return RobotState(self.position.copy(), np.zeros(self.profile.dimension), time.monotonic_ns())

    def write(self, target):
        self.commands.append(target.copy())
        return {"accepted": True, "simulated": True, "physical_accepted": False}

    def hold(self):
        self.holds += 1

    def close(self):
        self.connected = False


@pytest.fixture
def runtime_factory(tmp_path):
    created = []

    def make(profile_id="arm7", **options):
        profile = load_profile(profile_id)
        driver = MemoryDriver(profile)
        runtime = TeleopRuntime(profile, driver, recording_root=tmp_path, **options)
        runtime.start(background=False)
        created.append(runtime)
        return runtime, driver

    yield make
    for runtime in created:
        runtime.close()


def packet(sequence=0, **changes):
    data = {"source_id": "controller", "sequence": sequence, "enabled": True}
    data.update(changes)
    return InputFrame(**data)


def test_slow_ik_cannot_dispatch_stale_intent(runtime_factory, monkeypatch):
    runtime, driver = runtime_factory()
    clock = [time.monotonic_ns()]
    monkeypatch.setattr("teleop_v2.runtime.time.monotonic_ns", lambda: clock[0])

    def slow_ik(q, _targets):
        clock[0] += 200_000_000
        return IKResult(q.copy(), True, {}, 1)

    monkeypatch.setattr(runtime.model, "solve", slow_ik)
    runtime.submit(packet(poses={"right": Pose((0, 0, 0))}))
    runtime.action("arm", source_id="controller")
    sample = runtime.tick()
    assert sample["mode"] == "hold"
    assert sample["receipt"]["reason"] == "computation_deadline"
    assert not driver.commands and runtime.latest is None


@pytest.mark.parametrize("profile_id,dimension", [("arm6", 6), ("arm7", 7), ("g2_24", 24), ("g1_29", 29), ("g1_43", 43)])
def test_dynamic_dimensions_and_explicit_arm_deadman(runtime_factory, profile_id, dimension):
    runtime, driver = runtime_factory(profile_id)
    index = next(i for i, spec in enumerate(runtime.profile.joints) if spec.group == "right_arm")
    joint = runtime.profile.joint_names[index]
    goal = driver.position[index] + .1
    runtime.submit(packet(joints={joint: goal}))
    assert runtime.tick()["mode"] == "disarmed"
    assert not driver.commands
    runtime.action("arm", source_id="controller")
    active = runtime.tick()
    assert active["mode"] == "active"
    assert len(active["command"]) == dimension
    assert driver.commands[-1][index] > driver.position[index]
    runtime.submit(packet(1, enabled=False, joints={joint: goal}))
    assert runtime.tick()["mode"] == "hold"
    assert len(driver.commands) == 1
    assert driver.holds == 1
    runtime.submit(packet(2, clutch=True, joints={joint: goal}))
    assert runtime.tick()["mode"] == "hold"
    assert len(driver.commands) == 1


def test_source_and_sequence_ownership_are_enforced(runtime_factory):
    runtime, driver = runtime_factory()
    runtime.submit(packet(4))
    with pytest.raises(ValueError, match="sequence"):
        runtime.submit(packet(4))
    with pytest.raises(ValueError, match="sequence"):
        runtime.submit(packet(3))
    runtime.action("arm", source_id="controller")
    with pytest.raises(ValueError, match="owns control"):
        runtime.submit(packet(1, source_id="other"))
    runtime.action("disarm")
    runtime.submit(packet(1, source_id="other"))
    runtime.action("arm", source_id="other")
    assert runtime.owner == "other"
    assert not driver.commands


def test_timeout_holds_and_reanchors_pose_on_resume(runtime_factory, monkeypatch):

    # Advance the receive clock explicitly to avoid slow/flaky wall-clock sleeps.
    now = [10_000_000_000]
    monkeypatch.setattr("teleop_v2.runtime.time.monotonic_ns", lambda: now[0])
    runtime, driver = runtime_factory(timeout=.1)
    runtime.submit(packet(poses={"right": Pose((0, 0, 0))}))
    runtime.action("arm", source_id="controller")
    assert runtime.tick()["mode"] == "active"
    anchor = runtime.model.forward(driver.position, "right")
    np.testing.assert_allclose(runtime.retarget.targets["right"], anchor)

    desired = driver.position.copy()
    desired[0] += .04
    target = runtime.model.forward(desired, "right")
    device = np.eye(4)
    device[:3, 3] = target[:3, 3] - anchor[:3, 3]
    device[:3, :3] = target[:3, :3] @ anchor[:3, :3].T
    now[0] += 20_000_000
    runtime.submit(packet(1, poses={"right": matrix_pose(device)}))
    assert runtime.tick()["mode"] == "active"
    np.testing.assert_allclose(runtime.retarget.targets["right"], target, atol=1e-10)

    now[0] += 200_000_000
    count = len(driver.commands)
    timed_out = runtime.tick()
    assert timed_out["mode"] == "hold"
    assert not timed_out["input_fresh"]
    assert len(driver.commands) == count
    assert driver.holds == 1
    assert runtime.retarget.anchors == {}
    # A large human movement while paused must not become a jump on resume.
    now[0] += 20_000_000
    runtime.submit(packet(2, poses={"right": Pose((10, 20, 30))}))
    resumed = runtime.tick()
    assert resumed["mode"] == "active"
    np.testing.assert_allclose(driver.commands[-1], driver.position)
    np.testing.assert_allclose(runtime.retarget.targets["right"], anchor)


@pytest.mark.parametrize("profile_id", ["g2_24", "g1_29", "g1_43"])
def test_noncontrolled_groups_preserve_measured_feedback(runtime_factory, profile_id):
    runtime, driver = runtime_factory(profile_id, groups=["right_arm"])
    left = next(j for j in runtime.profile.joints if j.group == "left_arm")
    right = next(j for j in runtime.profile.joints if j.group == "right_arm")
    runtime.submit(packet(joints={left.name: left.initial + .1, right.name: right.initial + .1}))
    runtime.action("arm", source_id="controller")
    sample = runtime.tick()
    assert sample["mode"] == "active"
    for index, joint in enumerate(runtime.profile.joints):
        if joint.group != "right_arm":
            assert driver.commands[-1][index] == driver.position[index]
    assert driver.commands[-1][runtime.profile.index(right.name)] > right.initial


def test_ik_failure_never_dispatches_partial_solution(runtime_factory, monkeypatch):
    runtime, driver = runtime_factory()
    runtime.submit(packet(poses={"right": Pose((0, 0, 0))}))
    runtime.action("arm", source_id="controller")
    partial = driver.position + .05
    monkeypatch.setattr(runtime.model, "solve", lambda *args: IKResult(
        partial, False, {"right": {"position": .2, "orientation": .1}}, 5))
    sample = runtime.tick()
    assert sample["mode"] == "hold"
    assert sample["receipt"]["reason"] == "ik_failed"
    assert not driver.commands
    assert driver.holds == 1
    np.testing.assert_array_equal(sample["command"], driver.position)
    assert runtime.retarget.anchors == {}


@pytest.mark.parametrize("changes", [{"valid": False}, {"confidence": .2}, {"clutch": True}, {"enabled": False}])
def test_invalid_tracking_cannot_drive_robot(runtime_factory, changes):
    runtime, driver = runtime_factory()
    runtime.submit(packet(joints={"joint_1": .2}, **changes))
    runtime.action("arm", source_id="controller")
    assert runtime.tick()["mode"] == "hold"
    assert not driver.commands


def test_latched_stop_and_stale_arm_require_fresh_explicit_actions(runtime_factory, monkeypatch):
    now = [10_000_000_000]
    monkeypatch.setattr("teleop_v2.runtime.time.monotonic_ns", lambda: now[0])
    runtime, driver = runtime_factory(timeout=.1)
    runtime.submit(packet())
    now[0] += 200_000_000
    with pytest.raises(ValueError, match="stale"):
        runtime.action("arm", source_id="controller")
    runtime.action("estop")
    runtime.submit(packet(1))
    with pytest.raises(ValueError, match="latched"):
        runtime.action("arm", source_id="controller")
    runtime.action("reset")
    assert runtime.mode == "disarmed" and runtime.latest is None
    assert not driver.commands


@pytest.mark.parametrize("action", ["pause", "disarm", "calibrate"])
def test_software_stop_cannot_be_cleared_by_other_actions(runtime_factory, action):
    runtime, driver = runtime_factory()
    runtime.submit(packet())
    runtime.action("arm", source_id="controller")
    runtime.action("estop")
    runtime.action(action)
    assert runtime.mode == "estop"
    runtime.submit(packet(1))
    with pytest.raises(ValueError, match="latched"):
        runtime.action("arm", source_id="controller")
    assert not driver.commands


def test_stop_removes_authority_even_when_sdk_hold_fails(runtime_factory, monkeypatch):
    runtime, driver = runtime_factory()
    runtime.submit(packet(joints={"joint_1": .2}))
    runtime.action("arm", source_id="controller")

    def fail_hold():
        raise RuntimeError("SDK hold failed")

    with monkeypatch.context() as patch:
        patch.setattr(driver, "hold", fail_hold)
        try:
            runtime.action("estop")
        except RuntimeError:
            pass
        assert runtime.mode in {"estop", "fault"}
        assert runtime.owner is None and runtime.latest is None
    assert not driver.commands


def test_deadline_miss_discards_pending_input(runtime_factory):
    runtime, driver = runtime_factory()
    runtime.submit(packet(joints={"joint_1": .2}))
    runtime.action("arm", source_id="controller")
    assert runtime.tick()["mode"] == "active"
    count = len(driver.commands)
    delayed = runtime.tick(dt=.2)
    assert delayed["mode"] == "hold"
    assert runtime.latest is None
    assert len(driver.commands) == count
    assert any(event["kind"] == "deadline_missed" for event in runtime.events)


def test_stale_feedback_and_rejected_dispatch_fault_without_further_motion(runtime_factory, monkeypatch):
    runtime, driver = runtime_factory()
    runtime.submit(packet(joints={"joint_1": .2}))
    runtime.action("arm", source_id="controller")
    monkeypatch.setattr(driver, "read", lambda: RobotState(
        driver.position.copy(), np.zeros(runtime.profile.dimension), time.monotonic_ns() - 1_000_000_000))
    assert runtime.tick()["mode"] == "fault"
    assert "feedback is stale" in runtime.fault
    assert not driver.commands
    assert driver.holds == 1

    other, rejected_driver = runtime_factory()
    other.submit(packet(joints={"joint_1": .2}))
    other.action("arm", source_id="controller")
    monkeypatch.setattr(rejected_driver, "write", lambda target: {"accepted": False, "reason": "SDK rejected"})
    assert other.tick()["mode"] == "fault"
    assert "Robot rejected command" in other.fault
    assert other.owner is None and other.latest is None
    assert rejected_driver.holds == 1


def test_recording_roundtrip_preserves_command_feedback_and_input(runtime_factory, monkeypatch):
    now = [10_000_000_000]
    monkeypatch.setattr("teleop_v2.runtime.time.monotonic_ns", lambda: now[0])
    runtime, driver = runtime_factory("g1_43")
    runtime.action("record_start", task="synchronized capture")
    joint = "right_shoulder_pitch_joint"
    runtime.submit(packet(joints={joint: .2}, raw={"button": "trigger"}))
    runtime.action("arm", source_id="controller")
    emitted = []
    for _ in range(2):
        now[0] += 20_000_000
        emitted.append(runtime.tick())
    runtime.action("record_stop")
    summary = inspect_episode(runtime.last_episode)
    assert summary["complete"] and summary["verified"]
    episode = read_episode(runtime.last_episode)
    assert len(episode["profile"]["joints"]) == 43
    assert len(episode["samples"]) == 2
    for actual, expected in zip(episode["samples"], emitted):
        assert actual["sequence"] == expected["sequence"]
        assert actual["input"]["raw"]["button"] == "trigger"
        assert actual["receipt"]["accepted"]
        assert actual["state"] == driver.position.tolist()
        assert actual["command"] == expected["command"]
        assert actual["command"] != actual["state"]
