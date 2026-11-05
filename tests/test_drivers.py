"""Driver contract tests using fake SDKs only."""

from types import SimpleNamespace
import time

import numpy as np
import pytest

from teleop_v2 import drivers
from teleop_v2.drivers import DriverError, G1UnitreeDriver, G2NexusDriver, SimulationDriver
from teleop_v2.types import JointSpec, RobotProfile


def profile(names, *, verified=False):
    return RobotProfile("test", "test", tuple(
        JointSpec(name, 0 if "effector" in name else -3, 0.025 if "effector" in name else 3,
                  0.5, 1, 0.0125 if "effector" in name else 0) for name in names),
        metadata={"hardware_validated": verified})


def test_simulation_integrates_feedback_and_rejects_invalid_vectors():
    now = [1_000_000_000]
    p = profile([f"j{i}" for i in range(43)])
    driver = SimulationDriver(p, clock=lambda: now[0])
    driver.connect()
    driver.write(np.full(43, 0.4))
    assert np.all(driver.read().position == 0)
    now[0] += 100_000_000
    measured = driver.read()
    assert np.all(measured.position > 0)
    assert np.all(measured.position < 0.4)
    assert np.max(measured.velocity) <= 0.5
    assert measured.timestamp_ns == now[0]
    with pytest.raises(ValueError):
        driver.write(np.zeros(24))
    with pytest.raises(ValueError):
        driver.write(np.full(43, float("nan")))
    with pytest.raises(ValueError):
        driver.write(np.full(43, 4))
    driver.hold()
    held = driver.read().position
    now[0] += 100_000_000
    assert np.array_equal(driver.read().position, held)
    driver.close()
    with pytest.raises(DriverError):
        driver.read()


class FakeG2:
    instances = []

    def __init__(self, config):
        self.config = config
        self.prepared = False
        self.state = np.zeros(24, dtype=np.float32)
        self.state[7] = self.state[15] = 0.0125
        self.sent = []
        self.instances.append(self)

    def connect(self):
        pass

    def disconnect(self):
        pass

    def prepare_execution(self):
        self.prepared = True

    def read_state(self):
        return self.state.copy()

    def send_action(self, target):
        self.sent.append(target.copy())

    def dispatch_receipt(self):
        return {"sequence": len(self.sent), "action": self.sent[-1], "physical_accepted": True}

    def retire_unissued_motion(self, **kwargs):
        pass

    def reanchor_measured_hold(self, state):
        pass


def test_g2_readonly_never_prepares_or_sends(monkeypatch):
    monkeypatch.setattr(drivers, "_nexus_types", lambda root: (FakeG2, lambda **kw: SimpleNamespace(**kw)))
    driver = G2NexusDriver(profile(drivers._G2_NAMES))
    driver.connect()
    assert driver.read().position[7] == pytest.approx(0.0125)
    assert not driver._adapter.prepared
    with pytest.raises(DriverError):
        driver.write(driver.profile.initial)
    driver.hold()
    assert not driver._adapter.sent
    driver.close()


def test_g2_physical_gripper_coordinates_are_not_inverted(monkeypatch, tmp_path):
    monkeypatch.setattr(drivers, "_nexus_types", lambda root: (FakeG2, lambda **kw: SimpleNamespace(**kw)))
    safety = tmp_path / "safety.json"
    safety.write_text("{}")
    driver = G2NexusDriver(profile(drivers._G2_NAMES, verified=True), execute=True,
                           safety_config=safety, hardware_config={"site_verified": True})
    driver.connect()
    q = driver.profile.initial
    q[7], q[15] = 0.025, 0.0
    receipt = driver.write(q)
    assert driver._adapter.sent[-1][7] == pytest.approx(0.025)
    assert driver._adapter.sent[-1][15] == 0
    assert receipt["physical_accepted"] is True
    driver.close()


class FakeEndpoint:
    def __init__(self, topic, message_type):
        self.topic = topic
        self.messages = []
        self.closed = False

    def Init(self, handler=None, queue_len=None):
        if handler is not None:
            count = 7 if "dex3" in self.topic else 35
            handler(SimpleNamespace(motor_state=[SimpleNamespace(q=0.0, dq=0.0) for _ in range(count)],
                                    mode_machine=3))

    def Write(self, message):
        self.messages.append(message)
        return True

    def Close(self):
        self.closed = True


def fake_unitree():
    pubs = []

    def publisher(topic, message_type):
        endpoint = FakeEndpoint(topic, message_type)
        pubs.append(endpoint)
        return endpoint

    def command(count):
        return SimpleNamespace(motor_cmd=[SimpleNamespace(q=0.0, dq=0.0, tau=0.0, kp=0.0, kd=0.0)
                                         for _ in range(count)], mode_pr=0, mode_machine=0, crc=0)

    channel = SimpleNamespace(ChannelFactoryInitialize=lambda *args: None, ChannelSubscriber=FakeEndpoint,
                              ChannelPublisher=publisher)
    defaults = SimpleNamespace(unitree_hg_msg_dds__LowCmd_=lambda: command(35),
                               unitree_hg_msg_dds__HandCmd_=lambda: command(7))
    messages = SimpleNamespace(LowState_=object, LowCmd_=object, HandState_=object, HandCmd_=object)
    def crc():
        return SimpleNamespace(Crc=lambda cmd: 1234)
    return (channel, defaults, messages, crc), pubs


def g1_execution_config(p):
    return {"site_verified": True, "firmware_watchdog_verified": True, "mode_pr": 0,
            "controlled_joints": [name for name in p.joint_names if "shoulder" in name
                                  or "elbow" in name or "wrist" in name or "hand" in name],
            "gains": {name: {"kp": 1.0, "kd": 0.1} for name in p.joint_names}}


def test_g1_readonly_real_feedback_and_stale_timestamp(monkeypatch):
    sdk, pubs = fake_unitree()
    monkeypatch.setattr(drivers, "_unitree_sdk", lambda: sdk)
    driver = G1UnitreeDriver(profile(drivers._G1_BODY_NAMES))
    driver.connect()
    received = driver.read().timestamp_ns
    assert driver.read().timestamp_ns == received
    assert not pubs
    with pytest.raises(DriverError):
        driver.write(driver.profile.initial)
    driver._body_at = time.monotonic_ns() - 1_000_000_000
    with pytest.raises(DriverError, match="stale"):
        driver.read()
    driver.close()


def test_g1_arm_publication_is_not_physical_ack_and_preserves_legs(monkeypatch):
    sdk, pubs = fake_unitree()
    monkeypatch.setattr(drivers, "_unitree_sdk", lambda: sdk)
    p = profile(drivers._G1_BODY_NAMES, verified=True)
    driver = G1UnitreeDriver(p, execute=True, interface="test0", hardware_config=g1_execution_config(p))
    driver.connect()
    q = np.zeros(29)
    q[15] = 0.1
    receipt = driver.write(q)
    assert receipt["physical_accepted"] is None
    assert receipt["accepted"] is True
    assert pubs[0].topic == "rt/arm_sdk"
    message = pubs[0].messages[-1]
    assert message.motor_cmd[29].q == 1
    assert message.motor_cmd[15].q == 0.1
    assert message.motor_cmd[0].kp == 0
    assert message.crc == 1234
    q[0] = 0.1
    with pytest.raises(ValueError, match="does not control"):
        driver.write(q)
    driver.close()
    assert pubs[0].messages[-1].motor_cmd[29].q == 0


def test_g1_callback_between_read_and_write_does_not_reject_balancing_motion(monkeypatch):
    sdk, pubs = fake_unitree()
    monkeypatch.setattr(drivers, "_unitree_sdk", lambda: sdk)
    p = profile(drivers._G1_BODY_NAMES, verified=True)
    driver = G1UnitreeDriver(p, execute=True, interface="test0", hardware_config=g1_execution_config(p))
    driver.connect()
    q = driver.read().position

    # Caller preserves observed legs; balancing feedback updates just before dispatch.
    driver._on_body(SimpleNamespace(motor_state=[SimpleNamespace(q=0.0002 if i == 0 else 0.0, dq=0.0)
                                                for i in range(35)], mode_machine=3))
    q[15] = 0.1
    receipt = driver.write(q)
    assert receipt["command"][0] == pytest.approx(0.0002)
    assert pubs[0].messages[-1].motor_cmd[0].kp == 0
    driver.close()


def test_g1_43dof_uses_actual_dex3_feedback_and_asymmetric_motor_order(monkeypatch):
    sdk, pubs = fake_unitree()
    monkeypatch.setattr(drivers, "_unitree_sdk", lambda: sdk)
    hand_names = [f"{side}_hand_{finger}_joint" for side in ("left", "right")
                  for finger in ("thumb_0", "thumb_1", "thumb_2", "index_0", "index_1", "middle_0", "middle_1")]
    p = profile(list(drivers._G1_BODY_NAMES) + hand_names, verified=True)
    driver = G1UnitreeDriver(p, execute=True, interface="test0", hardware_config=g1_execution_config(p))
    driver.connect()
    assert driver.read().position.shape == (43,)
    q = np.zeros(43)
    q[p.index("left_hand_index_0_joint")] = 0.3
    q[p.index("right_hand_index_0_joint")] = 0.2
    driver.write(q)
    by_topic = {pub.topic: pub for pub in pubs}
    assert by_topic["rt/dex3/left/cmd"].messages[-1].motor_cmd[5].q == 0.3
    assert by_topic["rt/dex3/right/cmd"].messages[-1].motor_cmd[3].q == 0.2
    driver._hands["left"] = (driver._hands["left"][0], time.monotonic_ns() - 1_000_000_000)
    with pytest.raises(DriverError, match="stale"):
        driver.read()
    driver.close()


def test_execution_requires_verified_model_and_config():
    with pytest.raises(ValueError, match="site-verified"):
        G1UnitreeDriver(profile(drivers._G1_BODY_NAMES), execute=True)
    with pytest.raises(ValueError, match="site-verified"):
        G2NexusDriver(profile(drivers._G2_NAMES), execute=True)


def test_g1_connect_close_without_command_does_not_claim_ownership(monkeypatch):
    sdk, pubs = fake_unitree()
    monkeypatch.setattr(drivers, "_unitree_sdk", lambda: sdk)
    p = profile(drivers._G1_BODY_NAMES, verified=True)
    driver = G1UnitreeDriver(p, execute=True, interface="test0", hardware_config=g1_execution_config(p))
    driver.connect()
    driver.hold()
    driver.close()
    assert not pubs[0].messages


def test_external_driver_factory_reuses_arbitrary_profile_and_checks_contract(monkeypatch):
    p = profile(["a", "b", "c", "d", "e"])
    module = SimpleNamespace(make_driver=lambda profile, **options: SimulationDriver(profile))
    original = drivers.importlib.import_module
    monkeypatch.setattr(drivers.importlib, "import_module", lambda name: module if name == "demo_robot" else original(name))
    result = drivers.create_driver("demo_robot:make_driver", p, execute=False)
    assert result.profile.dimension == 5
    module.make_driver = lambda profile, **options: object()
    with pytest.raises(ValueError, match="contract"):
        drivers.create_driver("demo_robot:make_driver", p)


def test_g2_camera_pump_preserves_clock_metadata_and_skips_repeated_frames():
    import threading
    from teleop_v2.drivers import G2CameraPump
    received, metadata = [], []
    ready = threading.Event()

    class CameraDriver:
        def capture_cameras(self):
            return {"head": {"data": b"\xff\xd8fake-jpeg\xff\xd9", "encoding": "jpeg",
                             "timestamp_ns": 1_000_000, "source_timestamp_ns": 900_000,
                             "wall_minus_vendor_ns": 100_000, "clock": "host_unix_estimate"}}

    def callback(name, jpeg, timestamp):
        received.append((name, jpeg, timestamp))
        ready.set()

    pump = G2CameraPump(CameraDriver(), callback, fps=60, on_metadata=metadata.append)
    pump.start()
    assert ready.wait(1)
    time.sleep(0.04)
    pump.close()
    assert len(received) == 1
    assert metadata[0]["source_timestamp_ns"] == 900_000
    assert metadata[0]["wall_minus_vendor_ns"] == 100_000


def test_g2_camera_pump_reports_unmapped_clock():
    import threading
    from teleop_v2.drivers import G2CameraPump
    failed = threading.Event()

    class CameraDriver:
        def capture_cameras(self):
            return {"head": {"data": b"", "clock": "vendor"}}

    pump = G2CameraPump(CameraDriver(), lambda *args: None, on_error=lambda error: failed.set())
    pump.start()
    assert failed.wait(1)
    with pytest.raises(DriverError, match="clock bridge"):
        pump.close()
