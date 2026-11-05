"""Exercise the actual HTTP handler on an ephemeral loopback port, with an in-memory
robot driver. Authentication, input ordering and display-only replay are checked
at their public network boundary rather than by mocking handler internals.
"""

from contextlib import contextmanager
import json
import threading
import time
from urllib.error import HTTPError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

import pytest

from teleop_v2.profiles import load_profile
from teleop_v2.runtime import TeleopRuntime
from teleop_v2.server import ConsoleServer
from test_runtime import MemoryDriver


@contextmanager
def console(tmp_path):
    profile = load_profile("g1_43")
    driver = MemoryDriver(profile)
    runtime = TeleopRuntime(profile, driver, recording_root=tmp_path)
    runtime.start(background=False)
    server = ConsoleServer(("127.0.0.1", 0), runtime, token="test-session-token")
    worker = threading.Thread(target=server.serve_forever, kwargs={"poll_interval": .02}, daemon=True)
    worker.start()
    base = f"http://127.0.0.1:{server.server_port}"

    def request(path, body=None, *, token="test-session-token", origin=None):
        headers = {"Content-Type": "application/json"}
        if token is not None:
            headers["X-Teleop-Token"] = token
        if origin is not None:
            headers["Origin"] = base if origin == "self" else origin
        payload = None if body is None else json.dumps(body).encode("utf-8")
        query = Request(base + path, data=payload, headers=headers)
        try:
            with urlopen(query, timeout=3) as response:
                payload = response.read()
                return response.status, (payload if response.headers.get_content_type() == "image/jpeg"
                                         else json.loads(payload))
        except HTTPError as error:
            return error.code, json.loads(error.read())

    try:
        yield request, runtime, driver
    finally:
        server.shutdown()
        server.server_close()
        worker.join(timeout=2)
        runtime.close()


def network_input(sequence=0, **extra):
    data = {"source_id": "http-controller", "sequence": sequence, "enabled": True,
            "sent_wall_ns": time.time_ns(), "joints": {"right_shoulder_pitch_joint": .2}}
    data.update(extra)
    return data


def test_real_http_requires_token_and_rejects_cross_origin(tmp_path):
    with console(tmp_path) as (request, runtime, driver):
        assert request("/api/state", token=None)[0] == 401
        assert request("/api/state", token="incorrect")[0] == 401
        assert request("/api/input", network_input(), token=None)[0] == 401
        assert request("/api/input", network_input(), origin="https://another.example")[0] == 403
        status, state = request("/api/state", origin="self")
        assert status == 200 and len(state["state"]) == 43
        assert runtime.latest is None and not driver.commands


def test_http_input_order_source_and_clock_validated(tmp_path):
    with console(tmp_path) as (request, runtime, driver):
        assert request("/api/input", network_input(5), origin="self")[0] == 200
        assert request("/api/input", network_input(5))[0] == 400
        assert request("/api/input", network_input(4))[0] == 400
        assert runtime.latest.sequence == 5
        assert request("/api/action", {"action": "arm", "source_id": "http-controller"})[0] == 200
        assert request("/api/input", network_input(6, source_id="other"))[0] == 400
        assert request("/api/input", network_input(6, sent_wall_ns=time.time_ns()-3_000_000_000))[0] == 400
        assert runtime.latest.sequence == 5 and not driver.commands


def test_http_record_and_display_replay_never_dispatches(tmp_path):
    with console(tmp_path) as (request, runtime, driver):
        assert request("/api/action", {"action": "record_start", "task": "HTTP capture"})[0] == 200
        assert request("/api/input", network_input())[0] == 200
        assert request("/api/action", {"action": "arm", "source_id": "http-controller"})[0] == 200
        runtime.tick()
        assert len(driver.commands) == 1
        assert request("/api/action", {"action": "record_stop"})[0] == 200
        status, listing = request("/api/episodes")
        assert status == 200 and len(listing["episodes"]) == 1
        identifier = listing["episodes"][0]
        count = len(driver.commands)
        status, replay = request("/api/episode?" + urlencode({"id": identifier}))
        assert status == 200 and replay["sample_count"] == 1
        assert len(replay["frames"][0]["state"]) == 43
        assert replay["inspection"]["complete"] and replay["inspection"]["verified"]
        assert len(driver.commands) == count
        assert request("/api/episode?" + urlencode({"id": "../"}))[0] == 400


@pytest.mark.parametrize("clock_case", ["missing", "zero", "delayed", "future"])
def test_network_input_requires_timely_sender_timestamp(tmp_path, clock_case):
    with console(tmp_path) as (request, runtime, driver):
        data = network_input()
        if clock_case == "missing":
            data.pop("sent_wall_ns")
        elif clock_case == "zero":
            data["sent_wall_ns"] = 0
        elif clock_case == "delayed":
            data["sent_wall_ns"] -= 1_000_000_000
        else:
            data["sent_wall_ns"] += 1_000_000_000
        assert request("/api/input", data)[0] == 400
        assert runtime.latest is None and not driver.commands


@pytest.mark.parametrize("field", ["poses", "twists", "joints", "grasp"])
def test_malformed_input_objects_receive_json_error(tmp_path, field):
    with console(tmp_path) as (request, runtime, driver):
        status, response = request("/api/input", network_input(**{field: []}))
        assert status == 400 and "error" in response
        assert runtime.latest is None and not driver.commands


def test_replay_integrity_failure_receives_json_error(tmp_path):
    with console(tmp_path) as (request, runtime, driver):
        assert request("/api/action", {"action": "record_start"})[0] == 200
        runtime.tick()
        assert request("/api/action", {"action": "record_stop"})[0] == 200
        path = runtime.recording_root / runtime.last_episode
        with (path / "samples.jsonl").open("a", encoding="utf-8") as stream:
            stream.write("{}\n")
        status, response = request("/api/episode?" + urlencode({"id": path.name}))
        assert status == 400 and "integrity" in response["error"]
        assert not driver.commands


def test_recorded_camera_endpoint_checks_frame_hash(tmp_path):
    with console(tmp_path) as (request, runtime, driver):
        assert request("/api/action", {"action": "record_start"})[0] == 200
        runtime.tick()
        jpeg = b"\xff\xd8synthetic-jpeg-fixture\xff\xd9"
        runtime.camera("head", jpeg, time.time_ns())
        assert request("/api/action", {"action": "record_stop"})[0] == 200
        identifier = runtime.last_episode.rsplit("\\", 1)[-1].rsplit("/", 1)[-1]
        query = "/api/episode-camera?" + urlencode({"id": identifier, "camera": "head", "frame": 0})
        assert request(query) == (200, jpeg)
        target = runtime.recording_root / identifier / "cameras" / "head" / "00000000.jpg"
        target.write_bytes(jpeg + b"tampered")
        status, error = request(query)
        assert status == 400 and "integrity" in error["error"]
        assert not driver.commands
