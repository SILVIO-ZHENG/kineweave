"""Focused recording checks without hardware."""

import csv
from pathlib import Path
import sys
import threading
from types import SimpleNamespace

import h5py
import pytest

from teleop_v2.recording import (EpisodeRecorder, RecordingError, RecordingQueueFull,
                                export_csv, inspect_episode, read_episode)
from teleop_v2.types import JointSpec, RobotProfile


def profile(n=3):
    return RobotProfile("test", "Test", tuple(JointSpec(f"j{i}", -2, 2, 1, 2) for i in range(n)))


def sample(sequence=0, n=3):
    return {"wall_time_ns": 1_000_000 + sequence, "monotonic_ns": 500 + sequence,
            "sequence": sequence, "state": [0.1] * n, "velocity": [0.0] * n,
            "requested": [0.2] * n, "command": [0.15] * n, "mode": "human",
            "input": {"raw": {"button": True}}, "receipt": {"accepted": True},
            "ik": {"success": True}, "future_extension": {"owner_mask": [True] * n}}


@pytest.mark.parametrize("dimension", [1, 7, 24])
def test_round_trip_complete_samples_and_hashes(tmp_path, dimension):
    recorder = EpisodeRecorder(tmp_path, profile(dimension), {"camera_clock": "Unix UTC ns"})
    path = recorder.start()
    assert (path / "INCOMPLETE").exists()
    assert not inspect_episode(path, verify=False)["complete"]
    original = sample(n=dimension)
    recorder.append(original)
    original["input"]["raw"]["button"] = False
    recorder.event("ownership_changed", {"owner": "human"})
    recorder.camera("left", b"\xff\xd8test-jpeg\xff\xd9", 100)
    assert recorder.close() == path
    summary = inspect_episode(path)
    assert summary["complete"] and summary["verified"]
    assert summary["counts"] == {"samples": 1, "events": 1, "camera_frames": 1}
    episode = read_episode(path)
    assert episode["samples"][0]["input"]["raw"]["button"] is True
    assert episode["samples"][0]["future_extension"]["owner_mask"] == [True] * dimension
    assert episode["cameras"][0]["timestamp_ns"] == 100
    with h5py.File(path / "samples.h5", "r") as h5:
        assert h5["state"].shape == (1, dimension)
        assert h5["command"][0].tolist() == [0.15] * dimension
    csv_path = export_csv(path, tmp_path / "export.csv")
    with csv_path.open(encoding="utf-8-sig", newline="") as stream:
        row = next(csv.DictReader(stream))
    assert float(row["state.j0"]) == 0.1
    assert '"button":true' in row["input"]
    (path / "cameras" / "left" / "00000000.jpg").write_bytes(b"tampered")
    assert not inspect_episode(path)["verified"]
    with pytest.raises(RecordingError, match="integrity"):
        read_episode(path)


def test_overflow_is_explicit_and_episode_cannot_be_complete(tmp_path):
    recorder = EpisodeRecorder(tmp_path, profile(), queue_size=1)
    ready, release = threading.Event(), threading.Event()
    real_loop = recorder._write_loop

    def paused_loop():
        recorder._ready.set()
        ready.set()
        release.wait(5)
        real_loop()

    recorder._write_loop = paused_loop
    recorder.start()
    assert ready.wait(1)
    recorder.append(sample())
    with pytest.raises(RecordingQueueFull):
        recorder.append(sample(1))
    release.set()
    path = recorder.close()
    inspected = inspect_episode(path)
    assert inspected["verified"] and not inspected["complete"]
    assert "recording_queue_overflow" in inspected["manifest"]["incomplete_reasons"]
    assert (path / "INCOMPLETE").is_file()


def test_wrong_dimensions_and_unsafe_camera_rejected(tmp_path):
    recorder = EpisodeRecorder(tmp_path, profile())
    recorder.start()
    with pytest.raises(ValueError, match="length 3"):
        recorder.append(sample(n=2))
    with pytest.raises(ValueError, match="safe"):
        recorder.camera("../escape", b"\xff\xd8x\xff\xd9", 100)
    recorder.append(sample())
    with pytest.raises(ValueError, match="strictly increase"):
        recorder.append(sample())
    assert inspect_episode(recorder.close())["complete"]


def test_application_exception_remains_visible_and_marks_incomplete(tmp_path):
    recorder = EpisodeRecorder(tmp_path, profile())
    with pytest.raises(ValueError, match="operator failure"):
        with recorder:
            recorder.append(sample())
            raise ValueError("operator failure")
    inspected = inspect_episode(recorder.close())
    assert not inspected["complete"]
    assert inspected["verified"]
    assert inspected["manifest"]["incomplete_reasons"] == ["application_error: operator failure"]


def test_background_disk_failure_cannot_seal_complete(tmp_path, monkeypatch):
    recorder = EpisodeRecorder(tmp_path, profile())
    path = recorder.start()
    write_bytes = Path.write_bytes

    def fail_jpeg(target, data):
        if target.suffix == ".jpg":
            raise OSError("disk full")
        return write_bytes(target, data)

    monkeypatch.setattr(Path, "write_bytes", fail_jpeg)
    recorder.camera("left", b"\xff\xd8x\xff\xd9", 100)
    with pytest.raises(RecordingError, match="writer failed"):
        recorder.close()
    assert not inspect_episode(path)["complete"]
    assert "disk full" in inspect_episode(path)["manifest"]["incomplete_reasons"][0]


def test_camera_is_opt_in_and_reports_worker_failure(monkeypatch):
    from teleop_v2.cameras import OpenCVCamera

    opened, received = [], []

    class Capture:
        def __init__(self, source):
            opened.append(source)

        def isOpened(self):
            return False

        def release(self):
            pass

    monkeypatch.setitem(sys.modules, "cv2", SimpleNamespace(VideoCapture=Capture))
    camera = OpenCVCamera("left", 0, lambda *args: received.append(args))
    assert not opened
    camera.start()
    camera._thread.join(1)
    assert opened == [0] and not received
    with pytest.raises(RuntimeError, match="Cannot open"):
        camera.close()
