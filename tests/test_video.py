"""Real JPEG/MP4 export without opening a camera."""

import time

import numpy as np
import pytest

from teleop_v2.cameras import export_video
from teleop_v2.profiles import load_profile
from teleop_v2.recording import EpisodeRecorder, inspect_episode, read_episode


def test_encoded_camera_frames_roundtrip_and_video(tmp_path):
    cv2 = pytest.importorskip("cv2")
    recorder = EpisodeRecorder(tmp_path, load_profile("arm6"),
                               metadata={"camera_clock": "host UTC ns", "synthetic_test": True})
    episode = recorder.start()
    timestamps = []
    for index in range(3):
        image = np.full((48, 64, 3), 40+index*50, dtype=np.uint8)
        ok, jpeg = cv2.imencode(".jpg", image)
        assert ok
        timestamp = time.time_ns()
        timestamps.append(timestamp)
        recorder.camera("head", jpeg.tobytes(), timestamp)
    recorder.close()
    assert inspect_episode(episode)["verified"]
    assert [frame["timestamp_ns"] for frame in read_episode(episode)["cameras"]] == timestamps
    output = export_video(episode, "head", tmp_path / "preview.mp4")
    video = cv2.VideoCapture(str(output))
    try:
        assert int(video.get(cv2.CAP_PROP_FRAME_COUNT)) == 3
        ok, image = video.read()
        assert ok and image.shape[:2] == (48, 64)
    finally:
        video.release()
