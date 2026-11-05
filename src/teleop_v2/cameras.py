"""Explicit opt-in camera workers. Capture and encoding never run in the control loop.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time
from typing import Callable


class OpenCVCamera:
    """Read, encode and invoke callbacks in a worker. Failures remain visible through
    ``error`` and ``check``. Callback signature matches ``EpisodeRecorder.camera``.
    No camera is opened until ``start`` is explicitly called.
    """

    def __init__(self, name: str, source: int | str, callback: Callable[[str, bytes, int], None],
                 *, fps: float = 15, width: int | None = None, height: int | None = None,
                 jpeg_quality: int = 90, on_error: Callable[[Exception], None] | None = None):
        if not math.isfinite(fps) or fps <= 0 or not 1 <= jpeg_quality <= 100:
            raise ValueError("fps must be positive and JPEG quality must be in [1,100]")
        if any(value is not None and (type(value) is not int or value < 1) for value in (width, height)):
            raise ValueError("Camera width and height must be positive integers")
        self.name, self.source, self.callback = name, source, callback
        self.fps, self.width, self.height, self.jpeg_quality = fps, width, height, jpeg_quality
        self.on_error = on_error
        self.error: Exception | None = None
        self.frames = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._capture = None

    def start(self) -> OpenCVCamera:
        """Explicitly start; check exposes failures."""
        if self._thread is not None:
            raise RuntimeError("Camera worker has already been started")
        self._thread = threading.Thread(target=self._run, name=f"camera-{self.name}", daemon=True)
        self._thread.start()
        return self

    def _run(self) -> None:
        try:
            import cv2
            capture = cv2.VideoCapture(self.source)
            self._capture = capture
            if not capture.isOpened():
                raise RuntimeError(f"Cannot open camera source {self.source!r}")
            capture.set(cv2.CAP_PROP_FPS, self.fps)
            if self.width is not None:
                capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            if self.height is not None:
                capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            while not self._stop.is_set():
                started = time.monotonic()
                ok, frame = capture.read()
                timestamp = time.time_ns()
                if not ok:
                    raise RuntimeError(f"Camera {self.name!r} failed to read a frame")
                ok, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                if not ok:
                    raise RuntimeError(f"Camera {self.name!r} failed to encode JPEG")
                self.callback(self.name, jpeg.tobytes(), timestamp)
                self.frames += 1
                self._stop.wait(max(0, 1 / self.fps - (time.monotonic() - started)))
        except Exception as exc:
            self.error = exc
            if self.on_error is not None:
                self.on_error(exc)
        finally:
            if self._capture is not None:
                self._capture.release()

    def check(self) -> None:
        if self.error is not None:
            raise RuntimeError(f"Camera {self.name!r} failed: {self.error}") from self.error

    def close(self, timeout: float = 3) -> None:
        """Stop cameras before sealing the recorder so trailing frames cannot arrive late.
        """
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)
            if self._thread.is_alive():
                raise RuntimeError("Camera backend did not stop within timeout")
        self.check()


def export_video(episode: str | Path, camera: str, destination: str | Path, *, fps: float = 15) -> Path:
    """Optionally export a constant-rate preview; original timing remains in cameras.jsonl.
    """
    if not math.isfinite(fps) or fps <= 0:
        raise ValueError("fps must be positive")
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("Video export requires the optional OpenCV camera dependency") from exc
    from .recording import read_episode
    import numpy as np
    root = Path(episode).resolve()
    records = [item for item in read_episode(root)["cameras"] if item["camera"] == camera]
    if not records:
        raise ValueError(f"No frames recorded for camera {camera!r}")
    frames = sorted(records, key=lambda item: item["frame_index"])

    # Read bytes through Python so Unicode Windows paths do not depend on imread.
    def decode(record):
        return cv2.imdecode(np.frombuffer((root / record["relative_path"]).read_bytes(), dtype=np.uint8),
                            cv2.IMREAD_COLOR)
    first = decode(frames[0])
    if first is None:
        raise ValueError("Cannot decode first camera frame")
    size = (first.shape[1], first.shape[0])
    target = Path(destination)
    target.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(target), cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
    if not writer.isOpened():
        writer.release()
        raise RuntimeError("OpenCV could not open video encoder")
    try:
        for record in frames:
            frame = decode(record)
            if frame is None or (frame.shape[1], frame.shape[0]) != size:
                raise ValueError("Camera frame is unreadable or changes resolution")
            writer.write(frame)
    finally:
        writer.release()
    target.with_suffix(target.suffix + ".timestamps.json").write_text(
        json.dumps([record["timestamp_ns"] for record in frames]), encoding="utf-8")
    return target
