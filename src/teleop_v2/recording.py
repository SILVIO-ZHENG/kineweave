"""Asynchronous recording, inspection and reading without control or learning dependencies.
"""

from __future__ import annotations

from contextlib import ExitStack
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
import csv
import hashlib
import json
from pathlib import Path
import queue
import re
import threading
import time
from typing import Any
from uuid import uuid4

import h5py
import numpy as np

from .types import RobotProfile, finite_vector


SCHEMA = "teleop.episode/v1"
VECTOR_FIELDS = ("state", "velocity", "requested", "command")
TIME_FIELDS = ("wall_time_ns", "monotonic_ns", "sequence")
_CAMERA_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,63}\Z")


class RecordingError(RuntimeError):
    """Recording cannot be claimed complete."""


class RecordingQueueFull(RecordingError):
    """Explicit bounded-queue overflow."""


def _json_default(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if is_dataclass(value) and not isinstance(value, type):
        return asdict(value)
    if isinstance(value, Path):
        return str(value)
    raise TypeError(f"Cannot record JSON value of type {type(value).__name__}")


def _json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, allow_nan=False, default=_json_default,
                      separators=(",", ":"))


def _hash(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_json(path: Path, value: dict) -> None:
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(_json(value) + "\n", encoding="utf-8")
    temporary.replace(path)


def _records(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    with path.open(encoding="utf-8") as stream:
        return [json.loads(line) for line in stream if line.strip()]


class EpisodeRecorder:
    """One writer owns all disk resources. Control calls validate, snapshot and enqueue
    without waiting for disk I/O. Queue overflow invalidates the entire episode.
    ``close`` drains accepted records and must be called outside the control loop.
    """

    def __init__(self, root: str | Path, profile: RobotProfile, metadata: dict | None = None,
                 queue_size: int = 512):
        if type(queue_size) is not int or queue_size < 1:
            raise ValueError("queue_size must be a positive integer")
        self.root = Path(root)
        self.profile = profile
        self.metadata = json.loads(_json(metadata or {}))
        self._queue: queue.Queue = queue.Queue(maxsize=queue_size)
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._thread: threading.Thread | None = None
        self._path: Path | None = None
        self._closed = False
        self._closing = False
        self._overflowed = False
        self._writer_error: BaseException | None = None
        self._incomplete_reasons: list[str] = []
        self._last_sequence = -1
        self._last_monotonic = -1
        self._counts = {"samples": 0, "events": 0, "camera_frames": 0}
        self._created_ns = 0

    def start(self) -> Path:
        """Create the crash marker before starting the writer, outside the servo loop.
        """
        with self._lock:
            if self._path is not None:
                if self._closed:
                    raise RecordingError("Recorder is already closed")
                return self._path
            self._created_ns = time.time_ns()
            label = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S")
            self._path = self.root / f"episode-{label}-{uuid4().hex[:12]}"
            self._path.mkdir(parents=True, exist_ok=False)
            (self._path / "INCOMPLETE").write_text("Recorder has not completed sealing.\n", encoding="utf-8")
            _atomic_json(self._path / "profile.json", self.profile.to_dict())
            _atomic_json(self._path / "metadata.json", self.metadata)
            _atomic_json(self._path / "manifest.json", self._manifest(False, []))
            self._thread = threading.Thread(target=self._write_loop, name="episode-writer", daemon=True)
            self._thread.start()
        self._ready.wait()
        if self._writer_error is not None:
            raise RecordingError(f"Cannot start episode writer: {self._writer_error}") from self._writer_error
        return self._path

    def _check_open(self) -> None:
        if self._path is None or self._closed or self._closing:
            raise RecordingError("Recorder must be started and open")
        if self._writer_error is not None:
            raise RecordingError(f"Episode writer failed: {self._writer_error}") from self._writer_error
        if self._overflowed:
            raise RecordingQueueFull("Episode is incomplete after a recording queue overflow")

    def _enqueue(self, item: tuple) -> None:
        self._check_open()
        try:
            self._queue.put_nowait(item)
        except queue.Full as exc:
            self._overflowed = True
            raise RecordingQueueFull("Recording queue full; episode marked incomplete") from exc

    def append(self, sample: dict) -> None:
        """Preserve every sample field while validating dimension and increasing order.
        """
        snapshot = json.loads(_json(sample))
        for field in TIME_FIELDS:
            value = snapshot.get(field)
            if (type(value) is not int or value < (0 if field == "sequence" else 1)
                    or value > np.iinfo(np.int64).max):
                raise ValueError(f"{field} must be a nonnegative sequence or positive integer timestamp")
        for field in VECTOR_FIELDS:
            finite_vector(snapshot.get(field), self.profile.dimension, field)
        encoded = _json(snapshot)
        with self._lock:
            self._check_open()
            if snapshot["sequence"] <= self._last_sequence or snapshot["monotonic_ns"] <= self._last_monotonic:
                raise ValueError("Sample sequence and monotonic_ns must strictly increase")
            self._enqueue(("sample", snapshot, encoded))
            self._last_sequence = snapshot["sequence"]
            self._last_monotonic = snapshot["monotonic_ns"]

    def event(self, kind: str, details: dict) -> None:
        """Events carry receive clocks; source timestamps can be retained in details.
        """
        if not isinstance(kind, str) or not kind or not isinstance(details, dict):
            raise ValueError("Events require a nonempty kind and a details mapping")
        record = {"event_id": str(uuid4()), "kind": kind, "wall_time_ns": time.time_ns(),
                  "monotonic_ns": time.monotonic_ns(), "details": details}
        encoded = _json(record)
        with self._lock:
            self._enqueue(("event", encoded))

    def camera(self, name: str, jpeg: bytes, timestamp_ns: int) -> None:
        """Enqueue encoded JPEG bytes; image encoding and disk writes stay off control paths.
        """
        if not isinstance(name, str) or not _CAMERA_NAME.fullmatch(name) or name in {".", ".."}:
            raise ValueError("Camera name must be a safe bounded identifier")
        if type(timestamp_ns) is not int or timestamp_ns <= 0:
            raise ValueError("Camera timestamp_ns must be a positive integer")
        if not isinstance(jpeg, (bytes, bytearray, memoryview)):
            raise ValueError("Camera frame must contain encoded JPEG bytes")
        data = bytes(jpeg)
        if not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9"):
            raise ValueError("Camera frame lacks JPEG start/end markers")
        with self._lock:
            self._enqueue(("camera", name, data, timestamp_ns, time.monotonic_ns()))

    def _write_loop(self) -> None:
        try:
            assert self._path is not None
            with ExitStack() as stack:
                samples = stack.enter_context((self._path / "samples.jsonl").open("w", encoding="utf-8",
                                                                                 buffering=1))
                events = stack.enter_context((self._path / "events.jsonl").open("w", encoding="utf-8",
                                                                                buffering=1))
                camera_index = stack.enter_context((self._path / "cameras.jsonl").open("w", encoding="utf-8",
                                                                                      buffering=1))
                h5 = stack.enter_context(h5py.File(self._path / "samples.h5", "w"))
                h5.attrs["schema_version"] = SCHEMA
                h5.attrs["joint_names_json"] = _json(self.profile.joint_names)
                h5.attrs["joint_units_json"] = _json([joint.unit for joint in self.profile.joints])
                datasets = {}
                for field in TIME_FIELDS:
                    datasets[field] = h5.create_dataset(field, (0,), maxshape=(None,), dtype="i8", chunks=True)
                for field in VECTOR_FIELDS:
                    datasets[field] = h5.create_dataset(field, (0, self.profile.dimension),
                                                       maxshape=(None, self.profile.dimension), dtype="f8",
                                                       chunks=True)
                h5.flush()
                self._ready.set()
                camera_counts: dict[str, int] = {}
                while not self._stop.is_set() or not self._queue.empty():
                    try:
                        item = self._queue.get(timeout=0.05)
                    except queue.Empty:
                        continue
                    try:
                        if item[0] == "sample":
                            _, snapshot, encoded = item
                            index = self._counts["samples"]
                            for field, dataset in datasets.items():
                                dataset.resize(index + 1, axis=0)
                                dataset[index] = snapshot[field]
                            samples.write(encoded + "\n")
                            self._counts["samples"] += 1
                            h5.flush()
                        elif item[0] == "event":
                            events.write(item[1] + "\n")
                            self._counts["events"] += 1
                        elif item[0] == "camera":
                            _, name, data, timestamp, received = item
                            index = camera_counts.get(name, 0)
                            relative = f"cameras/{name}/{index:08d}.jpg"
                            target = self._path / relative
                            target.parent.mkdir(parents=True, exist_ok=True)
                            target.write_bytes(data)
                            camera_index.write(_json({"camera": name, "frame_index": index,
                                                      "timestamp_ns": timestamp,
                                                      "received_monotonic_ns": received,
                                                      "relative_path": relative, "size_bytes": len(data),
                                                      "sha256": hashlib.sha256(data).hexdigest()}) + "\n")
                            camera_counts[name] = index + 1
                            self._counts["camera_frames"] += 1
                    finally:
                        self._queue.task_done()
        except BaseException as exc:
            self._writer_error = exc
        finally:
            self._ready.set()

    def _manifest(self, complete: bool, files: list[dict]) -> dict:
        reasons = list(self._incomplete_reasons)
        if self._overflowed:
            reasons.append("recording_queue_overflow")
        if self._writer_error is not None:
            reasons.append(f"writer_error: {self._writer_error}")
        return {"schema_version": SCHEMA, "episode_id": self._path.name if self._path else "",
                "profile_id": self.profile.id, "dimension": self.profile.dimension,
                "joint_names": list(self.profile.joint_names), "created_wall_time_ns": self._created_ns,
                "closed_wall_time_ns": time.time_ns() if self._closing else None,
                "complete": complete, "status": "complete" if complete else "incomplete",
                "incomplete_reasons": reasons, "counts": dict(self._counts), "files": files,
                "clock": {"sample_wall": "Unix UTC nanoseconds", "sample_monotonic": "process monotonic ns",
                          "camera": "timestamp_ns supplied by camera source; declare its clock in metadata",
                          "cross_process_monotonic_join": False}}

    def mark_incomplete(self, reason: str) -> None:
        """Let callers register camera or device failures without performing disk I/O.
        """
        if not isinstance(reason, str) or not reason:
            raise ValueError("An incomplete reason must be a nonempty string")
        with self._lock:
            if self._closed:
                raise RecordingError("Cannot invalidate an already sealed episode")
            self._incomplete_reasons.append(reason)

    def close(self) -> Path:
        """Drain and seal. Abnormal termination leaves INCOMPLETE and cannot report completion.
        """
        with self._lock:
            if self._path is None:
                raise RecordingError("Recorder was not started")
            if self._closed:
                return self._path
            self._closing = True
            self._stop.set()
        assert self._thread is not None
        self._thread.join()
        files = [{"relative_path": item.relative_to(self._path).as_posix(),
                  "size_bytes": item.stat().st_size, "sha256": _hash(item)}
                 for item in sorted(self._path.rglob("*"))
                 if item.is_file() and item.name not in {"manifest.json", "INCOMPLETE"}]
        complete = not self._overflowed and self._writer_error is None and not self._incomplete_reasons
        _atomic_json(self._path / "manifest.json", self._manifest(complete, files))
        if complete:
            (self._path / "INCOMPLETE").unlink()
        else:
            (self._path / "INCOMPLETE").write_text(_json(self._manifest(False, [])[
                "incomplete_reasons"]) + "\n", encoding="utf-8")
        self._closed = True
        if self._writer_error is not None:
            raise RecordingError(f"Episode writer failed; incomplete data at {self._path}") from self._writer_error
        return self._path

    def __enter__(self) -> EpisodeRecorder:
        self.start()
        return self

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        if exc_type is not None:
            self.mark_incomplete(f"application_error: {exc_value}")
        self.close()


def inspect_episode(path: str | Path, verify: bool = True) -> dict:
    """Report completeness and integrity errors, including discoverable incomplete recordings.
    """
    root = Path(path).resolve()
    manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    if manifest.get("schema_version") != SCHEMA:
        raise ValueError("Unsupported episode schema")
    errors = []
    if verify:
        declared = set()
        for record in manifest.get("files", []):
            relative = record["relative_path"]
            target = (root / relative).resolve()
            if not target.is_relative_to(root) or target == root:
                errors.append(f"Invalid file path: {relative}")
                continue
            declared.add(relative)
            if not target.is_file():
                errors.append(f"Missing file: {relative}")
            elif target.stat().st_size != record["size_bytes"] or _hash(target) != record["sha256"]:
                errors.append(f"File hash/size mismatch: {relative}")
        if manifest.get("complete"):
            actual = {item.relative_to(root).as_posix() for item in root.rglob("*") if item.is_file()
                      and item.name not in {"manifest.json", "INCOMPLETE"}}
            if actual != declared:
                errors.append("Manifest file inventory does not match episode files")
        try:
            samples = _records(root / "samples.jsonl")
            events = _records(root / "events.jsonl")
            camera_frames = _records(root / "cameras.jsonl")
            with h5py.File(root / "samples.h5", "r") as h5:
                if any(len(h5[field]) != len(samples) for field in (*TIME_FIELDS, *VECTOR_FIELDS)):
                    errors.append("HDF5 and full sample log lengths differ")
                if manifest.get("complete"):
                    for name, count in (("samples", len(samples)), ("events", len(events)),
                                        ("camera_frames", len(camera_frames))):
                        if count != manifest["counts"][name]:
                            errors.append(f"Manifest {name} count differs from stored records")
            for frame in camera_frames:
                target = (root / frame["relative_path"]).resolve()
                if not target.is_relative_to(root) or target == root:
                    errors.append(f"Invalid camera path: {frame['relative_path']}")
                elif (not target.is_file() or target.stat().st_size != frame["size_bytes"]
                      or _hash(target) != frame["sha256"]):
                    errors.append(f"Camera frame hash/size mismatch: {frame['relative_path']}")
        except (OSError, KeyError, ValueError, json.JSONDecodeError) as exc:
            errors.append(f"Cannot read sample storage: {exc}")
    complete = manifest.get("complete") is True and not (root / "INCOMPLETE").exists()
    return {"path": str(root), "manifest": manifest, "complete": complete,
            "status": "complete" if complete else "incomplete", "verified": verify and not errors,
            "integrity_errors": errors, "counts": manifest.get("counts", {}),
            "profile_id": manifest.get("profile_id"), "dimension": manifest.get("dimension")}


def read_episode(path: str | Path) -> dict:
    """Read complete sample fields for UI replay and independent dataset adapters.
    """
    root = Path(path)
    inspected = inspect_episode(root, verify=True)
    if inspected["integrity_errors"]:
        raise RecordingError("Episode integrity verification failed: " + "; ".join(inspected["integrity_errors"]))
    return {"path": str(root.resolve()), "profile": json.loads((root / "profile.json").read_text("utf-8")),
            "metadata": json.loads((root / "metadata.json").read_text("utf-8")),
            "samples": _records(root / "samples.jsonl"), "events": _records(root / "events.jsonl"),
            "cameras": _records(root / "cameras.jsonl"), "manifest": inspected["manifest"],
            "complete": inspected["complete"]}


def export_csv(path: str | Path, destination: str | Path) -> Path:
    """Expand numeric joint columns and retain other complete fields as JSON columns.
    """
    episode = read_episode(path)
    names = [joint["name"] for joint in episode["profile"]["joints"]]
    samples = episode["samples"]
    extras = sorted(set().union(*(set(sample) for sample in samples)) - set(TIME_FIELDS) - set(VECTOR_FIELDS))
    fields = list(TIME_FIELDS) + [f"{field}.{name}" for field in VECTOR_FIELDS for name in names] + extras
    target = Path(destination)
    target.parent.mkdir(parents=True, exist_ok=True)
    with target.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for sample in samples:
            row = {field: sample[field] for field in TIME_FIELDS}
            row.update({f"{field}.{name}": sample[field][i]
                        for field in VECTOR_FIELDS for i, name in enumerate(names)})
            row.update({field: _json(sample.get(field)) for field in extras})
            writer.writerow(row)
    return target
