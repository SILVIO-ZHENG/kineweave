"""Teleoperation runtime with a single robot writer.

HTTP threads only replace the latest input under a lock. The control thread owns
read/retarget/IK/guard/write in order; camera capture and disk I/O use other workers.
No queued command is replayed after a timeout, disarm or source change.
"""

from __future__ import annotations

from collections import deque
import threading
import time
from pathlib import Path
from typing import Any

from .control import JointGuard, Retargeter
from .kinematics import KinematicModel
from .recording import EpisodeRecorder
from .types import InputFrame, RobotDriver, RobotProfile


class TeleopRuntime:
    """Serialize hardware access and isolate display, input and disk workers. All SDK
    calls share one lock; only the periodic control path issues moving targets.
    Operator actions may request a serialized hold or read-only reset.
    """
    def __init__(self, profile: RobotProfile, driver: RobotDriver, *, frequency: float = 50,
                 timeout: float = 0.3, recording_root: str | Path = "recordings",
                 groups: list[str] | None = None, scale: float = 1.0):
        if not 5 <= frequency <= 200 or not 0.05 <= timeout <= 2:
            raise ValueError("frequency must be 5..200 Hz and timeout 0.05..2 seconds")
        self.profile, self.driver = profile, driver
        self.frequency, self.timeout = frequency, timeout
        self.recording_root = Path(recording_root).resolve()
        self.groups = groups or profile.metadata.get("default_control_groups")
        self.model = KinematicModel(profile)
        self.retarget = Retargeter(profile, self.model, scale)
        self.guard = JointGuard(profile, self.groups)
        self.lock = threading.RLock()
        self.recording_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread: threading.Thread | None = None
        self.mode = "disarmed"
        self.fault: str | None = None
        self.estop_latched = False
        self.owner: str | None = None
        self.latest: InputFrame | None = None
        self.received_ns = 0
        self.sequences: dict[str, int] = {}
        self.sequence = 0
        self.recorder: EpisodeRecorder | None = None
        self.last_episode: str | None = None
        self.state = None
        self.last_sample: dict = {}
        self.events: deque[dict] = deque(maxlen=30)
        self._was_active = False
        self._camera_last: dict[str, bytes] = {}
        self.camera_errors: dict[str, str] = {}

    def start(self, background: bool = True) -> None:
        """Connect and anchor to actual initial feedback."""
        self.driver.connect()
        try:
            state = self.driver.read()
            state.validate(self.profile)
            self.state = state
            self.guard.reset(state.position)
        except Exception:
            self.driver.close()
            raise
        self._event("connected", {"hardware": self.driver.hardware, "writable": self.driver.writable})
        if background:
            self.thread = threading.Thread(target=self._loop, name="teleop-control", daemon=True)
            self.thread.start()

    def _event(self, kind: str, details: dict | None = None) -> None:
        event = {"kind": kind, "wall_time_ns": time.time_ns(), "details": details or {}}
        self.events.append(event)
        if self.recorder:
            self.recorder.event(kind, event["details"])

    def submit(self, frame: InputFrame) -> None:
        """Keep only the newest control intent, while recording every admitted packet.
        Sequence numbers are scoped to a source and survive disarm/re-arm.
        """
        with self.lock:
            if self.owner and self.owner != frame.source_id:
                raise ValueError("Another input source owns control; disarm before changing source")
            if frame.sequence <= self.sequences.get(frame.source_id, -1):
                raise ValueError("Duplicate or out-of-order input sequence")
            if frame.sent_wall_ns:
                age = time.time_ns() - frame.sent_wall_ns
                if age > self.timeout*1e9 or age < -100_000_000:
                    raise ValueError("Input exceeded its transport TTL or sender clock is ahead; synchronize clocks")
            if len(self.sequences) >= 64 and frame.source_id not in self.sequences:
                raise ValueError("Too many input sources in one session")
            self.sequences[frame.source_id] = frame.sequence
            self.latest, self.received_ns = frame, time.monotonic_ns()
            if self.recorder:
                self.recorder.event("input_frame", {"frame": frame.to_dict(),
                                                    "received_monotonic_ns": self.received_ns})

    def action(self, action: str, **options: Any) -> dict:
        """Apply explicit operator actions. Arming grants ownership, while each fresh
        input packet must separately carry its continuously held enable signal.
        """

        # File creation, worker draining and hashing happen outside the control lock.
        if action in {"record_start", "record_stop"}:
            return self._record_action(action, **options)
        with self.lock:
            if action == "arm":
                if self.estop_latched:
                    raise ValueError("Reset the latched software stop first")
                if self.fault:
                    raise ValueError("Clear the fault before arming")
                if not self.driver.writable:
                    raise ValueError("This driver is read-only")
                source = options.get("source_id")
                if not source or not self.latest or self.latest.source_id != source:
                    raise ValueError("Send a fresh input packet from the selected source first")
                if time.monotonic_ns()-self.received_ns > self.timeout*1e9:
                    raise ValueError("Selected input is stale")
                self.owner, self.mode = source, "armed"
                self.retarget.reset()
                self.guard.reset(self.state.position)
            elif action in {"disarm", "pause", "estop"}:

                # Revoke authority before a fallible SDK call. Only reset clears the stop latch.
                self.estop_latched = self.estop_latched or action == "estop"
                self.mode = "estop" if self.estop_latched else "disarmed"
                self.owner, self.latest = None, None
                self.retarget.reset()
                self.guard.reset(self.state.position)
                self._was_active = False
                try:
                    self.driver.hold()
                except Exception as error:
                    self.fault = f"Robot hold failed: {error}"
                    if not self.estop_latched:
                        self.mode = "fault"
                    self._event("hold_failed", {"message": str(error)})
                    raise RuntimeError(self.fault) from error
            elif action == "reset":
                if self.mode not in {"fault", "estop", "disarmed"}:
                    raise ValueError("Disarm before resetting")
                checked = self.driver.read()
                checked.validate(self.profile)
                self.state = checked
                self.mode, self.fault, self.owner, self.latest = "disarmed", None, None, None
                self.estop_latched = False
                self.retarget.reset()
                self.guard.reset(checked.position)
            elif action == "calibrate":
                self.retarget.reset()
                self.guard.reset(self.state.position)
                self.latest = None
                self._was_active = False
                if self.owner:
                    self.mode = "armed"
                try:
                    self.driver.hold()
                except Exception as error:
                    self.owner = None
                    self.mode = "estop" if self.estop_latched else "fault"
                    self.fault = f"Robot hold failed: {error}"
                    raise RuntimeError(self.fault) from error
            else:
                raise ValueError(f"Unknown action: {action}")
            self._event(action)
            return self.snapshot()

    def _record_action(self, action: str, **options: Any) -> dict:
        with self.recording_lock:
            if action == "record_start":
                with self.lock:
                    if self.recorder:
                        raise ValueError("An episode is already recording")
                    if self.camera_errors:
                        raise ValueError(f"Configured cameras have failed: {self.camera_errors}")
                recorder = EpisodeRecorder(self.recording_root, self.profile,
                    metadata={"hardware": self.driver.hardware, "groups": self.groups,
                              "frequency": self.frequency, "task": str(options.get("task", ""))[:4096],
                              "camera_clocks": {"opencv": "host Unix UTC ns after frame read",
                                                "g2": "host estimate; raw vendor time retained by adapter"}})
                path = recorder.start()
                with self.lock:
                    self.last_episode, self.recorder = str(path), recorder
                    self._event(action)
            else:
                with self.lock:
                    if not self.recorder:
                        raise ValueError("No active recording")
                    self._event(action)
                    recorder, self.recorder = self.recorder, None
                path = recorder.close()
                with self.lock:
                    self.last_episode = str(path)
            return self.snapshot()

    def tick(self, dt: float | None = None) -> dict:
        """Execute one cycle; tests may step it explicitly."""
        with self.lock:
            try:
                return self._tick(dt or 1/self.frequency)
            except Exception as error:
                try:
                    self.driver.hold()
                except Exception:
                    pass
                self.mode, self.fault, self.owner = ("estop" if self.estop_latched else "fault"), str(error), None
                self.latest = None
                self.retarget.reset()
                self._was_active = False

                # Do not recursively write into a failed recorder and hide the original fault.
                try:
                    self._event("fault", {"message": str(error)})
                except Exception:
                    pass
                return {"mode": self.mode, "fault": self.fault}

    def _tick(self, dt: float) -> dict:
        state = self.driver.read()
        state.validate(self.profile)
        now = time.monotonic_ns()
        if now - state.timestamp_ns > max(0.25, self.timeout)*1e9 or state.timestamp_ns > now + 10_000_000:
            raise ValueError("Robot feedback is stale or has an invalid monotonic clock")
        self.state = state
        frame = self.latest
        fresh = frame is not None and now-self.received_ns <= self.timeout*1e9
        ready = self.mode in {"armed", "active", "hold"} and bool(self.owner)
        active = ready and fresh and frame.enabled and not frame.clutch and frame.valid and frame.confidence >= 0.5
        if ready and dt > 0.1:
            active = False
            self.latest = None
            self._event("deadline_missed", {"dt": dt})
        requested, command = state.position.copy(), state.position.copy()
        receipt: dict = {"accepted": False, "reason": "not_active"}
        dispatch_error: Exception | None = None
        ik: dict = {}
        guard: dict = {}
        if active:
            if not self._was_active:
                self.retarget.reset()
                self.guard.reset(state.position)
            targets, requested = self.retarget.map(frame, state.position, min(dt, 0.1))
            if targets:
                result = self.model.solve(state.position, targets)
                ik = {"success": bool(result.success), "errors": result.errors, "iterations": result.iterations}
                if not result.success:

                    # Hold on unreachable/unconverged targets; never dispatch a partial solution.
                    self.driver.hold()
                    self.retarget.reset()
                    self.guard.reset(state.position)
                    self.mode, active = "hold", False
                    receipt = {"accepted": False, "reason": "ik_failed"}
                else:
                    for name in targets:
                        for segment in self.profile.chains[name]["segments"]:
                            i = self.profile.index(segment["joint"])
                            requested[i] = result.position[i]
            if active:
                dispatch_now = time.monotonic_ns()
                if (dispatch_now-now > min(0.1, self.timeout)*1e9
                        or dispatch_now-self.received_ns > self.timeout*1e9):

                    # Solver time consumes command TTL; slow IK must never dispatch stale intent.
                    self.driver.hold()
                    self.retarget.reset()
                    self.guard.reset(state.position)
                    self.latest = None
                    self.mode, active = "hold", False
                    receipt = {"accepted": False, "reason": "computation_deadline"}
                    self._event("computation_deadline")
            if active:
                command, guard = self.guard.step(requested, state.position, min(dt, 0.1))
                try:
                    receipt = self.driver.write(command)
                except Exception as error:
                    receipt = {"accepted": False, "published": None, "reason": "driver_exception",
                               "error": str(error), "command_evidence": "attempted_only"}
                if not receipt.get("accepted", False):
                    dispatch_error = RuntimeError(f"Robot rejected command: {receipt}")

                # Drivers may shape further; retain both the adopted command and guard output.
                guarded_command = command.copy()
                if "command" in receipt:
                    from .types import finite_vector
                    command = finite_vector(receipt["command"], self.profile.dimension, "driver receipt command")
                guard["output"] = guarded_command.tolist()
                self.mode = "fault" if dispatch_error else "active"
        elif ready:
            if self._was_active:
                self.driver.hold()
            self.retarget.reset()
            self.guard.reset(state.position)
            self.mode = "hold"
        self._was_active = bool(active)
        self.sequence += 1
        sample = {"sequence": self.sequence, "wall_time_ns": time.time_ns(), "monotonic_ns": now,
                  "state": state.position.tolist(), "velocity": state.velocity.tolist(),
                  "requested": requested.tolist(), "command": command.tolist(), "mode": self.mode,
                  "input": frame.to_dict() if frame else None, "input_fresh": fresh,
                  "input_received_monotonic_ns": self.received_ns,
                  "feedback_monotonic_ns": state.timestamp_ns,
                  "feedback_metadata": state.metadata, "receipt": receipt, "ik": ik, "guard": guard}
        self.last_sample = sample
        if self.recorder:
            self.recorder.append(sample)
        if dispatch_error:

            # Record failed dispatch evidence before fault handling clears the input.
            raise dispatch_error
        return sample

    def _loop(self) -> None:
        previous = time.monotonic()
        while not self.stop_event.is_set():
            started = time.monotonic()
            self.tick(started-previous)
            previous = started
            self.stop_event.wait(max(0, 1/self.frequency - (time.monotonic()-started)))

    def camera(self, name: str, jpeg: bytes, timestamp_ns: int) -> None:
        """Camera workers pass encoded frames only."""
        with self.lock:
            self._camera_last[name] = jpeg
            if self.recorder:
                self.recorder.camera(name, jpeg, timestamp_ns)

    def camera_failed(self, name: str, error: Exception) -> None:
        """Preserve camera failures and invalidate capture."""
        with self.lock:
            self.camera_errors[name] = str(error)
            if self.recorder:
                self.recorder.mark_incomplete(f"camera {name}: {error}")
            self._event("camera_error", {"name": name, "message": str(error)})

    def camera_metadata(self, details: dict) -> None:
        """Preserve vendor timestamps and their host-clock mapping."""
        with self.lock:
            self._event("camera_timestamp", details)

    def snapshot(self) -> dict:
        """Read-only snapshot. Display playback never routes recorded motion to a driver.
        """
        with self.lock:
            q = self.state.position if self.state else self.profile.initial
            return {"profile": self.profile.to_dict(), "mode": self.mode, "fault": self.fault,
                    "hardware": self.driver.hardware, "writable": self.driver.writable,
                    "software_stop_latched": self.estop_latched,
                    "owner": self.owner, "state": q.tolist(),
                    "velocity": self.state.velocity.tolist() if self.state else [0]*self.profile.dimension,
                    "points": self.model.points(q), "recording": bool(self.recorder),
                    "episode": self.last_episode, "sequence": self.sequence,
                    "input_source": self.latest.source_id if self.latest else None,
                    "input_age_ms": (time.monotonic_ns()-self.received_ns)/1e6 if self.received_ns else None,
                    "events": list(self.events), "cameras": list(self._camera_last),
                    "camera_errors": dict(self.camera_errors), "control_groups": self.groups,
                    "ik": self.last_sample.get("ik", {})}

    def close(self) -> None:
        """Stop control, seal capture and release the SDK."""
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=5)
            if self.thread.is_alive():
                raise RuntimeError("Control thread did not stop; refusing concurrent SDK shutdown")
        with self.lock:
            try:
                self.driver.hold()
            finally:
                try:
                    if self.recorder:
                        recorder, self.recorder = self.recorder, None
                        self.last_episode = str(recorder.close())
                finally:
                    self.driver.close()
