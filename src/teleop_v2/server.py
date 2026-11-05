"""Local operator-console HTTP service.

Mutation requires a per-process bearer token and a same-origin request. Default
binding is loopback. Remote XR clients can use explicit TLS certificate options;
the same console never opens SDK connections or performs IK inside HTTP handlers.
"""

from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import hmac
import hashlib
import json
from pathlib import Path
import secrets
import ssl
from urllib.parse import parse_qs, urlsplit

from .kinematics import KinematicModel
from .recording import inspect_episode, read_episode
from .runtime import TeleopRuntime
from .types import InputFrame, RobotProfile


class ConsoleServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: tuple[str, int], runtime: TeleopRuntime, *, token: str | None = None,
                 cert: str | None = None, key: str | None = None):
        self.runtime = runtime
        self.token = token or secrets.token_urlsafe(32)
        self.tls = bool(cert)
        if bool(cert) != bool(key):
            raise ValueError("TLS certificate and key must be supplied together")
        super().__init__(address, ConsoleHandler)
        if cert:
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.load_cert_chain(cert, key)
            self.socket = context.wrap_socket(self.socket, server_side=True)


class ConsoleHandler(BaseHTTPRequestHandler):
    server: ConsoleServer

    def log_message(self, *_args) -> None:
        # Keep tokens and frame polling out of logs.
        pass

    def _send(self, status: int, body: bytes, mime: str = "application/json; charset=utf-8") -> None:
        self.send_response(status)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Permissions-Policy", "xr-spatial-tracking=(self), gamepad=(self)")
        self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; "
                         "img-src 'self' blob: data:; connect-src 'self'; object-src 'none'; frame-ancestors 'none'")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, value: dict, status: int = 200) -> None:
        self._send(status, json.dumps(value, ensure_ascii=False, allow_nan=False).encode("utf-8"))

    def _authorized(self) -> bool:
        value = self.headers.get("X-Teleop-Token", "")
        authorization = self.headers.get("Authorization", "")
        if authorization.startswith("Bearer "):
            value = authorization[7:]
        if not hmac.compare_digest(value, self.server.token):
            self._json({"error": "Missing or invalid session token"}, 401)
            return False

        # Origin does not replace authentication; additionally reject cross-site browser requests.
        origin = self.headers.get("Origin")
        if origin:
            expected = ("https" if self.server.tls else "http") + "://" + self.headers.get("Host", "")
            if origin != expected:
                self._json({"error": "Cross-origin control is disabled"}, 403)
                return False
        return True

    def _episode(self, identifier: str) -> Path:
        root = self.server.runtime.recording_root
        selected = (root / identifier).resolve()
        if selected.parent != root or not selected.is_dir():
            raise ValueError("Choose an episode directly inside the configured recording directory")
        return selected

    def do_GET(self) -> None:
        parsed = urlsplit(self.path)
        if not parsed.path.startswith("/api/"):
            resources = {"/": ("index.html", "text/html; charset=utf-8"),
                         "/app.js": ("app.js", "text/javascript; charset=utf-8"),
                         "/style.css": ("style.css", "text/css; charset=utf-8")}
            if parsed.path not in resources:
                self._json({"error": "Not found"}, 404)
                return
            name, mime = resources[parsed.path]
            self._send(200, (Path(__file__).parent / "web" / name).read_bytes(), mime)
            return
        if not self._authorized():
            return
        try:
            runtime = self.server.runtime
            query = parse_qs(parsed.query)
            if parsed.path == "/api/state":
                self._json(runtime.snapshot())
            elif parsed.path == "/api/episodes":
                paths = sorted(runtime.recording_root.glob("*/manifest.json"), reverse=True)
                self._json({"episodes": [p.parent.name for p in paths[:200]]})
            elif parsed.path == "/api/episode":
                path = self._episode(query.get("id", [""])[0])
                if (path / "samples.jsonl").exists() and (path / "samples.jsonl").stat().st_size > 100_000_000:
                    raise ValueError("Episode too large for browser playback; use CLI export")
                data = read_episode(path)
                profile = RobotProfile.from_dict(data["profile"])
                model = KinematicModel(profile)
                samples = data["samples"]
                stride = max(1, (len(samples)+2999)//3000)
                frames = [{"wall_time_ns": s["wall_time_ns"], "state": s["state"],
                           "mode": s["mode"], "points": model.points(s["state"])} for s in samples[::stride]]
                self._json({"profile": data["profile"], "frames": frames, "stride": stride,
                            "sample_count": len(samples), "cameras": data["cameras"],
                            "id": path.name, "inspection": inspect_episode(path)})
            elif parsed.path == "/api/episode-camera":
                root = self._episode(query.get("id", [""])[0])
                name = query.get("camera", [""])[0]
                index = int(query.get("frame", ["-1"])[0])

                # Admit only frames present in the recorded index, never arbitrary paths.
                found = None
                with (root / "cameras.jsonl").open(encoding="utf-8") as stream:
                    for line in stream:
                        item = json.loads(line)
                        if item["camera"] == name and item["frame_index"] == index:
                            found = item
                            break
                if found is None:
                    raise ValueError("Recorded camera frame not found")
                target = (root / found["relative_path"]).resolve()
                if not target.is_relative_to(root / "cameras"):
                    raise ValueError("Invalid recorded camera path")
                jpeg = target.read_bytes()
                if hashlib.sha256(jpeg).hexdigest() != found["sha256"]:
                    raise ValueError("Recorded camera frame failed its integrity check")
                self._send(200, jpeg, "image/jpeg")
            elif parsed.path.startswith("/api/camera/"):
                name = parsed.path.removeprefix("/api/camera/")
                with runtime.lock:
                    jpeg = runtime._camera_last.get(name)
                if jpeg is None:
                    raise ValueError("Camera has no frame")
                self._send(200, jpeg, "image/jpeg")
            else:
                self._json({"error": "Not found"}, 404)
        except (ValueError, KeyError, OSError, RuntimeError) as error:
            self._json({"error": str(error)}, 400)

    def do_POST(self) -> None:
        if not self._authorized():
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 128_000:
                raise ValueError("Request body must be 1..128000 bytes")
            def reject_constant(value):
                raise ValueError(f"Non-finite JSON constant is forbidden: {value}")
            data = json.loads(self.rfile.read(length), parse_constant=reject_constant)
            if not isinstance(data, dict):
                raise ValueError("A JSON object is required")
            path = urlsplit(self.path).path
            if path == "/api/input":
                if type(data.get("sent_wall_ns")) is not int or data["sent_wall_ns"] <= 0:
                    raise ValueError("Input must include a positive integer sent_wall_ns timestamp")
                self.server.runtime.submit(InputFrame.from_dict(data))
                self._json({"accepted": True})
            elif path == "/api/action":
                action = data.pop("action")
                self._json(self.server.runtime.action(action, **data))
            else:
                self._json({"error": "Not found"}, 404)
        except (ValueError, TypeError, KeyError, OSError, RuntimeError) as error:
            self._json({"error": str(error)}, 400)
