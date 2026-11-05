"""Launch, offline demo and dataset inspection."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
import threading
import time
import webbrowser

from .drivers import create_driver
from .profiles import list_profiles, load_profile
from .recording import export_csv, inspect_episode, read_episode
from .runtime import TeleopRuntime
from .server import ConsoleServer
from .types import InputFrame


def _print(value) -> None:
    print(json.dumps(value, ensure_ascii=False, indent=2, default=str))


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description="KineWeave: modular multi-robot teleoperation and data capture")
    commands = result.add_subparsers(dest="command", required=True)
    commands.add_parser("profiles", help="List robot layouts")
    demo = commands.add_parser("demo", help="Offline simulated recording")
    demo.add_argument("--robot", default="g1_43")
    demo.add_argument("--seconds", type=float, default=3)
    demo.add_argument("--output", default="recordings")
    demo.add_argument("--all-profiles", action="store_true")
    serve = commands.add_parser("serve", help="Start console")
    serve.add_argument("--robot", default="g2_24", help="Built-in ID or JSON")
    serve.add_argument("--driver", default="sim", help="sim, g2, g1 or module:factory")
    serve.add_argument("--host", default="127.0.0.1")
    serve.add_argument("--port", type=int, default=8765)
    serve.add_argument("--frequency", type=float, default=50)
    serve.add_argument("--timeout", type=float, default=0.3)
    serve.add_argument("--scale", type=float, default=1)
    serve.add_argument("--groups", help="Comma-separated control groups")
    serve.add_argument("--output", default="recordings")
    serve.add_argument("--execute", action="store_true", help="Opt into physical writes")
    serve.add_argument("--nexus-root", help="Existing G2 Nexus project")
    serve.add_argument("--safety-config", help="Validated G2 safety config")
    serve.add_argument("--interface", help="G1 DDS network interface")
    serve.add_argument("--hardware-config", help="Site hardware config JSON")
    serve.add_argument("--cert", help="TLS certificate for remote XR")
    serve.add_argument("--key", help="TLS private key")
    serve.add_argument("--open", action="store_true", help="Open local browser")
    serve.add_argument("--camera", action="append", default=[], metavar="NAME=DEVICE",
                       help="Explicitly enable OpenCV camera")
    serve.add_argument("--g2-cameras", action="store_true", help="Capture native G2 cameras")
    video = commands.add_parser("export-video", help="Export camera preview video")
    video.add_argument("episode")
    video.add_argument("--camera", required=True)
    video.add_argument("--output", required=True)
    video.add_argument("--fps", type=float, default=15)
    for name in ("inspect", "replay", "export"):
        command = commands.add_parser(name)
        command.add_argument("episode")
        if name == "export":
            command.add_argument("--output", required=True)
    return result


def run_demo(robot: str, seconds: float, output: str) -> dict:
    if not 0.1 <= seconds <= 60:
        raise ValueError("Demo duration must be 0.1..60 seconds")
    profile = load_profile(robot)
    runtime = TeleopRuntime(profile, create_driver("sim", profile), recording_root=output)
    runtime.start(background=False)
    try:
        runtime.action("record_start", task="Local synthetic teleoperation acceptance")
        selected = next(j for j in profile.joints if j.group in runtime.groups)
        target = min(selected.upper-0.01, max(selected.lower+0.01, selected.initial+0.15))
        frame = InputFrame("demo", 0, enabled=True, joints={selected.name: target})
        runtime.submit(frame)
        runtime.action("arm", source_id="demo")
        start = time.monotonic()
        count = 0
        while time.monotonic()-start < seconds:
            count += 1
            runtime.submit(InputFrame("demo", count, enabled=True, joints={selected.name: target}))
            sample = runtime.tick(1/runtime.frequency)
            if runtime.fault:
                raise RuntimeError(runtime.fault)
            time.sleep(1/runtime.frequency)
        runtime.action("disarm")
        runtime.tick()
        runtime.action("record_stop")
        return {"robot": profile.id, "dimension": profile.dimension,
                "episode": runtime.last_episode, "last_mode": sample["mode"],
                "inspection": inspect_episode(runtime.last_episode)}
    finally:
        runtime.close()


def serve(args) -> None:
    profile = load_profile(args.robot)
    driver = create_driver(args.driver, profile, execute=args.execute, nexus_root=args.nexus_root,
                           safety_config=args.safety_config, interface=args.interface,
                           hardware_config=args.hardware_config, control_hz=args.frequency)
    groups = args.groups.split(",") if args.groups else None
    runtime = TeleopRuntime(profile, driver, frequency=args.frequency, timeout=args.timeout,
                           recording_root=args.output, groups=groups, scale=args.scale)
    server = ConsoleServer((args.host, args.port), runtime, cert=args.cert, key=args.key)
    cameras = []
    try:
        runtime.start()
        for specification in args.camera:
            from .cameras import OpenCVCamera
            name, device = specification.split("=", 1)

            # Open only explicitly selected devices; never probe all available cameras.
            camera = OpenCVCamera(name, int(device) if device.isdecimal() else device, runtime.camera,
                                  on_error=lambda error, camera_name=name: runtime.camera_failed(camera_name, error))
            camera.start()
            cameras.append(camera)
        if args.g2_cameras:
            from .drivers import G2CameraPump
            pump = G2CameraPump(driver, runtime.camera,
                                on_error=lambda error: runtime.camera_failed("g2_native", error),
                                on_metadata=runtime.camera_metadata)
            pump.start()
            cameras.append(pump)
        host = "127.0.0.1" if args.host == "0.0.0.0" else args.host
        url = f"{'https' if args.cert else 'http'}://{host}:{server.server_port}/#token={server.token}"
        _print({"console": url, "robot": profile.id, "dimension": profile.dimension,
                "driver": args.driver, "hardware": driver.hardware, "writable": driver.writable})
        if args.open:
            threading.Timer(0.3, lambda: webbrowser.open(url)).start()
        server.serve_forever(poll_interval=0.2)
    finally:
        try:
            for camera in cameras:
                try:
                    camera.close()
                except RuntimeError as error:
                    runtime.camera_failed(getattr(camera, "name", "native"), error)
        finally:
            server.server_close()
            runtime.close()


def main(argv: list[str] | None = None) -> int:
    # Preserve Unicode output through Windows pipes too.
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8")
    args = parser().parse_args(argv)
    try:
        if args.command == "profiles":
            _print({"profiles": list_profiles()})
        elif args.command == "demo":
            robots = [p["id"] for p in list_profiles()] if args.all_profiles else [args.robot]
            _print({"results": [run_demo(robot, args.seconds, args.output) for robot in robots]})
        elif args.command == "serve":
            serve(args)
        elif args.command == "inspect":
            _print(inspect_episode(args.episode))
        elif args.command == "export":
            _print({"csv": str(export_csv(args.episode, Path(args.output)))})
        elif args.command == "export-video":
            from .cameras import export_video
            _print({"video": str(export_video(args.episode, args.camera, args.output, fps=args.fps))})
        elif args.command == "replay":
            data = read_episode(args.episode)
            samples = data["samples"]
            _print({"robot": data["profile"]["id"], "samples": len(samples),
                    "first": samples[0] if samples else None,
                    "last": samples[-1] if samples else None,
                    "note": "Offline read; console provides interactive playback"})
        return 0
    except KeyboardInterrupt:
        return 0
    except (ValueError, RuntimeError, OSError, ImportError) as error:
        print(f"Error: {error}")
        return 1
