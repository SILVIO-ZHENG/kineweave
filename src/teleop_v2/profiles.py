"""Load and discover robot descriptions, and import explicit serial chains from URDF.
The supplied profiles are deliberately labelled demonstration models. Their joint
layouts exercise different dimensions; their geometry is not a hardware calibration.
"""

from __future__ import annotations

from importlib.resources import files
import json
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET

import numpy as np

from .types import JointSpec, RobotProfile


_ALIASES = {
    "g2": "g2_24", "g2a": "g2_24", "g1": "g1_29", "g1-dex3": "g1_43",
    "generic6": "arm6", "generic7": "arm7",
}


def list_profiles() -> list[dict[str, Any]]:
    """List bundled layouts without importing any SDK."""
    result = []
    for item in sorted(files("teleop_v2").joinpath("profiles").iterdir(), key=lambda p: p.name):
        if item.name.endswith(".json"):
            profile = RobotProfile.from_dict(json.loads(item.read_text(encoding="utf-8")))
            result.append({
                "id": profile.id,
                "display_name": profile.display_name,
                "dimension": profile.dimension,
                "chains": list(profile.chains),
                "simulation_only": bool(profile.metadata.get("simulation_only", True)),
                "hardware_validated": bool(profile.metadata.get("hardware_validated", False)),
            })
    return result


def load_profile(id_or_path: str | Path) -> RobotProfile:
    """Load an ID or a custom JSON file, then validate the layout and kinematic model.
    A missing custom path is never silently replaced with a demonstration robot.
    """
    name = str(id_or_path)
    path = Path(name).expanduser()
    if path.is_file():
        data = json.loads(path.read_text(encoding="utf-8-sig"))
    else:
        name = _ALIASES.get(name, name)
        if Path(name).name != name or name.lower().endswith(".json"):
            raise FileNotFoundError(f"Robot profile does not exist: {path}")
        resource = files("teleop_v2").joinpath("profiles", f"{name}.json")
        if not resource.is_file():
            available = ", ".join(p["id"] for p in list_profiles())
            raise ValueError(f"Unknown robot profile {name!r}; available: {available}")
        data = json.loads(resource.read_text(encoding="utf-8"))
    profile = RobotProfile.from_dict(data)

    # Import lazily so contract definitions remain independent of kinematics.
    from .kinematics import KinematicModel
    KinematicModel(profile)
    return profile


def save_profile(profile: RobotProfile, destination: str | Path) -> Path:
    """Save a human-reviewable robot JSON description."""
    path = Path(destination)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(profile.to_dict(), indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return path


def _numbers(text: str | None, default: list[float], count: int = 3) -> list[float]:
    values = default if text is None else [float(x) for x in text.split()]
    if len(values) != count or not all(math.isfinite(x) for x in values):
        raise ValueError("URDF vector has invalid length or non-finite coordinates")
    return values


def profile_from_urdf(
    path: str | Path,
    chains: dict[str, dict[str, str]],
    *,
    profile_id: str = "custom_urdf",
    display_name: str | None = None,
    initial: dict[str, float] | None = None,
    groups: dict[str, str] | None = None,
    acceleration: float = 1.0,
    continuous_limit: float = math.pi,
    include_all_joints: bool = True,
) -> RobotProfile:
    """Import a URDF using ``{task: {base: link_name, tip: link_name}}``. Every task
    base must name the same root reference link, keeping task poses in one frame.
    Revolute, continuous and prismatic joints are supported; fixed transforms are
    preserved exactly. Mimic, floating and planar joints are rejected explicitly:
    they require an actuator/constraint model that this importer cannot infer.

    URDF velocity and position limits are required. Acceleration is not standard
    URDF data, so the caller supplies a conservative positive value. Continuous
    coordinates receive an explicitly bounded local range rather than infinity.
    Imported descriptions remain unvalidated for hardware until site acceptance.
    """
    from .kinematics import _rpy_matrix, _matrix_rpy

    if not chains or acceleration <= 0 or not math.isfinite(acceleration):
        raise ValueError("At least one chain and a positive acceleration limit are required")
    if continuous_limit <= 0 or not math.isfinite(continuous_limit):
        raise ValueError("continuous_limit must be finite and positive")
    bases = {description["base"] for description in chains.values()}
    if len(bases) != 1:
        raise ValueError("All imported chains must share one base link / coordinate frame")
    root = ET.parse(Path(path)).getroot()
    if root.tag != "robot":
        raise ValueError("URDF root element must be robot")
    elements = list(root.findall("joint"))
    joints_by_name: dict[str, ET.Element] = {}
    parent_by_child: dict[str, ET.Element] = {}
    links = {element.attrib["name"] for element in root.findall("link")}
    for element in elements:
        name = element.attrib["name"]
        if name in joints_by_name:
            raise ValueError(f"Duplicate URDF joint: {name}")
        if element.find("mimic") is not None:
            raise ValueError(f"Mimic joint requires an explicit actuator mapping: {name}")
        if element.attrib.get("type") not in {"fixed", "revolute", "continuous", "prismatic"}:
            raise ValueError(f"Unsupported URDF joint type: {element.attrib.get('type')}")
        parent = element.find("parent")
        child = element.find("child")
        if parent is None or child is None:
            raise ValueError(f"Joint {name} is missing parent or child")
        if parent.attrib["link"] not in links or child.attrib["link"] not in links:
            raise ValueError(f"Joint {name} references an unknown link")
        if child.attrib["link"] in parent_by_child:
            raise ValueError("URDF must be a tree: one link has multiple parents")
        joints_by_name[name] = element
        parent_by_child[child.attrib["link"]] = element

    def path_to(base: str, tip: str) -> list[ET.Element]:
        if base not in links or tip not in links:
            raise ValueError(f"Unknown base/tip link: {base} / {tip}")
        reversed_path = []
        seen = set()
        while tip != base:
            if tip in seen or tip not in parent_by_child:
                raise ValueError(f"No acyclic directed URDF path from {base} to requested tip")
            seen.add(tip)
            element = parent_by_child[tip]
            reversed_path.append(element)
            tip = element.find("parent").attrib["link"]
        return list(reversed(reversed_path))

    chain_paths = {name: path_to(item["base"], item["tip"]) for name, item in chains.items()}
    selected = {joint.attrib["name"] for chain in chain_paths.values() for joint in chain}
    specs = []
    initial = initial or {}
    groups = groups or {}
    for element in elements:
        name = element.attrib["name"]
        joint_type = element.attrib["type"]
        if joint_type == "fixed" or (not include_all_joints and name not in selected):
            continue
        limit = element.find("limit")
        if limit is None or "velocity" not in limit.attrib:
            raise ValueError(f"URDF joint {name} requires a velocity limit")
        if joint_type == "continuous":
            low, high = -continuous_limit, continuous_limit
        else:
            if "lower" not in limit.attrib or "upper" not in limit.attrib:
                raise ValueError(f"URDF joint {name} requires lower and upper limits")
            low, high = float(limit.attrib["lower"]), float(limit.attrib["upper"])
        owner = next((task for task, chain in chain_paths.items() if element in chain), "other")
        specs.append(JointSpec(
            name=name, lower=low, upper=high, velocity=float(limit.attrib["velocity"]),
            acceleration=acceleration,
            initial=float(initial.get(name, min(high, max(low, 0.0)))),
            unit="m" if joint_type == "prismatic" else "rad",
            group=groups.get(name, f"{owner}_arm" if owner != "other" else "other"),
        ))
    if set(initial) - {spec.name for spec in specs}:
        raise ValueError("Initial values include joints outside the imported layout")
    converted_chains = {}
    for task, chain in chain_paths.items():
        pending = np.eye(4)
        segments = []
        for element in chain:
            origin = element.find("origin")
            fixed = np.eye(4)
            fixed[:3, 3] = _numbers(None if origin is None else origin.get("xyz"), [0, 0, 0])
            fixed[:3, :3] = _rpy_matrix(_numbers(None if origin is None else origin.get("rpy"), [0, 0, 0]))
            pending = pending @ fixed
            if element.attrib["type"] == "fixed":
                continue
            axis = element.find("axis")
            segments.append({
                "joint": element.attrib["name"],
                "xyz": pending[:3, 3].tolist(),
                "rpy": _matrix_rpy(pending[:3, :3]).tolist(),
                "axis": _numbers(None if axis is None else axis.get("xyz"), [1, 0, 0]),
                "type": "prismatic" if element.attrib["type"] == "prismatic" else "revolute",
            })
            pending = np.eye(4)
        converted_chains[task] = {
            "base_xyz": [0, 0, 0], "base_rpy": [0, 0, 0], "segments": segments,
            "tool_xyz": pending[:3, 3].tolist(), "tool_rpy": _matrix_rpy(pending[:3, :3]).tolist(),
        }
    profile = RobotProfile(
        id=profile_id, display_name=display_name or root.get("name", profile_id),
        joints=tuple(specs), chains=converted_chains,
        metadata={
            "source_urdf": str(Path(path).resolve()), "simulation_only": True,
            "hardware_validated": False, "geometry": "imported_urdf_unvalidated",
            "base_link": next(iter(bases)), "default_control_groups": sorted({s.group for s in specs if s.group != "other"}),
            "notes": "URDF supplies kinematics; driver mapping, collision geometry and site calibration still require acceptance.",
        },
    )
    from .kinematics import KinematicModel
    KinematicModel(profile)
    return profile
