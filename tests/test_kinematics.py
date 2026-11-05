"""Focused kinematic checks without hardware."""

import math

import numpy as np
import pytest

from teleop_v2.kinematics import KinematicModel, matrix_pose, pose_matrix, rotation_vector
from teleop_v2.profiles import list_profiles, load_profile, profile_from_urdf, save_profile
from teleop_v2.types import Pose


@pytest.mark.parametrize("profile_id,dimension", [("arm6", 6), ("arm7", 7), ("g2_24", 24), ("g1_29", 29), ("g1_43", 43)])
def test_profiles_have_dynamic_dimensions_and_bounded_ik(profile_id, dimension):
    profile = load_profile(profile_id)
    assert profile.dimension == dimension
    assert profile.metadata["simulation_only"]
    assert not profile.metadata["hardware_validated"]
    model = KinematicModel(profile)
    known = profile.initial
    for name, chain in profile.chains.items():
        for index, segment in enumerate(chain["segments"]):
            known[profile.index(segment["joint"])] += 0.03 * (1 if index % 2 else -1)
    targets = {name: model.forward(known, name) for name in profile.chains}
    result = model.solve(profile.initial, targets, position_tolerance=0.001, orientation_tolerance=0.01)
    assert result.success, result.errors
    assert all(j.lower <= q <= j.upper for j, q in zip(profile.joints, result.position))
    active = {segment["joint"] for chain in profile.chains.values() for segment in chain["segments"]}
    for index, joint in enumerate(profile.joints):
        if joint.name not in active:
            assert result.position[index] == profile.initial[index]
    assert set(model.points(result.position)) == set(profile.chains)


def test_jacobian_matches_actual_geometric_motion():
    profile = load_profile("arm7")
    model = KinematicModel(profile)
    position = profile.initial
    actual = model.forward(position, "right")
    jacobian = model.jacobian(position, "right")
    epsilon = 1e-7
    for index in range(profile.dimension):
        changed = position.copy()
        changed[index] += epsilon
        moved = model.forward(changed, "right")
        numerical = np.r_[(moved[:3, 3] - actual[:3, 3]) / epsilon,
                          rotation_vector(moved[:3, :3] @ actual[:3, :3].T) / epsilon]
        np.testing.assert_allclose(jacobian[:, index], numerical, atol=1e-6)


def test_unreachable_target_is_a_bounded_failure():
    profile = load_profile("arm6")
    model = KinematicModel(profile)
    target = model.forward(profile.initial, "right")
    target[:3, 3] += [50, 50, 50]
    result = model.solve(profile.initial, {"right": target}, max_iterations=15)
    assert not result.success
    assert result.iterations <= 15
    assert np.isfinite(result.position).all()
    assert all(j.lower <= q <= j.upper for j, q in zip(profile.joints, result.position))


@pytest.mark.parametrize("quaternion", [(0, 0, 0, 1), (1, 0, 0, 0), (0, math.sin(1.2), 0, math.cos(1.2)), (0.5, 0.5, 0.5, 0.5)])
def test_pose_roundtrip_including_pi(quaternion):
    matrix = pose_matrix(Pose((0.2, -0.4, 1.0), quaternion))
    np.testing.assert_allclose(pose_matrix(matrix_pose(matrix)), matrix, atol=1e-10)


def test_bad_transforms_and_unknown_tasks_rejected():
    model = KinematicModel(load_profile("arm6"))
    bad = np.eye(4)
    bad[0, 0] = 2
    with pytest.raises(ValueError):
        model.solve(model.profile.initial, {"right": bad})
    with pytest.raises(ValueError):
        model.solve(model.profile.initial, {"missing": np.eye(4)})
    with pytest.raises(ValueError):
        model.forward([0], "right")


def test_urdf_fixed_origins_and_prismatic_motion(tmp_path):
    urdf = tmp_path / "slider.urdf"
    urdf.write_text('''<robot name="slider">
      <link name="base"/><link name="mount"/><link name="carriage"/><link name="tool"/>
      <joint name="mount_fixed" type="fixed"><parent link="base"/><child link="mount"/>
        <origin xyz="1 0 0" rpy="0 0 1.5707963267948966"/></joint>
      <joint name="slide" type="prismatic"><parent link="mount"/><child link="carriage"/>
        <origin xyz="0.2 0 0"/><axis xyz="1 0 0"/><limit lower="0" upper="1" velocity="0.1"/></joint>
      <joint name="tool_fixed" type="fixed"><parent link="carriage"/><child link="tool"/>
        <origin xyz="0 0 0.3" rpy="0.1 0 0"/></joint>
    </robot>''', encoding="utf-8")
    profile = profile_from_urdf(urdf, {"right": {"base": "base", "tip": "tool"}})
    model = KinematicModel(profile)
    np.testing.assert_allclose(model.forward([0.4], "right")[:3, 3], [1, .6, .3], atol=1e-12)
    np.testing.assert_allclose(model.jacobian([0.4], "right")[:, 0], [0, 1, 0, 0, 0, 0], atol=1e-12)
    target = model.forward([.7], "right")
    result = model.solve([.2], {"right": target})
    assert result.success
    assert abs(result.position[0] - .7) < .003
    saved = save_profile(profile, tmp_path / "profile.json")
    assert load_profile(saved).joint_names == ("slide",)


def test_urdf_mimic_is_explicitly_rejected(tmp_path):
    path = tmp_path / "mimic.urdf"
    path.write_text('''<robot name="invalid"><link name="base"/><link name="tip"/>
      <joint name="finger" type="revolute"><parent link="base"/><child link="tip"/>
      <limit lower="0" upper="1" velocity="1"/><mimic joint="other"/></joint></robot>''', encoding="utf-8")
    with pytest.raises(ValueError, match="Mimic"):
        profile_from_urdf(path, {"right": {"base": "base", "tip": "tip"}})


def test_profile_discovery_and_missing_path(tmp_path):
    assert {p["dimension"] for p in list_profiles()} >= {6, 7, 24, 29, 43}
    with pytest.raises(FileNotFoundError):
        load_profile(tmp_path / "missing.json")
