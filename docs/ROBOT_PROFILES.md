# Robot descriptions and kinematics

The system operates on named coordinates and control groups. Robot dimension comes from a profile; input devices, recording and IK do not use a fixed 24- or 43-coordinate array.

## Bundled demonstration profiles

| ID | Coordinates | Contents |
|---|---:|---|
| `arm6` | 6 | Generic six-axis arm |
| `arm7` | 7 | Generic seven-axis arm |
| `g2_24` | 24 | Two 7-axis arms, two grippers, 3 head and 5 waist coordinates |
| `g1_29` | 29 | 12 leg, 3 waist and 14 arm coordinates |
| `g1_43` | 43 | 29 body coordinates and 7 coordinates per hand |

Every bundled profile sets `simulation_only: true` and `hardware_validated: false`. G2/G1 layouts exercise adapter boundaries. Link lengths, initial postures, finger synergies and most limits are demonstration parameters, not accepted hardware calibrations. G2 gripper coordinates represent physical opening in metres: 0 closed, 0.025 open. The old learned-policy gripper convention is not used here.

The G1 43-coordinate layout does not imply direct dynamically balanced leg control. Current FK/IK covers fixed-base upper-body geometry; leg coordinates can be observed and recorded. Locomotion should use the manufacturer's supported walking and balance interface.

## JSON profile

```json
{
  "schema_version": "teleop.robot/v1",
  "id": "my_robot",
  "display_name": "My robot",
  "joints": [
    {
      "name": "shoulder_joint", "group": "right_arm", "unit": "rad",
      "lower": -2.5, "upper": 2.5, "velocity": 0.5,
      "acceleration": 1.0, "initial": 0.0
    }
  ],
  "chains": {
    "right": {
      "base_xyz": [0, 0, 0], "base_rpy": [0, 0, 0],
      "segments": [
        {"joint": "shoulder_joint", "xyz": [0, 0, 0.3],
         "rpy": [0, 0, 0], "axis": [0, 0, 1], "type": "revolute"}
      ],
      "tool_xyz": [0.4, 0, 0], "tool_rpy": [0, 0, 0]
    }
  },
  "effectors": {},
  "metadata": {
    "simulation_only": true, "hardware_validated": false,
    "default_control_groups": ["right_arm"]
  }
}
```

The `joints` array defines the unique vector order for feedback, commands and recordings. Names must be unique. Groups select control ownership. Revolute joints use radians and prismatic joints use metres, with matching per-second velocity and per-second-squared acceleration units. Each segment applies the fixed origin before the joint motion. The tool transform follows the final joint. RPY follows the URDF convention and all quaternion APIs use XYZW order. Effector maps define named coordinates and open/closed endpoints; multi-joint hands may use a simple grasp synergy.

## Import an actual URDF

```python
from teleop_v2.profiles import profile_from_urdf, save_profile

profile = profile_from_urdf(
    "robot.urdf",
    chains={
        "left": {"base": "base_link", "tip": "left_tool"},
        "right": {"base": "base_link", "tip": "right_tool"},
    },
    profile_id="my_dual_arm",
    acceleration=0.8,
    initial={"left_elbow_joint": -0.5},
    groups={"waist_yaw_joint": "waist"},
)
save_profile(profile, "my_dual_arm.json")
```

The importer supports revolute, continuous, prismatic and fixed joints. Fixed transforms are multiplied exactly, preserving tool/mount offsets. Continuous joints receive an explicit local angle bound, ±π by default, configurable with `continuous_limit`. All tasks must share one base reference frame.

By default every non-fixed URDF joint becomes a recorded coordinate. Set `include_all_joints=False` to include only selected task paths. URDF position and velocity limits must be explicit; the caller supplies acceleration because it is not a standard URDF limit field. Mimic, floating and planar joints are explicitly rejected instead of silently treated as independent actuators.

A URDF does not prove SDK coordinate mapping, gripper direction, collision margins or site calibration. Imported profiles remain unvalidated for hardware. Hardware integration additionally requires driver mapping, permitted control groups and appropriate stop/hold behavior.

## IK capabilities and limits

`forward` returns a homogeneous task transform. `jacobian` returns a geometric Jacobian with the full profile width. `points` supplies preview polylines. `solve` handles all requested tasks jointly, including shared coordinates between chains.

The solver uses bounded damped least squares with backtracking. Iterates respect active joint position limits and preserve unrelated coordinates. Results include success, per-task position/orientation errors and iteration count. Unreachable targets remain failures; callers must not interpret a bounded approximation as a reached target.

This IK does not implement self-collision, environment avoidance, contact dynamics or humanoid balance. It does not replace robot-side velocity/acceleration limits, command expiry or hardware protection. A more constrained backend can be substituted behind the same task and result interfaces.
