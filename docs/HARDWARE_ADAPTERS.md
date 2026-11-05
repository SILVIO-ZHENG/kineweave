# Hardware adapters and validation boundary

The core is independent of robot dimensions. `drivers.py` provides a simulator,
a G2 execution bridge and a G1 DDS driver. The implementations have been tested
with fake SDKs; no physical robot was connected or moved during development.

## 1. Common contract

Drivers implement `RobotDriver`: `connect/read/write/hold/close`. Vector ordering
must match `RobotProfile.joint_names`, with explicit joint units. State timestamps
use the process monotonic clock; metadata includes host Unix wall time. G1 retains
the actual DDS receive time, so repeated reads cannot make stale feedback appear fresh.

`accepted` means driver admission/publication. `physical_accepted` is true only
when the SDK supplies hardware admission evidence. G1 DDS publication leaves it
null; later feedback is needed to assess motion. Simulation is explicitly marked.

## 2. Simulation driver

The simulator accepts any valid profile. Writes update targets; reads integrate
motion with speed/acceleration bounds. It is a kinematic preview, without gravity,
contact, friction, whole-body balance or physical communication dynamics.

## 3. G2: reuse the existing GDK executor

Run in the robot's Linux environment with `agibot_gdk` and Nexus dependencies.
`nexus_root` points at the original repository or its `thor_project` directory.
Only the G2 execution adapter is imported; no learner or model weights are needed.

```python
from teleop_v2.drivers import create_driver
from teleop_v2.profiles import load_profile

# Read real G2 feedback; writes are disabled and connect does not enter servo mode.
driver = create_driver(
    "g2", load_profile("g2_24"),
    nexus_root="/opt/nexus",
    execute=False,
)
```

Use the deployment path on the actual Linux machine. The default mapping follows
the existing 24D canonical order. Custom names require a complete bijective
`hardware_config.joint_map`, from canonical names to profile names.

Dahuan grippers use physical measured opening: 0 m closed, 0.025 m open. The
driver applies no policy-action inversion. The original adapter still handles
GDK conversion, SDK locking, rate limits and collision checks.

Physical writes require the following site-specific materials; bundled demo
models do not satisfy them:

- Explicit `execute=True` or CLI `--execute`.
- A custom profile with actual kinematics and reviewed joint limits, `metadata.hardware_validated=true` and `simulation_only=false`.
- A calibrated Nexus `safety_config` file.
- `hardware_config.site_verified=true` and the installed `effector_profile`.

These flags declare completed site verification; they do not automatically prove
geometry, firmware or workspace validity. Connection reads feedback before
calling the existing execution preparation. Hold retires pending motion and
anchors at fresh measured pose, never a zero target.

Camera capture is separate from joint reads. It includes encoding, dimensions,
native vendor timestamps and the estimated host Unix mapping. The estimated
clock bridge is not a claim of hardware synchronization. Raw frames must be
encoded before passing them to a JPEG-only recording entry point.

## 4. G1: 29D body and 43D Dex3

Use the official SDK2 Python and CycloneDDS. Read-only mode subscribes only to:

| Data | DDS topic | Type |
|---|---|---|
| Body | `rt/lowstate` | `unitree_hg.msg.dds_.LowState_` |
| Left hand | `rt/dex3/left/state` | `HandState_` |
| Right hand | `rt/dex3/right/state` | `HandState_` |

Body motors 0–11 are legs, 12–14 waist, 15–28 arms. The 43D profile additionally
requires real feedback from both hands. Missing/stale hand feedback fails instead
of fabricating zeros. Standard names map automatically; other hands require an
explicit mapping or a dedicated driver.

Dex3 motor ordering differs by side:

| motor | Left | Right |
|---|---|---|
| 0,1,2 | thumb 0,1,2 | thumb 0,1,2 |
| 3,4 | middle 0,1 | index 0,1 |
| 5,6 | index 0,1 | middle 0,1 |

Physical writes use `rt/arm_sdk` and optional hand command topics. This driver
does not create `rt/lowcmd`, switch locomotion controllers or actively command
leg positions. Waist targets remain measured; active waist motion requires
separate integration and validation. Motor slot 29 carries official arm SDK
ownership weight, ramped down during normal close.

G1 execution configuration has the following shape. Gains must come from the
actual reviewed hardware configuration; no production gains are bundled.

```json
{
  "site_verified": false,
  "firmware_watchdog_verified": false,
  "mode_pr": 0,
  "controlled_joints": ["left_shoulder_pitch_joint"],
  "gains": {
    "left_shoulder_pitch_joint": {"kp": 1.0, "kd": 0.1}
  }
}
```

This is a structural fragment, not an executable configuration. Every body
waist/arm coordinate requires gains; controlling a Dex3 side requires gains for
all seven motors on that side. `controlled_joints` declares allowed arm/hand
motion. Optional body/hand maps override defaults. Execution requires the full
29-motor body map and an explicit network interface.

Uncontrolled coordinates must preserve the latest observation returned to the caller,
refreshed each runtime tick. Before dispatch the driver independently checks freshness
and refreshes uncontrolled coordinates to avoid balancing-feedback races.
Receipts prove publication only. Final behavior on process loss,
network loss or feedback failure depends on the site-verified robot watchdog;
this Python client does not provide hard-real-time balance control.

## 5. Add another robot

Add a profile, IK/retargeting parameters and a driver implementing the five
methods. Inject it into the runtime or select `--driver my_robot.driver:make_driver`.
The input protocol, recorder, episode format and UI remain unchanged. Declare
the robot's actual supported control groups explicitly.

The factory receives `profile, **options` and returns a RobotDriver. Construction
must not connect hardware; connection belongs in `connect()`. Respect `execute=False`
and expose accurate hardware/writable/profile properties. Only a user-selected
local module is loaded through the CLI; browser requests cannot load plugins.

### Background G2 camera capture

`--g2-cameras` explicitly enables the built-in G2 cameras. After the runtime
connects the driver, `G2CameraPump` captures and encodes in a separate worker.
JPEG passes through; raw RGB/BGR uses optional OpenCV. Duplicate vendor frames
are skipped. Backward clocks, missing clock mappings and encoding failures are
reported via on_error/error/check. Per-frame on_metadata retains native timestamps,
clock offsets and encoding. Shutdown order is camera pump → recorder/runtime → SDK.

## Official API references

- [G1 seven-DoF-arm SDK DDS example](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/g1/high_level/g1_arm7_sdk_dds_example.py)
- [Unitree official SDK2 Python](https://github.com/unitreerobotics/unitree_sdk2_python)
- [Official Dex3 hand controller and DDS message usage](https://github.com/unitreerobotics/xr_teleoperate/blob/main/teleop/robot_control/robot_hand_unitree.py)
- G2 bridge dependency: a compatible external installation of `nexus_deploy.robot.g2a.gdk_adapter`. The related [G2 collection and execution project](https://github.com/SILVIO-ZHENG/VLA_RL_collect_data) provides project background; check the actual adapter API in the selected installation.

The interfaces were checked during development on 2026-11-05. Deployment must
match the installed SDK, firmware and exact robot configuration.
