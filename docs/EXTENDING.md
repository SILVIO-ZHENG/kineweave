# Extend robots, drivers and inputs

Adding a robot has two separate parts: RobotProfile declares coordinates/kinematics, and RobotDriver implements feedback, dispatch and hold. Input sources implement the [InputFrame protocol](INPUT_PROTOCOL.md). Recording automatically uses the profile dimension N without format changes.

## 1. Custom JSON profile

The complete two-joint **simulation demonstration** below can be saved as `config/two_joint.json` and used by the same joint/task console. Values demonstrate the format and are not hardware calibration.

```json
{
  "schema_version": "teleop.robot/v1",
  "id": "two_joint_demo",
  "display_name": "Two-joint demonstration",
  "joints": [
    {"name": "joint1", "lower": -1.5, "upper": 1.5, "velocity": 0.4,
     "acceleration": 1.0, "initial": 0.0, "unit": "rad", "group": "right_arm"},
    {"name": "joint2", "lower": -1.2, "upper": 1.2, "velocity": 0.4,
     "acceleration": 1.0, "initial": 0.0, "unit": "rad", "group": "right_arm"}
  ],
  "chains": {
    "right": {
      "base_xyz": [0, 0, 0.2], "base_rpy": [0, 0, 0],
      "segments": [
        {"joint": "joint1", "xyz": [0, 0, 0], "rpy": [0, 0, 0],
         "axis": [0, 0, 1], "type": "revolute"},
        {"joint": "joint2", "xyz": [0.25, 0, 0], "rpy": [0, 0, 0],
         "axis": [0, 0, 1], "type": "revolute"}
      ],
      "tool_xyz": [0.2, 0, 0], "tool_rpy": [0, 0, 0]
    }
  },
  "effectors": {},
  "metadata": {
    "simulation_only": true, "hardware_validated": false,
    "default_control_groups": ["right_arm"], "geometry": "demonstration"
  }
}
```

```powershell
kineweave serve --robot config/two_joint.json --driver sim --open
kineweave demo --robot config/two_joint.json --seconds 2 --output recordings
```

A two-joint chain cannot reach arbitrary six-DoF poses; unreachable IK results are expected. Use Joint target to inspect coordinates first. For N joints, keep names unique, initial positions within limits, and finite positive velocity/acceleration limits. Segment xyz/rpy, axes and joint type determine FK; chain names are poses/twists keys in InputFrame.

Effectors use `{task: {joints: [...], open: [...], closed: [...]}}`. Grasp 0 means open and 1 means closed; endpoints must match units and limits. This describes a gripper or finger synergy, not fingertip-target IK. Control groups come from metadata by default or CLI `--groups left_arm,right_arm`; unselected coordinates retain fresh measured values.

## 2. Import URDF

URDF import is a Python API rather than an import-urdf CLI command. Select serial paths from one shared base link to task tips, then save JSON. Replace the filenames/link names below with actual values from your URDF.

```python
from teleop_v2.profiles import profile_from_urdf, save_profile

profile = profile_from_urdf(
    "config/robot.urdf",
    chains={"right": {"base": "base_link", "tip": "tool0"}},
    profile_id="my_arm",
    acceleration=0.5,
    continuous_limit=3.141592653589793,
    include_all_joints=True,
)
save_profile(profile, "config/my_arm.json")
```

```powershell
kineweave serve --robot config/my_arm.json --driver sim --open
```

Revolute, continuous, prismatic and fixed joints are supported; fixed transforms are folded rather than discarded. URDF supplies position/velocity limits, continuous joints get an explicit local bounded range, and callers supply acceleration. Mimic/floating/planar joints are rejected because they need actuator/constraint models. Tasks share one base. Including all joints preserves off-chain coordinates for recording; they default to group other and do not automatically acquire chains.

Import provides kinematics, not collision meshes, inertial dynamics, motor maps, hand synergies or calibration. Imported profiles remain simulation_only until actual site acceptance. **Changing metadata flags is a declaration, not acceptance.**

## 3. RobotDriver plugin contract

A driver exposes profile, boolean hardware/writable and the five methods below. Construction and module import must not connect or send hardware commands; explicit connect opens the connection. execute=False must remain read-only.

| Method | Semantics |
|---|---|
| `connect() -> None` | Connect, wait for real feedback, fail explicitly and release partial resources |
| `read() -> RobotState` | Measured N-dimensional position/velocity and local monotonic feedback timestamp |
| `write(target) -> dict` | Send N-dimensional physical-position targets and return delivery evidence |
| `hold() -> None` | Retire pending motion and hold while preserving vendor gravity/stabilization semantics |
| `close() -> None` | Stop or hand back ownership, release resources and handle partial connection failures |

Stamp actual local receive time for new feedback and preserve that timestamp when returning cached feedback. Do not refresh timestamps on every read to hide stale data. Runtime checks expiry. SDK send success often establishes publication only; physical_accepted should remain false/unknown unless the SDK has explicit acceptance evidence. If a vendor safety layer changes targets, retain dispatched command and native evidence in receipt.

Use --driver module:factory to load an explicitly selected Python module. The callable receives `(profile, **options)` and returns a conforming driver. Common options include execute, hardware_config, interface, nexus_root, safety_config and control_hz; accept unused options deliberately. Put vendor settings in hardware-config JSON rather than adding vendor fields to the runtime.

This **runnable simulated plugin** demonstrates discovery and the common interface; it is not a hardware driver. Save it as importable my_driver.py:

```python
from teleop_v2.drivers import SimulationDriver


def create(profile, **options):
    # Demonstration returns simulation only.
    return SimulationDriver(profile)
```

```powershell
kineweave serve --robot arm7 --driver my_driver:create --open
```

A real plugin returns its SDK driver instead. The framework checks methods, profile agreement and hardware/writable types. Writable hardware additionally requires execute=True and site-profile metadata. These checks do not establish third-party code safety; load explicitly chosen, reviewed modules.

Drivers can also be injected directly without CLI:

```python
from teleop_v2.drivers import SimulationDriver
from teleop_v2.profiles import load_profile
from teleop_v2.runtime import TeleopRuntime

profile = load_profile("arm7")
driver = SimulationDriver(profile)
runtime = TeleopRuntime(profile, driver, recording_root="recordings")
runtime.start()
try:
    print(runtime.snapshot()["profile"]["id"])
finally:
    runtime.close()
```

## 4. Existing G2 connector

G2 imports the external Nexus gdk_adapter. Nexus-root may name that project's root or its thor_project directory. The selected installation must provide the compatible G2AAdapter API, its dependencies and GDK on the target platform. Read-only is the default. Site JSON paths below are placeholders, not accepted calibration files.

```powershell
kineweave serve --robot config/g2-site.json --driver g2 --nexus-root /opt/nexus --safety-config config/g2-safety.json --hardware-config config/g2-hardware.json
```

Add --g2-cameras for native G2 cameras. Background G2CameraPump reads/encodes separately from joint feedback. Duplicate vendor frames are skipped; clock reversal or a missing vendor-to-host bridge fails explicitly. Camera timestamp events retain source time, offset, host receive time and frame properties. Generic OpenCV USB/IP cameras use --camera NAME=DEVICE and do not require G2.

Execution additionally requires --execute, a hardware_validated profile with simulation_only=false, a valid safety configuration and hardware_config.site_verified=true. Hardware configuration can declare effector_profile and a bijective map from all 24 canonical names to profile names; defaults use identical names. Dahuan uses physical measured coordinates: zero closed and 0.025 m open. VLA model-action inversion is not applied. The existing Nexus safety, rate and collision path is retained.

## 5. Existing G1/Dex3 connector

G1 uses external unitree_sdk2_python and CycloneDDS. It subscribes to rt/lowstate and, for hand layouts, left/right Dex3 state topics. Read-only connections create no command publishers. Execution uses rt/arm_sdk plus explicitly enabled Dex3 command topics; it creates no leg lowcmd publisher, locomotion switching or independent balance controller.

```bash
kineweave serve --robot config/g1-site.json --driver g1 --interface eth0 --hardware-config config/g1-hardware.json
```

Replace interface and paths. Execution also needs site_verified, firmware_watchdog_verified, a reviewed complete 29-motor body map, explicit mode_pr 0/1, controlled_joints and positive finite kp/kd gains for published waist/upper-body coordinates and enabled hands. The 43-coordinate layout needs both seven-motor hand maps; left/right index/middle orders differ and must not be guessed by sorting. Default controlled_joints includes arms only; explicitly include hand joints to command fingers. Legs and waist cannot be active targets.

Hold retains arm SDK ownership at measured targets; close attempts to ramp ownership weight down. With stale feedback, software cannot establish an effective hold, hence the robot-side watchdog requirement. Fake SDK tests check code paths; target-device acceptance is still required.

## 6. New inputs and data adapters

New controllers, VR runtimes, gloves and trackers submit InputFrame. Preserve raw data; tracking loss should disable or invalidate packets rather than send an apparently valid zero pose. Continue increasing heartbeat sequence numbers. Transform coordinates once and declare frame_id. Client send rate does not replace server expiry checks.

Dataset adapters read episodes and preserve profiles, full samples and provenance. Interpret requested targets, dispatch evidence, measured feedback, ownership and reset events separately. Requested targets are not automatically executed trajectories. Keep downstream transformations outside the control loop.
