# Input and HTTP protocol

Third-party devices can submit InputFrame through HTTP or call runtime.submit within Python. Packets describe current intent, not a queued trajectory. Only the latest packet is retained; the control thread decides whether a tick can send.

## 1. Address, credentials and requests

The default address is http://127.0.0.1:8765. Startup creates a token and prints a URL containing #token=TOKEN. The browser reads the fragment and adds X-Teleop-Token to API requests; the fragment is not part of the server path. Custom clients may alternatively send Authorization: Bearer TOKEN. Every API GET/POST requires authentication, including reads.

POST bodies use application/json and must be JSON objects of 1..128000 bytes. If an Origin header is present, it must exactly match the server scheme and Host; cross-site browser requests return 403. Scripts normally omit Origin and authenticate with the token. Invalid tokens return 401; input/state errors normally return 400 with an error field. There is no multi-user login or open-CORS mode.

| Method | Path | Content |
|---|---|---|
| GET | `/api/state` | Profile, mode, owner, feedback, input age, events and recording |
| POST | `/api/input` | One InputFrame; successful submission returns accepted=true |
| POST | `/api/action` | Action and options |
| GET | `/api/episodes` | Up to 200 episode names under configured recording root |
| GET | `/api/episode?id=NAME` | Verified display-playback frames, no motion execution |
| GET | `/api/camera/NAME` | Latest JPEG, authenticated by header |
| GET | `/api/episode-camera?id=NAME&camera=CAM&frame=N` | Indexed recorded JPEG with frame-hash validation |

## 2. InputFrame fields

This shows the complete structure. HTTP sources need source_id, sequence, current sent_wall_ns and their chosen intent fields. The illustrative zero timestamp is allowed only for in-process Python submission; HTTP requires current Unix nanoseconds. Send direct-joint and overlapping pose/twist examples separately so one coordinate has one semantic owner.

```json
{
  "source_id": "tracker-session-unique-id",
  "sequence": 1,
  "enabled": false,
  "clutch": false,
  "frame_id": "robot_base",
  "poses": {},
  "twists": {},
  "joints": {},
  "grasp": {},
  "valid": true,
  "confidence": 1.0,
  "sent_wall_ns": 0,
  "raw": {"device": "my-tracker"}
}
```

| Field | Rule |
|---|---|
| source_id | Nonempty, ≤128 characters; use a sender-session UUID |
| sequence | Nonnegative integer, strictly increasing per source for the server session |
| enabled | Defaults false; true still requires Arm and other checks |
| clutch | True suspends motion and discards relative anchors |
| frame_id | Robot-base or WebXR coordinate convention |
| poses | `{task: {position:[x,y,z], quaternion:[x,y,z,w]}}`; metres and approximately unit XYZW quaternion |
| twists | `{task:[vx,vy,vz,wx,wy,wz]}`; robot-base m/s and rad/s |
| joints | `{joint_name:absolute_position}` in the coordinate's declared units |
| grasp | `{effector:amount}` in 0..1, open to closed |
| valid | Defaults true; false on tracking loss |
| confidence | Finite 0..1, at least 0.5 to activate |
| sent_wall_ns | HTTP requires positive integer Unix UTC ns, no older than timeout and no more than 100 ms ahead |
| raw | Retained device data, not directly interpreted as control |

Pose/twist tasks must exist in profile.chains, grasp names in profile.effectors, and joints must match exact profile names. A task cannot have pose and twist simultaneously. Direct joints cannot overlap active IK-chain joints, and grasp cannot overlap direct targets for the same hand coordinates. Numeric values must be finite.

WebXR basis conversion applies to tracked poses; twists retain robot-base semantics. Robot-base axes are forward/left/up; WebXR uses right/up/negative-Z forward. Tracked poses are absolute in their source space, but relative anchoring prevents the first enabled packet from jumping to device coordinates. Linear/angular twist norms are bounded at 0.25 m/s and 0.8 rad/s before scale and joint guards. These are input constraints, not validated hardware safety values.

## 3. Sources, sequence and expiry

The server does not sort or catch up old packets. Duplicate/out-of-order sequence numbers are rejected per source. Pause, Reset and re-anchor do not reset seen sequences. On sender restart, choose a new source ID or continue sequence numbering. A server session tracks at most 64 source IDs.

While disarmed, different sources may submit and the latest can be selected. Once armed, the owner is exclusive and other sources are rejected. Change sources by disarming, submitting a fresh packet and arming the new source. accepted=true acknowledges packet submission, not execution or physical target attainment.

Freshness uses the **server's monotonic receive clock**, default 0.3 seconds. Continue sending increasing sequences even without motion; replaying old packets cannot refresh input. HTTP requires positive sent_wall_ns and checks transport TTL: sender time cannot be older than timeout or over 100 ms ahead. Synchronize remote clocks. Only in-process runtime.submit calls may use zero to omit sender-clock checks; receive expiry still applies. RobotState uses local feedback monotonic time; drivers must not substitute a device epoch clock.

During recording, every admitted packet is retained as an input_frame event. Control samples contain the latest snapshot used for that tick. Control stays latest-only while higher-frequency input remains traceable.

Release, clutch, invalid tracking, confidence below 0.5 or expiry holds motion and clears anchors while retaining owner. New valid packets from that owner may resume, anchored at then-current feedback. A tick over 0.1 seconds discards latest and requires another packet. Disarm/Pause and software stop clear latest; old input cannot run after re-arming.

## 4. Action requests

| action | Options | Behavior |
|---|---|---|
| arm | `source_id` | Requires matching fresh input, writable driver and no fault/estop |
| disarm / pause | None | Hold and clear owner/latest |
| calibrate | None | Hold, clear anchors/latest, retain selected owner |
| estop | None | Latch software stop and clear owner/latest |
| reset | None | From fault/estop/disarmed, read valid feedback and return disarmed |
| record_start | Optional task | Start an episode |
| record_stop | None | Drain and seal the episode |

Recording does not arm control. Reset resets software state rather than homing the robot. Software stop is not a physical emergency-stop interface.

## 5. curl example

Start the README's simulated server and copy its token. This Windows PowerShell example submits a disabled packet without motion intent, then selects its source. Increase sequence or use a new source ID when repeating.

```powershell
$token = 'REPLACE_WITH_PRINTED_TOKEN'
$packet = @{
    source_id = 'curl-session-1'
    sequence = 1
    enabled = $false
    sent_wall_ns = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds() * 1000000
    raw = @{ device = 'curl' }
} | ConvertTo-Json -Compress
$packet | Set-Content -Encoding utf8 input.json
curl.exe --fail-with-body http://127.0.0.1:8765/api/input -H "X-Teleop-Token: $token" -H "Content-Type: application/json" --data-binary '@input.json'
'{"action":"arm","source_id":"curl-session-1"}' | Set-Content -Encoding utf8 action.json
curl.exe --fail-with-body http://127.0.0.1:8765/api/action -H "X-Teleop-Token: $token" -H "Content-Type: application/json" --data-binary '@action.json'
curl.exe --fail-with-body http://127.0.0.1:8765/api/state -H "X-Teleop-Token: $token"
'{"action":"disarm"}' | Set-Content -Encoding utf8 action.json
curl.exe --fail-with-body http://127.0.0.1:8765/api/action -H "X-Teleop-Token: $token" -H "Content-Type: application/json" --data-binary '@action.json'
```

Arm requires the packet to remain within timeout. Slow manual execution may return stale; submit a higher sequence and arm immediately. Use a heartbeat program for continuous motion rather than manually issued curl requests.

## 6. Third-party Python source

This uses only the standard library and requires hardware=false before sending intent. It selects a joint from the default control groups, requests a small simulated change and maintains heartbeat. Set TELEOP_TOKEN first and optionally TELEOP_URL. Pause any existing browser owner before running.

```powershell
$env:TELEOP_TOKEN = 'REPLACE_WITH_PRINTED_TOKEN'
python my_input_source.py
```

Save this as my_input_source.py:

```python
import json
import os
import time
from urllib.request import Request, urlopen
from uuid import uuid4

base = os.environ.get("TELEOP_URL", "http://127.0.0.1:8765").rstrip("/")
token = os.environ["TELEOP_TOKEN"]


def request(path, body=None):
    payload = None if body is None else json.dumps(body, allow_nan=False).encode("utf-8")
    req = Request(base + path, data=payload,
                  headers={"X-Teleop-Token": token, "Content-Type": "application/json"})
    with urlopen(req, timeout=2) as response:
        return json.load(response)


state = request("/api/state")
if state["hardware"]:
    raise RuntimeError("This example is simulation-only")
if state["owner"]:
    raise RuntimeError("Pause the existing input source first")

profile = state["profile"]
groups = profile["metadata"].get("default_control_groups")
joint = next(j for j in profile["joints"] if not groups or j["group"] in groups)
index = next(i for i, j in enumerate(profile["joints"]) if j["name"] == joint["name"])
target = min(joint["upper"], max(joint["lower"], state["state"][index] + 0.05))
source = "python-" + uuid4().hex
sequence = 0


def submit(enabled):
    global sequence
    sequence += 1
    request("/api/input", {
        "source_id": source, "sequence": sequence,
        "enabled": enabled, "valid": True, "confidence": 1.0,
        "frame_id": "robot_base", "sent_wall_ns": time.time_ns(),
        "joints": {joint["name"]: target},
        "raw": {"device": "example-python-source"},
    })


submit(False)
request("/api/action", {"action": "arm", "source_id": source})
try:
    for _ in range(40):
        submit(True)
        time.sleep(0.05)
finally:
    try:
        submit(False)
    finally:
        request("/api/action", {"action": "disarm"})
```

Real trackers can replace joints with poses or twists and retain source packets in raw. On send failure, stop replaying old targets and reacquire state/calibration after reconnecting. Retrying the same sequence is not a new heartbeat.
