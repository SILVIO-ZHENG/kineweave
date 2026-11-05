# Recording format

KineWeave records a versioned episode directory. The recorder depends on a robot profile, NumPy and HDF5; SDKs and input devices stay outside the storage layer. External tools can read episodes without connecting to a robot.

```text
episode-<UTC>-<unique-id>/
  INCOMPLETE          # Present until successful sealing
  manifest.json       # Status, counts, sizes and SHA256 hashes
  profile.json        # Complete robot profile and joint order
  metadata.json       # Task and clock declarations
  samples.h5          # Numeric sample fields
  samples.jsonl       # Complete samples
  events.jsonl        # Input packets and operator/runtime events
  cameras.jsonl       # Per-frame paths, timestamps and hashes
  cameras/<name>/<frame-index>.jpg
```

The schema is `teleop.episode/v1`. Coordinate count N comes from `profile.joints`; there is no fixed 24- or 43-coordinate recording format. Vectors follow profile joint order. Revolute coordinates use radians, prismatic/effector coordinates may use metres, Cartesian positions use metres and quaternions use XYZW.

## Samples and command evidence

`append(sample)` requires positive integer Unix-UTC `wall_time_ns`, positive process-local `monotonic_ns`, a strictly increasing integer `sequence` and four finite N-coordinate arrays:

| Field | Meaning |
|---|---|
| `state` | Measured coordinate positions |
| `velocity` | Measured or explicitly identified estimated velocity |
| `requested` | Desired robot target before generic motion constraints |
| `command` | Driver-adopted target when reported; otherwise the attempted target |

`receipt` establishes dispatch evidence. An accepted DDS publication is not proof of physical arrival. Failed dispatches retain error/rejection evidence. The generic guard output stays in `guard.output` when the driver further shapes a command.

Complete input, mode, receipt, IK, timing, feedback metadata and extension fields remain in JSONL. Reading preserves fields it does not interpret. HDF5 uses sample count as the first axis and N as the second axis for robot vectors. Clocks and sequence use int64; robot values use float64. The `joint_names_json` and `joint_units_json` attributes declare order and units. HDF5 and JSONL sample counts must match.

## Events and clocks

`event(kind, details)` records a UUID, receive UTC/monotonic clocks and details. During recording, every admitted packet is retained as an `input_frame` event, while the control loop consumes its latest snapshot. Control avoids a backlog and higher-frequency input stays traceable.

Retain device-source timestamps separately. Process-local monotonic clocks cannot be joined across processes. Preserve source clocks, mappings and estimated alignment error in metadata/events. The runtime records input and feedback receive times alongside sample times. These distinguish capture, transport, processing and feedback age; they do not establish hardware exposure synchronization.

## Cameras

`camera(name, jpeg, timestamp_ns)` accepts encoded JPEG bytes. Each frame is independent. Its index contains camera name, increasing frame index, source timestamp, recorder receive monotonic time, relative path, byte count and SHA256.

Declare the source clock in metadata. No automatic synchronization or silent frame interpolation is applied. `OpenCVCamera` opens a selected device only after explicit `start()`; capture, encoding and callbacks run in a worker. Its timestamp is host Unix UTC immediately after frame read, not hardware exposure time. Check `error` or call `check()` for failures. Stop cameras before closing the recorder.

The G2 camera worker preserves native timestamps and vendor-to-host mappings in `camera_timestamp` events. Missing mappings and backward clocks fail explicitly.

Optional video export creates a constant-rate MP4 preview and an original-timestamp sidecar. Original JPEGs and the index remain the source evidence.

## Integrity and failures

Recording starts with `INCOMPLETE` and a `complete=false` manifest. Append/event/camera calls validate, copy and enqueue through a bounded asynchronous queue rather than waiting for disk.

Overflow raises `RecordingQueueFull` and permanently marks the episode incomplete. Accepted queued data is drained where possible. End that capture and address the underlying storage or throughput problem.

`close()` waits for the writer, hashes content files, atomically replaces the manifest, then removes `INCOMPLETE` only after successful sealing. File creation, draining and hashing run outside the control lock. Application crashes and interrupted sealing remain identifiable; this is not a claim of power-loss or hardware-failure durability.

`mark_incomplete(reason)` registers a camera/device failure without waiting for disk. An exception within the recorder context preserves incomplete status.

## Reading and exporting

`inspect_episode(path, verify=True)` returns completeness, verification, errors, counts and manifest. **Complete and verified are separate properties:** incomplete but drained data may have consistent hashes. SHA256 establishes content consistency, not identity authentication; manifests have no external signature.

`read_episode` verifies before returning profile, metadata, full samples, events, cameras and manifest. Integrity failures raise explicitly. CSV export expands numeric joint fields and retains other fields as JSON columns. Keep exports outside sealed episodes because extra files invalidate their inventory.

## Dataset adapters

Independent dataset adapters can consume episodes for analysis or downstream work. Preserve coordinate semantics, configuration identity, command evidence, measured feedback, source clocks, reset events and missing-data flags. A requested target is not an executed trajectory without delivery evidence and subsequent feedback.
