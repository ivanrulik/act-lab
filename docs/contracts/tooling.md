# Articulated tool and wrist camera contract

Preset: `configs/sim/ur5e_2f85_d405.toml`. Decision: ADR 019.

## Model and acquisition

Six canonical arm joints retain their domain order and SI units. Eight `rq_`
joints are measured simulation linkage coordinates, published separately in
standard JointState. Their velocities/efforts are omitted rather than invented;
arm velocity/effort remain in existing state/telemetry messages. The assembled
URDF and matching MJCF are generated from pinned sources. A rigid 50 mm nominal
aperture URDF supplies CRISP's six arm DOFs. Finger dynamics still run in MuJoCo.

Normalized gripper feedback is actual jaw separation / 85 mm, clipped to [0,1].
A blocked grasp can have a nonzero measured aperture while intent requests zero.
Fault holds retain the last accepted actuator intent. Robot access belongs to
the consuming control thread; ROS/image callbacks do not command the robot.

RGB calibration identifies camera ID, frame, size, K/D, fixed extrinsics and
synthetic sample-grid frequency. Each new-model recording includes model hashes
and all selected camera profiles in resolved provenance. Quality validation
requires matching cameras/shapes/calibration. Dataset conversion rejects mixed
identities. Checkpoints contain `pretrained_model/act_lab_model.json`; missing
new-model metadata or a different model/camera contract fails before inference.
Required image input aged 100 ms or more, future input, missing wrist or wrong
shape produces disabled measured-pose intent through the shared safety path.

## Viewer projection

All three wrist topics use best effort, volatile, keep-last 1:

- `/act_lab/view/wrist/image_raw`: sensor_msgs/Image, RGB8.
- `/act_lab/view/wrist/camera_info`: paired sensor_msgs/CameraInfo.
- `/act_lab/view/wrist/sample`: CameraSample, schema version 1; episode UUID,
  original capture sequence/tick and model/calibration hashes, image and info.

ROS stamp = original domain stamp + 1 s. Never restamp reused pixels. The optical
pose corresponds to the captured scratch state. A separate latest-state slot
carries primitive state only, with no blocking producer wait. A render result
crossing a reset, changing model, or aged at least 100 ms is discarded. A clock
pause cannot renew freshness. Old episodes cannot re-enter after reset.
The independent observer diagnoses renderer failure/staleness even when the
renderer stops. Cached viewer pixels are not evidence of current capture.

## Dedicated simulation safety review

| Scenario | Required behavior | Evidence gate |
|---|---|---|
| Startup / missing capture | Hold until motion contract permits; camera stale | Inbox and camera tests |
| Lost command producer | Independent motion lease expires; accepted grip retained | Loaded motion qualification |
| Slow / dead renderer | Drop or stale image; no motion authority renewal | Separate-process DDS and injected clock tests |
| Clock pause | Exact 100 ms steady expiry; fresh sequence on resume | Camera / inbox unit tests |
| Backward reset | New episode; discard old/in-flight image and intent | Reset and replay tests |
| Malformed / future camera source | Reject source, mark stale | Primitive envelope tests |
| Missing / stale policy wrist | Disabled intent through shared hold | Policy-input contract tests |
| Loaded contact / closure | Existing bounds apply; fault hold if violated | Actual contact and effort tests |
| Shutdown | Stop renderer/observer, close motion with dynamic hold | Bounded runtime shutdown |

No row constitutes hardware qualification. Physical mounting, stop/hold,
independent watchdog, dynamics and camera drivers remain deferred.

## Accelerated and compressed viewing (ADR 020)

`tooling` explicitly uses a low-cost CPU viewer profile. `tooling-gpu` reserves
one NVIDIA GPU for headless EGL and fails when actual OpenGL identity is not
NVIDIA. No automatic backend fallback is supported. Local acquisition/learning
profiles and images remain unchanged.

The compressed topic `/act_lab/view/wrist/image_raw/compressed` is standard
sensor_msgs/CompressedImage, quality-90 JPEG, best effort/volatile/keep-last 1.
Its original capture header matches CameraInfo and any subscribed raw sample.
JPEG is a lossy preview and does not enter recording or ACT input. Raw viewer
Image/CameraSample are sent only to subscribed readers. The preview profile hash
and actual OpenGL identity are recorded separately from acquisition identity.

The independent latest-state channel receives the 50 Hz simulation sample grid.
The renderer owns a 25 Hz steady target and skips superseded states. It never
refreshes paused/reused capture identity, queues old frames, or weakens the
100 ms deadline. Encoding is followed by the same expiry/reset check as rendering.
