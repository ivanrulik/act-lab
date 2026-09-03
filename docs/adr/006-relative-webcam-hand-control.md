# ADR 006: Relative webcam hand control with a gesture clutch

- Status: accepted
- Date: 2026-09-02

## Context

Webcam landmarks are noisy, camera-relative, and produced on a different clock
from deterministic simulation. Camera reads and inference can also block or
fail. Directly treating landmarks as robot commands would create jumps after
tracking loss and could prevent the control loop from reaching its watchdog.

## Decision

OpenCV capture and MediaPipe Hand Landmarker run on a background thread and
publish only the latest dependency-free hand signal. The 50 Hz simulation loop
never waits for a camera frame. It stamps accepted actions with the current
domain observation time; repeated samples retain that timestamp.

Calibration is explicit and captures a stable median palm anchor, scale,
handedness, and robot pose. Screen-right maps to world `-Y`, screen-up to world
`+X`, and increasing apparent palm scale maps to world `-Z`. The depth proxy uses a
log scale ratio for symmetric toward/away motion, plus a wider dead zone and a
stronger depth-specific exponential moving average. Orientation is fixed.
Filtering precedes the existing application safety limits.

Middle, ring, and little fingers must remain extended to clutch motion. The
clutch engages after three frames, releases immediately, and re-anchors hand
and robot poses on every engagement. Thumb/index pinch independently maps the
gripper. Missing, old, malformed, low-confidence, or changed-hand samples emit
disabled intent immediately and then become stale through the existing 100 ms
watchdog.

## Consequences

Users can reposition their hand without moving the robot, and camera stalls do
not stall safety processing. Apparent hand scale is an intentionally simple
depth proxy, so calibration and camera placement matter. Wrist orientation,
multi-hand control, camera recording, and physical robot use remain deferred.

The operator sees both sides of the mapping: the camera preview renders the
clutch anchor, dead zone, input vector, and generated XYZ values, while custom
MuJoCo scene geometry renders the requested Cartesian target and the error
vector from the measured tool pose. Accepted, limited, and rejected targets use
distinct colors so lack of motion is attributable to input, filtering, or the
safety layer.

MediaPipe and OpenCV become pinned runtime dependencies. The licensed task
model is checksum-pinned into the container rather than committed to Git.
