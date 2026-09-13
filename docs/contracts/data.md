# Data contract

## Raw episodes

MCAP is the immutable source of truth. Each episode has a stable ID and records
monotonic timestamps, observations, executed/target actions, task events, and
provenance. Wall-clock timestamps may supplement but never order data.

Required provenance includes:

- schema and application versions;
- Git revision and dirty-state indication;
- resolved configuration and random seed;
- control/physics/sensor rates;
- robot, task, camera, and calibration identifiers;
- operator pseudonym when applicable;
- outcome and discard reason;
- container image identity when available.

## Validation

PR 6 implements `act_lab.recording.v1`; see [recording](../recording.md) and
ADR 010 for channels, clock semantics, retention and atomic publication.
Discarded attempts retain their data and reason. Recovered attempts are marked
interrupted, preserve their source and require later quality validation.
Scene images and measured results share the post-step simulation timestamp;
requested intent retains its original timestamp. Webcam diagnostic and
calibration snapshots are recorded separately; webcam pixels are not retained.
Webcam acquisition provenance includes model content hash and live-camera ID or
recorded-video content hash. Incomplete inspection results have no final episode
outcome; task results are reported separately from terminal lifecycle labels.

Validation detects missing streams, non-monotonic time, stale/dropped samples,
NaNs, incompatible dimensions, invalid commands, tracking gaps, and inconsistent
task labels. Rejection does not delete the raw episode.

## Derived datasets

Conversion to LeRobotDataset is deterministic for a given raw manifest,
configuration, and converter revision. Dataset splits occur by episode, never
by frame. The output records its source episode IDs and fingerprint.

## Retention

Raw webcam images may contain faces, rooms, screens, or identifying material.
Default examples should prefer scene/wrist images for policy observations and
document whether webcam frames are retained. Never publish recordings without
review and consent.

PR 5 uses webcam frames only for live diagnostics and does not persist them.
The small regression asset is a cropped, downsampled derivative of an
Apache-2.0 MediaPipe test image; it contains hands only and has provenance next
to the fixture. Raw operator-camera material remains prohibited in Git.
