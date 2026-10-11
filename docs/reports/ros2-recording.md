# PR 14 local acquisition and conversion qualification

Date: 2026-10-10. Base: merged `5e2116a`.
Branch: `feature/pr14-ros-mcap-equivalence`.

## Evidence

- Actual generated CDR, two-process DDS, rosbag2 MCAP storage and canonical import
  ran in the isolated Jazzy/Fast DDS Compose image.
- Complete seed-zero expert on the articulated gripper/wrist preset: **success**,
  **569 synchronized samples**, **11.36 seconds of domain duration**, three original
  RGB cameras. Quality valid and training eligible. One disabled final hold appears
  as the expected rejected-command warning; it is retained in both recordings.
- Local and ROS-imported complete decoded episodes match exactly: all state,
  requested/executed intent, safety fields, source timestamps, pixels, provenance
  and labels. Deterministic resampling also matches.
- Actual LeRobot conversion in the ROS-free CPU image produces **285 frames** at
  25 Hz for both versions. Public validation/manifest/conversion gates pass;
  both logical fingerprints are
  `526a4cd7c7353a754826bd855157107e9bc1e54ce0cbba9e31497461f99bfab0`.
  Dataset lineage verifies and no ROS modules load.
- Bounded real CRISP capture runs against the independent physics process and
  scratch gateway. Its recorded failure outcome remains excluded from training.
  A disabled final hold is part of acquisition; this is not CRISP task-success
  qualification.
- Required ROS cases cover generated serialization, actual acquisition, contiguous
  sequences, replay, wrong episodes, backward source time, missing terminal event,
  producer loss, recorder loss and new-UUID reset/discard segmentation.
- MCAP CRC/schema inspection runs before import; source hashes embed in canonical
  metadata and an import report. Failures retain bag and staged import evidence.

Artifacts remain under ignored `runs/pr14-*`: complete expert capture/reference/
import, conversion reports and datasets, ROS cases and check logs. CI independently
reproduces capture and uploads evidence; a separate ROS-free conversion job consumes
that artifact through the same public manifest/convert CLI.

## Required checks

- `docker compose build dev`: passed.
- Ruff and mypy: passed.
- Dev pytest: **329 passed, 4 expected optional skips** (LeRobot/X11).
- Doctor: supported Python 3.12, status ok.
- Required ROS recording suite: **12 passed**, no skips.
- Existing ROS contract suite: **3 passed**, no skips.
- Existing nonrecording moving-controller/fault regression: **10 passed**, no skips.
- CPU dataset/training/ACT checkpoint and resume regression: **10 passed**, no skips.
- Actual CPU local/ROS LeRobot conversion and fingerprint comparison: passed.

## Limits and follow-ups

Capture/import at full RGB resolution is offline stepped simulation and can take
minutes on the CPU renderer. It does not establish sustained 25/50 FPS acquisition,
GPU recording performance or hardware real-time behavior. Existing viewer performance
qualification is separate. No physical robot, gripper or camera was used.

The replay layout uses recorded original Image/CameraInfo projections, reports and
metadata. Generated CDR/schema/file validation ran; **interactive Foxglove playback
was not validated in this PR**. The earlier Foxglove flange appearance review remains
explicitly deferred. No robot model or replay-to-command control is provided in the
recording layout. Continuous paced acquisition, physical timestamp synchronization,
crash-prefix repair and full CRISP pick/place qualification remain follow-ups.

## CI reset-handoff follow-up

The first remote run failed the existing observer reset check while the recording,
container, learning and controller checks progressed. CI retained four handoff
drops. Inspection found that an idle stepped reset offered its new-episode
snapshot only once: if the observer held the shared lock, the optional offer was
lost permanently until another capture.

The fix retains one bounded encoded pending snapshot and retries its nonblocking
offer during idle. Retries preserve capture identity, timestamp and freshness;
new captures supersede old pending snapshots, oversized packets are dropped, and
observer errors cannot affect physics ownership. The required DDS reset test now
forces lock contention during reset. ROS-free regressions cover delivery, latest
replacement, oversized capture and the unchanged exact 100 ms staleness boundary.

Follow-up validation passed: Compose dev build, Ruff, mypy (71 source files),
doctor, and default pytest (333 passed, four optional-environment skips). The
required observer DDS suite passed all six tests with forced reset contention;
the two targeted owner watchdog tests also passed. Remote CI must pass on the
updated branch before merge.

## CI acquisition-gap follow-up

The observer repair passed remote CI. The recording job then exposed an offline
handoff problem: four samples spanned 1.206 seconds; 573 owner ticks were labelled
`gateway_wall_timeout`. RGB capture/ACK waits outlasted the unchanged 100 ms
authorization watchdog. The quality gate correctly rejected `sample_gap`.

ADR 022 makes the acquisition pause explicit: the owner revokes task authorization
atomically after each command interval and before returning the snapshot used for
rendering. Actual hold state and completed execution status are distinct. Fresh
commands recover normally. No quality threshold, timestamp or watchdog is relaxed.
The regression deliberately pauses for 250 ms and checks nominal sample spacing,
hold ownership, retained gripper and valid failed-episode import.

Local follow-up qualification passed: dev image build, Ruff, strict mypy,
doctor, 333 default tests (four expected optional-environment skips), all 13
required ROS recording tests and all 10 moving/controller fault tests. The
combined required ROS run passed 23 tests in 161.34 seconds. Remote qualification
must run on the updated commit; the earlier failed run is retained as evidence.
