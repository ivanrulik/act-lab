# ADR 021: Acknowledged ROS acquisition and canonical import

- Status: accepted
- Date: 2026-10-10
- Stepped command/acquisition handoff refined by ADR 022.

## Context

The viewer drops old frames, publishes JPEG previews, and reports state and
commands on independent topics. Those streams cannot establish synchronized
policy observations or complete attempt lifecycle. Training must remain independent
of ROS and use the existing ADR 010/011 quality and selection gates.

## Decision

Add generated `RecordedSample` and `RecordingPacket` interfaces to the optional
Jazzy image. One reliable, volatile, keep-last-10 acquisition topic carries a
contiguous sequence across attempts. Each post-command sample carries the complete
domain report, measured state and every declared original rgb8 camera. Metadata
packets carry versioned provenance, task labels, diagnostics and explicit terminal
outcomes. One producer and one recorder are supported per recording session.
Existing v1 command/state/report interfaces are unchanged.

A separate rosbag2 process writes MCAP with Zstd and chunk/data CRCs, and
acknowledges writer acceptance. Receipt times use an anchored wall epoch plus
steady elapsed time; source headers retain simulation time plus one second.
Source timestamps, not arrival time, reconstruct canonical episodes. A new UUID
is required for a reset; packet ordering continues across episodes. Writer close,
file/directory fsync and publication rename finalize the bag. Acceptance ACKs do
not promise per-packet durability. Crash prefixes remain in `bag.partial`.

Use the application `RecordingRobot` on the local driver and on the CRISP
scratch-snapshot gateway in **stepped mode**. Capture follows the completed control
command. All cameras render the same immutable post-command scratch snapshot;
the physics process performs no rendering. Acquisition waits for recorder ACKs;
this is bounded offline simulation, not a paced or hardware recording design.
Recorder failure ends acquisition through the existing safety hold/cleanup path.
The independent CRISP owner retains its watchdog and bounded dynamic hold.

The importer verifies CRCs, contiguous packet sequences, episode ownership,
lifecycle, source clocks, RGB dimensions/encoding and calibration camera coverage.
It retains original episode IDs and produces the existing Protobuf MCAP profile.
Source bag hashes are embedded as MCAP metadata and written to an import report.
Canonical files publish together only after complete transport validation; failures
retain a staged directory. Existing application quality validation may still reject
a structurally imported episode. Failed/discarded attempts remain evidence and
are excluded by the unchanged selection manifest. Interrupted attempts do not
publish as complete episodes.

Standard Image/CameraInfo, state, report and Clock projections support inspection
of the ROS bag in Foxglove. They are derived from acquisition packets and never
used to reconstruct training samples. Offline record time and source simulation
time have different purposes; playback cannot authorize robot motion.

## Consequences

Local and DDS acquisition can be compared exactly before resampling, then through
the same quality/selection and LeRobotDataset APIs. ROS dependencies exist only in
an opt-in recording image. No training dependencies or numerical-stack replacement
are installed there. Continuous paced acquisition, hardware sensor synchronization,
crash-prefix repair tooling and full CRISP task qualification need separate work.
A bounded CRISP recording marked failure is not a successful demonstration.

The [contract and safety matrix](../contracts/ros2-recording.md) define operating
limits. The rosbag2 APIs and storage options are from the installed Jazzy runtime;
see [upstream rosbag2](https://github.com/ros2/rosbag2/tree/jazzy) and
[MCAP storage configuration](https://github.com/ros2/rosbag2/tree/jazzy/rosbag2_storage_mcap).
