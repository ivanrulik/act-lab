# ADR 010: Versioned durable episode recording

- Status: accepted
- Date: 2026-09-13

## Context

Acquisition must preserve intent, safety decisions, measured results, images,
and failed attempts without coupling domain types to MCAP or ROS.

## Decision

Use the `act_lab.recording.v1` Protobuf package and MCAP profile. Compile an
embedded FileDescriptorSet during builds; do not maintain generated Python
source. Pin `grpcio-tools` as the build-only compiler, `protobuf` for encoding,
and `mcap` for acquisition. Constrain MCAP compression dependencies. Git in the
container supplies revision and worktree status for mounted source checkouts.

Application RecordingRobot extends the existing safety controller and sends
post-command observations, reports and scene images through EpisodeSink. No
privileged task geometry or webcam images are recorded. Task result events and
webcam diagnostic/calibration snapshots are separate from policy observations.

One reset starts an attempt; normal session exit stops it. Explicit CLI outcome
and reason options support operator labels and discard, retaining all data.
Viewer F6/F7/F8 queue success/failure/discard on the control thread and show
finalization status. Each expert seed gets its own episode. Restart interactive
commands for another attempt; multiple attempts in one viewer are deferred.
F9 is avoided because the operator desktop consumes it for global dictation.

MCAP timestamps use simulation time. Observation, command result and scene
images share a post-step timestamp; requests retain original timestamps,
including stale values. Host monotonic time and wall start time are separate
metadata, not substitutes for simulation time.

Write CRC-protected independently flushed compressed chunks to an exclusively
created `.mcap.partial`. Fsync each sample. Finish writes the footer and fsyncs,
then publishes a same-directory hard link without overwrite, removes the partial
name and syncs the directory after both steps. This gives atomic visibility on
supported local Linux filesystems. A crash between linking and unlinking can
leave both names for the same complete file.

Recovery locks the source inode and copies its CRC-checked complete prefix to
a separate `.recovered.mcap`, preserving the original. A recovery event marks
it interrupted even if a stop event survived; metadata records source filename
and SHA-256. Incomplete trailing records may be lost. Corruption is an error,
not silently skipped. A recovered final cycle can contain only some channels
and must undergo PR 7 quality validation.

## Consequences

Files are self-describing; failed/discarded/interrupted attempts stay traceable.
Synchronous rendering and durable writes can reduce real-time factor; simulated
physics does not advance while blocked. Recording errors apply the existing
disabled hold and exit acquisition. This is not a hardware real-time recorder.
Interactive recording performance needs separate operator validation. Replay,
quality validation and conversion remain PR 7; training remains PR 8.
