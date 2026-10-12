# ROS acquisition and conversion contract

See ADR 021. Compose profile `ros2-recording` is optional. It uses the pinned
Jazzy/Noble/Fast DDS runtime and localhost discovery. No host network, physical
devices, display, GPU or privileged permissions are requested.

## Acquisition

`/act_lab/recording/v1/packet` is `act_lab_interfaces/RecordingPacket`, version 1.
Reliable/volatile/keep-last 10; no expiration silently deletes acquisition evidence.
`/act_lab/recording/v1/ack` is `std_msgs/String` containing episode ID and sequence,
with the same QoS. Identical current episode/sequence transport retries are
acknowledged without writing again; all decoded fields and original pixels must
match. CDR padding is excluded from this comparison. Conflicting retries, older
replay and gaps fail. The canonical importer still rejects every duplicate.
Retries never refresh source timestamps or extend the paced 100 ms lease.
Discovery and ACK waits are bounded at ten steady seconds.
Exactly one recorder must match. One producer owns the session and sequence.
ACKs indicate storage-writer acceptance; finalization establishes file durability.

Kinds are start, sample, task, diagnostics, stop and discard. Start carries the
exact `EpisodeProvenance` dataclass JSON schema. Diagnostics preserve the existing
host monotonic, values and calibration fields. Command report sequences retain producer identity independently of acquisition packet
sequences (CRISP can poll the same command repeatedly). Packet sequences start at one and
increase without gaps across episodes. Each attempt starts with a new canonical
UUID; every packet must match the active episode. Terminal labels require a
reason. Task labels remain privileged metadata, never policy input.

A sample carries the complete command report (including optional executed intent,
original command timestamp and all metrics), observation camera keys and all
original tightly packed rgb8 images. Image headers, measured-state timestamp and
sample header agree exactly. No nearest-topic joins, frame interpolation, JPEG,
preview quality settings or missing-camera substitution enter acquisition.
NaN requested intent remains inspectable in its safety report; application quality
rules determine eligibility. Gripper state and executed holds remain intact.

Source ROS timestamps equal domain simulation timestamps plus one second. Bag
receipt time remains monotonic across resets using a steady clock anchored to a
wall epoch. Import ignores receipt time for sample reconstruction. Pauses generate
no synthetic repeated samples; backward source time inside an attempt is invalid.
Finish the recorded attempt before resetting the stepped CRISP runtime. Begin a
new recorded attempt afterward. Paced CRISP acquisition is opt-in under ADR 024.

During stepped CRISP recording, the owner revokes task authorization into its
existing bounded hold immediately after each commanded interval, before rendering
and ACK waits. Snapshots retain the actual hold mode and separately report the
completed interval's mode/reason. Fresh enabled intent requires guard recovery;
the accepted gripper aperture is retained. Offline pauses add no synthetic ticks
or samples. Independent watchdog faults still preserve actual elapsed simulation
time and may make a recording invalid for training. See ADR 022.

## Storage and import

### Optional paced capture

`--controller crisp --paced` keeps the owner integrating physics while a separate
worker renders immutable post-command snapshots and waits for recorder ACKs.
All camera images, measured state and command report retain one original source
timestamp. The worker never steps physics or refreshes command intent.

The control side offers without waiting, with four pending samples maximum.
At 100 ms of pending receipt/render/ACK age, on queue overload, lost worker or
out-of-order completion, acquisition interrupts through the existing disabled
gripper-preserving hold and owner cleanup. No snapshot is overwritten. The
independent source/receipt/output watchdog remains authoritative if the producer
or gateway disappears. Startup/warmup and final drain occur under hold. Active
reset remains rejected; a new attempt creates a new worker and episode UUID,
while acquisition packet ordering continues across attempts.

Paced capture's duplicate canonical reference has one bounded writer thread,
with at most four pending immutable samples and sixteen total operations.
The comparison writer flushes each operation and requires `fsync` on finalization
or orderly interruption; its crash prefix has no durability guarantee.
Its file stages in a dedicated 1 GiB tmpfs supplied by the recording Compose
profile, with no disk fallback. Complete files and interrupted prefixes copy
and sync to disk only at the disabled lifecycle boundary; completed copies
publish atomically. Capacity exhaustion, copy errors and incomplete drain fail
acquisition/finalization. Successful copy removes the staged attempt.
The unchanged capture
lease measures receipt, rendering and the authoritative DDS/rosbag ACK; reference
durability timings are reported separately. Reference backpressure/write failure
interrupts acquisition through the existing hold path. Terminal drain completes
before a successful recording/equivalence result; startup waits are bounded
at two seconds and terminal drain at ten seconds while disabled. These waits
do not extend the 100 ms active capture lease. Ordinary local/stepped sinks retain per-operation
`fsync` and synchronous behavior. Authoritative rosbag MCAP storage is unchanged.

In the recording profile, active physics/expert/capture/recorder diagnostic
traces also stage in the size-limited tmpfs, with bounded file buffers.
Expert traces persist after recording stops or shutdown; physics traces persist
after owner shutdown. Capture and recorder progress retain ordered JSONL and a
final snapshot. These files grant no authorization or durability ACK. Abrupt
process death may lose their staged prefix; the authoritative bag remains the
raw acquisition record. Diagnostic logs cannot grant or renew motion authority.

Full rendering remains default. `--capture-profile reduced-effects` explicitly
disables shadows/reflections and multisampling; those settings enter each camera
fingerprint. Full and reduced effects are incompatible acquisition identities.
The optional `ros2-recording-gpu` service verifies the requested NVIDIA renderer;
it cannot silently fall back. Capture failure is retained as partial evidence,
not converted into a successful recording or a promised real-time rate.

The recorder writes `bag.partial`, closing, syncing and renaming it to `bag` only
on orderly completion. MCAP uses Zstd, chunk and data CRCs. rosbag2's buffered
writer may lose unacknowledged or acknowledged tail records on abrupt termination;
no per-message fsync guarantee is made. Preserve the partial bag. Manual rosbag2
repair can be investigated later; the importer rejects partial directories.

`recording-import` verifies every MCAP CRC and generated CDR schema, then validates
the ordered acquisition stream. It preserves episode IDs, domain timestamps,
provenance, original pixels and every command report. Source-file hashes are
embedded in canonical MCAP metadata and `import.json`. A malformed or unfinished
stream fails, keeps source evidence and leaves `OUTPUT.partial/failure.json`.
The output directory publishes atomically after transport/lifecycle validation.

Import is not a training decision. Existing quality reports and selection manifests
exclude failed, discarded, interrupted, invalid and contradictory task outcomes.
Use `act-lab recording manifest` and `recording convert` in the data/training image.
Imports never overwrite existing output. Complete failed/discarded episodes remain
inspectable. Local/ROS equality is defined over complete decoded episodes, including
pixels and metadata, not MCAP container bytes; import adds source lineage metadata.

The ROS-free conversion image sets `LOGNAME=actlab` as its logical service
identity. CI runs it with the runner's numeric UID/GID for artifact ownership;
that UID need not have a passwd entry. This permits dependencies using Python
`getpass.getuser()` to initialize without changing runtime privileges.

## ROS safety review

| Case | Acquisition behavior | Motion ownership / acceptance |
|---|---|---|
| Startup / absent recorder | Bounded discovery failure; no silent recording | Local failure propagates; CRISP independent owner remains authoritative |
| Missing dependencies | Explicit Compose hint or required CI failure | No fallback transport |
| Producer disappearance | No terminal packet; import fails; preserve bag | Existing independent watchdog applies dynamic hold |
| Recorder loss / missing ACK | Bounded failure, partial evidence retained | Shared recording error path holds, then session cleanup stops owner |
| Pause / repeated state | No fabricated fresh timestamp; duplicate sample rejected | Command/steady clock freshness remains the PR 11/13 safety contract |
| Backward reset | Active recording rejects reset; finish then new UUID | Owner and inbox reset together; fresh command required |
| Forward jump | Preserve source age/gaps; quality policy may reject | Existing command freshness rules remain authoritative |
| Replay / gap / wrong episode | Import fails, staged evidence retained | Recorder cannot authorize commands |
| Invalid frame / numeric intent | Full application safety report preserved | Shared safety validation rejects enabled motion |
| Stop / discard | Explicit labelled lifecycle, durable publication | CRISP records a final disabled hold before stopping acquisition |
| Shutdown / exception | Unfinished stream remains excluded | Existing hold/owner shutdown path executes |
| Bag playback | Read-only historical data | No replay-to-command adapter is provided |

## Dependencies

- Ubuntu `ros-jazzy-rosbag2-py` and `ros-jazzy-rosbag2-storage-mcap`: actual ROS
  CDR/MCAP storage and reading, isolated from the default image.
- Existing canonical `mcap==1.3.0` / `protobuf==5.29.4` with `lz4==4.4.5` /
  `zstandard==0.25.0`: canonical import and exact reference recording. No new
  default project dependency. ROS-compatible apt numerical packages are retained.
- Existing optional simulator/controller/viewer base: original RGB and scratch
  snapshots. Learning frameworks remain in the separate CPU training image.

## Complete-task candidate

`--full-task` requires paced CRISP. Owner task state is coherent with the measured
observation; the existing scripted expert remains the only phase machine. Source
shaping, resolved configuration and controller hash are recorded. Task success
alone is insufficient for qualification: actual release requires measured aperture
at least 0.95 and zero finger/cube contacts. Timeout/failure remains authoritative.
The local and CRISP qualification paths apply this same completion rule.

Per-sample render, queue and transport timings remain ignored raw evidence.
The independent owner records cube position/contact alongside physics limits so
loaded faults can be checked after gateway death. Qualification never changes
motion limits, effort gating, freshness, image identity or quality selection.
See ADR 025 for explicit source/transport choices and measured acceptance limits.

## Deferred limits

Long-duration paced throughput qualification, camera hardware synchronization, rosbag crash repair,
multiple producers/recorders, physical mounting, depth/stereo and hardware use
remain deferred. The Foxglove flange assembly appearance follow-up remains open.
Stationary/short CRISP recording does not establish successful pick/place or
hardware real-time performance. Interactive Foxglove playback acceptance must be
reported separately from CDR/schema and file validation.
