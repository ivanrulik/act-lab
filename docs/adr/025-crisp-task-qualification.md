# ADR 025: Explicit CRISP task qualification profile

- Status: accepted for the locally qualified simulation profile
- Date: 2026-10-11
- Extends ADRs 016, 019, 021 and 024 for an opt-in qualification profile.
- Refines ADR 021 transport acceptance for identical current-packet retries
  within the original lease; stored packet order stays strict.

## Context

Short nearby motion and loaded grasp/lift evidence do not establish a full task
or sustained recording. The recording demonstrator initially submitted one fixed
pose. Owner task state and success must be available to the existing privileged
scripted expert without exposing task geometry in policy observations.

Initial complete-task trials exposed capture expiration, slow loaded transfer,
and an acceleration rejection. These failures remain evidence. A 25 Hz source
with a hardcoded ten-tick gateway retained a 50 Hz target horizon despite a
longer acquisition period. A higher-gain candidate alone did not complete the
transfer and is not selected.

## Decision

Use an explicit qualification preset with 500 Hz physics and 25 Hz application
commands. The gateway advances the configured substeps; the default 50 Hz paths
still advance ten ticks. Use the explicit 0.05/s gripper approach,
computed from original source simulation time. The preset has a bounded
240-second episode budget, 0.10 m transit clearance and a stricter 0.06 m/s
command translation ceiling. During transit, bound the source XY goal to a
6 mm measured-relative lookahead while retaining vertical authority under load.
Use 4 mm for descent/close alignment and 2 mm before opening at the support;
approach retains the existing 15 mm general tolerance. Existing presets have equal tolerances and
retain their behavior. The qualification grip target is 0.55 with a 0.04 feedback
tolerance, avoiding the full-close dead zone before opening a 50 mm cube.
Downward source shaping also bounds XY to half its 1.2 mm vertical lookahead and
freezes the measured wrist pose on entering opening. This prevents the source
from continuing to lower the wrist while releasing pad contact. This is source
policy, not a safety-limit relaxation.
Bound downward source lookahead to 1.2 mm during descent/placement; smaller
0.4 mm trials stalled. These source choices are candidate experiments, not a
new controller or weakened application guard.
The transfer crosses the tray rim at world `(0.54, 0.19, 0.575)` m before
approaching the destination at 0.065 m clearance. The lower constant-height
trial clipped the 0.490 m rim; raising the whole transfer instead reached an
infeasible near-straight elbow configuration. These optional waypoints stay
inside the existing expert's TRANSIT phase. Default presets have no waypoints.
The first complete frozen GPU trial rejected seed 10 near a straight elbow
during descent. A 0.8 mm downward-lookahead trial still reached IK failure there;
an inward waypoint introduced only at descent also reached the acceleration
guard. The explicit candidate now moves inward to world X 0.46 m at measured
height during APPROACH_CUBE, lowers there to Z 0.56 m, then returns to the cube
XY at a 0.105 m approach clearance. These measured waypoint transitions remain
inside the existing approach phase. Original timestamps and shared limits apply,
and local and CRISP sources use the same route. Default approach behavior is
retained when the optional clearance/route is absent.
The existing task criterion can succeed while the cube is pinched: complete-task
qualification additionally requires measured aperture at least 0.95 and zero
pad/cube contacts. Missing contact evidence fails closed; actual failures/timeouts
are never masked. Task success, measured-motion ceilings, effort,
independent watchdog and the 100 ms acquisition lease remain unchanged.

Reuse `ScriptedPickPlaceExpert` through a narrow coherent owner task snapshot.
An optional task provider supplies recording labels from this same authority.
Full intent travels DDS; source timestamps are preserved. Recovery records every
hold and requires fresh state/sequence; application safety rejection is not
silently retried. Source geometry stays privileged metadata, not a policy feature.

The optional preset may select a versioned ROS controller parameter file.
Runtime copies the actual file and records its hash/content in provenance.
The selected candidate retains the original tooling gains and filters. This
selection does not change defaults or qualify other controllers.

The acquisition profile uses explicit 32 MiB shared-memory and loopback-only
UDP transports, with 256 MiB container shared memory and asynchronous publication.
The writer's reliability blocking limit is 1 ms; acceptance still requires an
application ACK within the unchanged 100 ms capture lease. A synchronous trial
blocked inside packet publication for 330 ms and interrupted acquisition. The
asynchronous mode moves transport work off the publishing thread; it does not
establish delivery or durability without the existing ACK/finalization checks.
Use `SYSTEM_DEFAULT` discovery to preserve this XML configuration. RMW 8.4.4's
`LOCALHOST` branch disables built-in transports and appends default SHM/UDP
transports; the large-data environment variable therefore did not control those
trials. Explicit loopback locators/whitelist preserve localhost discovery without
host networking or published DDS ports. Bound reliable heartbeat/response delays
at 10/1 ms; application lease checks remain authoritative.

Storage and scratch capture are transport-only nodes: their timestamps come from
immutable source packets and steady receipt time, never `node.get_clock()`.
They do not enable an automatic `/clock` subscription. Motion authority, gateway
and controller still require simulation time; this is not system-clock motion
support. Removing storage clock callbacks did not by itself fix the transport
failures, and is not presented as their proven cause.

Within the unchanged capture lease, retry the same acquisition packet every
10 ms until ACK. The recorder compares every decoded field and original pixel
byte, excluding unspecified CDR padding, and acknowledges identical current
episode/sequence retries without writing another sample. Conflicting duplicates,
older replay and gaps still fail. The importer remains strictly ordered: duplicate
packets must never reach the bag. This narrowly extends ADR 021 replay handling.
It does not retry motion authorization or renew source/receipt timestamps.

The producer's optional report observer uses compatible best-effort telemetry;
its bounded RPC report remains authoritative. Bounded callback draining prevents
observer starvation. Recorded reports remain reliable. RGB arrays convert to
primitive byte buffers without expanding pixels into Python integers; generated
serialization and canonical equivalence tests must prove unchanged contents.

A prior buffered reference trial did not resolve DDS publication delays.
The later frozen cohort exposed 89/133 ms reference `fsync` pauses after the
authoritative DDS ACK, blocking rendering of the next sample. Paced capture now
delegates only its duplicate canonical comparison artifact to one writer thread.
The comparison sink flushes each operation and commits on finalization or orderly
interruption, with one file owner. Ordinary acquisition sinks retain per-operation
`fsync`. This duplicate artifact has no crash-prefix durability guarantee.
The reference queue admits at most four immutable samples and sixteen total
operations. Backpressure or a writer error interrupts acquisition through the
existing error/hold path. Startup waits are bounded at two seconds; terminal
drain is bounded at ten seconds while motion is disabled.
Finalization/equivalence cannot pass before completion.
Ordinary local/stepped canonical sinks retain synchronous behavior.

The revised 20-seed cohort passed 20/20, but its first sustained run interrupted
after four complete attempts: a 210 ms per-operation reference `fsync` filled
the four-sample reference queue while primary ACK ages stayed around 27 ms.
This motivates commit at the duplicate artifact's lifecycle boundary. The
authoritative rosbag MCAP storage policy is unchanged. The four pending capture
slots, four reference slots, 100 ms lease, overflow faults and terminal drain
remain fixed; this decision supersedes per-operation durability only for the
paced comparison artifact. Incomplete reference finalization fails equivalence.

A subsequent frozen candidate passed six seeds before a reference append stalled
for 226 ms, filling the unchanged four-sample queue while primary ACK ages stayed
around 28 ms. Removing per-operation fsync alone did not isolate disk writes.
Paced comparison files now stage in a dedicated, size-limited 1 GiB Compose
tmpfs. Startup verifies the tmpfs mount; there is no disk-staging fallback.
Each attempt uses an exclusive temporary directory. Finalization or orderly
interruption copies its completed file or partial prefix to the output directory,
syncs it, and only a complete file receives atomic publication. Successful copy
removes the staged files. Disk-copy errors fail finalization; capacity exhaustion
or writer failure interrupts through the same guard. Terminal drain remains
bounded at ten seconds and occurs with motion disabled. The authoritative rosbag
continues to write its configured disk storage, and ordinary canonical sinks
retain their original behavior. Abrupt worker loss may lose the duplicate
comparison prefix; that diagnostic artifact has no crash-durability guarantee.
The extra tmpfs budget is bounded independently of the four queued samples.

The unchanged 100 ms capture lease covers original receipt, rendering and the
authoritative rosbag/DDS ACK. Reference durability is traced separately and
cannot renew a source timestamp or grant motion. Reference queue/write timings
and peak pending samples are recorded; storage failures remain explicit faults.
No captured RGB or metadata changes.

A subsequent trace-stage stall exceeded the lease even though actual message
construction took only 0.6 ms and had no GC. Diagnostic file opens/writes consumed
about 79 ms. Transport, capture, mirror and progress traces therefore use bounded
1 MiB buffers and close during orderly cleanup. Progress retains ordered JSONL
and a final snapshot; trace persistence is diagnostic, not an ACK or freshness
authority. File failures remain errors. Abrupt process loss may truncate these
diagnostics, while the authoritative bag and acquisition lease remain unchanged.

The staged-reference cohort completed every task but failed data/motion
qualification: eleven recordings had excessive sample gaps and three attempts
exceeded measured acceleration bounds during freshness/watchdog takeover.
Expert logging still flushed each command, and physics trace buffers could
write disk during active motion. In the recording profile, these diagnostic
files, capture/reference timings and recorder progress now stage in the same
size-limited tmpfs. Expert traces commit after `record_stop` or shutdown;
physics traces commit after owner shutdown. Other runtime profiles retain their
ordinary trace paths. Abrupt process loss can truncate staged diagnostics.
Independent hold behavior, original source timestamps, quality limits and
physics ceilings remain unchanged. Task success alone cannot pass qualification.

The staged-diagnostic trial passed four targeted cases but failed seed 13's
disabled finalization: its durable disk commit took 2.38 seconds, exceeding the
initial two-second drain bound. The ten-second terminal bound is separate from
the unchanged 100 ms active acquisition lease and source freshness. Startup
remains two seconds, pending queue limits are unchanged, and an incomplete or
timed-out final commit still fails the recording result. Motion is inhibited
before the longer housekeeping wait; it cannot renew any authorization.

Use a persistent single-thread executor for each transport node; a sink reuses
its caller's executor when one is already attached. Drain matched acquisition
endpoints for two bounded startup seconds and the gateway for one second while
disabled, before resetting and timestamping the first recorded sample. This
settling stage grants no authorization and fabricates no acquisition samples.
The runtime retains timing traces rather than claiming that publication mode or
executor membership alone explains every observed middleware pause.

Stream physics evidence with one pending editable tick and a 1 MiB file buffer,
then publish its complete JSON array atomically at shutdown. Every tick remains
in order, including budget overruns and fault holds. Previously retaining all
tick objects grew full garbage-collection pauses to 56 ms in a complete CPU
task; the bounded trace trial reduced the owner's observed maximum to 7.6 ms.
This changes evidence storage, not the controller, watchdog or hold gains.
Physical trace writes are included in the owner's measured tick budget; disk
and scheduler timing remain measured limitations rather than real-time claims.

Full-task demo exit status includes task completion, valid quality and measured
motion limits. Successful transport alone cannot label a failed qualification
as passed. Synthetic pause injection samples the current owner clock before
advancing steady time, so a pause test cannot accidentally inject a backward jump.

Worker render/transport timings and retained recorder failures make overload
inspectable. Sustained runs reuse the physics owner and recorder across attempts,
with new episode UUIDs/workers and continuous packet ordering. Reports distinguish
cohort success from the three 600-second full-quality GPU gate and manual replay.
Raw bags/traces remain ignored artifacts.

## Safety and consequences

The staged plan contains the required safety matrix. Existing independent owner
hold and accepted gripper retention remain the fault path. Playback cannot grant
motion authority. Successful simulated tasks, throughput and replay do not
constitute physical mounting, stop/hold or hardware commissioning evidence.

The larger container shared-memory allowance increases the optional resource budget.
No new dependency is added. The source cadence and task preset affect experiment
identity, and camera quality profiles remain incompatible. A failed qualification
reports its unmet gates and exits unsuccessfully; it cannot enable PR 15 hardware
claims.

## Sources

- [Pinned RMW 8.4.4 participant discovery configuration](https://github.com/ros2/rmw_fastrtps/blob/8.4.4/rmw_fastrtps_shared_cpp/src/participant.cpp)

- [Fast DDS 2.14 large-data transport configuration](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/env_vars/env_vars.html)

- [Fast DDS shared memory configuration](https://fast-dds.docs.eprosima.com/en/2.x/fastdds/transport/shared_memory/shared_memory.html)
- [Fast DDS reliable writer timing](https://fast-dds.docs.eprosima.com/en/2.14.x/fastdds/xml_configuration/datawriter.html)
- [ROS Fast DDS publication modes and XML configuration](https://github.com/ros2/rmw_fastrtps/tree/jazzy)

## Evidence

See the [staged implementation plan](../plans/crisp-task-qualification.md) and
[qualification report](../reports/crisp-task-qualification.md).
All timing, configuration and safety claims require their actual run artifacts.
