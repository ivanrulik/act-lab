# CRISP task and continuous acquisition qualification

Post-merge main CI exposed a loaded watchdog takeover above the joint
acceleration bound. [ADR 026](../adr/026-loaded-watchdog-hold-damping.md) records
the failing trace and correction. The frozen campaign below remains evidence
for its recorded revision; it does not qualify every possible timeout state or
automatically transfer to revised hold parameters.

Status: local qualification complete. Delivery checks and review are tracked on
the implementing PR. Historical trials below retain their individual failures.

The explicit candidate and experiment boundary are defined in
[ADR 025](../adr/025-crisp-task-qualification.md) and
[the staged plan](../plans/crisp-task-qualification.md). Raw evidence remains in
ignored run directories. The source/configuration hashes in each JSON report
identify the candidate; earlier unsuccessful trials are retained separately.

## Completed task and fault stages

The required CPU fixture completed actual grasp, lift, transfer, placement and
release in 95.062 simulation seconds / 101.610 wall seconds, with 1,954 samples.
The recorded episode is valid and training-eligible, with exact equality between
the canonical mirror and imported ROS bag. The maximum receipt-to-ACK age was
32.36 ms. The current fixture passed the unchanged joint-acceleration
4 rad/s² ceiling. All measured motion limits passed, with no unexpected fault
hold. This fixture uses reduced
effects and software rendering; it is not full-quality GPU throughput evidence.

The freshly built recording image passed all 29 tests in 782.87 seconds, including loaded
active reset and the stricter gateway-loss episode check. Generated
serialization, lifecycle, original RGB identity, loss, ACK retries and all eight
loaded fault cases ran against actual DDS and CRISP.

| Scenario | Observed result |
| --- | --- |
| Startup | Disabled readiness and bounded settling precede the first timestamped sample |
| Nominal complete task | Actual release; valid quality; exact canonical equality; measured limits pass |
| Producer or controller loss | Independent fault hold retains the accepted aperture; fresh-sequence recovery passes |
| Gateway death | Owner finalizes independent fault-hold evidence with the original episode; loaded grip retained |
| Capture worker loss/stall or recorder loss | Recording interrupts and remains partial; loaded hold retains grip |
| Clock pause | Injected exact 100 ms pause inhibits motion; fresh-sequence recovery passes |
| Reset during loaded recording | Reset rejected; original episode/grip retained in shutdown hold; unfinished bag remains partial |
| Completed attempt followed by reset | New UUID and contiguous acquisition packet ordering pass existing lifecycle tests |
| Shutdown | Disabled hold precedes drain/finalization; task effort is zero in hold |
| Replay | Historical camera/report data is read-only and grants no motion authority |

Loaded checks retain the existing 2 mm wrist-drift, 1 cm cube-drop and continued
pad-contact bounds. Reset rejection follows orderly shutdown; abrupt gateway
death is tested separately. Neither demonstrates a physical stop or hold.

## Findings retained from unsuccessful trials

The first complete frozen GPU cohort finished at **17/20**, with matched local
20/20. Seed 10 hit joint-acceleration rejection; seeds 13 and 15 expired capture.
Its source hash is `6b426406459e9d8df5f1c1197772da251adf5fbf4a8b3f37489de8895da91ae3`.
It failed qualification and correctly skipped sustained runs. A subsequent
0.8 mm downward-lookahead trial on seed 10 failed IK near a straight elbow;
introducing an inward waypoint during descent also reached acceleration rejection.
These results remain separate from the revised approach/mirror candidate.

The revised approach with a bounded reference writer passed the frozen **20/20**
GPU cohort and matched local 20/20. Its first sustained run interrupted after
412 active seconds/four complete attempts when a 210 ms reference `fsync`
filled its four-sample queue; primary ACKs remained about 27 ms. The comparison
artifact now commits at lifecycle boundaries while retaining ordered flushes.
The authoritative bag, capture/reference queue bounds and freshness limits stay
fixed. That revision was rerun in the subsequent retained campaign.

The next campaign was stopped after seed 1 exceeded capture age at 172 ms.
Its 146 ms construction-stage interval did not identify the cause on its own.
A separate instrumented diagnostic passed seeds 1 and 3, while seed 2 exceeded
capture age at 106 ms: actual message construction took 0.6 ms with no GC,
but the preceding diagnostic trace write blocked for about 79 ms. Timing and
progress logs now use bounded 1 MiB buffers and close during orderly cleanup,
instead of opening/flushing files for each stage. This changes diagnostic I/O;
it does not extend the authoritative acquisition lease or change image bytes.
Both unsuccessful runs remain separate evidence; later campaigns tested the revision.

The buffered-log candidate passed six seeds before seed 6 hit reference
backpressure: an append stalled for 226 ms with fast explicit flushes, while
primary ACK ages remained about 28 ms. That campaign was stopped and retained.
The comparison file now stages in a dedicated 1 GiB tmpfs and copies/syncs to disk
at the disabled lifecycle boundary. Queue capacity and terminal drain bounds
are unchanged. ROS-free tests cover completed and interrupted copies and disk
failure refusing final publication. The next retained GPU campaign tested this revision.

The staged-reference revision completed 20/20 CRISP and matched local tasks,
with no capture-timeout or reference-backpressure error. It still **failed** the
cohort gate: 11 recordings had excessive source sample gaps, and seeds 12, 13
and 16 exceeded measured acceleration ceilings during watchdog/freshness holds.
Only eight attempts passed all data and motion gates. The maximum source gap
was 726 ms; nominal median spacing was 48 ms. Its source hash is
`688bf1d7d6dca25a79f4949edad5f1961fe8445986e4cc7eb56242ba2f0f4cb5`.
No sustained run started. Per-seed task completion is not full qualification.
Active expert/physics/capture/recorder diagnostic traces now stage in the same
bounded tmpfs and persist at disabled finalization. The authoritative bag still
uses its disk storage. Targeted gap/acceleration cases and the final frozen campaign followed
this revision.

The targeted staged-diagnostic trial passed all data and motion gates for
seeds 2, 12, 16 and 17. Seed 13 failed the two-second disabled reference drain:
its durable disk commit took 2.38 seconds. Terminal commit now has a separate
ten-second bound while motion is inhibited; startup remains two seconds and
the active acquisition lease remains 100 ms. This failed trial is retained.

- A constant-height transfer clipped the tray rim. Raising the entire transfer
  reached a near-straight elbow configuration. An explicit intermediate waypoint
  and destination clearance address the geometry within the existing expert FSM.
- Task success could occur while the cube remained pinched. Qualification now
  additionally requires measured aperture ≥0.95 and zero pad/cube contacts.
- Opening while continuing to lower the wrist caused a release acceleration
  rejection. Tight placement alignment and a measured-pose opening hold address
  the source policy without changing safety limits.
- A synchronous packet publication blocked for 330 ms. Explicit loopback/SHM
  configuration, asynchronous publication and bounded identical retries preserve
  the original timestamps and unchanged 100 ms application acceptance lease.
- Retaining every physics tick in memory produced owner GC pauses up to 56 ms;
  a later nominal task exceeded the joint-acceleration gate during takeover.
  Streaming every row through one editable pending tick reduced the measured
  owner GC maximum to 7.6 ms. Finalization remains atomic; interrupted evidence
  stays explicitly partial.
- A stopped executor retained by a reused ROS node prevented ACK delivery after
  reset. Detaching the owned executor before shutdown fixes that lifecycle; both
  reset regressions and the full recording suite pass.
- Mirror `fsync` pauses of 89 and 133 ms blocked subsequent rendering after
  DDS ACK delivery. A bounded reference writer permits authoritative capture to
  continue; finalization/interruption requires sync. Ordering, write-failure
  propagation and four-sample backpressure have ROS-free tests.

These findings do not attribute every scheduling/middleware delay to a single
cause. Timing traces retain separate rendering, storage, callback and GC stages.

## Frozen cohort for the current revision

The current full-quality NVIDIA cohort passed every task, data and measured
motion gate: **20/20 CRISP and 20/20 matched local tasks**. All twenty recordings
are valid and training-eligible, with exact canonical mirror/import equality.
Source hash: `81c9545f020c80074e825b782b21c16cf5e760058ca8a596af83d8784a65fe3d`.
Configuration hash:
`a0525aa28daaa58c616dc24224d273c08f77082907d7dd40c0c06a83989518de`.

| Cohort metric | Measured maximum | Existing ceiling |
| --- | --- | --- |
| Receipt-to-ACK age | 43.75 ms | Strictly below 100 ms |
| Source sample gap | 60 ms | At most two median sample periods |
| Joint acceleration | 2.133 rad/s² | 4 rad/s² |
| Cartesian acceleration | 0.595 m/s² | 1 m/s² |

Cohort completion ranged from 92.336 to 99.902 simulation seconds. The identified
local platform is an Intel Core i7-13620H with an NVIDIA RTX 4060 Laptop GPU,
NVIDIA driver 610.57.04, Python 3.12.3, Jazzy and Fast DDS.

Median source spacing is approximately 48 ms (about 20.8 samples/s), distinct
from the configured 25 Hz source ceiling. No timestamps or pixels are fabricated
to fill intervals. The three sustained full-quality runs passed.

## Sustained full-quality NVIDIA capture

Active wall time sums the expert execution intervals and excludes reset,
disabled finalization and offline import. Each run reused its physics owner and
recorder for six complete attempts with
new episode UUIDs. All 18 recordings are valid, training-eligible and exactly
equal to their canonical comparison artifacts. No measured motion-limit violation,
capture expiration or incomplete finalization occurred. Maximum source gaps
were 62, 56 and 58 ms; pending capture depth peaked at one in each run.

| Run | Active wall seconds | Samples | Maximum receipt-to-ACK | Physics budget overruns |
| --- | --- | --- | --- | --- |
| 1 | 612.213 | 11,777 | 34.96 ms | 21,864 / 301,062 ticks |
| 2 | 612.293 | 11,772 | 34.37 ms | 22,929 / 300,651 ticks |
| 3 | 611.065 | 11,761 | 40.50 ms | 18,280 / 301,214 ticks |

Budget overruns remain recorded even when freshness and physical limits pass.
These results establish this local simulated capture gate, not hard real-time
performance. Maximum task effort was 2.953 Nm against 5 Nm; hold task effort
was zero. The matched cohort success interval is 83.9–100% (Wilson 95%). These
inspected seeds are qualification fixtures, not an untouched generalization
cohort.

The frozen campaign took 5,807.75 wall seconds including offline imports.
A separate monitor sampled Docker, GPU and free disk every ten seconds.
Sampled peak container memory was 10.02 GiB, peak GPU allocation 180 MiB, and peak
GPU temperature 49 °C. Offline imports ended around 468 MiB container memory.
Free disk decreased from 60.71 to 30.66 GiB as artifacts accumulated. These
measurements cover the whole campaign and do not prove absence of a memory leak.
Resource measurements and raw per-tick traces remain under the ignored run
directory, alongside source/configuration hashes and earlier failed campaigns.

## Manual Foxglove replay

The frozen full-quality NVIDIA seed-0 recording was replayed in Foxglove using
three original image topics and the command report. Wrist orientation, grasp,
release, middle/end scrubbing, original episode/sequence and source versus bag
receipt timestamps were checked. The final report is disabled, sequence 1954,
for episode `dfb65bec-d857-4059-8cf3-df698dcc7f38`. The README capture shows
loaded transfer at sequence 1301. The original source clock and wall receipt
time remain visibly distinct; terminal source/report timestamps are 95.028 /
95.070 simulation seconds before the one-second ROS offset.

The screenshot in the [README](../../README.md) is an actual browser capture.
The saved browser layout was assembled interactively; exact JSON layout import
was not verified. The versioned layout disables annotation timestamp sync for
raw-only image panels. This replay does not qualify the deferred URDF flange
appearance or physical camera fidelity.

## Controlled contention and ROS-free conversion

A separate full-quality GPU trial held a lifted cube while four CPU competitor
processes ran for 10.07 seconds. Capture completed; 132 measured snapshots
retained the accepted aperture and pad contact. Maximum measured wrist drift
was 0.196 mm against the existing 2 mm bound. Final independent hold retained
contact and cube height; measured physics limits passed with zero task effort
in hold. The first harness attempt failed after contention because it performed
offline import before shutting down the gateway, exceeding the bounded IPC
wait. Its failed report is retained. The corrected harness shuts down before
import; no runtime/source change or safety relaxation was needed.

The public manifest/quality/conversion CLI ran in the ROS-free CPU training
image on frozen seed 0. Both canonical and ROS-imported sources produced 2,376
derived frames with identical validated logical fingerprint:
`e564d469f4a0800d002212cea40dd389969dcec3107df220442fc7279c2e903d`.
Neither conversion imported `rclpy`, `rosbag2_py` or generated ROS interfaces.
A second public ROS import into an independent fresh output matched the first
exactly for every decoded episode field and pixel.
The existing video decoder reported its TorchCodec loading warning and used
its configured PyAV fallback; this is CPU dataset conversion, not GPU rendering
or training-device fallback.

## Delivery

The recording image was rebuilt successfully. Hosted checks and review are
tracked on the implementing PR; the local evidence does not substitute for them.

## Container regressions

The authoritative Compose dev build, Ruff, mypy and doctor passed. Default
pytest passed 383 tests with four expected optional skips and one warning.
The separate CPU training suite covers the LeRobot tests skipped in dev.

| Optional runtime | Result |
| --- | --- |
| Generated ROS contracts | 3 passed |
| Actual CRISP feasibility controller | 5 passed |
| Moving simulation and independent hold | 10 passed |
| Observability, tool frames and wrist images | 6 passed |
| Recording and loaded faults | 29 passed |
| ROS-free CPU training/data | 10 passed |
| ACT teaching notebook | Executed successfully |

Hardware, operational-space moving control, ACT training through ROS, camera
depth/stereo, physical mounting, system clocks and crash-prefix repair remain
outside this branch. Successful simulation evidence permits later planning;
it does not authorize hardware use.
