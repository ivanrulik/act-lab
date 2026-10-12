# CRISP task and continuous acquisition qualification

Status: implemented and locally qualified; hosted delivery checks and review
are tracked on the implementing PR.
Branch: `feature/crisp-task-qualification`.
Base: merged main `8c39c37270168b7754546a11481353cb6afe6ecc` (GitHub PR #25).

## Goal and boundary

Qualify complete scripted pick/place through generated DDS commands, shared
application safety, real Cartesian-impedance CRISP, and the independent MuJoCo
physics owner while recording original full-quality RGB. Demonstrate sustained
capture, canonical import/conversion, and read-only Foxglove replay before PR 15.

Preserve ADRs 015–016, 018–022 and 024. The local expert is the baseline;
operational-space control, hardware commissioning, ACT training, physical
sensors, clock synchronization, crash repair and flange appearance remain
separate work. No additional viewer or runtime dependency is planned. Add an
ADR if implementation requires a consequential architectural change.

## Starting evidence and gaps

- The local articulated preset previously qualified 20/20 expert seeds.
- Moving CRISP has grasp/lift and loaded fault-hold evidence, but the CRISP
  recording CLI currently submits a fixed nearby target and labels it failure.
- PR #25 captured 101 full-quality GPU samples over 2.608 simulation seconds;
  it did not establish successful task recording or sustained throughput.
- A concurrent-workload GPU run and full-quality software capture expired the
  unchanged 100 ms capture lease. Retain those failures as baseline evidence.
- Interactive Foxglove bag playback remains unqualified.

## Implementation

The explicit candidate uses 500 Hz physics / 25 Hz application commands, the
original tooling gains, a bounded 240-second task budget, explicit transfer
waypoints and destination clearance, an inward/lower cube approach, 6 mm XY
transit lookahead, slower 0.05/s finger input, tighter
grasp alignment and a stricter command-speed ceiling. ADR 025 records these choices
and retained unsuccessful configuration trials.

### 1. Complete expert execution

Add an explicit full-task mode to the ROS recording command; retain the existing
bounded movement mode and its failure label. Reuse `ScriptedPickPlaceExpert`
without duplicating its phase machine. Supply coherent measured observations,
command reports and privileged task geometry from the authoritative owner through
a narrow adapter. Task geometry remains unavailable to generic policy inputs;
do not infer task success from the commanded target or a stale scratch model.

Deliver complete pose/gripper/enable intent over the existing v1 command topic.
Preserve original source timestamp, episode and sequence. Record expert phase,
actual owner mode/reason, recovery events, contact/lift/release and terminal task
state. Outcome labels come from existing task criteria; timeout, expert failure,
capture failure or incomplete evidence cannot become success.

Use the existing bounded readiness/recovery path. Log every intervening hold;
nominal retries require freshly sampled state and a fresh command sequence.
Explicit fault scenarios must not retry invisibly. Finish acquisition before
reset; each attempt receives a new UUID/worker and correct packet continuity.

### 2. Qualification runner and evidence

Provide a lazy CLI qualification runner under `act-lab ros2`, with explicit
config, fixed seed list, output, renderer/profile and bounded duration. Execute
through existing Compose services. Write a versioned JSON summary, resolved
configuration, provenance and per-attempt traces under ignored `runs/`.

Report task success and Wilson 95% intervals, completion simulation/wall time,
expert phase failures, safety rejections, holds/recoveries, joint/Cartesian
limits, raw/gated effort, gripper/contact events, queue depth, sample gaps,
receipt-to-ACK median/p95/p99/max, physics budget overruns, throughput and resource
usage. Retain unsuccessful and interrupted attempts alongside successful ones.

Compare local and CRISP task results on identical seeds/model/task settings and
the same application safety path. Controller-dependent trajectories need not
match; canonical mirror versus imported ROS recording must match exactly.
Record actual clock/hold events rather than claiming bitwise determinism from
seed equality alone.

### 3. Data and replay handoff

Validate/import complete bags, run existing quality and selection rules, and
derive a small LeRobotDataset in the ROS-free training/data image. Compare local
canonical mirror and ROS import for pixels, timestamps, state, command reports,
calibration, quality decisions and deterministic derived logical fingerprints.
Keep separate model/profile identities; no relabelling or weakened quality rules.

Use `configs/ros2/foxglove/act-lab-recording.json` for read-only playback. Verify
all three cameras, wrist orientation, source versus receipt time, report/episode
identity, start/end boundaries and scrubbing. Verify fault evidence independently
of whatever cached pixels the viewer displays. Record a manual acceptance
checklist and screenshots; the robot-mesh assembly review is outside this plan.

## Testing stages and gates

| Stage | Run | Gate before advancing |
| --- | --- | --- |
| 0: baseline | Fresh local articulated expert on seeds 0–19; CRISP motion/loaded-hold and paced recording regressions | Record current results/configuration; resolve regressions before new task claims |
| 1: task plumbing | ROS-free tests for expert adapter, task outcomes, timeout/rejection, new attempts and import isolation; short generated DDS integration | Owner-derived coherent task state; full intent preserved; no fabricated success or renewed stale intent |
| 2: one complete task | Seed 0 full-quality NVIDIA paced task, with phase and physics traces | Actual grasp, lift, transit, place and release; existing task success; complete valid bag and exact canonical equality |
| 3: seeded cohort | Freeze seeds 0–19 before tuning; matched local/CRISP runs, retain all attempts | At least 18/20 CRISP task successes, no existing motion/effort limit violation or capture deadline failure in nominal qualification; report all failures and confidence intervals |
| 4: sustained capture | Three standalone full-quality GPU runs, each at least 10 steady minutes, cycling complete attempts with new UUIDs; separate controlled contention run | Standalone runs have no silent losses, unexpected queue/lease failure or incomplete finalization; report per-run ages/gaps/resource trends; contention either passes or interrupts into verified hold with retained evidence |
| 5: loaded faults | Producer/gateway/controller loss or stall, worker/recorder loss, slow capture/queue overload, pause/reset and recovery during grasp/lift | Existing 100 ms boundaries and independent owner safety hold; retained accepted grip; loaded drift/contact within existing qualified bounds; no old authorization recovery |
| 6: pipeline/replay | Repeated import, quality/manifest/LeRobot equivalence, interactive Foxglove playback | Exact canonical/derived equivalence, interrupted attempts excluded, manual replay acceptance recorded |
| 7: delivery | Required dev and optional container suites, hosted CI, evidence review | All required checks green; findings and limits explicit; draft PR ready for review |

The 18/20 task target follows the existing expert acceptance target; it is not a
hardware reliability claim. Cohort selection cannot be adjusted after inspecting
failures. Any tuning gets an explicit config identity and a complete rerun of
the frozen cohort. Hardware-sensitive GPU throughput gates run locally on the
identified machine; shared CPU CI uses the existing explicit reduced-effects
fixture and never claims full-quality GPU performance. A failed stage remains a
reported qualification gap, not a reason to silently lower its gate.

## Safety review to implement and record

| Scenario | Motion and acquisition expectation |
| --- | --- |
| Startup/warmup or unavailable requested GPU | Existing hold; no task authorization before ready; explicit renderer failure |
| Nominal loaded transfer | Shared limits and independent effort gate apply; original image/state identity retained |
| Producer/gateway/controller disappearance | Independent owner takes over; no recording or recovery event renews intent |
| Capture overload/renderer/recorder loss | Interrupt acquisition at existing lease/queue bounds; disabled gripper-preserving hold and partial evidence |
| Clock pause, replay or wrong episode | Existing freshness/sequence checks; fresh command required for recovery |
| Backward reset | Reject active-attempt reset; complete/interrupt first, then new episode and readiness |
| Fault while carrying cube | Retain accepted aperture; measure contact, drift and effort against existing loaded-hold gates |
| Terminal success/failure or shutdown | Disabled hold before bounded capture drain/finalization; record actual terminal state |
| Playback | Historical read-only data grants no motion authority |

Inject clocks for exact boundary assertions; use bounded discovery for DDS.
Retain actual owner traces after abrupt process death. Do not substitute orderly
cleanup for independent watchdog evidence. Do not relax safety, capture age,
image identity or training quality to achieve throughput/task success.

## Acceptance and delivery

- Run `docker compose build dev`, Ruff, mypy, pytest and `act-lab doctor` through
  Compose; run existing ROS contracts, CRISP feasibility, moving simulation,
  observability, recording/conversion and CPU training checks.
- Add bounded full-task/fault coverage to existing required CI jobs; long local
  GPU runs remain explicit evidence, not mandatory shared-runner timing tests.
- Commit a concise qualification report and safety matrix; update usage,
  contracts, architecture and roadmap. Preserve raw artifacts under ignored
  `runs/`; publish selected CI evidence as artifacts.
- Open one draft PR with passed/failed stages and validation evidence. PR 15
  planning may use the completed simulator evidence; this branch authorizes no
  physical robot motion. Merge requires subsequent user authorization.
