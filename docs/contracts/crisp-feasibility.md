# CRISP stationary feasibility bench (PR 12)

This opt-in bench assesses an unmodified controller through real ROS lifecycle,
pluginlib, DDS, and loaned mock interfaces. It does not integrate effort into
motion. The default simulation and learning paths do not import ROS or CRISP.

## Reproduce

```bash
docker compose --profile crisp-feasibility build crisp-feasibility
docker compose --profile crisp-feasibility run --rm crisp-feasibility \
  act-lab ros2 crisp-feasibility --output runs/crisp-feasibility --json
docker compose --profile crisp-feasibility run --rm crisp-feasibility \
  python -m pytest -o 'addopts=-ra -p no:cacheprovider' tests/crisp
```

A completed no-go returns zero. Missing dependencies, revision mismatches,
incomplete evidence, timeout, or broken harness assertions fail execution.
`--ci-timing` records measurements with outcome `ci_evidence_only`; it cannot
issue a local conditional go. CI uploads evidence even on failure. CI runs with
the checkout owner UID/GID so evidence is writable; ROS logs use a container-local
`/tmp` directory. On hosts whose user is not UID 1000, add
`--user "$(id -u):$(id -g)"` to Compose run commands.

## Runtime and dependencies

The pinned Jazzy/Noble image is shared with PR 11. CRISP and the official UR
model are built from the revisions in `Dockerfile.crisp`; the official UR driver
is retained as pinned audit source only. The C++ harness uses controller_interface,
hardware_interface and pluginlib to load the actual controller, rclcpp and
geometry_msgs for separate-process pose delivery, Pinocchio for model/FK/gravity,
and nlohmann_json for an inspectable trace protocol. Colcon/CMake/GCC build the
packages. Pytest validates the required bench; Ubuntu numerical packages remain
in place. No crisp_py, Gym, Franka drivers, MuJoCo or learning packages are installed.

Build type is Release, native CPU optimization off, upstream precompiled headers
on, one compiler job. The controller's configure callback fetches the model from
a `robot_state_publisher` parameter service; the harness supplies that test service
on a background executor. Pose callbacks only update the upstream realtime buffer.
Mock state/effort access and lifecycle transitions are owned by the bench thread.
Fast DDS uses localhost discovery in one container without host networking,
devices, display, GPU, or privileged access.

## Configuration and native cases

UR5e model: official description, no prefix, `world` root, `tool0` end-effector.
The six joint names follow the v1 contract. Home joints match
`configs/sim/ur5e_pick_place.toml`; nearby joints alternate +/-0.02 rad; the wrist
fixture sets wrist_2 to 1e-6 rad. No state integrates torque into motion.

Both modes use the same controller plugin and pinned default task gains:
translation 500, rotation 30, automatic damping `2*sqrt(k)`. Impedance computes
`J^T (K e - D J dq)`; operational-space adds the regularized task inertia before
`J^T`. Equal numbers therefore have different gain semantics; stationary torque
cannot rank tracking quality.

Baseline: noise, variable stiffness, external wrench input, friction, gravity,
Coriolis and nullspace stiffness are disabled. Friction arrays contain six zeros.
Joint repulsion is disabled **and its maximum torque is zero** because the pinned
update does not consult its enable flag. The explicit qualification profile is
`configs/ros2/crisp-feasibility.json` (schema 2): target pose 0.1; q, dq and q_ref
0.0; output torque 1.0. For target/state, alpha is previous-sample retention:
`(1-alpha)*current + alpha*previous`; orientation is `target.slerp(alpha, previous)`.
Thus target 0.1 retains 10% history, state 0.0 passes current feedback. The target
parameter cannot be 0.0 under pinned validation. Output operands are reversed,
so output 1.0 passes new torque. This is an explicit requalification configuration,
not a claim that one setting disables every filter.

The `historical` case retains all target/state filters at 1.0 and must still freeze
pose response. `responsive_probe` repeats candidate native loss behavior;
`smoothing` uses target 0.2 to expose retention. Position and velocity feedback
are changed after activation and checked against independent Pinocchio task-torque
calculations. Translation, pure rotation, home/nearby/near-singular efforts must
match those calculations within 1e-6 Nm. Compensation toggles are isolated;
gravity is compared with the model and never authorized for UR hardware.

### Local scene and UR tool frame

The bench validates local scene gripper poses before mapping them into CRISP's
UR-world/tool0 convention. Write `T_scene_tip = A * T_UR_tool0 * B`, where A is
Rz(pi) without translation and B is a 110 mm translation along tool0 Z without
rotation. Approved targets use the exact inverse `A^-1 * T_scene_tip * B^-1`.
This adapter-local mapping preserves source timestamp, sequence and episode;
ROS v1 codecs still only reorder XYZW/WXYZ. Workspace limits stay unchanged.

The committed `tests/fixtures/crisp/kinematics.json` is generated in the ROS-free
dev environment with `docker compose run --rm dev python
scripts/crisp-frame-reference.py`. It uses `mj_forward` without a physics step,
records both flange and tip at nine joint configurations, model hashes and runtime
version. `--check` and dev integration tests regenerate it; stale asset hashes
fail the ROS runner. The ROS image consumes the fixture without installing MuJoCo.
The C++ bench checks mapped UR FK against local tip FK, and inverse round trips.
The models differ slightly: translation must be <=2 mm and rotation <=0.02 rad,
matching existing local convergence tolerances, not claiming calibrated equivalence.
PR 13 must validate moving dynamics, tool/payload conventions and residuals.

Potentially unsafe nonfinite/nonunit inputs and invalid configurations run in
isolated process groups, bounded to 90 seconds. A timeout is incomplete evidence
and fails execution. Native outputs and abnormal exits remain findings; successful
plugin initialization and all required baseline/guard executions are mandatory.

## Test-only trace and effort gate

`authorization.jsonl` retains the original event protocol; `report.json` is schema 2.
Events are `clock`, `authorize`, `fault`, or asserted `step`. Every event carries
monotonic steady nanoseconds. Clocks carry domain nanoseconds and episode UUID;
ROS pose stamps use domain + 1 second. An authorization contains source stamp,
original steady receipt, episode, uint64 sequence, enable, world pose in WXYZ,
and accepted gripper aperture. Numeric/domain shape validation runs without ROS.

Python generates normal authorizations through `CommandInbox` and
`SafeCartesianRobot`, with a stationary fake driver. Explicit `adversarial`
events bypass that approval solely to challenge the independent C++ gate.
The gate processes clock/step events independently of authorization events:
receipt, source age or clock nonprogress at **100 ms** inhibits output even when
no further authorization arrives. Source stamps are never refreshed on reuse.
Backward reset latches until a new episode; pause recovery needs fresh sequence.
Every fault inhibits mock effort immediately to zero and requires controller
reconfiguration/reactivation before fresh intent can resume.

Raw CRISP effort is recorded separately from gate input and guarded mock output.
Enabled output must be finite, within +/-5 Nm and change by at most 100 Nm/s.
Large/nonfinite synthetic gate inputs are marked; actual controller output remains
available alongside them. Fault zeroing overrides slew. **Zero mock effort is not
a safe physical hold**, and this event-driven gate is not a production independent
watchdog. No hardware controller or stop mechanism is supplied by this PR.

## Dedicated safety review matrix

| Condition | Native assessment | Independent mock gate assertion | Production requirement |
|---|---|---|---|
| Startup / no command | Activation initializes measured FK target | No enabled effort authorization | Validate physical startup/ownership |
| Fresh command | Translation/rotation response and quaternion mapping | Shared safety approval, validated frame mapping, finite/ceiling/slew | Validate moving dynamics and tool calibration |
| Exact source age 100 ms / forward jump | Stamp ignored in responsive probe | Zero at boundary, original stamp retained | Source freshness through whole adapter |
| Publisher loss | Separate pose process killed; inspect retained effort | Receipt expires at 100 ms | Independent actuator watchdog |
| Approval process loss | No more authorize events; clock/steps continue | Receipt expires even with clock progress | Independently scheduled/process-isolated gate |
| Clock pause | Injected clock stops, steady advances | Zero at 100 ms despite renewed approvals | Real clock binding and watchdog |
| Resume | Native target remains cached | Fresh sequence plus reconfigure/reactivate | Validate physical recovery transition |
| Backward reset | No native episode contract | Cached intent cleared, new UUID required | Reset ownership and state synchronization |
| Replay / wrong episode | No native metadata | Clear intent and inhibit immediately | Preserve producer/episode contract |
| Future / invalid frame | Header not validated natively | Independently rejected | Bind world frame explicitly |
| Nonfinite/nonunit input/output | Isolated native payload case | Immediate zero; quaternion norm and finite recovery | Validate upstream numeric handling |
| Deactivate / shutdown | Capture unchanged effort buffer | Immediate zero outside slew | Implement actual stop/hold and confirm driver behavior |
| Gripper on holds | CRISP exposes joint effort only | Shared safety retains accepted aperture | Separate validated gripper control |

## Timing and qualification

Each mode runs 250, 500 and 1,000 Hz settings; each rate has three independent
home-state processes, each with 1,000 warmup and 10,000 measured updates. Reports
contain median/p95/p99/max and budget overruns. Per-update torque delta is
`100/rate` Nm. Updates run as a throughput benchmark, without realtime scheduling
or a claim that a physical loop meets its deadline. Nearby/singular states are
exercised separately in native numeric/lifecycle tests.

A conditional go requires lifecycle/interfaces, independent pose/feedback torque
checks, stationary frame binding, all guarded
fault assertions and p99 strictly below 2 ms in every local 500 Hz run. Impedance
wins if both qualify; OSC only if impedance does not. Otherwise selection is
deferred with no-go. These rules do not establish hardware real-time performance.

Evidence under ignored `runs/`: report/provenance, resolved parameters for both
modes, authorization JSONL, application decisions, per-case raw/guarded traces,
process logs and benchmark summaries. Source revisions, generated model SHA-256,
compiler/CMake flags, installed packages, ROS/RMW and machine are recorded.

See [ADR 015](../adr/015-crisp-configuration-requalification.md) and the
[requalification report](../reports/crisp-requalification.md) for the current decision.
[ADR 014](../adr/014-crisp-feasibility-and-controller-decision.md) and the
[original assessment](../reports/crisp-feasibility.md) retain the historical no-go,
UR driver source audit, upstream license discrepancies, and deferred work.
