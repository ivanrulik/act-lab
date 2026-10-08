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
update does not consult its enable flag. Target/state/output filters remain 1.0
as specified by the assessment plan. That setting freezes target/state history
in this CRISP revision, rather than disabling smoothing. A numeric pose-response
test includes pure translation and pure rotation and records failure explicitly.

`responsive_probe` and `smoothing` use target filter 0.1 solely to diagnose native
target/loss behavior. They are not candidate baseline configurations. Compensation
cases initialize measured velocities before activation so the frozen state filter
does not hide their effects. Gravity is compared with the same model's calculated
gravity; it is never authorized as a UR hardware configuration.

Potentially unsafe nonfinite/nonunit inputs and invalid configurations run in
isolated process groups, bounded to 90 seconds. A timeout is incomplete evidence
and fails execution. Native outputs and abnormal exits remain findings; successful
plugin initialization and all required baseline/guard executions are mandatory.

## Test-only trace and effort gate

`authorization.jsonl` is a version-1 event stream (`report.json.schema_version`).
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
| Fresh command | Translation/rotation response and quaternion mapping | Shared safety approval, finite/ceiling/slew | Resolve filter semantics and frames |
| Exact source age 100 ms / forward jump | Stamp ignored in responsive probe | Zero at boundary, original stamp retained | Source freshness through whole adapter |
| Publisher loss | Separate pose process killed; inspect retained effort | Receipt expires at 100 ms | Independent actuator watchdog |
| Approval process loss | No more authorize events; clock/steps continue | Receipt expires even with clock progress | Independently scheduled/process-isolated gate |
| Clock pause | Injected clock stops, steady advances | Zero at 100 ms despite renewed approvals | Real clock binding and watchdog |
| Resume | Native target remains cached | Fresh sequence plus reconfigure/reactivate | Validate physical recovery transition |
| Backward reset | No native episode contract | Cached intent cleared, new UUID required | Reset ownership and state synchronization |
| Replay / wrong episode | No native metadata | Clear intent and inhibit immediately | Preserve producer/episode contract |
| Future / invalid frame | Header not validated natively | Independently rejected | Bind world frame explicitly |
| Nonfinite input/output | Isolated native payload case | Immediate zero; finite recovery | Validate upstream numeric handling |
| Deactivate / shutdown | Capture unchanged effort buffer | Immediate zero outside slew | Implement actual stop/hold and confirm driver behavior |
| Gripper on holds | CRISP exposes joint effort only | Shared safety retains accepted aperture | Separate validated gripper control |

## Timing and qualification

Each mode runs 250, 500 and 1,000 Hz settings; each rate has three independent
home-state processes, each with 1,000 warmup and 10,000 measured updates. Reports
contain median/p95/p99/max and budget overruns. Per-update torque delta is
`100/rate` Nm. Updates run as a throughput benchmark, without realtime scheduling
or a claim that a physical loop meets its deadline. Nearby/singular states are
exercised separately in native numeric/lifecycle tests.

A conditional go requires lifecycle/interfaces, numeric pose response, all guarded
fault assertions and p99 strictly below 2 ms in every local 500 Hz run. Impedance
wins if both qualify; OSC only if impedance does not. Otherwise selection is
deferred with no-go. These rules do not establish hardware real-time performance.

Evidence under ignored `runs/`: report/provenance, resolved parameters for both
modes, authorization JSONL, application decisions, per-case raw/guarded traces,
process logs and benchmark summaries. Source revisions, generated model SHA-256,
compiler/CMake flags, installed packages, ROS/RMW and machine are recorded.

See [ADR 014](../adr/014-crisp-feasibility-and-controller-decision.md) and the
[committed assessment](../reports/crisp-feasibility.md) for the local decision,
UR driver source audit, upstream license discrepancies, and deferred work.
