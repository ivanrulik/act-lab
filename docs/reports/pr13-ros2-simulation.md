# PR 13 moving CRISP simulation evidence

Local qualification: 2026-10-09, Jazzy/Noble Python 3.12, Fast DDS localhost,
MuJoCo 3.12.0, unmodified CRISP `0279dc8`, UR description `6b639c2`.
This is simulator evidence; hardware is not approved.

## Outcome

The generated v1 command -> shared safety -> actual controller manager/CRISP ->
MuJoCo effort path produces motion through six canonical effort interfaces.
A competing CRISP instance is rejected by actual controller-manager ownership.
The independent physics process installs dynamic hold on loss/fault and keeps
integrating after gateway SIGKILL or controller stall/death. Fresh intent plus
controller reactivation recovers; reset also clears application limiter history.

The fixed local seed 0 sweep executes +/-10 mm in all axes around home and two
nearby Cartesian fixtures, with a 0.01 rad orientation trial for each fixture.
The existing local driver receives matched intents and records measured
trajectories/outcomes. Settled translation errors are below 1.00 mm (required
2 mm). The initial same-seed stepped repeat gave zero maximum position difference.
During merge validation, a wall watchdog during lifecycle recovery inserted hold
ticks and gave 0.056 mm difference. Repeat reports now distinguish bitwise equality
from the existing 2 mm physical comparison tolerance and list held commands.
Identical seeds alone do not imply identical wall-clock fault inputs.

CRISP also drives the scripted grasp/lift over DDS: the cube rises 51.13 mm;
loaded fault hold drifts 0.186 mm over 0.5 s, retains aperture/contact, then
resumes on fresh intent. The cube retains normal gravity; arm/palm/finger body
compensation remains unchanged, and CRISP compensation is off.

Across the moving/loaded trace, measured maxima were:

| Quantity | Measured | Existing limit |
|---|---:|---:|
| Cartesian speed | 0.0292 m/s | 0.25 m/s |
| Cartesian acceleration | 0.820 m/s² | 1.0 m/s² |
| Angular speed | 0.0248 rad/s | 1.0 rad/s |
| Angular acceleration | 1.152 rad/s² | 4.0 rad/s² |
| Joint speed | 0.151 rad/s | 1.0 rad/s |
| Joint acceleration | 3.545 rad/s² | 4.0 rad/s² |
| Task actuator effort | 1.019 Nm | 5 Nm |
| Total actuator effort | 1.228 Nm | 150/28 Nm |

Values above are rounded upward. Actuator force excludes body compensation and
contact forces; it must not be presented as physical UR torque qualification.
Full reports preserve configuration, model hashes, compiler flags, dependency
manifests, process IDs, raw/gated/hold effort and individual faults under `runs/`.

## Findings resolved during implementation

- Fixed mapping-only short-horizon targets amplified model residual, giving
  3.674 mm settled error. ADR 016 adds bounded measured residual correction.
- Controller-manager list handoffs need concurrent updates during lifecycle
  operations; bounded asynchronous lifecycle calls prevent the initial deadlock.
- DDS acknowledgments and serialized callback/update execution remove target
  delivery races. Episode reset must clear application motion history as well as
  the transport inbox, owner cache and controller state.
- Bounding the original velocity servos exposed unstable braking. Dedicated
  bounded PD hold uses 50/10 Nm s/rad damping. An earlier 100/20 damping setting
  exceeded loaded-stop acceleration limits (1.201 m/s², 5.506 rad/s²); the final
  setting satisfies the existing limits without relaxing drift/retention gates.

## Validation and handoff

Required dev build, Ruff, mypy (57 source files), ordinary pytest and doctor pass.
Ordinary pytest: 253 passed, 3 explicit optional-environment skips. Dedicated
CPU training tests: 9 passed; teaching notebook executed. ROS contracts: 3
passed; CRISP stationary tests: 5 passed; ROS simulation: 5 passed. The ROS job
requires motion/fault, interface/configuration, gateway SIGKILL, controller stall
and paced-owner lease tests; missing packages fail execution. Existing ROS
contract and CRISP stationary suites pass. CPU dataset/training checks and the
teaching notebook pass. Default headless simulation and import isolation pass.

The dedicated [safety review matrix](../contracts/ros2-simulation.md) accompanies
this evidence. Keep the PR draft until its CI gates pass and review is complete.
ROS MCAP recording/conversion equivalence is next roadmap work. Hardware stop,
payload/dynamics calibration, system-clock synchronization and operational-space
moving qualification remain separate decisions. Near-singular moving trajectories
are not qualified by these home/nearby fixtures. A separate rank-deficient wrist fixture verifies unreachable IK
rejection without changing live state, followed by dynamic hold; joint-margin
crossing is independently rejected by the effort predictor. These rejection
tests do not qualify motion through a singularity.
