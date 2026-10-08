# ADR 014: CRISP feasibility and controller decision

- Status: Accepted — no-go for the assessed configuration; controller selection deferred
- Date: 2026-10-08
- Scope: roadmap PR 12, stationary mock effort bench

## Context

PR 11 establishes optional ROS executable contracts. Before moving MuJoCo through
ROS, ACT Lab needs evidence about controller interfaces, pose semantics, safety
gaps, compensation and update cost. CRISP offers Cartesian impedance and
operational-space control in one plugin. Neither its origin on other robots nor
stationary finite effort establishes UR5e motion or hardware safety.

## Decision

Assess unmodified CRISP revision `0279dc8f1196dab6a2b2cd5af853dd3d4dc3a969`
against the pinned official UR model with actual lifecycle/plugin/DDS code and
loaned mock interfaces. Keep a separately tested effort gate exclusively in the
bench. Preserve local adapters, domain ports, ROS v1 messages and learning paths.

**No-go for the assessed filter=1 baseline in both modes.** Native numeric pose
response fails: target translation/orientation history is frozen at 1.0. The
specified assumption that 1.0 disables smoothing is false for this revision.
Finite output, successful guard assertions and fast updates cannot make a
nonresponsive pose configuration suitable for PR 13. The separate filter=0.1
probe diagnoses native stale/publisher-loss behavior and is not silently promoted
to a qualifying candidate. No alternative controller is introduced.

The [assessment](../reports/crisp-feasibility.md) records local measurements and
source findings. The [bench contract and safety matrix](../contracts/crisp-feasibility.md)
define exact boundaries and the reproducible protocol. Completed no-go is a
successful assessment; missing dependencies or incomplete evidence fail.

Future qualification requires lifecycle/interface/numeric pose tests and every
guarded fault case to pass, plus all three local 500 Hz benchmark p99 values
strictly below 2 ms. Select Cartesian impedance if both qualify; select
operational-space only if it alone qualifies. CI collects timing without making
a local qualification claim. Stationary evidence cannot compare motion tracking.

## Consequences and deferred work

PR 13 moving simulation work is blocked until a separately reviewed follow-up
resolves filter semantics and reruns qualification on an explicit configuration.
It must also reconcile the official-model `world` FK with the local scene frame
and workspace, validate moving dynamics/compensation, and implement a production
guard with independently scheduled watchdog and a validated stop/hold mechanism.
Fault recovery must be validated through real controller ownership transitions.
Zero mock effort provides no physical hold evidence.

Hardware remains later work: verify the deployed UR driver and client-library
versions, exported effort interfaces/units, calibration, software gates,
controller exclusivity, robot-internal gravity/friction compensation, gripper
control and supervised commissioning. Pinned implementation checks are not
hardware compatibility certification; rolling documentation has different
software gates. ROS recording, CRISP Python/Gym integration and learning through
ROS are deferred.

Upstream license metadata is inconsistent: retain its MIT license file,
Apache 2.0 package declaration and source notices; do not relabel them as a
single consistent license. The bench image retains the original source tree
and records these notices in provenance. Dependency rationale is in the contract.
