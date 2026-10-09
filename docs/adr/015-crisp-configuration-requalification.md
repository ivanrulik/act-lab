# ADR 015: CRISP configuration requalification

- Status: Accepted — conditional go for Cartesian impedance in PR 13
- Date: 2026-10-09
- Supersedes: ADR 014 controller-selection decision; retains its safety requirements
- Scope: stationary mock bench, corrected filters and explicit frame binding

## Context

ADR 014 correctly rejected the filter=1 configuration: target and state history
freeze in pinned CRISP. Its diagnostic responsive target was not a qualified
configuration. Official UR FK also uses a different base/tool convention from
ACT Lab's local scene. Advancing to moving simulation requires an explicit
configuration review, frame evidence and a repeated full qualification.

## Decision

Keep CRISP and UR sources unmodified at the revisions recorded in ADR 014.
Version the candidate in `configs/ros2/crisp-feasibility.json`: target pose 0.1,
state q/dq/q_ref 0.0, output torque 1.0. Target/state alpha weights previous
history; output uses reversed operands. Target 0.1 retains 10% history and is
the minimum accepted by pinned parameter validation. State 0.0 passes current
feedback; output 1.0 passes current torque. Preserve filter=1 as a failing-response
regression case. Do not rewrite the original assessment.

Map only bench-approved domain poses: `T_scene_tip = A * T_UR_tool0 * B`, with
A = Rz(pi), B = 110 mm translation along tool0 Z. Use its exact inverse for
controller targets, preserving original timestamps and authorization metadata.
ROS v1 codecs and shared application workspace/safety checks are unchanged.
Compare nine stationary local MJCF fixtures with mapped official UR FK and require
<=2 mm translation and <=0.02 rad rotation, the existing convergence tolerances.
Maximum observed translation residual is 1.168 mm; the two models are not identical
or physically calibrated. Check exact inverse round trips separately.

**Conditional go for PR 13, Cartesian impedance as the default.** Both modes pass
actual plugin lifecycle/interfaces, translation/orientation response, changed
position/velocity feedback, independent effort calculations, frame checks and all
65 guarded fault assertions. All three local 500 Hz p99 measurements are below
2 ms. The [requalification report](../reports/crisp-requalification.md) records
measurements. Impedance is selected by the predefined tie rule, not a demonstrated
tracking advantage. CI requires functional evidence but does not qualify latency.

## Consequences

PR 13 can be planned and implemented as moving simulation work. Before enabling
it, implement a production guard, independently scheduled watchdog, validated
stop/hold and controller-ownership/recovery transitions. Preserve accepted gripper
aperture and validate its separate actuator path. Validate tool/payload and moving
mass/damping/compensation semantics; measured stationary FK residuals cannot prove
trajectory equivalence. Native CRISP still ignores freshness/frame headers,
retains effort after publisher loss, and leaves the mock effort buffer on
deactivation. Nonfinite/nonunit inputs remain unsafe native cases. The independent
mock gate's immediate zero is a test assertion, not a physical hold.

Hardware, ROS recording and system-clock operation remain deferred. Resolve actual
UR effort units/interfaces, compensation, calibration, software versions and
exclusive ownership against the deployed stack before supervised hardware work.
The original pinned driver audit and inconsistent upstream license notices remain
in ADR 014 and its report. Learning and local simulation stay ROS-free.
