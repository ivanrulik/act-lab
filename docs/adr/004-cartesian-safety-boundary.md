# ADR 004: Application-owned Cartesian safety with adapter feasibility

- Status: accepted
- Date: 2026-08-31

## Context

Cartesian intent is shared by teleoperators and learned policies, while inverse
kinematics and contact prediction depend on a particular robot and simulator.
Expected runtime safety interventions need to be inspectable without changing
the stable `Robot.command(Action) -> RobotState` port or leaking MuJoCo values
into the domain.

## Decision

`SafeCartesianRobot` in the application layer owns validation order, workspace
and rate policy, hold behavior, and `CommandReport` outcomes. MuJoCo owns
damped-least-squares IK, joint-range metadata, and predicted-contact
classification. Candidate joint targets are bounded and contact-checked before
execution.

Disabled, stale, malformed, IK-infeasible, and collision-causing commands hold
the current joint/gripper positions and return state normally. The associated
report distinguishes the reason. Only programmer, configuration, or simulator
failures raise exceptions. No dependency is added: Python implements the
framework-neutral safety math, and the existing NumPy and MuJoCo dependencies
provide linear algebra, Jacobians, and contacts.

## Consequences

Every local policy and future device adapter uses one inspectable safety path,
while simulator-specific feasibility remains replaceable. A physical driver
must implement equivalent feasibility hooks and retains manufacturer safety as
defense in depth. Contact prediction checks candidate configurations, not swept
volumes; collision-aware path planning is intentionally deferred.
