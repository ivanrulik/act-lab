# ADR 016: CRISP moving simulation ownership and hold

Status: Accepted for bounded simulation; hardware remains unapproved.
Date: 2026-10-09

## Context

ADR 015 qualifies a stationary Cartesian impedance configuration. Motion needs
exclusive actuator ownership, a dynamic hold and independent freshness checks.
The fixed scene/UR mapping has submillimetre model residual; applying it directly
to short-horizon safety targets amplified that residual into a 3.674 mm settled
error. A moving adapter cannot silently relax the existing 2 mm tolerance.

## Decision

Use separate DDS producer, application gateway, actual C++ controller manager
and Python physics-owner processes. Only the last integrates live MuJoCo state.
The gateway uses the existing inbox and shared Cartesian safety path against
scratch snapshots. An optional internal execution sink distinguishes approved
intent from hold after all application validation; public domain ports stay intact.

The physics owner checks source, receipt, clock and controller-output progress
leases at 100 ms, plus generation/episode/sequence identity. A separate wall
watchdog transfers ownership if gateway or controller work stops. Recovery needs
fresh intent and controller cleanup/configure/reactivation. The real CRISP plugin
runs unmodified through six effort interfaces at 500 Hz. Application commands
arrive at 50 Hz. The default scheduler uses deterministic tick barriers. Optional steady pacing
keeps the same owner guard running between requests and records budget overruns.
Neither scheduler establishes hardware realtime performance.

ENABLED disables all arm servos and limits task effort to +/-5 Nm and 100 Nm/s.
HOLD removes task effort and latches measured joints. A bounded position PD servo
integrates dynamics without freezing state, keeping the accepted gripper aperture.
Hold gains retain the scene position stiffness; damping is explicitly 50 Nm s/rad
for proximal joints and 10 for wrists. Total hold actuator force is bounded by
+/-150 Nm proximal and +/-28 Nm wrist. Original independently saturated velocity
servos caused instability once their otherwise unbounded braking contribution was
bounded. The dedicated hold owns one bounded actuator per joint instead.

Before accepting each effort tick, predict the resulting state and check joint,
workspace, contact, velocity and acceleration limits. Discard an unsafe predicted
step and integrate that tick under dynamic hold with the original velocity.
Prediction rollback is a simulator mechanism and supplies no hardware guarantee.
Hold transients remain measured evidence rather than being clipped by teleportation.

For moving control, supersede ADR 015's fixed mapping-only use with measured
residual correction. Keep its base Rz(pi) and 0.11 m tool offset. At each snapshot,
compute official UR tool FK and mapped scene FK. Require residual <=2 mm/.02 rad,
then send `T_UR_actual * inverse(T_mapped_current) * T_mapped_target` to CRISP.
Keep original command timestamps and metadata. This reconciles the two simulation
models locally; it is not a hardware calibration or a change to v1 ROS codecs.

Preserve scene body gravity compensation for arm, palm and fingers. CRISP gravity,
Coriolis and friction stay disabled. The 80 g cube retains normal gravity and
contact dynamics; held payload weight is not compensated by CRISP. Report actuator
force separately from MuJoCo body gravity compensation and contact forces.

## Consequences

ROS/CRISP is opt-in; local simulation and learning keep their existing adapters.
The optional image retains ROS numerical packages and adds MuJoCo 3.12.0,
controller manager and this simulator SystemInterface. No training dependencies,
host networking, devices or GPU are needed. Evidence and the safety matrix live
in the ROS simulation contract. Hardware, ROS recording and operational-space
motion remain deferred. The accompanying report records local moving qualification.
