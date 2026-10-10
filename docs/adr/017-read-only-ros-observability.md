# ADR 017: Read-only ROS observability

Status: Proposed, implementation under validation

## Context

ADR 016's moving demonstrator has independent physics ownership and watchdogs,
but its execution/lease evidence is difficult to inspect live. Viewers must not
introduce a second command producer or a blocking dependency in control.

## Decision

Add optional RViz2 and local Foxglove projections, additive versioned execution
telemetry/events and a separate observer process. Use a bounded primitive shared
slot with nonblocking owner offers and visible event/sample loss. Convert and
publish ROS messages only in the observer. Observation failure cannot change
motion authority; existing command validation, guard and dynamic hold remain.

Keep official UR model FK namespaced and separate from authoritative MuJoCo tip
TF, preserving ADR 016's mapping and residual checks. Preserve timestamps and
join raw output with its source episode/generation/tick. Report independent
observer freshness and lease clocks rather than treating reception as approval.

Select optional Jazzy apt viewer dependencies, pin the inspected Foxglove bridge
package, restrict capabilities/topics/assets and test actual protocol rejections.
Forward only a loopback WebSocket port; share localhost DDS namespace for RViz.
Separate the RViz image stage and leave learning/default local workflows ROS-free.

## Consequences

The stream is diagnostic and may drop data. A cached visual is not a safety
indicator; explicit health and connection state are required. Gripper geometry,
recording and viewer control are deferred. Foxglove application entitlement and
interactive UI validation are separate from open source bridge/protocol tests.
RViz remains an independent viewer. ADR 016 is extended, not superseded.

See the [contract and safety matrix](../contracts/ros2-observability.md) and
[implementation plan](../plans/ros2-observability.md).
