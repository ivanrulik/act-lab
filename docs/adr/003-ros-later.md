# ADR 003: ROS 2 is an optional adapter added after the learning loop

- Status: accepted
- Date: 2026-08-29

## Context

ROS 2 is valuable for distributed robotics, standard tools, and physical UR
integration, but it adds DDS, networking, packaging, and clock complexity to an
initially local simulation pipeline.

## Decision

Keep domain contracts independent of ROS and deliver the local MuJoCo-to-ACT
loop first. Add ROS 2 through adapters when a distributed or hardware use case
exists. Training and dataset workflows remain ROS-independent.

## Consequences

Early learning work has a smaller dependency surface. Later ROS integration
must translate messages and prove behavioral equivalence with local adapters.

