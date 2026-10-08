# ADR 013: Isolated ROS 2 executable contracts with simulation clocks

- Status: accepted
- Date: 2026-10-07

## Context

The local acquisition-to-ACT loop is established. Distributed command transport
needs executable interfaces and independent loss guards before a moving ROS
adapter can be assessed. ROS time can pause, jump backward, or start at zero;
transport lifespan alone cannot guarantee application freshness.

## Decision

Extend ADR 003's optional adapter boundary with generated v1 interfaces, primitive
bidirectional codecs, and a ROS-independent latest-command inbox. Preserve domain
ports and `SafeCartesianRobot` as the sole application safety execution path.
Callbacks store intent; the control thread owns driver access.

Support simulation time first: ROS time is domain time plus one second,
`use_sim_time=true` is mandatory, and one trusted clock/state authority owns
an episode UUID that changes on reset. One producer supplies strictly increasing
uint64 command sequences. Commands do not create or change episodes. Combine
source-time freshness with injected steady receipt/progress watchdogs at 100 ms.
Clear intent on transport faults, pause, and reset. A backward jump requires a
new UUID; resume requires fresh intent. Preserve accepted gripper aperture during
shared safety holds. See the full [contract and safety review matrix](../contracts/ros2.md).

Build/run generated messages in a pinned Jazzy/Noble Python 3.12 image through
an opt-in `ros2-contracts` Compose profile, with Fast DDS and localhost-only discovery.
Retain Ubuntu's ROS-compatible Python numerical packages; source imports avoid
installing simulation/training dependencies. Use generated serialization tests
and an actual two-process DDS fake-driver demonstration in a dedicated required
CI job. Ordinary CI remains ROS-free.

## Consequences

We can test wire conversions, transport faults, and clock guards reproducibly
before connecting ROS to motion. This adds build tools and ROS packages only to
the isolated image. Application/domain tests require neither generated messages
nor ROS. QoS complements safety; it does not replace it.

The clock offset is versioned wire behavior, not a wall-clock synchronization
scheme. UUID/sequence checks do not authenticate producers. The demonstrator
validates no physical robot, MuJoCo motion through ROS, camera, or GPU behavior.
CRISP feasibility, external clock/state binding, moving adapters, ROS recording,
system-time synchronization, arbitration, and hardware remain explicit roadmap
work. Driver-level watchdogs and supervised hardware safety review are still
required before physical integration.
