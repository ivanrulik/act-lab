# ADR 022: Revoke stepped motion before offline acquisition

- Status: accepted for stepped simulation; CI qualification pending
- Date: 2026-10-10
- Extends ADR 021; partially supersedes its command-to-acquisition handoff

## Context

PR 23 CI recorded four CRISP samples across 1.206 seconds of simulation time.
The independent owner correctly expired the gateway's authorization while the
gateway rendered RGB and waited for recorder ACKs. The owner then integrated its
watchdog hold between sparse recorded samples. The unchanged quality validator
rejected the resulting sample gap. Offline image/ACK duration must not silently
keep task effort authorized or determine the nominal sample cadence.

## Decision

For active stepped CRISP acquisition, each execute request includes an explicit
hold-after boundary. After its requested physics ticks, the owner atomically
revokes authorization and selects the existing bounded dynamic hold before
acknowledging completion. No physics tick, timestamp substitution, state
teleportation or fabricated sample is introduced at that boundary. The accepted
gripper target is retained. Rendering uses the completed immutable scratch state.

Owner snapshots expose the actual hold state and the pre-boundary execution mode
and reason separately. A completed enabled interval followed by acquisition hold
is successful execution; controller/transport faults remain faults. Fresh enabled
commands must recover/reactivate through the existing guard. No authorization is
renewed while rendering or waiting for ACKs. Paced acquisition remains rejected.
Non-recording watchdog behavior, including hold integration after gateway loss,
is unchanged. Real watchdog-generated gaps still fail the existing quality gate.

## Consequences and verification

Offline acquisition is an explicit sequence of motion intervals and revoked
authorization pauses. It cannot establish continuous tracking or physical safety.
An injected 250 ms I/O pause must leave measured state and accepted gripper
unchanged in the stepped scheduler; fresh recovery advances the next nominal
20 ms interval. Required recording tests also check real DDS/import validity.
Existing controller-loss and gateway-loss tests must continue to pass.
