# ADR 032: Opt-in numeric projection for future action queries

- Status: accepted
- Date: 2026-10-16
- Extends: ADR 012
- Supersedes: ADR 012 resume configuration immutability only for this explicit
  projection toggle after equivalence validation; all learning/data drift remains forbidden

## Context

Pinned LeRobot's future-action column lookup causes datasets 4.8.5 to decode
camera columns for every future target. A 100-action paired sample decoded 202
images although only its two current images enter the model. Four read-only
probes returned identical tensors after projecting the numeric action column,
reducing those reads from 0.248–0.369 s to 0.0069–0.0076 s. End-to-end throughput
and model equivalence need separate validation.

## Decision

Add explicit, default-false `project_action_columns` training configuration.
Inside the optional LeRobot adapter, project only future numeric action queries
through the framework's dataset column-selection API before formatting. Preserve
current images/state, target order, episode mapping, masks, padding, dtype and all
other query paths. Use a process-local factory hook and importable reader subclass
for spawned workers; restore the factory even on failure. Keep pinned upstream
source unchanged. Reject unexpected reader types rather than silently changing
semantics. Ordinary application/domain imports must remain framework-independent.

No dependency, model, loss, normalization, sampler, optimizer or safety change is
introduced. Historical configs and manifests resolve false. Existing active
experiments continue unchanged. Require exact tensor and seeded tiny-training
comparison plus spawned-worker tests before pilot use. Measure GPU throughput
separately; the read microbenchmark alone cannot establish training speedup.

## Consequences

The compatibility hook depends on the inspected pinned reader and needs real
integration coverage on dependency updates. The extra option is visible in
resolved configuration and resume evidence. It is a performance intervention,
not a new training representation or task-success claim. See the
[bounded validation plan](../plans/projected-action-loading.md).
