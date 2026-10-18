# ADR 033: Qualified paired ACT release

- Status: accepted
- Date: 2026-10-17
- Extends: ADR 027, ADR 028, ADR 030, ADR 031 and ADR 032

## Context

The articulated wrist/fixed policy has passed its frozen simulation gate, but
the original private Hub release targets the legacy model. Training defaults
(100 queued actions, no ensemble) differ from qualified inference (fresh horizon
one, ensemble 0.01). Publishing weights without that distinction is misleading.

## Decision

Publish a separate, explicitly private paired dataset/model release with the
existing staged inventory and immutable download workflow. Keep all original
payload bytes and episode splits. Add optional, checksummed `inference.json`
bound to the primary evaluation and supplied simulation config. Check the model
and dataset camera contract, training completion and episode indices at stage.
Historical packages retain schema-1 compatibility without inference metadata.

Retain both UR5e and Robotiq source notices, reproduction config/instructions,
resume provenance and full final per-episode evidence. Include one success and
one timeout Foxglove demonstration in the model repository only. They are new
reruns on consumed seeds, never substitutes for original qualification traces.
Keep raw acquisition, optimizer state and intermediate checkpoints local.

Run a clean-source, commit-pinned download/load/evaluation check using only
downloaded payloads and explicit inference settings. Recheck hashes afterward.
No new dependency, domain type, ROS requirement or automatic upload is added.

## Consequences

The combined target inventory is below 1.1 GB (decimal); receipts pin independent
dataset/model Hub commits. Verification establishes reproducible artifact use,
not fresh statistical evaluation, bit-identical retraining or hardware safety.
Public visibility, independent newcomer acceptance, broader generalization and
learned ROS motion remain separate work. Existing legacy repositories remain
immutable and clearly labeled historical.
