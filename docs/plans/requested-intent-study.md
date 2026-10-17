# Causal requested-intent experiment

Status: completed 5,000 CUDA updates; diagnostic gate failed (0/20).
No fresh final cohort was consumed.
Protocol was predeclared before implementation/training. Extends ADR 030.

The alignment-only run was interrupted at update 789 after the oracle found a
representation mismatch: filtered executed targets are filtered again when used
as application intent. No completed alignment-only model exists, so this trial
cannot attribute improvement separately to timing versus requested labels.

1. Reuse the original 40 raw episodes, exact 30/10 split, cameras wrist + policy,
   and 25 Hz grid. Convert v4 causal requested labels into a separate run root.
   Verify raw identities, current state/RGB and next accepted requested intent;
   preserve explicit/reconstructed disabled holds and accepted aperture. Reject
   missing aperture history, bad frame,
   timestamp and numeric payloads. Preserve v2/v3 defaults and fingerprints.
2. Fresh CUDA training with the original ACT pilot configuration: seed 1000,
   batch 8, 5,000 updates, final save at 5,000. No changed-data resume, safety,
   model, optics, rate or loss changes. Keep the 5 GiB experiment soft gate.
3. Descriptive first-action errors against requested labels and the zero-motion
   baseline. Report separate train/reserved samples and all run provenance.
4. Evaluate 20 diagnostic seeds 21000–21019 with actual horizon one and fresh
   predictions at 25 Hz. Report events, individual failures, geometry and latency.
   At least 18/20 successes advances; otherwise diagnose another bounded factor.
5. Only a frozen passing candidate may use a verified unused final cohort:
   candidate 23000–23099, target >=90/100 with Wilson intervals. Do not reuse
   consumed 22000–22099 as fresh evidence. Retain failed and interrupted evidence.

The oracle gave 16/20 for requested holds, with four IK failures. This is a
separate known risk, not a learned-policy ceiling or permission to relax safety.

## Verified data

- Logical fingerprint: `49e3f6f6fa7f016bef373cf4050b88f8b6c81905106332cdde70813291a2be04`.
- Storage fingerprint: `baace059af172bc39a3df8c4a06d7066493dc61c82681a5b845da07860ce6d52`.
- All 11,624 current states and 23,248 encoded images match the applied v3
  reference. Every next requested/hold label matches immutable raw evidence.
- Original raw hashes and exact 30/10 episode splits match. No source copying,
  camera changes, safety changes or final-seed use occurred.
