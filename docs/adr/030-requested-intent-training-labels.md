# ADR 030: Explicit requested-intent labels for feedback learning

- Status: accepted
- Date: 2026-10-16
- Extends: ADR 011, ADR 012 and ADR 029
- Supersedes: applied-action labels only in the new opt-in requested mode

## Context

The application accepts Cartesian intent and applies a 100 ms response filter,
rate/acceleration limits, IK and collision checks. Its reported executed action
is a filtered intermediate target. Reusing that target as new intent applies the
response filter again. Causal alignment corrects observation timing but does not
correct this mismatch in action meaning.

A diagnostic oracle on seeds 21000–21019 held targets across the same two 50 Hz
safety ticks used by 25 Hz inference. Holding the first tick's applied target
failed all 20 episodes during descent, with no safety rejections. Holding the
expert's requested target completed 16/20; four IK failures remain explicit.
These are controller diagnostics, not learned-policy results or qualification.
The incomplete alignment-only training run was preempted at update 789; it has
no completed checkpoint and supplies no model-success comparison.

## Decision

Add explicit `recording convert --action-source requested`, requiring
`--action-alignment next-command`, converter **v4**. Read requested Cartesian
intent for accepted (`applied`/`limited`) commands. Disabled/rejected commands
retain an explicit executed disabled hold when available. Existing safety reports
encode holds as resulting state with no executed action: reconstruct a disabled
measured-pose target using the last accepted executed gripper aperture. Reject
missing aperture history; never copy rejected intent or contact-deflected fingers.
Check the selected label's world frame and original timestamp, then run numeric/quality
validation as well as the original applied-data quality check. Do not turn a
rejected request into an enabled teaching target.

Keep current input images, measured state, source times, raw hashes, episode
splits and causal pairing unchanged. Record source and alignment in lineage,
and use a distinct fingerprint namespace. Default applied-source v2/v3 data and
fingerprints remain reproducible. Changed data requires fresh training.

At inference, predicted intent continues through the existing safety path.
Do not change rates, response horizon, limits, freshness, IK or collisions to
compensate for learning errors. No new dependency or raw schema is introduced.

The existing ACT output is still eight pose/gripper scalars; it does not learn
an enable flag or a fault detector. Disabled sample metadata becomes hold
pose/aperture targets. Runtime inhibition remains owned by application safety.

## Consequences and limits

Requested targets are farther from measured state and express the interface the
policy actually commands. Their learnability and closed-loop success remain
experimental. The oracle's four IK failures also require reporting; this choice
does not qualify the 25 Hz expert or any learned model. The preempted v3 run
cannot isolate the effect of alignment in a trained-policy comparison.
