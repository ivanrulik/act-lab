# ADR 029: Explicit causal action alignment for derived learning data

- Status: accepted
- Date: 2026-10-16
- Extends: ADR 011 and ADR 012
- Supersedes: same-sample action labeling for the new causal conversion mode only

## Context

MCAP correctly records post-command observations, synchronized images and the
command report producing that state. Converter v2 used that same applied command
as the observation's supervised action. It therefore paired an action with its
result, rather than the input state available before it. In the paired pilot,
copying the observed pose nearly perfectly fits XYZ labels while never moving.
This is a confirmed timing mismatch; its contribution to learned task failure
still needs a controlled retraining experiment.

## Decision

Keep immutable raw acquisition and existing derived artifacts unchanged. Add
explicit `recording convert --action-alignment next-command`, converter **v3**.
Within each episode, pair observation/image/state at raw sample i with the action
fields of raw sample i+1: position, WXYZ quaternion, gripper, enable and outcome.
The final observation has no following label and is dropped. Require at least
two samples and strictly positive adjacent intervals below 100 ms. Shift before
resampling; a 50 Hz source remains a next-20-ms command label even when derived
frames are 25 Hz. This is a supervised target, not a future policy input.

Only action fields move. Preserve observation timestamps, camera RGB, measured
state, source contracts, selected raw episode identities and episode splits.
Use a v3 fingerprint namespace even when numeric values happen to match. Record
`action_alignment=next-command` in lineage. Existing resume/dataset bindings
reject the changed fingerprint. No domain action, inference or safety changes.

Default `same-sample` retains v2 numeric/fingerprint behavior for historical
reproduction. New causal experiments must request the explicit option. Do not
silently reconvert the frozen study or continue its checkpoint against v3 data.

## Consequences and limits

An aligned dataset needs a separately frozen training experiment and untouched
final evaluation seeds. A few millimeters of prospective target motion do not
remove the need for accurate first-action learning. Causal pairing alone is not
a demonstrated fix for the zero-success checkpoint. No longer training run,
hyperparameter sweep, raw deletion or publication is authorized by conversion.
