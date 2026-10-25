# Public paired ACT showcase

Date: 2026-10-25. The user authorized making the qualified paired model and
dataset public as an inspectable robotics project showcase. The legacy
educational repositories remain outside this change.

## Public artifacts

| Artifact | Public presentation commit |
| --- | --- |
| [Dataset](https://huggingface.co/datasets/ivanrulik/act-lab-ur5e-paired-pick-place-v1) | `9b1e1286aa4cc6047e9dab5a9112fc23b2d644b3` |
| [Model and video](https://huggingface.co/ivanrulik/act-lab-ur5e-paired-act-v1) | `71f495f007ce142fc3be9845deb3595430f325b7` |

Separate `configs/releases/paired-act-v1-*-public-receipt.json` files preserve
the preceding commits. Original private-publication receipts remain unchanged;
their original commits are now anonymously accessible too. The model's initial
public edition was `fa9f0f4ae004e8922c1485339e32d5afe85c7fce`; the final card
also supplies an explicit widget video source. Every edition preserves original
weights, data, evidence, inference settings, examples and license notices.

## Presentation

Both cards lead with the measured result and scope limits, link GitHub, model,
dataset and evaluation evidence, and credit LeRobot ACT, MuJoCo and upstream
assets. ACT Lab's contribution is the reproducible integration, causal labeling,
safety path, experiments and diagnosis. No algorithm novelty, hardware result,
broader generalization or new held-out evaluation is claimed.

The new MP4 uses the original consumed-seed 23000 recording: side-by-side wrist
and fixed RGB, sampled from 50 Hz at 25 Hz, **254 frames / 10.16 seconds** at 1x
simulation time. It is H.264, 640×336, no audio, with readable labels identifying
the policy, seed, simulation clock and demonstration scope. A poster shows an
actual grasp frame. The original MCAP is retained unchanged, including the
separate failed episode; the MP4 is only a lossy presentation derivative.
Preview hashes and source identity are in
[the metrics](public-paired-showcase-metrics.json).

## Publication checks

- Presentation packages verify against their original pinned baseline. Only
  README/reproduction instructions and preview media differ. Payload identities,
  30/10 split, all original evidence and qualified inference settings are intact.
- `release promote` commits with an expected parent SHA, verifies inventory
  before visibility changes, and writes separate public receipts. Tests reject
  changed weights, identity, removals, arbitrary additions, excessive media and
  stale remote heads. No dependency or domain/control change was introduced.
- Model page, dataset page and MP4 returned HTTP 200 without authentication.
  Actual anonymous snapshots of both initial public editions passed every
  inventory checksum. The final model-card-only update was downloaded and all
  package hashes reverified; unchanged payloads were retained from those fresh
  anonymous snapshots. Original commits were anonymously accessible.
- Browser inspection confirmed the card's result, upstream attribution and
  playable video: loaded duration **10.16 s**, ready state 4, no media error.
  The Hub's separate automatic Robotics sidebar still reports no preview for
  this nested inference package; the embedded card player and direct MP4 work.
- Required dev build, Ruff, mypy (82 source files), doctor and full pytest passed:
  **453 passed, 14 optional-runtime/display skips**, with one existing fork
  deprecation warning. Focused release tests: **23 passed**.

This is public artifact/presentation validation, not another training campaign,
hardware test, cold-environment GPU qualification or independent newcomer trial.
