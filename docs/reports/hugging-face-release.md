# Lean educational ACT release — first publication

Date: 2026-10-13. The dataset and inference model were published **privately**
under `ivanrulik`. Public visibility remains a separate user choice.

| Artifact | Immutable Hub commit |
| --- | --- |
| `ivanrulik/act-lab-ur5e-educational-pick-place-v1` (dataset) | `56c3ec20690e2cdf8ddda19bfbb30f5460efea0a` |
| `ivanrulik/act-lab-ur5e-educational-act-v1` (model) | `eee24515167a73b56baf8cc3501e6ccca0aeba73` |

The combined release is approximately 1.41 GB: a 1,202,209,390-byte validated
dataset and 206,738,257-byte inference checkpoint, plus cards, licenses,
checksums and compact evidence. Raw MCAP examples and videos are not included
in this first upload. No local acquisition archive or checkpoint was deleted.

## Validation performed

- Required dev build passed; Ruff, mypy (79 source files), doctor and Compose
  configuration passed.
- Full dev suite: 400 passed, four expected optional-runtime/display skips.
- Final focused release suite: nine passed, including drift/symlink/missing-file
  rejection, checkpoint/evaluation mismatch, public-sidecar path redaction,
  inventory-only private upload, lazy Hub/ROS imports and immutable downloads.
- Both original dataset finalized-byte fingerprint and evaluated checkpoint
  digest matched. Staging additionally binds dataset identity to training lineage.
- Actual authenticated uploads used the pinned Compose Hub SDK, with private
  visibility and explicit inventories. No token was copied into the artifacts.
- Fresh downloads pinned the exact commits above. Every published file and
  both payload identities verified.
- LeRobot 0.6.1 loaded the downloaded dataset using only its 58 frozen training
  episodes. State/action dimensions and both 240×320 RGB features passed.
- A clean source export of commit `f7d5c44b75697206dfbdf5f8b48a8b3891427b65`
  ran the downloaded model through the existing evaluator on CPU, with an
  explicit 25 Hz policy cadence and the educational preset. ACT and the expert
  both completed seed 11000. The separate pre-upload staged smoke also passed.

Ignored local receipts, download logs and smoke report are under
`runs/releases/`. The source export uses the existing pinned training image;
it is not a new cold image build or an independent newcomer acceptance trial.
LeRobot emitted its historical CUDA-config warning while constructing the
checkpoint. CPU was explicitly requested and the actual resolved policy device
and evaluation report were CPU; no requested GPU run fell back.

## Interpretation and remaining work

This verifies the first upload/download/load workflow, not a new 100-seed
evaluation or bit-identical retraining. Original untouched holdout results are
96/100; later 97/100 results reuse inspected seeds under revised solver settings.
Training's original dirty Git state remains disclosed in the cards and evidence.
This legacy two-camera policy does not qualify the articulated 2F85/wrist model,
physical hardware, other tasks or generalization outside the recorded bounds.

Independent newcomer completion, public visibility, optional demonstration
examples, current-tool policy training/evaluation and archive bucket retention
remain follow-ups. Training continues to work locally without ROS or Hub auth.

## Action-horizon interpretation audit

The later [paired-pilot diagnosis](paired-camera-diagnosis.md) found LeRobot
0.6.1 ignored the adapter's one-step loading kwarg. The saved legacy release
config also has horizon 100. Historical success counts remain recorded
observations, but a claimed one-step fresh-feedback interpretation requires
audit of the original runtime. This diagnosis did not replay or requalify the
legacy model, and does not alter immutable published artifacts.
