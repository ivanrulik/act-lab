# Curated Hugging Face releases

Release publication is an explicit adapter step, separate from local training.
The current paired recipe packages the simulation-qualified articulated-tool
dataset/model, explicit inference settings and two Foxglove demonstration
reruns. Combined inventory: approximately **1.015 GB**, with 40 episodes (30 train,
10 reserved), one inference checkpoint and no optimizer/raw acquisition archive.
See [paired reproduction](releases/paired-reproduction.md),
[dataset card](releases/paired-dataset-card.md),
[model card](releases/paired-model-card.md) and ADR 033.

The historical educational recipe remains distinct and immutable: approximately
1.41 GB, legacy robot/two-camera configuration and no replay examples. Its
later action-horizon audit remains attached; do not treat it as the paired
qualification or use its execution settings for the new model.

## Qualified paired recipe

```bash
docker compose --profile hub-release run --rm hub-release \
  act-lab release stage --config configs/releases/paired-act-v1.toml \
  --output runs/releases/paired-v1
docker compose --profile hub-release run --rm hub-release \
  act-lab release verify --package runs/releases/paired-v1/dataset
docker compose --profile hub-release run --rm hub-release \
  act-lab release verify --package runs/releases/paired-v1/model
```

Publish/download using the commands below with the paired package paths. New
repositories are `ivanrulik/act-lab-ur5e-paired-pick-place-v1` (dataset) and
`ivanrulik/act-lab-ur5e-paired-act-v1` (model), private by default. Immutable
commits and actual clean-download evidence are recorded in the
[paired release report](reports/paired-act-release.md).

`inference.json` is checked against the primary evaluation and checkpoint;
dataset packages also check 25 Hz FPS. Staging checks completed training,
model/camera contracts and the exact frozen split. Run
`scripts/check-paired-release.py` on downloaded packages: it checks tensor
shapes, exact simulation settings and two consumed-seed reruns, then rechecks
hashes. It does not expand the 100-task qualification cohort.

The packaged success/timeout replays contain the policy's actual wrist/fixed
inputs, state and safety reports. They are new demonstration reruns rather than
original evaluation videos. Do not include them in expert training selection.
Training and inference remain ROS-free. The Hub SDK is the existing pinned
optional dependency; this increment adds none.

## Prepare and verify

[First-publication evidence](reports/hugging-face-release.md) records actual
private Hub commits, fresh download verification and clean-source CPU smoke.

Build the pinned ROS-free training runtime once:

```bash
docker compose --profile training build train-cpu
docker compose --profile hub-release run --rm hub-release \
  act-lab release stage --config configs/releases/educational-act-v1.toml \
  --output runs/releases/educational-v1
docker compose --profile hub-release run --rm hub-release \
  act-lab release verify --package runs/releases/educational-v1/dataset
docker compose --profile hub-release run --rm hub-release \
  act-lab release verify --package runs/releases/educational-v1/model
```

The recipe requires the selected local artifact paths. A fresh source clone
cannot stage those artifacts until downloaded or regenerated. Staging verifies
the dataset bytes/splits and checkpoint/evaluation identity before copying.
Existing outputs are refused. Payload files retain their original hashes.
Cards, asset notices, public sidecars and per-file checksums remain outside
`payload/`. Historical absolute paths are redacted only in copied sidecars.
The checkpoint retains historical `/workspace` settings; pass explicit local
paths and `--policy-hz 25` when evaluating.

## Authenticate and publish

Use `hf auth login` on the host. Compose mounts only the configured Hub auth
directory read-only, or accepts `HF_TOKEN` from the environment. Never commit
tokens or place them in command arguments. Override the host directory with
`ACT_LAB_HF_HOME` if needed. Refresh OAuth login on the host before publishing.
The existing pinned `huggingface-hub==1.6.0` dependency is used only in the
optional service; no new dependency enters ordinary application/domain imports.

```bash
docker compose --profile hub-release run --rm hub-release \
  act-lab release publish --package runs/releases/educational-v1/dataset
docker compose --profile hub-release run --rm hub-release \
  act-lab release publish --package runs/releases/educational-v1/model
```

Repositories are private unless `--public` is explicitly supplied. Repository
IDs come from the reviewed TOML recipe. Existing visibility mismatch, unexpected
remote files or a different release manifest are rejected. Re-running an
identical package is supported; a changed release needs a new repository name.
Only checksummed inventory files are uploaded. Receipts are saved outside the
packages, with immutable Hub commit IDs; neither upload modifies local source
data nor deletes local files. A dataset and model upload are separate commits;
if one fails, retain its successful receipt and retry the failed package.

## Download and qualify

```bash
docker compose --profile hub-release run --rm hub-release \
  act-lab release download --receipt runs/releases/educational-v1/dataset-receipt.json \
  --output runs/releases/downloaded-dataset
docker compose --profile hub-release run --rm hub-release \
  act-lab release download --receipt runs/releases/educational-v1/model-receipt.json \
  --output runs/releases/downloaded-model
docker compose --profile hub-release run --rm hub-release \
  act-lab sim evaluate --checkpoint runs/releases/downloaded-model/payload \
  --config configs/sim/ur5e_pick_place.toml --policy-hz 25 --device cpu \
  --seed-start 11000 --episodes 1 --output runs/releases/download-smoke
```

Download pins the receipt commit and verifies every payload/card/evidence file
plus dataset/checkpoint identity. A new directory is required. The one-seed
smoke checks current integration; it is not a fresh full-policy evaluation.
The original 96/100 result and later 97/100 regression remain attached to their
recorded configurations. Later solver results reuse already inspected seeds.
Historical training recorded dirty Git state; bit-identical retraining is not
claimed. A separate newcomer trial remains PR 10 acceptance work.

Keep local copies until remote verification and loading succeed. Bucket archive
retention, automatic upload during training, raw webcam publication and paid
cloud jobs are outside this feature. Current tool-model paired simulation
qualification is documented separately from hardware and broader generalization.
