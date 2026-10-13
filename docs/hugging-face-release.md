# Curated Hugging Face releases

Release publication is an explicit adapter step, separate from local training.
The first recipe selects the legacy educational two-camera dataset and its
evaluated ACT checkpoint. It is approximately 1.41 GB without optional videos
or raw examples. It does not publish the newer articulated-tool smoke model.

## Prepare and verify

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
retention, automatic upload during training, raw webcam publication, current
tool-model policy qualification and paid cloud jobs are outside this feature.
