# Reproduce the paired ACT release

Use ACT Lab source with ADR 033 (the qualified release branch or its merged
successor). Docker Compose is authoritative. No ROS, CRISP or physical devices
are needed. The paired dataset and model are public; Hub login is unnecessary
for download. Publication receipts record immutable commit IDs. Historical
private-publication receipts still pin their original, now accessible commits.

## Download and verify

Save the dataset/model receipts from the source release report as
`runs/releases/paired-dataset-receipt.json` and
`runs/releases/paired-model-receipt.json`. Each receipt has `schema_version: 1`,
`kind` (`dataset` or `model`), `repo_id`, a full immutable `revision`, and
`public: false`. Use the reported commits, never substitute `main`.

```bash
docker compose --profile hub-release run --rm hub-release \
  act-lab release download --receipt runs/releases/paired-dataset-receipt.json \
  --output runs/releases/downloaded-paired-dataset
docker compose --profile hub-release run --rm hub-release \
  act-lab release download --receipt runs/releases/paired-model-receipt.json \
  --output runs/releases/downloaded-paired-model
```

Downloads verify the inventory and exact dataset/checkpoint identities. The
original training paths in frozen payloads are historical and need not exist.
`inference.json` supplies explicit execution settings, distinct from serialized
training defaults. Never drop temporal ensembling or accept the saved horizon
100 when reproducing the qualified recipe.

## Integration check

```bash
docker compose --profile gpu run --rm \
  -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute train-gpu \
  python scripts/check-paired-release.py \
  --dataset runs/releases/downloaded-paired-dataset \
  --model runs/releases/downloaded-paired-model --device cuda \
  --output runs/releases/paired-download-smoke
```

CUDA is explicit and must be available; it never falls back to CPU. For a CPU
integration check use `hub-release` instead of `train-gpu` and `--device cpu`;
CPU is not the measured GPU timing configuration. The script loads only the 30
frozen training episodes, checks tensor shapes, validates exact simulation
configuration against the qualification report, runs the model on two consumed
seeds and rechecks package hashes. This is an integration check, not an
independent newcomer trial or fresh policy evaluation.

For an explicit evaluator invocation use the model `payload` checkpoint,
`reproduction/ur5e_2f85_d405_paired.toml`, `--policy-hz 25`,
`--temporal-ensemble-coeff 0.01`. The adapter enforces fresh horizon-one inference
and model/camera compatibility. Preserve all safety limits and report failures.

## Replay

Open `examples/act-seed-23000.mcap` in Foxglove for a successful 10.16 s rerun;
`examples/act-seed-23003.mcap` shows a 15 s timeout. Add Image panels on
`/foxglove/camera/wrist` and `/foxglove/camera/policy`, disable annotation
timestamp synchronization, and play at 1×. Add a Raw Messages panel for
`/command`. The time axis is monotonic simulation time, so a viewer may label it
as Unix epoch time; it is not acquisition wall time. Original source MCAP and
Foxglove derivatives are distinct. The two included viewer derivatives preserve
source messages and add standard image topics; use canonical raw logs for
training validation. Policy demonstration reruns are not expert training data.

## Train anew

Use `configs/training/act-camera-pilot-projected.toml` with the downloaded
dataset's `payload` as `--dataset` and a new run path. Verify the dataset before
training. Projection changes numeric target loading performance, not values.
The released inference files exclude optimizer/RNG state and therefore cannot
resume the original run. Original training used staged 5k/10k/20k budgets and a
late projection toggle; a fresh 20k run is not claimed to be bit-identical.
