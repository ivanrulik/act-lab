# ACT training

PR 8 trains only from a converter-version-2 dataset whose logical and finalized
storage fingerprints match and whose episode split is valid. Regenerate older
derived datasets from the immutable selection manifest; raw MCAP files do not
change.

## Run on CPU or NVIDIA GPU

Choose a new output directory. Existing runs are never overwritten.

```bash
docker compose --profile training run --rm train-cpu \
  act-lab train act --dataset data/lerobot/pick-place \
  --output runs/training/act-cpu --device cpu

docker compose --profile gpu run --rm train-gpu \
  act-lab train act --dataset data/lerobot/pick-place \
  --output runs/training/act-gpu --device cuda
```

The GPU service requests one NVIDIA device through Compose. `--device cuda` is
checked before run creation and fails if CUDA is unavailable; it never falls
back to CPU. The production recipe uses batch size 8 and a ResNet-18 ACT model.
CPU training is useful for correctness demonstrations but 100,000 updates can
take many hours or days. GPU time and memory depend strongly on image size and
camera count. Reduce settings only in an explicit alternate TOML used for smoke
tests; the checked-in production configuration preserves upstream defaults.

Each run contains `act_lab_run.json`, append-only `metrics.jsonl`, `train.log`,
and `lerobot/checkpoints/`. The manifest records Git state, package/CUDA identity,
resolved settings, frozen train and reserved episode IDs, fingerprints, status,
and failures. Held-out validation episodes are recorded but not loaded in PR 8.

## Resume and W&B

Resume uses the last complete LeRobot checkpoint and restores policy, optimizer,
scheduler, RNG, and data order:

```bash
docker compose --profile gpu run --rm train-gpu \
  act-lab train resume --run runs/training/act-gpu --device cuda

# Extend a completed target; TOTAL must exceed the original target.
docker compose --profile gpu run --rm train-gpu \
  act-lab train resume --run runs/training/act-gpu --device cuda --steps 120000
```

Dataset identity, device type, and batch size cannot change. Local artifacts are
authoritative. W&B is disabled by default; choose `--wandb-mode offline` or
`online` on the initial command to mirror metrics. Online mode expects credentials
already configured outside the run and the manifest never stores secrets.

Launch the thin teaching notebook with
`docker compose --profile notebook up notebook`. It inspects the same lineage and
configuration APIs, optionally invokes the production CLI, and plots local
metrics. Training loss is an offline optimization diagnostic, not evidence of
safe closed-loop performance. Seeded held-out rollouts and confidence intervals
belong to PR 9.
