#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"
cd "${repo_root}"

usage() {
  cat <<'EOF'
Validate PR 8 without remembering the underlying Docker commands.

Usage:
  ./scripts/validate-training.sh check
  ./scripts/validate-training.sh cuda
  ./scripts/validate-training.sh smoke DATASET [cpu|cuda]

Commands:
  check   Run the focused unit/integration tests and execute the notebook.
  cuda    Build the GPU image and prove PyTorch can use the reserved GPU.
  smoke   Train for 2 updates, checkpoint, resume to update 3, and summarize.

Example:
  ./scripts/validate-training.sh smoke data/lerobot/pick-place cpu
EOF
}

run_check() {
  docker compose --profile training build train-cpu
  docker compose --profile training run --rm train-cpu pytest \
    tests/unit/test_training.py \
    tests/unit/test_dataset_pipeline.py \
    tests/integration/test_act_training.py
  docker compose --profile training run --rm train-cpu \
    jupyter nbconvert --to notebook --execute \
    --ExecutePreprocessor.timeout=600 \
    --output /tmp/act_training.executed.ipynb \
    notebooks/act_training.ipynb
  echo "PASS: ACT conversion, training, checkpoint/resume, and notebook checks passed."
}

run_cuda() {
  docker compose --profile gpu build train-gpu
  docker compose --profile gpu run --rm train-gpu python -c \
    'import torch; assert torch.cuda.is_available(), "CUDA is not usable by PyTorch"; print(f"PASS: {torch.cuda.get_device_name(0)} via CUDA {torch.version.cuda}")'
}

run_smoke() {
  local dataset="${1:-}"
  local device="${2:-cpu}"
  if [[ -z "${dataset}" ]]; then
    echo "error: smoke requires a converted dataset path" >&2
    usage >&2
    exit 2
  fi
  if [[ ! -d "${dataset}" ]]; then
    echo "error: dataset directory does not exist: ${dataset}" >&2
    exit 2
  fi
  if [[ "${device}" != "cpu" && "${device}" != "cuda" ]]; then
    echo "error: device must be cpu or cuda" >&2
    exit 2
  fi

  local profile="training"
  local service="train-cpu"
  if [[ "${device}" == "cuda" ]]; then
    profile="gpu"
    service="train-gpu"
  fi

  local smoke_config
  smoke_config="$(mktemp --suffix=.toml)"
  trap "rm -f -- '${smoke_config}'" EXIT
  cat >"${smoke_config}" <<'EOF'
[act]
version = 1
seed = 1000
batch_size = 2
steps = 2
log_freq = 1
save_freq = 1
num_workers = 0
chunk_size = 2
n_action_steps = 2
vision_backbone = "resnet18"
pretrained_backbone_weights = ""
dim_model = 32
n_heads = 4
dim_feedforward = 64
n_encoder_layers = 1
n_decoder_layers = 1
use_vae = true
latent_dim = 4
n_vae_encoder_layers = 1
dropout = 0.0
kl_weight = 1.0
temporal_ensembling = false
use_amp = false
cudnn_deterministic = true
EOF

  local run="runs/training/validation-${device}-$(date -u +%Y%m%dT%H%M%SZ)"
  docker compose --profile "${profile}" build "${service}"
  docker compose --profile "${profile}" run --rm \
    --volume "${smoke_config}:/tmp/act-smoke.toml:ro" \
    "${service}" act-lab train act \
    --dataset "${dataset}" --output "${run}" --device "${device}" \
    --config /tmp/act-smoke.toml
  docker compose --profile "${profile}" run --rm "${service}" \
    act-lab train resume --run "${run}" --device "${device}" --steps 3
  docker compose --profile "${profile}" run --rm "${service}" python -c \
    "import json, pathlib; p=pathlib.Path('${run}'); m=json.loads((p/'act_lab_run.json').read_text()); rows=(p/'metrics.jsonl').read_text().splitlines(); assert m['status']=='completed' and len(rows)==3; print(f\"PASS: status={m['status']}, metric_steps={len(rows)}, run={p}\")"
}

case "${1:-help}" in
  check)
    run_check
    ;;
  cuda)
    run_cuda
    ;;
  smoke)
    run_smoke "${2:-}" "${3:-cpu}"
    ;;
  help|-h|--help)
    usage
    ;;
  *)
    echo "error: unknown command: ${1}" >&2
    usage >&2
    exit 2
    ;;
esac
