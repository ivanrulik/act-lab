#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"
cd "${repo_root}"

latest_recording() {
  find data/raw -maxdepth 1 -type f -name '*.mcap' -printf '%T@ %p\n' 2>/dev/null \
    | sort -nr | head -n 1 | cut -d' ' -f2-
}

recordings() {
  shopt -s nullglob
  local files=(data/raw/*.mcap)
  if (( ${#files[@]} == 0 )); then
    echo "error: no recordings found; run '$0 record' or '$0 record-auto' first" >&2
    exit 1
  fi
  printf '%s\n' "${files[@]}"
}

case "${1:-help}" in
  record)
    docker compose --profile ui run --rm keyboard-teleop \
      act-lab sim keyboard-teleop --seed "${2:-0}" \
      --record-dir data/raw --operator "${ACT_LAB_OPERATOR:-learner}"
    ;;
  record-auto)
    docker compose run --rm dev act-lab sim expert \
      --seed-start 0 --episodes "${2:-5}" --record-dir data/raw --json
    ;;
  replay)
    episode="${2:-$(latest_recording)}"
    if [[ -z "${episode}" ]]; then
      echo "error: no recordings found; run '$0 record' or '$0 record-auto' first" >&2
      exit 1
    fi
    docker compose --profile ui run --rm keyboard-teleop \
      act-lab recording replay "${episode}" --camera policy --display
    ;;
  validate)
    mapfile -t files < <(recordings)
    docker compose run --rm dev act-lab recording validate "${files[@]}"
    ;;
  foxglove)
    episode="${2:-$(latest_recording)}"
    if [[ -z "${episode}" ]]; then
      echo "error: no recordings found; run '$0 record' or '$0 record-auto' first" >&2
      exit 1
    fi
    output="runs/foxglove/$(basename "${episode}" .mcap).foxglove.mcap"
    docker compose run --rm dev act-lab recording foxglove \
      "${episode}" --output "${output}"
    echo "Open ${output} in Foxglove and select /foxglove/camera/policy."
    ;;
  convert)
    mapfile -t files < <(recordings)
    docker compose run --rm dev act-lab recording manifest "${files[@]}" \
      --output data/selection.json --split-seed 0 --validation-fraction 0.2
    docker compose --profile data build data
    docker compose --profile data run --rm data act-lab recording convert \
      data/selection.json --output data/lerobot/pick-place \
      --repo-id local/act-lab-pick-place --fps 25
    ;;
  inspect)
    docker compose --profile data run --rm data python -c \
      'from lerobot.datasets.lerobot_dataset import LeRobotDataset; d = LeRobotDataset("local/act-lab-pick-place", root="data/lerobot/pick-place"); print(f"episodes: {d.num_episodes}\nframes: {len(d)}\nfeatures: {list(d.features)}"); print({key: getattr(value, "shape", value) for key, value in d[0].items()})'
    ;;
  *)
    echo "usage: $0 {record [seed]|record-auto [episodes]|replay [episode.mcap]|validate|foxglove [episode.mcap]|convert|inspect}"
    ;;
esac
