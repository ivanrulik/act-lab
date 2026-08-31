#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"

cd "${repo_root}"

if [[ -z "${DISPLAY:-}" ]]; then
  echo "error: DISPLAY is not set; start the simulator from a graphical session" >&2
  exit 1
fi

display_number="${DISPLAY#*:}"
display_number="${display_number%%.*}"
if [[ ! -S "/tmp/.X11-unix/X${display_number}" ]]; then
  echo "error: no X11/XWayland socket found for DISPLAY=${DISPLAY}" >&2
  exit 1
fi

docker compose --profile ui up --build --detach sim-ui
docker compose --profile ui ps sim-ui
