#!/usr/bin/env bash
set -euo pipefail
echo 'VD12: this is truth-command diagnostic playback, not vision deployment. For dual-camera control use: bash scripts/deploy_vd12_vision.sh --viewer --vision-window --duration 30' >&2
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VD12_MUJOCO_PYTHON="${VD12_MUJOCO_PYTHON:-/home/dong/anaconda3/envs/crack_vision/bin/python}"
if [[ ! -x "$VD12_MUJOCO_PYTHON" ]]; then
  echo "VD12 MuJoCo Python not found: $VD12_MUJOCO_PYTHON" >&2
  exit 1
fi
if ! "$VD12_MUJOCO_PYTHON" -c 'import mujoco, torch, numpy, yaml' ; then
  echo 'VD12 dependency preflight failed. Use the existing crack_vision environment or set VD12_MUJOCO_PYTHON; nothing was installed.' >&2
  exit 1
fi
exec "$VD12_MUJOCO_PYTHON" -B "$SCRIPT_DIR/play_vd12_mujoco.py" "$@"
