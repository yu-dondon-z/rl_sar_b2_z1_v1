#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
VD12_VISION_PYTHON="${VD12_VISION_PYTHON:-/home/dong/anaconda3/envs/crack_vision/bin/python}"
if [[ ! -x "$VD12_VISION_PYTHON" ]]; then
  echo "VD12 vision Python not found: $VD12_VISION_PYTHON" >&2
  exit 1
fi
"$VD12_VISION_PYTHON" -c 'import mujoco, torch, cv2, numpy, yaml'
export OMP_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
exec "$VD12_VISION_PYTHON" -B "$SCRIPT_DIR/deploy_vd12_vision.py" "$@"
