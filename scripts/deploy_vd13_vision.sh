#!/usr/bin/env bash
set -euo pipefail
vd13_vision_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
vd13_vision_python="${VD13_VISION_PYTHON:-/home/dong/anaconda3/envs/crack_vision/bin/python}"
if [[ ! -x "$vd13_vision_python" ]]; then
  printf 'VD13 vision Python not found: %s\n' "$vd13_vision_python" >&2
  exit 1
fi
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
"$vd13_vision_python" -c 'import mujoco, torch, cv2, numpy, yaml'
exec "$vd13_vision_python" -B "$vd13_vision_dir/deploy_vd13_vision.py" "$@"
