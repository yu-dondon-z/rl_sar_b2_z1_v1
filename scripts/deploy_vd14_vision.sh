#!/usr/bin/env bash
set -euo pipefail
vd14_vision_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
vd14_vision_python="${VD14_VISION_PYTHON:-/home/dong/anaconda3/envs/crack_vision/bin/python}"
if [[ ! -x "$vd14_vision_python" ]]; then
  printf 'VD14 vision Python not found: %s\n' "$vd14_vision_python" >&2
  exit 1
fi
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
"$vd14_vision_python" -c 'import mujoco, torch, cv2, numpy, yaml'
exec "$vd14_vision_python" -B "$vd14_vision_dir/deploy_vd14_vision.py" "$@"
