#!/usr/bin/env bash
# Historical launcher; original options retained, not the VD13 entry.
printf '[历史入口] debug_vd5_mujoco.sh：仅供旧版本复现；当前 VD13 见 docs/README.md。\n' >&2
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../.." && pwd)"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
exec /home/dong/anaconda3/envs/crack_vision/bin/python "$script_dir/play_vd5_mujoco.py" \
  --duration 60 --speed 0.20 --curvature 0.05 --physics-dt 0.005 \
  --velocity-constraint impulse \
  --output-root "$script_dir/../outputs/VD5_MUJOCO_DEBUG" \
  --cache-dir /dev/shm/vd5_mujoco_cache "$@"
