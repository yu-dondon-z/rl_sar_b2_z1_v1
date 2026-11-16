#!/usr/bin/env bash
# Historical launcher; original options retained, not the VD13 entry.
printf '[历史入口] play_vd3_mujoco.sh：仅供旧版本复现；当前 VD13 见 docs/README.md。\n' >&2
# Local VD3 MuJoCo deployment with rendered front RGB and rear RGBD inputs.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../.." && pwd)"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
exec /home/dong/anaconda3/envs/crack_vision/bin/python "$script_dir/play_vd1_mujoco.py" \
  --robot-version vd3 --visual-controller guarded --speed 0.2 \
  --curvature-stage 4 --road-surface tape --physics-dt 0.005 \
  --velocity-constraint impulse --physx-implicit-position \
  --output-root "$script_dir/../outputs/VD3_MUJOCO" \
  --cache-dir /dev/shm/vd3_mujoco_visual_cache "$@"
