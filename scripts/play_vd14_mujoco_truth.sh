#!/usr/bin/env bash
set -euo pipefail
printf '%s\n' 'VD14 truth-command diagnostic only. For RGB/RGB-D + RHANet use scripts/deploy_vd14_vision.sh.' >&2
vd14_truth_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
vd14_truth_python="${VD14_MUJOCO_PYTHON:-/home/dong/anaconda3/envs/crack_vision/bin/python}"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
"$vd14_truth_python" -c 'import mujoco, torch, numpy, yaml'
exec "$vd14_truth_python" -B "$vd14_truth_dir/play_vd14_mujoco_truth.py" "$@"
