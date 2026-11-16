#!/usr/bin/env bash
# Historical launcher; original options retained, not the VD13 entry.
printf '[历史入口] debug_vd3_rear_vision.sh：仅供旧版本复现；当前 VD13 见 docs/README.md。\n' >&2
# Current VD3 MuJoCo rear RGB-D diagnostic; never starts real hardware.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../.." && pwd)"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
exec /home/dong/anaconda3/envs/crack_vision/bin/python \
  "$script_dir/debug_vd3_rear_vision.py" "$@"
