#!/usr/bin/env bash
# Historical launcher; original options retained, not the VD13 entry.
printf '[历史入口] debug_vd5_work_distance.sh：仅供旧版本复现；当前 VD13 见 docs/README.md。\n' >&2
# Local MuJoCo candidate: visible cross sections and a labelled 0.15 s target hold.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../.." && pwd)"
exec bash "$script_dir/debug_vd5_mujoco.sh" \
  --tcp-trailing-distance 0.80 --rear-occlusion-extension 0.05 \
  --rear-centerline-mode cross_sections --rear-hold-s 0.15 \
  --output-root "$script_dir/../outputs/VD5_WORK_DISTANCE_DEBUG" "$@"
