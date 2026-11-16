#!/usr/bin/env bash
# Historical launcher; original options retained, not the VD13 entry.
printf '[历史入口] play_vd3_front_path.sh：仅供旧版本复现；当前 VD13 见 docs/README.md。\n' >&2
# Front-only local path estimation; rear RGBD continues to command the tool.
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../.." && pwd)"
exec bash "$script_dir/play_vd3_mujoco.sh" \
  --front-path-estimation --front-nominal-height-m 0.55 \
  --output-root "$script_dir/../outputs/VD3_FRONT_PATH_MUJOCO" "$@"
