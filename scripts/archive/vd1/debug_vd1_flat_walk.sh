#!/usr/bin/env bash
# Historical launcher; original options retained, not the VD13 entry.
printf '[历史入口] debug_vd1_flat_walk.sh：仅供旧版本复现；当前 VD13 见 docs/README.md。\n' >&2
# Local MuJoCo only. No cameras, robot SDK, board sync or world-state feedback.
set -euo pipefail
task_script_dir="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")/../.." && pwd)"
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
exec /home/dong/anaconda3/envs/crack_vision/bin/python \
  "$task_script_dir/diagnose_vd1_base_response.py" \
  --flat-ground --duration 60 --cases straight --speed 0.4 \
  --physics-dt 0.005 --velocity-constraint impulse --physx-implicit-position \
  --tool-sample "$task_script_dir/fixtures/vd1_fixed_camera_tool_command.json" \
  --output-root "$task_script_dir/../outputs/VD1_FLAT_WALK_DEBUG" \
  --viewer --start-paused "$@"
