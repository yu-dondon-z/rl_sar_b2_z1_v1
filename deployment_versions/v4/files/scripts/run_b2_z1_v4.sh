#!/usr/bin/env bash
set -euo pipefail
repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python_bin="${B2Z1_MUJOCO_PYTHON:-/home/dong/anaconda3/envs/crack_vision/bin/python}"
cd "${repo_dir}"
launcher_args=(--mode vision)
if [[ "${B2Z1_DISPLAY_VISION:-1}" == "1" ]]; then
    launcher_args+=(--display-vision)
fi
exec "${python_bin}" scripts/run_b2_z1_v4.py "${launcher_args[@]}" "$@"
