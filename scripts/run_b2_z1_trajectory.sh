#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

cmake --build "${repo_dir}/cmake_build" --target rl_sim_mujoco -j2
cd "${repo_dir}"
exec "${repo_dir}/cmake_build/bin/rl_sim_mujoco" \
  b2_z1_no_gun scene_flat
