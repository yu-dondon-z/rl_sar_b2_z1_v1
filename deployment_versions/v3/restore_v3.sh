#!/usr/bin/env bash
set -euo pipefail

version_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ $# -gt 1 ]]; then
  echo "用法: $0 [rl_sar_b2_z1仓库目录]" >&2
  exit 2
fi

if [[ $# -eq 1 ]]; then
  target_dir="$(cd -- "$1" && pwd)"
else
  target_dir="$(cd -- "${version_dir}/../.." && pwd)"
fi

if [[ ! -d "${target_dir}/src/rl_sar" ]]; then
  echo "目标不是有效的 rl_sar_b2_z1 仓库: ${target_dir}" >&2
  exit 1
fi

rsync -a "${version_dir}/files/" "${target_dir}/"
chmod +x \
  "${target_dir}/scripts/run_b2_z1_real.sh" \
  "${target_dir}/scripts/run_b2_z1_trajectory.sh" \
  "${target_dir}/scripts/run_b2_z1_vision_trajectory.sh" \
  "${target_dir}/scripts/run_mujoco_rhacracknet_bridge.py"

echo "v3 已恢复到: ${target_dir}"
echo "重新编译: cmake --build ${target_dir}/cmake_build --target rl_sim_mujoco -j2"
