#!/usr/bin/env bash
set -euo pipefail

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exchange_dir="/tmp/b2_z1_mujoco_vision"
vision_python="/home/dong/anaconda3/envs/crack_vision/bin/python"
vision_script="${repo_dir}/scripts/run_mujoco_rhacracknet_bridge.py"
scene_file="${repo_dir}/src/rl_sar_zoo/b2_z1_no_gun_description/mjcf/scene_vision.xml"
checkpoint="/home/dong/RHACrackNet/code/exps/rhanet_dep_CamCrack789_b2_stage1/ckpt/best_ckpt.pth"
controller_log="${exchange_dir}/controller.log"

for required in "${vision_python}" "${vision_script}" "${scene_file}" "${checkpoint}"; do
  if [[ ! -e "${required}" ]]; then
    echo "Missing required file: ${required}" >&2
    exit 1
  fi
done

cmake --build "${repo_dir}/cmake_build" --target rl_sim_mujoco -j2
mkdir -p "${exchange_dir}"
rm -f \
  "${exchange_dir}/latest_state.txt" \
  "${exchange_dir}/latest_trajectory.csv" \
  "${exchange_dir}/latest_vision_status.txt" \
  "${exchange_dir}/latest_overlay.png"

vision_pid=""
cleanup() {
  if [[ -n "${vision_pid}" ]] && kill -0 "${vision_pid}" 2>/dev/null; then
    kill "${vision_pid}" 2>/dev/null || true
    wait "${vision_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Starting rear-Z1 RHACrackNet MuJoCo bridge..."
echo "Front RGB-D guides B2; the 180-degree rear Z1 tracks cached crack points behind the base."
echo "Controls: press 0 and wait for GetUp to settle; press 1 to enter the policy, then press T for visual trajectory tracking. Space stops."
echo "Controller diagnostics will be saved to ${controller_log}"
MUJOCO_GL=egl "${vision_python}" -u "${vision_script}" \
  --exchange-dir "${exchange_dir}" \
  --scene "${scene_file}" \
  --camera front_depth_camera \
  --checkpoint "${checkpoint}" \
  --device cuda:0 \
  --segmentation-mode hybrid \
  --path-height-offset 0.005 \
  --display \
  --save-overlay &
vision_pid=$!

cd "${repo_dir}"
"${repo_dir}/cmake_build/bin/rl_sim_mujoco" \
  b2_z1_no_gun scene_vision 2>&1 | tee "${controller_log}"
