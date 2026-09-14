#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  run_b2_z1_real_vision.sh <network_interface> [--preflight]
  run_b2_z1_real_vision.sh <network_interface> --run

Before --run:
  1. Calibrate config/b2_builtin_front_camera.yaml and set calibrated: true.
  2. Verify the vision window stays VALID while B2 is moved by hand.
  3. Use a support frame and keep the remote emergency stop ready.

Controls after --run: 0 stand, 1 policy, T visual tracking, Space stop, P passive.
EOF
}

if [[ $# -lt 1 || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

network_interface="$1"
shift
mode="${1:---preflight}"
if [[ "${mode}" == "--run" || "${mode}" == "--preflight" ]]; then
    shift || true
else
    echo "Expected --preflight or --run, got: ${mode}" >&2
    exit 2
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "${script_dir}/.." && pwd)"
exchange_dir="${RL_SAR_REAL_VISION_EXCHANGE_DIR:-/tmp/b2_z1_real_vision}"
# The base environment already contains both Unitree SDK2's CycloneDDS binding
# and the RHACrackNet runtime.  The crack_vision environment does not contain
# cyclonedds on this machine, so it cannot open B2's built-in camera stream.
vision_python="${RL_SAR_VISION_PYTHON:-/home/dong/anaconda3/bin/python}"
camera_calibration="${RL_SAR_B2_FRONT_CAMERA_CALIBRATION:-${repo_dir}/config/b2_builtin_front_camera.yaml}"
checkpoint="${RL_SAR_CRACK_CHECKPOINT:-/home/dong/RHACrackNet/code/exps/rhanet_dep_CamCrack789_b2_stage1/ckpt/best_ckpt.pth}"

for required in "${vision_python}" "${camera_calibration}" "${checkpoint}"; do
    if [[ ! -e "${required}" ]]; then
        echo "Required file is missing: ${required}" >&2
        exit 1
    fi
done

if [[ "${mode}" == "--preflight" ]]; then
    exec "${repo_dir}/scripts/run_b2_z1_real.sh" \
        "${network_interface}" --preflight --vision \
        --vision-exchange-dir "${exchange_dir}" "$@"
fi

rm -f "${exchange_dir}/latest_trajectory.json" \
      "${exchange_dir}/latest_vision_status.json"
mkdir -p "${exchange_dir}"
vision_pid=""
cleanup() {
    if [[ -n "${vision_pid}" ]] && kill -0 "${vision_pid}" 2>/dev/null; then
        kill "${vision_pid}" 2>/dev/null || true
        wait "${vision_pid}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

"${vision_python}" -u "${repo_dir}/scripts/run_b2_builtin_front_rhacracknet_bridge.py" \
    "${network_interface}" \
    --camera-calibration "${camera_calibration}" \
    --exchange-dir "${exchange_dir}" \
    --checkpoint "${checkpoint}" \
    --display --save-overlay &
vision_pid=$!

echo "Waiting for a validated B2 built-in front-camera trajectory..."
for _ in $(seq 1 300); do
    if [[ -s "${exchange_dir}/latest_trajectory.json" ]]; then
        break
    fi
    if ! kill -0 "${vision_pid}" 2>/dev/null; then
        echo "Vision process exited before producing a valid trajectory." >&2
        wait "${vision_pid}"
        exit 1
    fi
    sleep 0.1
done
if [[ ! -s "${exchange_dir}/latest_trajectory.json" ]]; then
    echo "No validated built-in front-camera trajectory after 30 seconds; refusing real control." >&2
    exit 1
fi

"${repo_dir}/scripts/run_b2_z1_real.sh" \
    "${network_interface}" --run --vision \
    --vision-exchange-dir "${exchange_dir}" "$@"
