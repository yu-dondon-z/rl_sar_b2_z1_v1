#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  run_b2_z1_real_vision.sh <network_interface> [--preflight]
  run_b2_z1_real_vision.sh <network_interface> --run

Before --run:
  1. Calibrate config/b2_builtin_front_camera.yaml and set calibrated: true.
  2. The prone camera view may be INVALID; require VALID only after B2 stands.
  3. Use a support frame and keep the remote emergency stop ready.

Controls after --run: 0 stand, 1 policy, T visual tracking, Space stop, P passive.

Path-lock tuning environment variables:
  RL_SAR_VISION_MIN_PATH_WIDTH_PX       default 5.0
  RL_SAR_VISION_LOCK_MAX_ERROR_PX       default 35.0
  RL_SAR_VISION_LOCK_RESET_FRAMES       default 30
  RL_SAR_VISION_LOCK_STREAK             default 6
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
if [[ -x "${HOME}/anaconda3/bin/python" ]]; then
    default_vision_python="${HOME}/anaconda3/bin/python"
else
    default_vision_python="$(command -v python3)"
fi
vision_python="${RL_SAR_VISION_PYTHON:-${default_vision_python}}"
unitree_sdk2_python="${RL_SAR_UNITREE_SDK2_PYTHON:-${HOME}/unitree_sdk2_python}"
rhacracknet_root="${RL_SAR_RHACRACKNET_ROOT:-${HOME}/RHACrackNet/code}"
vision_device="${RL_SAR_VISION_DEVICE:-auto}"
camera_calibration="${RL_SAR_B2_FRONT_CAMERA_CALIBRATION:-${repo_dir}/config/b2_builtin_front_camera.yaml}"
checkpoint="${RL_SAR_CRACK_CHECKPOINT:-${rhacracknet_root}/exps/rhanet_dep_CamCrack789_b2_stage1/ckpt/best_ckpt.pth}"
min_path_width_px="${RL_SAR_VISION_MIN_PATH_WIDTH_PX:-5.0}"
lock_max_error_px="${RL_SAR_VISION_LOCK_MAX_ERROR_PX:-35.0}"
lock_reset_frames="${RL_SAR_VISION_LOCK_RESET_FRAMES:-30}"
lock_streak="${RL_SAR_VISION_LOCK_STREAK:-6}"

for required in "${vision_python}" "${unitree_sdk2_python}" \
                "${rhacracknet_root}" "${camera_calibration}" "${checkpoint}"; do
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
    --unitree-sdk2-python "${unitree_sdk2_python}" \
    --rhacracknet-root "${rhacracknet_root}" \
    --checkpoint "${checkpoint}" \
    --device "${vision_device}" \
    --min-near-path-width-px "${min_path_width_px}" \
    --temporal-lock-max-error-px "${lock_max_error_px}" \
    --temporal-lock-reset-frames "${lock_reset_frames}" \
    --minimum-valid-streak "${lock_streak}" \
    --display --save-overlay &
vision_pid=$!

echo "Waiting for the B2 built-in front-camera vision pipeline..."
for _ in $(seq 1 300); do
    if [[ -s "${exchange_dir}/latest_vision_status.json" ]]; then
        break
    fi
    if ! kill -0 "${vision_pid}" 2>/dev/null; then
        echo "Vision process exited before producing a valid trajectory." >&2
        wait "${vision_pid}"
        exit 1
    fi
    sleep 0.1
done
if [[ ! -s "${exchange_dir}/latest_vision_status.json" ]]; then
    echo "No built-in front-camera frame after 30 seconds; refusing real control." >&2
    exit 1
fi
if [[ -s "${exchange_dir}/latest_trajectory.json" ]]; then
    echo "Built-in camera trajectory is VALID."
else
    echo "Built-in camera is healthy, but no trajectory is valid in the prone view."
    echo "This is expected when the floor path is outside the camera ROI. Stand B2 first,"
    echo "wait for the window to show VALID, enter policy mode, and only then enable tracking."
fi

"${repo_dir}/scripts/run_b2_z1_real.sh" \
    "${network_interface}" --run --vision \
    --vision-exchange-dir "${exchange_dir}" "$@"
