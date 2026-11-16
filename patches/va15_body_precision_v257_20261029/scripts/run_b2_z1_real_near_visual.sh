#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || "$1" == "-h" || "$1" == "--help" ]]; then
  echo "Usage: $0 <network_interface> [--headless]"
  echo "Example: $0 eth0"
  exit 0
fi

network_interface="$1"
shift
display_flag="--display"
if [[ "${1:-}" == "--headless" ]]; then
  display_flag=""
  shift
fi
if [[ $# -ne 0 ]]; then
  echo "Unknown argument: $1" >&2
  exit 2
fi

repo_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
vision_dir="${repo_dir}/scripts/va15_near"
session_id="$(openssl rand -hex 16)"
output_dir="${repo_dir}/outputs/va15_near_visual_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${output_dir}"
# Keep raw packet reasons and bounded rejection images with this run. Deleting
# the /tmp exchange on exit previously erased the cause of visual stops.
exchange_dir="$(mktemp -d "${output_dir}/vision_XXXXXX")"
vision_pid=""

cleanup() {
  if [[ -n "${vision_pid}" ]] && kill -0 "${vision_pid}" 2>/dev/null; then
    kill "${vision_pid}" 2>/dev/null || true
    wait "${vision_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

mkdir -p "${exchange_dir}"
mkdir -p "${output_dir}"
export VA15_TRIAL_DIAGNOSTICS_DIR="${output_dir}"
echo "Near visual exchange: ${exchange_dir}"
echo "Diagnostics: ${output_dir}"
echo "Starting camera-only guide process; it publishes no motor command."
python3 -u "${vision_dir}/camera_near.py" "${network_interface}" \
  --exchange "${exchange_dir}" \
  --session-id "${session_id}" \
  ${display_flag} > >(tee "${output_dir}/camera.log") 2>&1 &
vision_pid=$!

sleep 2
if ! kill -0 "${vision_pid}" 2>/dev/null; then
  echo "Camera process exited before the controller started." >&2
  exit 1
fi

echo "Camera process is alive. The controller will create its phase context next."
echo "The controller will ask for RUN; type RUN only when the B2 safety setup is ready."
"${repo_dir}/scripts/run_b2_z1_real.sh" "${network_interface}" --run \
  --forward-trial \
  --guide-exchange-dir "${exchange_dir}" \
  --guide-session "${session_id}" 2>&1 | tee "${output_dir}/controller.log"
