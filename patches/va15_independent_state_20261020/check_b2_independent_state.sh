#!/usr/bin/env bash
# Read-only standalone observer. Never starts a B2/Z1 controller.
set -euo pipefail
if (( $# > 2 )); then
  echo "Usage: $0 [interface=eth0] [seconds=10]; READ ONLY" >&2
  exit 2
fi
b2_check_script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
b2_check_repo_dir=$(cd -- "${b2_check_script_dir}/.." && pwd)
b2_check_observer_dir="${b2_check_repo_dir}/diagnostics/independent_state_20261020_L7VGqH"
b2_check_interface=${1:-eth0}
b2_check_seconds=${2:-10}
if [[ ! -d "/sys/class/net/${b2_check_interface}" ]]; then
  echo "No such interface: ${b2_check_interface}" >&2
  exit 2
fi
if [[ ! -x "${b2_check_observer_dir}/build/independent_probe" ]]; then
  echo "Read-only observer is not built: ${b2_check_observer_dir}" >&2
  exit 2
fi
b2_check_sdk_lib_dir="${b2_check_repo_dir}/src/rl_sar/library/thirdparty/robot_sdk/unitree/unitree_sdk2/thirdparty/lib/x86_64"
if [[ ! -r "${b2_check_sdk_lib_dir}/libddsc.so" || ! -r "${b2_check_sdk_lib_dir}/libddscxx.so" ]]; then
  echo "Missing matched Unitree DDS runtime: ${b2_check_sdk_lib_dir}" >&2
  exit 2
fi
# A normal PC4 shell sources ROS Humble, which otherwise overrides the SDK's
# RUNPATH with an incompatible C DDS library. Only this child environment is
# changed; do not modify ~/.bashrc or the system ROS installation.
export LD_LIBRARY_PATH="${b2_check_sdk_lib_dir}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
b2_check_linkage=$(ldd "${b2_check_observer_dir}/build/independent_probe")
b2_check_ddsc=$(awk '$1 ~ /^libddsc\.so/ {print $3}' <<<"${b2_check_linkage}")
b2_check_ddscxx=$(awk '$1 ~ /^libddscxx\.so/ {print $3}' <<<"${b2_check_linkage}")
if [[ ! -f "${b2_check_ddsc}" || ! -f "${b2_check_ddscxx}" ]] || \
   [[ "$(readlink -f -- "${b2_check_ddsc}")" != "$(readlink -f -- "${b2_check_sdk_lib_dir}/libddsc.so")" ]] || \
   [[ "$(readlink -f -- "${b2_check_ddscxx}")" != "$(readlink -f -- "${b2_check_sdk_lib_dir}/libddscxx.so")" ]]; then
  echo "DDS runtime mismatch; refusing to start read-only probe." >&2
  exit 2
fi
echo "[DDS runtime] Matched Unitree libddsc + libddscxx verified."
mkdir -p "${b2_check_repo_dir}/outputs"
b2_check_output_dir=$(mktemp -d "${b2_check_repo_dir}/outputs/independent_state_check_XXXXXX")
echo "READ ONLY: no B2/Z1 commands, no factory-controller release; authority=0."
echo "Reports: ${b2_check_output_dir}"
set +e
"${b2_check_observer_dir}/build/independent_probe" --subscribe \
  "${b2_check_observer_dir}/outputs/b2_independent_model.json" \
  "${b2_check_interface}" "${b2_check_output_dir}/stream" \
  --seconds "${b2_check_seconds}"
b2_check_result=$?
set -e
if (( b2_check_result == 0 )); then
  echo "Read-only probe exited normally (exit=0); this is not a motion acceptance."
else
  echo "Read-only probe failed or was interrupted (exit=${b2_check_result}); reports are not an acceptance." >&2
fi
exit "${b2_check_result}"
