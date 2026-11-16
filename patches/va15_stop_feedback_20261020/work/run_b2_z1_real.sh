#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  run_b2_z1_real.sh --offline-check
  run_b2_z1_real.sh <network_interface> [--preflight] [real-program options...]
  run_b2_z1_real.sh <network_interface> --run [real-program options...]

The default mode is --preflight. It reads B2/Z1 state, keeps Z1 passive,
does not publish B2 lowcmd, and does not release the B2 factory controller.

Examples:
  ./scripts/run_b2_z1_real.sh enp5s0
  ./scripts/run_b2_z1_real.sh eth0 --preflight --disable-z1
  ./scripts/run_b2_z1_real.sh eth0 --run

The local Z1 controller is started in the background automatically, or reused
if it is already running. No second terminal is needed. Logs are saved under
outputs/z1_controller. --offline-check and --disable-z1 skip Z1 startup.
The SDK uses the local controller at 127.0.0.1:8071; do not use the arm's
lower-level 192.168.123.110 address as --z1-ip.
EOF
}

if [[ $# -lt 1 || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
    usage
    exit 0
fi

network_interface="$1"
shift
mode="preflight"
if [[ "${1:-}" == "--run" ]]; then
    mode="run"
    shift
elif [[ "${1:-}" == "--preflight" ]]; then
    shift
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "${script_dir}/.." && pwd)"
real_binary="${repo_dir}/cmake_build_va15/bin/rl_real_b2_z1_no_gun"
python3 "${repo_dir}/scripts/check_va15_legacy.py"

if [[ ! -x "${real_binary}" ]]; then
    echo "Real deployment binary not found: ${real_binary}" >&2
    echo "Build it first: cmake --build cmake_build_va15 --target rl_real_b2_z1_no_gun -j2" >&2
    exit 1
fi

"${real_binary}" --offline-check
if [[ "${network_interface}" == "--offline-check" ]]; then
    exit 0
fi

if [[ ! -d "/sys/class/net/${network_interface}" ]]; then
    echo "Network interface does not exist: ${network_interface}" >&2
    ip -br addr >&2 || true
    exit 1
fi

link_state="$(<"/sys/class/net/${network_interface}/operstate")"
if [[ "${link_state}" == "down" ]]; then
    echo "Network interface ${network_interface} is DOWN; connect and configure the B2 Ethernet link first." >&2
    ip -br addr show dev "${network_interface}" >&2 || true
    exit 1
fi

if [[ "${mode}" == "preflight" ]]; then
    source "${script_dir}/local_z1_controller.sh"
    ensure_local_z1_controller "${repo_dir}" "$@"
    echo "Running read-only B2/Z1 preflight on ${network_interface}."
    echo "B2 factory motion control will not be released and B2 lowcmd will not be published."
    exec "${real_binary}" "${network_interface}" --preflight "$@"
fi

if ! ip -4 -o addr show dev "${network_interface}" | grep -q "inet "; then
    echo "No IPv4 address is configured on ${network_interface}; refusing real low-level control." >&2
    exit 1
fi

echo "REAL LOW-LEVEL CONTROL REQUESTED on ${network_interface}."
echo "Required: support frame attached, workspace clear, remote emergency stop ready."
echo "Keyboard emergency/passive command: P. Process stop: Ctrl+C."
read -r -p 'Type RUN to release the B2 factory controller and continue: ' confirmation
if [[ "${confirmation}" != "RUN" ]]; then
    echo "Cancelled; no control process was started."
    exit 1
fi

source "${script_dir}/local_z1_controller.sh"
ensure_local_z1_controller "${repo_dir}" "$@"
exec "${real_binary}" "${network_interface}" "$@"
