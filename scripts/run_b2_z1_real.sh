#!/usr/bin/env bash
set -euo pipefail

usage() {
    cat <<'EOF'
Usage:
  run_b2_z1_real.sh <network_interface> [--preflight] [real-program options...]
  run_b2_z1_real.sh <network_interface> --run [real-program options...]

The default mode is --preflight. It reads B2/Z1 state, keeps Z1 passive,
does not publish B2 lowcmd, and does not release the B2 factory controller.

Examples:
  ./scripts/run_b2_z1_real.sh enp5s0
  ./scripts/run_b2_z1_real.sh enp5s0 --preflight --z1-ip 192.168.123.110
  ./scripts/run_b2_z1_real.sh enp5s0 --run --z1-ip 192.168.123.110
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
real_binary="${repo_dir}/cmake_build/bin/rl_real_b2_z1_no_gun"

if [[ ! -x "${real_binary}" ]]; then
    echo "Real deployment binary not found: ${real_binary}" >&2
    echo "Build it first: cmake --build cmake_build --target rl_real_b2_z1_no_gun -j2" >&2
    exit 1
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

exec "${real_binary}" "${network_interface}" "$@"
