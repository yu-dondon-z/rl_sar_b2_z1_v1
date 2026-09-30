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
  ./scripts/run_b2_z1_real.sh enp5s0 --preflight
  ./scripts/run_b2_z1_real.sh enp5s0 --run

The launcher starts the local z1_controller when needed.  The Z1 SDK connects
to that controller on 127.0.0.1:8071/8072; 192.168.123.110 is the arm's
lower-level controller address and must not be passed as --z1-ip.
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

z1_disabled=false
z1_ip="127.0.0.1"
z1_ip_explicit=false
real_program_args=("$@")
for ((arg_index = 0; arg_index < ${#real_program_args[@]}; ++arg_index)); do
    argument="${real_program_args[arg_index]}"
    if [[ "${argument}" == "--disable-z1" ]]; then
        z1_disabled=true
    elif [[ "${argument}" == "--z1-ip" ]]; then
        next_index=$((arg_index + 1))
        if ((next_index >= ${#real_program_args[@]})); then
            echo "--z1-ip requires an address" >&2
            exit 2
        fi
        z1_ip="${real_program_args[next_index]}"
        z1_ip_explicit=true
        ((arg_index += 1))
    fi
done

if [[ "${z1_disabled}" == false && "${z1_ip}" == "192.168.123.110" ]]; then
    echo "Invalid --z1-ip 192.168.123.110: this is the Z1 lower-controller address." >&2
    echo "Start z1_controller locally and let the SDK use 127.0.0.1 (the default)." >&2
    exit 2
fi

ensure_local_z1_controller() {
    local controller_root="${RL_SAR_Z1_CONTROLLER_ROOT:-${HOME}/z1_controller}"
    local controller_binary="${controller_root}/build/z1_ctrl"
    local controller_log="${RL_SAR_Z1_CONTROLLER_LOG:-/tmp/z1_controller.log}"
    local controller_pid=""

    if ! pgrep -x z1_ctrl >/dev/null; then
        if [[ ! -x "${controller_binary}" ]]; then
            echo "Z1 controller executable not found: ${controller_binary}" >&2
            echo "Set RL_SAR_Z1_CONTROLLER_ROOT to the z1_controller checkout." >&2
            exit 1
        fi
        echo "Starting local z1_controller in PASSIVE mode. Log: ${controller_log}"
        pushd "${controller_root}/build" >/dev/null
        nohup ./z1_ctrl </dev/null >"${controller_log}" 2>&1 &
        controller_pid=$!
        popd >/dev/null
        echo "${controller_pid}" > /tmp/z1_controller.pid
    fi

    for _ in $(seq 1 30); do
        if ss -lunp 2>/dev/null | grep -Eq ':8071[[:space:]].*z1_ctrl'; then
            echo "Local z1_controller is ready on UDP 8071."
            return 0
        fi
        if [[ -n "${controller_pid}" ]] && ! kill -0 "${controller_pid}" 2>/dev/null; then
            echo "z1_controller exited during startup. Recent log:" >&2
            tail -40 "${controller_log}" >&2 || true
            exit 1
        fi
        sleep 0.1
    done

    echo "z1_controller is not listening on UDP 8071. Recent log:" >&2
    tail -40 "${controller_log}" >&2 || true
    exit 1
}

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

if [[ "${z1_disabled}" == false && "${z1_ip}" == "127.0.0.1" ]]; then
    ensure_local_z1_controller
fi

if [[ "${z1_disabled}" == false && "${z1_ip_explicit}" == false ]]; then
    set -- "$@" --z1-ip 127.0.0.1
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
echo "Remote: A=get up, Y=locomotion, B=get down, R1+Y=visual trajectory, X=zero velocity, L1+X=emergency passive."
read -r -p 'Type RUN to release the B2 factory controller and continue: ' confirmation
if [[ "${confirmation}" != "RUN" ]]; then
    echo "Cancelled; no control process was started."
    exit 1
fi

exec "${real_binary}" "${network_interface}" "$@"
