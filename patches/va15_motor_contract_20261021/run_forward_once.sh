#!/usr/bin/env bash
set -euo pipefail
trial_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "${trial_dir}/../.." && pwd)"

offline_checks() {
    python3 "${trial_dir}/check_trial.py"
    python3 "${trial_dir}/check_bundle.py"
    python3 "${trial_dir}/check_static_evidence.py"
}

if [[ $# -eq 1 && "$1" == '--offline-check' ]]; then
    offline_checks
    exit 0
fi
if [[ $# -ne 2 || "$2" != '--forward-test' ]]; then
    echo 'Usage: bash run_forward_once.sh --offline-check | eth0 --forward-test' >&2
    echo 'No default motion and no passthrough arguments.' >&2
    exit 2
fi
if [[ ! -t 0 || ! -t 1 ]]; then
    echo 'Attended interactive board terminal required; no control started.' >&2
    exit 2
fi
refuse_other_controller() {
    if pgrep -f '(^|/)(rl_real_b2_z1_no_gun|va15_forward_trial|run_b2_z1_real.sh)([[:space:]]|$)' >/dev/null; then
        echo 'Another deployment process exists; refusing a second controller.' >&2
        exit 2
    fi
}
refuse_other_controller
network_interface="$1"
[[ -d "/sys/class/net/${network_interface}" ]] || { echo 'Network interface missing.' >&2; exit 2; }
ip -4 -o addr show dev "${network_interface}" | grep -q 'inet ' || { echo 'Interface has no IPv4.' >&2; exit 2; }
command -v script >/dev/null || { echo 'Terminal recorder is required.' >&2; exit 2; }
offline_checks

echo 'EXPERIMENTAL ONE-SHOT FORWARD TEST: fixed 0.15 m/s target, maximum FOUR seconds positive request.'
echo 'This reuses the EXACT static-tested native binary. NOT hardware/motion qualified.'
echo 'Required NOW: B2 physically prone, previous controller exited, NO gun/hose, clear forward/arm workspace,'
echo 'reliable fall protection and a trained operator with a VERIFIED emergency procedure; keep people out of the fall/sweep area.'
echo 'RUN starts passive control only. Wait for [Motor contract] ready=1 latched=0 bit_flag=4 BEFORE A.'
echo 'Then A stand -> Y standby -> centre sticks -> wait for [Restricted trial] armed. NO vision / R1+Y.'
echo 'Forward stick is a HELD TRIGGER: an accepted light push also requests 0.15 m/s via the unchanged acceleration ramp.'
echo 'No lateral/backward/yaw motion. Only one positive request per process; held sticks/Y cannot re-arm it.'
echo 'If unstable, immediately release or X to REQUEST stop; do not wait for the four-second deadline.'
echo 'Four seconds bounds the positive request, NOT physical stopping time/distance.'
echo 'After actual stop, B requests supported get-down; faults may prevent stand capture/get-down.'
echo 'P / L1+X is emergency PASSIVE and can remove support, not a safe crouch; use the verified emergency procedure.'
echo 'Physically prone + Passive BEFORE Ctrl+C. Never use forced process exit/power-off as a normal stop.'
read -r -p 'Type RUN only if ALL current safety conditions above are satisfied: ' answer
[[ "${answer}" == 'RUN' ]] || { echo 'Cancelled; no controller or Z1 service started.'; exit 1; }
refuse_other_controller

mkdir -p "${repo_dir}/outputs"
trial_output="$(mktemp -d "${repo_dir}/outputs/motor_contract_forward_once_20261021_XXXXXX")"
export VA15_TRIAL_DIAGNOSTICS_DIR="${trial_output}"
source "${repo_dir}/scripts/local_z1_controller.sh"
ensure_local_z1_controller "${repo_dir}"
printf -v native_command '%q ' "${trial_dir}/native_build_v2/va15_forward_trial" "${network_interface}" --forward-trial
echo "CSV diagnostics and terminal log: ${trial_output}"
exec script --quiet --flush --return --command "${native_command}" "${trial_output}/terminal.log"
