#!/usr/bin/env bash
set -euo pipefail
trial_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "${trial_dir}/../.." && pwd)"
if [[ "${1:-}" == "--offline-check" ]]; then
    exec python3 "${trial_dir}/check_bundle.py"
fi
interface="${1:-eth0}"
mode="${2:---preview}"
if [[ $# -gt 2 ]]; then
    echo 'No passthrough arguments accepted.' >&2
    exit 2
fi
case "${mode}" in
    --preview)
        # The existing standalone probe has no publisher, policy or Z1 SDK.
        exec "${repo_dir}/scripts/check_b2_independent_state.sh" "${interface}" 10
        ;;
    --standby)
        python3 "${trial_dir}/check_bundle.py"
        [[ -d "/sys/class/net/${interface}" ]] || { echo 'Interface missing.' >&2; exit 2; }
        ip -4 -o addr show dev "${interface}" | grep -q 'inet ' || { echo 'Interface has no IPv4.' >&2; exit 2; }
        echo 'STATIC EXPERIMENT ONLY. No gun/hose; clear arm path; known working remote emergency stop.'
        echo 'A/0 stand, Y/1 policy preview (walking blocked), B/9 parallel get-down.'
        echo 'Do not push sticks or press R1+Y. P/L1+X is emergency passive, NOT a safe crouch.'
        echo 'Ctrl+C only after physically prone + Passive. Do not run a second controller.'
        read -r -p 'Type RUN to release factory B2 control for STATIC A/Y/B only: ' answer
        [[ "${answer}" == "RUN" ]] || { echo 'Cancelled; no control started.'; exit 1; }
        source "${repo_dir}/scripts/local_z1_controller.sh"
        ensure_local_z1_controller "${repo_dir}"
        exec "${trial_dir}/native_build_verified/va15_forward_trial" "${interface}" --trial-preview
        ;;
    --forward-test)
        # Operator reported this SAME candidate's static A/Y/B/exit test OK.
        # Only the launcher is opened; no validation flag, model, gain or
        # support threshold is changed. No motion is started automatically.
        if [[ ! -t 0 || ! -t 1 ]]; then
            echo 'Forward test requires an attended interactive board terminal; no control started.' >&2
            exit 2
        fi
        python3 "${trial_dir}/check_bundle.py"
        [[ -d "/sys/class/net/${interface}" ]] || { echo 'Interface missing.' >&2; exit 2; }
        ip -4 -o addr show dev "${interface}" | grep -q 'inet ' || { echo 'Interface has no IPv4.' >&2; exit 2; }
        if pgrep -f '(^|/)(rl_real_b2_z1_no_gun|va15_forward_trial|run_b2_z1_real.sh)([[:space:]]|$)' >/dev/null; then
            echo 'Another deployment process exists. Do not restart it while standing; no control started.' >&2
            exit 2
        fi
        command -v script >/dev/null || { echo 'Terminal recorder script is required.' >&2; exit 2; }
        echo 'EXPERIMENTAL, NOT MOTION-QUALIFIED: model-estimated support, common-mode slip not observable.'
        echo 'Only proceed after the static candidate test passed, B2 is prone, no other controller is running,'
        echo 'gun/hose are removed, arm path and forward area are clear, reliable fall protection is in place,'
        echo 'and a trained operator has a verified emergency-stop method. Keep people outside the fall/sweep area.'
        echo 'A stand -> Y standby -> wait for [Restricted trial] armed. NO R1+Y / vision.'
        echo 'One forward request <=0.10 m/s, <=2 s. FIRST test: release the stick within 0.5 s.'
        echo 'Release/X asks for a controlled stop; B additionally requests get-down. No automatic retry.'
        echo 'Faults may prevent supported capture/get-down: do not use a timeout, Ctrl+C or power-off as a safe stop.'
        echo 'P/L1+X is emergency passive and may remove support. Follow the verified emergency procedure.'
        echo 'Only after physically prone + Passive: Ctrl+C. Restart is needed for another motion trial.'
        read -r -p 'Type RUN only if all conditions above are satisfied: ' answer
        [[ "${answer}" == "RUN" ]] || { echo 'Cancelled; no control started.'; exit 1; }
        source "${repo_dir}/scripts/local_z1_controller.sh"
        ensure_local_z1_controller "${repo_dir}"
        mkdir -p "${repo_dir}/outputs"
        trial_output="$(mktemp -d "${repo_dir}/outputs/forward_trial_20261021_XXXXXX")"
        printf -v native_command '%q ' "${trial_dir}/native_build_verified/va15_forward_trial" "${interface}" --forward-trial
        echo "Terminal log: ${trial_output}/terminal.log"
        exec script --quiet --flush --return --command "${native_command}" "${trial_output}/terminal.log"
        ;;
    *) echo 'Usage: bash run_trial.sh --offline-check | [eth0] [--preview|--standby|--forward-test]' >&2; exit 2 ;;
esac
