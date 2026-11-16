#!/usr/bin/env bash
set -euo pipefail
trial_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd -- "${trial_dir}/../.." && pwd)"
if [[ "${1:-}" == "--offline-check" ]]; then
    python3 "${trial_dir}/check_trial.py"
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
        if [[ ! -t 0 || ! -t 1 ]]; then
            echo 'Static control requires an attended board terminal; no control started.' >&2
            exit 2
        fi
        python3 "${trial_dir}/check_bundle.py"
        python3 "${trial_dir}/check_trial.py"
        if pgrep -f '(^|/)(rl_real_b2_z1_no_gun|va15_forward_trial|run_b2_z1_real.sh)([[:space:]]|$)' >/dev/null; then
            echo 'Another deployment process exists; no second controller will be started.' >&2
            exit 2
        fi
        [[ -d "/sys/class/net/${interface}" ]] || { echo 'Interface missing.' >&2; exit 2; }
        ip -4 -o addr show dev "${interface}" | grep -q 'inet ' || { echo 'Interface has no IPv4.' >&2; exit 2; }
        echo 'STATIC EXPERIMENT ONLY. No gun/hose; clear arm path; known working remote emergency stop.'
        echo 'Wait for [Motor contract] ready=1 latched=0 bit_flag=4 BEFORE A. If command timeout remains, do NOT stand.'
        echo 'A/0 stand, Y/1 policy preview (walking blocked), B/9 parallel get-down.'
        echo 'Do not push sticks or press R1+Y. P/L1+X is emergency passive, NOT a safe crouch.'
        echo 'Ctrl+C only after physically prone + Passive. Do not run a second controller.'
        read -r -p 'Type RUN to release factory B2 control for STATIC A/Y/B only: ' answer
        [[ "${answer}" == "RUN" ]] || { echo 'Cancelled; no control started.'; exit 1; }
        source "${repo_dir}/scripts/local_z1_controller.sh"
        ensure_local_z1_controller "${repo_dir}"
        mkdir -p "${repo_dir}/outputs"
        trial_output="$(mktemp -d "${repo_dir}/outputs/motor_contract_static_20261021_XXXXXX")"
        export VA15_TRIAL_DIAGNOSTICS_DIR="${trial_output}"
        echo "Diagnostic CSVs: ${trial_output}"
        command -v script >/dev/null || { echo 'Terminal recorder required.' >&2; exit 2; }
        printf -v native_command '%q ' "${trial_dir}/native_build_v2/va15_forward_trial" "${interface}" --trial-preview
        exec script --quiet --flush --return --command "${native_command}" "${trial_output}/terminal.log"
        ;;
    --forward-test)
        echo 'New current-health contract requires an attended static A/Y/B + shutdown review first.' >&2
        echo 'Walking entry remains CLOSED in this version; use --standby, do not call the binary directly.' >&2
        exit 2
        # Preserved bounded forward launcher below is deliberately unreachable
        # until the new raw-field contract and command timeout are reviewed
        # through the real takeover/standing/down sequence.
        if [[ ! -t 0 || ! -t 1 ]]; then
            echo 'Attended interactive board terminal required; no control started.' >&2
            exit 2
        fi
        python3 "${trial_dir}/check_trial.py"
        python3 "${trial_dir}/check_bundle.py"
        if pgrep -f '(^|/)(rl_real_b2_z1_no_gun|va15_forward_trial|run_b2_z1_real.sh)([[:space:]]|$)' >/dev/null; then
            echo 'Another deployment process exists; no second controller will be started.' >&2
            exit 2
        fi
        [[ -d "/sys/class/net/${interface}" ]] || { echo 'Interface missing.' >&2; exit 2; }
        ip -4 -o addr show dev "${interface}" | grep -q 'inet ' || { echo 'Interface has no IPv4.' >&2; exit 2; }
        command -v script >/dev/null || { echo 'Terminal recorder script is required.' >&2; exit 2; }
        echo 'EXPERIMENTAL FIXED-TARGET FORWARD TEST, NOT HARDWARE-QUALIFIED.'
        echo 'Required: B2 physically prone, previous controller exited, NO gun/hose, clear arm/forward workspace,'
        echo 'reliable fall protection, trained operator with a VERIFIED emergency-stop method; people outside fall/sweep area.'
        echo 'A stand -> Y standby -> centre stick -> wait for [Restricted trial] armed. NO R1+Y / vision.'
        echo 'FOR THIS TEST ONLY: holding forward input requests a FIXED 0.15 m/s target through the existing acceleration ramp.'
        echo 'A light accepted push also requests 0.15 m/s, not its raw scaled value. Sideways/backward/yaw are forbidden.'
        echo 'One positive request per process, maximum FOUR monotonic-wall seconds, NOT a physical stop-time guarantee.'
        echo 'If stable, hold briefly until the automatic deadline; if unstable, release immediately or X to request stop.'
        echo 'B additionally requests get-down after supported stopping; no automatic retry from a held stick.'
        echo 'Bad feedback can prevent stand capture/get-down. Do not bypass the gate or use Ctrl+C/power-off as a safe stop.'
        echo 'P/L1+X is emergency passive and can remove support; follow the verified emergency procedure.'
        echo 'Physically prone + Passive BEFORE Ctrl+C. The original production program is unchanged.'
        read -r -p 'Type RUN only if ALL safety conditions above are satisfied: ' answer
        [[ "${answer}" == "RUN" ]] || { echo 'Cancelled; no control started.'; exit 1; }
        mkdir -p "${repo_dir}/outputs"
        trial_output="$(mktemp -d "${repo_dir}/outputs/motor_contract_forward_20261021_XXXXXX")"
        export VA15_TRIAL_DIAGNOSTICS_DIR="${trial_output}"
        source "${repo_dir}/scripts/local_z1_controller.sh"
        ensure_local_z1_controller "${repo_dir}"
        printf -v native_command '%q ' "${trial_dir}/native_build_v2/va15_forward_trial" "${interface}" --forward-trial
        echo "CSV diagnostics and terminal log: ${trial_output}"
        exec script --quiet --flush --return --command "${native_command}" "${trial_output}/terminal.log"
        ;;
    *) echo 'Usage: bash run_trial.sh --offline-check | [eth0] [--preview|--standby|--forward-test]' >&2; exit 2 ;;
esac
