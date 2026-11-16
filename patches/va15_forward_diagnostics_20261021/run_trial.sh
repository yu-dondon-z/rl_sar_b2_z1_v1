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
        if [[ ! -t 0 || ! -t 1 ]]; then
            echo 'Static control requires an attended board terminal; no control started.' >&2
            exit 2
        fi
        python3 "${trial_dir}/check_bundle.py"
        if pgrep -f '(^|/)(rl_real_b2_z1_no_gun|va15_forward_trial|run_b2_z1_real.sh)([[:space:]]|$)' >/dev/null; then
            echo 'Another deployment process exists; no second controller will be started.' >&2
            exit 2
        fi
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
        mkdir -p "${repo_dir}/outputs"
        trial_output="$(mktemp -d "${repo_dir}/outputs/forward_diag_static_20261021_XXXXXX")"
        export VA15_TRIAL_DIAGNOSTICS_DIR="${trial_output}"
        echo "Diagnostic CSVs: ${trial_output}"
        exec "${trial_dir}/native_build_verified/va15_forward_trial" "${interface}" --trial-preview
        ;;
    --forward-test)
        echo 'Paused: short-trial physics reproduced posture changes without foot lift.' >&2
        echo 'Use --standby for telemetry checks; extended motion is offline-only until gait validation.' >&2
        exit 2
        ;;
    *) echo 'Usage: bash run_trial.sh --offline-check | [eth0] [--preview|--standby|--forward-test]' >&2; exit 2 ;;
esac
