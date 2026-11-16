#!/usr/bin/env bash
set -euo pipefail
candidate_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--offline-check" && $# -eq 1 ]]; then
    exec python3 "${candidate_dir}/check_candidate.py"
fi
echo 'V3 is OFFLINE ONLY: startup improved but lateral drift regressed.' >&2
echo 'The original V2 control entrypoint is unchanged. No robot process started.' >&2
echo 'Usage: bash run_trial.sh --offline-check' >&2
exit 2
