#!/usr/bin/env bash
set -euo pipefail
kit="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd -- "${kit}/../.." && pwd)"
interface="${1:?interface}"; mode="${2:?mode}"; exchange="${3:?exchange}"; nonce="${4:?nonce}"; output="${5:?output}"
[[ $# == 5 ]] || exit 2
[[ -t 0 && -t 1 ]] || { echo 'Attended board terminal required.' >&2; exit 2; }
case "${mode}" in
  --shadow) native_mode=--trial-preview ;;
  --walk) native_mode=--forward-trial ;;
  *) echo 'Invalid control mode.' >&2; exit 2 ;;
esac
python3 "${kit}/check_bundle.py"
if pgrep -f '(^|/)(rl_real_b2_z1_no_gun|va15_forward_trial|run_b2_z1_real.sh)([[:space:]]|$)' >/dev/null; then
  echo 'Another controller exists. No second controller started.' >&2; exit 2
fi
[[ -d "/sys/class/net/${interface}" && -d "${exchange}" && -d "${output}" ]] || exit 2
ip -4 -o addr show dev "${interface}" | grep -q 'inet '
echo 'EXPERIMENT ONLY, NOT HARDWARE QUALIFIED. Existing motor-health/support/stop gates remain active.'
echo 'B2 must be physically prone; prior control exited; NO gun or hose; clear sweep/fall area.'
echo 'Use reliable fall protection and a verified remote emergency-stop procedure; keep people clear.'
echo 'A/0 stand -> wait READY -> Y/1 standby -> wait warmup + quiet support.'
echo 'Confirm the YELLOW near / CYAN far curve is on the intended tape, not a tile seam. Wait LONG PATH.'
echo 'If the prone-view lock persists, focus the CAMERA WINDOW and press R once while standing STILL to reacquire.'
echo 'FIRST R1+Y / T: four-second ZERO-OUTPUT review. Wait SHADOW PASSED; this does not start walking.'
echo 'Release buttons. SECOND R1+Y / T within 30 s: one four-second walk (only --walk).'
echo 'Manual sticks must stay centred; moving them cancels. A changed/stale path requires a new explicit review.'
echo "Mode: ${mode}. Shadow applies ZERO motion; walk requests fixed 0.15 m/s with yaw limited to +/-0.06 rad/s."
echo 'TCP TARGET DOES NOT CHANGE. This is near/lookahead pixel-curve body guidance, not full metric/TCP trajectory following.'
echo 'X/Space requests supported stop; B/9 additionally requests get-down. No automatic retry.'
echo 'A deadline bounds the REQUEST, not stopping time/distance. Invalid support may prevent capture/get-down.'
echo 'P/L1+X is emergency passive and can remove support. Follow the verified emergency procedure.'
echo 'Only Ctrl+C after physically prone + Passive; do not use shutdown/power-off as a safe standing stop.'
read -r -p 'Type RUN only when the above conditions are satisfied: ' answer
[[ "${answer}" == RUN ]] || { echo 'Cancelled. No control started.'; exit 1; }
source "${repo}/scripts/local_z1_controller.sh"
ensure_local_z1_controller "${repo}"
export VA15_TRIAL_DIAGNOSTICS_DIR="${output}"
printf -v cmd '%q ' "${kit}/native_build_visual/va15_forward_trial" "${interface}" "${native_mode}" \
  --guide-exchange-dir "${exchange}" --guide-session "${nonce}"
# Preserve this shell as parent of a daemon it may have started. Never exec
# util-linux script with the long-lived Z1 child, and never kill Z1 on exit.
script --quiet --flush --return --command "${cmd}" "${output}/terminal.log"
