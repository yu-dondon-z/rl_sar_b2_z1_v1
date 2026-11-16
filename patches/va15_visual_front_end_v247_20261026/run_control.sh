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
if [[ "${mode}" == --walk ]]; then
  python3 - "${kit}" <<'PY'
import hashlib,json,sys
from pathlib import Path
kit=Path(sys.argv[1]);q=json.loads((kit/'qualification.json').read_text())
if not q.get('simulation_pass') or not q.get('walk_release_allowed') or q.get('hardware_qualified'):
    raise SystemExit('V2.4.7 visual experiment is not released; inspect qualification and board checks.')
for name,expected in q['production_sha256'].items():
    if hashlib.sha256((kit/'production'/name).read_bytes()).hexdigest()!=expected:
        raise SystemExit('V2.4.7 source no longer matches the offline-tested candidate.')
PY
fi
if pgrep -f '(^|/)(rl_real_b2_z1_no_gun|va15_forward_trial|run_b2_z1_real.sh)([[:space:]]|$)' >/dev/null; then
  echo 'Another controller exists. No second controller started.' >&2; exit 2
fi
[[ -d "/sys/class/net/${interface}" && -d "${exchange}" && -d "${output}" ]] || exit 2
ip -4 -o addr show dev "${interface}" | grep -q 'inet '
echo 'EXPERIMENT ONLY, NOT HARDWARE QUALIFIED. Existing motor-health/support/stop gates remain active.'
echo 'B2 must be physically prone; prior control exited; NO gun or hose; clear sweep/fall area.'
echo 'Use reliable fall protection and a verified remote emergency-stop procedure; keep people clear.'
echo 'A/0 stand -> wait READY -> Y/1 standby -> wait warmup + quiet support.'
echo 'V2.4.7: confirmed front-camera crack end -> original supported standing stop.'
echo 'The rear TCP need NOT reach the physical endpoint; an end seen ahead can stop B2 early.'
echo 'Look row 820.8..885.6, INSIDE a current >=200 px continuous observed path; no extrapolation or stale timestamp refresh.'
echo 'Actual motor faults, controller timeout, source freshness, Z1 watchdog and support/stop gates remain active.'
echo 'Image display window: near +/-0.60 rad, lookahead +/-0.65 rad; valid metric reference <=0.45 rad.'
echo 'Confirm the YELLOW near / CYAN far curve is on the intended tape, not a tile seam. Wait PATH IN RANGE.'
echo 'Camera automatically reacquires after standing; no R key is required.'
echo 'R1+Y / T: directly START walking; press it again to STOP. No extra four-second static review.'
echo 'Walking continues until a confirmed visible END, R1+Y/X/B, or a live failure. NO fixed short-walk timer.'
echo 'Normal END requires six fresh same-track cap frames over >=0.5 s; no image-boundary extrapolation.'
echo 'Occlusion, lost line, image expiry or faults are PROTECTIVE stops, NOT normal END completion.'
echo 'END does not trigger automatic get-down or restart; stay at the controls.'
echo 'After a completed supported stop, R1+Y explicitly starts a new session; no process restart required.'
echo 'Manual sticks must stay centred. Visual loss stops; fresh vision NEVER restarts motion by itself.'
echo "Mode: ${mode}. Shadow applies ZERO motion; walk requests fixed 0.15 m/s with yaw limited to +/-0.06 rad/s."
echo 'TCP ENABLED: calibrated monocular FLAT-floor projection + IMU/joint no-slip LOCAL pose + front-to-rear path cache.'
echo 'This is NOT depth or certified odometry. Use a flat floor only; no stairs, obstacles or uneven ground.'
echo 'TCP holds entry clearance first; observed rear history enables a <=1 cm/s transition to the TRAINED 9 cm height.'
echo 'Trained z limits [-0.55,-0.30] are restored; no 1 cm ground target or IK override; trained gains after torque-equivalent join.'
echo 'The rear path is initially unseen: TCP waits at the original target until B2 has walked past front-observed tape.'
echo 'Watch active=1 in the TCP log. An unseen/unreachable point is not fabricated or clamped onto another path.'
echo 'X/Space requests supported stop; B/9 additionally requests get-down. No automatic retry.'
echo 'No duration timer guarantees a stop. Stay at the controls; invalid support may prevent capture/get-down.'
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
