#!/usr/bin/env bash
set -euo pipefail
kit="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd -- "${kit}/../.." && pwd)"
if [[ "${1:-}" == --offline-check && $# == 1 ]]; then
  exec python3 "${kit}/check_bundle.py"
fi
[[ $# == 1 && -t 0 && -t 1 ]] || { echo 'Usage in attended PC4 terminal: bash run_blind.sh eth0'; exit 2; }
interface="$1"
python3 "${kit}/check_bundle.py"
python3 - "${kit}" <<'PY'
import hashlib,json,sys
from pathlib import Path
kit=Path(sys.argv[1]);q=json.loads((kit/'qualification.json').read_text())
if not q.get('simulation_pass') or not q.get('blind_test_release_allowed') or q.get('hardware_qualified'):
    raise SystemExit('Blind diagnostic is not released; offline preparation only.')
for name,expected in q['production_sha256'].items():
    if hashlib.sha256((kit/'production'/name).read_bytes()).hexdigest()!=expected:
        raise SystemExit('Candidate no longer matches offline tests.')
PY
if pgrep -f '(^|/)(rl_real_b2_z1_no_gun|va15_forward_trial|run_b2_z1_real.sh)([[:space:]]|$)' >/dev/null; then
  echo 'Another controller exists. Refusing a second controller.' >&2; exit 2
fi
[[ -d "/sys/class/net/${interface}" ]] || exit 2
ip -4 -o addr show dev "${interface}" | grep -q 'inet '
echo 'V2.4.3 BLIND FIXED-TCP DIAGNOSTIC, NOT HARDWARE QUALIFIED; Z1 pre-positioning during Y.'
echo 'NO camera, NO visual yaw, NO dynamic TCP path. VA15 model/gains/70D history unchanged.'
echo 'Requires B2 physically prone, old controller exited, NO gun/hose, clear arm/fall area.'
echo 'Keep reliable fall protection and remote emergency stop ready; people must stay clear.'
echo 'RUN -> A/0 stand -> wait stable -> Y/1 standby -> wait warmup and healthy support.'
echo 'R1+Y/T starts fixed 0.15 m/s forward, y=yaw=0. Do not push sticks.'
echo 'First test: only a short observed advance, then X/Space. R1+Y also stops.'
echo 'B/9 requests supported stop and get-down. Repeat start needs an explicit new R1+Y.'
echo 'There is no timer guarantee of stopping. On renewed shaking, stop the test; do not increase speed/gains.'
echo 'P/L1+X removes active support: use only the verified emergency procedure.'
echo 'Physically prone and Passive before Ctrl+C. Do not power off a standing robot.'
read -r -p 'Type RUN only if the conditions above are satisfied: ' answer
[[ "${answer}" == RUN ]] || { echo 'Cancelled; no control started.'; exit 1; }
mkdir -p "${repo}/outputs"
output="$(mktemp -d "${repo}/outputs/va15_blind_v243_XXXXXX")"
export VA15_TRIAL_DIAGNOSTICS_DIR="${output}"
python3 - "${output}" "${kit}" <<'PY'
import json,sys,time
from pathlib import Path
sys.path.insert(0,sys.argv[2])
from check_bundle import check
p=Path(sys.argv[1]);(p/'session.json').write_text(json.dumps(dict(version='V2.4.3',mode='blind_fixed_tcp',
    bundle_sha256=check(False),created_unix_s=time.time(),speed=.15,yaw=0,vision_used=False,
    dynamic_tcp=False,hardware_qualified=False),indent=2)+'\n')
PY
source "${repo}/scripts/local_z1_controller.sh"
ensure_local_z1_controller "${repo}"
printf -v cmd '%q ' "${kit}/native_build_visual/va15_forward_trial" "${interface}" --forward-trial --blind-fixed-target
echo "Records: ${output}"
# Keep this shell as parent of an independently started Z1 daemon.
script --quiet --flush --return --command "${cmd}" "${output}/terminal.log"
