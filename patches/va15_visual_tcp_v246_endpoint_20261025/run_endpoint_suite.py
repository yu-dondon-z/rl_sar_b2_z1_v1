"""Finite-endpoint VA15/FSM physics, synthetic camera and harness-only truth pose."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parent
    name = sys.argv[1] if len(sys.argv) > 1 else 'endpoint_physics'
    if len(sys.argv) > 3 or Path(name).name != name:
        raise SystemExit('NEW_OUTPUT_NAME [smoke|long|full|estimated]')
    mode = sys.argv[2] if len(sys.argv) > 2 else 'full'
    if mode not in ('smoke', 'long', 'full','estimated'):
        raise SystemExit('Mode must be smoke, long, full or estimated')
    out = root / 'outputs' / name
    out.mkdir(parents=True, exist_ok=False)
    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    tracked = [*(root/'production').glob('*'), root/'simulate_endpoint.cpp', root/'simulate_endpoint_estimated.cpp',root/'run_endpoint_suite.py']
    hashes = {str(path.relative_to(root)):sha(path) for path in tracked if path.is_file()}
    executable = root/('build/simulate_endpoint_estimated' if mode=='estimated' else 'build/simulate_endpoint')
    executable_sha = sha(executable)
    cases = [('straight', .8, 5.)]
    if mode == 'long':
        cases = [(scenario,.8,7.) for scenario in ('straight','curve')]
    if mode == 'full':
        cases = [(scenario, mu, length) for length in (5.,7.)
                 for scenario in ('straight','curve') for mu in (.5,.8,1.)]
    if mode=='estimated':
        cases=[(scenario,.8,length) for length in (5.,7.) for scenario in ('straight','curve')]
    cases += [(scenario,.8,7. if mode != 'smoke' else 5.)
              for scenario in ('unknown_loss','camera_failure','pose_loss','operator_x')]

    def run(case):
        scenario, mu, length = case
        tag = f'{scenario}_mu{mu}_end{length}'
        csv = out/(tag+'.csv')
        try:
            with (out/(tag+'.log')).open('w') as log:
                result = subprocess.run([str(executable),scenario,str(csv),str(mu),str(length)],
                    stdout=log,stderr=subprocess.STDOUT,timeout=240)
            path = Path(str(csv)+'.json')
            report = json.loads(path.read_text()) if path.exists() else {'pass':False,'reason':'No physical report'}
            return dict(name=tag,exit=result.returncode,report=report)
        except subprocess.TimeoutExpired:
            return dict(name=tag,exit=-1,report={'pass':False,'reason':'Physical harness timeout'})

    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(run,cases))
    changed = [path for path,digest in hashes.items() if sha(root/path) != digest]
    if sha(executable) != executable_sha:
        changed.append('build/simulate_endpoint')
    passed = sum(result['exit']==0 and result['report']['pass'] for result in results)
    summary = dict(passed=passed,total=len(results),results=results,sources_sha256=hashes,
        executable_sha256=executable_sha,sources_changed_during_run=changed,
        hardware_qualified=False,precision_qualified=False,synthetic_rgb=True,
        truth_pose_harness_only=mode!='estimated',independent_estimated_pose=mode=='estimated',
        real_camera_or_segmentation_qualification=False)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({key:value for key,value in summary.items() if key not in
        ('sources_sha256','executable_sha256')},indent=2))
    return 0 if passed==len(cases) and not changed else 2


if __name__ == '__main__':
    raise SystemExit(main())
