#!/usr/bin/env python3
"""Headless tests only. No robot clients, real launcher or network access."""
import concurrent.futures
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

def main():
    root=Path(__file__).resolve().parent
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-name',default='suite_verified')
    args=parser.parse_args()
    if Path(args.output_name).name!=args.output_name:raise SystemExit('One new output directory name required')
    out=root/'outputs'/args.output_name
    out.mkdir(parents=True,exist_ok=False)
    tracked=list((root/'production').glob('*'))+[root/name for name in (
        'simulate_trial.cpp','test_trial.cpp','test_telemetry.cpp','analyze_telemetry.py','test_analysis.py',
        'test_motor_contract.cpp','test_startup.cpp','test_startup_fsm.cpp','CMakeLists.txt')]
    digest=lambda path:hashlib.sha256(path.read_bytes()).hexdigest()
    sources={str(path.relative_to(root)):digest(path) for path in tracked if path.is_file()}
    cases=[(s,.8,1.) for s in ('deadline','release','x','b','lost','stale','preview','held',
        'motor_fault','frequency','timeout','component','lost_drop','motor_mode','timeout_at_start',
        'startup_counts_new','startup_change','startup_churn')]
    cases += [(s,mu,tau) for s in ('deadline','release','x','b')
              for mu,tau in ((.5,1.),(1.,1.),(.8,.9),(.8,1.1))]
    def run(case):
        scenario,mu,tau=case
        name=f'{scenario}_mu{mu:.1f}_tau{tau:.1f}'
        csv=out/(name+'.csv')
        with (out/(name+'.log')).open('w') as log:
            result=subprocess.run([str(root/'build/simulate_trial'),
                str(root/'production/b2_independent_model.json'),str(csv),scenario,str(mu),str(tau)],
                stdout=log,stderr=subprocess.STDOUT,timeout=120)
        report=json.loads(Path(str(csv)+'.json').read_text()) if Path(str(csv)+'.json').exists() else {}
        return dict(name=name,exit=result.returncode,report=report)
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        results=list(pool.map(run,cases))
    passed=sum(r['exit']==0 and r['report'].get('pass',False) for r in results)
    if any(digest(root/name)!=value for name,value in sources.items()):raise RuntimeError('Sources changed during suite')
    summary=dict(passed=passed,total=len(results),hardware_qualified=False,results=results,
        sources_sha256=sources,simulation_executable_sha256=digest(root/'build/simulate_trial'))
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(dict(passed=passed,total=len(results),hardware_qualified=False,
                         summary=str(out/'summary.json'))))
    raise SystemExit(0 if passed==len(results) else 2)

if __name__=='__main__':main()
