#!/usr/bin/env python3
"""Headless tests only. No robot clients, real launcher or network access."""
import concurrent.futures
import json
from pathlib import Path
import subprocess

def main():
    root=Path(__file__).resolve().parent
    out=root/'outputs'/'suite1'
    out.mkdir(parents=True,exist_ok=False)
    cases=[(s,.8,1.) for s in ('deadline','release','x','b','lost','stale','preview','held')]
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
    summary=dict(passed=passed,total=len(results),hardware_qualified=False,results=results)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))
    raise SystemExit(0 if passed==len(results) else 2)

if __name__=='__main__':main()
