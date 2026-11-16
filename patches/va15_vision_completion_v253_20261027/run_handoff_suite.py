"""Offline blind fixed-TCP VA15 comparison. Does not import any robot SDK."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import subprocess
import sys

def main():
    root=Path(__file__).resolve().parent
    name=sys.argv[1] if len(sys.argv)==2 else 'handoff_blind_verified'
    if Path(name).name!=name:raise SystemExit('A new output directory name is required')
    out=root/'outputs'/name;out.mkdir(parents=True,exist_ok=False)
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    files=[*root.joinpath('production').glob('*'),root/'simulate_handoff.cpp',root/'run_handoff_suite.py']
    hashes={str(p.relative_to(root)):sha(p) for p in files if p.is_file()}
    cases=[(version,speed,mu,'walk') for version in ['v242','candidate'] for speed in [.15,.20] for mu in [.5,.8,1.]]
    cases += [('candidate',.15,mu,stop) for mu in [.5,.8,1.] for stop in ['b_early','x_early']]
    def run(case):
        version,speed,mu,scenario=case
        name=f'{version}_{speed}_{mu}_{scenario}';csv=out/(name+'.csv')
        with (out/(name+'.log')).open('w') as log:
            result=subprocess.run([str(root/'build/simulate_handoff'),version,str(speed),str(mu),scenario,str(csv)],stdout=log,stderr=subprocess.STDOUT,timeout=120)
        report=json.loads(Path(str(csv)+'.json').read_text()) if Path(str(csv)+'.json').exists() else {}
        return dict(name=name,exit=result.returncode,report=report)
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:results=list(pool.map(run,cases))
    if any(sha(root/p)!=h for p,h in hashes.items()):raise RuntimeError('Sources changed during comparison')
    candidate=[r for r in results if r['report'].get('mode')=='candidate']
    report=dict(passed=sum(r['exit']==0 and r['report'].get('pass',False) for r in candidate),total=len(candidate),results=results,
        hardware_qualified=False,vision_used=False,fixed_tcp=True,sources_sha256=hashes,executable_sha256=sha(root/'build/simulate_handoff'))
    (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='sources_sha256'},indent=2))
    return 0 if report['passed']==report['total'] else 2

if __name__=='__main__':raise SystemExit(main())
