"""Headless continuous/repeatable session regressions. No robot SDK."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    root=Path(__file__).resolve().parent
    out=root/'outputs'/sys.argv[1];out.mkdir(parents=True,exist_ok=False)
    tracked=[*root.glob('*.cpp'),*(root/'production').glob('*')]
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    sources={str(p.relative_to(root)):sha(p) for p in tracked if p.is_file()}
    cases=[(s,.8) for s in ['repeat','x','b','loss','motor_fault','stale','stick','b_start_same_frame']]
    cases.extend([('repeat',.5),('repeat',1.0)])
    def run(case):
        scenario,mu=case;name=f'{scenario}_mu{mu}';csv=out/(name+'.csv')
        with (out/(name+'.log')).open('w') as log:
            done=subprocess.run([str(root/'build/simulate_sessions'),scenario,str(csv),str(mu)],
                                stdout=log,stderr=subprocess.STDOUT,timeout=180)
        report=json.loads(Path(str(csv)+'.json').read_text()) if Path(str(csv)+'.json').exists() else {}
        return dict(name=name,exit=done.returncode,report=report)
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:results=list(pool.map(run,cases))
    if any(sha(root/p)!=h for p,h in sources.items()):raise RuntimeError('Inputs changed during test')
    summary=dict(passed=sum(r['exit']==0 and r['report'].get('pass',False) for r in results),
                 total=len(results),results=results,sources_sha256=sources,
                 executable_sha256=sha(root/'build/simulate_sessions'),hardware_qualified=False)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(dict(passed=summary['passed'],total=summary['total'],failed=[r for r in results if r['exit']!=0]),indent=2))
    return 0 if summary['passed']==summary['total'] else 2


if __name__=='__main__':raise SystemExit(main())
