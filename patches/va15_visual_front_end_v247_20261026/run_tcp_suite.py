"""SDK-free physics acceptance: synthetic RGB + independent IMU/joint estimator."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import subprocess
import sys

def main():
    root=Path(__file__).resolve().parent
    name=sys.argv[1] if len(sys.argv)==2 else 'tcp_acceptance'
    if Path(name).name!=name:raise SystemExit('One new output directory name required')
    out=root/'outputs'/name;out.mkdir(parents=True,exist_ok=False)
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    tracked=[*(root/'production').glob('*'),root/'simulate_tcp.cpp',root/'run_tcp_suite.py']
    hashes={str(p.relative_to(root)):sha(p) for p in tracked if p.is_file()}
    cases=[('estimated_curve',mu) for mu in [.5,.8,1.]]
    cases += [(n,.8) for n in ['estimated_straight','estimated_adaptive','estimated_dropout','estimated_loss','estimated_pose_loss']]
    def run(case):
        scenario,mu=case;name=f'{scenario}_mu{mu}';csv=out/(name+'.csv')
        with (out/(name+'.log')).open('w') as log:
            result=subprocess.run([str(root/'build/simulate_tcp'),scenario,str(csv),str(mu)],
                                  stdout=log,stderr=subprocess.STDOUT,timeout=180)
        report=json.loads(Path(str(csv)+'.json').read_text()) if Path(str(csv)+'.json').exists() else {'pass':False}
        return dict(name=name,exit=result.returncode,report=report)
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:results=list(pool.map(run,cases))
    if any(sha(root/p)!=h for p,h in hashes.items()):raise RuntimeError('Sources changed during tests')
    report=dict(passed=sum(r['exit']==0 and r['report']['pass'] for r in results),total=len(results),results=results,
                sources_sha256=hashes,executable_sha256=sha(root/'build/simulate_tcp'),hardware_qualified=False,
                precision_qualified=False,synthetic_rgb=True,independent_estimated_pose=True)
    (out/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ['sources_sha256','executable_sha256']},indent=2))
    return 0 if report['passed']==report['total'] else 2

if __name__=='__main__':raise SystemExit(main())
