#!/usr/bin/env python3
"""Local headless MuJoCo ONLY. Never imports Unitree or invokes a real launcher."""
import argparse
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parent
SNAPSHOT=ROOT.parent/'va15_stop_feedback_20261020/work'

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--jobs',type=int,choices=(1,2),default=2)
    args=parser.parse_args()
    out=args.output_dir.resolve()
    out.mkdir(parents=True,exist_ok=False)
    binary=ROOT/'build/simulate_controlled_getdown'
    env=dict(os.environ,LD_LIBRARY_PATH='/home/dong/rl_sar_b2_z1_old/library/inference_runtime/libtorch/lib',
             OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    cases=[]
    def add(name,**options):
        cases.append((name,options))
    add('standby_B',speed=0,walk_seconds=0)
    for speed in (.1,.15,.2,.3):
        for phase in (0.,.15,.3,.45):
            add(f'walk_{speed:.2f}_phase_{phase:.2f}',speed=speed,walk_seconds=8+phase)
    for phase in (.05,.25,.75):
        add(f'handoff_B_{phase:.2f}',speed=.2,walk_seconds=phase)
    for friction in (.5,1.):
        add(f'friction_{friction:.1f}',speed=.2,walk_seconds=8.1,friction=friction)
    for speed in (.2,.3):
        for phase in (0.,.15,.3,.45):
            add(f'payload_1kg_{speed:.2f}_{phase:.2f}',speed=speed,walk_seconds=8+phase,tool_mass=1)
        add(f'long_walk_{speed:.2f}',speed=speed,walk_seconds=30)
    for fault in ('contacts','partial','velocity','stale','stale_after_catch'):
        add(f'fault_{fault}',speed=.2,walk_seconds=8.15,fault=fault)
    add('legacy_reference',speed=.2,walk_seconds=8,legacy=True)

    def run(item):
        name,options=item
        command=[str(binary),'--output',str(out/(name+'.csv'))]
        for key,value in options.items():
            command.append('--'+key.replace('_','-'))
            if not isinstance(value,bool):command.append(str(value))
        started=time.monotonic()
        try:
            with (out/(name+'.log')).open('w') as log:
                result=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT,timeout=120,check=False)
            path=out/(name+'.csv.json')
            report=json.loads(path.read_text()) if path.exists() else dict(success=False,reason='no result JSON')
            report.update(name=name,returncode=result.returncode,wall_s=time.monotonic()-started)
        except subprocess.TimeoutExpired:
            report=dict(name=name,success=False,returncode=124,reason='simulation timed out')
        report['acceptance_case']=name!='legacy_reference'
        print(json.dumps(report,ensure_ascii=False),flush=True)
        return report
    with concurrent.futures.ThreadPoolExecutor(max_workers=args.jobs) as executor:
        results=list(executor.map(run,cases))
    acceptance=[r for r in results if r['acceptance_case']]
    files=[ROOT/'va15_controlled_stop.hpp',ROOT/'fsm_b2_z1_no_gun.hpp',ROOT/'simulate_controlled_getdown.cpp',
           SNAPSHOT/'b2_z1_no_gun/VA15/policy.pt',SNAPSHOT/'b2_z1_no_gun/VA15/config.yaml']
    report=dict(hardware_qualified=False,real_motion_enabled=False,
        passed=sum(r['success'] and r['returncode']==0 for r in acceptance),total=len(acceptance),
        hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in files},results=results)
    (out/'summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(f"SUMMARY: {report['passed']}/{report['total']} candidate cases; hardware NOT qualified",flush=True)
    return 0 if report['passed']==report['total'] else 2

if __name__=='__main__':
    raise SystemExit(main())
