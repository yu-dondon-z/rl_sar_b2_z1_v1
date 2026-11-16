#!/usr/bin/env python3
"""SDK-free headless input-duration benchmark; NEVER runs a real binary."""
import concurrent.futures
import argparse
import json
from pathlib import Path
import subprocess

def main():
    root=Path(__file__).resolve().parent
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--speeds',nargs='+',type=float,default=[.07,.1,.2,.3])
    p.add_argument('--durations',nargs='+',type=float,default=[2.,8.])
    p.add_argument('--output-name',default='offline_duration_benchmark1')
    args=p.parse_args()
    if Path(args.output_name).name!=args.output_name:raise SystemExit('One new output directory name required')
    out=root/'outputs'/args.output_name
    out.mkdir(parents=True,exist_ok=False)
    cases=[(speed,duration) for speed in args.speeds for duration in args.durations]
    def run(case):
        speed,duration=case;name=f'speed{speed:.2f}_seconds{duration:.0f}'
        csv=out/(name+'.csv')
        with (out/(name+'.log')).open('w') as log:
            result=subprocess.run([str(root/'build/simulate_trial'),
                str(root/'production/b2_independent_model.json'),str(csv),'steady','.8','1.',str(speed),str(duration)],
                stdout=log,stderr=subprocess.STDOUT,timeout=120)
        report_path=Path(str(csv)+'.json')
        report=json.loads(report_path.read_text()) if report_path.exists() else {}
        return dict(name=name,exit=result.returncode,report=report)
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
        results=list(pool.map(run,cases))
    summary=dict(offline_only=True,real_supervisor_unchanged=True,hardware_qualified=False,results=results)
    (out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    for r in results:
        p=r['report']
        print(r['name'],'checks=',p.get('pass'),'forward_m=',p.get('forward_at_request_end_m'),
              'air_s=',p.get('foot_air_time_s_FL_FR_RL_RR'),'fell=',p.get('fell'))

if __name__=='__main__':main()
