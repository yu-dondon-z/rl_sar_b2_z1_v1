#!/usr/bin/env python3
"""Local shadow observer testing. No Unitree SDK/robot entrypoint is invoked."""
import argparse
import concurrent.futures
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time

ROOT=Path(__file__).resolve().parent


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output-dir",type=Path,required=True)
    a=p.parse_args();out=a.output_dir.resolve();out.mkdir(parents=True,exist_ok=False)
    cases=[("stand",dict(speed=0,walk_seconds=0))]
    for speed in (.1,.2,.3):
        for phase in (0.,.15):cases.append((f"walk_{speed:.2f}_{phase:.2f}",dict(speed=speed,walk_seconds=8+phase)))
    for friction in (.5,1.):cases.append((f"friction_{friction}",dict(speed=.2,walk_seconds=8,friction=friction)))
    for speed in (.2,.3):cases.append((f"long_{speed}",dict(speed=speed,walk_seconds=30)))
    for fault in ("stale","tau_nan","imu_sign"):cases.append((f"fault_{fault}",dict(speed=.2,walk_seconds=8,fault=fault)))
    env=dict(os.environ,LD_LIBRARY_PATH="/home/dong/rl_sar_b2_z1_old/library/inference_runtime/libtorch/lib",OMP_NUM_THREADS="1",MKL_NUM_THREADS="1")
    def run(case):
        name,options=case
        command=[str(ROOT/"build/simulate_independent_review"),"--model",str(ROOT/"outputs/b2_independent_model.json"),"--output",str(out/(name+".csv"))]
        for key,value in options.items():command.extend(["--"+key.replace("_","-"),str(value)])
        begin=time.monotonic()
        try:
            with (out/(name+".log")).open("x") as log:
                result=subprocess.run(command,stdout=log,stderr=subprocess.STDOUT,env=env,timeout=120,check=False)
            report=json.loads((out/(name+".csv.json")).read_text())
            report.update(name=name,returncode=result.returncode,wall_seconds=time.monotonic()-begin)
        except (subprocess.TimeoutExpired,FileNotFoundError,json.JSONDecodeError) as error:
            report=dict(name=name,offline_review_pass=False,reason=str(error))
        print(json.dumps(report),flush=True);return report
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:reports=list(pool.map(run,cases))
    files=[ROOT/n for n in ("va15_independent_state.hpp","simulate_independent_review.cpp","outputs/b2_independent_model.json")]
    report=dict(control_authorized=False,hardware_qualified=False,observer_in_control_loop=False,
        source_hashes={str(path):hashlib.sha256(path.read_bytes()).hexdigest() for path in files},
        total=len(reports),passed=sum(r["offline_review_pass"] and r.get("returncode")==0 for r in reports),results=reports)
    (out/"summary.json").write_text(json.dumps(report,indent=2)+"\n")
    print("SUMMARY",report["passed"],"/",report["total"],"shadow only; no robot qualification")
    return 0 if report["passed"]==report["total"] else 2


if __name__=="__main__":raise SystemExit(main())
