#!/usr/bin/env python3
"""Replay original policy inputs and compare stop references, without robot IO.

Measurements/observations are frozen recorded inputs, not a simulation of how
the robot would respond to the changed commands. Original logs remain intact.
"""
import argparse
import gzip
import json
from pathlib import Path
import sys
import numpy as np
import torch
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'python/vision')]
from control import ARM,LEGS,Actor,DEFAULT,Session

def max_step(values):
    return float(np.abs(np.diff(np.asarray(values),axis=0)).max()) if len(values)>1 else 0.

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if args.output is not None and args.output.exists():raise FileExistsError(args.output)
    with gzip.open(args.session/'control.jsonl.gz','rt') as stream:
        rows=[json.loads(line) for line in stream if line.strip()]
    torch.set_num_threads(1);actor=Actor();s=Session();old=[];new=[];raw_error=0.;legs_error=0.;tracking_error=0.
    previous=None;stopping=False
    for row in rows:
        if 'target_q' not in row:continue
        if row['state']=='tracking' and s.mode=='idle':
            if previous is None:raise ValueError('missing pre-tracking reference')
            s.mode='ready';s.target=np.array(previous['target_q'],dtype=float)
            s.event('track',row['monotonic'],np.asarray(row['q']),True)
        if row['state'] in ('tracking','braking') and 'observation' in row:
            if row['state']=='braking' and s.mode=='tracking':
                s.event('stop',row['monotonic'],np.asarray(row['q']));stopping=True
            observation=torch.tensor(row['observation'],dtype=torch.float32)[None]
            with torch.inference_mode():raw=actor.model(observation).numpy()[0]
            raw_error=max(raw_error,float(np.max(np.abs(raw-np.asarray(row['raw_action'])))))
            target=np.clip(DEFAULT+.25*raw,actor.bounds[:,0],actor.bounds[:,1])
            result=s.actor_target(target,row['monotonic'])
            legs_error=max(legs_error,float(np.max(np.abs(result[LEGS]-np.asarray(row['target_q'])[LEGS]))))
            if stopping:old.append(np.asarray(row['target_q'])[ARM]);new.append(result[ARM].copy())
            else:tracking_error=max(tracking_error,float(np.max(np.abs(result-row['target_q']))))
        previous=row
    if not old:raise ValueError('no recorded braking policy steps')
    report=dict(offline_only=True,robot_io_opened=False,session=str(args.session),
        raw_action_max_difference=raw_error,leg_reference_max_difference_rad=legs_error,
        tracking_reference_max_difference_rad=tracking_error,braking_steps=len(old),
        original_braking_arm_max_step_rad=max_step(old),candidate_braking_arm_max_step_rad=max_step(new),
        held_arm_reference_rad=new[0].tolist(),
        limitation='recorded observations held fixed; not actuator, closed-loop stability or TCP accuracy validation')
    print(json.dumps(report,indent=2))
    if raw_error>2e-6 or legs_error>2e-6 or tracking_error>2e-6 or max_step(new)>1e-12:
        raise RuntimeError('recorded reference regression failed')
    if args.output:args.output.write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__':main()
