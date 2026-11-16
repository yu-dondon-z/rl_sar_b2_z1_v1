#!/usr/bin/env python3
"""Replay retained commands/feedback only. Never opens native or camera IO."""
import argparse
import gzip
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',required=True,type=Path)
    parser.add_argument('--bundle',type=Path,default=Path(__file__).resolve().parents[1])
    args=parser.parse_args()
    sys.path[:0]=[str(args.bundle/'python'),str(args.bundle/'python/vision')]
    from control import ARM,LEGS,Session
    from feedback import Z1MotorResponse
    with gzip.open(args.session/'control.jsonl.gz','rt') as stream:
        rows=[json.loads(line) for line in stream if line.strip()]
    rows=[r for r in rows if 'target_q' in r]
    first=next(i for i,r in enumerate(rows) if r['state']=='tracking')
    previous=rows[first-1];start=rows[first]['monotonic']
    session=Session();session.mode='ready';session.target=np.array(previous['target_q'])
    session.event('track',start,np.array(previous['q']),True)
    monitor=Z1MotorResponse();blocked=None;maximum=0.;old_maximum=0.;leg_difference=0.;count=0
    for index,r in enumerate(rows[first:],first):
        now=r['monotonic'];prior=session.target.copy()
        state=SimpleNamespace(**{name:r[name] for name in ('q','dq','tau')})
        feedback=monitor.observe(state,now)
        if feedback['z1_motor_vector_repeated_while_moving']:
            session.block_motor_feedback(now)
            if blocked is None:blocked=now-start
        if session.mode=='tracking' and r['state']=='braking':session.event('stop',now,np.array(r['q']))
        if session.mode in ('tracking','braking') and 'actor_requested_target_q' in r:
            session.actor_target(np.array(r['actor_requested_target_q']),now);count+=1
            leg_difference=max(leg_difference,float(np.max(np.abs(session.target[LEGS]-np.array(r['target_q'])[LEGS]))))
        maximum=max(maximum,float(np.max(np.abs(session.target[ARM]-prior[ARM]))))
        if r['state']=='tracking':
            old_maximum=max(old_maximum,float(np.max(np.abs(np.array(r['target_q'])[ARM]-np.array(rows[index-1]['target_q'])[ARM]))))
    assert maximum<=.020000001,(maximum,'reference slew violation')
    assert leg_difference<1e-9,(leg_difference,'leg references changed before inhibit')
    assert blocked is not None,'input does not reproduce the moving-feedback freeze'
    print(json.dumps(dict(offline_only=True,closed_loop_validation=False,
        input_session=str(args.session),old_max_arm_step_rad=old_maximum,
        candidate_max_arm_step_rad=maximum,leg_reference_difference_before_inhibit=leg_difference,
        inhibit_s_since_tracking=blocked,actor_reference_steps_before_inhibit=count,
        note='Recorded-input replay; changed references would change real feedback. Does not prove UDP loss or falls resolved.'),indent=2))


if __name__=='__main__':main()
