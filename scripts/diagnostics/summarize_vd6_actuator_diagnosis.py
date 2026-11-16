#!/usr/bin/env python3
"""Index new diagnostic evidence and verify recording did not alter trajectories."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from analyze_vd6_joint_oscillation import read_rows

ROOT=Path(__file__).resolve().parents[2]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    args=parser.parse_args();root=args.root.resolve()
    original=json.loads((ROOT/'outputs/VD6_REVIEW_20261111/comparison.json').read_text())['fixed']
    diagnostics=json.loads((root/'joint_response_analysis.json').read_text())
    checks=[]
    for r in diagnostics['sessions']:
        m=r['manifest']
        reference=next(o for o in original if
            o['manifest']['engine']==m['engine'] and
            o['manifest'].get('checkpoint_iteration',4498)==m['checkpoint_iteration'] and
            o['manifest']['speed']==m['speed'] and
            o['manifest']['fixed_body_tool_target']==m['fixed_body_tool_target'])
        p=Path(r['path'])
        a=read_rows(p/'physics_truth.jsonl.gz')
        b=read_rows(ROOT/reference['path']/'physics_truth.jsonl.gz')[:len(a)]
        delta=np.max(np.abs(np.array([v['tcp_position_b'] for v in a])-
                            np.array([v['tcp_position_b'] for v in b])))
        response=read_rows(p/'joint_response.jsonl.gz')
        actor=read_rows(p/'actor_trace.jsonl.gz')
        if len(response)!=len(a) or not np.allclose(np.diff([v['time_s'] for v in response]),.005):
            raise ValueError('Inconsistent physics/joint stream: '+str(p))
        # Each new row holds exactly the original 50 Hz command in actor order.
        error=max(float(np.max(np.abs(np.asarray(v['joint_target18_rad'])-
                  np.asarray(actor[i//4]['joint_target18'])))) for i,v in enumerate(response))
        checks.append(dict(session=str(p),original=str(ROOT/reference['path']),steps=len(a),
            tcp_max_abs_difference_m=float(delta),held_target_max_abs_difference_rad=error))
    replays=[]
    for p in sorted(root.glob('replay_*/*/manifest.json')):
        manifest=json.loads(p.read_text())
        summary=p.parent/'summary.json'
        replays.append(dict(path=str(p.parent),manifest=manifest,
            summary=json.loads(summary.read_text()) if summary.exists() else None,
            usable=summary.exists()))
    calibration=[]
    for p in sorted(root.glob('effort_fixture_*.json')):
        d=json.loads(p.read_text())
        usable=d['engine']=='mujoco' or all(c.get('fixed_base') is True for c in d['cases'])
        entry=dict(path=str(p),engine=d['engine'],usable=usable)
        if not usable:
            entry['exclusion_reason']='Development fixture did not assert a truly fixed base. Retained, not used for momentum calibration.'
        else:
            entry['cases']=[]
            for c in d['cases']:
                rows=c['rows']
                x=dict(case=c['case'],first_step=rows[0])
                if d['engine']=='isaac':
                    # Extreme effort stress case is interpreted only over the
                    # first three steps, before large excursions/wrapping.
                    selected=rows[:3] if c['case']=='effort_limit' else rows
                    diff=np.array([v['projected_joint_effort_nm']-v['net_momentum_effort_nm'] for v in selected])
                    x.update(momentum_check_rms_nm=float(np.sqrt(np.mean(diff**2))),
                             momentum_check_steps=len(selected))
                entry['cases'].append(x)
        calibration.append(entry)
    files=['scripts/diagnose_vd5_gait.py','scripts/vd1_mujoco_velocity_impulse.py',
        'scripts/diagnostics/analyze_vd6_joint_oscillation.py','scripts/diagnostics/analyze_vd6_joint_response.py',
        'scripts/diagnostics/probe_vd6_effort_readback.py','scripts/diagnostics/replay_vd6_fixed_arm.py',
        'scripts/diagnostics/summarize_vd6_actuator_diagnosis.py']
    result=dict(unchanged_trajectory_checks=checks,replays=replays,effort_calibration=calibration,
        code_sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in files},
        deployed_policy_sha256=hashlib.sha256((ROOT/'policy/b2_z1_no_gun/mujoco_vd5_startup_4498/policy.pt').read_bytes()).hexdigest(),
        outstanding=['Isolated implicit PhysX drive torque and exact per-joint speed-constraint multipliers are not exposed by installed tensor API; remain null, not PD estimates.',
                     'Closed-loop trigger upstream of joint1/joint6 target oscillations still needs isolation; replay does not identify a root cause.'],
        simulation_only=True,training_started=False,robot_io=False)
    (root/'diagnosis.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('Trajectory checks:',len(checks),'all identical:',all(c['tcp_max_abs_difference_m']==0 and c['held_target_max_abs_difference_rad']==0 for c in checks))
    for r in replays:
        if r['usable']:
            print(Path(r['path']).parent.name,r['manifest']['engine'],r['summary']['tcp_body_hp_axis_rms_mm'])


if __name__=='__main__':
    main()
