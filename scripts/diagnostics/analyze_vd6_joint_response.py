#!/usr/bin/env python3
"""Summarize 200 Hz diagnostic streams; never equate PD with measured effort."""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.signal import welch
from analyze_vd6_joint_oscillation import read_rows, hp, rms, touchdowns

ARM = [8,13,14,15,16,17]


def bands(values):
    f,p=welch(values,fs=200,nperseg=min(len(values),800),axis=0)
    take=(f>=3)&(f<=95)
    return dict(peak_hz=f[take][np.argmax(p[take],axis=0)].tolist(),
                rms_3_to25=np.sqrt(np.sum(p[(f>=3)&(f<25)],axis=0)*(f[1]-f[0])).tolist(),
                rms_25_to95=np.sqrt(np.sum(p[(f>=25)&(f<=95)],axis=0)*(f[1]-f[0])).tolist())


def report(path):
    manifest=json.loads((path/'manifest.json').read_text())
    schema=json.loads((path/'joint_diagnostic_schema.json').read_text())
    all_rows=read_rows(path/'joint_response.jsonl.gz')
    physics=read_rows(path/'physics_truth.jsonl.gz')
    rows=[r for r in all_rows if 5 <= r['time_s'] <= 15]
    t=np.array([r['time_s'] for r in rows])
    if not len(t) or not np.allclose(np.diff(t),.005,atol=1e-10):
        raise ValueError('Missing/nonuniform 200 Hz response stream: '+str(path))
    q=np.array([r['joint_position18_rad'] for r in rows])[:,ARM]
    q0=np.array([r['joint_position_before18_rad'] for r in rows])[:,ARM]
    v=np.array([r['joint_velocity18_radps'] for r in rows])[:,ARM]
    tar=np.array([r['joint_target18_rad'] for r in rows])[:,ARM]
    displacement_velocity=(q-q0)/.005
    keep=slice(200,-200)
    result=dict(path=str(path),manifest=manifest,schema=schema,samples=len(t),window_s=[5,15],
        q_highpass_rms_rad=rms(hp(q,200)[keep]).tolist(),
        target_highpass_rms_rad=rms(hp(tar,200)[keep]).tolist(),
        q_spectrum=bands(q[keep]),velocity_spectrum=bands(v[keep]),
        displacement_velocity_spectrum=bands(displacement_velocity[keep]),
        velocity_abs_max_radps=np.max(np.abs(v),axis=0).tolist(),
        displacement_velocity_abs_max_radps=np.max(np.abs(displacement_velocity),axis=0).tolist(),
        velocity_vs_displacement_rms_difference_radps=rms(v-displacement_velocity).tolist(),
        velocity_limit_exceed_fraction=np.mean(np.abs(v)>np.asarray(schema['velocity_limit18_radps'])[ARM]+1e-4,axis=0).tolist(),
        held_target_tracking_rms_rad=rms(q-tar).tolist())
    for key in ('drive_effort18_nm','joint_solver_effort18_nm','user_actuation_effort18_nm','speed_constraint_impulse18_nms'):
        if rows[0].get(key) is None:
            result[key]=None
            continue
        x=np.array([r[key] for r in rows])[:,ARM]
        result[key]=dict(rms=rms(x).tolist(),abs_max=np.max(np.abs(x),axis=0).tolist(),
                         nonzero_samples=np.sum(np.abs(x)>1e-10,axis=0).tolist())
    events=[e for e in touchdowns(physics) if 5.2<=e['time_s']<=14.8]
    lags=np.arange(-20,41)*.005
    association={}
    for foot in ('FL','FR','RL','RR'):
        chosen=[e['time_s'] for e in events if e['foot']==foot]
        if not chosen:
            continue
        points=np.array(chosen)
        average=np.array([[np.mean(np.interp(points+lag,t,np.abs(v[:,j]))) for j in range(6)] for lag in lags])
        association[foot]=dict(count=len(chosen),lags_s=lags.tolist(),mean_abs_velocity_radps=average.tolist(),
                               peak_lags_s=lags[np.argmax(average,axis=0)].tolist())
    result['touchdown_phase_association']=association
    return result,(t,q,v,displacement_velocity,tar,physics)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,required=True)
    args=parser.parse_args()
    reports,traces=[],[]
    for file in sorted(args.root.glob('vd*_fixed*/*/joint_response.jsonl.gz')):
        r,trace=report(file.parent)
        reports.append(r);traces.append(trace)
        print(r['manifest']['engine'],r['manifest']['checkpoint_iteration'],
              -r['manifest']['fixed_body_tool_target'][0],
              'q6 peak',r['q_spectrum']['peak_hz'][5],
              'qd6 peak',r['velocity_spectrum']['peak_hz'][5],
              'qd6 max',round(r['velocity_abs_max_radps'][5],3))
    result=dict(sessions=reports,arm_joint_order=['joint'+str(i) for i in range(1,7)],
        notes=['Window 5..15 s is a short diagnostic, not a replacement for the completed 5..30 s evaluation.',
               'PhysX drive-only effort and exact speed constraint activity remain null when not exposed.',
               'MuJoCo native force buffers saved before forward recomputation, not post-step re-evaluation.',
               'State velocity and displacement/dt are reported separately; neither is substituted into observations.',
               'Event averages show gait phase associations, not causality.'])
    (args.root/'joint_response_analysis.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(len(reports),3,figsize=(16,2.6*len(reports)),squeeze=False)
    for i,(r,trace) in enumerate(zip(reports,traces)):
        t,q,v,fd,tar,physics=trace
        # Largest RL touchdown in a common admissible interval.
        events=[e for e in touchdowns(physics) if e['foot']=='RL' and 6<=e['time_s']<=14]
        center=max(events,key=lambda e:e['peak_force_n'])['time_s']
        take=(t>=center-.15)&(t<=center+.3)
        axes[i,0].plot(t[take],q[take,5],label='q6')
        axes[i,0].plot(t[take],tar[take,5],label='held target6')
        axes[i,1].plot(t[take],v[take,5],label='runtime qd6')
        axes[i,1].plot(t[take],fd[take,5],label='delta q6 / dt')
        pt=np.array([x['time_s'] for x in physics])
        f=np.array([x['foot_force_w'][2][2] for x in physics])
        take_p=(pt>=center-.15)&(pt<=center+.3)
        axes[i,2].plot(pt[take_p],f[take_p],label='RL vertical force N')
        m=r['manifest']
        axes[i,0].set_ylabel('%s %s %.2fm'%(m['engine'],m['checkpoint_iteration'],-m['fixed_body_tool_target'][0]))
        for ax in axes[i]:
            ax.axvline(center,color='gray');ax.legend(fontsize=7);ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(args.root/'joint_response_touchdown.png',dpi=150)


if __name__=='__main__':
    main()
