#!/usr/bin/env python3
"""Postprocess saved attribution; compare actor derivatives on identical inputs."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import torch

from analyze_vd6_actor_feedback import (ROOT, BASELINE, OUT, GROUPS, FEATURES,
    load_actor, summarize_contributions)
from analyze_vd6_joint_oscillation import hp, read_rows, touchdowns


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    args=parser.parse_args();directory=args.directory.resolve()
    data=json.loads((directory/'analysis.json').read_text())
    if not data['complete']:
        raise ValueError('Attribution is not complete')
    torch.set_num_threads(1)
    checkpoint=next(r['checkpoint_used'] for r in data['sessions'] if r['manifest'].get('checkpoint_iteration')==4997)
    models={4498:load_actor(BASELINE).double(),4997:load_actor(checkpoint).double()}
    # Scaling converts observation derivatives to physical input units.
    probes={10:('gyro_x',.2),24:('q_joint1',1.),33:('q_joint6',1.),
            42:('qd_joint1',.05),51:('qd_joint6',.05),
            41:('qd_RR_thigh',.05),40:('qd_RL_thigh',.05),
            60:('previous_joint1',1.),69:('previous_joint6',1.)}
    summaries=[]
    checks=[]
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(4,2,figsize=(16,14),squeeze=False)
    selected=[]
    for r in data['sessions']:
        trace=np.load(directory/(r['label']+'.npz'))
        change=trace['target_delta_rad'];features=trace['feature_contribution_rad']
        # Integrating attributed increments reconstructs the target up to a
        # constant. High-pass only used for energy attribution, never timing.
        actual=hp(np.cumsum(change,axis=0),50)[50:-50]
        components=hp(np.cumsum(features,axis=0),50)[50:-50]
        grouped=np.stack([components[:,:,ids].sum(axis=2) for ids in GROUPS.values()],axis=2)
        recorded=[v for v in read_rows(Path(r['source_session'])/'actor_trace.jsonl.gz')
                  if 5<=v['time_s']<=30]
        recorded_target=np.asarray([v['joint_target18'] for v in recorded])[:,OUT]
        target_error=float(np.max(abs(hp(recorded_target,50)[50:-50]-actual)))
        if target_error>3e-6:
            raise ValueError('Highpass target reconstruction disagrees with original target log')
        checks.append(dict(label=r['label'],recorded_target_highpass_max_abs_difference_rad=target_error,
            increment_contribution_sum_max_abs_error_rad=float(np.max(abs(features.sum(2)-change))),
            signed_covariance_share_sum=np.sum([v['signed_covariance_share'] for v in r['groups'].values()],axis=0).tolist()))
        entry=dict(label=r['label'],target_highpass_rms_rad=np.sqrt(np.mean(actual**2,axis=0)).tolist(),
            highpass_features=summarize_contributions(components,actual,FEATURES),
            highpass_groups=summarize_contributions(grouped,actual,list(GROUPS)),
            highpass_reconstruction_max_error_rad=float(np.max(abs(components.sum(2)-actual))),
            same_input_derivatives={})
        if entry['highpass_reconstruction_max_error_rad']>1e-5:
            raise ValueError('Accumulated contribution reconstruction failed')
        for iteration,model in models.items():
            x=torch.tensor(trace['observation'],dtype=torch.float64,requires_grad=True)
            y=model(x)[:,OUT]*.25
            gradient=torch.stack([torch.autograd.grad(y[:,j].sum(),x,retain_graph=j==0)[0]
                                  for j in range(2)],dim=1).detach().numpy()
            entry['same_input_derivatives'][str(iteration)]={name:dict(
                median_target_rad_per_input_unit=np.median(gradient[:,:,k]*scale,axis=0).tolist(),
                rms_target_rad_per_input_unit=np.sqrt(np.mean((gradient[:,:,k]*scale)**2,axis=0)).tolist())
                for k,(name,scale) in probes.items()}
        # Record one complete event window at the largest RL impact in 5..15 s.
        physics=read_rows(Path(r['source_session'])/'physics_truth.jsonl.gz')
        events=[e for e in touchdowns(physics) if e['foot']=='RL' and 6<e['time_s']<14]
        e=max(events,key=lambda e:e['peak_force_n']);t=trace['time_s']
        take=(t>=e['time_s']-.12)&(t<=e['time_s']+.22)
        entry['largest_RL_impact_window']=dict(event=e,time_s=t[take].tolist(),
            target_delta_rad=change[take].tolist(),
            qd1_attribution_rad=features[take,:,42].tolist(),qd6_attribution_rad=features[take,:,51].tolist(),
            q1_attribution_rad=features[take,:,24].tolist(),
            gyro_x_attribution_rad=features[take,:,10].tolist())
        summaries.append(entry)
        if r['label'].endswith('0.80'):
            selected.append((r,trace,entry))
    for row,(r,trace,entry) in enumerate(selected):
        t=trace['time_s'];event=entry['largest_RL_impact_window']['event']
        take=(t>=event['time_s']-.12)&(t<=event['time_s']+.22)
        for j,ax in enumerate(axes[row]):
            ax.plot(t[take],trace['target_delta_rad'][take,j],color='black',label='actual target change')
            for k in (24,42,51,10):
                ax.plot(t[take],trace['feature_contribution_rad'][take,j,k],label=FEATURES[k],alpha=.85)
            ax.axvline(event['time_s'],color='gray',linestyle='--',label='RL touchdown')
            ax.set_title(r['label']+' / '+['joint1','joint6'][j]);ax.set_ylabel('target change rad / 20 ms')
            ax.set_xlabel('time s');ax.grid(alpha=.2);ax.legend(fontsize=7)
    fig.tight_layout();fig.savefig(directory/'actor_touchdown_attribution.png',dpi=160)
    result=dict(sessions=summaries,notes=[
        'Highpass energy decomposition uses cumulative adjacent-input attribution and excludes 1 s at both edges.',
        'Identical recorded observations evaluated by both actors isolate local policy derivative differences; off-policy inputs are not a physical rollout.',
        'Signed contributions can cancel or exceed 100%; neither attribution nor derivatives establish closed-loop causality.'])
    (directory/'highpass_and_sensitivity.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    sources=['scripts/diagnostics/analyze_vd6_actor_feedback.py','scripts/diagnostics/report_vd6_actor_feedback.py',
             'tests/test_vd6_actor_feedback.py']
    audit=dict(checks=checks,code_sha256={f:hashlib.sha256((ROOT/f).read_bytes()).hexdigest() for f in sources},
        policy_sha256=hashlib.sha256((ROOT/'policy/b2_z1_no_gun/mujoco_vd5_startup_4498/policy.pt').read_bytes()).hexdigest(),
        source_logs_sha256={str(Path(r['source_session'])/'actor_trace.jsonl.gz'):
            hashlib.sha256((Path(r['source_session'])/'actor_trace.jsonl.gz').read_bytes()).hexdigest()
            for r in data['sessions']},
        note='Hashes identify files consumed; no source logs or policies were written by this analysis.')
    (directory/'validation.json').write_text(json.dumps(audit,indent=2,allow_nan=False)+'\n')
    for r in summaries:
        print(r['label'],'HF RMS',np.round(r['target_highpass_rms_rad'],5),flush=True)
        print('joint6 HF top',sorted([(k,round(v['signed_covariance_share'][1],3))
              for k,v in r['highpass_features'].items()],key=lambda x:-x[1])[:6],flush=True)


if __name__=='__main__':
    main()
