#!/usr/bin/env python3
"""Offline adjacent-observation actor attribution; never changes a rollout.

Line-integrated gradients decompose target(t)-target(t-20ms). The path between
two measured observations is a mathematical attribution path, not a physically
realizable intervention. Signed covariance shares are not causal percentages.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import tempfile

import numpy as np
import torch

from analyze_vd6_joint_oscillation import read_rows, touchdowns

ROOT = Path(__file__).resolve().parents[2]
ARM = [8, 13, 14, 15, 16, 17]
LEG = [i for i in range(18) if i not in ARM]
OUT = [8, 17]
JOINTS = ['FL_hip','FR_hip','RL_hip','RR_hip','FL_thigh','FR_thigh',
          'RL_thigh','RR_thigh','joint1','FL_calf','FR_calf','RL_calf',
          'RR_calf','joint2','joint3','joint4','joint5','joint6']
FEATURES = (['command_vx','command_vy','command_wz'] +
            ['tool_'+s for s in ('x','y','z','qw','qx','qy','qz')] +
            ['gyro_'+s for s in ('x','y','z')] + ['gravity_'+s for s in ('x','y','z')] +
            ['q_'+n for n in JOINTS] + ['qd_'+n for n in JOINTS] + ['previous_'+n for n in JOINTS])
GROUPS = dict(command=list(range(10)), gyro=list(range(10,13)), gravity=list(range(13,16)),
              leg_position=[16+i for i in LEG], arm_position=[16+i for i in ARM],
              leg_velocity=[34+i for i in LEG], arm_velocity=[34+i for i in ARM],
              previous_leg_action=[52+i for i in LEG], previous_arm_action=[52+i for i in ARM])
BASELINE = Path('/home/dong/b2_z1_VD2/logs/rsl_rl/b2_z1_vd5_startup_amp/'
                '2026-11-10_11-47-57_vd5_startup_80ref_20stand_continue1000/model_4498.pt')


def load_actor(path):
    checkpoint = torch.load(path, map_location='cpu', weights_only=False)
    model = torch.nn.Sequential(torch.nn.Linear(70,512),torch.nn.ELU(),
        torch.nn.Linear(512,256),torch.nn.ELU(),torch.nn.Linear(256,128),
        torch.nn.ELU(),torch.nn.Linear(128,18)).eval()
    model.load_state_dict({k[6:]:v for k,v in checkpoint['model_state_dict'].items() if k.startswith('actor.')})
    model.requires_grad_(False)
    return model


def integrated_change(model, previous, current, nodes=16):
    """Gauss-Legendre path integral for selected output targets, in radians."""
    points, weights = np.polynomial.legendre.leggauss(nodes)
    delta = current-previous
    total = torch.zeros((len(current),len(OUT),70),dtype=current.dtype)
    for alpha,weight in zip((points+1)/2,weights/2):
        x = (previous+float(alpha)*delta).detach().requires_grad_(True)
        target = model(x)[:,OUT]*.25
        for j in range(len(OUT)):
            gradient = torch.autograd.grad(target[:,j].sum(),x,retain_graph=j<len(OUT)-1)[0]
            total[:,j] += float(weight)*gradient*delta
    return total.detach().numpy()


def summarize_contributions(contributions, change, names):
    energy = np.mean(change**2,axis=0)
    rms = np.sqrt(np.mean(contributions**2,axis=0))
    share = np.mean(contributions*change[:,:,None],axis=0)/energy[:,None]
    return {name:dict(rms_rad=rms[:,j].tolist(),signed_covariance_share=share[:,j].tolist())
            for j,name in enumerate(names)}


def analyze(session, output, new_session):
    manifest = json.loads((session/'manifest.json').read_text())
    iteration = manifest.get('checkpoint_iteration',4498)
    policy = Path(manifest['policy'])
    if hashlib.sha256(policy.read_bytes()).hexdigest()!=manifest['policy_sha256']:
        raise ValueError('Policy fingerprint mismatch')
    model = load_actor(BASELINE if iteration==4498 else policy)
    actor = [r for r in read_rows(session/'actor_trace.jsonl.gz') if r['time_s']<=30]
    times = np.array([r['time_s'] for r in actor])
    obs = np.array([r['observation'] for r in actor],dtype=np.float32)
    actions = np.array([r['action'] for r in actor])
    with torch.no_grad():
        prediction = model(torch.from_numpy(obs)).numpy()
    reproduction = float(np.max(np.abs(prediction-actions)))
    if reproduction>1e-5:
        raise ValueError('Actor reproduction failed: '+str(reproduction))
    previous_error = float(np.max(np.abs(obs[1:,52:]-np.clip(actions[:-1],-100,100))))
    if previous_error>1e-6:
        raise ValueError('Previous action alignment failed')
    use = np.where((times>=5)&(times<=30))[0]
    t = times[use]
    double = copy.deepcopy(model).double()
    x0=torch.tensor(obs[use-1],dtype=torch.float64)
    x1=torch.tensor(obs[use],dtype=torch.float64)
    with torch.no_grad():
        change=(double(x1)[:,OUT]-double(x0)[:,OUT]).numpy()*.25
    c8=integrated_change(double,x0,x1,8)
    c16=integrated_change(double,x0,x1,16)
    convergence=float(np.max(np.abs(c16-c8)))
    nodes=16
    while (convergence>2e-6 or np.max(abs(change-c16.sum(axis=2)))>2e-6) and nodes<256:
        nodes*=2
        refined=integrated_change(double,x0,x1,nodes)
        convergence=float(np.max(np.abs(refined-c16)))
        c16=refined
    residual=change-c16.sum(axis=2)
    if np.max(np.abs(residual))>1e-5 or convergence>1e-5:
        raise ValueError('Attribution quadrature failed to converge: residual=%g convergence=%g' %
                         (np.max(np.abs(residual)),convergence))
    grouped=np.stack([c16[:,:,indices].sum(axis=2) for indices in GROUPS.values()],axis=2)
    # Same physical units in both policies/engines; finite differences are
    # offline sensitivity probes, not substitutions into the simulated actor.
    probes = {'gyro_x_0.01radps':(10,.002),'gyro_y_0.01radps':(11,.002),
              'gyro_z_0.01radps':(12,.002),'q_joint1_0.001rad':(24,.001),
              'q_joint6_0.001rad':(33,.001),'qd_joint1_0.1radps':(42,.005),
              'qd_joint6_0.1radps':(51,.005),'previous_joint1_0.01action':(60,.01),
              'previous_joint6_0.01action':(69,.01)}
    sensitivity={}
    for name,(index,epsilon) in probes.items():
        plus=x1.clone();minus=x1.clone()
        plus[:,index]+=epsilon;minus[:,index]-=epsilon
        with torch.no_grad():
            effect=(double(plus)[:,OUT]-double(minus)[:,OUT]).numpy()*.125
        sensitivity[name]=dict(target_effect_rms_rad=np.sqrt(np.mean(effect**2,axis=0)).tolist(),
                               target_effect_abs_p95_rad=np.percentile(abs(effect),95,axis=0).tolist())
    physics=read_rows(session/'physics_truth.jsonl.gz')
    events=[e for e in touchdowns(physics) if 5.2<=e['time_s']<=29.7]
    event_results={}
    offsets=np.arange(-5,11)
    for foot in ('FL','FR','RL','RR'):
        chosen=[e for e in events if e['foot']==foot]
        indices=np.searchsorted(t,[e['time_s']-1e-9 for e in chosen])
        valid=(indices+offsets[0]>=0)&(indices+offsets[-1]<len(t))
        indices=indices[valid]
        if not len(indices):
            continue
        samples=indices[:,None]+offsets[None,:]
        event_results[foot]=dict(count=len(indices),
            definition='First actor sample at or after physical touchdown; raw increments, no filtering/interpolation.',
            offsets_from_first_actor_sample_s=(offsets*.02).tolist(),
            touchdown_to_first_actor_delay_s=(t[indices]-np.array([e['time_s'] for e in chosen])[valid]).tolist(),
            mean_abs_target_delta_rad=np.mean(abs(change[samples]),axis=0).tolist(),
            mean_abs_group_contribution_rad=np.mean(abs(grouped[samples]),axis=0).tolist())
    # Inspect high-rate speed spikes at exactly the samples the actor consumed.
    highrate=read_rows(new_session/'joint_response.jsonl.gz')
    highrate=[r for r in highrate if 5<=r['time_s']<15]
    joint_t=np.array([r['time_s'] for r in highrate])
    v=np.array([r['joint_velocity18_radps'][17] for r in highrate])
    sampled=(np.rint(joint_t/.005).astype(int)%4)==0
    ai=np.rint(joint_t[sampled]/.02).astype(int)
    actor_v=obs[ai,51]/.05
    alignment=float(np.max(abs(actor_v-v[sampled])))
    if alignment>5e-5:
        raise ValueError('200 Hz / actor velocity mismatch')
    spike=np.abs(v)>3.1415+1e-4
    # Match each actually observed spike to its adjacent-observation attribution.
    spike_times=joint_t[sampled][spike[sampled]]
    spike_i=np.searchsorted(t,spike_times-1e-9)
    spike_stats=dict(window_s=[5,15],physical_samples=len(v),physical_over_limit_samples=int(spike.sum()),
        actor_samples=int(sampled.sum()),actor_over_limit_samples=int(spike[sampled].sum()),
        actor_velocity_alignment_max_abs_radps=alignment,
        sampled_abs_max_radps=float(np.max(abs(v[sampled]))),
        physical_abs_max_radps=float(np.max(abs(v))))
    if len(spike_i):
        spike_stats['observed_spike_qd6_attribution_rms_rad']=np.sqrt(np.mean(c16[spike_i,:,51]**2,axis=0)).tolist()
        spike_stats['observed_spike_target_delta_rms_rad']=np.sqrt(np.mean(change[spike_i]**2,axis=0)).tolist()
    # Individual largest target changes, with actual nearby touchdown times.
    notable=[]
    for j in range(2):
        for i in np.argsort(abs(change[:,j]))[-5:][::-1]:
            top=np.argsort(abs(c16[i,j]))[-6:][::-1]
            preceding=[e for e in events if 0<=t[i]-e['time_s']<=.3]
            notable.append(dict(output=JOINTS[OUT[j]],time_s=float(t[i]),target_delta_rad=float(change[i,j]),
                top_features=[dict(name=FEATURES[k],contribution_rad=float(c16[i,j,k])) for k in top],
                preceding_touchdowns=preceding))
    label=f'{iteration}_{manifest["engine"]}_{-manifest["fixed_body_tool_target"][0]:.2f}'
    np.savez_compressed(output/(label+'.npz'),time_s=t,target_delta_rad=change,
                        feature_contribution_rad=c16,group_contribution_rad=grouped,
                        observation=obs[use],feature_names=np.array(FEATURES),group_names=np.array(list(GROUPS)))
    return dict(label=label,source_session=str(session),highrate_session=str(new_session),manifest=manifest,
        checkpoint_used=str(BASELINE if iteration==4498 else policy),
        checkpoint_sha256=hashlib.sha256((BASELINE if iteration==4498 else policy).read_bytes()).hexdigest(),
        window_s=[5,30],output_joints=['joint1','joint6'],samples=len(t),
        action_reproduction_max_abs=reproduction,previous_action_alignment_max_abs=previous_error,
        integration_nodes=nodes,quadrature_convergence_max_rad=convergence,
        attribution_sum_max_abs_error_rad=float(np.max(abs(residual))),
        target_delta_rms_rad=np.sqrt(np.mean(change**2,axis=0)).tolist(),
        groups=summarize_contributions(grouped,change,list(GROUPS)),
        features=summarize_contributions(c16,change,FEATURES),physical_unit_sensitivity=sensitivity,
        touchdown_association=event_results,velocity_spikes=spike_stats,largest_changes=notable)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-root',type=Path,default=ROOT/'outputs/VD6_ACTOR_FEEDBACK_20261111')
    args=parser.parse_args();torch.set_num_threads(1)
    args.output_root.mkdir(parents=True,exist_ok=True)
    output=Path(tempfile.mkdtemp(prefix='analysis_',dir=args.output_root))
    new=json.loads((ROOT/'outputs/VD6_ACTUATOR_DIAGNOSIS_20261111/diagnosis.json').read_text())['unchanged_trajectory_checks']
    sessions=[]
    for pair in new:
        session=Path(pair['original'])
        result=analyze(session,output,ROOT/pair['session'])
        sessions.append(result)
        print(result['label'],'rms',result['target_delta_rms_rad'],'groups',
              sorted([(k,v['signed_covariance_share'][1]) for k,v in result['groups'].items()],key=lambda x:-x[1]),flush=True)
        (output/'analysis.json').write_text(json.dumps(dict(sessions=sessions,complete=False),indent=2,allow_nan=False)+'\n')
    report=dict(sessions=sessions,complete=True,simulation_run=False,actor_feedback_modified=False,
        notes=['Adjacent actual observations connected by a straight path; path is mathematical, not a real intervention.',
               'Signed covariance shares partition target INCREMENT energy; they are not causal contributions to TCP RMS.',
               'Previous-action input is downstream of earlier observations and must not be treated as an independent cause.',
               'Raw event timing uses first actor sample after contact; no acausal filtering.',
               '5..30 s attribution; 200 Hz spike audit restricted to available 5..15 s recordings.'],
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (output/'analysis.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('OUTPUT',output,flush=True)


if __name__=='__main__':
    main()
