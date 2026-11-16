#!/usr/bin/env python3
"""Summarize finite VD6 evaluation artifacts without changing any policy."""
import argparse
import gzip
import json
from pathlib import Path
import sys
import numpy as np
from scipy.signal import butter, sosfiltfilt

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vd5_gait_metrics import summarize


def stats(values):
    a = np.asarray(values)
    return dict(mean=float(np.mean(a)), p95=float(np.percentile(a,95)), max=float(np.max(a)))


def task_report(path):
    d = json.loads(path.read_text())
    rows, ended = [], set()
    for row in d['samples']:
        if row['env'] in ended:
            continue
        if row['terminated']:
            ended.add(row['env'])
            continue
        if row['episode_step'] > 100:
            rows.append(row)
    metrics = {k:stats([r[k] for r in rows]) for k in (
        'tcp_seam_projection_error_m','base_cross_track_m','actual_forward_speed_mps',
        'command_forward_speed_mps','tilt_deg','tcp_error_m')}
    feet = np.array([r['foot_seam_center_distances_m'] for r in rows])
    contact = np.array([r['foot_ground_contact_gt10n'] for r in rows])
    metrics['support_foot_inside52mm_fraction'] = float(np.sum((feet<.052)&contact)/np.sum(contact))
    metrics['tcp_projection_over50mm_fraction'] = float(np.mean([r['tcp_seam_projection_error_m']>.05 for r in rows]))
    return dict(path=str(path),checkpoint=d['checkpoint'],seed=d['seed'],num_envs=d['num_envs'],
        steps=d['steps'],termination_counts=d['termination_counts'],metrics=metrics,
        rewards=d.get('vd6_reward_activation'),first_episode_after2s_only=True)


def fixed_report(path):
    with gzip.open(str(path/'physics_truth.jsonl.gz'),'rt') as f:
        physics = [r for r in map(json.loads,f) if r['time_s'] <= 30.]
    with gzip.open(str(path/'actor_trace.jsonl.gz'),'rt') as f:
        actions = [r for r in map(json.loads,f) if r['time_s'] <= 30.]
    result = summarize(physics, actions)
    settled = [r for r in physics if r['time_s']>=5.]
    for key in ('tcp_position_b','tcp_position_w'):
        xyz = np.array([r[key] for r in settled])
        hp = sosfiltfilt(butter(3,3.,fs=200,btype='highpass',output='sos'),xyz,axis=0)[200:-200]
        result[key+'_highpass_axis_rms_m'] = np.sqrt(np.mean(hp**2,axis=0)).tolist()
    feet = np.array([r['foot_position_w'] for r in settled])
    velocity = np.array([r['foot_velocity_w'] for r in settled])
    contact = np.array([r['foot_force_w'] for r in settled])[:,:,2]>10.
    result['feet'] = {}
    for j,name in enumerate(('FL','FR','RL','RR')):
        result['feet'][name] = dict(contact_fraction=float(contact[:,j].mean()),
            flight_clearance_p95_m=float(np.percentile(feet[~contact[:,j],j,2]-.032,95)),
            support_horizontal_slip_p95_mps=float(np.percentile(np.linalg.norm(velocity[contact[:,j],j,:2],axis=1),95)))
    return dict(path=str(path),manifest=json.loads((path/'manifest.json').read_text()),
        reason=json.loads((path/'summary.json').read_text())['reason'],
        equal_window_s=[5,30],metrics=result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task-root',type=Path,required=True)
    parser.add_argument('--fixed',type=Path,nargs='*',default=[])
    parser.add_argument('--extra-task',type=Path,nargs='*',default=[])
    parser.add_argument('--output',type=Path,required=True)
    a = parser.parse_args()
    result = dict(task=[task_report(p) for p in [*sorted(a.task_root.glob('model_*_seed*.json')), *a.extra_task]],
        fixed=[fixed_report(p) for p in a.fixed],
        note='Task: nominal resets, first episode after 2 s; fixed-command: equal 5..30 s window. No reward-only ranking.')
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    for r in result['task']:
        m = r['metrics']
        print(Path(r['path']).stem,dict(tcp_p95_mm=round(m['tcp_seam_projection_error_m']['p95']*1000,2),
            vx=round(m['actual_forward_speed_mps']['mean'],4),base_cross_p95_m=round(m['base_cross_track_m']['p95'],4),
            foot_seam_fraction=round(m['support_foot_inside52mm_fraction'],4),terminations=r['termination_counts']))
    for r in result['fixed']:
        m = r['metrics']
        print(r['manifest'].get('checkpoint_iteration',4498),r['manifest']['engine'],
            dict(work_distance_m=-r['manifest']['fixed_body_tool_target'][0],
                 tcp_highpass_xyz_mm=np.round(np.array(m['tcp_position_b_highpass_axis_rms_m'])*1000,2).tolist(),
                 vx=round(m['base_speed_mean_mps'],4)))
    print(a.output)


if __name__ == '__main__':
    main()
