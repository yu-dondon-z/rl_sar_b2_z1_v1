#!/usr/bin/env python3
"""Offline-only transport accuracy using recorded evaluation poses, NEVER control.

Compares the exact same image plane with/without compensation at its actual
control tick. Simulator poses are read ONLY here, not by the sensor estimator.
Supports the explicit stationary nominal reset of deploy_vd14_vision.py.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys
import tempfile

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from vd5_mujoco_control import quaternion_matrix


def jsonlines(path):
    opener = gzip.open if str(path).endswith('.gz') else open
    with opener(path, 'rt') as stream:
        return [json.loads(line) for line in stream]


def distribution(values):
    a = np.asarray(values, dtype=float)
    if not a.size:
        return None
    if not np.isfinite(a).all():
        raise ValueError('Nonfinite evaluation metric')
    return dict(count=int(a.size), median=float(np.median(a)), p95=float(np.percentile(a,95)),
                maximum=float(np.max(a)), rms=float(np.sqrt(np.mean(a*a))))


def plane_height(plane_b, rotation, position, xy):
    normal = rotation @ np.asarray(plane_b[:3])
    offset = plane_b[3] - normal @ position
    if normal[2] < .7:
        raise ValueError('Evaluation plane too far from the nominal horizontal surface')
    return -(normal[:2] @ xy + offset)/normal[2]


def command_motion_metrics(actors, physics, start=3., end=12.2):
    """Descriptive shared-window metrics, not a deterministic dynamics A/B."""
    selected=[row for row in actors if start-1.e-9<=row['time_s']<=end+1.e-9]
    states=[row for row in physics if start-1.e-9<=row['time_s']<=end+1.e-9]
    if len(selected)<2 or len(states)<2:
        return None
    at={round(row['time_s'],8):row for row in physics}
    goals=[]
    for row in selected:
        state=at[round(row['time_s'],8)]
        goals.append(np.asarray(state['base_position_w_m'])+
            quaternion_matrix(state['base_quaternion_wxyz'])@np.asarray(row['tool_pose_b'][:3]))
    quaternions=np.asarray([row['tool_pose_b'][3:] for row in selected])
    quaternions/=np.linalg.norm(quaternions,axis=1,keepdims=True)
    dots=np.sum(quaternions[:-1]*quaternions[1:],axis=1)
    arm_targets=np.asarray([row['joint_target18_rad'][12:] for row in selected])
    z=np.asarray([row['tcp_position_w_m'][2] for row in states])
    steps=1000.*np.linalg.norm(np.diff(np.asarray(goals),axis=0)[:,:2],axis=1)
    return dict(requested_window_s=[start,end],actual_actor_window_s=[selected[0]['time_s'],selected[-1]['time_s']],
        consecutive_quaternion_hemisphere_switch_count=int(np.count_nonzero(dots<0.)),
        minimum_consecutive_quaternion_dot=float(dots.min()),
        maximum_adjacent_arm_joint_target_step_deg=float(np.max(np.abs(np.rad2deg(np.diff(arm_targets,axis=0))))),
        actual_tcp_world_z_min_max_mm=[float(1000*z.min()),float(1000*z.max())],
        actual_tcp_world_z_range_mm=float(1000*np.ptp(z)),
        commanded_world_target_xy_step_mm=distribution(steps),
        async_visual_different_rollouts_not_controlled_AB=True)


def high_frequency_tcp_metrics(physics, start=2., end=12.2):
    """Fixed common raw window, >3Hz at 200Hz; trim 1s from each filter edge."""
    from scipy.signal import butter, sosfiltfilt
    rows=[row for row in physics if start-1.e-9<=row['time_s']<=end+1.e-9]
    times=np.asarray([row['time_s'] for row in rows])
    if len(rows)<801 or abs(times[0]-start)>1.e-8 or abs(times[-1]-end)>1.e-8:
        return None  # Do not silently compare a shorter window against others.
    if not np.allclose(np.diff(times),.005,rtol=0.,atol=1.e-8):
        raise ValueError('High-frequency comparison requires complete 200Hz samples')
    positions=np.asarray([row['tcp_position_w_m'] for row in rows])
    filtered=sosfiltfilt(butter(3,3.,fs=200.,btype='highpass',output='sos'),positions,axis=0)[200:-200]
    axes=np.sqrt(np.mean(filtered**2,axis=0))*1000.
    return dict(raw_window_s=[start,end],effective_window_s=[float(times[200]),float(times[-201])],
        frequency_hz=200,filter='third-order Butterworth >3Hz, forward-backward, trim 1s per edge',
        actual_world_xyz_rms_mm=axes.tolist(),actual_world_3axis_rms_mm=float(np.linalg.norm(axes)),
        note='Different asynchronous rollouts, not a controlled A/B. World Y is not always arm lateral.')


def review(directory):
    directory = directory.resolve(strict=True)
    manifest = json.loads((directory/'manifest.json').read_text())
    if manifest.get('motion_compensation') != 'leg_imu' or 'nominal' not in manifest['reset_mode']:
        raise ValueError('Only VD14 leg_imu nominal-reset logs are supported')
    physics = jsonlines(directory/'physics_200hz.jsonl.gz')
    actors = jsonlines(directory/'actor_50hz.jsonl.gz')
    samples = jsonlines(directory/'sensor_motion_200hz.jsonl')
    packets = jsonlines(directory/'dual_vision_packets.jsonl')
    if not physics or not actors:
        raise ValueError('Empty rollout')
    # 200Hz simulator clocks, not host wall time, bind capture to sensor sample.
    physical = {round(row['time_s'],8):row for row in physics}
    physical[0.] = dict(base_position_w_m=[0.,0.,.5],base_quaternion_wxyz=[1.,0.,0.,0.])
    motion = {round(row['sample_time_s'],8):row for row in samples if 'sample_time_s' in row}
    motion[0.] = dict(rotation_local=np.eye(3).tolist(),position_local_m=[0.,0.,0.])
    images = {(p['capture_generation'],p['rear']['sequence']):p for p in packets}
    errors, angles, z_errors, corrected_height, raw_height, ticks = [],[],[],[],[],[]
    goals = []
    for actor in actors:
        status = actor['visual_status']
        now, capture = (round(status[k],8) for k in ('control_use_sim_time_s','rear_capture_sim_time_s'))
        if now < 2.-1.e-9:
            continue
        n,c = physical[now],physical[capture]
        rn,rc = quaternion_matrix(n['base_quaternion_wxyz']),quaternion_matrix(c['base_quaternion_wxyz'])
        pn,pc = np.array(n['base_position_w_m']),np.array(c['base_position_w_m'])
        en,ec = motion[now],motion[capture]
        en_r,ec_r = np.array(en['rotation_local']),np.array(ec['rotation_local'])
        estimated_r = en_r.T @ ec_r
        estimated_t = en_r.T @ (np.array(ec['position_local_m'])-en['position_local_m'])
        true_r,true_t = rn.T @ rc,rn.T @ (pc-pn)
        error = rn @ (estimated_t-true_t)
        errors.append(1000.*np.linalg.norm(error));z_errors.append(1000.*error[2])
        angles.append(np.degrees(np.arccos(np.clip((np.trace(estimated_r.T @ true_r)-1.)/2.,-1.,1.))))
        goal = pn + rn @ np.array(actor['tool_pose_b'][:3])
        raw = images[(status['capture_generation'],status['source_sequence'])]['rear']['ground_plane_b']
        corrected_height.append(1000.*plane_height(status['ground_plane_b'],rn,pn,goal[:2]))
        raw_height.append(1000.*plane_height(raw,rn,pn,goal[:2]))
        goals.append(goal);ticks.append(dict(time_s=now,source_sequence=status['source_sequence'],
            rear_target_source=status.get('rear_target_source'),
            rear_tangent_b=status.get('rear_tangent_b'),
            rear_near_field_lateral_residual_m=status.get('rear_near_field_lateral_residual_m')))
    if len(errors)<2:
        raise ValueError('Insufficient post-startup evaluation samples')
    # Large image-command changes are evidence for further diagnosis, not a
    # new runtime limiter or an invented measurement rejection threshold.
    jumps = np.linalg.norm(np.diff(np.array(goals)[:,:2],axis=0),axis=1)
    largest = [dict(ticks[i+1],previous_time_s=ticks[i]['time_s'],
                    previous_source_sequence=ticks[i]['source_sequence'],
                    target_xy_step_mm=float(1000.*jumps[i]))
               for i in np.argsort(jumps)[-5:][::-1]]
    guard = manifest['geometry_guard']
    tool = dict(zip(guard['actual_body_names'],guard['actual_body_ids']))['link06']
    tool_peak = max(float(row['body_contact_force_world_n'][tool][2]) for row in physics)
    # This is the finite cylinder's analytic exterior support height, not a
    # physics-engine penetration measurement or the robot keep-out lower bound.
    exterior = []
    for row in physics:
        ends = np.asarray(row['geometry']['tube_endpoints_w_m'])
        axis = (ends[1]-ends[0])/1.1
        exterior.append(float(np.min(ends[:,2])-.028*np.sqrt(max(0.,1.-axis[2]**2))))
    estimates = [row for row in samples if 'sample_time_s' in row]
    return dict(directory=str(directory),summary=json.loads((directory/'summary.json').read_text()),
        checkpoint=manifest['checkpoint'],checkpoint_sha256=manifest['checkpoint_sha256'],
        measurement_window_s=[ticks[0]['time_s'],ticks[-1]['time_s']],
        capture_to_control_rotation_error_deg=distribution(angles),
        capture_to_control_translation_error_mm=distribution(errors),
        capture_to_control_vertical_error_signed_mm=distribution(z_errors),
        same_plane_corrected_height_absolute_mm=distribution(np.abs(corrected_height)),
        same_plane_UNCOMPENSATED_height_absolute_mm=distribution(np.abs(raw_height)),
        tool_native_upward_force_peak_n=tool_peak,
        minimum_analytic_cylinder_floor_height_m=min(exterior),
        inertial_prediction_fraction=float(np.mean([r['short_velocity_prediction'] for r in estimates])),
        largest_world_target_xy_steps=largest,
        shared_window_command_motion=command_motion_metrics(actors,physics),
        shared_window_high_frequency_tcp=high_frequency_tcp_metrics(physics),
        source_hashes={name:hashlib.sha256((directory/name).read_bytes()).hexdigest() for name in
            ('manifest.json','physics_200hz.jsonl.gz','actor_50hz.jsonl.gz','sensor_motion_200hz.jsonl','dual_vision_packets.jsonl')},
        evaluation_world_pose_used_OFFLINE_ONLY=True,deployment_accepted=False,
        note='Short samples and ideal simulated sensors; no real-sensor/real-robot validation. Keep-out violations are not proven mesh penetration.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories',type=Path,nargs='+')
    parser.add_argument('--output-root',type=Path,default=Path(__file__).resolve().parents[2]/'outputs/vd14_motion_review')
    args=parser.parse_args()
    reports=[review(p) for p in args.directories]
    args.output_root.mkdir(parents=True,exist_ok=True)
    out=Path(tempfile.mkdtemp(prefix='review_',dir=args.output_root))/'analysis.json'
    with out.open('x') as stream:
        json.dump(dict(reports=reports),stream,indent=2,allow_nan=False)
    print(out)
    for r in reports:
        print(r['directory'],json.dumps({k:r[k] for k in (
            'same_plane_corrected_height_absolute_mm','same_plane_UNCOMPENSATED_height_absolute_mm',
            'capture_to_control_translation_error_mm','tool_native_upward_force_peak_n')}))
    return 0


if __name__=='__main__': raise SystemExit(main())
