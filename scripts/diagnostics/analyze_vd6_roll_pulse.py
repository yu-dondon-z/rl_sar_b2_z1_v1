#!/usr/bin/env python3
"""Compare matched MuJoCo roll-pulse sessions without interpreting correlations as causation."""

import argparse
import gzip
import json
from pathlib import Path

import numpy as np


PHYSICS_CHANNELS = {
    'base_roll_rad': ('base_roll_rad', None),
    'joint1_position_rad': ('joint_position18_rad', 8),
    'joint1_velocity_radps': ('joint_velocity18_radps', 8),
    'joint6_position_rad': ('joint_position18_rad', 17),
    'joint6_velocity_radps': ('joint_velocity18_radps', 17),
    'tcp_lateral_body_m': ('tcp_position_b', 1),
}
ACTOR_CHANNELS = {
    'joint1_target_rad': ('joint_target18', 8),
    'joint6_target_rad': ('joint_target18', 17),
    'base_gyro_roll_radps': ('gyro_b', 0),
    'gravity_lateral_b': ('gravity_b', 1),
}


def load_session(path):
    result = dict(path=str(path))
    for name in ('manifest', 'summary'):
        result[name] = json.loads((path / (name + '.json')).read_text(encoding='utf-8'))
    for key, name in (('physics', 'physics_truth'), ('joints', 'joint_response'),
                      ('actor', 'actor_trace')):
        with gzip.open(path / (name + '.jsonl.gz'), 'rt') as source:
            result[key] = [json.loads(line) for line in source]
    return result


def signal(rows, field, index):
    values = np.asarray([row[field] for row in rows], dtype=float)
    return values if index is None else values[:, index]


def channel_summary(base, pulse, *, sample_dt, start, channels, time_key):
    times = signal(base, time_key, None)
    if not np.array_equal(times, signal(pulse, time_key, None)):
        raise ValueError('Paired sample timestamps differ')
    output = {}
    for name, (field, index) in channels.items():
        delta = signal(pulse, field, index) - signal(base, field, index)
        pre = times <= start + 1e-9
        if not np.array_equal(signal(base, field, index)[pre], signal(pulse, field, index)[pre]):
            raise ValueError(f'Pre-pulse {name} differs')
        windows = {}
        for lo, hi in ((0., .2), (.2, .6), (.6, 1.4), (1.4, 3.)):
            keep = (times > start + lo + 1e-9) & (times <= start + hi + 1e-9)
            segment = delta[keep]
            expected = round((hi - lo) / sample_dt)
            if len(segment) != expected:
                raise ValueError(f'{name}: expected {expected} samples, got {len(segment)}')
            windows[f'{lo:g}..{hi:g}s'] = dict(rms=float(np.sqrt(np.mean(segment ** 2))),
                peak_abs=float(np.max(np.abs(segment))), final=float(segment[-1]))
        output[name] = windows
    return output


def compare_pair(base, pulse):
    bm, pm = base['manifest'], pulse['manifest']
    fields = ('policy_sha256', 'checkpoint_iteration', 'training_experiment',
              'fixed_body_tool_target', 'physics_dt', 'actor_dt', 'speed',
              'gains_kp_kd', 'initial_base_height_m')
    if bm['engine'] != pm['engine'] or bm['engine'] != 'mujoco':
        raise ValueError('Expected paired MuJoCo runs')
    if any(bm[key] != pm[key] for key in fields):
        raise ValueError('Paired run configuration differs')
    bp, pp = bm['external_roll_pulse'], pm['external_roll_pulse']
    if bp['torque_nm'] != 0 or not 0 < pp['torque_nm'] <= 20:
        raise ValueError('Expected zero baseline and small nonzero pulse')
    if (bp['start_s'], bp['duration_s']) != (pp['start_s'], pp['duration_s']):
        raise ValueError('Pulse timing differs between paired runs')
    if base['summary']['reason'] != pulse['summary']['reason'] or pulse['summary']['reason'] != 'duration_complete':
        raise ValueError('Incomplete paired run')
    start, duration = pp['start_s'], pp['duration_s']
    for key in ('physics', 'joints', 'actor'):
        left, right = base[key], pulse[key]
        if len(left) != len(right):
            raise ValueError('Paired run lengths differ')
        count = round(start / (.005 if key != 'actor' else .02))
        if left[:count] != right[:count]:
            raise ValueError(f'Paired {key} trace differs before pulse')
    times = signal(pulse['physics'], 'time_s', None)
    torques = np.asarray([row['external_roll_torque_w_nm'] for row in pulse['physics']])
    baseline_torques = np.asarray([row['external_roll_torque_w_nm'] for row in base['physics']])
    mask = (times > start + 1e-9) & (times <= start + duration + 1e-9)
    if (np.any(baseline_torques) or np.any(torques[~mask])
            or mask.sum() != round(duration / .005)
            or not np.allclose(np.linalg.norm(torques[mask], axis=1), pp['torque_nm'], atol=1e-9)):
        raise ValueError('Actual physical torque samples disagree with requested pulse')
    at_start = base['physics'][round(start / .005) - 1]
    if abs(at_start['time_s'] - start) > 1e-9:
        raise ValueError('Missing pre-pulse physical state')
    forces = np.asarray(at_start['foot_force_w'])[:, 2]
    joint_channels = channel_summary(base['joints'], pulse['joints'], sample_dt=.005,
        start=start, channels={
            k:v for k,v in PHYSICS_CHANNELS.items() if k.startswith('joint')}, time_key='time_s')
    body_channels = channel_summary(base['physics'], pulse['physics'], sample_dt=.005,
        start=start, channels={
            k:v for k,v in PHYSICS_CHANNELS.items() if not k.startswith('joint')}, time_key='time_s')
    joint_channels.update(body_channels)
    return dict(baseline=base['path'], perturbed=pulse['path'], policy_sha256=bm['policy_sha256'],
        checkpoint_iteration=bm['checkpoint_iteration'],
        torque_nm=pp['torque_nm'], start_s=start, duration_s=duration,
        impulse_magnitude_nms=float(pp['torque_nm'] * duration),
        pre_pulse_base_roll_rad=float(at_start['base_roll_rad']),
        pre_pulse_foot_vertical_force_n=forces.tolist(),
        pre_pulse_foot_contact_gt10n=(forces > 10).tolist(),
        physics_steps=len(times), actor_steps=len(base['actor']),
        exact_pre_pulse_trace_match=True, physical_torque_samples=int(mask.sum()),
        channels_200hz=joint_channels,
        channels_50hz=channel_summary(base['actor'], pulse['actor'], sample_dt=.02,
            start=start, channels=ACTOR_CHANNELS, time_key='time_s'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('vd5_baseline', 'vd5_pulse', 'vd6_baseline', 'vd6_pulse'):
        parser.add_argument('--' + name.replace('_', '-'), type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    vd5 = compare_pair(load_session(args.vd5_baseline), load_session(args.vd5_pulse))
    vd6 = compare_pair(load_session(args.vd6_baseline), load_session(args.vd6_pulse))
    if (vd5['torque_nm'], vd5['start_s'], vd5['duration_s']) != (
            vd6['torque_nm'], vd6['start_s'], vd6['duration_s']):
        raise ValueError('VD5 and VD6 perturbations differ')
    result = dict(vd5=vd5, vd6=vd6,
        interpretation='Perturbation minus matched no-pulse trajectory, not between-policy raw trace. '
                       'The paired run tests closed-loop susceptibility in one gait phase; '
                       'it cannot separate actor from contact dynamics or establish a universal root cause.',
        archived_log_caveat='The first four sessions logged base_angular_velocity_b using a field that disagrees '
                            'with the sampled IMU gyro; it is excluded. Actor gyro_b and physical base_roll_rad '
                            'are used. Their joint_response actor_inputs_changed=false means no artificial '
                            'input overwrite, not that real observations stayed fixed after perturbation.')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    for key in ('vd5', 'vd6'):
        print(key, '0..0.2s RMS delta:', {name: round(windows['0..0.2s']['rms'], 6)
            for name, windows in result[key]['channels_200hz'].items()})
    print(args.output)


if __name__ == '__main__':
    main()
