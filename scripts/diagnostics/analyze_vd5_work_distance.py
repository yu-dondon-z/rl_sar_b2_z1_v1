#!/usr/bin/env python3
"""Compare VD5 work distances without treating parked TCP error as walking accuracy."""
from __future__ import annotations

import argparse
from collections import Counter
import gzip
import json
from pathlib import Path

import numpy as np


def read_rows(path):
    with gzip.open(path, 'rt') as handle:
        return [json.loads(line) for line in handle if line.strip()]


def longest_interval(mask, times):
    indices = np.flatnonzero(mask)
    groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1)+1)
    return max((float(times[g[-1]]-times[g[0]]+.02) for g in groups if len(g)), default=0.)


def percentile(values, mask, percent=95):
    selected = np.asarray(values)[mask]
    return float(np.percentile(selected, percent)) if len(selected) else None


def inspect_session(session):
    session = Path(session).resolve()
    summary = json.loads((session/'summary.json').read_text())
    contract = json.loads((session/'runtime_contract.json').read_text())
    trace = read_rows(session/'actor_trace.jsonl.gz')
    truth = read_rows(session/'evaluation_truth.jsonl.gz')
    if not trace or len(trace) != len(truth) or any(a['step'] != b['step'] for a, b in zip(trace, truth)):
        raise ValueError('actor and independent evaluator ticks do not match: '+str(session))
    times = np.array([r['sim_time_s'] for r in truth])
    settled = times >= 2.
    commands = np.array([r['base_command_b'] for r in trace])
    walking = settled & (commands[:, 0] >= .10)
    invalid_mask = np.array([not r['visual_command_diagnostics'].get('valid') for r in trace])
    invalid_indices = np.flatnonzero(invalid_mask)
    invalid_groups = np.split(invalid_indices, np.flatnonzero(np.diff(invalid_indices)>1)+1)
    invalid_durations = [len(group)*.02 for group in invalid_groups if len(group)]
    velocities = np.array([r['base_velocity_b'][0] for r in truth])
    moving = settled & (velocities >= .05)
    positions = np.array([r['base_position_w'] for r in truth])
    tcp_error = np.array([r['tcp_projection_error_m'] for r in truth])
    base_error = np.array([r['base_cross_track_m'] for r in truth])
    packets = [json.loads(line) for line in (session/'dual_vision_packets.jsonl').read_text().splitlines() if line.strip()]
    accepted_status = [r['visual_command_diagnostics'] for r in trace
                       if r['visual_command_diagnostics'].get('valid')]
    estimated_status = [s for s in accepted_status if s.get('rear_target_occlusion_extension_m', 0.) > 0.]
    invalid = Counter(r['visual_command_diagnostics']['reason'] for r in trace
                      if not r['visual_command_diagnostics'].get('valid'))
    result = dict(session=str(session), work_distance_m=summary['tcp_trailing_distance_m'],
                  occlusion_extension_limit_m=summary.get('rear_occlusion_extension_limit_m', 0.),
                  rear_centerline_mode=summary.get('rear_centerline_mode', 'skeleton'),
                  stop_reason=summary['stop_reason'], runtime_error=summary['runtime_error'],
                  simulated_s=summary['sim_time_s'], actor_steps=len(trace),
                  valid_command_fraction=summary.get('active_control_actor_steps',
                                                     summary['valid_visual_actor_steps'])/len(trace),
                  measurement_command_fraction=sum(r['visual_command_diagnostics'].get(
                      'measurement_valid', r['visual_command_diagnostics'].get('valid', False))
                      for r in trace)/len(trace),
                  hold_command_fraction=sum(r['visual_command_diagnostics'].get('control_state')=='rear_hold'
                                            for r in trace)/len(trace),
                  rear_hold_s=summary.get('rear_hold_s', 0.),
                  rear_path_memory_s=summary.get('rear_path_memory_s', 0.),
                  memory_command_fraction=sum(s.get('control_state') == 'rear_memory'
                                              for s in accepted_status)/len(trace),
                  memory_source_age_max_s=max((s.get('rear_memory_source_age_max_s', 0.)
                                               for s in accepted_status), default=0.),
                  paired_valid_packet_fraction=sum(bool(r['valid']) for r in packets)/len(packets),
                  rear_valid_packet_fraction=sum(bool(r['rear']['valid']) for r in packets)/len(packets),
                  packet_count=len(packets),
                  accepted_ticks_with_far_endpoint_estimate=len(estimated_status),
                  max_used_occlusion_extension_m=max((s['rear_target_occlusion_extension_m']
                                                     for s in estimated_status), default=0.),
                  target_source_counts=dict(Counter(s.get('rear_target_source', 'unrecorded')
                                                    for s in accepted_status)),
                  invalid_episode_count=len(invalid_durations),
                  invalid_episode_p50_s=float(np.percentile(invalid_durations, 50)) if invalid_durations else 0.,
                  invalid_episode_p95_s=float(np.percentile(invalid_durations, 95)) if invalid_durations else 0.,
                  invalid_episode_max_s=max(invalid_durations, default=0.),
                  latest_packet_age_p95_s=float(np.percentile([r['capture_age_s'] for r in packets], 95)),
                  base_displacement_xy_m=(positions[-1, :2]-positions[0, :2]).tolist(),
                  settled_mean_actual_vx_mps=float(velocities[settled].mean()),
                  walking_mean_actual_vx_mps=float(velocities[walking].mean()) if walking.any() else None,
                  walking_command_s=float(walking.sum()*.02), actual_moving_s=float(moving.sum()*.02),
                  longest_walking_command_s=longest_interval(walking, times),
                  last_walking_command_time_s=float(times[walking][-1]) if walking.any() else None,
                  walking_tcp_projection_p95_m=percentile(tcp_error, walking),
                  walking_tcp_projection_max_m=percentile(tcp_error, walking, 100),
                  walking_base_cross_track_p95_m=percentile(base_error, walking),
                  settled_tcp_projection_p95_m=percentile(tcp_error, settled),
                  settled_tcp_projection_max_m=percentile(tcp_error, settled, 100),
                  tilt_max_deg=max(r['tilt_deg'] for r in truth),
                  support_foot_samples_inside_52mm_seam_buffer=summary['evaluation'][
                      'support_foot_samples_inside_52mm_seam_buffer'],
                  invalid_actor_tick_reasons=dict(invalid),
                  rear_packet_reasons=dict(Counter(r['rear']['reason'] for r in packets)),
                  note='Walking subset: t>=2 s and commanded vx>=0.10 m/s; short subsets do not establish tracking accuracy.')
    signature = {key: contract[key] for key in ('policy_sha256', 'camera_calibration', 'speed_mps',
                 'curve_max_curvature_per_m', 'physics_dt_s', 'velocity_constraint', 'perception_mode')}
    return result, signature


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('sessions', type=Path, nargs='+')
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    inspected = [inspect_session(session) for session in args.sessions]
    if any(signature != inspected[0][1] for _, signature in inspected[1:]):
        raise ValueError('Compared runs have different policies, camera calibration, or simulation settings')
    results = sorted((row for row, _ in inspected),
                     key=lambda r: (r['work_distance_m'], r['occlusion_extension_limit_m']))
    report = dict(shared_settings=inspected[0][1], runs=results,
                  note='Separate sequential real-time image-processing runs; asynchronous capture timing can change outcomes. No training or hardware control.')
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir/'comparison.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    lines = ['# VD5 末端工作距离对照', '',
             '相同实测相机参数、RHANet、枪体遮罩、策略与仿真设置；时长与指令速度见 JSON。', '',
             '行走统计限定在起步 2 秒之后且 vx 指令至少 0.10 m/s 的时段；其余时段含停车。', '',
             '| 距离 / 外推上限 / 中心线 / 保持 / 路径缓存 | 有效后视图像 | 当帧目标 / 保持 / 历史路径 / 总有效控制 | 前进位移 | 行走指令累计/最长连续 | 行走 TCP 投影 P95 | 最大倾角 |',
             '|---|---:|---:|---:|---:|---:|---:|']
    for row in results:
        error = row['walking_tcp_projection_p95_m']
        lines.append('| %.2f m / %.0f cm / %s / %.2f s / %.2f s | %.1f%% | %.1f%% / %.1f%% / %.1f%% / %.1f%% | %.2f m | %.2f / %.2f s | %s | %.2f° |' % (
            row['work_distance_m'], 100*row['occlusion_extension_limit_m'],
            row['rear_centerline_mode'], row['rear_hold_s'], row['rear_path_memory_s'],
            100*row['rear_valid_packet_fraction'], 100*row['measurement_command_fraction'],
            100*row['hold_command_fraction'], 100*row['memory_command_fraction'],
            100*row['valid_command_fraction'], row['base_displacement_xy_m'][0],
            row['walking_command_s'], row['longest_walking_command_s'],
            ('%.2f cm' % (100*error)) if error is not None else '无样本', row['tilt_max_deg']))
    lines.extend(['', '这些数值不表示通过连续行走验收；异步视觉采样会使不同运行出现差异。', '',
                  '## 原始日志', ''])
    lines.extend('- %.2f m / %.0f cm 外推上限: `%s`' % (
        row['work_distance_m'], 100*row['occlusion_extension_limit_m'], row['session']) for row in results)
    (args.output_dir/'comparison.md').write_text('\n'.join(lines)+'\n', encoding='utf-8')
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
