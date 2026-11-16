#!/usr/bin/env python3
"""Saved telemetry only. Preserve raw files, report damaged tails explicitly."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from analyze_recorded_tcp import fk, matrix, stats


def read_saved(path):
    raw = path.read_bytes()
    quality = dict(sha256=hashlib.sha256(raw).hexdigest(), bytes=len(raw),
                   nul_bytes=raw.count(b'\0'), skipped_rows=0)
    rows = []
    with path.open(newline='') as stream:
        def complete_lines():
            for line in stream:
                if '\0' in line:
                    quality['skipped_rows'] += 1
                    continue
                yield line
        for row in csv.DictReader(complete_lines()):
            if None in row or any(v is None or '\0' in v for v in row.values()):
                quality['skipped_rows'] += 1
            else:
                rows.append(row)
    quality['complete_rows'] = len(rows)
    return rows, quality


def intervals(mask):
    edges = np.diff(np.r_[False, mask, False].astype(int))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def analyze(path):
    commands, cq = read_saved(path/'commands.csv')
    controls, sq = read_saved(path/'control.csv')
    actors, aq = read_saved(path/'actor.csv')
    origin = int(commands[0]['publish_begin_ns'])
    t = (np.array([int(r['control_start_ns']) for r in controls])-origin)*1e-9
    widths = np.diff(t, append=t[-1])
    cols = lambda name: np.array([float(r[name]) for r in controls])
    height = cols('h_lower_m')
    stage = cols('stop_stage')
    forward = cols('trial_stage') == 3
    actual = matrix(controls, 'actual_q_policy_', 18)
    target = matrix(controls, 'fsm_target_q_policy_', 18)
    desired_tcp = matrix(controls, 'tcp_target_b_', 3)
    actual_tcp = fk(actual)
    valid_tcp = cols('tcp_measured_valid') == 1
    recorded_tcp = matrix(controls, 'tcp_measured_b_', 3)
    geometry_error = float(np.max(np.linalg.norm(actual_tcp[valid_tcp]-recorded_tcp[valid_tcp],axis=1))) if valid_tcp.any() else None
    if geometry_error is not None and geometry_error > 1.e-6:
        raise ValueError('FK differs from recorded TCP; stop interpreting positional error')
    active = forward & (cols('tcp_active') == 1) & valid_tcp
    report = dict(name=path.name, source=str(path.resolve()),
                  quality=dict(commands=cq, control=sq, actor=aq),
                  forward_s=float(widths[forward].sum()),
                  tracking_s=float(widths[active].sum()),
                  visual_stopped_but_forward_s=float(widths[forward & (cols('visual_stage') == 2)].sum()),
                  fault_ever=bool(np.any(cols('safety_fault'))), sessions=[],
                  fk_vs_recorded_max_error_m=geometry_error,
                  terminal_confirmed=bool(np.any(cols('terminal_confirmed'))))
    summary_raw = (path/'telemetry_summary.json').read_bytes()
    try:
        report['telemetry_summary'] = json.loads(summary_raw)
    except (ValueError, UnicodeError):
        report['telemetry_summary'] = None
        report['summary_file_damaged'] = dict(bytes=len(summary_raw), nul_bytes=summary_raw.count(b'\0'))
    starts = intervals(forward)
    traces = []
    for number, (begin, end) in enumerate(starts, 1):
        next_begin = starts[number][0] if number < len(starts) else len(controls)
        session = dict(number=number, start_s=float(t[begin]), stop_s=float(t[end]) if end < len(t) else None,
                       forward_s=float(widths[begin:end].sum()),
                       tracking_s=float(widths[begin:end][active[begin:end]].sum()),
                       rear_gap_start_m=float(cols('tcp_rear_gap_m')[begin]),
                       rear_gap_end_m=float(cols('tcp_rear_gap_m')[end-1]))
        first_active = np.flatnonzero(active[begin:end])
        if len(first_active):
            session['first_tracking_s'] = float(t[begin+first_active[0]])
            session['tracking_delay_s'] = float(t[begin+first_active[0]]-t[begin])
        if end < len(t):
            previous, event = controls[end-1], controls[end]
            session['stop_event'] = {key:event[key] for key in (
                'visual_stage','visual_sequence','visual_qualified','raw_keys','getdown_pending',
                'tcp_pose_valid','safety_fault','visual_path_count','visual_path_span_px')}
            session['stop_event']['retained_capture_age_s'] = (
                int(event['control_start_ns'])-int(event['visual_capture_ns']))*1e-9
            session['stop_event']['same_retained_packet'] = event['visual_sequence'] == previous['visual_sequence']
            catches = np.flatnonzero(stage[end:next_begin] == 2)
            if len(catches):
                catch = end+catches[0]
                session['capture_s'] = float(t[catch])
                session['capture_delay_s'] = float(t[catch]-t[end])
                blocks = np.flatnonzero(stage[catch:next_begin] == 5)
                if len(blocks):
                    block = catch+blocks[0]
                    session['blocked_after_capture_s'] = float(t[block]-t[catch])
                    session['height_at_block_m'] = float(height[block])
                session['capture_height_m'] = float(height[catch])
                session['min_post_capture_height_m'] = float(np.nanmin(height[catch:next_begin]))
                session['ready_ever'] = bool(np.any(stage[catch:next_begin] == 4))
                session['rear_calf_at_capture_rad'] = actual[catch, [11,12]].tolist()
                session['rear_calf_final_rad'] = actual[next_begin-1, [11,12]].tolist()
                traces.append((number, t[catch:next_begin]-t[catch], height[catch:next_begin],
                               actual[catch:next_begin,11:13], stage[catch:next_begin]))
        report['sessions'].append(session)
    if active.any():
        report['tracking_error'] = dict(
            actual_minus_goal=stats(actual_tcp[active]-desired_tcp[active]),
            fsm_fk_minus_goal=stats(fk(target[active])-desired_tcp[active]),
            actual_minus_fsm_fk=stats(actual_tcp[active]-fk(target[active])),
            workspace_limited_fraction=float(np.mean(cols('tcp_workspace_limited')[active])),
            allocation_error_mean_m=float(np.mean(cols('tcp_allocation_error_m')[active])),
            body_cross_track_range_m=[float(cols('body_cross_track_m')[active].min()),float(cols('body_cross_track_m')[active].max())],
            applied_yaw_range_rad_s=[float(cols('limited_2')[active].min()),float(cols('limited_2')[active].max())],
            observed_goal_lateral_max_m=float(cols('tcp_observed_goal_body_1')[active].max()))
        if actors:
            ns = np.array([int(r['control_start_ns']) for r in controls])
            ans = np.array([int(r['actor_end_ns']) for r in actors])
            indices = np.searchsorted(ans, ns, side='right')-1
            mask = active & (indices >= 0)
            actor_q = matrix(actors, 'target_q_policy_', 18)
            report['tracking_error']['actor_fk_minus_goal'] = stats(fk(actor_q[indices[mask]])-desired_tcp[mask])
    return report, traces


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('logs', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    args.output.mkdir(exist_ok=False, parents=True)
    reports = []
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    paths = sorted(args.logs.glob('va15_near_visual_*'))
    fig, axes = plt.subplots(len(paths), 2, figsize=(11, 2.8*len(paths)), squeeze=False)
    for row, path in enumerate(paths):
        report, traces = analyze(path)
        reports.append(report)
        for number, seconds, height, calf, stage in traces:
            mask = seconds <= 5.
            axes[row,0].plot(seconds[mask],height[mask],label=f'stop {number}')
            axes[row,1].plot(seconds[mask],calf[mask].mean(axis=1),label=f'stop {number}')
        axes[row,0].axhline(.48,color='gray',linestyle='--',linewidth=.8)
        axes[row,0].set_ylabel(path.name[-6:]+' height (m)')
        axes[row,1].set_ylabel('rear calf mean (rad)')
        for ax in axes[row]:
            ax.set_xlabel('seconds from stop capture');ax.grid(alpha=.2)
            if traces:ax.legend()
    fig.tight_layout();fig.savefig(args.output/'recorded_stops.png',dpi=150);plt.close(fig)
    (args.output/'analysis.json').write_text(json.dumps(dict(
        schema='va15_four_run_review_v1', runs=reports,
        note='Recorded model estimates and commanded references; no hardware or precision qualification.'
    ),indent=2,allow_nan=False)+'\n')
    for r in reports:
        print(r['name'],json.dumps({k:r[k] for k in ('forward_s','tracking_s','fault_ever')},ensure_ascii=False))
        for s in r['sessions']:
            print(' ',json.dumps(s,ensure_ascii=False))


if __name__ == '__main__':
    main()
