#!/usr/bin/env python3
"""Offline camera-to-command audit; truth is used for evaluation only.

Replay recorded joint targets without running an actor or image controller.
Verify encoder agreement before comparing captured measurements with replay
geometry. Original sessions remain read-only; results get a new directory.
"""
from __future__ import annotations

import argparse
from collections import Counter
import csv
from datetime import datetime
import gzip
import json
from pathlib import Path
import sys
import tempfile

import mujoco
import numpy as np

from debug_vd1_truth_curve import make_curve
from vd1_mujoco_contract import VD1MujocoAdapter
from vd1_mujoco_scene import TRAINING_ROOT, build_scene
from vd1_mujoco_velocity_impulse import VelocityImpulseConstraints

sys.path.insert(0, str(TRAINING_ROOT/'scripts/vision'))
from vd1_control import _front_geometry, _rear_geometry
from vd1_visual_guard import rear_seam_target


ROOT = Path(__file__).resolve().parents[1]


def read_gzip(path):
    with gzip.open(path, 'rt') as handle:
        return [json.loads(line) for line in handle]


def stats(values):
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return dict(mean=float(values.mean()), p95=float(np.percentile(values, 95)),
                max=float(values.max())) if len(values) else None


def analyze(session, output_root, contacts=False):
    session = Path(session).resolve()
    contract = json.loads((session/'runtime_contract.json').read_text())
    if (contract['velocity_constraint'] != 'impulse'
            or contract['physics_hz'] != 200 or not contract['physx_implicit_position']):
        raise ValueError('This audit supports the existing impulse / 5ms / implicit-position debug preset')
    output_root.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='audit_'+datetime.now().strftime('%Y%m%d_%H%M%S')+'_',
                                   dir=str(output_root)))
    rows = read_gzip(session/'actor_trace.jsonl.gz')
    packets = read_gzip(session/'dual_vision_packets.jsonl.gz')
    curve = make_curve(contract['curvature_max'], extent=95.)
    scene = build_scene(contract['curvature_max'], output/'scene_cache', timestep=.005,
                        tool_markers=contract.get('tool_markers_in_scene', True),
                        road_surface=contract.get('road_surface', 'crack'),
                        gun_mass_mode=contract.get('gun_mass_mode', 'original'))
    if contract.get('road_contact', 'prisms') == 'heightfield':
        from vd1_mujoco_road_contact import continuous_road_contact
        scene = continuous_road_contact(scene, contract['curvature_max'])
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('vd1_default').id)
    mujoco.mj_forward(model, data)
    adapter = VD1MujocoAdapter(model, physx_implicit_position=True)
    brake = VelocityImpulseConstraints(model, adapter.joint_ids18, adapter.velocity_limit18)
    contact_recorder = None
    if contacts:
        from replay_vd1_fall_contacts import ContactRecorder
        contact_recorder = ContactRecorder(model, output)
    base, tcp = model.body('base_link').id, model.site('tool_tcp').id
    states, encoder_errors = {}, []
    for row in rows:
        measured = adapter.read_proprioception(data)
        q_recorded = np.asarray(row['observation'][16:34])+adapter.default_q18
        encoder_errors.append(float(np.max(np.abs(q_recorded-measured['q']))))
        states[row['step']] = dict(position=data.xpos[base].copy(),
            rotation=data.xmat[base].reshape(3, 3).copy(), tcp=data.site_xpos[tcp].copy())
        target = np.asarray(row['joint_target'])
        for tick in range(4):
            mujoco.mj_forward(model, data)
            adapter.apply_targets(data, target)
            brake.step(data)
            if contact_recorder is not None:
                contact_recorder.write(data, row['step'], tick)
        mujoco.mj_forward(model, data)
    if contact_recorder is not None:
        contact_recorder.close()
    replay_error = max(encoder_errors, default=0.)
    if replay_error > 2e-5:
        raise ValueError('Replay diverged from recorded encoders (max %.6g rad); no truth comparison emitted' % replay_error)

    camera_rows = []
    for packet in packets:
        rear, front = packet['rear'], packet['front']
        capture_time = rear.get('capture_sim_time_s')
        if capture_time is None:
            continue
        index = int(round(capture_time/.02))
        if index not in states:
            continue
        state = states[index]
        rotation, position = state['rotation'], state['position']
        tilt = float(np.degrees(np.arccos(np.clip(rotation[2, 2], -1, 1))))
        result = dict(sequence=rear['sequence'], capture_sim_time_s=capture_time,
                      packet_valid=packet['valid'], front_valid=front['valid'], rear_valid=rear['valid'],
                      reason=packet['reason'], tilt_deg=tilt, packet_age_s=packet['capture_age_s'],
                      processing_ms=packet['processing_ms'])
        for name, stream in [('front', front), ('rear', rear)]:
            points = np.asarray(stream.get('points_b', []), dtype=float)
            if points.size:
                world = points @ rotation.T + position
                distances = [curve.project(point[:2])['distance'] for point in world]
                result[name+'_seam_world_xy_mean_error_m'] = float(np.mean(distances))
                result[name+'_seam_world_xy_max_error_m'] = float(np.max(distances))
                result[name+'_min_x_b'] = float(points[:, 0].min())
                result[name+'_max_x_b'] = float(points[:, 0].max())
                result[name+'_seam_world_z_mean_m'] = float(world[:, 2].mean())
        if 'tcp_measured_b' in rear:
            actual = rotation.T @ (state['tcp']-position)
            error = np.asarray(rear['tcp_measured_b'])-actual
            result.update(tcp_measurement_error_m=float(np.linalg.norm(error)),
                          tcp_measurement_error_x_m=float(error[0]),
                          tcp_measurement_error_y_m=float(error[1]),
                          tcp_measurement_error_z_m=float(error[2]),
                          actual_tcp_curve_distance_m=curve.project(state['tcp'][:2])['distance'])
        if 'ground_plane_b' in rear:
            plane = np.asarray(rear['ground_plane_b'])
            plane /= np.linalg.norm(plane[:3])
            actual_normal = rotation.T @ np.array([0., 0., 1.])
            result['ground_normal_error_deg'] = float(np.degrees(np.arccos(np.clip(plane[:3] @ actual_normal, -1, 1))))
            result['ground_offset_error_m'] = float(plane[3]-position[2])
        if rear['valid'] and rear.get('points_b'):
            try:
                if contract.get('tool_target_mode') == 'rear_seam_direct':
                    _, diagnostics = rear_seam_target(np.asarray(rear['points_b']),
                                                      np.asarray(rear['ground_plane_b']))
                else:
                    _, diagnostics = _rear_geometry(np.asarray(rear['points_b']),
                        np.asarray(rear['tcp_measured_b']), np.asarray(rear['ground_plane_b']))
                local = (curve.points-position) @ rotation
                crosses = np.flatnonzero((local[:-1, 0]+1.02)*(local[1:, 0]+1.02) <= 0)
                candidates = []
                for i in crosses:
                    dx = local[i+1, 0]-local[i, 0]
                    if abs(dx) > 1e-10:
                        blend = (-1.02-local[i, 0])/dx
                        candidates.append(local[i]+blend*(local[i+1]-local[i]))
                if candidates:
                    actual = min(candidates, key=lambda p: abs(p[1]))
                    measured_y = diagnostics['rear_nominal_target_b'][1]
                    result.update(rear_nominal_y_measured_m=measured_y,
                                  rear_nominal_y_actual_m=float(actual[1]),
                                  rear_nominal_y_error_m=float(measured_y-actual[1]),
                                  rear_target_clipped=diagnostics['target_clipped'])
            except ValueError:
                pass
        if front['valid'] and front.get('points_b'):
            try:
                _, _, _, _, lookahead = _front_geometry(np.asarray(front['points_b']))
                result['front_lookahead_x_m'] = float(lookahead[0])
                result['front_yaw_command_raw_radps'] = float(np.clip(
                    2*contract['speed']*lookahead[1]/np.sum(lookahead[:2]**2), -.8, .8))
            except ValueError:
                pass
        camera_rows.append(result)

    columns = sorted({key for row in camera_rows for key in row})
    with (output/'camera_geometry.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        writer.writerows(camera_rows)
    timeline = []
    packet_by_sequence = {p['rear']['sequence']: p for p in packets}
    for row in rows:
        gravity = np.asarray(row['observation'][13:16])
        status = row['status']
        packet = packet_by_sequence.get(status.get('source_sequence'))
        capture_time = packet['rear'].get('capture_sim_time_s') if packet else None
        timeline.append(dict(sim_time_s=row['sim_time_s'], step=row['step'],
            valid=status['valid'], reason=status['reason'],
            vx=row['base_command_b'][0], yaw_rate=row['base_command_b'][2],
            tool_y=row['tool_pose_b'][1], tool_z=row['tool_pose_b'][2],
            clipped=status.get('target_clipped', False),
            capture_age_s=status.get('capture_age_s'),
            accepted_capture_sim_age_s=row['sim_time_s']-capture_time if capture_time is not None else None,
            source_sequence=status.get('source_sequence'),
            tilt_deg=float(np.degrees(np.arccos(np.clip(-gravity[2], -1, 1))))))
    with (output/'command_timeline.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(timeline[0]))
        writer.writeheader()
        writer.writerows(timeline)
    windows = []
    for start in range(0, int(rows[-1]['sim_time_s'])+1, 2):
        window = [r for r in timeline if start <= r['sim_time_s'] < start+2]
        cameras = [r for r in camera_rows if start <= r['capture_sim_time_s'] < start+2]
        if not window:
            continue
        report = dict(interval_s=[start, start+2], command_valid_fraction=float(np.mean([r['valid'] for r in window])),
                      vx_mean_mps=float(np.mean([r['vx'] for r in window])),
                      tool_y_mean_m=float(np.mean([r['tool_y'] for r in window])),
                      tool_clipped_fraction=float(np.mean([r['clipped'] for r in window])),
                      tilt_max_deg=max(r['tilt_deg'] for r in window))
        for key in ['capture_age_s', 'accepted_capture_sim_age_s']:
            report[key] = stats([r[key] for r in window if r[key] is not None])
        for key in ['front_seam_world_xy_mean_error_m', 'rear_seam_world_xy_mean_error_m',
                    'tcp_measurement_error_m', 'ground_normal_error_deg', 'ground_offset_error_m',
                    'rear_nominal_y_error_m', 'front_lookahead_x_m']:
            report[key] = stats([r[key] for r in cameras if key in r])
        windows.append(report)
    first_invalid = next((r for r in timeline if not r['valid']), None)
    summary = dict(session=str(session), replay_encoder_max_error_rad=replay_error,
        truth_usage='offline evaluation only; no controller or actor runs',
        contacts=contact_recorder.summary() if contact_recorder is not None else None,
        first_invalid=first_invalid,
        invalid_reasons=dict(Counter(r['reason'] for r in timeline if not r['valid'])),
        first_tilt_over_8deg=next((r for r in timeline if r['sim_time_s'] > 2 and r['tilt_deg'] > 8), None),
        windows=windows)
    (output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output), session=str(session),
        replay_encoder_max_error_rad=replay_error, first_invalid=first_invalid,
        first_tilt_over_8deg=summary['first_tilt_over_8deg'],
        invalid_reasons=summary['invalid_reasons']), ensure_ascii=False, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session', type=Path)
    parser.add_argument('--output-root', type=Path, default=ROOT/'outputs/VD1_VISION_COMMAND_AUDIT')
    parser.add_argument('--contacts', action='store_true', help='Also record evaluation-only 5ms ground contacts')
    args = parser.parse_args()
    analyze(args.session, args.output_root.resolve(), args.contacts)
