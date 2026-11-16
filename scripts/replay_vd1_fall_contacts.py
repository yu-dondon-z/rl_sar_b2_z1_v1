#!/usr/bin/env python3
"""Replay recorded VD1 actuator targets to diagnose contact/fall chronology.

This is simulation-only actuation playback, not policy inference or a camera
controller. It does not import/run an actor, use evaluation truth as feedback,
or communicate with a robot. Original sessions are opened read-only. Every
result and regenerated asset belongs to a new uniquely allocated directory.
"""
from __future__ import annotations

import argparse
from collections import deque
from datetime import datetime, timedelta, timezone
import gzip
import hashlib
from itertools import zip_longest
import json
import math
import os
from pathlib import Path
import sys
import tempfile

import mujoco
import numpy as np

from vd1_mujoco_contract import ACTOR_JOINT_NAMES, VD1MujocoAdapter
from vd1_mujoco_scene import TRAINING_ROOT, URDF_PATH, build_scene, validate_model
from vd1_mujoco_velocity_impulse import VelocityImpulseConstraints


SCRIPTS = Path(__file__).resolve().parent
DEFAULT_SESSIONS = tuple(Path('/dev/shm/vd1_mujoco_runs') / name for name in (
    'session_20261103_125842_uiozy96t', 'session_20261103_125907_5nhelup5',
    'session_20261103_125936_czdp9sxi'))
PHYSICS_SOURCES = (
    SCRIPTS / 'vd1_mujoco_scene.py', SCRIPTS / 'vd1_mujoco_contract.py',
    SCRIPTS / 'vd1_mujoco_velocity_impulse.py',
    TRAINING_ROOT / 'scripts/vision/vd1_scene.py', URDF_PATH)
FORCE_THRESHOLD_N = 1e-6


def fingerprint(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return dict(sha256=digest.hexdigest(), size_bytes=Path(path).stat().st_size)


def fingerprints(paths):
    return {str(path): fingerprint(path) for path in paths}


def save_json(path, value):
    with Path(path).open('x', encoding='utf-8') as handle:
        json.dump(value, handle, indent=2, allow_nan=False)
        handle.write('\n')


def load_json(path):
    with Path(path).open('r', encoding='utf-8') as handle:
        return json.load(handle)


def runtime_details():
    # A different BLAS thread count can change last-bit matrix solutions and
    # then amplify into a different late contact sequence. Do not force one.
    try:
        from threadpoolctl import threadpool_info
        pools = threadpool_info()
    except ImportError:
        pools = None
    return dict(python=sys.version, executable=sys.executable, numpy=np.__version__,
                mujoco=mujoco.__version__, threadpools=pools,
                environment={key: os.environ.get(key) for key in (
                    'OPENBLAS_NUM_THREADS', 'OMP_NUM_THREADS', 'MKL_NUM_THREADS',
                    'VECLIB_MAXIMUM_THREADS', 'NUMEXPR_NUM_THREADS')})


def validate_original(session, contract, current_hashes):
    """Refuse an undocumented physics reconstruction before running it."""
    launch = session['source_sha256_at_launch']
    finish = session.get('source_sha256_at_finish', {})
    for path in PHYSICS_SOURCES:
        key = str(path)
        if key not in launch or launch[key] != current_hashes[key]:
            raise ValueError('Physics dependency no longer matches original launch: ' + key)
        if key in finish and finish[key] != launch[key]:
            raise ValueError('Original physics dependency changed during session: ' + key)
    checks = dict(simulator_version=mujoco.__version__, actor_hz=50, pd_hz=200,
                  physics_hz=200.0, physics_substeps_per_pd=1, curvature_max=.05,
                  static_only=False, velocity_constraint='impulse', physx_implicit_position=True)
    for key, expected in checks.items():
        if contract.get(key) != expected:
            raise ValueError('Unsupported original runtime contract %s=%r, expected %r' %
                             (key, contract.get(key), expected))
    if contract.get('joint_names') != list(ACTOR_JOINT_NAMES):
        raise ValueError('Original recorded target joint order differs')


class ContactRecorder:
    """Evaluation-only observer; no method returns anything to actuation."""

    def __init__(self, model, output):
        self.model = model
        self.base = model.body('base_link').id
        self.tcp = model.site('tool_tcp').id
        self.feet = {model.body(leg + '_foot').id for leg in ('FL', 'FR', 'RL', 'RR')}
        self.handle = gzip.open(str(output / 'contacts_5ms.jsonl.gz'), 'xt', compresslevel=3)
        self.first_tilt = {str(value): None for value in (8, 15, 30, 45, 60)}
        self.first_gun_geometry = self.first_gun_force = self.first_nonfoot_force = None
        self.gun_contact_ticks = self.gun_force_ticks = 0
        self.max_gun_normal = self.max_nonfoot_normal = 0.
        self.max_gun_total_normal = 0.
        self.max_tilt = 0.
        self.rows = 0
        self.gun_force_episodes = []
        self._gun_episode = None
        self.first_gun_force_after = {str(value): None for value in (2, 5)}
        self.recent_gun_force = deque()

    def write(self, data, actor_step, pd_tick):
        rotation = data.xmat[self.base].reshape(3, 3)
        tilt = math.degrees(math.acos(float(np.clip(rotation[2, 2], -1., 1.))))
        force = np.empty(6)
        contacts = []
        gun_force_total = 0.
        for index in range(data.ncon):
            contact = data.contact[index]
            b1, b2 = self.model.geom_bodyid[[contact.geom1, contact.geom2]]
            if (b1 == 0) == (b2 == 0):
                continue
            robot_body = int(b2 if b1 == 0 else b1)
            if robot_body in self.feet:
                continue
            robot_geom = int(contact.geom2 if b1 == 0 else contact.geom1)
            ground_geom = int(contact.geom1 if b1 == 0 else contact.geom2)
            mujoco.mj_contactForce(self.model, data, index, force)
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, robot_body)
            normal = float(force[0])
            item = dict(body=name,
                        robot_geom=mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, robot_geom),
                        ground_geom=mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, ground_geom),
                        normal_force_N=normal, contact_frame_force_torque=force.tolist(),
                        signed_distance_m=float(contact.dist), position_w=contact.pos.tolist(),
                        gun_contact=(name == 'link_gun'), force_bearing=normal > FORCE_THRESHOLD_N)
            contacts.append(item)
            self.max_nonfoot_normal = max(self.max_nonfoot_normal, normal)
            if name == 'link_gun':
                self.max_gun_normal = max(self.max_gun_normal, normal)
                gun_force_total += normal
        event = dict(sim_time_s=float(data.time), actor_step=actor_step, pd_tick=pd_tick,
                     tilt_deg=tilt, tcp_position_w=data.site_xpos[self.tcp].tolist(),
                     base_position_w=data.xpos[self.base].tolist())
        gun_contacts = [item for item in contacts if item['gun_contact']]
        force_contacts = [item for item in contacts if item['force_bearing']]
        gun_force = [item for item in gun_contacts if item['force_bearing']]
        self.recent_gun_force.append((float(data.time), bool(gun_force), gun_force_total,
                                     max([item['normal_force_N'] for item in gun_force] or [0.])))
        while self.recent_gun_force and self.recent_gun_force[0][0] < float(data.time) - 1. - 1e-9:
            self.recent_gun_force.popleft()
        for threshold in self.first_tilt:
            if tilt >= float(threshold) and self.first_tilt[threshold] is None:
                self.first_tilt[threshold] = dict(event, gun_force_in_prior_1s_inclusive=dict(
                    window_start_sim_time_s=max(0., float(data.time) - 1.),
                    window_end_sim_time_s=float(data.time),
                    any_force_bearing_contact=any(row[1] for row in self.recent_gun_force),
                    peak_total_normal_force_N=max(row[2] for row in self.recent_gun_force),
                    peak_individual_normal_force_N=max(row[3] for row in self.recent_gun_force)))
        for threshold in self.first_gun_force_after:
            if gun_force and data.time > float(threshold) and self.first_gun_force_after[threshold] is None:
                self.first_gun_force_after[threshold] = dict(event, contacts=gun_force)
        if gun_contacts and self.first_gun_geometry is None:
            self.first_gun_geometry = dict(event, contacts=gun_contacts)
        if gun_force and self.first_gun_force is None:
            self.first_gun_force = dict(event, contacts=gun_force)
        if force_contacts and self.first_nonfoot_force is None:
            self.first_nonfoot_force = dict(event, contacts=force_contacts)
        self.gun_contact_ticks += bool(gun_contacts)
        self.gun_force_ticks += bool(gun_force)
        if gun_force:
            if self._gun_episode is None:
                self._gun_episode = dict(start_sim_time_s=float(data.time),
                                         start_tilt_deg=tilt, pd_ticks=0,
                                         peak_total_normal_force_N=0., maximum_tilt_deg=tilt)
                self.gun_force_episodes.append(self._gun_episode)
            self._gun_episode['last_sim_time_s'] = float(data.time)
            self._gun_episode['pd_ticks'] += 1
            self._gun_episode['peak_total_normal_force_N'] = max(
                self._gun_episode['peak_total_normal_force_N'], gun_force_total)
            self._gun_episode['maximum_tilt_deg'] = max(self._gun_episode['maximum_tilt_deg'], tilt)
        else:
            self._gun_episode = None
        self.max_gun_total_normal = max(self.max_gun_total_normal, gun_force_total)
        self.max_tilt = max(self.max_tilt, tilt)
        self.rows += 1
        row = dict(event, nonfoot_ground_contacts=contacts,
                   gun_ground_contact=bool(gun_contacts), gun_force_bearing=bool(gun_force),
                   gun_total_normal_force_N=gun_force_total,
                   joint_actuator_force=data.actuator_force.tolist(), evaluation_only=True)
        self.handle.write(json.dumps(row, allow_nan=False) + '\n')

    def close(self):
        self.handle.close()

    def summary(self):
        return dict(pd_tick_samples=self.rows, first_tilt_crossings_deg=self.first_tilt,
                    first_gun_geometric_contact=self.first_gun_geometry,
                    first_gun_force_bearing_contact=self.first_gun_force,
                    first_gun_force_bearing_contact_after_seconds=self.first_gun_force_after,
                    first_nonfoot_force_bearing_contact=self.first_nonfoot_force,
                    force_bearing_threshold_N=FORCE_THRESHOLD_N,
                    gun_geometric_contact_ticks=self.gun_contact_ticks,
                    gun_force_bearing_ticks=self.gun_force_ticks,
                    gun_force_bearing_episodes=self.gun_force_episodes,
                    maximum_individual_gun_contact_normal_force_N=self.max_gun_normal,
                    maximum_total_gun_normal_force_N=self.max_gun_total_normal,
                    maximum_individual_nonfoot_contact_normal_force_N=self.max_nonfoot_normal,
                    maximum_tilt_deg=self.max_tilt)


def replay(original, output_root, tolerance):
    original = Path(original).resolve()
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    output_root.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='run_%s_' % stamp, dir=str(output_root)))
    watched = tuple(PHYSICS_SOURCES) + (Path(__file__).resolve(),) + tuple(
        original / name for name in ('session.json', 'runtime_contract.json',
                                     'actor_trace.jsonl.gz', 'evaluation_truth.jsonl.gz'))
    start_hashes = fingerprints(watched)
    session = load_json(original / 'session.json')
    contract = load_json(original / 'runtime_contract.json')
    validate_original(session, contract, start_hashes)
    save_json(output / 'manifest.json', dict(original_session=str(original),
        source_and_input_sha256_at_start=start_hashes, simulator_version=mujoco.__version__,
        numpy_version=np.__version__, runtime=runtime_details(),
        physics_dependency_hashes_match_original_launch=True,
        mode='recorded_joint_target_actuation_playback_no_actor_no_vision',
        actor_used=False, evaluation_truth_used_as_feedback=False, hardware_started=False,
        poststep_base_position_tolerance_m=tolerance))
    scene = build_scene(contract['curvature_max'], output / 'assets', timestep=.005,
                        tool_markers=contract.get('tool_markers_in_scene', True),
                        road_surface=contract.get('road_surface', 'crack'),
                        gun_mass_mode=contract.get('gun_mass_mode', 'original'))
    model = mujoco.MjModel.from_xml_path(str(scene))
    validation = validate_model(model, gun_mass_mode=contract.get('gun_mass_mode', 'original'))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('vd1_default').id)
    mujoco.mj_forward(model, data)
    adapter = VD1MujocoAdapter(model, physx_implicit_position=True)
    brake = VelocityImpulseConstraints(model, adapter.joint_ids18, adapter.velocity_limit18)
    recorder = ContactRecorder(model, output)
    max_error = max_time_error = maximum_first_second_error = 0.
    first_divergence = None
    actor_steps = 0
    error = None
    try:
        with gzip.open(str(original / 'actor_trace.jsonl.gz'), 'rt') as targets, \
                gzip.open(str(original / 'evaluation_truth.jsonl.gz'), 'rt') as expected_rows, \
                gzip.open(str(output / 'poststep_verification.jsonl.gz'), 'xt', compresslevel=3) as verification:
            for step, pair in enumerate(zip_longest(targets, expected_rows)):
                if None in pair:
                    raise ValueError('Original actor/evaluation row counts differ')
                recorded, expected = (json.loads(line) for line in pair)
                if recorded['step'] != step or expected['step'] != step:
                    raise ValueError('Original rows are not contiguous in actor-step order')
                if abs(float(recorded['sim_time_s']) - data.time) > 1e-8:
                    raise ValueError('Recorded target is not a pre-actor-step target')
                target = np.asarray(recorded['joint_target'], dtype=float)
                if target.shape != (18,) or not np.isfinite(target).all():
                    raise ValueError('Recorded joint_target is not finite 18D')
                for tick in range(4):
                    mujoco.mj_forward(model, data)
                    adapter.apply_targets(data, target)
                    brake.step(data)
                    recorder.write(data, step, tick)
                # Match the original runner's post-actor-step synchronization.
                mujoco.mj_forward(model, data)
                delta = data.xpos[recorder.base] - np.asarray(expected['base_position_w'], dtype=float)
                position_error = float(np.max(np.abs(delta)))
                time_error = abs(float(data.time) - float(expected['sim_time_s']))
                max_error, max_time_error = max(max_error, position_error), max(max_time_error, time_error)
                if float(data.time) <= 1. + 1e-8:
                    maximum_first_second_error = max(maximum_first_second_error, position_error)
                row = dict(step=step, sim_time_s=float(data.time),
                           max_abs_base_position_error_m=position_error, time_error_s=time_error,
                           replay_base_position_w=data.xpos[recorder.base].tolist(),
                           original_base_position_w=expected['base_position_w'])
                verification.write(json.dumps(row, allow_nan=False) + '\n')
                if (position_error > tolerance or time_error > 1e-8) and first_divergence is None:
                    first_divergence = row.copy()
                actor_steps += 1
    except Exception as exception:
        error = repr(exception)
    finally:
        recorder.close()
    end_hashes = fingerprints(watched)
    changed = [key for key in start_hashes if start_hashes[key] != end_hashes[key]]
    summary = dict(original_session=str(original), output_dir=str(output),
                   exact_replay_verified=error is None and first_divergence is None and not changed,
                   actor_steps_replayed=actor_steps, sim_time_s=float(data.time),
                   poststep_base_position_tolerance_m=tolerance,
                   maximum_poststep_base_position_error_m=max_error,
                   maximum_poststep_base_position_error_first_1s_m=maximum_first_second_error,
                   maximum_poststep_time_error_s=max_time_error,
                   first_poststep_divergence=first_divergence, runtime_error=error,
                   source_and_input_sha256_at_finish=end_hashes,
                   files_changed_during_replay=changed, model_validation=validation,
                   velocity_constraint=brake.summary(), contact_chronology=recorder.summary(),
                   interpretation='Contact order is diagnostic evidence, not a causal counterfactual.',
                   actor_used=False, truth_feedback_used=False, hardware_started=False)
    save_json(output / 'summary.json', summary)
    print(json.dumps({key: summary[key] for key in (
        'original_session', 'output_dir', 'exact_replay_verified', 'actor_steps_replayed',
        'maximum_poststep_base_position_error_m', 'first_poststep_divergence', 'runtime_error',
        'contact_chronology')}, allow_nan=False), flush=True)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sessions', nargs='+', type=Path, default=list(DEFAULT_SESSIONS))
    parser.add_argument('--output-root', type=Path, default=Path('/dev/shm/vd1_fall_replay'))
    parser.add_argument('--tolerance-m', type=float, default=1e-6)
    args = parser.parse_args()
    if not math.isfinite(args.tolerance_m) or args.tolerance_m <= 0:
        parser.error('tolerance-m must be finite and positive')
    summaries = [replay(path, args.output_root, args.tolerance_m) for path in args.sessions]
    return 0 if all(row['exact_replay_verified'] for row in summaries) else 1


if __name__ == '__main__':
    raise SystemExit(main())
