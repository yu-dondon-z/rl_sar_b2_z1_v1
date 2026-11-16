#!/usr/bin/env python3
"""Simulation-only VD1 open-loop command response; no vision or robot SDK.

The actor receives exactly the existing 70D sensor-only observation, scheduled
base commands and one frozen camera-derived tool command. Simulator truth is
written to a separate evaluation sink and is never returned to the actor.
This is a diagnostic, not a deployable replacement for camera perception.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import gzip
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import signal
import sys
import tempfile
import time
import xml.etree.ElementTree as ET

import numpy as np


TRAINING_ROOT = Path('/home/dong/b2_z1')
DEFAULT_TRACE = Path('/dev/shm/vd1_mujoco_runs/session_20261103_125936_czdp9sxi/actor_trace.jsonl.gz')
DEFAULT_POLICY = TRAINING_ROOT / '.pretrained_checkpoints/VD1/exported/policy.pt'
POLICY_SHA256 = '86be6c2d1c7a0e56fe4bd93c2d62622469c030b0cdc6d10db7af001eae3f889a'
CASES = ('straight', 'left', 'right', 'stop', 'restart')
ACTOR_DT, PD_DT = .02, .005


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group()
    source.add_argument('--trace', type=Path, default=DEFAULT_TRACE)
    source.add_argument('--tool-sample', type=Path,
                        help='Persistent recorded fixed 7D command JSON; no live camera or RAM trace required')
    parser.add_argument('--sample-time', type=float, default=4.)
    parser.add_argument('--tool-ramp-s', type=float, default=0.,
                        help='Optional diagnostic XYZ-linear ramp from initial encoder FK; default off')
    parser.add_argument('--tool-ramp-orientation', choices=('target', 'slerp'), default='target',
                        help='Use target orientation immediately (source behavior), or a separate SLERP diagnostic')
    parser.add_argument('--policy', type=Path, default=DEFAULT_POLICY)
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--cases', nargs='+', choices=CASES, default=list(CASES))
    parser.add_argument('--speed', type=float, default=.4)
    parser.add_argument('--yaw-rate', type=float, default=.15)
    parser.add_argument('--physics-dt', type=float, choices=(.001, .005), default=.005)
    parser.add_argument('--velocity-constraint', choices=('off', 'impulse'), default='impulse')
    parser.add_argument('--physx-implicit-position', dest='physx_implicit_position', action='store_true', default=True)
    parser.add_argument('--no-physx-implicit-position', dest='physx_implicit_position', action='store_false')
    parser.add_argument('--curvature-max', type=float, default=.05)
    parser.add_argument('--flat-ground', action='store_true')
    parser.add_argument('--viewer', action='store_true', help='Open a presentation-only following camera; SPACE pauses/resumes')
    parser.add_argument('--start-paused', action='store_true', help='Wait for SPACE before integrating (requires --viewer)')
    parser.add_argument('--realtime-factor', type=float, default=1., help='Viewer pacing only; 0.5 is slow motion, default 1')
    parser.add_argument('--output-root', type=Path, default=Path('/dev/shm/vd1_base_response'))
    args = parser.parse_args(argv)
    for name in ('duration', 'speed', 'yaw_rate', 'sample_time', 'curvature_max', 'tool_ramp_s', 'realtime_factor'):
        if not math.isfinite(getattr(args, name)):
            parser.error('--' + name.replace('_', '-') + ' must be finite')
    if not 0 < args.duration <= 600 or not 0 < args.speed <= 1:
        parser.error('duration must be in (0,600] seconds and speed in (0,1] m/s')
    if not 0 <= args.sample_time or not 0 <= args.yaw_rate <= 1 or not 0 <= args.curvature_max <= .15:
        parser.error('invalid sample-time, yaw-rate or curvature-max')
    if not 0 <= args.tool_ramp_s <= 600:
        parser.error('tool-ramp-s must be within [0,600] seconds')
    if len(set(args.cases)) != len(args.cases):
        parser.error('cases must be distinct')
    if not 0 < args.realtime_factor <= 4:
        parser.error('realtime-factor must be in (0,4]')
    if args.start_paused and not args.viewer:
        parser.error('--start-paused requires --viewer')
    return args


def smoothstep(value):
    value = min(1., max(0., float(value)))
    return value * value * (3. - 2. * value)


def command_at(case, elapsed_s, speed=.4, yaw_rate=.15):
    """Deterministic commands; deliberately has no model/data argument."""
    if case not in CASES:
        raise ValueError('Unknown diagnostic case: ' + case)
    if not all(math.isfinite(x) for x in (elapsed_s, speed, yaw_rate)):
        raise ValueError('Nonfinite command schedule input')
    forward = speed * smoothstep(elapsed_s / 2.)
    if case in ('stop', 'restart'):
        forward *= 1. - smoothstep((elapsed_s - 15.) / 2.)
        if case == 'restart':
            forward += speed * smoothstep((elapsed_s - 22.) / 2.)
    turn = 0.
    if case in ('left', 'right'):
        turn = yaw_rate * smoothstep((elapsed_s - 10.) / .5) * (1. - smoothstep((elapsed_s - 19.5) / .5))
        if case == 'right':
            turn = -turn
    return np.asarray((forward, 0., turn), dtype=float)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def write_json(path, payload):
    # Every result path belongs to a newly allocated directory. Never replace
    # another session, checkpoint, source file or previous diagnostic result.
    with Path(path).open('x', encoding='utf-8') as handle:
        json.dump(payload, handle, indent=2, allow_nan=False)
        handle.write('\n')


def load_tool_sample(path, sample_time):
    """Extract only an actually recorded 7D command, not evaluation truth."""
    opener = gzip.open if Path(path).suffix == '.gz' else open
    best = None
    with opener(path, 'rt', encoding='utf-8') as handle:
        for number, line in enumerate(handle, 1):
            row = json.loads(line)
            if not isinstance(row, dict):
                raise ValueError('Trace row must be an object')
            timestamp = float(row['sim_time_s'])
            if not math.isfinite(timestamp):
                raise ValueError('Trace has a nonfinite timestamp')
            if best is None or abs(timestamp - sample_time) < abs(best['sim_time_s'] - sample_time):
                tool = np.asarray(row['tool_pose_b'], dtype=float)
                if tool.shape != (7,) or not np.isfinite(tool).all() or abs(np.linalg.norm(tool[3:]) - 1.) > 1e-3:
                    raise ValueError('Trace tool_pose_b must be finite xyz and unit wxyz')
                best = dict(line=number, step=row.get('step'), sim_time_s=timestamp, tool_pose_b=tool.tolist(),
                            status=row.get('status'), base_command_b=row.get('base_command_b'))
    if best is None:
        raise ValueError('Trace is empty')
    if abs(best['sim_time_s'] - sample_time) > .021:
        raise ValueError('No trace sample within one actor tick of requested sample time')
    return best


def load_fixed_tool_sample(path):
    """Load a recorded command only; do not use any actual TCP/world state."""
    with Path(path).open('r', encoding='utf-8') as handle:
        sample = json.load(handle)
    if not isinstance(sample, dict) or sample.get('schema') != 'vd1_fixed_camera_tool_command_v1':
        raise ValueError('Unsupported fixed tool sample schema')
    tool = np.asarray(sample.get('tool_pose_b'), dtype=float)
    if tool.shape != (7,) or not np.isfinite(tool).all() or abs(np.linalg.norm(tool[3:]) - 1.) > 1e-3:
        raise ValueError('Fixed tool sample must contain finite XYZ/unit wxyz')
    return dict(schema=sample['schema'], tool_pose_b=tool.tolist(),
                source_trace=sample.get('source_trace'), source_trace_sha256=sample.get('source_trace_sha256'),
                line=sample.get('line'), step=sample.get('step'), sim_time_s=sample.get('sim_time_s'))


def quaternion_from_rotation(rotation):
    """Ordinary rotation-matrix to unit wxyz quaternion, no simulator access."""
    matrix = np.asarray(rotation, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError('Rotation must be a finite 3x3 matrix')
    # Symmetric eigenproblem avoids division near a 180-degree rotation.
    r00, r01, r02 = matrix[0]
    r10, r11, r12 = matrix[1]
    r20, r21, r22 = matrix[2]
    symmetric = np.asarray([
        [r00-r11-r22, r01+r10, r02+r20, r21-r12],
        [r01+r10, r11-r00-r22, r12+r21, r02-r20],
        [r02+r20, r12+r21, r22-r00-r11, r10-r01],
        [r21-r12, r02-r20, r10-r01, r00+r11+r22]]) / 3.
    _, vectors = np.linalg.eigh(symmetric)
    xyzw = vectors[:, -1]
    quaternion = xyzw[[3, 0, 1, 2]]
    if quaternion[0] < 0:
        quaternion = -quaternion
    return quaternion / np.linalg.norm(quaternion)


def initial_encoder_pose(q18, joint_names):
    """Full TCP pose using the source pure-URDF FK chain and encoders only."""
    import torch
    from vd1_mujoco_scene import URDF_PATH
    path = TRAINING_ROOT / 'source/b2_z1/b2_z1/tasks/manager_based/b2_z1/mdp/commands/dual_vision_kinematics.py'
    spec = importlib.util.spec_from_file_location('vd1_diagnostic_encoder_fk', str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    fk = module.URDFToolKinematics(URDF_PATH, device='cpu')
    arm = torch.tensor([[q18[joint_names.index(name)] for name in fk.joint_names]], dtype=torch.float64)
    with torch.inference_mode():
        position = fk.position(arm).numpy()[0]
        rotation = torch.eye(3, dtype=arm.dtype).unsqueeze(0)
        for number, _, index in fk.chain:
            rotation = rotation @ getattr(fk, 'origin_R_%d' % number).to(dtype=arm.dtype)
            if index is not None:
                rotation = rotation @ module._axis_rotation(getattr(fk, 'axis_%d' % number).to(dtype=arm.dtype), arm[:, index])
        quaternion = quaternion_from_rotation(rotation.numpy()[0])
    return np.concatenate((position, quaternion))


def tool_command_at(elapsed_s, initial_pose_b, target_pose_b, ramp_s=0., orientation='target'):
    """Optional diagnostic ramp, not the exact training waypoint trajectory.

    Source pose_command_wbc uses a base-frame cubic Bezier position curve and
    immediate target orientation. This isolated diagnostic uses one base-frame
    straight line, with either immediate target orientation or optional SLERP,
    to separate initial target discontinuity from terrain/drive differences.
    No world state or actual simulated TCP is consumed here.
    """
    target = np.asarray(target_pose_b, dtype=float)
    if target.shape != (7,) or not np.isfinite(target).all() or abs(np.linalg.norm(target[3:]) - 1.) > 1e-3:
        raise ValueError('Target tool pose must be finite XYZ/unit wxyz')
    if not math.isfinite(elapsed_s) or not math.isfinite(ramp_s) or ramp_s < 0:
        raise ValueError('Invalid tool ramp time')
    if orientation not in ('target', 'slerp'):
        raise ValueError('Unknown tool ramp orientation mode')
    if ramp_s == 0:
        return target.copy()
    start = np.asarray(initial_pose_b, dtype=float)
    if start.shape != (7,) or not np.isfinite(start).all() or abs(np.linalg.norm(start[3:]) - 1.) > 1e-3:
        raise ValueError('Initial encoder tool pose must be finite XYZ/unit wxyz')
    blend = np.clip(elapsed_s / ramp_s, 0., 1.)
    position = (1.-blend)*start[:3] + blend*target[:3]
    if orientation == 'target':
        return np.concatenate((position, target[3:]))
    q0 = start[3:] / np.linalg.norm(start[3:])
    q1 = target[3:] / np.linalg.norm(target[3:])
    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1, dot = -q1, -dot
    dot = float(np.clip(dot, -1., 1.))
    if dot > .9995:
        quaternion = (1.-blend)*q0 + blend*q1
    else:
        angle = math.acos(dot)
        quaternion = (math.sin((1.-blend)*angle)*q0 + math.sin(blend*angle)*q1) / math.sin(angle)
    quaternion /= np.linalg.norm(quaternion)
    return np.concatenate((position, quaternion))


def make_flat_ground(scene):
    """Change terrain only in this session's generated XML cache."""
    scene = Path(scene)
    tree = ET.parse(scene)
    root = tree.getroot()
    world = root.find('worldbody')
    removed = []
    for geom in list(world.findall('geom')):
        name = geom.get('name', '')
        if name == 'crack_floor' or name.startswith('road_'):
            removed.append(name)
            world.remove(geom)
    if not removed or 'crack_floor' not in removed:
        raise ValueError('Expected VD1 road geoms were not found')
    asset = root.find('asset')
    if asset is not None:
        for mesh in list(asset.findall('mesh')):
            if mesh.get('name', '').startswith('road_'):
                asset.remove(mesh)
    ET.SubElement(world, 'geom', name='diagnostic_flat_ground', type='plane', size='100 100 .1',
                  pos='0 0 0', rgba='.42 .42 .42 1', contype='1', conaffinity='2',
                  friction='1 .005 .0001', condim='3', solref='.005 1')
    output = scene.with_name('vd1_flat_ground_diagnostic.xml')
    if output.exists():
        raise FileExistsError(output)
    tree.write(output, encoding='utf-8', xml_declaration=True)
    return output


def dynamics_hash(model):
    """Hash immutable dynamics configuration, not changing integration state."""
    digest = hashlib.sha256()
    names = ('body_mass', 'body_inertia', 'body_ipos', 'body_iquat', 'body_pos', 'body_quat',
             'jnt_type', 'jnt_pos', 'jnt_axis', 'jnt_range', 'jnt_stiffness',
             'dof_damping', 'dof_armature', 'dof_frictionloss',
             'actuator_gear', 'actuator_gainprm', 'actuator_biasprm', 'actuator_forcerange',
             'actuator_ctrlrange', 'actuator_dyntype', 'actuator_gaintype', 'actuator_biastype',
             'geom_type', 'geom_size', 'geom_pos', 'geom_quat', 'geom_friction',
             'geom_solref', 'geom_solimp', 'geom_contype', 'geom_conaffinity')
    for name in names:
        array = np.ascontiguousarray(getattr(model, name))
        digest.update(name.encode())
        digest.update(str(array.dtype).encode())
        digest.update(str(array.shape).encode())
        digest.update(array.tobytes())
    digest.update(json.dumps(dict(timestep=float(model.opt.timestep), integrator=int(model.opt.integrator),
                                 cone=int(model.opt.cone), iterations=int(model.opt.iterations),
                                 gravity=model.opt.gravity.tolist(), impratio=float(model.opt.impratio)),
                             sort_keys=True, allow_nan=False).encode())
    return digest.hexdigest()


def link_frame_velocity(model, data, body_id):
    """Angular then linear velocity at the ordinary link frame origin.

    mjOBJ_BODY with local=1 instead returns the principal-inertia frame; a
    nonidentity body_iquat makes its XYZ labels unsuitable for base commands.
    mjOBJ_XBODY selects the regular body/link frame and origin. This evaluation
    helper reads no sensors and is never called by observation construction.
    """
    import mujoco
    velocity = np.empty(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY, body_id, velocity, 1)
    return velocity


class TruthRecorder:
    """One-way evaluation sink; these measurements never build observations."""
    def __init__(self, model, output):
        import mujoco
        self.model = model
        self.base = model.body('base_link').id
        self.tcp = model.site('tool_tcp').id
        self.feet = [model.body(leg + '_foot').id for leg in ('FL', 'FR', 'RL', 'RR')]
        self.handle = gzip.open(str(output / 'evaluation_truth.jsonl.gz'), 'xt', compresslevel=3)
        self.rows = []
        self.first_threshold = {str(value): None for value in (15, 30, 45, 60)}
        self.first_position = None
        self.gyro_crosscheck_max_abs_error = 0.

    def write(self, data, step):
        import mujoco
        rotation = data.xmat[self.base].reshape(3, 3)
        velocity = link_frame_velocity(self.model, data, self.base)
        # Independent invariant: with the validated identity IMU/site rotation,
        # link-frame angular velocity must equal the raw gyro sensor. Never
        # replace either with the other to manufacture agreement.
        gyro_error = float(np.max(np.abs(velocity[:3] - data.sensor('imu_gyro').data)))
        self.gyro_crosscheck_max_abs_error = max(self.gyro_crosscheck_max_abs_error, gyro_error)
        if gyro_error > 1e-8:
            raise ValueError('Evaluation link angular velocity disagrees with raw IMU gyro: %g' % gyro_error)
        tilt = math.degrees(math.acos(float(np.clip(rotation[2, 2], -1., 1.))))
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        forces = np.zeros((4, 3))
        contact_force = np.empty(6)
        nonfoot_ground = []
        for index in range(data.ncon):
            contact = data.contact[index]
            b1 = self.model.geom_bodyid[contact.geom1]
            b2 = self.model.geom_bodyid[contact.geom2]
            ground_contact = (b1 == 0) != (b2 == 0)
            robot_body = b2 if b1 == 0 else b1
            nonfoot = ground_contact and robot_body not in self.feet
            if b1 not in self.feet and b2 not in self.feet and not nonfoot:
                continue
            mujoco.mj_contactForce(self.model, data, index, contact_force)
            force_w = contact.frame.reshape(3, 3).T @ contact_force[:3]
            for foot, body in enumerate(self.feet):
                if b1 == body:
                    forces[foot] -= force_w
                if b2 == body:
                    forces[foot] += force_w
            if nonfoot:
                body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, robot_body)
                robot_geom = contact.geom2 if b1 == 0 else contact.geom1
                ground_geom = contact.geom1 if b1 == 0 else contact.geom2
                nonfoot_ground.append(dict(body=body_name,
                    robot_geom=mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, robot_geom),
                    ground_geom=mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, ground_geom),
                    normal_force_N=float(contact_force[0]), position_w=contact.pos.tolist(),
                    signed_distance_m=float(contact.dist), gun_contact=(body_name == 'link_gun')))
        tcp_position_w = data.site_xpos[self.tcp]
        row = dict(step=step, sim_time_s=float(data.time), base_position_w=data.xpos[self.base].tolist(),
                   body_velocity_b=velocity[3:].tolist(), body_angular_velocity_b=velocity[:3].tolist(),
                   velocity_frame='base_link ordinary frame and origin (mjOBJ_XBODY)',
                   body_velocity_w=(rotation @ velocity[3:]).tolist(),
                   gyro_crosscheck_max_abs_error_radps=gyro_error,
                   yaw_rad=yaw, roll_rad=math.atan2(rotation[2, 1], rotation[2, 2]),
                   pitch_rad=math.asin(float(np.clip(-rotation[2, 0], -1., 1.))), tilt_deg=tilt,
                   foot_net_force_w=forces.tolist(), foot_contact_over_5N=(forces[:, 2] > 5.).tolist(),
                   tcp_position_w=tcp_position_w.tolist(),
                   tcp_position_b=(rotation.T @ (tcp_position_w - data.xpos[self.base])).tolist(),
                   nonfoot_ground_contacts=nonfoot_ground,
                   gun_ground_contact=any(contact['gun_contact'] for contact in nonfoot_ground),
                   actuator_force=data.actuator_force.tolist(), joint_constraint_force=data.qfrc_constraint.tolist(),
                   evaluation_only=True)
        if not np.isfinite(np.asarray(row['base_position_w'] + row['body_velocity_b'] + [tilt])).all():
            raise ValueError('Nonfinite simulation evaluation state')
        self.handle.write(json.dumps(row, allow_nan=False) + '\n')
        if self.first_position is None:
            self.first_position = np.asarray(row['base_position_w'])
        self.rows.append(row)
        for threshold in self.first_threshold:
            if tilt >= float(threshold) and self.first_threshold[threshold] is None:
                self.first_threshold[threshold] = float(data.time)

    def summary(self, case, duration):
        if not self.rows:
            return dict(evaluation_only=True, samples=0)
        windows = dict(startup=(0., 2.), straight_steady=(4., 10.))
        if case in ('left', 'right'):
            windows.update(turn_steady=(10.5, 19.5), after_turn=(22., duration))
        if case in ('stop', 'restart'):
            windows['stopped'] = (17., 22. if case == 'restart' else duration)
        if case == 'restart':
            windows['restarted'] = (24., duration)
        metrics = {}
        for label, (start, end) in windows.items():
            rows = [row for row in self.rows if start <= row['sim_time_s'] <= end + 1e-7]
            if not rows:
                metrics[label] = dict(samples=0, requested_interval_s=[start, end], covered=False)
                continue
            linear = np.asarray([r['body_velocity_b'] for r in rows])
            angular = np.asarray([r['body_angular_velocity_b'] for r in rows])
            tilts = np.asarray([r['tilt_deg'] for r in rows])
            displacement = np.asarray(rows[-1]['base_position_w']) - rows[0]['base_position_w']
            yaws = np.unwrap([r['yaw_rad'] for r in rows])
            metrics[label] = dict(samples=len(rows), requested_interval_s=[start, end],
                                  actual_interval_s=[rows[0]['sim_time_s'], rows[-1]['sim_time_s']],
                                  covered=rows[-1]['sim_time_s'] >= end - ACTOR_DT - 1e-7,
                                  body_velocity_mean_mps=linear.mean(axis=0).tolist(),
                                  body_velocity_std_mps=linear.std(axis=0).tolist(),
                                  yaw_rate_mean_radps=float(angular[:, 2].mean()),
                                  yaw_change_rad=float(yaws[-1] - yaws[0]),
                                  tilt_max_deg=float(tilts.max()), tilt_rms_deg=float(np.sqrt(np.mean(tilts**2))),
                                  displacement_w_m=displacement.tolist())
        return dict(samples=len(self.rows), windows=metrics,
                    first_tilt_threshold_time_s=self.first_threshold,
                    tilt_max_deg=max(r['tilt_deg'] for r in self.rows),
                    velocity_frame='base_link ordinary frame and origin (mjOBJ_XBODY)',
                    gyro_crosscheck_max_abs_error_radps=getattr(self, 'gyro_crosscheck_max_abs_error', None),
                    first_nonfoot_ground_contact_time_s=next((r['sim_time_s'] for r in self.rows
                        if r.get('nonfoot_ground_contacts')), None),
                    first_gun_ground_contact_time_s=next((r['sim_time_s'] for r in self.rows
                        if r.get('gun_ground_contact')), None),
                    nonfoot_ground_contact_max_normal_force_N=max([c['normal_force_N'] for r in self.rows
                        for c in r.get('nonfoot_ground_contacts', [])] or [0.]),
                    base_displacement_w_m=(np.asarray(self.rows[-1]['base_position_w']) - self.first_position).tolist(),
                    evaluation_only=True, truth_never_returned_to_actor=True)


def run_case(args, case, actor, tool_pose, output, stopped):
    import mujoco
    import torch
    from vd1_control import build_actor_observation
    from vd1_mujoco_contract import VD1MujocoAdapter, ACTOR_JOINT_NAMES
    from vd1_mujoco_scene import build_scene, validate_model
    from vd1_mujoco_velocity_impulse import VelocityImpulseConstraints
    output.mkdir()
    scene = build_scene(args.curvature_max, output / 'scene_cache', timestep=args.physics_dt)
    if args.flat_ground:
        scene = make_flat_ground(scene)
    model = mujoco.MjModel.from_xml_path(str(scene))
    validation = validate_model(model)
    source_hash = dynamics_hash(model)
    adapter = VD1MujocoAdapter(model, physx_implicit_position=args.physx_implicit_position)
    initial_hash = dynamics_hash(model)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('vd1_default').id)
    mujoco.mj_forward(model, data)
    encoder_pose = (initial_encoder_pose(adapter.read_proprioception(data)['q'], list(ACTOR_JOINT_NAMES))
                    if args.tool_ramp_s > 0 else None)
    brake = (VelocityImpulseConstraints(model, adapter.joint_ids18, adapter.velocity_limit18)
             if args.velocity_constraint == 'impulse' else None)
    truth = TruthRecorder(model, output)
    previous_action = np.zeros(18, dtype=np.float32)
    steps = 0
    reason = 'duration_complete'
    error = None
    viewer = None
    try:
        if args.viewer:
            from vd1_base_response_viewer import DiagnosticViewer
            viewer = DiagnosticViewer(model, data, start_paused=args.start_paused,
                                      realtime_factor=args.realtime_factor)
            viewer.update(data, command_at(case, 0., args.speed, args.yaw_rate))
            print('[VD1 debug] SPACE: start/pause; mouse: orbit/zoom; close window or Ctrl+C: exit', flush=True)
        with gzip.open(str(output / 'actor_trace.jsonl.gz'), 'xt', compresslevel=3) as handle:
            for step in range(int(math.ceil(args.duration / ACTOR_DT))):
                if stopped[0]:
                    reason = 'interrupted'
                    break
                if viewer is not None:
                    previous_command = command_at(case, float(data.time), args.speed, args.yaw_rate)
                    if not viewer.wait_until_running(data, previous_command, truth.rows[-1] if truth.rows else None, stopped):
                        reason = 'interrupted' if stopped[0] else 'viewer_closed'
                        break
                tick_start = time.monotonic()
                measured = adapter.read_proprioception(data)
                command = command_at(case, float(data.time), args.speed, args.yaw_rate)
                commanded_tool_pose = tool_command_at(float(data.time), encoder_pose, tool_pose,
                                                     args.tool_ramp_s, args.tool_ramp_orientation)
                observation = build_actor_observation(command, commanded_tool_pose, measured['gyro_b'],
                              measured['gravity_b'], measured['q'], measured['qd'], adapter.default_q18, previous_action)
                with torch.inference_mode():
                    action = actor(torch.from_numpy(observation).unsqueeze(0)).numpy()[0]
                if action.shape != (18,) or not np.isfinite(action).all():
                    raise ValueError('Actor action is not finite 18D')
                target = np.clip(adapter.default_q18 + .25 * action, -100., 100.)
                row = dict(step=step, sim_time_s=float(data.time), base_command_b=command.tolist(),
                           tool_pose_b=commanded_tool_pose.tolist(), observation=observation.tolist(), action=action.tolist(),
                           previous_action=previous_action.tolist(), joint_target=target.tolist(),
                           sensors={name: value.tolist() for name, value in measured.items()},
                           raw_sensordata=data.sensordata.tolist(), privileged_actor_inputs=[])
                diagnostics = None
                for _ in range(4):
                    mujoco.mj_forward(model, data)
                    diagnostics = adapter.apply_targets(data, target)
                    for _ in range(int(round(PD_DT / args.physics_dt))):
                        if brake is None:
                            mujoco.mj_step(model, data)
                        else:
                            brake.step(data)
                mujoco.mj_forward(model, data)
                row['actuator_diagnostics_last_pd_tick'] = diagnostics
                handle.write(json.dumps(row, allow_nan=False, default=lambda value: value.tolist()) + '\n')
                truth.write(data, step)
                previous_action = action.copy()
                steps += 1
                if viewer is not None:
                    viewer.update(data, command, truth.rows[-1])
                if step == 0 or steps % (50 if args.viewer else 500) == 0:
                    evaluation = truth.rows[-1]
                    print('[VD1 base response] case=%s sim=%.2fs cmd=%s actual_vx=%.3f m/s tilt=%.2f deg [EVAL ONLY]' %
                          (case, data.time, np.round(command, 3).tolist(),
                           evaluation['body_velocity_b'][0], evaluation['tilt_deg']), flush=True)
                # Same sensor-only 60-degree tilt termination as the existing
                # simulation runner. Truth remains an independent log sink.
                gravity = adapter.read_proprioception(data)['gravity_b']
                if gravity[2] > -.5:
                    reason = 'IMU_tilt_stop'
                    break
                if viewer is not None:
                    viewer.pace(tick_start, ACTOR_DT, stopped)
    except Exception as exc:
        reason, error = 'runtime_error', repr(exc)
    finally:
        if viewer is not None:
            viewer.close()
        truth.handle.close()
        final_hash = dynamics_hash(model)
        result = dict(case=case, actor_steps=steps, sim_time_s=float(data.time), stop_reason=reason,
                      runtime_error=error, source_dynamics_sha256=source_hash,
                      viewer_enabled=args.viewer, viewer_physics_isolated=True,
                      tool_ramp_s=args.tool_ramp_s,
                      tool_ramp_orientation=args.tool_ramp_orientation,
                      initial_tool_pose_from_encoders_b=encoder_pose.tolist() if encoder_pose is not None else None,
                      initial_dynamics_sha256=initial_hash, final_dynamics_sha256=final_hash,
                      dynamics_unchanged_during_trial=(initial_hash == final_hash),
                      model_validation=validation, evaluation=truth.summary(case, args.duration),
                      velocity_constraint=brake.summary() if brake else None)
        write_json(output / 'summary.json', result)
    print('[VD1 base response] case=%s result=%s sim=%.2fs' % (case, reason, data.time), flush=True)
    return result


def main(argv=None):
    args = arguments(argv)
    sys.path.insert(0, str(TRAINING_ROOT / 'scripts/vision'))
    import mujoco
    import torch
    torch.set_num_threads(1)
    args.trace = args.trace.expanduser().resolve()
    if args.tool_sample is not None:
        args.tool_sample = args.tool_sample.expanduser().resolve()
    args.policy = args.policy.expanduser().resolve()
    if sha256(args.policy) != POLICY_SHA256:
        raise ValueError('Policy SHA256 differs from original VD1 actor')
    sample = (load_fixed_tool_sample(args.tool_sample) if args.tool_sample is not None
              else load_tool_sample(args.trace, args.sample_time))
    actor = torch.jit.load(str(args.policy), map_location='cpu').eval()
    with torch.inference_mode():
        if actor(torch.zeros(1, 70)).shape != (1, 18):
            raise ValueError('Actor must map 70 observations to 18 actions')
    root = args.output_root.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    output = Path(tempfile.mkdtemp(prefix='response_' + stamp + '_', dir=str(root)))
    from vd1_mujoco_contract import ACTOR_JOINT_NAMES
    from vd1_mujoco_scene import URDF_PATH
    source_paths = [Path(__file__).resolve(), Path(__file__).with_name('vd1_mujoco_contract.py'),
                    Path(__file__).with_name('vd1_mujoco_scene.py'),
                    Path(__file__).with_name('vd1_mujoco_velocity_impulse.py'),
                    TRAINING_ROOT / 'scripts/vision/vd1_control.py', URDF_PATH,
                    TRAINING_ROOT / 'scripts/vision/vd1_scene.py',
                    TRAINING_ROOT / 'source/b2_z1/b2_z1/tasks/manager_based/b2_z1/mdp/commands/dual_vision_kinematics.py']
    if args.tool_sample is not None:
        source_paths.append(args.tool_sample)
    if args.viewer:
        source_paths.append(Path(__file__).with_name('vd1_base_response_viewer.py'))
    initial_source_hashes = {str(path): sha256(path) for path in source_paths}
    write_json(output / 'manifest.json', dict(
        simulation_only=True, robot_communication=False, vision_disabled_for_diagnostic=True,
        command_source='deterministic open-loop schedule', tool_pose_source='frozen recorded camera-derived command',
        recorded_sample=sample,
        trace=str(args.trace) if args.tool_sample is None else None,
        trace_sha256=sha256(args.trace) if args.tool_sample is None else None,
        fixed_tool_sample=str(args.tool_sample) if args.tool_sample is not None else None,
        fixed_tool_sample_sha256=sha256(args.tool_sample) if args.tool_sample is not None else None,
        viewer_enabled=args.viewer, viewer_start_paused=args.start_paused,
        viewer_realtime_factor=args.realtime_factor, viewer_physics_isolated=True,
        tool_ramp_s=args.tool_ramp_s,
        tool_ramp_orientation=args.tool_ramp_orientation,
        tool_ramp_method='optional base-frame XYZ-linear ramp from initial encoder FK; orientation mode recorded separately; not exact source cubic Bezier',
        policy=str(args.policy), policy_sha256=POLICY_SHA256, actor_hz=50, pd_hz=200,
        physics_dt=args.physics_dt, velocity_constraint=args.velocity_constraint,
        physx_implicit_position=args.physx_implicit_position, speed_mps=args.speed, yaw_rate_radps=args.yaw_rate,
        terrain='flat plane z=0' if args.flat_ground else 'unchanged physical crack', curvature_max=args.curvature_max,
        duration_per_case_s=args.duration, cases=args.cases, joint_names=list(ACTOR_JOINT_NAMES),
        mujoco_version=mujoco.__version__, torch_version=torch.__version__,
        actor_inputs=['scheduled base command', 'frozen recorded tool command', 'encoder jointpos and jointvel',
                      'IMU gyro and projected gravity', 'previous raw actor action'],
        privileged_actor_inputs=[], evaluation_sink='separate evaluation_truth.jsonl.gz; no returned values',
        schedule=dict(speed_ramp_s=2., turn_start_s=10., turn_end_s=20., turn_transition_s=.5,
                      stop_start_s=15., stop_end_s=17., restart_start_s=22., restart_end_s=24.),
        source_hashes=initial_source_hashes))
    print('[VD1 base response] output: ' + str(output), flush=True)
    stopped = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.__setitem__(0, True))
    results = []
    for case in args.cases:
        if stopped[0]:
            break
        results.append(run_case(args, case, actor, np.asarray(sample['tool_pose_b']), output / case, stopped))
    final_source_hashes = {str(path): sha256(path) for path in source_paths}
    write_json(output / 'summary.json', dict(cases=results, requested_cases=args.cases, interrupted=stopped[0],
               source_hashes_at_finish=final_source_hashes,
               source_unchanged_during_run=initial_source_hashes == final_source_hashes))
    return int(any(row['runtime_error'] or not row['dynamics_unchanged_during_trial'] for row in results))


if __name__ == '__main__':
    sys.exit(main())
