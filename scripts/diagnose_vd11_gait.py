#!/usr/bin/env python3
"""Single-episode VD11 new-gun MuJoCo diagnostic with an independent geometry guard.

By default, simulator truth generates a straight WORLD seam command using the
training command equations. This is NOT camera-based deployment. The unchanged
70-value actor receives only generated commands and encoder/IMU observations;
world state is not appended to its observations. ``legacy-fixed-body`` preserves
the old, non-equivalent diagnostic explicitly. Existing VD5 defaults are untouched.
"""

from __future__ import annotations

import argparse
from collections import deque
import gzip
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import sys
import tempfile
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
TRAINING = Path('/home/dong/b2_z1_VD2')
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, '/home/dong/b2_z1/scripts/vision')

from vd1_mujoco_scene import (ARM_JOINT_NAMES, DEFAULT_JOINT_POSITION,
                             EXPECTED_JOINT_NAMES, VD10_TCP_POSITION,
                             VD10_TCP_ROTATION, VD10_URDF_PATH, VD10_URDF_SHA256)
from vd5_mujoco_control import (_tracking_errors, normalized_lerp_quaternion,
    quaternion_conjugate, quaternion_from_euler_xyz, quaternion_matrix,
    quaternion_multiply, sample_polyline_behind_position, startup_fraction)

DT, ACTOR_DT, DECIMATION = .005, .02, 4
BASE_HEIGHT, RAMP_S = .5, 2.
ARM_RESET = (0., .66647356, -1.25178768, .88531411, 0., 0.)
PITCH = math.pi / 2 - .45
FEET = ('FL_foot', 'FR_foot', 'RL_foot', 'RR_foot')
COMMAND_MODES = ('world-seam', 'legacy-fixed-body')
GEOMETRY_MODULE = TRAINING / 'source/b2_z1/b2_z1/assets/vd11_geometry_audit.py'
# Exact ideal-camera command parameters saved with VD11 model_5999. They are
# teacher geometry, not rendered images or claims of measured camera freshness.
COMMAND_CAMERA = dict(position_b=[.44643878, -.00569431, -.00031663],
    rpy_b=[-.01715695, .02446081, .00610092],
    intrinsics=[251.47, 251.47, 320., 180.], image_size=[640, 360],
    depth_range=[.05, 5.], lookahead_m=.6, heading_gain=1.5, yaw_rate_limit=.8)
EXPERIMENTS = {'b2_z1_vd10_hollow_gun_amp': 5998,
               'b2_z1_vd11_motion_quality_amp': 5999}
PROFILE = dict(version=1, duration_s=RAMP_S, reference_initialization_prob=.8,
               nominal_base_height_m=BASE_HEIGHT, nominal_joint_velocity_range=[0., 0.],
               vd10=dict(version=2, safe_pitch_rad=.3, seam_pitch_rad=PITCH,
                         working_surface_clearance_m=.005, tube_length_m=1.1,
                         tube_outer_radius_m=.028, reset_commands_after_forward=True,
                         arm_joint_reset_rad=list(ARM_RESET)))


def default_position():
    positions = dict(DEFAULT_JOINT_POSITION)
    positions.update(zip(ARM_JOINT_NAMES, ARM_RESET))
    return np.array([positions[name] for name in EXPECTED_JOINT_NAMES])


def tube_tip_height(axis_z, clearance=.005):
    axis_z = float(np.clip(axis_z, -1., 1.))
    return max(1.1 * axis_z, 0.) + .028 * math.sqrt(max(1. - axis_z**2, 0.)) + clearance


def final_body_target(work_distance=.8):
    """Retired nominal-height target, only for explicit legacy reproduction."""
    quaternion = quaternion_from_euler_xyz(0., PITCH, math.pi)
    axis_z = (quaternion_matrix(quaternion) @ np.array((1., 0., 0.)))[2]
    return np.r_[-work_distance, 0., tube_tip_height(axis_z) - BASE_HEIGHT, quaternion]


def straight_world_seam(base_position, base_quaternion, speed, duration):
    """192-sample zero-curvature subset of the training's world path."""
    rotation = quaternion_matrix(base_quaternion)
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    distances = np.linspace(0., speed * max(31., duration) + 1.2, 192)
    points = np.zeros((192, 3))
    points[:, :2] = (np.asarray(base_position)[:2]
                    + distances[:, None] * np.array([math.cos(yaw), math.sin(yaw)]))
    return points


def world_seam_command(base_position, base_quaternion, start_position_w,
                       start_quaternion_w, seam_points_w, time_s, work_distance=.8):
    """VD10SafeSeamCommand at ground_blend=1, including the actual world rim.

    The initial TCP is anchored in WORLD, not reattached to a sagging base.
    Body orientation remains the training's interpolated relative quaternion;
    its world axis determines the finite cylinder's required centre height.
    """
    base = np.asarray(base_position, dtype=np.float64)
    rotation = quaternion_matrix(base_quaternion)
    fraction = startup_fraction(time_s, RAMP_S)
    start_quat_b = quaternion_multiply(quaternion_conjugate(base_quaternion), start_quaternion_w)
    target_quat_b = normalized_lerp_quaternion(start_quat_b,
        quaternion_from_euler_xyz(0., PITCH, math.pi), fraction)
    axis_w = rotation @ quaternion_matrix(target_quat_b)[:, 0]
    seam_w = sample_polyline_behind_position(seam_points_w, base, work_distance)
    seam_w[2] += tube_tip_height(axis_w[2])
    target_w = np.asarray(start_position_w) + fraction * (seam_w - np.asarray(start_position_w))
    pose_b = np.r_[rotation.T @ (target_w - base), target_quat_b]
    return pose_b, target_w, fraction


def ideal_front_command(base_position, base_quaternion, seam_points_w, speed):
    """Training's ideal pinhole/ray-plane teacher; no rendered image is read."""
    base = np.asarray(base_position, dtype=np.float64)
    rotation = quaternion_matrix(base_quaternion)
    camera_position = base + rotation @ np.asarray(COMMAND_CAMERA['position_b'])
    camera_rotation = rotation @ quaternion_matrix(quaternion_from_euler_xyz(*COMMAND_CAMERA['rpy_b']))
    points_c = (np.asarray(seam_points_w) - camera_position) @ camera_rotation
    depth = points_c[:, 0]
    fx, fy, cx, cy = COMMAND_CAMERA['intrinsics']
    near, far = COMMAND_CAMERA['depth_range']
    width, height = COMMAND_CAMERA['image_size']
    safe_depth = np.maximum(depth, near)
    pixels = np.column_stack((cx - fx * points_c[:, 1] / safe_depth,
                              cy - fy * points_c[:, 2] / safe_depth))
    rays = np.column_stack((np.ones(len(depth)), -(pixels[:, 0] - cx) / fx,
                            -(pixels[:, 1] - cy) / fy))
    denominator = rays @ (camera_rotation.T @ np.array((0., 0., 1.)))
    valid_ray = denominator < -1e-6
    distance = -camera_position[2] / np.where(valid_ray, denominator, -1.)
    visible = ((depth >= near) & (depth <= far) & (pixels[:, 0] >= 0.)
               & (pixels[:, 0] < width) & (pixels[:, 1] >= 0.) & (pixels[:, 1] < height)
               & valid_ray & (distance > 0.) & np.isfinite(points_c).all(axis=1))
    recovered_w = camera_position + np.where(visible[:, None], rays * distance[:, None], 0.) @ camera_rotation.T
    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    c, s = math.cos(yaw), math.sin(yaw)
    points_h = (recovered_w[:, :2] - base[:2]) @ np.array(((c, -s), (s, c)))
    lateral, heading, curvature, path_visible = _tracking_errors(
        points_h, visible, COMMAND_CAMERA['lookahead_m'])
    yaw_rate = (float(np.clip(COMMAND_CAMERA['heading_gain'] * heading + speed * curvature,
                             -COMMAND_CAMERA['yaw_rate_limit'], COMMAND_CAMERA['yaw_rate_limit']))
                if path_visible and speed > 0. else 0.)
    return np.array((speed, 0., yaw_rate)), dict(ideal_visible_points=int(visible.sum()),
        ideal_path_visible=path_visible, lateral_error_m=lateral, heading_error_rad=heading,
        curvature_m_inv=curvature, rendered_camera=False, truth_generated=True)


def geometry_is_invalid(result):
    """Only a confirmed geometric intersection trips this guard, never fake force."""
    status, confirmed = result.get('status'), result.get('confirmed_invalid')
    if status not in ('safe', 'invalid', 'uncertain') or not isinstance(confirmed, bool):
        raise ValueError('Malformed geometry audit result')
    if (status == 'invalid') != confirmed:
        raise ValueError('Contradictory geometry audit result')
    return confirmed


def load_geometry_audit():
    spec = importlib.util.spec_from_file_location('vd11_geometry_audit', GEOMETRY_MODULE)
    if spec is None or spec.loader is None:
        raise RuntimeError('VD11 geometry audit is unavailable; refusing unguarded replay')
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module.GunRobotGeometryAudit(VD10_URDF_PATH)


def actor_observation(base_command, tool_pose, sensor, default, previous):
    """Keep the direct actor state boundary at named encoder/IMU fields only.

    In world-seam mode the *commands* are truth-generated, exactly as in
    training; this function does not append a base world pose, contact or TCP.
    """
    from vd1_control import build_actor_observation
    return build_actor_observation(base_command, tool_pose, sensor['gyro_b'], sensor['gravity_b'],
                                   sensor['q'], sensor['qd'], default, previous)


def validate_checkpoint(checkpoint, *, compare_vd10=False):
    experiment = checkpoint.get('training_experiment_name')
    if experiment not in EXPERIMENTS:
        raise ValueError('Requires the isolated VD10 or VD11 hollow-gun experiment')
    if experiment == 'b2_z1_vd10_hollow_gun_amp' and not compare_vd10:
        raise ValueError('VD10 requires explicit --compare-vd10; default is VD11 only')
    iteration = checkpoint.get('iter')
    if isinstance(iteration, bool) or iteration != EXPERIMENTS[experiment]:
        raise ValueError('This review accepts VD10 model_5998 or VD11 model_5999 only')
    if 'obs_norm_state_dict' in checkpoint:
        raise ValueError('Actor observation normalization is not supported')
    state = checkpoint.get('vd5_curriculum_state')
    if not isinstance(state, dict) or state.get('version') != 1:
        raise ValueError('Missing supported VD5/VD10 curriculum state')
    if state.get('startup_profile') != PROFILE:
        raise ValueError('Requires the exact safe-start profile version 2 and matching nominal joints')
    if (state.get('ground_blend') != 1. or state.get('ground_transition_steps') != 20_000
            or state.get('seam_trailing_distance_m') != .8):
        raise ValueError('Requires a completed, matching 0.80 m ground-seam curriculum')
    return experiment, int(iteration)


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n', encoding='utf-8')


def parse(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', required=True, type=Path)
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--speed', type=float, default=.2)
    parser.add_argument('--work-distance', type=float, default=.8)
    parser.add_argument('--command-mode', choices=COMMAND_MODES, default='world-seam',
                        help='Default: training-equivalent straight world seam; legacy is not task validation')
    parser.add_argument('--compare-vd10', action='store_true', help='Explicitly allow VD10 model_5998 comparison')
    parser.add_argument('--output-root', type=Path, default=ROOT / 'outputs/VD11_MUJOCO_DIAGNOSTIC')
    parser.add_argument('--viewer', action='store_true', help='Optional local diagnostic viewer; no robot IO')
    parser.add_argument('--save-frames', nargs='*', type=float, default=None,
                        help='Save honest state PNGs at these seconds; with no values use 0,.5,1,2,5,10,20,30')
    args = parser.parse_args(argv)
    if not (math.isfinite(args.duration) and 5. < args.duration <= 60.
            and math.isfinite(args.speed) and 0. <= args.speed <= .4
            and math.isfinite(args.work_distance) and .6 <= args.work_distance <= .9):
        parser.error('duration must be >5 and <=60 s, speed [0,.4], work-distance [.6,.9]')
    if args.save_frames is not None:
        args.save_frames = args.save_frames or [0., .5, 1., 2., 5., 10., 20., 30.]
        if any(not math.isfinite(t) or t < 0. for t in args.save_frames):
            parser.error('frame times must be finite and nonnegative')
        args.save_frames = sorted(set(t for t in args.save_frames if t <= args.duration))
    return args


def load_actor(path, *, compare_vd10=False):
    import torch
    checkpoint = torch.load(path, map_location='cpu', weights_only=False)
    experiment, iteration = validate_checkpoint(checkpoint, compare_vd10=compare_vd10)
    actor = torch.nn.Sequential(torch.nn.Linear(70, 512), torch.nn.ELU(),
        torch.nn.Linear(512, 256), torch.nn.ELU(), torch.nn.Linear(256, 128),
        torch.nn.ELU(), torch.nn.Linear(128, 18)).eval()
    state = {key[len('actor.'):]: value for key, value in checkpoint['model_state_dict'].items()
             if key.startswith('actor.')}
    actor.load_state_dict(state, strict=True)
    if not all(torch.isfinite(value).all() for value in state.values()):
        raise ValueError('Actor contains nonfinite parameters')
    with torch.inference_mode():
        output = actor(torch.zeros(1, 70))
    if tuple(output.shape) != (1, 18) or not torch.isfinite(output).all():
        raise ValueError('Expected a finite unnormalized 70-to-18 actor')
    return actor, experiment, iteration


def termination_causes(body_forces, tool_history, illegal_ids, tilt):
    """Real engine net forces; twenty 5 ms samples reproduce the 100 ms window."""
    history = np.asarray(tool_history)
    sustained = len(history) == 20 and bool(np.all(history > 20.))
    return dict(tool_support=sustained or bool(np.any(history > 150.)),
                base_contact=bool(np.any(np.linalg.norm(body_forces[illegal_ids], axis=1) > 1.)),
                bad_orientation=bool(tilt > .5))


def configure_camera(camera, base_position):
    """A three-quarter side view including the entire long pipe and all legs."""
    camera.lookat[:] = np.asarray(base_position) + np.array((-.28, 0., .05))
    camera.distance, camera.azimuth, camera.elevation = 3.2, 120., -18.


def main(argv=None):
    args = parse(argv)
    os.environ.setdefault('MUJOCO_GL', 'egl' if not args.viewer else 'glfw')
    import mujoco
    import torch
    from vd1_mujoco_contract import VD1MujocoAdapter
    from vd1_mujoco_scene import build_scene, validate_vd10_model
    from vd1_mujoco_velocity_impulse import VelocityImpulseConstraints
    from vd5_visual_control import encoder_tcp_pose

    torch.set_num_threads(1)
    path = args.checkpoint.expanduser().resolve(strict=True)
    actor, experiment, iteration = load_actor(path, compare_vd10=args.compare_vd10)
    if sha256(VD10_URDF_PATH) != VD10_URDF_SHA256:
        raise ValueError('New-gun URDF SHA mismatch')
    args.output_root.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='mujoco_' + time.strftime('%Y%m%d_%H%M%S') + '_',
                                  dir=str(args.output_root.resolve())))
    cache = Path(tempfile.mkdtemp(prefix='vd11_gait_', dir='/dev/shm'))
    scene = build_scene(0., cache, timestep=DT, tool_markers=False,
                        road_surface='tape', robot_version='vd10')
    model = mujoco.MjModel.from_xml_path(str(scene))
    save(output / 'asset_audit.json', validate_vd10_model(model))
    geometry_audit = load_geometry_audit()
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('vd1_default').id)
    adapter = VD1MujocoAdapter(model, physx_implicit_position=True)
    default = default_position()
    # Per-instance change only. The shared adapter and VD5 defaults remain intact.
    adapter.default_q18 = default.copy()
    qpos_ids, dofs = model.jnt_qposadr[adapter.joint_ids18], model.jnt_dofadr[adapter.joint_ids18]
    data.qpos[2] = BASE_HEIGHT
    data.qpos[qpos_ids] = default
    data.qvel[:] = 0.
    adapter.apply_targets(data, default)
    mujoco.mj_forward(model, data)
    constraint = VelocityImpulseConstraints(model, adapter.joint_ids18, adapter.velocity_limit18)
    feet = [model.body(name).id for name in FEET]
    base, tcp, tool = model.body('base_link').id, model.site('tool_tcp').id, model.body('link06').id
    illegal_ids = [body for body in range(1, model.nbody)
                   if re.fullmatch(r'base_link|.*_hip|.*_thigh|.*_calf|link0[0-5]',
                                   mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body) or '')]
    if not illegal_ids:
        raise ValueError('No illegal-contact bodies were resolved')
    initial_position, initial_quaternion = encoder_tcp_pose(
        default[[EXPECTED_JOINT_NAMES.index(name) for name in ARM_JOINT_NAMES]],
        VD10_URDF_PATH, VD10_TCP_POSITION, VD10_TCP_ROTATION)
    rotation = data.xmat[base].reshape(3, 3)
    np.testing.assert_allclose(rotation.T @ (data.site_xpos[tcp] - data.xpos[base]),
                               initial_position, atol=1e-6)
    final = final_body_target(args.work_distance)
    start_position_w = data.site_xpos[tcp].copy()
    start_quaternion_w = quaternion_multiply(data.xquat[base].copy(), initial_quaternion)
    seam_points_w = straight_world_seam(data.xpos[base], data.xquat[base], args.speed, args.duration)
    initial_geometry = geometry_audit.audit(EXPECTED_JOINT_NAMES, data.qpos[qpos_ids].copy())
    initial_invalid = geometry_is_invalid(initial_geometry)
    geometry_minimum = initial_geometry['minimum_clearance_lower_bound_m']
    geometry_counts = dict(safe=0, uncertain=0, invalid=0)
    geometry_counts[initial_geometry['status']] += 1
    geometry_worst = initial_geometry
    schema = dict(version=2, engine='mujoco', joint_order=list(EXPECTED_JOINT_NAMES), dt_s=DT,
        state_phase='step-end positions/velocities AFTER split speed projection and mj_forward',
        force_phase='native mj_step beginning-state solve BEFORE speed projection and mj_forward',
        contact_phase='same native beginning-state solve as drive/constraint force, not step-end response',
        drive_effort_source='MjData.actuator_force indexed by named actuator; engine force, NEVER PD estimate',
        drive_effort_note='Implicitfast force buffer is not a separately measured step-average delivered torque',
        native_constraint_source='MjData.qfrc_constraint and efc buffers before split speed impulse',
        speed_impulse_source='VelocityImpulseConstraints.last.generalized_impulse; Nm*s, not drive torque',
        effort_limit18_nm=adapter.effort_limit18.tolist(), velocity_limit18_radps=adapter.velocity_limit18.tolist(),
        source_gains=[160., 5.], numerical_arm_kd=adapter.arm_discrete_kd18.tolist(),
        joint_position_limits_rad=model.jnt_range[adapter.joint_ids18].tolist())
    save(output / 'joint_diagnostic_schema.json', schema)
    save(output / 'manifest.json', dict(checkpoint=str(path), checkpoint_sha256=sha256(path),
        checkpoint_iteration=iteration, training_experiment=experiment, engine='mujoco',
        urdf=str(VD10_URDF_PATH), urdf_sha256=VD10_URDF_SHA256, scene=str(scene),
        default_q18_rad=default.tolist(), tcp_position_link_m=list(VD10_TCP_POSITION),
        tcp_rotation_link=VD10_TCP_ROTATION.tolist(), physical_gun_axis='TCP +X',
        command_mode=args.command_mode,
        initial_body_tool_pose=np.r_[initial_position, initial_quaternion].tolist(),
        initial_world_tool_pose=np.r_[start_position_w, start_quaternion_w].tolist(),
        legacy_fixed_body_tool_target=final.tolist(), legacy_target_horizon_base_height_m=BASE_HEIGHT,
        target_note=('Training-equivalent ground_blend=1 straight WORLD seam: measured initial world TCP, '
            '2s world-position startup, initial world quaternion transformed into current base before nlerp, '
            'finite-cylinder rim height recomputed from current target world axis. Truth-generated diagnostic, NOT visual deployment.'
            if args.command_mode == 'world-seam' else
            'EXPLICIT LEGACY: fixed BODY setpoint; nominal-height rim calculation can put the WORLD target underground. '
            'Not training-equivalent and not deployable; geometry guard still applies.'),
        world_seam_points_m=seam_points_w.tolist(), ideal_command_camera=COMMAND_CAMERA,
        trace_target_note='env_tcp_target_world_m holds the actual command-generation WORLD target at 50Hz. The continuous projection of the held BODY command is logged separately; neither is an extra actor observation.',
        physics_dt_s=DT, control_dt_s=ACTOR_DT, ramp_s=RAMP_S, speed=args.speed,
        simulation_only=True, robot_io=False, exported_actor=False,
        commands_generated_from_simulator_truth=args.command_mode == 'world-seam',
        world_truth_uses=['diagnostic command generation', 'evaluation'] if args.command_mode == 'world-seam' else ['evaluation'],
        direct_world_state_actor_observations=False, extra_actor_observations=[],
        actor_observation_size=70, actor_observation_boundary='generated base/TCP commands + encoder/IMU + previous action',
        observation_noise=False, domain_randomization=False,
        automatic_resets=False, source_startup_profile=PROFILE, force_response_phase=schema,
        geometry_guard=dict(metadata=geometry_audit.metadata, period_s=DT,
            source='independent URDF finite-cylinder/robot-primitive FK geometry from actual joint angles',
            not_contact_force=True, stop_on='confirmed_invalid', uncertain_is_not_a_safety_pass=True),
        viewer_real_time=bool(args.viewer), saved_frame_request_s=args.save_frames,
        termination_check_period_s=DT,
        termination_note='Same thresholds as training, checked every 5ms rather than 20ms; stop single episode immediately. Native contact-force phase precedes step-end pose.',
        termination_thresholds=dict(tool_sustained_n=20., tool_duration_s=.1, tool_peak_n=150.,
                                    illegal_net_force_n=1., tilt_rad=.5)))
    print('VD11 MuJoCo diagnostic output: ' + str(output), flush=True)
    previous = np.zeros(18, dtype=np.float32)
    target = default.copy()
    command_pose = np.r_[initial_position, initial_quaternion]
    target_world_zoh = data.xpos[base] + rotation @ command_pose[:3]
    trace, tool_history = [], deque(maxlen=20)
    counts = dict(tool_support=0, base_contact=0, bad_orientation=0, time_out=0,
                  invalid_geometry=int(initial_invalid))
    reason = 'invalid_geometry' if initial_invalid else 'duration_complete'
    terminal = (dict(physics_step=0, time_s=0., causes=dict(invalid_geometry=True),
                     geometric_evidence=initial_geometry) if initial_invalid else None)
    actor_steps = 0
    viewer = None
    if args.save_frames is not None:
        # Graphics buffer size only; no physical property, collision mask or
        # robot pose is changed to obtain the screenshots.
        model.vis.global_.offwidth = max(1280, model.vis.global_.offwidth)
        model.vis.global_.offheight = max(720, model.vis.global_.offheight)
    if args.viewer:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)
        with viewer.lock():
            configure_camera(viewer.cam, data.xpos[base])
        viewer.sync()
    renderer = None
    frame_times = deque(args.save_frames or [])
    frame_records = []
    frame_camera = mujoco.MjvCamera()
    if args.save_frames is not None:
        from PIL import Image
        renderer = mujoco.Renderer(model, height=720, width=1280)
        (output / 'state_frames').mkdir()

    def save_frame(time_s, label):
        if renderer is None:
            return
        configure_camera(frame_camera, data.xpos[base])
        renderer.update_scene(data, camera=frame_camera)
        filename = 'state_frames/%.3fs_%s.png' % (time_s, label)
        Image.fromarray(renderer.render()).save(output / filename)
        frame_records.append(dict(time_s=time_s, label=label, image=filename,
                                  source='unmodified simulator state; no repositioned links'))

    if frame_times and frame_times[0] == 0.:
        frame_times.popleft()
        save_frame(0., 'initial')
    if initial_invalid:
        print('INVALID initial geometry: confirmed intersection; no physics steps will run.', flush=True)
        save_frame(0., 'invalid_geometry')
    wall_start = time.perf_counter()
    with gzip.open(output / 'physics_truth.jsonl.gz', 'xt', compresslevel=3) as physics_file, \
         gzip.open(output / 'joint_response.jsonl.gz', 'xt', compresslevel=3) as joint_file, \
         gzip.open(output / 'actor_trace.jsonl.gz', 'xt', compresslevel=3) as actor_file, \
         gzip.open(output / 'geometry_audit.jsonl.gz', 'xt', compresslevel=3) as geometry_file:
        geometry_file.write(json.dumps(dict(time_s=0., physics_step=0, **initial_geometry), allow_nan=False) + '\n')
        for tick in range(0 if initial_invalid else round(args.duration / DT)):
            if tick % DECIMATION == 0:
                sensor = adapter.read_proprioception(data)
                blend = startup_fraction(tick * DT, RAMP_S)
                if args.command_mode == 'world-seam':
                    command_pose, target_world_zoh, blend = world_seam_command(
                        data.xpos[base], data.xquat[base], start_position_w, start_quaternion_w,
                        seam_points_w, tick * DT, args.work_distance)
                    command, guidance = ideal_front_command(
                        data.xpos[base], data.xquat[base], seam_points_w, args.speed)
                    command *= blend
                else:
                    command_pose = np.r_[initial_position + blend * (final[:3] - initial_position),
                        normalized_lerp_quaternion(initial_quaternion, final[3:], blend)]
                    target_world_zoh = (data.xpos[base]
                        + data.xmat[base].reshape(3, 3) @ command_pose[:3])
                    command = np.array([args.speed * blend, 0., 0.])
                    guidance = dict(legacy_fixed_body=True, truth_generated=False)
                obs = actor_observation(command, command_pose, sensor, default, previous)
                with torch.inference_mode():
                    action = actor(torch.from_numpy(obs).unsqueeze(0)).numpy()[0]
                if action.shape != (18,) or not np.isfinite(action).all():
                    raise ValueError('Nonfinite or wrong-shaped action')
                target = default + .25 * action
                actor_file.write(json.dumps(dict(time_s=tick * DT, observation=obs.tolist(),
                    action=action.tolist(), joint_target18_rad=target.tolist(),
                    tool_pose_b=command_pose.tolist(), tool_target_world_command=target_world_zoh.tolist(),
                    base_command_b=command.tolist(), command_guidance=guidance)) + '\n')
                previous = action.copy()
                actor_steps += 1
            before_q, before_qd = data.qpos[qpos_ids].copy(), data.qvel[dofs].copy()
            adapter.apply_targets(data, target)
            native = {}

            def snapshot(native_data):
                body_forces = np.zeros((model.nbody, 3))
                for number in range(native_data.ncon):
                    contact = native_data.contact[number]
                    force = np.zeros(6)
                    mujoco.mj_contactForce(model, native_data, number, force)
                    force_w = contact.frame.reshape(3, 3).T @ force[:3]
                    first, second = model.geom_bodyid[[contact.geom1, contact.geom2]]
                    body_forces[first] -= force_w
                    body_forces[second] += force_w
                native.update(body_contact_force_world_n=body_forces.tolist(),
                    drive_effort18_nm=native_data.actuator_force[adapter.actuator_ids18].copy().tolist(),
                    joint_native_constraint_effort18_nm=native_data.qfrc_constraint[dofs].copy().tolist(),
                    velocity_before_speed_projection18_radps=native_data.qvel[dofs].copy().tolist(),
                    native_constraint_type=native_data.efc_type[:native_data.nefc].copy().tolist(),
                    native_constraint_id=native_data.efc_id[:native_data.nefc].copy().tolist(),
                    native_constraint_force=native_data.efc_force[:native_data.nefc].copy().tolist(),
                    native_constraint_state=native_data.efc_state[:native_data.nefc].copy().tolist())

            constraint.step(data, evaluation_observer=snapshot)
            impulse = np.asarray(constraint.last['generalized_impulse'])[dofs]
            after_qd = data.qvel[dofs].copy()
            joint_file.write(json.dumps(dict(step_start_s=tick * DT, time_s=(tick + 1) * DT,
                actor_target_time_s=(tick // DECIMATION) * ACTOR_DT,
                joint_target18_rad=target.tolist(), joint_position_before18_rad=before_q.tolist(),
                joint_velocity_before18_radps=before_qd.tolist(),
                joint_position18_rad=data.qpos[qpos_ids].tolist(), joint_velocity18_radps=after_qd.tolist(),
                speed_constraint_impulse18_nms=impulse.tolist(), speed_constraint_active18=(np.abs(impulse) > 1e-10).tolist(),
                speed_constraint_delta_velocity18_radps=(after_qd - np.asarray(native['velocity_before_speed_projection18_radps'])).tolist(),
                drive_effort_at_limit18=(np.abs(native['drive_effort18_nm']) >= adapter.effort_limit18 - 1e-5).tolist(),
                speed_near_limit18=(np.abs(after_qd) >= .99 * adapter.velocity_limit18).tolist(),
                evaluation_only=True, actor_inputs_overwritten=False, **native), allow_nan=False) + '\n')
            rotation = data.xmat[base].reshape(3, 3)
            actual = data.site_xpos[tcp].copy()
            axis = data.site_xmat[tcp].reshape(3, 3)[:, 0].copy()
            target_world = data.xpos[base] + rotation @ command_pose[:3]
            target_axis = rotation @ quaternion_matrix(command_pose[3:])[:, 0]
            forces = np.asarray(native['body_contact_force_world_n'])

            def link_velocity(body):
                jacobian = np.zeros((3, model.nv))
                mujoco.mj_jac(model, data, jacobian, None, data.xpos[body], body)
                return jacobian @ data.qvel

            foot_velocities = np.array([link_velocity(body) for body in feet])
            tilt = math.acos(float(np.clip(rotation[2, 2], -1., 1.)))
            geometric_result = geometry_audit.audit(EXPECTED_JOINT_NAMES, data.qpos[qpos_ids].copy())
            invalid_geometry = geometry_is_invalid(geometric_result)
            geometry_counts[geometric_result['status']] += 1
            clearance_bound = geometric_result['minimum_clearance_lower_bound_m']
            if clearance_bound is not None and (geometry_minimum is None or clearance_bound < geometry_minimum):
                geometry_minimum = clearance_bound
                geometry_worst = geometric_result
            geometry_file.write(json.dumps(dict(time_s=(tick + 1) * DT, physics_step=tick + 1,
                                                **geometric_result), allow_nan=False) + '\n')
            row = dict(time_s=(tick + 1) * DT, evaluation_only=True,
                base_position_w=data.xpos[base].tolist(), base_quat_wxyz=data.xquat[base].tolist(),
                base_velocity_b=(rotation.T @ link_velocity(base)).tolist(), tilt_deg=math.degrees(tilt),
                tcp_position_w=actual.tolist(), tcp_position_b=(rotation.T @ (actual-data.xpos[base])).tolist(),
                tool_target_b=command_pose[:3].tolist(), tcp_target_world_evaluation_only=target_world.tolist(),
                tcp_target_world_50hz_zoh_evaluation_only=target_world_zoh.tolist(),
                tcp_target_world_command_50hz_zoh=target_world_zoh.tolist(),
                tube_axis_world=axis.tolist(), tube_surface_min_z_m=float(actual[2] - tube_tip_height(axis[2], 0.)),
                geometry_status=geometric_result['status'], confirmed_invalid_geometry=invalid_geometry,
                geometry_clearance_lower_bound_m=geometric_result['minimum_clearance_lower_bound_m'],
                foot_position_w=data.xpos[feet].tolist(), foot_velocity_w=foot_velocities.tolist(),
                foot_force_w=forces[feet].tolist(), tool_force_w=forces[tool].tolist())
            physics_file.write(json.dumps(row, allow_nan=False) + '\n')
            trace.append(dict(physics_step=tick + 1, env_episode_actor_step=[tick // DECIMATION],
                env_tcp_world_m=[actual.tolist()], env_tcp_target_world_m=[target_world_zoh.tolist()],
                env_tcp_target_world_held_body_projection_m=[target_world.tolist()],
                env_base_quat_wxyz=[data.xquat[base].tolist()],
                env_gun_axis_error_rad=[math.acos(float(np.clip(axis @ target_axis, -1., 1.)))],
                env_foot_force_world_n=[forces[feet].tolist()],
                env_foot_velocity_world_m_s=[foot_velocities.tolist()],
                env_foot_height_m=[(data.xpos[feet, 2] - .032).tolist()],
                env_tube_surface_min_z_m=[row['tube_surface_min_z_m']],
                env_tool_up_force_n=[float(forces[tool, 2])]))
            tool_history.append(max(float(forces[tool, 2]), 0.))
            causes = termination_causes(forces, tool_history, illegal_ids, tilt)
            causes['invalid_geometry'] = invalid_geometry
            if frame_times and frame_times[0] <= (tick + 1) * DT + 1e-10:
                while frame_times and frame_times[0] <= (tick + 1) * DT + 1e-10:
                    frame_times.popleft()
                save_frame((tick + 1) * DT, 'requested')
            if any(causes.values()):
                for cause, active in causes.items():
                    counts[cause] += int(active)
                reason = '+'.join(cause for cause, active in causes.items() if active)
                terminal = dict(physics_step=tick + 1, time_s=(tick + 1) * DT, causes=causes,
                                geometric_evidence=geometric_result if invalid_geometry else None)
                save_frame((tick + 1) * DT, reason)
                if viewer:
                    viewer.sync()
                print('Diagnostic stopped: ' + reason + ' (geometry is not a contact-force measurement).', flush=True)
                break
            if viewer:
                if not viewer.is_running():
                    reason = 'viewer_closed'
                    break
                with viewer.lock():
                    viewer.cam.lookat[:] = data.xpos[base] + np.array((-.28, 0., .05))
                viewer.sync()
                remaining = wall_start + (tick + 1) * DT - time.perf_counter()
                if remaining > 0.:
                    time.sleep(min(remaining, DT))
            if (tick + 1) % 2000 == 0:
                print('VD11 diagnostic %.1f s tilt %.2f deg' % ((tick + 1) * DT, math.degrees(tilt)), flush=True)
    if viewer:
        viewer.close()
    if renderer is not None:
        if reason == 'duration_complete' or reason == 'viewer_closed':
            save_frame(len(trace) * DT, reason)
        renderer.close()
        save(output / 'state_frames.json', frame_records)
    save(output / 'velocity_constraint.json', constraint.summary())
    geometry_status = ('invalid' if geometry_counts['invalid'] else
                       'uncertain' if geometry_counts['uncertain'] else 'safe')
    geometry_summary = dict(status=geometry_status, confirmed_invalid=geometry_status == 'invalid',
        counts=geometry_counts, minimum_clearance_lower_bound_m=geometry_minimum,
        initial=initial_geometry, worst=geometry_worst, audit_metadata=geometry_audit.metadata,
        source='actual joint angles -> independent URDF FK/finite-cylinder geometry',
        evaluation_only=True, not_contact_force=True, never_fed_to_actor=True,
        uncertain_is_not_a_safety_pass=True)
    save(output / 'geometry_summary.json', geometry_summary)
    metadata = dict(num_envs=1, actor_steps=actor_steps, control_dt_s=ACTOR_DT,
        physics_dt_s=DT, decimation=DECIMATION, foot_names=list(FEET), trace_schema_version=2,
        action_mode='deterministic_actor_truth_generated_world_seam' if args.command_mode == 'world-seam' else 'deterministic_actor_legacy_fixed_body_diagnostic',
        command_mode=args.command_mode, checkpoint=str(path),
        frozen_ground_blend=1., total_resets=0, reset_events=[], termination_events=counts,
        terminal_event=terminal, reason=reason, single_episode=True,
        target_source='truth-generated straight world seam' if args.command_mode == 'world-seam' else 'explicit legacy fixed body setpoint',
        rendered_camera_commands=False, direct_world_state_actor_observations=False,
        commands_generated_from_simulator_truth=args.command_mode == 'world-seam',
        geometry_validity=geometry_status,
        trace_target_note='env_tcp_target_world_m is the 50Hz world command target; continuous held-body projection is separate. No world state appended to 70-value actor observations.',
        force_response_phase=schema,
        termination_events_note='Single terminal episode, then stop. No reset and no timeout before requested duration; physics and native force phases differ as documented.')
    # Historical filenames intentionally preserve the existing schema-2 analyzer interface.
    save(output / 'vd10_200hz_trace.json', trace)
    save(output / 'vd10_policy_probe.json', metadata)
    save(output / 'summary.json', dict(reason=reason, physics_steps=len(trace), actor_steps=actor_steps,
        duration_s=len(trace) * DT, termination_events=counts, terminal_event=terminal,
        command_mode=args.command_mode, geometry_validity=geometry_status,
        valid_for_motion_quality_comparison=geometry_status == 'safe' and args.command_mode == 'world-seam',
        deployment_validation=False,
        artifact_note='An invalid/uncertain geometry run is NOT a passed deployment or motion-quality comparison. Preserve its evidence; do not concatenate episodes.'))
    print(json.dumps(dict(output=str(output), reason=reason, duration_s=len(trace) * DT), indent=2), flush=True)
    return 2 if geometry_status == 'invalid' else 0


if __name__ == '__main__':
    raise SystemExit(main())
