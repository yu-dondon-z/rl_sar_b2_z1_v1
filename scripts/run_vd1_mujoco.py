#!/usr/bin/env python3
"""VD1 MuJoCo runner: camera-only external perception, IMU and encoders.

The independently labelled EvaluationRecorder is an output sink.  No method
returns its world-pose/contact/path measurements to the controller or actor.
This entry point imports no old FSM, SDK, DDS, real-robot or IK controller.
"""
from __future__ import annotations

import argparse
import gzip
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

from vd3_mujoco_policy import VD3_POLICY_SHA256

TRAINING_ROOT = Path('/home/dong/b2_z1')
sys.path.insert(0, str(TRAINING_ROOT / 'scripts/vision'))
POLICY_SHA256 = '86be6c2d1c7a0e56fe4bd93c2d62622469c030b0cdc6d10db7af001eae3f889a'


def arguments():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--policy', type=Path, required=True)
    p.add_argument('--robot-version', choices=('vd1', 'vd3'), default='vd1')
    p.add_argument('--output-dir', type=Path, required=True)
    p.add_argument('--vision-result', type=Path, required=True)
    p.add_argument('--curvature-max', type=float, default=.05)
    p.add_argument('--speed', type=float, default=.4)
    p.add_argument('--visual-controller', choices=('original', 'guarded'), default='original')
    p.add_argument('--front-only-guidance', action='store_true')
    p.add_argument('--front-path-estimation', action='store_true')
    p.add_argument('--front-nominal-height-m', type=float, default=.55)
    p.add_argument('--road-contact', choices=('prisms', 'heightfield'), default='prisms')
    p.add_argument('--road-surface', choices=('crack', 'tape'), default='crack')
    p.add_argument('--gun-mass-mode', choices=('original', 'zero'), default='original')
    p.add_argument('--max-steps', type=int, default=0)
    p.add_argument('--headless', action='store_true')
    p.add_argument('--static-only', action='store_true')
    p.add_argument('--velocity-constraint', choices=('off', 'implicit_brake', 'native_velocity', 'impulse'), default='off')
    p.add_argument('--physics-dt', type=float, choices=(.001, .005), default=.001)
    p.add_argument('--physx-implicit-position', action='store_true')
    p.add_argument('--screenshot', type=Path)
    p.add_argument('--cache-dir', type=Path, default=Path('/dev/shm/vd1_mujoco'))
    a = p.parse_args()
    if not 0 <= a.curvature_max <= .50 or a.max_steps < 0 or not 0 < a.speed <= 1:
        p.error('invalid curvature, max-steps or speed')
    if a.front_path_estimation and a.visual_controller != 'guarded':
        p.error('front path estimation requires the guarded visual controller')
    return a


def save_json(path, value):
    tmp = path.with_suffix(path.suffix + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False), encoding='utf-8')
    os.replace(tmp, path)


def load_packet(path, publisher):
    try:
        packet = json.loads(path.read_text(encoding='utf-8'))
        accepted, reason = publisher.validate_packet(packet)
        return (packet, '') if accepted else (None, reason)
    except (OSError, ValueError) as error:
        return None, str(error)


def packet_diagnostics(packet, now):
    """Record the actual input packet, distinct from the last accepted frame.

    This is an output-only trace helper: it never supplies controller inputs.
    """
    if not isinstance(packet, dict):
        return dict(received=False)
    result = dict(received=True, paired_valid=packet.get('valid'), reason=packet.get('reason'))
    for name in ('front', 'rear'):
        stream = packet.get(name, {})
        if not isinstance(stream, dict):
            stream = {}
        capture = stream.get('capture_timestamp')
        points = stream.get('points_b', [])
        if not isinstance(points, list):
            points = []
        detail = dict(valid=stream.get('valid'), reason=stream.get('reason'),
                      sequence=stream.get('sequence'), confidence=stream.get('confidence'),
                      capture_age_s=now-capture if isinstance(capture, (int, float)) else None,
                      capture_sim_time_s=stream.get('capture_sim_time_s'), point_count=len(points))
        if points and all(isinstance(p, list) and len(p) >= 2
                          and all(isinstance(v, (int, float)) and math.isfinite(v) for v in p[:2])
                          for p in points):
            xs = [point[0] for point in points]
            ys = [point[1] for point in points]
            detail.update(x_range_b_m=[min(xs), max(xs)], y_range_b_m=[min(ys), max(ys)])
            if name == 'rear':
                detail['covers_working_x'] = min(xs) <= -1.02 <= max(xs)
        result[name] = detail
    return result


class EvaluationRecorder:
    """Independent simulation-truth logger, never a controller dependency."""

    def __init__(self, model, output, curvature, road_surface='crack'):
        import mujoco
        import numpy as np
        from vd1_scene import road_stations
        self.model = model
        self.handle = gzip.open(str(output / 'evaluation_truth.jsonl.gz'), 'wt', compresslevel=3)
        self.base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
        self.tcp = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'tool_tcp')
        self.feet = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, leg + '_foot')
                     for leg in ('FL', 'FR', 'RL', 'RR')]
        self.seam = np.asarray(road_stations(curvature), dtype=float)
        self.delta = np.diff(self.seam, axis=0)
        self.length2 = np.maximum(np.sum(self.delta*self.delta, axis=1), 1.e-12)
        self.errors, self.clearances, self.base_errors, self.tilts = [], [], [], []
        self.first_base = self.last_base = None
        self.road_surface = road_surface

    def _distance(self, point):
        import numpy as np
        t = np.clip(np.sum((point-self.seam[:-1])*self.delta, axis=1)/self.length2, 0, 1)
        return float(np.linalg.norm(self.seam[:-1]+t[:, None]*self.delta-point, axis=1).min())

    def write(self, data, step):
        import mujoco
        import numpy as np
        base = data.xpos[self.base].copy()
        tcp = data.site_xpos[self.tcp].copy()
        feet = data.xpos[self.feet].copy()
        distance = [self._distance(p[:2]) for p in feet]
        error = self._distance(tcp[:2])
        base_error = self._distance(base[:2])
        rotation = data.xmat[self.base].reshape(3, 3)
        tilt = float(np.degrees(np.arccos(np.clip(rotation[2, 2], -1., 1.))))
        forces = np.zeros((4, 3))
        contact_force = np.zeros(6)
        for i in range(data.ncon):
            contact = data.contact[i]
            b1, b2 = self.model.geom_bodyid[contact.geom1], self.model.geom_bodyid[contact.geom2]
            if b1 not in self.feet and b2 not in self.feet:
                continue
            mujoco.mj_contactForce(self.model, data, i, contact_force)
            force_w = contact.frame.reshape(3, 3).T @ contact_force[:3]
            for index, body in enumerate(self.feet):
                if b1 == body:
                    forces[index] -= force_w
                if b2 == body:
                    forces[index] += force_w
        row = dict(step=step, sim_time_s=float(data.time), base_position_w=base.tolist(),
                   base_cross_track_m=base_error, tilt_deg=tilt,
                   base_yaw_rad=float(np.arctan2(rotation[1, 0], rotation[0, 0])),
                   tcp_position_w=tcp.tolist(), tcp_projection_error_m=error,
                   foot_center_distance_m=distance, foot_position_w=feet.tolist(),
                   foot_horizontal_edge_clearance_m=(np.asarray(distance)-.042).tolist()
                   if self.road_surface == 'crack' else None,
                   foot_net_force_w=forces.tolist(), note='evaluation only; never returned to control')
        self.handle.write(json.dumps(row, allow_nan=False)+'\n')
        self.errors.append(error)
        self.base_errors.append(base_error)
        self.tilts.append(tilt)
        if self.road_surface == 'crack':
            self.clearances.extend((np.asarray(distance)-.042).tolist())
        if self.first_base is None:
            self.first_base = base
        self.last_base = base

    def close(self):
        self.handle.close()

    def summary(self):
        import numpy as np
        return dict(tcp_projection_error_p95_m=float(np.percentile(self.errors, 95)) if self.errors else None,
                    base_cross_track_p95_m=float(np.percentile(self.base_errors, 95)) if self.base_errors else None,
                    tilt_max_deg=max(self.tilts) if self.tilts else None,
                    tcp_projection_error_max_m=max(self.errors) if self.errors else None,
                    foot_horizontal_edge_clearance_min_m=min(self.clearances) if self.clearances else None,
                    base_displacement_xyz_m=(self.last_base-self.first_base).tolist()
                    if self.first_base is not None else None,
                    note='All steps including startup; use independent evaluator for post-startup metrics')


def initial_encoder_fk(q, names, urdf):
    import torch
    path = TRAINING_ROOT / 'source/b2_z1/b2_z1/tasks/manager_based/b2_z1/mdp/commands/dual_vision_kinematics.py'
    spec = importlib.util.spec_from_file_location('vd1_encoder_fk', str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    fk = module.URDFToolKinematics(urdf, device='cpu')
    arm = torch.tensor([[q[names.index('joint%d' % i)] for i in range(1, 7)]], dtype=torch.float32)
    with torch.inference_mode():
        return fk.position(arm).numpy()[0]


def main():
    a = arguments()
    if a.headless:
        os.environ.setdefault('MUJOCO_GL', 'egl')
    import cv2
    import mujoco
    import numpy as np
    import torch
    from vd1_control import VD1VisualController, build_actor_observation
    from vd1_frames import VD1FramePublisher
    from vd1_mujoco_contract import VD1MujocoAdapter, ACTOR_JOINT_NAMES
    from vd1_mujoco_scene import (build_scene, validate_model, validate_vd3_model,
                                  vd3_camera_calibration, CAMERA_CALIBRATION, VD3_URDF_PATH,
                                  MARKER_CALIBRATION, URDF_PATH, TAPE_VISUAL_HALF_WIDTH_M,
                                  ROAD_HALF_WIDTH_M)
    torch.set_num_threads(1)
    a.output_dir.mkdir(parents=True, exist_ok=True)
    expected_policy_sha = VD3_POLICY_SHA256 if a.robot_version == 'vd3' else POLICY_SHA256
    if hashlib.sha256(a.policy.read_bytes()).hexdigest() != expected_policy_sha:
        raise ValueError(a.robot_version.upper() + ' actor SHA256 mismatch')
    actor = torch.jit.load(str(a.policy), map_location='cpu').eval()
    with torch.inference_mode():
        if tuple(actor(torch.zeros(1, 70)).shape) != (1, 18):
            raise ValueError('VD1 actor must map 70 observations to 18 actions')
    # Unique RAM path: concurrent sessions must never overwrite each other's MJCF.
    cache = a.cache_dir / ('runner_%d' % os.getpid())
    seam_only = a.visual_controller == 'guarded'
    scene = build_scene(a.curvature_max, cache, timestep=a.physics_dt, tool_markers=not seam_only,
                        road_surface=a.road_surface, gun_mass_mode=a.gun_mass_mode,
                        robot_version=a.robot_version)
    road_contact_details = dict(model='independent_convex_prisms')
    if a.road_surface == 'tape':
        road_contact_details = dict(model='flat_plane', ground_z_m=0.,
                                    line_width_m=.02, line_thickness_m=0., line_has_collision=False,
                                    line_rendering='ground_texture', trench_depth_m=0.,
                                    visual_ground_x_range_m=[-95., 95.],
                                    visual_ground_y_range_m=[-TAPE_VISUAL_HALF_WIDTH_M, TAPE_VISUAL_HALF_WIDTH_M],
                                    tape_texture_y_range_m=[-ROAD_HALF_WIDTH_M, ROAD_HALF_WIDTH_M])
    elif a.road_contact == 'heightfield':
        from vd1_mujoco_road_contact import continuous_road_contact, ROAD_CONTACT
        scene = continuous_road_contact(scene, a.curvature_max)
        road_contact_details = dict(ROAD_CONTACT)
    if a.velocity_constraint == 'native_velocity':
        from vd1_mujoco_velocity_solver import add_native_velocity_brakes
        limits = [3.1415 if name.startswith('joint') else 14. if 'calf' in name else 23.
                  for name in ACTOR_JOINT_NAMES]
        scene = add_native_velocity_brakes(scene, ACTOR_JOINT_NAMES, limits)
    model = mujoco.MjModel.from_xml_path(str(scene))
    validation = (validate_vd3_model(model) if a.robot_version == 'vd3'
                  else validate_model(model, gun_mass_mode=a.gun_mass_mode))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, 'vd1_default'))
    mujoco.mj_forward(model, data)
    adapter = VD1MujocoAdapter(model, physx_implicit_position=a.physx_implicit_position)
    brake = None
    if a.velocity_constraint == 'implicit_brake':
        from vd1_mujoco_velocity_constraint import ImplicitVelocityBrake
        brake = ImplicitVelocityBrake(model, adapter.joint_ids18, adapter.velocity_limit18)
    elif a.velocity_constraint == 'native_velocity':
        from vd1_mujoco_velocity_solver import NativeVelocityLimitBrakes
        brake = NativeVelocityLimitBrakes(model, adapter.joint_ids18, adapter.velocity_limit18)
    elif a.velocity_constraint == 'impulse':
        from vd1_mujoco_velocity_impulse import VelocityImpulseConstraints
        brake = VelocityImpulseConstraints(model, adapter.joint_ids18, adapter.velocity_limit18)
    initial = adapter.read_proprioception(data)
    controller_class = VD1VisualController
    if a.visual_controller == 'guarded':
        from vd1_visual_guard import GuardedVD1VisualController
        controller_class = GuardedVD1VisualController
    controller_options = dict(paired_guidance=not a.front_only_guidance) if a.visual_controller == 'guarded' else {}
    if a.front_path_estimation:
        controller_options.update(front_path_estimation=True)
    # VD3's virtual TCP has the same fixed transform as the source URDF's
    # dummy_target, which remains the pure encoder-FK reference.
    controller = controller_class(initial_encoder_fk(initial['q'], list(ACTOR_JOINT_NAMES), URDF_PATH),
                                  forward_speed=a.speed, **controller_options)

    camera_calibration = vd3_camera_calibration() if a.robot_version == 'vd3' else CAMERA_CALIBRATION

    def calibration(name):
        c = camera_calibration[name]
        return dict(position=list(c['position_b']), quaternion_wxyz=list(c['quaternion_ros_wxyz']),
                    intrinsic_matrix=[[c['fx'], 0, c['width']/2],
                                      [0, c['fy'], c['height']/2], [0, 0, 1]])

    publisher = VD1FramePublisher(a.output_dir, calibration('front_rgb'), calibration('rear_d435i'),
                                  MARKER_CALIBRATION)
    renderer = mujoco.Renderer(model, height=480, width=640)
    # Collision geoms are invisible; actual visual meshes and trench are visible.
    scene_options = mujoco.MjvOption()
    scene_options.geomgroup[3] = 0
    frame = 0

    def capture():
        nonlocal frame
        renderer.disable_depth_rendering()
        renderer.update_scene(data, camera='front_rgb', scene_option=scene_options)
        front = renderer.render().copy()
        renderer.update_scene(data, camera='rear_d435i', scene_option=scene_options)
        rear = renderer.render().copy()
        renderer.enable_depth_rendering()
        depth = renderer.render().copy()
        renderer.disable_depth_rendering()
        publisher.publish(front, rear, depth, front_frame=frame, rear_frame=frame,
                          capture_timestamp=time.monotonic(), sim_time_s=float(data.time),
                          front_gravity_b=(adapter.read_proprioception(data)['gravity_b']
                                           if a.front_path_estimation else None))
        frame += 1

    viewer = None
    if not a.headless:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 3.4, 135, -25
        viewer.opt.geomgroup[3] = 0

    def screenshot():
        if a.screenshot:
            # Presentation-only third camera; never published as a sensor.
            camera = mujoco.MjvCamera()
            camera.lookat[:] = data.xpos[model.body('base_link').id]
            camera.distance, camera.azimuth, camera.elevation = 3.4, 135, -25
            renderer.update_scene(data, camera=camera, scene_option=scene_options)
            image = renderer.render()
            a.screenshot.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(a.screenshot), cv2.cvtColor(image, cv2.COLOR_RGB2BGR)):
                raise OSError('Cannot save presentation screenshot')

    contract = dict(version=a.robot_version.upper() + '_MUJOCO', simulator_version=mujoco.__version__, **validation,
                    policy_sha256=expected_policy_sha, joint_names=list(ACTOR_JOINT_NAMES),
                    default_joint_position=adapter.default_q18.tolist(), default_q=adapter.default_q18.tolist(),
                    actor_hz=50, pd_hz=200, physics_hz=1/a.physics_dt,
                    physics_substeps_per_pd=int(round(.005/a.physics_dt)),
                    curvature_max=a.curvature_max, speed=a.speed, static_only=a.static_only,
                    visual_controller=a.visual_controller, visual_guard=getattr(controller, 'guard_config', None),
                    front_path_estimation=a.front_path_estimation, body_motion_estimation=False,
                    front_projection=('imu_gravity_and_configured_nominal_height' if a.front_path_estimation
                                      else 'paired_rear_depth_plane'),
                    front_nominal_height_m=a.front_nominal_height_m if a.front_path_estimation else None,
                    tool_target_mode='rear_seam_direct' if seam_only else 'visual_tcp_feedback',
                    visual_tcp_feedback=not seam_only, requires_tcp_markers=not seam_only,
                    tool_markers_in_scene=not seam_only,
                    road_surface=a.road_surface,
                    road_contact='flat_plane' if a.road_surface == 'tape' else a.road_contact,
                    road_contact_details=road_contact_details,
                    external_inputs=['front_rgb', 'rear_rgb_and_aligned_optical_z_depth'],
                    proprioception=['encoder_jointpos', 'encoder_jointvel', 'imu_gyro',
                                    'imu_attitude_to_projected_gravity_only', 'previous_raw_actor_action'],
                    privileged_controller_inputs=[], imu_attitude='ideal sensor, not implemented real IMU fusion',
                    arm_actuation='native implicit position kp160 kv5 +/-120Nm',
                    physx_implicit_position=a.physx_implicit_position,
                    source_arm_pd_gains=dict(kp=160., kd=5.),
                    native_arm_velocity_coefficient=5.+a.physics_dt*160. if a.physx_implicit_position else 5.,
                    leg_actuation='explicit 160/5 PD 200Hz, original effort limits',
                    velocity_limit_note='Separate simulator constraint model; full coupled impulse if selected, never sensor clipping',
                    velocity_constraint=a.velocity_constraint,
                    startup='physics paused until first valid real rendered camera packet',
                    scene_cache=str(scene), calibration=camera_calibration)
    save_json(a.output_dir / 'runtime_contract.json', contract)
    trace = gzip.open(str(a.output_dir / 'actor_trace.jsonl.gz'), 'wt', compresslevel=3)
    evaluation = EvaluationRecorder(model, a.output_dir, a.curvature_max, a.road_surface)
    stopped = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.__setitem__(0, True))
    last_action = np.zeros(18, dtype=np.float32)
    step, stop_reason, runtime_error = 0, 'max_steps', None
    maximum_velocity_ratio = 0.
    velocity_exceedances = 0
    valid_commands = 0
    control_states = {}
    rear_discontinuity_steps = 0
    last_pipeline_state = None
    try:
        deadline = time.monotonic()+45
        while not stopped[0]:
            capture()
            packet, rejection = load_packet(a.vision_result, publisher)
            if packet is not None and packet.get('valid') is True:
                break
            if time.monotonic() > deadline:
                raise RuntimeError('No valid paired camera perception within 45 seconds: '+rejection)
            if viewer is not None:
                if not viewer.is_running():
                    stopped[0] = True
                else:
                    viewer.sync()
            time.sleep(.08)
        print('[VD1 MuJoCo] visual warmup complete; static_only=%s' % a.static_only, flush=True)
        wall_start = time.monotonic()
        while not stopped[0] and (a.max_steps == 0 or step < a.max_steps):
            if viewer is not None and not viewer.is_running():
                stop_reason = 'viewer_closed'
                break
            tick = time.monotonic()
            if step % 2 == 0:
                capture()
            packet, rejection = load_packet(a.vision_result, publisher)
            command_time = time.monotonic()
            command = controller.update(packet, now=command_time, dt=.02)
            measured = adapter.read_proprioception(data)
            observation = build_actor_observation(command['base_command_b'], command['tool_pose_b'],
                           measured['gyro_b'], measured['gravity_b'], measured['q'], measured['qd'],
                           adapter.default_q18, last_action)
            with torch.inference_mode():
                action = actor(torch.from_numpy(observation).unsqueeze(0)).numpy()[0]
            if not np.isfinite(action).all():
                stop_reason = 'nonfinite_actor_action'
                break
            target = np.clip(adapter.default_q18+.25*action, -100, 100)
            row = dict(step=step, sim_time_s=float(data.time), wall_monotonic=time.monotonic(),
                       observation=observation.tolist(), action=action.tolist(), joint_target=target.tolist(),
                       base_command_b=command['base_command_b'], tool_pose_b=command['tool_pose_b'],
                       status=command['status'], packet_rejection=rejection, static_only=a.static_only,
                       vision_input=packet_diagnostics(packet, command_time))
            if not a.static_only:
                for _ in range(4):
                    mujoco.mj_forward(model, data)
                    diagnostics = adapter.apply_targets(data, target)
                    maximum_velocity_ratio = max(maximum_velocity_ratio, diagnostics['max_velocity_limit_ratio'])
                    velocity_exceedances += int(np.count_nonzero(diagnostics['velocity_limit_exceeded18']))
                    for _ in range(int(round(.005/a.physics_dt))):
                        if brake is None:
                            mujoco.mj_step(model, data)
                        else:
                            brake.step(data)
                mujoco.mj_forward(model, data)
                row['actuator_diagnostics'] = diagnostics
            trace.write(json.dumps(row, allow_nan=False, default=lambda v: v.tolist())+'\n')
            evaluation.write(data, step)
            last_action = action.copy()
            valid_commands += int(command['status'].get('valid', False))
            control_state = command['status'].get('state', 'original')
            control_states[control_state] = control_states.get(control_state, 0)+1
            rear_discontinuity_steps += int('discontinuity' in command['status'].get('rear_reason', ''))
            step += 1
            if viewer is not None:
                with viewer.lock():
                    viewer.cam.lookat[:] = data.xpos[model.body('base_link').id]
                viewer.sync()
            pipeline_state = (command['status'].get('valid'), command['status'].get('reason'),
                              command['status'].get('target_clipped'))
            if step == 1 or step % 100 == 0 or pipeline_state != last_pipeline_state:
                print('[VD1 MuJoCo] step=%d sim=%.2fs command=%s status=%s' %
                      (step, data.time, np.round(command['base_command_b'], 3), command['status'].get('reason', 'valid')),
                      flush=True)
                print('[VD1 pipeline] state=%s front_valid=%s rear_valid=%s input=%s target_clipped=%s tool_xyz=%s' %
                      (command['status'].get('state', 'original'), command['status'].get('front_valid'),
                       command['status'].get('rear_valid'), json.dumps(row['vision_input'], allow_nan=False),
                       command['status'].get('target_clipped'), np.round(command['tool_pose_b'][:3], 3)),
                      flush=True)
                last_pipeline_state = pipeline_state
            if step == 1 or step % 250 == 0:
                screenshot()
                trace.flush()
                evaluation.handle.flush()
            # A detector failure decelerates through the pure visual controller.
            # Only IMU tilt/nonfinite proprioception ends this local simulation.
            gravity = adapter.read_proprioception(data)['gravity_b']
            if not np.isfinite(gravity).all() or gravity[2] > -.5:
                stop_reason = 'IMU_tilt_stop'
                break
            remaining = .02-(time.monotonic()-tick)
            if remaining > 0:
                time.sleep(remaining)
        if stopped[0]:
            stop_reason = 'interrupted'
        screenshot()
    except Exception as error:
        stop_reason = 'runtime_error'
        runtime_error = repr(error)
        raise
    finally:
        trace.close()
        evaluation.close()
        save_json(a.output_dir / 'summary.json', dict(stop_reason=stop_reason, actor_steps=step,
                  sim_time_s=float(data.time), static_only=a.static_only, runtime_error=runtime_error,
                  velocity_limit_max_ratio=maximum_velocity_ratio,
                  velocity_limit_exceeded_joint_ticks=velocity_exceedances,
                  velocity_constraint=brake.summary() if brake is not None else None,
                  valid_command_steps=valid_commands, visual_controller=a.visual_controller,
                  front_path_estimation=a.front_path_estimation, body_motion_estimation=False,
                  tool_target_mode=contract['tool_target_mode'], visual_tcp_feedback=not seam_only,
                  requires_tcp_markers=not seam_only, tool_markers_in_scene=not seam_only,
                  road_surface=a.road_surface, road_contact=contract['road_contact'],
                  gun_mass_mode=validation['gun_mass_mode'], gun_mass_kg=validation['gun_mass_kg'],
                  robot_mass_kg=validation['robot_mass_kg'],
                  control_state_steps=control_states, rear_discontinuity_steps=rear_discontinuity_steps,
                  evaluation=evaluation.summary()))
        if viewer is not None:
            viewer.close()
        renderer.close()
    return 0 if stop_reason not in ('runtime_error', 'IMU_tilt_stop', 'nonfinite_actor_action') else 1


if __name__ == '__main__':
    sys.exit(main())
