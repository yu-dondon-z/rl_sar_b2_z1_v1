#!/usr/bin/env python3
"""VD12 final actor driven by rendered front RGB and rear RGB-D perception.

Simulation only. No ideal-camera/world-seam command fallback, automatic reset,
policy export, board synchronization or robot I/O. The legacy truth rollout is
kept separately in play_vd12_mujoco.py for differential diagnosis only.
"""
from __future__ import annotations

import argparse
from collections import deque
import gzip
import itertools
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
VISION_ROOT = ROOT / 'scripts/vision'
RHANET_ROOT = VISION_ROOT / 'rhacracknet'
sys.path.insert(0, str(VISION_ROOT))
sys.path.insert(0, str(RHANET_ROOT))
import play_vd12_mujoco as checked
from vd12_visual_control import VD12VisualController

DETECTOR = ROOT / 'policy/vision/rhanet_dep_camcrack789_b2_stage1/best_ckpt.pth'


def parse(argv=None, *, version=12):
    if version == 14:
        import vd14_mujoco_contract as backend
    elif version == 13:
        import vd13_mujoco_contract as backend
    elif version == 12:
        backend = checked
    else:
        raise ValueError('Unsupported vision deployment version')
    parser = argparse.ArgumentParser(description=(__doc__ if version == 12 else
        f'VD{version}: rendered RGB/RGB-D -> RHANet -> 70-to-18 actor. Simulation only; no truth fallback.'))
    parser.add_argument('--checkpoint', type=Path, default=backend.DEFAULT_CHECKPOINT)
    parser.add_argument('--duration', type=float, default=0. if version == 14 else 30.,
                        help='VD14: 0 runs until stopped (default); a positive value sets an optional simulation-time limit.')
    parser.add_argument('--speed', type=float, default=.2)
    parser.add_argument('--curvature', type=float, default=.05,
                        help='Rendered road only; never passed to the visual controller.')
    display = parser.add_mutually_exclusive_group()
    display.add_argument('--viewer', dest='viewer', action='store_true', default=version == 14,
                         help='Open MuJoCo viewer (default for VD14).')
    display.add_argument('--headless', dest='viewer', action='store_false',
                         help='Run without the MuJoCo viewer.')
    vision_window = parser.add_mutually_exclusive_group()
    vision_window.add_argument('--vision-window', dest='vision_window', action='store_true',
                               default=version == 14,
                               help='Show front/rear perception overlay (default for VD14).')
    vision_window.add_argument('--no-vision-window', dest='vision_window', action='store_false',
                               help='Disable the perception overlay window.')
    parser.add_argument('--vision-mode', choices=('rhanet', 'classical'), default='rhanet',
                        help='RHANet is default; classical is an explicit pixel-only diagnostic, never a fallback.')
    parser.add_argument('--vision-device', default='cuda:0')
    parser.add_argument('--detector-checkpoint', type=Path, default=DETECTOR)
    parser.add_argument('--vision-max-age', type=float, default=1.)
    parser.add_argument('--motion-compensation', choices=('leg_imu', 'none'),
                        default='leg_imu' if version == 14 else 'none',
                        help='VD14: sensor-local capture-to-control transform. none retains the historical diagnostic.')
    parser.add_argument('--rear-centerline-mode', choices=('skeleton', 'cross_sections'), default='cross_sections')
    parser.add_argument('--rear-path-memory-s', type=float, default=.6,
                        help=('Observed-path cache. VD14 leg_imu uses sensor local estimates; none/older entries use SENT commands. 0 disables.'))
    parser.add_argument('--rear-occlusion-extension', type=float, default=.05,
                        help='Existing reliable visible-line estimate limit in metres; 0 disables far-end extrapolation.')
    historical = parser.add_mutually_exclusive_group()
    historical.add_argument('--historical-tangent-estimate', dest='historical_tangent_estimate',
                            action='store_true', default=False,
                            help='Diagnostic only: allow a bounded tangent fit of historical support; disabled by default.')
    historical.add_argument('--no-historical-tangent-estimate', dest='historical_tangent_estimate',
                            action='store_false',
                            help='Explicitly retain the default: historical paths must cover the work point.')
    parser.add_argument('--warmup-timeout', type=float, default=45.)
    parser.add_argument('--output-root', type=Path, default=ROOT/f'outputs/vd{version}_vision')
    args = parser.parse_args(argv)
    if version != 14 and args.motion_compensation != 'none':
        parser.error('Sensor motion compensation is isolated to VD14')
    if not (math.isfinite(args.duration) and
            ((version == 14 and args.duration == 0.) or
             (args.duration >= .02 and (version == 14 or args.duration <= 30.)
              and math.isclose(args.duration/.02, round(args.duration/.02), rel_tol=0, abs_tol=1.e-9)))):
        parser.error('VD14 duration must be 0 (continuous) or positive on the 20ms grid; '
                     'VD12/VD13 retain [.02,30] seconds')
    if not (math.isfinite(args.speed) and 0 < args.speed <= .4
            and math.isfinite(args.curvature) and 0 <= args.curvature <= .15):
        parser.error('speed (0,.4] and curvature [0,.15] required')
    if not (math.isfinite(args.vision_max_age) and .1 <= args.vision_max_age <= 1.
            and math.isfinite(args.warmup_timeout) and 1 <= args.warmup_timeout <= 90):
        parser.error('vision age [.1,1] and warmup [1,90] seconds required')
    if not (math.isfinite(args.rear_path_memory_s) and 0 <= args.rear_path_memory_s <= .8
            and math.isfinite(args.rear_occlusion_extension) and 0 <= args.rear_occlusion_extension <= .05):
        parser.error('rear path memory [0,.8]s and local far estimate [0,.05]m required')
    return args


def control_steps(duration):
    """Stream control ticks without allocating an unbounded counter/history."""
    return itertools.count() if duration == 0. else range(round(duration/.02))


def rear_support_wait_can_step(visual, *, version, motion_compensation):
    """Allow only VD14's explicit zero-travel wait, never arbitrary invalid vision."""
    status = visual['status']
    if not (version == 14 and motion_compensation == 'leg_imu'
            and status.get('failure_code') == 'rear_target_support_shortfall'
            and status.get('control_state') == 'rear_support_wait'
            and status.get('runtime_must_terminate_on_invalid') is False
            and status.get('valid') is False
            and status.get('measurement_valid') is False
            and status.get('measurement_is_new') is False
            and status.get('rear_target_held') is True):
        return False
    command, pose = np.asarray(visual['base_command_b']), np.asarray(visual['tool_pose_b'])
    return bool(command.shape == (3,) and pose.shape == (7,) and np.isfinite(pose).all()
                and np.array_equal(command, np.zeros(3))
                and math.isclose(float(np.linalg.norm(pose[3:])), 1., abs_tol=1.e-6))


def update_sensor_scene(renderer, data, camera, options):
    """The light spectator backdrop must never change robot camera images."""
    import mujoco
    renderer.update_scene(data, camera=camera, scene_option=options)
    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SKYBOX] = 0


def capture_clock_ages(packet, sim_time_s, capture_sim_times, max_age_s):
    """Check the original simulated capture clock against private publication.

    Wall-clock freshness is independently checked by the visual controller.
    Simulation time here is a clock only, never a base pose or seam input.
    """
    if not math.isfinite(sim_time_s) or not math.isfinite(max_age_s) or max_age_s <= 0.:
        raise ValueError('invalid simulation clock or capture-age limit')
    result = {}
    for name in ('front', 'rear'):
        stream = packet[name]
        sequence, stamp = stream['sequence'], stream.get('capture_sim_time_s')
        if (not isinstance(stamp, (int, float)) or isinstance(stamp, bool)
                or not math.isfinite(stamp) or sequence not in capture_sim_times
                or abs(stamp - capture_sim_times[sequence]) > 1.e-9):
            raise ValueError(name+' simulated capture time differs from private publication history')
        age = float(sim_time_s - stamp)
        if not -1.e-9 <= age <= max_age_s + 1.e-9:
            raise ValueError(name+' simulated capture is stale or from the future')
        result[name+'_capture_age_sim_s'] = age
        result[name+'_capture_sim_time_s'] = float(stamp)
    if abs(result['front_capture_sim_time_s']-result['rear_capture_sim_time_s']) > 1.e-9:
        raise ValueError('simulated front/rear captures are not paired')
    result['capture_age_sim_s'] = max(result['front_capture_age_sim_s'], result['rear_capture_age_sim_s'])
    return result


def visual_packet(path, publisher, *, sim_time_s=None, capture_sim_times=None, max_age_s=None):
    """Provenance is checked before image-derived geometry can enter control."""
    try:
        packet = json.loads(path.read_text())
        accepted, reason = publisher.validate_packet(packet)
        if accepted and sim_time_s is not None:
            capture_clock_ages(packet, sim_time_s, capture_sim_times, max_age_s)
        return (packet, '') if accepted else (None, reason)
    except (OSError, ValueError, TypeError) as error:
        return None, str(error)


def native_snapshot(mujoco, model, data, adapter, dofs):
    forces = np.zeros((model.nbody, 3))
    for index in range(data.ncon):
        contact = data.contact[index]
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, index, force)
        world = contact.frame.reshape(3, 3).T @ force[:3]
        first, second = model.geom_bodyid[[contact.geom1, contact.geom2]]
        forces[first] -= world
        forces[second] += world
    return dict(body_contact_force_world_n=forces.tolist(),
                drive_effort18_nm=data.actuator_force[adapter.actuator_ids18].copy().tolist(),
                native_constraint_effort18_nm=data.qfrc_constraint[dofs].copy().tolist(),
                native_efc_type=data.efc_type[:data.nefc].copy().tolist(),
                native_efc_force=data.efc_force[:data.nefc].copy().tolist())


def main(argv=None, *, version=12):
    args = parse(argv, version=version)
    if version == 14:
        import vd14_mujoco_contract as backend
    elif version == 13:
        import vd13_mujoco_contract as backend
    else:
        backend = checked
    os.environ.setdefault('MUJOCO_GL', 'glfw' if args.viewer else 'egl')
    import mujoco
    import torch
    from vd1_frames import VD1FramePublisher
    from vd1_mujoco_contract import ACTOR_JOINT_NAMES, VD1MujocoAdapter
    from vd1_mujoco_scene import build_scene, validate_vd10_model
    from vd1_mujoco_velocity_impulse import VelocityImpulseConstraints
    from vd5_camera_geometry import measured_camera_calibration
    from vd5_visual_control import encoder_tcp_pose

    shared = backend.shared
    if args.vision_mode == 'rhanet' and not args.detector_checkpoint.is_file():
        raise FileNotFoundError('RHANet checkpoint missing; no classical/truth fallback: '+str(args.detector_checkpoint))
    torch.set_num_threads(1)
    path = args.checkpoint.expanduser().resolve(strict=True)
    checkpoint = torch.load(path, map_location='cpu', weights_only=False)
    profile = backend.load_pure_module('vd12_visual_contract', backend.ASSETS/'vd12_contract.py').geometry_profile(backend.URDF)
    saved = backend.load_yaml(path.parent/'params/env.yaml')
    agent = backend.load_yaml(path.parent/'params/agent.yaml')
    curriculum = backend.validate_checkpoint(checkpoint, agent, saved, profile)
    settings = backend.validate_saved_settings(saved)
    actor = torch.nn.Sequential(torch.nn.Linear(70,512), torch.nn.ELU(), torch.nn.Linear(512,256),
        torch.nn.ELU(), torch.nn.Linear(256,128), torch.nn.ELU(), torch.nn.Linear(128,18)).eval()
    weights = {key[6:]: value for key,value in checkpoint['model_state_dict'].items() if key.startswith('actor.')}
    actor.load_state_dict(weights, strict=True)
    if not all(torch.isfinite(value).all() for value in weights.values()):
        raise ValueError('Nonfinite actor weights')
    if tuple(ACTOR_JOINT_NAMES) != tuple(shared.EXPECTED_JOINT_NAMES):
        raise ValueError('Actor joint order differs')
    if shared.sha256(backend.URDF) != backend.URDF_SHA256:
        raise ValueError('New-gun source URDF differs')
    args.output_root.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='vision_'+time.strftime('%Y%m%d_%H%M%S')+'_', dir=args.output_root.resolve()))
    cameras = measured_camera_calibration()
    scene = build_scene(args.curvature, Path(tempfile.mkdtemp(prefix=f'vd{version}_vision_', dir='/dev/shm')),
        timestep=.005, tool_markers=False, road_surface='tape', robot_version=backend.ROBOT_VERSION,
        camera_calibration=cameras, odometry_sensors=args.motion_compensation == 'leg_imu',
        light_background=version == 14)
    model = mujoco.MjModel.from_xml_path(str(scene))
    audit = backend.validate_model(model) if version in (13, 14) else validate_vd10_model(model)
    shared.save(output/'asset_audit.json', audit)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('vd1_default').id)
    adapter = VD1MujocoAdapter(model, physx_implicit_position=True)
    default = shared.default_position()
    adapter.default_q18 = default.copy()
    qids = model.jnt_qposadr[adapter.joint_ids18]
    dofs = model.jnt_dofadr[adapter.joint_ids18]
    data.qpos[2], data.qpos[qids], data.qvel[:] = .5, default, 0.
    adapter.apply_targets(data, default)
    mujoco.mj_forward(model, data)
    constraint = VelocityImpulseConstraints(model, adapter.joint_ids18, adapter.velocity_limit18)
    guard = backend.MujocoGunGeometryGuard(model)
    if guard.profile != profile:
        raise ValueError('Runtime geometry/profile differs')
    initial_geometry = guard.evaluate(data)
    base, tcp, tool = model.body('base_link').id, model.site('tool_tcp').id, model.body('link06').id
    feet = [model.body(name).id for name in shared.FEET]
    illegal = [index for index in range(1, model.nbody)
               if re.fullmatch(r'base_link|.*_hip|.*_thigh|.*_calf|link0[0-5]', model.body(index).name or '')]
    if not illegal:
        raise ValueError('Illegal-contact body registry empty')
    arm_indices = [ACTOR_JOINT_NAMES.index(name) for name in shared.ARM_JOINT_NAMES]
    initial_p, initial_q = encoder_tcp_pose(default[arm_indices], backend.URDF,
                                           backend.TCP_POSITION, backend.TCP_ROTATION)
    np.testing.assert_allclose(data.xmat[base].reshape(3,3).T @ (data.site_xpos[tcp]-data.xpos[base]), initial_p, atol=1.e-6)

    motion = motion_loads = None
    motion_trace = None
    if args.motion_compensation == 'leg_imu':
        from vd14_motion_compensation import (FootLoadSensors, LegKinematics,
            SensorMotionHistory, MotionCompensatedVisualController)
        motion_loads = FootLoadSensors(model)
        motion = SensorMotionHistory(LegKinematics(backend.URDF, ACTOR_JOINT_NAMES))
        motion.sample(0., motion_loads.proprioception(adapter.read_proprioception(data), data),
                      motion_loads.read(data))
        motion_trace = (output/'sensor_motion_200hz.jsonl').open('w')

    def make_controller():
        if motion is not None:
            return MotionCompensatedVisualController(initial_p, initial_q, motion_history=motion,
                speed=args.speed, trailing_distance=backend.WORK_DISTANCE_M,
                stale_timeout_s=args.vision_max_age, rear_path_memory_s=args.rear_path_memory_s,
                max_occlusion_extension=args.rear_occlusion_extension,
                historical_tangent_estimate=args.historical_tangent_estimate)
        return VD12VisualController(initial_p, initial_q, speed=args.speed,
            trailing_distance=.8 if version == 12 else backend.WORK_DISTANCE_M, stale_timeout_s=args.vision_max_age,
            rear_path_memory_s=args.rear_path_memory_s, max_occlusion_extension=args.rear_occlusion_extension,
            historical_tangent_estimate=args.historical_tangent_estimate)

    def controller_update(controller, packet, now, dt):
        if motion is not None:
            return controller.update(packet, now, dt, control_time_s=float(data.time))
        return controller.update(packet, now, dt)

    def calibration(name):
        camera = cameras[name]
        return dict(position=camera['position_b'], quaternion_wxyz=camera['quaternion_ros_wxyz'],
                    intrinsic_matrix=camera['intrinsic_matrix'])

    publisher = VD1FramePublisher(output, calibration('front_rgb'), calibration('rear_d435i'), None)
    renderer = mujoco.Renderer(model, height=480, width=640)
    options = mujoco.MjvOption()
    options.geomgroup[3] = 0
    frames = 0
    capture_sim_times = {}
    captures = (output/'camera_captures.jsonl').open('w')

    def capture():
        nonlocal frames
        # Simulation state is unchanged across this paired render. Time is taken
        # BEFORE render/publication/inference, never renewed after processing.
        stamp = time.monotonic()
        sensor = adapter.read_proprioception(data)
        renderer.disable_depth_rendering()
        update_sensor_scene(renderer, data, 'front_rgb', options)
        front = renderer.render().copy()
        update_sensor_scene(renderer, data, 'rear_d435i', options)
        rear = renderer.render().copy()
        renderer.enable_depth_rendering()
        depth = renderer.render().copy()
        renderer.disable_depth_rendering()
        published = publisher.publish(front, rear, depth, front_frame=frames, rear_frame=frames,
            capture_timestamp=stamp, sim_time_s=float(data.time),
            rear_arm_joint_positions=sensor['q'][arm_indices])
        if not published:
            raise RuntimeError('Paired camera capture did not advance')
        capture_sim_times[frames] = float(data.time)
        while len(capture_sim_times) > 128:
            del capture_sim_times[next(iter(capture_sim_times))]
        captures.write(json.dumps(dict(frame=frames, capture_timestamp=stamp, sim_time_s=float(data.time),
                                       publication_complete_timestamp=time.monotonic()))+'\n')
        captures.flush()
        frames += 1

    bridge = ROOT/'scripts/run_vd1_dual_camera_bridge.py'
    command = [sys.executable, '-B', str(bridge), '--exchange-dir', str(output),
        '--rhacracknet-root', str(RHANET_ROOT),
        '--mode', args.vision_mode, '--device', args.vision_device,
        '--checkpoint', str(args.detector_checkpoint), '--gun-mask-urdf', str(backend.URDF),
        '--gun-mask-near-clip', '--gun-mask-near-plane-m', str(model.vis.map.znear*model.stat.extent),
        '--rear-centerline-mode', args.rear_centerline_mode,
        '--rear-component-mode', 'vd14_rgb_evidence' if version == 14 else 'workpoint_support',
        '--rear-work-distance-m', str(backend.WORK_DISTANCE_M),
        '--rear-target-occlusion-extension-m', str(args.rear_occlusion_extension),
        '--save-overlay', '--max-frame-age-s', str(args.vision_max_age)]
    if args.vision_window:
        command.append('--display')
    source_paths = [Path(__file__), Path(checked.__file__), Path(shared.__file__),
                    ROOT/'scripts/vd12_visual_control.py', bridge, ROOT/'scripts/vd5_gun_mask.py',
                    ROOT/'scripts/vd12_gun_mask.py',
                    ROOT/'scripts/vd12_visual_rear.py',
                    ROOT/'scripts/vd1_visual_rear.py', ROOT/'scripts/vd5_camera_geometry.py',
                    ROOT/'scripts/vd5_visual_control.py', ROOT/'scripts/vd5_rear_path_memory.py',
                    ROOT/'scripts/vd5_rear_centerline.py', ROOT/'scripts/vd1_mujoco_scene.py',
                    ROOT/'scripts/vd1_mujoco_contract.py', ROOT/'scripts/vd1_mujoco_velocity_impulse.py',
                    ROOT/'scripts/vd12_geometry_guard.py', VISION_ROOT/'vd1_frames.py',
                    VISION_ROOT/'vd1_control.py', VISION_ROOT/'run_dual_camera_bridge.py',
                    VISION_ROOT/'dual_camera_perception.py']
    if version in (13, 14):
        source_paths += [Path(backend.__file__), Path(backend.evaluation.__file__),
                         ROOT/f'scripts/deploy_vd{version}_vision.py', backend.ASSETS/'vd13_contract.py',
                         backend.URDF, backend.ASSETS/'vd12_geometry.py', backend.ASSETS/'vd12_contract.py']
    if version == 14:
        source_paths += [backend.ASSETS/'vd14_contract.py',
                         Path(backend.evaluation.mount_evaluation.__file__),
                         ROOT/'scripts/vd13_mujoco_contract.py', ROOT/'scripts/vd14_visual_rear.py',
                         ROOT/'scripts/vd14_viewer.py']
    if motion is not None:
        source_paths.append(ROOT/'scripts/vd14_motion_compensation.py')
    manifest = dict(checkpoint=str(path), checkpoint_sha256=shared.sha256(path), iteration=checkpoint['iter'],
        run_mode='continuous_until_user_or_guard_stop' if args.duration == 0. else 'timed_sample',
        requested_duration_s=None if args.duration == 0. else args.duration,
        viewer_background='light_skybox_no_geometry' if version == 14 else 'default_black',
        spectator_far_clip_m=250. if version == 14 else float(model.vis.map.zfar*model.stat.extent),
        spectator_state_isolated=version == 14,
        vision_window=bool(args.vision_window),
        robot_camera_skybox_rendered=False,
        experiment=checkpoint['training_experiment_name'], geometry_profile=profile, saved_curriculum=curriculum,
        saved_settings_verified=settings, source_sha256={str(source):shared.sha256(source) for source in source_paths},
        actor_observations=70, actor_actions=18, joint_order=list(ACTOR_JOINT_NAMES),
        command_source='rendered front RGB and rear aligned RGB-D through pixel perception only',
        truth_command_fallback=False, actor_extra_world_fields=[], camera_calibration=cameras,
        perception_mode=args.vision_mode, detector_checkpoint=str(args.detector_checkpoint),
        render_near_plane_m=float(model.vis.map.znear*model.stat.extent), gun_mask_near_clipping=True,
        detector_sha256=shared.sha256(args.detector_checkpoint) if args.vision_mode=='rhanet' else None,
        maximum_capture_age_s=args.vision_max_age, capture_age_checked_clocks=['host_monotonic', 'simulation'],
        requested_camera_sim_hz=25., physics_hz=200, actor_hz=50,
        actual_camera_timing='See camera_captures.jsonl; render/vision/wall throughput is not guaranteed 25Hz.',
        startup_note='2s quintic interpolation, encoder-FK BODY initial anchor; not training WORLD anchor without measured odometry.',
        frame_transport_note='Measurements stay capture-base-relative, age checked; no invented odometry or pose transport.',
        warmup_note='Physics paused before first valid visual pair only. No resets or dynamic pauses on vision failure.',
        work_distance_m=backend.WORK_DISTANCE_M, tcp_position_link_m=list(backend.TCP_POSITION),
        mount_profile=checkpoint.get('vd13_mount_profile'),
        posture_profile=checkpoint.get('vd14_posture_profile'),
        rear_hold=False, rear_memory=args.rear_path_memory_s > 0,
        rear_path_memory_s=args.rear_path_memory_s, rear_memory_motion_source='sent base command integration, NOT measured odometry',
        historical_tangent_estimate=args.historical_tangent_estimate,
        historical_tangent_bounds='Original near-field <=.25m; >=8 measured points spanning >=.12m, line-fit residual P90<=8mm and max<=15mm; original cache clock/motion/gap limits still apply.',
        rear_centerline_mode=args.rear_centerline_mode,
        rear_component_mode='workpoint_support_single_visible_component_no_gap_stitching',
        rear_target_near_field_estimate='Existing visible-endpoint local tangent, at most .25m toward the base; explicitly labelled, never directly observed.',
        far_endpoint_occlusion_estimate_limit_m=args.rear_occlusion_extension,
        far_endpoint_estimate_note='Existing local visible-line fit: >=8 measured points, >=.12m support, residual P90<=8mm; explicitly estimated, no synthetic pixels/depth.',
        vision_invalid_action='End simulation before next actor/physics step. Last setpoint is not a new measurement.',
        geometry_guard=guard.metadata, initial_geometry=initial_geometry,
        reset_mode='nominal differential; no RSI/randomization',
        sample_clock=dict(state='step * .005', native_force='(step - 1) * .005', phase_difference_s=.005),
        actuator_force_note='Actual native actuator_force/constraints before split velocity projection, not PD estimate.',
        drive_note='Existing implicit-position adapter and split velocity constraint, not an exact PhysX port.',
        simulation_only=True, hardware_io=False, default_actor_modified=False, policy_exported=False)
    if motion is not None:
        manifest.update(motion_compensation='leg_imu',
            motion_inputs=['encoder_q_qd', 'IMU_gyro_gravity_accelerometer', 'actual_named_touch_sensors'],
            motion_reference='identity_at_reset_local_frame_not_simulator_world',
            motion_assumption='loaded_spherical_foot_no_slip_on_locally_level_ground; <=50ms inertial prediction; explicit zero-velocity nominal-reset prior',
            motion_sensor_note='Ideal simulated IMU and named foot touch sensors; no hardware calibration/validation claimed.',
            startup_note='2s quintic interpolation, encoder-FK initial anchor in SENSOR local motion frame.',
            startup_quaternion_transport='relative_sensor_rotation_quaternion * signed_initial_encoder_FK_quaternion; no q.w>=0 recanonicalization of the start pose',
            frame_transport_note='Original capture clocks/ages retained; sensor estimate maps capture geometry to each control tick.',
            rear_memory_motion_source='leg_IMU_local_estimate_NOT_sent_commands')
    else:
        manifest['motion_compensation'] = 'none'
    if version == 14:
        manifest.update(rear_component_mode='vd14_rgb_evidence',
            rear_rgb_evidence='RHANet intersect bilateral dark-ridge support; dark-seam assumption; neural probability unchanged',
            rear_target_continuity=('sensor-local work-point step <= .025m + configured_speed * elapsed_sim_s; '
                                    'reject and terminate, not a low-pass, clipping or hold') if motion is not None else 'not available without sensor motion')
        if motion is not None:
            manifest.update(rear_support_shortfall_action='Zero base travel; transport last accepted COMMAND in sensor-local frame; keep physics/cameras running; resume on supported continuous work point.',
                rear_support_shortfall_timeout_s=None,
                vision_invalid_action='Only typed rear target support shortfall may wait without exiting. Other visual failures and all physical guards retain latched termination.',
                warmup_note='Physics paused before first valid target only. Support wait after startup continues physics, sensors and camera processing; never resets.')
    shared.save(output/'manifest.json', manifest)
    print(f'VD{version} dual-camera deployment output: '+str(output), flush=True)
    stopped = [False]
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopped.__setitem__(0, True))
    viewer = process = None
    spectator = None
    vision_log = (output/'vision_bridge.log').open('w')
    count = actor_count = new_measurements = 0
    measured_steps = memory_steps = estimated_steps = 0
    directly_observed_steps = front_fit_fallback_steps = 0
    historical_tangent_steps = 0
    support_wait_steps = support_wait_events = support_wait_recoveries = 0
    was_support_waiting = False
    maximum_support_wait_s = maximum_held_setpoint_age_s = 0.
    front_fit_fallback_sequences = set()
    maximum_capture_age = maximum_target_source_age = 0.
    maximum_capture_sim_age = maximum_target_source_sim_age = 0.
    minimum = initial_geometry['minimum_clearance_lower_bound_m']
    reason, terminal, error_text = 'not_started', None, None
    previous = np.zeros(18, dtype=np.float32)
    history, latch = deque(maxlen=20), checked.GeometryLatch()
    latch.sample(initial_geometry, 0)
    warmup_rejection = None
    started_wall = None
    active_end_wall = None
    try:
        if latch.violated:
            reason, terminal = 'vd12_exclusion_at_reset', latch.first
        else:
            env = dict(os.environ, PYTHONUNBUFFERED='1', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1')
            process = subprocess.Popen(command, stdout=vision_log, stderr=subprocess.STDOUT, env=env)
            if args.viewer:
                import mujoco.viewer
                if version == 14:
                    from vd14_viewer import SpectatorState
                    spectator = SpectatorState(scene, data)
                    viewer = mujoco.viewer.launch_passive(spectator.model, spectator.data)
                else:
                    viewer = mujoco.viewer.launch_passive(model, data)
                with viewer.lock():
                    shared.configure_camera(viewer.cam, data.xpos[base])
                    viewer.opt.geomgroup[3] = 0
            deadline = time.monotonic()+args.warmup_timeout
            first_visual = None
            while time.monotonic() < deadline:
                if stopped[0] or (viewer and not viewer.is_running()):
                    reason = 'interrupted_before_vision_ready'
                    break
                if process.poll() is not None:
                    raise RuntimeError('Vision bridge exited during warmup; see vision_bridge.log')
                capture()
                packet, rejection = visual_packet(output/'latest_dual_trajectory.json', publisher,
                    sim_time_s=float(data.time), capture_sim_times=capture_sim_times, max_age_s=args.vision_max_age)
                if packet is not None and packet.get('valid') is True:
                    candidate = make_controller()
                    visual = controller_update(candidate, packet, now=time.monotonic(), dt=.02)
                    if visual['status']['valid']:
                        controller, first_visual = candidate, visual
                        break
                    rejection = visual['status']['reason']
                warmup_rejection = rejection or (packet or {}).get('reason')
                if viewer:
                    viewer.sync()
                time.sleep(.02)
            if first_visual is None and reason == 'not_started':
                reason = 'vision_warmup_timeout'
                terminal = dict(reason=warmup_rejection)
            if first_visual is not None:
                reason = 'duration_complete'
                started_wall = time.monotonic()
                print(f'First measured front/rear pair accepted; starting 70-to-18 VD{version} actor.', flush=True)
                with gzip.open(output/'actor_50hz.jsonl.gz', 'wt') as actor_log, \
                        gzip.open(output/'physics_200hz.jsonl.gz', 'wt') as physics_log:
                    for step in control_steps(args.duration):
                        tick_wall = time.monotonic()
                        if stopped[0] or (viewer and not viewer.is_running()):
                            reason = 'interrupted_or_viewer_closed'
                            break
                        if process.poll() is not None:
                            reason = 'vision_bridge_exit'
                            break
                        if step == 0:
                            visual = first_visual
                        else:
                            if step % 2 == 0:
                                capture()
                            packet, rejection = visual_packet(output/'latest_dual_trajectory.json', publisher,
                                sim_time_s=float(data.time), capture_sim_times=capture_sim_times, max_age_s=args.vision_max_age)
                            visual = controller_update(controller, packet, now=time.monotonic(), dt=.02)
                        status = visual['status']
                        support_waiting = rear_support_wait_can_step(visual, version=version,
                            motion_compensation=args.motion_compensation)
                        if version == 14:
                            display_status = dict(control_state=status['control_state'],
                                valid=status['valid'], reason=status['reason'],
                                control_update_monotonic_s=time.monotonic(),
                                rear_support_required_extension_m=status.get('rear_support_required_extension_m'),
                                rear_support_allowed_extension_m=status.get('rear_support_allowed_extension_m'))
                            display_tmp = output/'.latest_control_status.tmp.json'
                            shared.save(display_tmp, display_status)
                            display_tmp.replace(output/'latest_control_status.json')
                        if not status['valid'] and not support_waiting:
                            reason = 'vision_invalid'
                            terminal = dict(sim_time_s=count*.005, visual_status=status,
                                            packet_rejection=rejection)
                            shared.save(output/'visual_terminal.json', terminal)
                            break
                        status.update(capture_clock_ages(packet, float(data.time), capture_sim_times, args.vision_max_age))
                        status['control_use_monotonic_s'] = time.monotonic()
                        status['control_use_sim_time_s'] = float(data.time)
                        if support_waiting:
                            # This is an old COMMAND, not cached visual support.
                            # Its age is explicit; do not refresh or validate it
                            # as though a new image had re-observed that target.
                            source_sim_age = None
                        elif status.get('rear_memory_used', False):
                            source_sequences = status['rear_memory_source_sequences']
                            if any(sequence not in capture_sim_times for sequence in source_sequences):
                                raise ValueError('historical target simulated capture left publication history')
                            source_sim_age = max(float(data.time)-capture_sim_times[sequence] for sequence in source_sequences)
                            if not -1.e-9 <= source_sim_age <= args.rear_path_memory_s + 1.e-9:
                                reason = 'vision_memory_sim_capture_expired'
                                terminal = dict(sim_time_s=count*.005, memory_source_sim_age_s=source_sim_age)
                                break
                        else:
                            source_sim_age = status['rear_capture_age_sim_s']
                        status['rear_target_source_age_sim_s'] = source_sim_age
                        sensor = adapter.read_proprioception(data)
                        pose = np.asarray(visual['tool_pose_b'])
                        base_command = np.asarray(visual['base_command_b'])
                        obs = shared.actor_observation(base_command, pose, sensor, default, previous)
                        if obs.shape != (70,) or not np.isfinite(obs).all():
                            raise ValueError('Invalid actor observation')
                        with torch.inference_mode():
                            action = actor(torch.from_numpy(obs).unsqueeze(0)).numpy()[0]
                        if action.shape != (18,) or not np.isfinite(action).all():
                            raise ValueError('Invalid actor action')
                        target = checked.processed_joint_target(default, action)
                        actor_log.write(json.dumps(dict(time_s=step*.02, observation=obs.tolist(),
                            base_command_b=base_command.tolist(), tool_pose_b=pose.tolist(),
                            visual_status=status, action=action.tolist(), joint_target18_rad=target.tolist()), allow_nan=False)+'\n')
                        previous = action.copy()
                        actor_count += 1
                        support_wait_steps += int(support_waiting)
                        support_wait_events += int(support_waiting and not was_support_waiting)
                        support_wait_recoveries += int(status.get('resumed_from_rear_support_wait', False))
                        if support_waiting:
                            maximum_support_wait_s = max(maximum_support_wait_s, status['support_wait_elapsed_s'])
                            maximum_held_setpoint_age_s = max(maximum_held_setpoint_age_s, status['held_setpoint_age_s'])
                        if support_waiting != was_support_waiting:
                            print('Rear work point: '+('WAIT; base command zero, cameras/physics continue' if support_waiting
                                else 'supported again; tracking resumed'), flush=True)
                        was_support_waiting = support_waiting
                        new_measurements += int(status['measurement_is_new'])
                        measured_steps += int(status['measurement_valid'])
                        memory_steps += int(status.get('rear_memory_used', False))
                        estimated_steps += int(status.get('target_estimated', False))
                        historical_tangent_steps += int(status.get('rear_memory_tangent_estimated', False))
                        directly_observed_steps += int(status.get('rear_target_directly_observed', False))
                        if status.get('front_fit_candidate_fallback', False):
                            front_fit_fallback_steps += 1
                            front_fit_fallback_sequences.add(status['source_sequence'])
                        if not support_waiting:
                            maximum_capture_age = max(maximum_capture_age, status['capture_age_s'])
                            maximum_capture_sim_age = max(maximum_capture_sim_age, status['capture_age_sim_s'])
                            maximum_target_source_sim_age = max(maximum_target_source_sim_age, source_sim_age)
                        source_age = status.get('rear_target_source_age_s')
                        if source_age is not None:
                            maximum_target_source_age = max(maximum_target_source_age, source_age)
                        # The following world-state computations are evaluation
                        # and guards only; they never feed visual/actor commands.
                        target_w = data.xpos[base]+data.xmat[base].reshape(3,3) @ pose[:3]
                        for _ in range(4):
                            adapter.apply_targets(data, target)
                            native = {}
                            def observe_native(current):
                                native.update(native_snapshot(mujoco, model, current, adapter, dofs))
                            constraint.step(data, evaluation_observer=observe_native)
                            count += 1
                            if motion is not None:
                                motion_input = motion_loads.proprioception(adapter.read_proprioception(data), data)
                                motion_pressure = motion_loads.read(data)
                                # Write genuine inputs even if the estimator
                                # rejects this very step. Never log fabricated
                                # support or replace the engine sensor buffer.
                                motion_trace.write(json.dumps(dict(kind='sensors', time_s=float(data.time),
                                    foot_load_n=motion_pressure.tolist(),
                                    **{key:value.tolist() for key,value in motion_input.items()}), allow_nan=False)+'\n')
                                sensor_sample = motion.sample(float(data.time), motion_input, motion_pressure)
                                # Local estimates and genuine sensor inputs only.
                                sensor_sample['rotation_local'] = motion.samples[-1]['rotation'].tolist()
                                sensor_sample['position_local_m'] = motion.samples[-1]['position'].tolist()
                                motion_trace.write(json.dumps(sensor_sample, allow_nan=False)+'\n')
                            geometry = guard.evaluate(data)
                            reject = latch.sample(geometry, count)
                            minimum = min(minimum, geometry['minimum_clearance_lower_bound_m'])
                            rotation = data.xmat[base].reshape(3,3)
                            tilt = math.acos(float(np.clip(rotation[2,2],-1,1)))
                            forces = np.asarray(native['body_contact_force_world_n'])
                            history.append(max(float(forces[tool,2]),0.))
                            causes = shared.termination_causes(forces, history, illegal, tilt)
                            causes['vd12_gun_exclusion'] = reject
                            axis = data.site_xmat[tcp].reshape(3,3)[:,0]
                            desired_axis = rotation @ shared.quaternion_matrix(pose[3:])[:,0]
                            row = dict(physics_step=count, time_s=count*.005, state_sample_time_s=count*.005,
                                native_force_sample_time_s=(count-1)*.005, joint_position18_rad=data.qpos[qids].tolist(),
                                joint_velocity18_radps=data.qvel[dofs].tolist(), joint_target18_rad=target.tolist(),
                                base_position_w_m=data.xpos[base].tolist(), base_quaternion_wxyz=data.xquat[base].tolist(),
                                tcp_position_w_m=data.site_xpos[tcp].tolist(), tcp_target_w_50hz_zoh_m=target_w.tolist(),
                                foot_force_world_n=forces[feet].tolist(), foot_position_world_m=data.xpos[feet].tolist(),
                                gun_axis_error_rad=math.acos(float(np.clip(axis@desired_axis,-1,1))),
                                geometry=geometry, geometry_latched=latch.violated, velocity_impulse=constraint.last, **native)
                            physics_log.write(json.dumps(row, allow_nan=False)+'\n')
                        if any(causes.values()):
                            reason = '+'.join(name for name,value in causes.items() if value)
                            terminal = dict(physics_step=count, causes=causes, first_geometry_failure=latch.first)
                            shared.save(output/'terminal_state.json', row)
                            break
                        if step % 50 == 0:
                            physics_log.flush()
                            actor_log.flush()
                            print('sim=%.2fs visual=%s seq=%s capture_age=%.3fs cmd=%s' % (
                                count*.005, args.vision_mode, status['source_sequence'], status['capture_age_s'],
                                np.round(base_command,3).tolist()), flush=True)
                            overlay = output/'latest_dual_overlay.png'
                            if overlay.is_file():
                                shutil.copyfile(overlay, output/('overlay_%.2fs.png' % (count*.005)))
                        if viewer:
                            with viewer.lock():
                                if spectator is not None:
                                    spectator.update(data)
                                viewer.cam.lookat[:] = data.xpos[base]+np.array((-.28,0.,.05))
                            viewer.sync()
                        # Wall-clock pacing is required even headless: do not
                        # accelerate simulation past the real perception process.
                        delay = .02-(time.monotonic()-tick_wall)
                        if delay > 0:
                            time.sleep(delay)
    except Exception as error:
        reason, error_text = 'runtime_error', repr(error)
        print(error_text, file=sys.stderr, flush=True)
    finally:
        active_end_wall = time.monotonic()
        captures.close()
        if motion_trace is not None:
            motion_trace.close()
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
        vision_log.close()
        if viewer:
            viewer.close()
        renderer.close()
        completed = reason == 'duration_complete'
        passed = completed and support_wait_steps == 0
        user_stopped = version == 14 and reason in (
            'interrupted_before_vision_ready', 'interrupted_or_viewer_closed')
        summary = dict(status=('completed_vision_sample' if passed else
                              'completed_with_support_wait' if completed else
                              'stopped_by_user' if user_stopped else 'rejected'), reason=reason,
            runtime_error=error_text, duration_s=count*.005, requested_duration_s=args.duration,
            run_mode='continuous_until_user_or_guard_stop' if args.duration == 0. else 'timed_sample',
            time_limit_s=None if args.duration == 0. else args.duration,
            physics_steps=count, actor_steps=actor_count, camera_pairs_published=frames,
            distinct_accepted_camera_pairs=new_measurements, warmup_last_rejection=warmup_rejection,
            measured_current_path_actor_steps=measured_steps,
            historical_path_actor_steps=memory_steps, estimated_target_actor_steps=estimated_steps,
            historical_tangent_estimated_actor_steps=historical_tangent_steps,
            directly_observed_target_actor_steps=directly_observed_steps,
            repeated_camera_pair_actor_steps=actor_count-new_measurements-support_wait_steps,
            rear_support_wait_actor_steps=support_wait_steps,
            rear_support_wait_events=support_wait_events,
            rear_support_wait_recoveries=support_wait_recoveries,
            maximum_rear_support_wait_wall_s=maximum_support_wait_s,
            maximum_held_setpoint_age_wall_s=maximum_held_setpoint_age_s,
            front_fit_fallback_actor_steps=front_fit_fallback_steps,
            front_fit_fallback_distinct_camera_pairs=len(front_fit_fallback_sequences),
            maximum_accepted_capture_age_s=maximum_capture_age,
            maximum_accepted_capture_age_sim_s=maximum_capture_sim_age,
            maximum_target_source_age_s=maximum_target_source_age,
            maximum_target_source_age_sim_s=maximum_target_source_sim_age,
            measurement_count_note='Current-path validity is not direct work-point visibility or image-only segmentation accuracy; memory and target estimates are counted separately.',
            active_wall_s=None if started_wall is None else active_end_wall-started_wall,
            real_time_factor=None if started_wall is None else count*.005/max(active_end_wall-started_wall, 1.e-9),
            minimum_clearance_lower_bound_m=minimum, terminal_event=terminal, automatic_resets=0,
            perception_mode=args.vision_mode, truth_command_fallback=False, simulation_only=True,
            motion_compensation=args.motion_compensation,
            visual_sample_passed=passed, real_robot_approved=False, default_actor_modified=False,
            note='Rendered images through perception, not real sensor validation. A completed sample is not statistical gait/vision acceptance.')
        shared.save(output/'velocity_constraint.json', constraint.summary())
        shared.save(output/'summary.json', summary)
        print(json.dumps(dict(output=str(output), **summary), indent=2), flush=True)
    if user_stopped:
        return 0
    return 0 if reason == 'duration_complete' else 2


if __name__ == '__main__':
    raise SystemExit(main())
