#!/usr/bin/env python3
"""Standalone, simulation-only PhysX command-response counterpart to MuJoCo.

Launch with PYTHONPATH=/home/dong/b2_z1_VD2/source/b2_z1 and the IsaacLab
interpreter. This does not create a training environment, preload AMP data,
render cameras, communicate with a robot, or change gains. The original VD1
actor receives ideal IMU, encoders, scheduled commands, frozen recorded tool
command and previous raw action only. Ground-truth evaluation is a separate
one-way sink. Every USD and log file is created in a unique output directory.
"""
from __future__ import annotations

import argparse
import ast
from datetime import datetime, timedelta, timezone
import gzip
import json
import math
from pathlib import Path
import signal
import sys
import tempfile

import numpy as np

from diagnose_vd1_base_response import (
    ACTOR_DT, CASES, DEFAULT_POLICY, DEFAULT_TRACE, POLICY_SHA256,
    TruthRecorder, command_at, load_tool_sample, sha256, write_json,
)

SOURCE_ROOT = Path('/home/dong/b2_z1_VD2/source/b2_z1')
VISION_ROOT = Path('/home/dong/b2_z1/scripts/vision')


def source_actor_joint_names():
    """Read the immutable contract literal without importing MuJoCo in Isaac."""
    path = Path(__file__).with_name('vd1_mujoco_contract.py')
    for node in ast.parse(path.read_text(encoding='utf-8')).body:
        if isinstance(node, ast.Assign) and any(isinstance(target, ast.Name) and target.id == 'ACTOR_JOINT_NAMES'
                                                for target in node.targets):
            names = ast.literal_eval(node.value)
            if isinstance(names, tuple) and len(names) == 18 and all(isinstance(name, str) for name in names):
                return names
    raise ValueError('Missing or invalid immutable VD1 joint contract')


def arguments():
    from isaaclab.app import AppLauncher
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--cases', nargs='+', choices=CASES, default=['straight', 'left', 'right', 'stop'])
    parser.add_argument('--speed', type=float, default=.4)
    parser.add_argument('--yaw-rate', type=float, default=.15)
    parser.add_argument('--policy', type=Path, default=DEFAULT_POLICY)
    parser.add_argument('--trace', type=Path, default=DEFAULT_TRACE)
    parser.add_argument('--sample-time', type=float, default=4.)
    parser.add_argument('--output-root', type=Path, default=Path('/dev/shm/vd1_isaac_response'))
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(headless=True)
    args = parser.parse_args()
    if not all(math.isfinite(x) for x in (args.duration, args.speed, args.yaw_rate, args.sample_time)):
        parser.error('All schedule numbers must be finite')
    if not 0 < args.duration <= 600 or not 0 < args.speed <= 1 or not 0 <= args.yaw_rate <= 1:
        parser.error('Invalid duration, speed or yaw-rate')
    if not args.headless or args.enable_cameras:
        parser.error('This diagnostic is headless and does not render cameras')
    return args, AppLauncher


class EvaluationSink:
    """Articulation ground truth is available here, never in actor construction."""
    def __init__(self, output):
        self.handle = gzip.open(str(output / 'evaluation_truth.jsonl.gz'), 'xt', compresslevel=3)
        self.rows = []
        self.first_position = None
        self.first_threshold = {str(value): None for value in (15, 30, 45, 60)}
        self.gyro_crosscheck_max_abs_error = 0.

    def write(self, robot, imu, step, sim_time_s, actor_ids):
        state = robot.data
        quat = state.root_link_quat_w[0].detach().cpu().numpy()
        w, x, y, z = quat
        roll = math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
        pitch = math.asin(float(np.clip(2 * (w*y - z*x), -1, 1)))
        yaw = math.atan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
        tilt = math.degrees(math.acos(float(np.clip(1 - 2*(x*x + y*y), -1, 1))))
        row = dict(step=step, sim_time_s=sim_time_s,
                   base_position_w=state.root_link_pos_w[0].detach().cpu().tolist(),
                   body_velocity_b=state.root_link_lin_vel_b[0].detach().cpu().tolist(),
                   body_angular_velocity_b=state.root_link_ang_vel_b[0].detach().cpu().tolist(),
                   base_com_velocity_b=state.root_com_lin_vel_b[0].detach().cpu().tolist(),
                   roll_rad=roll, pitch_rad=pitch, yaw_rad=yaw, tilt_deg=tilt,
                   applied_torque_estimate18=state.applied_torque[0, actor_ids].detach().cpu().tolist(),
                   computed_torque_estimate18=state.computed_torque[0, actor_ids].detach().cpu().tolist(),
                   joint_velocity18=state.joint_vel[0, actor_ids].detach().cpu().tolist(),
                   root_quaternion_wxyz=quat.tolist(), evaluation_only=True)
        gyro_error = float(np.max(np.abs(np.asarray(row['body_angular_velocity_b'])
                                        - imu.data.ang_vel_b[0].detach().cpu().numpy())))
        self.gyro_crosscheck_max_abs_error = max(self.gyro_crosscheck_max_abs_error, gyro_error)
        row['gyro_crosscheck_max_abs_error_radps'] = gyro_error
        self.handle.write(json.dumps(row, allow_nan=False) + '\n')
        if self.first_position is None:
            self.first_position = np.asarray(row['base_position_w'])
        self.rows.append(row)
        for threshold in self.first_threshold:
            if tilt >= float(threshold) and self.first_threshold[threshold] is None:
                self.first_threshold[threshold] = sim_time_s

    def summary(self, case, duration):
        result = TruthRecorder.summary(self, case, duration)
        result['velocity_frame'] = 'root link origin, expressed in root link axes (PhysX root_link_..._b)'
        # This standalone source check has an IMU but no contact sensor. The
        # shared MuJoCo summarizer uses missing-key fallbacks; those must not
        # turn an unobserved contact into a claimed absence of contact.
        result['contact_measurements_collected'] = False
        result['contact_measurement_status'] = 'unknown: no PhysX contact measurements collected'
        for key in ('first_nonfoot_ground_contact_time_s', 'first_gun_ground_contact_time_s',
                    'nonfoot_ground_contact_max_normal_force_N'):
            result.pop(key, None)
        return result


def sensor_snapshot(robot, imu, actor_ids):
    """The observation boundary: no root/world/TCP/velocity-truth reads."""
    quat = imu.data.quat_w[0].detach().cpu().numpy()
    w, x, y, z = quat
    norm = float(np.linalg.norm(quat))
    if abs(norm - 1.) > 1e-3:
        raise ValueError('IMU attitude must be a unit wxyz quaternion')
    return dict(gyro_b=imu.data.ang_vel_b[0].detach().cpu().numpy().copy(),
                gravity_b=np.asarray((2*(w*y-x*z), -2*(y*z+w*x), 2*(x*x+y*y)-1)),
                q=robot.data.joint_pos[0, actor_ids].detach().cpu().numpy().copy(),
                qd=robot.data.joint_vel[0, actor_ids].detach().cpu().numpy().copy())


def run_case(args, case, actor, tool_pose, sim, robot, imu, actor_ids, output, stopped):
    import torch
    from vd1_control import build_actor_observation
    output.mkdir()
    # Reinitialize PhysX between trials, then write the exact same source state.
    # There is deliberately no resetting on falls or between actor steps.
    sim.reset()
    root = robot.data.default_root_state.clone()
    robot.write_root_pose_to_sim(root[:, :7])
    robot.write_root_velocity_to_sim(root[:, 7:])
    robot.write_joint_state_to_sim(robot.data.default_joint_pos.clone(), robot.data.default_joint_vel.clone())
    robot.reset()
    imu.reset()
    robot.update(.005)
    imu.update(.005, force_recompute=True)
    default_q = robot.data.default_joint_pos[0, actor_ids].detach().cpu().numpy().copy()
    previous_action = np.zeros(18, dtype=np.float32)
    truth = EvaluationSink(output)
    reason, error, steps = 'duration_complete', None, 0
    try:
        with gzip.open(str(output / 'actor_trace.jsonl.gz'), 'xt', compresslevel=3) as handle:
            for step in range(int(math.ceil(args.duration / ACTOR_DT))):
                if stopped[0]:
                    reason = 'interrupted'
                    break
                elapsed = step * ACTOR_DT
                measured = sensor_snapshot(robot, imu, actor_ids)
                command = command_at(case, elapsed, args.speed, args.yaw_rate)
                observation = build_actor_observation(command, tool_pose, measured['gyro_b'],
                              measured['gravity_b'], measured['q'], measured['qd'], default_q, previous_action)
                with torch.inference_mode():
                    action = actor(torch.from_numpy(observation).unsqueeze(0)).numpy()[0]
                if action.shape != (18,) or not np.isfinite(action).all():
                    raise ValueError('Actor action must be finite18D')
                target = np.clip(default_q + .25 * action, -100., 100.)
                target_tensor = torch.as_tensor(target, dtype=torch.float32, device=sim.device).unsqueeze(0)
                robot.set_joint_position_target(target_tensor, joint_ids=actor_ids)
                row = dict(step=step, sim_time_s=elapsed, base_command_b=command.tolist(),
                           tool_pose_b=tool_pose.tolist(), observation=observation.tolist(), action=action.tolist(),
                           previous_action=previous_action.tolist(), joint_target=target.tolist(),
                           sensors={name: value.tolist() for name, value in measured.items()},
                           privileged_actor_inputs=[])
                for _ in range(4):
                    robot.write_data_to_sim()
                    sim.step(render=False)
                    robot.update(.005)
                    imu.update(.005)
                handle.write(json.dumps(row, allow_nan=False) + '\n')
                truth.write(robot, imu, step, (step+1) * ACTOR_DT, actor_ids)
                previous_action = action.copy()
                steps += 1
                if step == 0 or steps % 500 == 0:
                    print('[VD1 PhysX response] case=%s sim=%.2fs command=%s' %
                          (case, steps * ACTOR_DT, np.round(command, 3).tolist()), flush=True)
                if sensor_snapshot(robot, imu, actor_ids)['gravity_b'][2] > -.5:
                    reason = 'IMU_tilt_stop'
                    break
    except Exception as exc:
        reason, error = 'runtime_error', repr(exc)
        import traceback
        traceback.print_exc()
    finally:
        truth.handle.close()
        result = dict(case=case, actor_steps=steps, sim_time_s=steps * ACTOR_DT,
                      stop_reason=reason, runtime_error=error,
                      evaluation=truth.summary(case, args.duration))
        write_json(output / 'summary.json', result)
    print('[VD1 PhysX response] case=%s result=%s sim=%.2fs' % (case, reason, steps * ACTOR_DT), flush=True)
    return result


def run(args, app):
    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.assets import Articulation
    from isaaclab.sensors import Imu, ImuCfg
    from b2_z1.assets.unitree import B2_Z1_GUN_CFG
    import b2_z1.assets.unitree as source_module
    ACTOR_JOINT_NAMES = source_actor_joint_names()

    if Path(source_module.__file__).resolve().parent.parent.parent != SOURCE_ROOT.resolve():
        raise ValueError('Wrong source package; set PYTHONPATH=' + str(SOURCE_ROOT))
    torch.set_num_threads(1)
    torch.manual_seed(0)
    if sha256(args.policy) != POLICY_SHA256:
        raise ValueError('VD1 actor policy hash does not match source')
    actor = torch.jit.load(str(args.policy), map_location='cpu').eval()
    with torch.inference_mode():
        if actor(torch.zeros(1, 70)).shape != (1, 18):
            raise ValueError('VD1 actor must be70in18out')
    sample = load_tool_sample(args.trace, args.sample_time)
    args.output_root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    output = Path(tempfile.mkdtemp(prefix='run_' + stamp + '_', dir=str(args.output_root)))
    print('[VD1 PhysX response] output: ' + str(output), flush=True)
    material = sim_utils.RigidBodyMaterialCfg(static_friction=1., dynamic_friction=1.,
                friction_combine_mode='multiply', restitution_combine_mode='multiply')
    sim_cfg = sim_utils.SimulationCfg(dt=.005, device=args.device, physics_material=material)
    # These are the source training scene's effective solver settings, not tuning.
    sim_cfg.physx.solver_type = 1
    sim_cfg.physx.max_position_iteration_count = 4
    sim_cfg.physx.max_velocity_iteration_count = 0
    sim_cfg.physx.bounce_threshold_velocity = .5
    sim = sim_utils.SimulationContext(sim_cfg)
    ground = sim_utils.GroundPlaneCfg(physics_material=material)
    ground.func('/World/Ground', ground)
    cfg = B2_Z1_GUN_CFG.copy()
    cfg.prim_path = '/World/Robot'
    cfg.spawn.usd_dir = str(output / 'usd_cache')
    cfg.spawn.usd_file_name = 'original_b2_z1_gun.usd'
    cfg.spawn.force_usd_conversion = True
    robot = Articulation(cfg)
    imu = Imu(ImuCfg(prim_path='/World/Robot/base_link', update_period=.005, debug_vis=False))
    sim.reset()
    actor_ids = [robot.joint_names.index(name) for name in ACTOR_JOINT_NAMES]
    if len(robot.joint_names) != 18 or len(set(actor_ids)) != 18:
        raise ValueError('Original source must expose exactly18 named joints')

    def aslist(value):
        return value.detach().cpu().tolist() if isinstance(value, torch.Tensor) else value

    actuator_manifest = {}
    for name, actuator in robot.actuators.items():
        actuator_manifest[name] = dict(class_name=type(actuator).__name__,
             joint_indices=aslist(actuator.joint_indices),
             joint_names=actuator.joint_names,
             stiffness=aslist(actuator.stiffness), damping=aslist(actuator.damping),
             effort_limit=aslist(actuator.effort_limit), velocity_limit=aslist(actuator.velocity_limit),
             effort_limit_sim=actuator.cfg.effort_limit_sim,
             velocity_limit_sim=actuator.cfg.velocity_limit_sim,
             min_delay=getattr(actuator.cfg, 'min_delay', None), max_delay=getattr(actuator.cfg, 'max_delay', None))
    sources = [Path(__file__).resolve(), Path(__file__).with_name('diagnose_vd1_base_response.py'),
               Path(__file__).with_name('vd1_mujoco_contract.py'),
               VISION_ROOT / 'vd1_control.py', Path(source_module.__file__), Path(cfg.spawn.asset_path),
               SOURCE_ROOT / 'b2_z1/tasks/manager_based/b2_z1/b2_z1_env_cfg.py']
    write_json(output / 'manifest.json', dict(simulation_only=True, robot_communication=False,
        training_environment_created=False, AMP_preload=False, rendering=False, physics_engine='PhysX',
        source_package=str(source_module.__file__), policy=str(args.policy), policy_sha256=POLICY_SHA256,
        trace=str(args.trace), trace_sha256=sha256(args.trace), recorded_sample=sample,
        actor_hz=50, pd_hz=200, physics_dt=.005, duration_per_case_s=args.duration, cases=args.cases,
        speed_mps=args.speed, yaw_rate_radps=args.yaw_rate, ground='flat plane z=0',
        robot_state_reset='full PhysX reset and exact source default root/joints between cases, no autoreset',
        initial_root_state=aslist(robot.data.default_root_state),
        joint_names_actor_order=list(ACTOR_JOINT_NAMES), joint_names_physx=robot.joint_names,
        actor_to_physx_joint_indices=actor_ids, default_joint_position_actor_order=aslist(robot.data.default_joint_pos[0, actor_ids]),
        body_names=robot.body_names, effective_default_mass=aslist(robot.data.default_mass),
        effective_default_inertia=aslist(robot.data.default_inertia),
        effective_body_com_position_b=aslist(robot.data.body_com_pos_b),
        effective_body_com_quaternion_b=aslist(robot.data.body_com_quat_b),
        effective_sim_stiffness=aslist(robot.data.joint_stiffness),
        effective_sim_damping=aslist(robot.data.joint_damping),
        effective_sim_effort_limit=aslist(robot.data.joint_effort_limits),
        effective_sim_velocity_limit=aslist(robot.data.joint_vel_limits),
        actuators=actuator_manifest,
        source_articulation_solver_requested=dict(position=8, velocity=4),
        effective_scene_solver=dict(type='TGS', max_position_iterations=4, max_velocity_iterations=0),
        actor_inputs=['scheduled base command', 'frozen recorded tool command', 'encoder jointpos and jointvel',
                      'ideal IMU gyro and attitude-derived projected gravity', 'previous raw action'],
        privileged_actor_inputs=[], evaluation_sink='separate evaluation_truth.jsonl.gz; never returned to actor',
        evaluation_velocity_reference='root link origin, expressed in root link axes; COM velocity logged separately',
        schedule=dict(speed_ramp_s=2., turn_start_s=10., turn_end_s=20., turn_transition_s=.5,
                      stop_start_s=15., stop_end_s=17., restart_start_s=22., restart_end_s=24.),
        torch_version=torch.__version__, source_hashes={str(path): sha256(path) for path in sources}))
    stopped = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.__setitem__(0, True))
    results = []
    for case in args.cases:
        if stopped[0] or not app.is_running():
            break
        results.append(run_case(args, case, actor, np.asarray(sample['tool_pose_b']), sim, robot,
                                imu, actor_ids, output / case, stopped))
    write_json(output / 'summary.json', dict(cases=results, requested_cases=args.cases, interrupted=stopped[0]))
    return int(any(row['runtime_error'] for row in results))


def main():
    args, launcher = arguments()
    sys.path.insert(0, str(VISION_ROOT))
    app_launcher = launcher(args)
    app = app_launcher.app
    try:
        return run(args, app)
    except Exception:
        # Kit's fast close may exit the process before Python emits a traceback.
        import traceback
        traceback.print_exc()
        sys.stdout.flush()
        sys.stderr.flush()
        return 1
    finally:
        # Follow IsaacLab's headless context cleanup: closing a playing stage
        # without stopping can hang Kit indefinitely on this installation.
        from isaaclab.sim import SimulationContext
        sim = SimulationContext.instance()
        if sim is not None:
            # Unsubscribe IsaacLab's standalone STOP->render-until-playing
            # callback before stopping; this diagnostic intentionally exits.
            sim.clear_instance()
            sim.stop()
            sim.clear_all_callbacks()
        app.close(wait_for_replicator=False)


if __name__ == '__main__':
    sys.exit(main())
