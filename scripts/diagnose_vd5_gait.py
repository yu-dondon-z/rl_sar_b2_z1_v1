#!/usr/bin/env python3
"""Matched VD5 fixed-command, fixed-body-TCP diagnostics in MuJoCo/PhysX.

No cameras, robot IO, gain changes, action filtering, or truth fed to actor.
Physics truth is written separately at 200 Hz. Existing deployments are untouched.
"""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import time

import numpy as np

sys.path.insert(0, '/home/dong/b2_z1/scripts/vision')
from vd5_mujoco_policy import VD5_POLICY, VD5_POLICY_SHA256
from vd5_mujoco_control import (startup_fraction, normalized_lerp_quaternion,
                               quaternion_from_euler_xyz, quaternion_matrix)
from vd1_mujoco_scene import (EXPECTED_JOINT_NAMES, DEFAULT_JOINT_POSITION, VD3_TCP_POSITION,
                              VD3_TCP_ROTATION, VD3_URDF_PATH)
from vd5_gait_metrics import summarize

DT = .005


def save(path, obj):
    path.write_text(json.dumps(obj, indent=2, allow_nan=False), encoding='utf-8')


def parse():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--engine', choices=['mujoco', 'isaac'], required=True)
    p.add_argument('--duration', type=float, default=60.)
    p.add_argument('--speed', type=float, default=.2)
    p.add_argument('--work-distance', type=float, default=.8)
    p.add_argument('--target-z', type=float, default=-.54)
    p.add_argument('--roll-torque-nm', type=float, default=0.,
                   help='MuJoCo-only external torque around the moving base roll axis; default off')
    p.add_argument('--roll-pulse-start-s', type=float, default=10.)
    p.add_argument('--roll-pulse-duration-s', type=float, default=.12)
    p.add_argument('--checkpoint', type=Path, default=None,
                   help='Evaluate a local VD5 Startup/VD6/VD7/VD8/VD9 checkpoint without changing the deployed actor')
    p.add_argument('--output-root', type=Path, default=Path('/home/dong/rl_sar_b2_z1/outputs/VD5_GAIT_COMPARE'))
    launcher = None
    preliminary, _unknown = p.parse_known_args()
    if preliminary.engine == 'isaac':
        from isaaclab.app import AppLauncher
        AppLauncher.add_app_launcher_args(p)
        p.set_defaults(headless=True)
        launcher = AppLauncher
    a = p.parse_args()
    if launcher and (not a.headless or a.enable_cameras):
        p.error('Matched diagnostic runs headless, without camera rendering')
    if not (5 < a.duration <= 120 and 0 <= a.speed <= .4 and .6 <= a.work_distance <= .9
            and -.65 <= a.target_z <= -.4):
        p.error('Invalid diagnostic schedule')
    if (not math.isfinite(a.roll_torque_nm) or not 0 <= a.roll_torque_nm <= 20
            or not math.isfinite(a.roll_pulse_start_s) or not math.isfinite(a.roll_pulse_duration_s)
            or a.roll_pulse_start_s < 5 or a.roll_pulse_duration_s <= 0):
        p.error('Invalid roll pulse parameters')
    if a.roll_torque_nm:
        if a.engine != 'mujoco' or a.roll_pulse_start_s + a.roll_pulse_duration_s + 3 > a.duration:
            p.error('Roll pulse requires MuJoCo and at least three seconds of recovery')
        for value in (a.roll_pulse_start_s, a.roll_pulse_duration_s):
            if abs(value / DT - round(value / DT)) > 1e-7:
                p.error('Roll pulse times must align to the 200 Hz physics step')
    return a, launcher


class Run:
    def __init__(self, args):
        import torch
        from vd5_visual_control import encoder_tcp_pose
        torch.set_num_threads(1)
        policy_path = args.checkpoint.resolve() if args.checkpoint else VD5_POLICY
        policy_sha = hashlib.sha256(policy_path.read_bytes()).hexdigest()
        experiment, iteration = 'b2_z1_vd5_startup_amp', 4498
        if args.checkpoint:
            checkpoint = torch.load(str(policy_path), map_location='cpu', weights_only=False)
            experiment, iteration = checkpoint.get('training_experiment_name'), checkpoint.get('iter')
            if experiment not in ('b2_z1_vd5_startup_amp', 'b2_z1_vd6_smooth_amp',
                                  'b2_z1_vd7_tcp_amp', 'b2_z1_vd8_roll_landing_amp',
                                  'b2_z1_vd9_physics_rate_amp'):
                raise ValueError('Diagnostic requires a VD5 Startup or VD6-VD9 checkpoint')
            if 'obs_norm_state_dict' in checkpoint:
                raise ValueError('Diagnostic expects no observation normalization')
            self.actor = torch.nn.Sequential(torch.nn.Linear(70,512), torch.nn.ELU(),
                torch.nn.Linear(512,256), torch.nn.ELU(), torch.nn.Linear(256,128),
                torch.nn.ELU(), torch.nn.Linear(128,18)).eval()
            self.actor.load_state_dict({k[len('actor.'):]:v for k,v in checkpoint['model_state_dict'].items()
                                       if k.startswith('actor.')}, strict=True)
        else:
            if policy_sha != VD5_POLICY_SHA256:
                raise ValueError('Pinned VD5 policy changed')
            self.actor = torch.jit.load(str(VD5_POLICY), map_location='cpu').eval()
        self.args = args
        args.output_root.mkdir(parents=True, exist_ok=True)
        self.output = Path(tempfile.mkdtemp(prefix=args.engine+'_'+time.strftime('%Y%m%d_%H%M%S')+'_', dir=str(args.output_root)))
        self.default = np.array([DEFAULT_JOINT_POSITION[n] for n in EXPECTED_JOINT_NAMES])
        self.initial = encoder_tcp_pose(self.default[[8, 13, 14, 15, 16, 17]],
            VD3_URDF_PATH, VD3_TCP_POSITION, VD3_TCP_ROTATION)
        self.final = np.r_[-args.work_distance, 0., args.target_z,
            quaternion_from_euler_xyz(0., math.pi/2-.45, math.pi)]
        self.previous = np.zeros(18, dtype=np.float32)
        self.physics, self.actions = [], []
        self.fp = gzip.open(str(self.output/'physics_truth.jsonl.gz'), 'xt', compresslevel=3)
        self.fa = gzip.open(str(self.output/'actor_trace.jsonl.gz'), 'xt', compresslevel=3)
        self.fj = gzip.open(str(self.output/'joint_response.jsonl.gz'), 'xt', compresslevel=3)
        self.tool = self.final.copy()
        save(self.output/'manifest.json', dict(engine=args.engine, policy=str(policy_path),
            policy_sha256=policy_sha, checkpoint_iteration=iteration, training_experiment=experiment,
            physics_dt=DT, actor_dt=.02, speed=args.speed,
            fixed_body_tool_target=self.final.tolist(), initial_body_tool_pose=np.r_[self.initial[0], self.initial[1]].tolist(),
            ramp_s=2., initial_base_height_m=.50, gains_kp_kd=[160.,5.],
            gun_added_mass_kg=0., observation_noise=False, domain_randomization=False,
            previous_action='raw policy output', camera_inputs=False, privileged_actor_inputs=[],
            simulation_only=True, robot_io=False,
            external_roll_pulse=dict(torque_nm=args.roll_torque_nm,
                start_s=args.roll_pulse_start_s, duration_s=args.roll_pulse_duration_s,
                applied_to='base_link xfrc_applied, torque along instantaneous body x axis in world coordinates',
                actor_inputs_overwritten=False),
            target_note='Body-relative target deliberately fixed after ramp, independent of base/world truth.'))
        print('VD5 gait output: '+str(self.output), flush=True)

    def control(self, t, sensor):
        import torch
        from vd1_control import build_actor_observation
        f = startup_fraction(t)
        self.tool = np.r_[self.initial[0]+f*(self.final[:3]-self.initial[0]),
            normalized_lerp_quaternion(self.initial[1], self.final[3:], f)]
        command = np.array([self.args.speed*f, 0., 0.])
        obs = build_actor_observation(command, self.tool, sensor['gyro_b'], sensor['gravity_b'],
            sensor['q'], sensor['qd'], self.default, self.previous)
        with torch.inference_mode():
            action = self.actor(torch.from_numpy(obs).unsqueeze(0)).numpy()[0]
        if action.shape != (18,) or not np.isfinite(action).all():
            raise ValueError('Invalid action')
        target = self.default+.25*action
        row = dict(time_s=t, base_command_b=command.tolist(), tool_pose_b=self.tool.tolist(),
            joint_target18=target.tolist(), action=action.tolist(), observation=obs.tolist(),
            joint_position18=sensor['q'].tolist(), joint_velocity18=sensor['qd'].tolist(),
            gyro_b=sensor['gyro_b'].tolist(), gravity_b=sensor['gravity_b'].tolist())
        self.fa.write(json.dumps(row)+'\n')
        self.actions.append(row)
        self.previous = action.copy()
        return target

    def record(self, row):
        row['tool_target_b'] = self.tool[:3].tolist()
        row['evaluation_only'] = True
        self.fp.write(json.dumps(row, allow_nan=False)+'\n')
        self.physics.append(row)
        if len(self.physics) % 2000 == 0:
            print('VD5 gait %.1f s tilt %.2f deg' % (row['time_s'], row['tilt_deg']), flush=True)

    def finish(self, reason):
        self.fp.close()
        self.fa.close()
        self.fj.close()
        result = dict(engine=self.args.engine, reason=reason, physics_steps=len(self.physics),
            actor_steps=len(self.actions), metrics=summarize(self.physics, self.actions))
        save(self.output/'summary.json', result)
        print(json.dumps(result, indent=2), flush=True)

    def record_joints(self, row):
        row.update(evaluation_only=True, actor_inputs_overwritten=False,
                   actor_target_time_s=math.floor((row['step_start_s']+1e-8)/.02)*.02)
        self.fj.write(json.dumps(row, allow_nan=False)+'\n')


def mujoco_run(run):
    import mujoco
    from vd1_mujoco_contract import VD1MujocoAdapter
    from vd1_mujoco_scene import build_scene, validate_vd3_model
    from vd1_mujoco_velocity_impulse import VelocityImpulseConstraints
    scene = build_scene(0., Path(tempfile.mkdtemp(prefix='vd5_gait_', dir='/dev/shm')),
        timestep=DT, tool_markers=False, road_surface='tape', robot_version='vd3')
    model = mujoco.MjModel.from_xml_path(str(scene))
    save(run.output/'asset_audit.json', validate_vd3_model(model))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('vd1_default').id)
    data.qpos[2] = .50
    mujoco.mj_forward(model, data)
    adapter = VD1MujocoAdapter(model, physx_implicit_position=True)
    np.testing.assert_allclose(adapter.default_q18, run.default, atol=1e-6)
    constraint = VelocityImpulseConstraints(model, adapter.joint_ids18, adapter.velocity_limit18)
    dofs = model.jnt_dofadr[adapter.joint_ids18]
    qpos_ids = model.jnt_qposadr[adapter.joint_ids18]
    from vd1_mujoco_contract import ACTOR_JOINT_NAMES
    save(run.output/'joint_diagnostic_schema.json', dict(
        version=1, engine='mujoco', joint_order=list(ACTOR_JOINT_NAMES), dt_s=DT,
        state_phase='position/velocity at step end after velocity projection',
        force_phase='native mj_step force buffers at step start, saved BEFORE projection/mj_forward',
        drive_effort_source='MjData.actuator_force indexed by named actuator; engine evaluated force',
        drive_effort_note='Engine force, not adapter PD estimate. Implicitfast velocity linearization means this is not a separately measured step-average delivered torque.',
        native_constraint_source='MjData.qfrc_constraint at native solve; excludes split speed impulse',
        speed_impulse_source='VelocityImpulseConstraints.last.generalized_impulse; Nm*s, not drive torque',
        effort_limit18_nm=adapter.effort_limit18.tolist(), velocity_limit18_radps=adapter.velocity_limit18.tolist(),
        source_gains=[160.,5.], numerical_arm_kd=adapter.arm_discrete_kd18.tolist(),
        joint_position_limits_rad=model.jnt_range[adapter.joint_ids18].tolist()))
    feet = [model.body(n+'_foot').id for n in ('FL','FR','RL','RR')]
    base, tcp = model.body('base_link').id, model.site('tool_tcp').id
    target, reason = run.default, 'duration_complete'
    for tick in range(round(run.args.duration/DT)):
        if tick % 4 == 0:
            sensor = adapter.read_proprioception(data)
            target = run.control(tick*DT, sensor)
        pulse = run.args.roll_torque_nm if (run.args.roll_pulse_start_s / DT <= tick
            < (run.args.roll_pulse_start_s + run.args.roll_pulse_duration_s) / DT) else 0.
        # MuJoCo's applied wrench is world-frame; derive its direction from the
        # physical base orientation, leaving IMU and encoder reads untouched.
        applied_torque_w = pulse * data.xmat[base].reshape(3, 3)[:, 0].copy()
        data.xfrc_applied[base, 3:6] = applied_torque_w
        q_before, qd_before = data.qpos[qpos_ids].copy(), data.qvel[dofs].copy()
        adapter.apply_targets(data, target)
        native = {}
        def snapshot(native_data):
            native.update(drive_effort18_nm=native_data.actuator_force[adapter.actuator_ids18].copy().tolist(),
                joint_native_constraint_effort18_nm=native_data.qfrc_constraint[dofs].copy().tolist(),
                velocity_before_speed_projection18_radps=native_data.qvel[dofs].copy().tolist(),
                native_constraint_type=native_data.efc_type[:native_data.nefc].copy().tolist(),
                native_constraint_id=native_data.efc_id[:native_data.nefc].copy().tolist(),
                native_constraint_force=native_data.efc_force[:native_data.nefc].copy().tolist(),
                native_constraint_state=native_data.efc_state[:native_data.nefc].copy().tolist())
        constraint.step(data, evaluation_observer=snapshot)
        mujoco.mj_forward(model, data)
        impulse = np.asarray(constraint.last['generalized_impulse'])[dofs]
        qd_after = data.qvel[dofs].copy()
        run.record_joints(dict(step_start_s=tick*DT, time_s=(tick+1)*DT,
            joint_target18_rad=target.tolist(), joint_position_before18_rad=q_before.tolist(),
            joint_velocity_before18_radps=qd_before.tolist(),
            joint_position18_rad=data.qpos[qpos_ids].tolist(), joint_velocity18_radps=qd_after.tolist(),
            speed_constraint_impulse18_nms=impulse.tolist(), speed_constraint_active18=(np.abs(impulse)>1e-10).tolist(),
            speed_constraint_delta_velocity18_radps=(qd_after-np.asarray(native['velocity_before_speed_projection18_radps'])).tolist(),
            drive_effort_at_limit18=(np.abs(native['drive_effort18_nm'])>=adapter.effort_limit18-1e-5).tolist(),
            speed_near_limit18=(np.abs(qd_after)>=.99*adapter.velocity_limit18).tolist(),
            **native))
        force = np.zeros((4,3))
        for i in range(data.ncon):
            contact = data.contact[i]
            b1, b2 = model.geom_bodyid[[contact.geom1, contact.geom2]]
            f = np.zeros(6)
            mujoco.mj_contactForce(model, data, i, f)
            fw = contact.frame.reshape(3,3).T@f[:3]
            for j, foot in enumerate(feet):
                force[j] += (int(b2 == foot)-int(b1 == foot))*fw
        def origin_velocity(body):
            # BODY objectVelocity uses inertial axes/origin; use the link origin
            # explicitly so this matches PhysX body_link_lin_vel_w.
            jac = np.zeros((3,model.nv))
            mujoco.mj_jac(model, data, jac, None, data.xpos[body], body)
            return jac@data.qvel
        velocities = [origin_velocity(foot).tolist() for foot in feet]
        rotation = data.xmat[base].reshape(3,3)
        base_gyro_b = adapter.read_proprioception(data)['gyro_b']
        run.record(dict(time_s=(tick+1)*DT, base_position_w=data.xpos[base].tolist(),
            base_velocity_b=(rotation.T@origin_velocity(base)).tolist(), tilt_deg=math.degrees(math.acos(np.clip(rotation[2,2],-1,1))),
            base_roll_rad=math.atan2(rotation[2,1], rotation[2,2]),
            base_angular_velocity_b=base_gyro_b.tolist(),
            external_roll_torque_w_nm=applied_torque_w.tolist(),
            tcp_position_w=data.site_xpos[tcp].tolist(),
            tcp_position_b=(rotation.T@(data.site_xpos[tcp]-data.xpos[base])).tolist(),
            foot_position_w=data.xpos[feet].tolist(), foot_velocity_w=velocities, foot_force_w=force.tolist()))
        if run.physics[-1]['tilt_deg'] > 60:
            reason = 'tilt_over_60deg'
            break
    save(run.output/'velocity_constraint.json', constraint.summary())
    return reason


def isaac_run(run, args):
    import torch
    import isaaclab.sim as su
    from isaaclab.assets import Articulation
    from isaaclab.sensors import Imu, ImuCfg, ContactSensor, ContactSensorCfg
    from b2_z1.assets.unitree import B2_Z1_GUN_CFG
    from b2_z1.assets.vd3_runtime_contract import audit_vd3_runtime_asset
    from diagnose_vd1_isaac_response import sensor_snapshot
    material = su.RigidBodyMaterialCfg(static_friction=1., dynamic_friction=1.,
        friction_combine_mode='multiply', restitution_combine_mode='multiply')
    scfg = su.SimulationCfg(dt=DT, device=args.device, physics_material=material)
    scfg.physx.solver_type = 1
    scfg.physx.max_position_iteration_count = 4
    scfg.physx.max_velocity_iteration_count = 0
    scfg.physx.bounce_threshold_velocity = .5
    sim = su.SimulationContext(scfg)
    ground = su.GroundPlaneCfg(physics_material=material)
    ground.func('/World/Ground', ground)
    cfg = B2_Z1_GUN_CFG.copy()
    cfg.prim_path = '/World/Robot'
    cfg.init_state.pos = (0.,0.,.50)
    cfg.spawn.usd_dir = str(run.output/'usd_cache')
    cfg.spawn.force_usd_conversion = True
    robot = Articulation(cfg)
    imu = Imu(ImuCfg(prim_path='/World/Robot/base_link', update_period=DT))
    contact = ContactSensor(ContactSensorCfg(prim_path='/World/Robot/.*', update_period=DT))
    sim.reset()
    robot.update(DT)
    imu.update(DT, force_recompute=True)
    contact.update(DT, force_recompute=True)
    ids = [robot.joint_names.index(n) for n in EXPECTED_JOINT_NAMES]
    feet = [robot.body_names.index(n+'_foot') for n in ('FL','FR','RL','RR')]
    cfeet = [contact.body_names.index(n+'_foot') for n in ('FL','FR','RL','RR')]
    end = robot.body_names.index('link06')
    np.testing.assert_allclose(robot.data.default_joint_pos[0,ids].cpu().numpy(), run.default, atol=1e-6)
    save(run.output/'asset_audit.json', audit_vd3_runtime_asset(robot))
    view = robot.root_physx_view
    # Runtime getters: do NOT substitute Isaac Lab's applied_torque estimate.
    effort_limits = view.get_dof_max_forces()[0,ids].cpu().numpy().copy()
    speed_limits = view.get_dof_max_velocities()[0,ids].cpu().numpy().copy()
    save(run.output/'joint_diagnostic_schema.json', dict(
        version=1, engine='isaac', joint_order=list(EXPECTED_JOINT_NAMES), dt_s=DT,
        state_phase='position/velocity after completed PhysX step',
        force_phase='PhysX runtime getter after completed step',
        drive_effort_source=None,
        drive_effort_unavailable_reason='Installed tensor API exposes user actuation and projected incoming joint effort, not isolated implicit drive torque. No PD estimate is substituted.',
        joint_solver_effort_source='root_physx_view.get_dof_projected_joint_forces: active component of incoming joint wrench; may include drive, friction and limits',
        user_actuation_source='root_physx_view.get_dof_actuation_forces: user-applied force, not implicit drive',
        speed_constraint_source=None,
        speed_constraint_note='Exact solver multiplier/activity unavailable. speed_near_limit18 is proximity only, not proof of constraint activity.',
        effort_limit18_nm=effort_limits.tolist(), velocity_limit18_radps=speed_limits.tolist(),
        runtime_stiffness18=view.get_dof_stiffnesses()[0,ids].cpu().tolist(),
        runtime_damping18=view.get_dof_dampings()[0,ids].cpu().tolist(),
        joint_position_limits_rad=view.get_dof_limits()[0,ids].cpu().tolist()))
    target, reason = run.default, 'duration_complete'
    for tick in range(round(run.args.duration/DT)):
        if tick % 4 == 0:
            target = run.control(tick*DT, sensor_snapshot(robot, imu, ids))
            robot.set_joint_position_target(torch.as_tensor(target, dtype=torch.float32, device=sim.device)[None], joint_ids=ids)
        q_before = robot.data.joint_pos[0,ids].cpu().numpy().copy()
        qd_before = robot.data.joint_vel[0,ids].cpu().numpy().copy()
        robot.write_data_to_sim()
        sim.step(render=False)
        robot.update(DT)
        imu.update(DT)
        contact.update(DT)
        state = robot.data
        qd_after = state.joint_vel[0,ids].cpu().numpy().copy()
        run.record_joints(dict(step_start_s=tick*DT, time_s=(tick+1)*DT,
            joint_target18_rad=target.tolist(), joint_position_before18_rad=q_before.tolist(),
            joint_velocity_before18_radps=qd_before.tolist(),
            joint_position18_rad=state.joint_pos[0,ids].cpu().tolist(), joint_velocity18_radps=qd_after.tolist(),
            drive_effort18_nm=None, drive_effort_at_limit18=None,
            joint_solver_effort18_nm=view.get_dof_projected_joint_forces()[0,ids].cpu().tolist(),
            user_actuation_effort18_nm=view.get_dof_actuation_forces()[0,ids].cpu().tolist(),
            speed_constraint_impulse18_nms=None, speed_constraint_active18=None,
            speed_near_limit18=(np.abs(qd_after)>=.99*speed_limits).tolist(),
            speed_limit_exceeded18=(np.abs(qd_after)>speed_limits+1e-4).tolist()))
        rotation = quaternion_matrix(state.root_link_quat_w[0].cpu().numpy())
        tip = (state.body_link_pos_w[0,end].cpu().numpy()+
            quaternion_matrix(state.body_link_quat_w[0,end].cpu().numpy())@np.array(VD3_TCP_POSITION))
        pos = state.root_link_pos_w[0].cpu().numpy()
        run.record(dict(time_s=(tick+1)*DT, base_position_w=pos.tolist(),
            base_velocity_b=state.root_link_lin_vel_b[0].cpu().tolist(),
            tilt_deg=math.degrees(math.acos(np.clip(rotation[2,2],-1,1))),
            tcp_position_w=tip.tolist(), tcp_position_b=(rotation.T@(tip-pos)).tolist(),
            foot_position_w=state.body_link_pos_w[0,feet].cpu().tolist(),
            foot_velocity_w=state.body_link_lin_vel_w[0,feet].cpu().tolist(),
            foot_force_w=contact.data.net_forces_w[0,cfeet].cpu().tolist()))
        if run.physics[-1]['tilt_deg'] > 60:
            reason = 'tilt_over_60deg'
            break
    return reason


def main():
    args, launcher = parse()
    app = launcher(args).app if launcher else None
    run = None
    try:
        run = Run(args)
        reason = isaac_run(run, args) if app else mujoco_run(run)
        run.finish(reason)
    except BaseException as exc:
        if run:
            run.finish('error: '+repr(exc))
        import traceback
        traceback.print_exc()
        sys.stderr.flush()
        raise
    finally:
        if app:
            from isaaclab.sim import SimulationContext
            sim = SimulationContext.instance()
            if sim:
                sim.clear_instance()
                sim.stop()
                sim.clear_all_callbacks()
            app.close(wait_for_replicator=False)


if __name__ == '__main__':
    main()
