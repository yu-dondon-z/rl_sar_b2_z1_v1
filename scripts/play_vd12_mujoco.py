#!/usr/bin/env python3
"""Isolated VD12 simulation-only rollout, never the default robot actor.

Nominal reset, straight truth-generated world seam (not camera perception).
The shared training keep-out is sampled at 200 Hz, latched within four physics
steps and rejected at the original 50 Hz episode boundary without autoreset.
"""
from __future__ import annotations

import argparse
from collections import deque
import gzip
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
sys.path.insert(0, str(ROOT / 'scripts'))
import diagnose_vd11_gait as shared
from vd12_geometry_guard import MujocoGunGeometryGuard, ASSETS, URDF, load_pure_module

DEFAULT_CHECKPOINT = Path('/home/dong/b2_z1_VD2/logs/rsl_rl/b2_z1_vd12_geometry_guard_amp/2026-11-14_15-07-38_vd12_geometry_guard_long_20261114/model_5999.pt')
URDF_SHA256 = shared.VD10_URDF_SHA256
TCP_POSITION, TCP_ROTATION = shared.VD10_TCP_POSITION, shared.VD10_TCP_ROTATION
ROBOT_VERSION, WORK_DISTANCE_M = 'vd10', .8


def load_yaml(path):
    import yaml
    class SnapshotLoader(yaml.SafeLoader):
        pass
    def tagged(loader, suffix, node):
        if isinstance(node, yaml.MappingNode):
            return loader.construct_mapping(node)
        if isinstance(node, yaml.SequenceNode):
            return loader.construct_sequence(node)
        return loader.construct_scalar(node)
    SnapshotLoader.add_multi_constructor('tag:yaml.org,2002:python/', tagged)
    return yaml.load(Path(path).read_text(), Loader=SnapshotLoader)


def validate_checkpoint(checkpoint, agent, saved, profile, *,
                        expected='b2_z1_vd12_geometry_guard_amp',
                        startup_profile=None, work_distance=.8):
    if checkpoint.get('training_experiment_name') != expected or agent.get('experiment_name') != expected:
        raise ValueError('Requires a real VD12 checkpoint and saved VD12 agent')
    iteration = checkpoint.get('iter')
    if isinstance(iteration, bool) or not isinstance(iteration, int) or iteration < 0:
        raise ValueError('Invalid saved iteration')
    if (not profile or profile.get('version') != 1 or checkpoint.get('vd12_geometry_profile') != profile
            or saved.get('vd12_geometry_profile') != profile):
        raise ValueError('Checkpoint, saved environment and current geometry source differ')
    if ('obs_norm_state_dict' in checkpoint or agent.get('empirical_normalization') is not False
            or 'clip_actions' not in agent or agent['clip_actions'] is not None):
        raise ValueError('Requires unchanged unnormalized/unclipped actor')
    policy = agent.get('policy', {})
    if policy.get('activation') != 'elu' or policy.get('actor_hidden_dims') != [512, 256, 128]:
        raise ValueError('Unexpected actor architecture')
    state = checkpoint.get('vd5_curriculum_state', {})
    if (state.get('version') != 1 or state.get('startup_profile') != (shared.PROFILE if startup_profile is None else startup_profile)
            or state.get('ground_blend') != 1. or state.get('ground_transition_steps') != 20000
            or state.get('seam_trailing_distance_m') != work_distance):
        raise ValueError('This entry supports only the saved completed .80m ground curriculum and exact startup')
    return state


def validate_saved_settings(saved, *, command_class='VD10SafeSeamCommand'):
    """Reject silent changes to the subset implemented by this adapter."""
    if saved.get('sim', {}).get('dt') != .005 or saved.get('decimation') != 4:
        raise ValueError('Saved integration/control clock differs')
    if saved.get('episode_length_s') != 30.:
        raise ValueError('Saved episode length differs from supported 30s task')
    action = saved.get('actions', {}).get('JointPositionAction', {})
    if (action.get('scale') != .25 or action.get('use_default_offset') is not True
            or action.get('joint_names') != ['.*'] or action.get('preserve_order') is not False
            or action.get('offset') != 0. or action.get('clip') != {'.*':[-100.,100.]}
            or action.get('asset_name') != 'robot'
            or action.get('class_type') != 'isaaclab.envs.mdp.actions.joint_actions:JointPositionAction'):
        raise ValueError('Saved joint-position action contract differs')
    robot = saved.get('scene', {}).get('robot', {})
    initial = robot.get('init_state', {})
    root = dict(pos=[0.,0.,.5],rot=[1.,0.,0.,0.],lin_vel=[0.,0.,0.],ang_vel=[0.,0.,0.],joint_vel={'.*':0.})
    for key,value in root.items():
        if initial.get(key) != value:
            raise ValueError('Saved nominal root/default velocity differs: '+key)
    authored = initial.get('joint_pos', {})
    if not isinstance(authored,dict) or not authored:
        raise ValueError('Saved default joint configuration missing')
    resolved = {}
    for name in shared.EXPECTED_JOINT_NAMES:
        matches = [value for pattern,value in authored.items() if re.fullmatch(pattern,name)]
        if len(matches) != 1:
            raise ValueError('Saved joint default not uniquely resolved: '+name)
        resolved[name] = matches[0]
    if [resolved[name] for name in shared.EXPECTED_JOINT_NAMES] != shared.default_position().tolist():
        raise ValueError('Saved nominal/default joint positions differ')
    actuators = robot.get('actuators', {})
    supported = {
        'M107-24-2': (['.*_hip_.*','.*_thigh_.*'],200,23,'DelayedPDActuator'),
        '2': (['.*_calf_.*'],320,14,'DelayedPDActuator'),
        'z1_motor1': (['joint1','joint3','joint4','joint5','joint6'],120.,3.1415,'ImplicitActuator'),
        'z1_motor2': (['joint2'],120.,3.1415,'ImplicitActuator'),
    }
    if set(actuators) != set(supported):
        raise ValueError('Saved actuator groups differ')
    for name,(joints,effort,velocity,kind) in supported.items():
        expected = dict(class_type='isaaclab.actuators.actuator_pd:'+kind,joint_names_expr=joints,
            effort_limit=None,velocity_limit=None,effort_limit_sim=effort,velocity_limit_sim=velocity,
            stiffness=160.,damping=5.,armature=None,friction=.01,dynamic_friction=None,viscous_friction=None)
        if kind=='DelayedPDActuator': expected.update(min_delay=0,max_delay=0)
        if actuators[name] != expected:
            raise ValueError('Saved actuator gain/limit/friction/delay contract differs: '+name)
    obs = saved.get('observations', {}).get('policy', {})
    terms = ('velocity_commands','target_commands','base_ang_vel','projected_gravity','joint_pos_rel','joint_vel_rel','actions')
    if tuple(k for k,v in obs.items() if isinstance(v,dict) and 'func' in v) != terms:
        raise ValueError('Saved 70-observation term order differs')
    if (obs.get('concatenate_terms') is not True or obs.get('concatenate_dim') != -1
            or obs.get('enable_corruption') is not False or obs.get('history_length') is not None
            or obs.get('flatten_history_dim') is not True):
        raise ValueError('Saved observation aggregation/noise/history differs')
    functions = ('generated_commands','generated_commands','base_ang_vel','projected_gravity','joint_pos_rel','joint_vel_rel','last_action')
    for term,func in zip(terms,functions):
        cfg = obs[term]
        scale = .2 if term=='base_ang_vel' else .05 if term=='joint_vel_rel' else None
        params = {'command_name':'base_velocity'} if term=='velocity_commands' else {'command_name':'ee_target'} if term=='target_commands' else {}
        if (cfg.get('scale') != scale or cfg.get('clip') != [-100,100]
                or cfg.get('func') != 'isaaclab.envs.mdp.observations:'+func or cfg.get('params') != params
                or cfg.get('modifiers') is not None or cfg.get('history_length') != 0
                or cfg.get('flatten_history_dim') is not True):
            raise ValueError('Saved observation function/scale/clip/history differs: '+term)
    base = saved.get('commands', {}).get('base_velocity', {})
    expected = dict(camera_position_b=shared.COMMAND_CAMERA['position_b'],camera_rpy_b=shared.COMMAND_CAMERA['rpy_b'],
        camera_intrinsics=shared.COMMAND_CAMERA['intrinsics'],camera_image_size=shared.COMMAND_CAMERA['image_size'],
        camera_depth_range=shared.COMMAND_CAMERA['depth_range'],lookahead_distance=.6,heading_control_stiffness=1.5,
        trajectory_num_points=192,vd5_startup_duration_s=2.)
    for key,value in expected.items():
        if base.get(key) != value:
            raise ValueError('Saved ideal-command geometry differs: '+key)
    ee = saved.get('commands', {}).get('ee_target', {})
    if not str(ee.get('class_type','')).endswith(':'+command_class):
        raise ValueError('Saved world-seam command implementation differs')
    return dict(policy_terms=list(terms),action_scale=.25,processed_target_clip_rad=[-100.,100.],clock_checked=True,camera_geometry_checked=True,
                saved_drive_default_and_observation_settings_checked=True,
                trajectory_scope='straight subset, not the complete training turn distribution',
                differential_overrides=['nominal reset instead of RSI/mixed','no domain randomization','no observation noise'],
                completeness='Selected supported source settings are verified; this is NOT a full simulator/training equivalence proof')


def processed_joint_target(default, action):
    """Training JointPositionAction clips processed scale+default, not raw actor."""
    return np.clip(np.asarray(default)+.25*np.asarray(action),-100.,100.)


class GeometryLatch:
    """Single episode, four-substep latch; never reset a violation away."""
    def __init__(self):
        self.violated = False
        self.first = None
    def sample(self, result, step):
        if result['conservative_keepout_violated']:
            self.violated = True
            if self.first is None:
                self.first = dict(physics_step=step, time_s=step * .005, result=result)
        return self.violated and step % 4 == 0


def parse(argv=None, *, version=12):
    if version == 14:
        import vd14_mujoco_contract as backend
        default_checkpoint = backend.DEFAULT_CHECKPOINT
    elif version == 13:
        import vd13_mujoco_contract as backend
        default_checkpoint = backend.DEFAULT_CHECKPOINT
    elif version == 12:
        default_checkpoint = DEFAULT_CHECKPOINT
    else:
        raise ValueError('Unsupported MuJoCo version')
    parser = argparse.ArgumentParser(description=(__doc__ if version == 12 else
        f'VD{version} truth-command diagnostic. NOT visual deployment; use deploy_vd{version}_vision.sh.'))
    parser.add_argument('--checkpoint', type=Path, default=default_checkpoint)
    parser.add_argument('--duration', type=float, default=30.)
    parser.add_argument('--speed', type=float, default=.2)
    parser.add_argument('--viewer', action='store_true')
    parser.add_argument('--output-root', type=Path, default=ROOT / f'outputs/vd{version}_mujoco_play')
    args = parser.parse_args(argv)
    if not (math.isfinite(args.duration) and .02 <= args.duration <= 30
            and math.isclose(args.duration/.02,round(args.duration/.02),abs_tol=1e-9,rel_tol=0)
            and math.isfinite(args.speed) and 0 <= args.speed <= .4):
        parser.error('duration [.02,30] on the 20ms control grid, speed [0,.4] required')
    return args


def main(argv=None, *, version=12):
    args = parse(argv, version=version)
    if version == 14:
        import vd14_mujoco_contract as backend
    elif version == 13:
        import vd13_mujoco_contract as backend
    else:
        backend = sys.modules[__name__]
    os.environ.setdefault('MUJOCO_GL', 'glfw' if args.viewer else 'disable')
    import mujoco
    import torch
    from vd1_mujoco_contract import VD1MujocoAdapter, ACTOR_JOINT_NAMES
    from vd1_mujoco_scene import build_scene, validate_vd10_model
    from vd1_mujoco_velocity_impulse import VelocityImpulseConstraints
    from vd5_visual_control import encoder_tcp_pose

    torch.set_num_threads(1)
    checkpoint_path = args.checkpoint.expanduser().resolve(strict=True)
    checkpoint = torch.load(checkpoint_path, map_location='cpu', weights_only=False)
    profile = load_pure_module('vd12_contract_replay', ASSETS / 'vd12_contract.py').geometry_profile(backend.URDF)
    saved = load_yaml(checkpoint_path.parent / 'params/env.yaml')
    agent = load_yaml(checkpoint_path.parent / 'params/agent.yaml')
    curriculum = backend.validate_checkpoint(checkpoint, agent, saved, profile)
    settings = backend.validate_saved_settings(saved)
    actor = torch.nn.Sequential(torch.nn.Linear(70,512), torch.nn.ELU(), torch.nn.Linear(512,256),
        torch.nn.ELU(), torch.nn.Linear(256,128), torch.nn.ELU(), torch.nn.Linear(128,18)).eval()
    weights = {k[6:]: v for k,v in checkpoint['model_state_dict'].items() if k.startswith('actor.')}
    actor.load_state_dict(weights, strict=True)
    if not all(torch.isfinite(v).all() for v in weights.values()):
        raise ValueError('Nonfinite actor weights')
    if tuple(ACTOR_JOINT_NAMES) != tuple(shared.EXPECTED_JOINT_NAMES):
        raise ValueError('Scene and actor joint-order contracts differ')
    if shared.sha256(backend.URDF) != backend.URDF_SHA256:
        raise ValueError('New-gun URDF hash mismatch')
    args.output_root.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='play_' + time.strftime('%Y%m%d_%H%M%S') + '_', dir=args.output_root.resolve()))
    scene = build_scene(0., Path(tempfile.mkdtemp(prefix=f'vd{version}_mj_',dir='/dev/shm')),
        timestep=.005, tool_markers=False, road_surface='tape', robot_version=backend.ROBOT_VERSION)
    model = mujoco.MjModel.from_xml_path(str(scene))
    shared.save(output/'asset_audit.json',backend.validate_model(model) if version in (13, 14) else validate_vd10_model(model))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('vd1_default').id)
    adapter = VD1MujocoAdapter(model,physx_implicit_position=True)
    default = shared.default_position()
    adapter.default_q18 = default.copy()
    qids,dofs = model.jnt_qposadr[adapter.joint_ids18],model.jnt_dofadr[adapter.joint_ids18]
    data.qpos[2] = .5
    data.qpos[qids],data.qvel[:] = default,0.
    adapter.apply_targets(data,default)
    mujoco.mj_forward(model,data)
    constraint = VelocityImpulseConstraints(model,adapter.joint_ids18,adapter.velocity_limit18)
    guard = backend.MujocoGunGeometryGuard(model)
    if guard.profile != profile:
        raise ValueError('Runtime guard/profile mismatch')
    base,tcp,tool = model.body('base_link').id,model.site('tool_tcp').id,model.body('link06').id
    feet = [model.body(n).id for n in shared.FEET]
    illegal = [b for b in range(1,model.nbody) if re.fullmatch(r'base_link|.*_hip|.*_thigh|.*_calf|link0[0-5]',model.body(b).name or '')]
    if not illegal:
        raise ValueError('No illegal contact bodies')
    initial_p,initial_q = encoder_tcp_pose(default[[shared.EXPECTED_JOINT_NAMES.index(n) for n in shared.ARM_JOINT_NAMES]],
        backend.URDF,backend.TCP_POSITION,backend.TCP_ROTATION)
    np.testing.assert_allclose(data.xmat[base].reshape(3,3).T @ (data.site_xpos[tcp]-data.xpos[base]), initial_p, atol=1e-6)
    start_p = data.site_xpos[tcp].copy()
    start_q = shared.quaternion_multiply(data.xquat[base].copy(),initial_q)
    seam = shared.straight_world_seam(data.xpos[base],data.xquat[base],args.speed,args.duration)
    initial_geometry = guard.evaluate(data)
    shared.save(output/'manifest.json',dict(checkpoint=str(checkpoint_path),checkpoint_sha256=shared.sha256(checkpoint_path),
        iteration=checkpoint['iter'],experiment=checkpoint['training_experiment_name'],geometry_profile=profile,
        mount_profile=checkpoint.get('vd13_mount_profile'),work_distance_m=backend.WORK_DISTANCE_M,
        posture_profile=checkpoint.get('vd14_posture_profile'),
        tcp_position_link_m=list(backend.TCP_POSITION),
        saved_curriculum=curriculum,restored_ground_blend=1.,startup_profile=curriculum['startup_profile'],
        saved_settings_verified=settings,source_sha256={str(Path(shared.__file__)):shared.sha256(shared.__file__),
            str(Path(__file__)):shared.sha256(__file__)},
        reset_mode='nominal differential, not RSI/mixed',joint_order=list(ACTOR_JOINT_NAMES),actor_observations=70,actions=18,
        checkpoint_exported=False,default_actor_modified=False,simulation_only=True,robot_io=False,
        command_mode='truth-generated straight world seam, not rendered camera', extra_actor_observations=[],
        physics_dt_s=.005,actor_dt_s=.02,geometry_guard=guard.metadata,initial_geometry=initial_geometry,
        observation_noise=False,domain_randomization=False,
        drive_note='Engine actuator_force; NOT PD estimate. Beginning-state native solve, before split velocity projection.',
        state_note='Step-end after split velocity constraint; not an exact PhysX solver port.',
        sample_clock=dict(state_sample_time_s='physics_step * .005',native_force_sample_time_s='(physics_step - 1) * .005',
            native_force_state_phase_difference_s=.005,
            note='Forces/contact from native beginning-state solve, pose/velocity from end state. Do not infer simultaneous touchdown velocity without explicit phase alignment.'),
        termination_note='200Hz geometry latch, decision at 4-substep/20ms boundary; other thresholds at 20ms; first episode only.'))
    print(f'VD{version} MuJoCo truth-diagnostic output: '+str(output),flush=True)
    latch = GeometryLatch()
    latch.sample(initial_geometry,0)
    previous,target = np.zeros(18,dtype=np.float32),default.copy()
    pose,target_w = np.r_[initial_p,initial_q],start_p.copy()
    history,reason,terminal = deque(maxlen=20),'duration_complete',None
    count,minimum = 0,initial_geometry['minimum_clearance_lower_bound_m']
    viewer = None
    if args.viewer:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model,data)
        with viewer.lock(): shared.configure_camera(viewer.cam,data.xpos[base])
    wall = time.perf_counter()
    try:
        with gzip.open(output/'physics_200hz.jsonl.gz','wt') as log, gzip.open(output/'actor_50hz.jsonl.gz','wt') as actor_log:
            if latch.violated:
                reason,terminal='vd12_exclusion_at_reset',latch.first
            else:
                for tick in range(round(args.duration/.005)):
                    if tick%4==0:
                        sensor=adapter.read_proprioception(data)
                        pose,target_w,blend=shared.world_seam_command(data.xpos[base],data.xquat[base],start_p,start_q,seam,tick*.005,backend.WORK_DISTANCE_M)
                        command,guidance=shared.ideal_front_command(data.xpos[base],data.xquat[base],seam,args.speed)
                        command*=blend
                        obs=shared.actor_observation(command,pose,sensor,default,previous)
                        if obs.shape!=(70,) or not np.isfinite(obs).all(): raise ValueError('Invalid actor observation')
                        with torch.inference_mode(): action=actor(torch.from_numpy(obs).unsqueeze(0)).numpy()[0]
                        if action.shape!=(18,) or not np.isfinite(action).all(): raise ValueError('Invalid actor output')
                        target=processed_joint_target(default,action)
                        actor_log.write(json.dumps(dict(time_s=tick*.005,observation=obs.tolist(),action=action.tolist(),joint_target18_rad=target.tolist(),tool_target_world_m=target_w.tolist(),guidance=guidance),allow_nan=False)+'\n')
                        previous=action.copy()
                    adapter.apply_targets(data,target)
                    native={}
                    def snapshot(d):
                        forces=np.zeros((model.nbody,3))
                        for i in range(d.ncon):
                            c=d.contact[i]; f=np.zeros(6); mujoco.mj_contactForce(model,d,i,f)
                            fw=c.frame.reshape(3,3).T@f[:3]
                            a,b=model.geom_bodyid[[c.geom1,c.geom2]]; forces[a]-=fw; forces[b]+=fw
                        native.update(body_contact_force_world_n=forces.tolist(),drive_effort18_nm=d.actuator_force[adapter.actuator_ids18].copy().tolist(),
                            native_constraint_effort18_nm=d.qfrc_constraint[dofs].copy().tolist(),native_efc_type=d.efc_type[:d.nefc].copy().tolist(),
                            native_efc_force=d.efc_force[:d.nefc].copy().tolist())
                    constraint.step(data,evaluation_observer=snapshot)
                    count=tick+1
                    geometry=guard.evaluate(data)
                    reject=latch.sample(geometry,count)
                    minimum=min(minimum,geometry['minimum_clearance_lower_bound_m'])
                    rotation=data.xmat[base].reshape(3,3)
                    tilt=math.acos(float(np.clip(rotation[2,2],-1,1)))
                    forces=np.asarray(native['body_contact_force_world_n'])
                    foot_velocities=[]
                    for foot in feet:
                        jac=np.zeros((3,model.nv))
                        mujoco.mj_jac(model,data,jac,None,data.xpos[foot],foot)
                        foot_velocities.append((jac@data.qvel).tolist())
                    actual_axis=data.site_xmat[tcp].reshape(3,3)[:,0]
                    target_axis=rotation@shared.quaternion_matrix(pose[3:])[:,0]
                    history.append(max(float(forces[tool,2]),0.))
                    causes=shared.termination_causes(forces,history,illegal,tilt)
                    causes['vd12_gun_exclusion']=reject
                    row=dict(physics_step=count,time_s=count*.005,state_sample_time_s=count*.005,native_force_sample_time_s=(count-1)*.005,
                        joint_position18_rad=data.qpos[qids].tolist(),joint_velocity18_radps=data.qvel[dofs].tolist(),
                        joint_target18_rad=target.tolist(),base_position_w_m=data.xpos[base].tolist(),base_quaternion_wxyz=data.xquat[base].tolist(),
                        tcp_position_w_m=data.site_xpos[tcp].tolist(),tcp_target_w_50hz_zoh_m=target_w.tolist(),foot_force_world_n=forces[feet].tolist(),
                        foot_position_world_m=data.xpos[feet].tolist(),foot_velocity_world_m_s=foot_velocities,
                        gun_axis_error_rad=math.acos(float(np.clip(actual_axis@target_axis,-1,1))),
                        tilt_rad=tilt,geometry=geometry,geometry_latched=latch.violated,velocity_impulse=constraint.last,**native)
                    log.write(json.dumps(row,allow_nan=False)+'\n')
                    if count%4==0 and any(causes.values()):
                        reason='+'.join(k for k,v in causes.items() if v)
                        terminal=dict(physics_step=count,time_s=count*.005,causes=causes,first_geometry_failure=latch.first)
                        shared.save(output/'terminal_state.json',row)
                        break
                    if viewer:
                        if not viewer.is_running(): reason='viewer_closed'; break
                        with viewer.lock(): viewer.cam.lookat[:]=data.xpos[base]+np.array((-.28,0.,.05))
                        viewer.sync()
                        delay=wall+count*.005-time.perf_counter()
                        if delay>0: time.sleep(min(delay,.005))
    finally:
        if viewer: viewer.close()
    # A partial requested control interval cannot certify a clean final boundary.
    if reason=='duration_complete' and latch.violated:
        reason='vd12_gun_exclusion_latched_at_duration'
        terminal=latch.first
    passed=reason=='duration_complete'
    shared.save(output/'velocity_constraint.json',constraint.summary())
    shared.save(output/'summary.json',dict(status='completed_sampled_play' if passed else 'rejected',reason=reason,
        physics_steps=count,duration_s=count*.005,minimum_clearance_lower_bound_m=minimum,terminal_event=terminal,
        automatic_resets=0,simulation_only=True,deployment_validation=False,
        note='A completed sample is not real-robot approval or statistical motion-quality acceptance; conservative keep-out is not measured penetration.'))
    print(json.dumps(dict(output=str(output),reason=reason,duration_s=count*.005),indent=2),flush=True)
    return 0 if passed else 2


if __name__=='__main__':
    raise SystemExit(main())
