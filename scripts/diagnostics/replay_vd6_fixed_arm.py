#!/usr/bin/env python3
"""Offline fixed-base arm target replay: no actor, ground or foot impacts.

Use identical recorded arm targets in both engines. Source gains/limits and
robot inertia are retained. Fixed base and removal of ground are intentional
experimental changes; this is not a walking or deployment evaluation.
"""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0,'/home/dong/b2_z1/scripts/vision')
from analyze_vd6_joint_oscillation import read_rows, hp, rms
from vd1_mujoco_scene import (EXPECTED_JOINT_NAMES, DEFAULT_JOINT_POSITION,
                              VD3_TCP_POSITION)
from vd5_mujoco_control import quaternion_matrix

ARM=[8,13,14,15,16,17]
DT=.005


def mujoco_run(args,targets,output):
    import mujoco
    from vd1_mujoco_scene import build_scene
    from vd1_mujoco_velocity_impulse import VelocityImpulseConstraints
    scene=build_scene(0.,Path(tempfile.mkdtemp(prefix='vd6_replay_',dir='/dev/shm')),
                      timestep=DT,tool_markers=False,road_surface='tape',robot_version='vd3')
    tree=ET.parse(scene)
    world=tree.getroot().find('worldbody')
    for geom in list(world.findall('geom')):
        world.remove(geom)
    base=world.find("body[@name='base_link']")
    base.remove(base.find('freejoint'))
    base.set('pos','0 0 .5')
    for key in tree.getroot().findall('keyframe/key'):
        key.set('qpos',' '.join(key.get('qpos').split()[7:]))
        if key.get('qvel'):
            key.set('qvel',' '.join(key.get('qvel').split()[6:]))
    scene=output/'fixed_base_scene.xml'
    tree.write(scene)
    model=mujoco.MjModel.from_xml_path(str(scene))
    data=mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('vd1_default').id)
    joints=np.array([model.joint(n).id for n in EXPECTED_JOINT_NAMES])
    aids=np.array([model.actuator(n+'_motor').id for n in EXPECTED_JOINT_NAMES])
    dofs=model.jnt_dofadr[joints]
    qpos=model.jnt_qposadr[joints]
    limits=np.array([3.1415 if i in ARM else 14. if 'calf' in n else 23.
                     for i,n in enumerate(EXPECTED_JOINT_NAMES)])
    efforts=np.array([120. if i in ARM else 320. if 'calf' in n else 200.
                      for i,n in enumerate(EXPECTED_JOINT_NAMES)])
    model.actuator_biasprm[aids[ARM],2]=-5.8
    solver=VelocityImpulseConstraints(model,joints,limits)
    rows=[]
    for tick in range(len(targets)*4):
        target=targets[tick//4]
        mujoco.mj_forward(model,data)
        torque=np.clip(160*(target-data.qpos[qpos])-5*data.qvel[dofs],-efforts,efforts)
        data.ctrl[aids]=torque
        data.ctrl[aids[ARM]]=target[ARM]
        native={}
        def snapshot(d):
            native['engine_actuator_force18_nm']=d.actuator_force[aids].copy().tolist()
        solver.step(data,evaluation_observer=snapshot)
        base_id=model.body('base_link').id
        rotation=data.xmat[base_id].reshape(3,3)
        tip=rotation.T@(data.site_xpos[model.site('tool_tcp').id]-data.xpos[base_id])
        rows.append(dict(time_s=(tick+1)*DT,joint_position18_rad=data.qpos[qpos].tolist(),
            joint_velocity18_radps=data.qvel[dofs].tolist(),joint_target18_rad=target.tolist(),
            tcp_position_b=tip.tolist(),speed_constraint_impulse18_nms=
            np.asarray(solver.last['generalized_impulse'])[dofs].tolist(),**native))
    return rows


def isaac_run(args,targets,output):
    import torch
    import isaaclab.sim as su
    from isaaclab.assets import Articulation
    from b2_z1.assets.unitree import B2_Z1_GUN_CFG
    scfg=su.SimulationCfg(dt=DT,device=args.device)
    scfg.physx.solver_type=1
    scfg.physx.max_position_iteration_count=4
    scfg.physx.max_velocity_iteration_count=0
    sim=su.SimulationContext(scfg)
    cfg=B2_Z1_GUN_CFG.copy()
    cfg.prim_path='/World/Robot'
    cfg.init_state.pos=(0.,0.,.5)
    cfg.spawn.fix_base=True
    cfg.spawn.usd_dir=str(output/'usd_cache')
    cfg.spawn.force_usd_conversion=True
    robot=Articulation(cfg)
    sim.reset();robot.update(DT)
    if not robot.is_fixed_base:
        raise RuntimeError('Replay requires a true fixed-base articulation')
    ids=[robot.joint_names.index(n) for n in EXPECTED_JOINT_NAMES]
    end=robot.body_names.index('link06')
    rows=[]
    for tick in range(len(targets)*4):
        target=targets[tick//4]
        if tick%4==0:
            robot.set_joint_position_target(torch.as_tensor(target,dtype=torch.float32,device=sim.device)[None],joint_ids=ids)
        robot.write_data_to_sim()
        sim.step(render=False);robot.update(DT)
        state=robot.data
        rotation=quaternion_matrix(state.root_link_quat_w[0].cpu().numpy())
        tip=(state.body_link_pos_w[0,end].cpu().numpy()+
             quaternion_matrix(state.body_link_quat_w[0,end].cpu().numpy())@np.array(VD3_TCP_POSITION))
        pos=state.root_link_pos_w[0].cpu().numpy()
        rows.append(dict(time_s=(tick+1)*DT,joint_position18_rad=state.joint_pos[0,ids].cpu().tolist(),
            joint_velocity18_radps=state.joint_vel[0,ids].cpu().tolist(),
            joint_target18_rad=target.tolist(),tcp_position_b=(rotation.T@(tip-pos)).tolist(),
            joint_solver_effort18_nm=robot.root_physx_view.get_dof_projected_joint_forces()[0,ids].cpu().tolist(),
            drive_effort18_nm=None))
    return rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--engine',choices=['mujoco','isaac'],required=True)
    parser.add_argument('--source-session',type=Path,required=True)
    parser.add_argument('--output-root',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=15.)
    first,_=parser.parse_known_args();launcher=None
    if first.engine=='isaac':
        from isaaclab.app import AppLauncher
        AppLauncher.add_app_launcher_args(parser)
        parser.set_defaults(headless=True);launcher=AppLauncher
    args=parser.parse_args()
    if not 5<args.duration<=30:
        parser.error('Replay duration must be >5 and <=30 s')
    actor=read_rows(args.source_session/'actor_trace.jsonl.gz')
    count=round(args.duration/.02)
    if len(actor)<count or not np.allclose([r['time_s'] for r in actor[:count]],np.arange(count)*.02):
        raise ValueError('Source must contain uniform 50 Hz commands from time zero')
    default=np.array([DEFAULT_JOINT_POSITION[n] for n in EXPECTED_JOINT_NAMES])
    targets=np.tile(default,(count,1))
    targets[:,ARM]=np.array([r['joint_target18'] for r in actor[:count]])[:,ARM]
    args.output_root=args.output_root.resolve()
    args.output_root.mkdir(parents=True,exist_ok=True)
    output=Path(tempfile.mkdtemp(prefix=args.engine+'_',dir=args.output_root))
    manifest=dict(engine=args.engine,source_session=str(args.source_session.resolve()),
        source_manifest=json.loads((args.source_session/'manifest.json').read_text()),
        arm_target_sequence_sha256=hashlib.sha256(targets[:,ARM].tobytes()).hexdigest(),
        duration_s=count*.02,physics_dt_s=DT,actor_present=False,actor_inputs=[],
        fixed_base=True,ground_present=False,leg_targets='constant default',gravity=True,
        source_kp_kd=[160.,5.],mujoco_numerical_arm_kd=5.8,simulation_only=True,robot_io=False,
        note='Local replay experiment; not walking/deployment metrics. Original arm targets held without filtering.')
    (output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    app=launcher(args).app if launcher else None
    try:
        rows=isaac_run(args,targets,output) if app else mujoco_run(args,targets,output)
        with gzip.open(output/'joint_replay.jsonl.gz','xt') as f:
            for r in rows:
                r['evaluation_only']=True
                f.write(json.dumps(r,allow_nan=False)+'\n')
        selected=[r for r in rows if r['time_s']>=5]
        tip=np.array([r['tcp_position_b'] for r in selected])
        q=np.array([r['joint_position18_rad'] for r in selected])[:,ARM]
        residual=hp(tip,200)[200:-200]
        summary=dict(tcp_body_hp_axis_rms_mm=(rms(residual)*1000).tolist(),
                     tcp_body_hp_vector_rms_mm=float(np.sqrt(np.mean(np.sum(residual**2,axis=1)))*1000),
                     joint_hp_rms_rad=rms(hp(q,200)[200:-200]).tolist(),
                     steps=len(rows),reason='duration_complete',window_s=[5,count*.02])
        (output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(output,summary,flush=True)
    finally:
        if app:
            from isaaclab.sim import SimulationContext
            sim=SimulationContext.instance()
            if sim:
                sim.clear_instance();sim.stop();sim.clear_all_callbacks()
            app.close(wait_for_replicator=False)


if __name__=='__main__':
    main()
