#!/usr/bin/env python3
"""Simulation-only 1-DOF readback calibration, separate from walking results.

No robot asset/policy is changed. Zero friction/gravity, no contact, fixed base,
known diagonal inertia. This verifies what runtime effort getters measure.
"""
import argparse
import json
from pathlib import Path
import numpy as np

DT = .005
# A modest-inertia fixture and a light wrist-like fixture. The latter is NOT
# claimed to reproduce the articulated B2+Z1 effective inertia.
CASES = [('linear', .05, .05, 1000.), ('speed_limit', .05, 1., 3.1415),
         ('effort_limit', .05, 10., 1000.), ('light', .0001, .05, 3.1415)]


def mujoco_probe():
    import mujoco
    results = []
    for name,inertia,target,limit in CASES:
        model = mujoco.MjModel.from_xml_string('''<mujoco>
          <option timestep=".005" integrator="implicitfast" gravity="0 0 0"/>
          <worldbody><body><joint name="hinge" axis="0 0 1"/>
          <inertial pos="0 0 0" mass="1" diaginertia="%g %g %g"/>
          </body></worldbody><actuator><position joint="hinge" kp="160" kv="5.8"
          forcelimited="true" forcerange="-120 120"/></actuator></mujoco>''' % ((inertia,)*3))
        data = mujoco.MjData(model)
        from vd1_mujoco_velocity_impulse import VelocityImpulseConstraints
        solver = VelocityImpulseConstraints(model,[0],[limit])
        rows = []
        for step in range(80):
            q0,v0 = float(data.qpos[0]),float(data.qvel[0])
            data.ctrl[0] = target
            readback = {}
            def snapshot(d):
                readback.update(engine_actuator_force_nm=float(d.actuator_force[0]),
                                native_v=float(d.qvel[0]))
            solver.step(data,evaluation_observer=snapshot)
            rows.append(dict(time_s=(step+1)*DT,q=float(data.qpos[0]),v=float(data.qvel[0]),
                q_before=q0,v_before=v0,net_momentum_effort_nm=inertia*(data.qvel[0]-v0)/DT,
                speed_impulse_nms=solver.last['generalized_impulse'][0],**readback))
        results.append(dict(case=name,inertia_kgm2=inertia,target_rad=target,speed_limit_radps=limit,rows=rows))
    return results


def isaac_probe(args):
    import torch
    import isaaclab.sim as su
    from isaaclab.assets import Articulation, ArticulationCfg
    from isaaclab.actuators import ImplicitActuatorCfg
    from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf
    scfg = su.SimulationCfg(dt=DT, device=args.device, gravity=(0.,0.,0.))
    scfg.physx.max_position_iteration_count = 4
    scfg.physx.max_velocity_iteration_count = 0
    sim = su.SimulationContext(scfg)
    stage = sim.stage
    robots = []
    for index,(name,inertia,target,limit) in enumerate(CASES):
        root = '/World/Probe'+str(index)
        UsdGeom.Xform.Define(stage,root)
        base = UsdGeom.Xform.Define(stage,root+'/base')
        UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
        UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr(1.)
        fixed = UsdPhysics.FixedJoint.Define(stage,root+'/fixed')
        fixed.CreateBody1Rel().SetTargets([base.GetPath()])
        # Root API belongs on the world fixed joint so PhysX creates a truly
        # fixed-base reduced-coordinate articulation, not a floating root held
        # by a separate maximal-coordinate weld constraint.
        art = UsdPhysics.ArticulationRootAPI.Apply(fixed.GetPrim())
        ap = PhysxSchema.PhysxArticulationAPI.Apply(art.GetPrim())
        ap.CreateSolverPositionIterationCountAttr(4)
        ap.CreateSolverVelocityIterationCountAttr(0)
        child = UsdGeom.Xform.Define(stage,root+'/child')
        UsdPhysics.RigidBodyAPI.Apply(child.GetPrim())
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(child.GetPrim())
        rb.CreateAngularDampingAttr(0.)
        rb.CreateLinearDampingAttr(0.)
        rb.CreateMaxAngularVelocityAttr(10000.)
        mass = UsdPhysics.MassAPI.Apply(child.GetPrim())
        mass.CreateMassAttr(1.)
        mass.CreateDiagonalInertiaAttr(Gf.Vec3f(inertia,inertia,inertia))
        joint = UsdPhysics.RevoluteJoint.Define(stage,root+'/hinge')
        joint.CreateBody0Rel().SetTargets([base.GetPath()])
        joint.CreateBody1Rel().SetTargets([child.GetPath()])
        joint.CreateAxisAttr('Z')
        pj = PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim())
        pj.CreateMaxJointVelocityAttr(limit)
        pj.CreateJointFrictionAttr(0.)
        UsdPhysics.DriveAPI.Apply(joint.GetPrim(),'angular').CreateTypeAttr('force')
        cfg = ArticulationCfg(prim_path=root,spawn=None,
            actuators={'drive':ImplicitActuatorCfg(joint_names_expr=['hinge'],stiffness=160.,
                damping=5.,effort_limit_sim=120.,velocity_limit_sim=limit,friction=0.)})
        robots.append(Articulation(cfg))
    sim.reset()
    results = [dict(case=name,inertia_kgm2=inertia,target_rad=target,speed_limit_radps=limit,rows=[])
               for name,inertia,target,limit in CASES]
    for robot,out in zip(robots,results):
        robot.update(DT)
        view=robot.root_physx_view
        out['runtime_inertias']=view.get_inertias().cpu().tolist()
        out['runtime_masses']=view.get_masses().cpu().tolist()
        out['body_names']=robot.body_names
        out['fixed_base']=robot.is_fixed_base
        if not robot.is_fixed_base:
            raise RuntimeError('Effort fixture must be fixed base; refuse invalid momentum calibration')
    for step in range(80):
        before = [(float(r.data.joint_pos[0,0]),float(r.data.joint_vel[0,0])) for r in robots]
        for robot,case in zip(robots,CASES):
            robot.set_joint_position_target(torch.tensor([[case[2]]],device=sim.device))
            robot.write_data_to_sim()
        sim.step(render=False)
        for robot,case,out,(q0,v0) in zip(robots,CASES,results,before):
            robot.update(DT)
            q,v=float(robot.data.joint_pos[0,0]),float(robot.data.joint_vel[0,0])
            view=robot.root_physx_view
            out['rows'].append(dict(time_s=(step+1)*DT,q=q,v=v,q_before=q0,v_before=v0,
                net_momentum_effort_nm=case[1]*(v-v0)/DT,
                child_angular_velocity_w=robot.data.body_ang_vel_w[0,robot.body_names.index('child')].cpu().tolist(),
                projected_joint_effort_nm=float(view.get_dof_projected_joint_forces()[0,0]),
                user_actuation_effort_nm=float(view.get_dof_actuation_forces()[0,0])))
    return results


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--engine',choices=['mujoco','isaac'],required=True)
    parser.add_argument('--output',type=Path,required=True)
    first,_=parser.parse_known_args()
    launcher=None
    if first.engine=='isaac':
        from isaaclab.app import AppLauncher
        AppLauncher.add_app_launcher_args(parser)
        parser.set_defaults(headless=True)
        launcher=AppLauncher
    args=parser.parse_args()
    app=launcher(args).app if launcher else None
    try:
        result=isaac_probe(args) if app else mujoco_probe()
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(json.dumps(dict(engine=args.engine,dt_s=DT,cases=result),indent=2)+'\n')
        print('Saved effort fixture:',args.output,flush=True)
    finally:
        if app:
            from isaaclab.sim import SimulationContext
            sim=SimulationContext.instance()
            if sim:
                sim.clear_instance()
                sim.stop()
                sim.clear_all_callbacks()
            app.close(wait_for_replicator=False)


if __name__=='__main__':
    main()
