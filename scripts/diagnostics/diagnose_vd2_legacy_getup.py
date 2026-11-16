#!/usr/bin/env python3
"""Full prone -> get-up -> hold -> B in MuJoCo; no hardware transports."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import mujoco


def simulate(args):
    sys.path[:0]=[str(args.controller/'python'),str(args.bundle/'python/vision')]
    import control
    control.ROOT=args.bundle
    model=mujoco.MjModel.from_xml_path(str(args.model))
    model.opt.timestep=.002;model.opt.integrator=mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    data=mujoco.MjData(model)
    removed=[]
    if args.no_tool:
        # User confirmed this field test has no gun/hose. The scene generator
        # merges the 1.1 m, zero-added-mass gun collider into link06.
        gun=model.geom('link06_collision_0').id
        if not np.allclose(model.geom_size[gun,:2],[.028,.55]):
            raise RuntimeError('unexpected gun geometry; cannot apply no-tool fixture')
        model.geom_contype[gun]=0;model.geom_conaffinity[gun]=0
        removed.append(model.geom(gun).name)
    joints=[model.joint(name).id for name in control.JOINTS]
    qa=model.jnt_qposadr[joints];va=model.jnt_dofadr[joints]
    actuators=[int(np.where(model.actuator_trnid[:,0]==j)[0][0]) for j in joints]
    prone=control.DEFAULT.copy()
    prone[:8]=[.294,-.285,.286,-.289,1.214,1.212,1.211,1.212]
    prone[9:13]=[-2.803,-2.804,-2.790,-2.800]
    prone[control.ARM]=[-.0244,0.,0.,-.15133,.02182,-.05431]
    data.qpos[qa]=prone;data.qpos[:3]=[0,0,0]
    mujoco.mj_forward(model,data)
    plane_ids=set(np.where(model.geom_type==mujoco.mjtGeom.mjGEOM_PLANE)[0])
    ground=[c.dist for c in data.contact if c.geom1 in plane_ids or c.geom2 in plane_ids]
    if not ground:raise RuntimeError('no ground contact available for prone placement')
    data.qpos[2]=.002-min(ground)
    mujoco.mj_forward(model,data)
    initial_height=float(data.qpos[2]);base=model.body('base_link').id
    for actuator in actuators:
        model.actuator_gaintype[actuator]=mujoco.mjtGain.mjGAIN_FIXED
        model.actuator_biastype[actuator]=mujoco.mjtBias.mjBIAS_AFFINE
        model.actuator_gainprm[actuator,:]=0;model.actuator_biasprm[actuator,:]=0
    session=control.Session()
    comp=control.StandLoadCompensation(control.limits()) if args.previous else None
    trace=[];modes=[]
    for step in range(round(60/.02)):
        now=step*.02;q=data.qpos[qa].copy();qd=data.qvel[va].copy()
        omega=data.qvel[3:6].copy();gravity=data.xmat[base].reshape(3,3).T@np.array([0,0,-1.])
        if step==0:session.event('stand',now,q)
        if step==round(args.down_at/.02):session.event('down',now,q)
        session.advance(now,q,qd,omega,gravity)
        command=session.target.copy()
        if comp:command,_=comp.update(session,now,q,qd,omega,gravity)
        kp=np.full(18,200.);kp[9:13]=300.;kd=np.full(18,3.)
        if not args.previous:kp=session.kp.copy();kd=session.kd.copy()
        # Simulated arm dynamics are not calibrated to Z1 RAW SDK gain units.
        kp[control.ARM]=160.;kd[control.ARM]=5.
        model.actuator_gainprm[actuators,0]=kp
        model.actuator_biasprm[actuators,1]=-kp
        model.actuator_biasprm[actuators,2]=-kd
        if not modes or modes[-1]['mode']!=session.mode:
            modes.append(dict(time_s=now,mode=session.mode,message=session.message))
        for _ in range(10):
            data.ctrl[actuators]=command;mujoco.mj_step(model,data)
        if step%5==0:
            trace.append(dict(time_s=now,state=session.mode,height_m=float(data.qpos[2]),
                tilt_deg=float(np.degrees(np.arccos(np.clip(data.xmat[base].reshape(3,3)[2,2],-1,1)))),
                q=data.qpos[qa].tolist(),target_q=command.tolist(),kp=kp.tolist(),kd=kd.tolist(),
                leg_error_rad=float(np.max(np.abs(data.qpos[qa][control.LEGS]-session.target[control.LEGS])))))
    hold=[r for r in trace if 10<=r['time_s']<args.down_at]
    return dict(hardware_tested=False,
        scope='Full prone get-up, 50 s B request, 60 s total; free base and original collision/mass; simulated arm 160/5 is NOT hardware SDK RAW gains.',
        control_inputs='joint encoders, joint velocity, angular velocity, projected gravity, elapsed time; world pose only for initial ground placement/evaluation',
        controller=str(args.controller),previous_bias_candidate=args.previous,initial_height_m=initial_height,
        mass_kg=float(model.body_mass.sum()),modes=modes,final_mode=session.mode,
        no_tool=args.no_tool,removed_collision_geometries=removed,
        down_at_s=args.down_at,
        getup_tilt_max_deg=max(r['tilt_deg'] for r in trace if r['time_s']<min(10,args.down_at)),
        hold_tilt_max_deg=max(r['tilt_deg'] for r in hold) if hold else None,
        hold_height_mean_m=float(np.mean([r['height_m'] for r in hold])) if hold else None,
        hold_leg_error_p95_rad=float(np.percentile([r['leg_error_rad'] for r in hold],95)) if hold else None,trace=trace)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('controller','bundle','model','output'):parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--previous',action='store_true')
    parser.add_argument('--no-tool',action='store_true',help='field-confirmed gun/hose absent; disable ONLY zero-mass gun collider')
    parser.add_argument('--down-at',type=float,default=50.)
    args=parser.parse_args()
    if not 0<args.down_at<55:raise ValueError('--down-at must be between 0 and 55 s')
    if args.output.exists():raise FileExistsError(args.output)
    report=simulate(args);args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='trace'},indent=2))


if __name__=='__main__':main()
