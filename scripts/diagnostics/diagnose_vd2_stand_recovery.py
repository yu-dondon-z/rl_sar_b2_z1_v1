#!/usr/bin/env python3
"""Headless support/load-bias comparison; no SDK, cameras, or real IO."""
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import mujoco

REPO=Path(__file__).resolve().parents[2]


def simulate(model_path,bundle,use_bias,hold_seconds):
    # Historical candidate is archived in the bundle. Never silently import
    # today's controller when reproducing the withdrawn load-bias experiment.
    sys.path[:0]=[str(bundle/'python'),str(bundle/'python/vision')]
    import control
    control.ROOT=bundle
    from control import ARM,LEGS,JOINTS,DEFAULT,STAND,Session,StandLoadCompensation
    model=mujoco.MjModel.from_xml_path(str(model_path));model.opt.timestep=.002
    model.opt.integrator=mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    data=mujoco.MjData(model)
    joints=[model.joint(name).id for name in JOINTS]
    qa=model.jnt_qposadr[joints];va=model.jnt_dofadr[joints]
    actuators=[int(np.where(model.actuator_trnid[:,0]==joint)[0][0]) for joint in joints]
    nominal=STAND.copy();nominal[ARM]=[-.0244,0,0,-.15133,.02182,-.05431]
    prone=nominal.copy();prone[:8]=[.294,-.285,.286,-.289,1.214,1.212,1.211,1.212]
    prone[9:13]=[-2.803,-2.804,-2.790,-2.800]
    data.qpos[qa]=nominal;data.qpos[:3]=[0,0,0]
    mujoco.mj_forward(model,data)
    foot_geoms=[model.body_geomadr[model.body(leg+'_foot').id] for leg in ('FL','FR','RL','RR')]
    data.qpos[2]=.003-min(data.geom_xpos[g,2]-model.geom_size[g,0] for g in foot_geoms)
    mujoco.mj_forward(model,data)
    base=model.body('base_link').id
    session=Session();session.mode='standing_up';session.entered=0
    session.start=nominal.copy();session.end=nominal.copy();session.target=nominal.copy();session.prone=prone
    load=StandLoadCompensation(control.limits())
    kp=np.full(18,200.);kp[9:13]=300.;kp[ARM]=160.
    kd=np.full(18,3.);kd[ARM]=5.
    # MuJoCo must see damping in its force derivatives. Evaluating stiff arm
    # PD externally as explicit torque is unstable at 2 ms for small inertias.
    for joint,actuator in enumerate(actuators):
        model.actuator_gaintype[actuator]=mujoco.mjtGain.mjGAIN_FIXED
        model.actuator_biastype[actuator]=mujoco.mjtBias.mjBIAS_AFFINE
        model.actuator_gainprm[actuator,:]=0;model.actuator_gainprm[actuator,0]=kp[joint]
        model.actuator_biasprm[actuator,:]=0
        model.actuator_biasprm[actuator,1:3]=[-kp[joint],-kd[joint]]
    trace=[];modes=[]
    for step in range(round((hold_seconds+10)/.02)):
        now=step*.02;q=data.qpos[qa].copy();qd=data.qvel[va].copy()
        omega=data.qvel[3:6].copy();gravity=data.xmat[base].reshape(3,3).T@np.array([0,0,-1.])
        if step==round(hold_seconds/.02):session.event('down',now,q)
        session.advance(now,q,qd,omega,gravity)
        if use_bias:command,detail=load.update(session,now,q,qd,omega,gravity)
        else:command=session.target.copy();detail={'stand_load_bias_q':np.zeros(18).tolist()}
        if not modes or modes[-1]['mode']!=session.mode:modes.append(dict(time_s=now,mode=session.mode,message=session.message))
        for _ in range(10):
            data.ctrl[actuators]=command;mujoco.mj_step(model,data)
        if step%5==0:
            trace.append(dict(time_s=now,mode=session.mode,height_m=float(data.qpos[2]),
                nominal_leg_error_rad=float(np.max(np.abs(data.qpos[qa][LEGS]-session.target[LEGS]))),
                tilt_deg=float(np.degrees(np.arccos(np.clip(data.xmat[base].reshape(3,3)[2,2],-1,1)))),
                max_bias_rad=float(np.max(np.abs(detail['stand_load_bias_q']))),
                actual_q=data.qpos[qa].tolist(),command_q=command.tolist()))
    hold=[r for r in trace if hold_seconds-3<=r['time_s']<hold_seconds]
    all_hold=[r for r in trace if 8<=r['time_s']<hold_seconds]
    return dict(use_bias=use_bias,robot_mass_kg=float(model.body_mass.sum()),modes=modes,
        stand_leg_error_p95_rad=float(np.percentile([r['nominal_leg_error_rad'] for r in hold],95)),
        stand_height_mean_m=float(np.mean([r['height_m'] for r in hold])),
        stand_tilt_max_deg=float(max(r['tilt_deg'] for r in hold)),final_mode=session.mode,
        post_startup_tilt_max_deg=float(max(r['tilt_deg'] for r in all_hold)),
        final_leg_error_rad=trace[-1]['nominal_leg_error_rad'],trace=trace)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model',type=Path,required=True)
    parser.add_argument('--bundle',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--hold-seconds',type=float,default=50.)
    args=parser.parse_args()
    report=dict(hardware_tested=False,
        scope='Free-base MuJoCo stationary support then B lowering; does not reproduce real hardware, full prone-to-stand transition, Z1 firmware or exact load.',
        controller_inputs='joint encoders, angular velocity, gravity projection, state and elapsed time; global poses used only for evaluation and initial placement',
        assumptions='Implicit MuJoCo joint servos, hardware leg Kp 200/300 and Kd 3; simulated arm held with 160/5, not claimed equivalent to real SDK raw arm gains; zero added gun mass.',
        baseline=simulate(args.model,args.bundle,False,args.hold_seconds),
        candidate=simulate(args.model,args.bundle,True,args.hold_seconds))
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({name:{key:value for key,value in report[name].items() if key!='trace'} for name in ('baseline','candidate')},indent=2))


if __name__=='__main__':main()
