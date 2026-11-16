#!/usr/bin/env python3
"""Independent floating-base MuJoCo inverse-dynamics reference; no hardware."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def xyz(node, attr="xyz"):
    return np.fromstring(node.get(attr,"0 0 0") if node is not None else "0 0 0",sep=" ")


def quat(angles):
    r,p,y=np.array(angles)/2
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return [cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]


def reference_xml(urdf):
    root=ET.parse(urdf).getroot();links={n.get("name"):n for n in root.findall("link")};joints={n.get("name"):n for n in root.findall("joint")}
    mj=ET.Element("mujoco");ET.SubElement(mj,"compiler",angle="radian",autolimits="true",fusestatic="false")
    ET.SubElement(mj,"option",gravity="0 0 -9.81")
    base=ET.SubElement(ET.SubElement(mj,"worldbody"),"body",name="base_link")
    ET.SubElement(base,"freejoint",name="free")
    ET.SubElement(base,"inertial",mass="30",pos="0 0 0",diaginertia="1 1 1")
    for name in ("FR","FL","RR","RL"):
        parent=base
        for part in ("hip","thigh","calf","foot"):
            joint=joints[name+"_"+part+"_joint"];origin=joint.find("origin")
            parent=ET.SubElement(parent,"body",name=name+"_"+part,pos=" ".join(map(str,xyz(origin))),quat=" ".join(map(str,quat(xyz(origin,"rpy")))))
            inertial=links[joint.find("child").get("link")].find("inertial");i=inertial.find("inertia")
            if np.linalg.norm(xyz(inertial.find("origin"),"rpy"))>1e-12:raise ValueError("Reference needs rotated inertial axes conversion")
            ET.SubElement(parent,"inertial",mass=inertial.find("mass").get("value"),pos=inertial.find("origin").get("xyz"),fullinertia=" ".join(i.get(k) for k in ("ixx","iyy","izz","ixy","ixz","iyz")))
            if part!="foot":
                lim=joint.find("limit");ET.SubElement(parent,"joint",name=joint.get("name"),type="hinge",axis=joint.find("axis").get("xyz"),range=lim.get("lower")+" "+lim.get("upper"))
            else:ET.SubElement(parent,"site",name=name+"_center",pos="0 0 0",size=".001")
    return ET.tostring(mj,encoding="unicode"),xyz(joints["joint_imu"].find("origin"))


def generate(urdf):
    xml,imu=reference_xml(urdf);m=mujoco.MjModel.from_xml_string(xml);d=mujoco.MjData(m)
    rng=np.random.default_rng(20261020);cases=[]
    for k in range(96):
        orient=quat([rng.uniform(-.3,.3),rng.uniform(-.3,.3),rng.uniform(-math.pi,math.pi)])
        q=np.array([[rng.uniform(-.4,.4),rng.uniform(.3,1.3),rng.uniform(-2.5,-.65)] for _ in range(4)]).flatten()
        dq=rng.uniform(-3.,3.,12) if k>=32 else np.zeros(12)
        qdd=rng.uniform(-8.,8.,12) if k>=32 else np.zeros(12)
        gyro=rng.uniform(-.7,.7,3) if k>=32 else np.zeros(3)
        alpha=rng.uniform(-1.,1.,3) if k>=32 else np.zeros(3)
        acceleration=rng.uniform(-2.,2.,3) if k>=32 else np.zeros(3)
        d.qpos[:3]=[.1,-.2,.55];d.qpos[3:7]=orient;d.qpos[7:]=q
        d.qvel[:3]=rng.uniform(-1.,1.,3);d.qvel[3:6]=gyro;d.qvel[6:]=dq
        d.qacc[:3]=acceleration;d.qacc[3:6]=alpha;d.qacc[6:]=qdd
        mujoco.mj_inverse(m,d)
        rotation=np.empty(9);mujoco.mju_quat2Mat(rotation,np.array(orient));rotation=rotation.reshape(3,3)
        specific=rotation.T@(acceleration+np.array([0,0,9.81]))+np.cross(alpha,imu)+np.cross(gyro,np.cross(gyro,imu))
        forces=[];torques=[];inverse=[]
        for i,name in enumerate(("FR","FL","RR","RL")):
            jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
            mujoco.mj_jacSite(m,d,jp,jr,mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,name+"_center"))
            force=np.array([rng.uniform(-20,20),rng.uniform(-20,20),rng.uniform(80,300)])
            if rng.random()<.25:force[:]=0.
            inv=d.qfrc_inverse[6+3*i:9+3*i].copy()
            tau=inv-jp[:,6+3*i:9+3*i].T@force
            forces.append(force.tolist());torques.extend(tau.tolist());inverse.append(inv.tolist())
        cases.append(dict(q=q.tolist(),dq=dq.tolist(),qdd=qdd.tolist(),tau=torques,quaternion=orient,
            gyro=gyro.tolist(),angular_acceleration=alpha.tolist(),specific_force=specific.tolist(),
            expected_inverse_torque=inverse,expected_force=forces))
    return dict(source_sha256=hashlib.sha256(Path(urdf).read_bytes()).hexdigest(),mujoco_version=mujoco.__version__,
        static_cases=32,dynamic_cases=64,hardware_qualified=False,control_authorized=False,cases=cases)


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("urdf",type=Path);p.add_argument("--output",type=Path,required=True);a=p.parse_args()
    if a.output.exists():p.error("Refusing existing output")
    report=generate(a.urdf);a.output.parent.mkdir(parents=True,exist_ok=True)
    with a.output.open("x") as f:json.dump(report,f,indent=2,allow_nan=False)
    print("Generated 32 static + 64 dynamic goldens; not a hardware qualification")
