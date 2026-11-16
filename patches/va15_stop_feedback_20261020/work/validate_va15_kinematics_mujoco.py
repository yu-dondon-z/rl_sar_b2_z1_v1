#!/usr/bin/env python3
"""Generate independent MuJoCo FK/Jacobian goldens from the source URDF.

No rendering, stepping, DDS or robot. This validates math, not real contact.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
from export_va15_kinematics import vector


def quat_from_rpy(rpy):
    r, p, y = (v*.5 for v in rpy)
    cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return [cr*cp*cy+sr*sp*sy,sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy]


def reference_xml(urdf):
    root = ET.parse(urdf).getroot()
    joints = {n.get("name"):n for n in root.findall("joint")}
    links = {n.get("name"):n for n in root.findall("link")}
    mj = ET.Element("mujoco", model="source_urdf_leg_reference")
    ET.SubElement(mj,"compiler",angle="radian",autolimits="true")
    world = ET.SubElement(mj,"worldbody")
    base = ET.SubElement(world,"body",name="base_link")
    # No duplicated hand-written leg lengths: read transforms directly from
    # the source tree, independently of exported JSON and C++ FK.
    for leg in ("FR","FL","RR","RL"):
        parent = base
        for part in ("hip","thigh","calf"):
            item = joints[f"{leg}_{part}_joint"]
            origin = item.find("origin")
            parent = ET.SubElement(parent,"body",name=f"{leg}_{part}",
                pos=" ".join(map(str,vector(origin,"xyz"))),
                quat=" ".join(map(str,quat_from_rpy(vector(origin,"rpy")))))
            ET.SubElement(parent,"inertial",mass="1",pos="0 0 0",diaginertia=".01 .01 .01")
            limits = item.find("limit")
            ET.SubElement(parent,"joint",name=item.get("name"),type="hinge",
                axis=" ".join(map(str,vector(item.find("axis"),"xyz"))),
                range=limits.get("lower")+" "+limits.get("upper"))
        item = joints[leg+"_foot_joint"]
        origin = item.find("origin")
        foot = ET.SubElement(parent,"body",name=leg+"_foot",
            pos=" ".join(map(str,vector(origin,"xyz"))),
            quat=" ".join(map(str,quat_from_rpy(vector(origin,"rpy")))))
        collision = links[item.find("child").get("link")].find("collision")
        ET.SubElement(foot,"geom",name=leg+"_sphere",type="sphere",
            size=collision.find("geometry/sphere").get("radius"),
            pos=" ".join(map(str,vector(collision.find("origin"),"xyz"))))
    return ET.tostring(mj,encoding="unicode")


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf",type=Path)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--cases",type=int,default=64)
    args=parser.parse_args()
    model=mujoco.MjModel.from_xml_string(reference_xml(args.urdf))
    data=mujoco.MjData(model)
    rng=np.random.default_rng(20261020)
    cases=[]
    for _ in range(args.cases):
        q=np.array([rng.uniform(-.5,.5),rng.uniform(.2,1.5),rng.uniform(-2.6,-.6)]*4)
        q+=rng.uniform(-.03,.03,12)
        dq=rng.uniform(-1,1,12)
        quaternion=quat_from_rpy([rng.uniform(-.4,.4),rng.uniform(-.4,.4),rng.uniform(-math.pi,math.pi)])
        gyro=rng.uniform(-.3,.3,3)
        data.qpos[:]=q
        mujoco.mj_forward(model,data)
        rotate=np.zeros(9);mujoco.mju_quat2Mat(rotate,np.asarray(quaternion));rotate=rotate.reshape(3,3)
        positions=[];jacobians=[];heights=[];velocities=[]
        for leg in ("FR","FL","RR","RL"):
            geom=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_GEOM,leg+"_sphere")
            position=data.geom_xpos[geom].copy()
            jac=np.zeros((3,model.nv));jacrot=np.zeros_like(jac)
            mujoco.mj_jacGeom(model,data,jac,jacrot,geom)
            first=("FR","FL","RR","RL").index(leg)*3
            positions.append(position.tolist());jacobians.append(jac[:,first:first+3].tolist())
            velocities.append((-rotate@(jac@dq+np.cross(gyro,position))).tolist())
            heights.append(float(model.geom_size[geom,0]-(rotate@position)[2]))
        cases.append(dict(q=q.tolist(),dq=dq.tolist(),quaternion=quaternion,gyro=gyro.tolist(),
                          foot_position_base=positions,jacobian_base=jacobians,
                          no_slip_velocity_world=velocities,flat_ground_height=heights))
    report={"source_urdf_sha256":hashlib.sha256(args.urdf.read_bytes()).hexdigest(),
            "mujoco_version":mujoco.__version__,"shadow_only":True,"cases":cases}
    with args.output.open("x") as handle:json.dump(report,handle,indent=2,allow_nan=False)
    print(f"Generated {len(cases)} independent MuJoCo FK/Jacobian reference cases; no hardware acceptance.")


if __name__=="__main__":main()
