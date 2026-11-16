#!/usr/bin/env python3
"""Independent static-gravity/Jacobian goldens from source URDF + MuJoCo.

No scene stepping, rendering, network or robot. This is a math verification,
not sensor, inverse-dynamics, dynamic-contact or real stop qualification.
"""
import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from va15_support_candidate import Candidate, ORDER, export_model, vector


def quaternion(rpy):
    r,p,y = np.array(rpy)/2.
    cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return [cr*cp*cy+sr*sp*sy, sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy, cr*cp*sy-sr*sp*cy]


def reference(urdf, orientation):
    root = ET.parse(urdf).getroot()
    joints = {n.get("name"):n for n in root.findall("joint")}
    links = {n.get("name"):n for n in root.findall("link")}
    mj = ET.Element("mujoco")
    ET.SubElement(mj,"compiler", angle="radian", autolimits="true", fusestatic="false")
    ET.SubElement(mj,"option", gravity="0 0 -9.81")
    base = ET.SubElement(ET.SubElement(mj,"worldbody"),"body",name="base_link",quat=" ".join(map(str,orientation)))
    for name in ORDER:
        parent = base
        for part in ("hip","thigh","calf","foot"):
            joint = joints[name+"_"+part+"_joint"]
            origin = joint.find("origin")
            parent = ET.SubElement(parent,"body",name=name+"_"+part,
                pos=" ".join(map(str,vector(origin,"xyz"))),
                quat=" ".join(map(str,quaternion(vector(origin,"rpy")))))
            link = links[joint.find("child").get("link")]
            inertial, inertia = link.find("inertial"), link.find("inertial/inertia")
            ET.SubElement(parent,"inertial", mass=inertial.find("mass").get("value"),
                pos=inertial.find("origin").get("xyz"),
                fullinertia=" ".join(inertia.get(k) for k in ("ixx","iyy","izz","ixy","ixz","iyz")))
            # Source leg inertial axes are all unrotated. Refuse otherwise.
            if np.linalg.norm(vector(inertial.find("origin"),"rpy")) > 1e-12:
                raise ValueError("Rotated inertial axes need explicit reference conversion")
            if part != "foot":
                limits = joint.find("limit")
                ET.SubElement(parent,"joint",name=name+"_"+part+"_joint",type="hinge",
                    axis=joint.find("axis").get("xyz"),range=limits.get("lower")+" "+limits.get("upper"))
            else:
                ET.SubElement(parent,"site",name=name+"_center",pos="0 0 0",size=".001")
    return mujoco.MjModel.from_xml_string(ET.tostring(mj,encoding="unicode"))


def validate(urdf, cases=64):
    candidate, rng = Candidate(export_model(urdf)), np.random.default_rng(20261020)
    max_error, max_gravity_error = 0., 0.
    for _ in range(cases):
        orient = quaternion([rng.uniform(-.2,.2), rng.uniform(-.2,.2), rng.uniform(-math.pi,math.pi)])
        model = reference(urdf,orient)
        data = mujoco.MjData(model)
        q = np.array([rng.uniform(-.3,.3),rng.uniform(.5,1.1),rng.uniform(-2.,-.8)]*4)
        data.qpos[:] = q
        mujoco.mj_forward(model,data)
        matrix = np.zeros(9); mujoco.mju_quat2Mat(matrix,np.array(orient));rotation=matrix.reshape(3,3)
        row = dict(low_age=.001,sequence_invalid=False)
        for key,values in (("q",q),("dq",np.zeros(12)),("ddq",np.zeros(12)),("gyro",np.zeros(3)),
                ("quat",orient),("accel",rotation.T@np.array([0.,0.,9.81])),("lost",[0]*12),("mode",[10]*12)):
            row.update({f"{key}_{i}":float(v) for i,v in enumerate(values)})
        expected = []
        for i,name in enumerate(ORDER):
            jp,jr = np.zeros((3,12)),np.zeros((3,12))
            mujoco.mj_jacSite(model,data,jp,jr,mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_SITE,name+"_center"))
            force = np.array([rng.uniform(-10.,10.),rng.uniform(-10.,10.),rng.uniform(0.,260.)])
            # Occasionally exactly unload one leg; preserve sign and ordering.
            if rng.random() < .25: force[:] = 0.
            expected.append(force)
            torque = data.qfrc_bias[i*3:i*3+3] - jp[:,i*3:i*3+3].T@force
            for j in range(3): row[f"tau_{i*3+j}"] = torque[j]
            _,_,gravity = candidate.leg(i,q[i*3:i*3+3],rotation.T@np.array([0.,0.,-9.81]))
            max_gravity_error = max(max_gravity_error,float(max(abs(gravity-data.qfrc_bias[i*3:i*3+3]))))
        result = candidate.evaluate(row)
        if not result["quasi_static"]: raise AssertionError(result)
        max_error = max(max_error,float(np.max(np.abs(np.array(result["force_equivalent_world"])-expected))))
        assert result["control_authorized"] is False
    if max_error > 1e-8 or max_gravity_error > 1e-8: raise AssertionError((max_error,max_gravity_error))
    return dict(cases=cases, max_force_error=max_error, max_gravity_torque_error=max_gravity_error,
                reference="MuJoCo static bias/Jacobian independently built from source URDF",
                source_urdf_sha256=candidate.model["source_urdf_sha256"],
                control_authorized=False, hardware_qualified=False)


if __name__ == "__main__":
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("urdf",type=Path)
    args=p.parse_args()
    print(json.dumps(validate(args.urdf),indent=2))
