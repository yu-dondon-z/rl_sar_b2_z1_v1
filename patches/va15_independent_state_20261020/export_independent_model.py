#!/usr/bin/env python3
"""Explicit URDF inertial/IMU model for an unqualified, read-only observer."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent / "../va15_support_observer_20261020"))
from va15_support_candidate import export_model, vector, rpy_rotation


def export(urdf):
    model = export_model(urdf)
    tree = ET.parse(urdf).getroot()
    links = {link.get("name"):link for link in tree.findall("link")}
    joints = {joint.get("name"):joint for joint in tree.findall("joint")}
    model.update(schema="va15_independent_state_v1", hardware_qualified=False,
                 acceleration_source="filtered finite difference of dq; reported ddq unused")
    imu = joints["joint_imu"]
    if (imu.get("type") != "fixed" or imu.find("parent").get("link") != "base_link"
            or imu.find("child").get("link") != "imu_link"):
        raise ValueError("Unexpected IMU topology")
    # Current DDS convention already uses base/body axes, and URDF agrees.
    # Refuse different orientation instead of silently double-rotating IMU.
    if np.linalg.norm(vector(imu.find("origin"), "rpy")) > 1e-12:
        raise ValueError("Nontrivial IMU axis transform needs explicit verification")
    model["imu_position_base"] = vector(imu.find("origin"),"xyz").tolist()
    for leg in model["legs"]:
        for body, part in zip(leg["bodies"], ("hip","thigh","calf","foot")):
            inertial = links[leg["name"]+"_"+part].find("inertial")
            i = inertial.find("inertia")
            xx, yy, zz, xy, xz, yz = [float(i.get(k)) for k in ("ixx","iyy","izz","ixy","ixz","iyz")]
            rotation = rpy_rotation(vector(inertial.find("origin"),"rpy"))
            matrix = rotation @ np.array([[xx,xy,xz],[xy,yy,yz],[xz,yz,zz]]) @ rotation.T
            values = np.linalg.eigvalsh(matrix)
            if min(values) <= 0 or 2*max(values) > sum(values)+1e-8:
                raise ValueError("Nonphysical link inertia")
            body["inertia_com_link"] = matrix.tolist()
    return model


if __name__ == "__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf",type=Path)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists(): parser.error("Refusing existing output")
    model=export(args.urdf)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open("x") as handle: json.dump(model,handle,indent=2,allow_nan=False)
    print("Exported",args.output,"source SHA256",model["source_urdf_sha256"],"control_authorized=false")
