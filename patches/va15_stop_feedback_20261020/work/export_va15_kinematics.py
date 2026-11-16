#!/usr/bin/env python3
"""Extract native DDS leg chains from an explicit URDF; never infer contact."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def vector(element, key, default="0 0 0"):
    values = [float(v) for v in (element.get(key, default) if element is not None else default).split()]
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError("Invalid vector: " + key)
    return values


def export(urdf):
    data = Path(urdf).read_bytes()
    tree = ET.fromstring(data)
    joints = {n.get("name"): n for n in tree.findall("joint")}
    links = {n.get("name"): n for n in tree.findall("link")}
    model = {"schema": "va15_leg_kinematics_v1", "dds_order": ["FR", "FL", "RR", "RL"],
             "source_urdf_sha256": hashlib.sha256(data).hexdigest(),
             "units": "metres/radians/seconds; base=base_link; gyro=body; quaternion=wxyz world<-base",
             "control_authorized": False, "contact_inference": "none", "legs": []}
    for leg in model["dds_order"]:
        items, parent = [], "base_link"
        for suffix in ("hip", "thigh", "calf"):
            name = leg + "_" + suffix + "_joint"
            joint = joints[name]
            if joint.get("type") != "revolute" or joint.find("parent").get("link") != parent:
                raise ValueError("Unexpected native leg topology: " + name)
            parent = joint.find("child").get("link")
            axis = vector(joint.find("axis"), "xyz")
            if abs(sum(v*v for v in axis) - 1) > 1e-6:
                raise ValueError("Expected a normalized joint axis")
            limits = joint.find("limit")
            items.append({"name": name, "origin": vector(joint.find("origin"), "xyz"),
                          "rpy": vector(joint.find("origin"), "rpy"), "axis": axis,
                          "lower": float(limits.get("lower")), "upper": float(limits.get("upper"))})
        fixed = joints[leg + "_foot_joint"]
        if fixed.get("type") != "fixed" or fixed.find("parent").get("link") != parent:
            raise ValueError("Unexpected foot topology")
        link = links[fixed.find("child").get("link")]
        collisions = link.findall("collision")
        if len(collisions) != 1 or collisions[0].find("geometry/sphere") is None:
            raise ValueError("Shadow model requires one explicit spherical foot collision")
        # Current B2 has zero collision-center offset, so its orientation is
        # irrelevant. Refuse a different chain rather than silently flatten it.
        if any(abs(v) > 1e-12 for v in vector(fixed.find("origin"), "rpy")) or any(
                abs(v) > 1e-12 for v in vector(collisions[0].find("origin"), "xyz")):
            raise ValueError("Nontrivial fixed-foot transform needs an explicit exporter update")
        radius = float(collisions[0].find("geometry/sphere").get("radius"))
        model["legs"].append({"name": leg, "joints": items,
                              "tip": vector(fixed.find("origin"), "xyz"), "radius": radius})
    return model


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("urdf", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x") as handle:
        json.dump(export(args.urdf), handle, indent=2, allow_nan=False)
    print("Exported shadow-only DDS leg geometry:", args.output)
