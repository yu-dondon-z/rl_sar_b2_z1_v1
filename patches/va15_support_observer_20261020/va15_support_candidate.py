"""Read-only, quasi-static support investigation. NEVER a motion authority.

With upward ground force f: tau = g_leg(q) - J_foot(q).T @ f.
The estimate ignores joint/base acceleration, rotor/friction torques and extra
leg contacts. It cannot certify dynamic contacts, all-feet slip or suspension
with external constraints. Even a plausible estimate is NOT a contact sensor.
"""
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

ORDER = ("FR", "FL", "RR", "RL")
SCHEMA = "va15_static_support_investigation_v1"
CONTROL_AUTHORIZED = False  # No YAML/CLI option can change this boundary.


def vector(node, attribute, default="0 0 0"):
    value = np.fromstring(node.get(attribute, default) if node is not None else default, sep=" ")
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError("Invalid vector: " + attribute)
    return value


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    if not np.isfinite(axis).all() or abs(np.linalg.norm(axis) - 1.) > 1e-6:
        raise ValueError("Non-unit joint axis")
    x, y, z = axis
    cross = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * cross + (1. - math.cos(angle)) * cross @ cross


def rpy_rotation(angles):
    r, p, y = angles
    return (axis_rotation([0., 0., 1.], y) @ axis_rotation([0., 1., 0.], p)
            @ axis_rotation([1., 0., 0.], r))


def quaternion_rotation(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.isfinite(q).all() or not .9 <= q @ q <= 1.1:
        raise ValueError("Invalid IMU quaternion")
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def export_model(urdf):
    data = Path(urdf).read_bytes()
    tree = ET.fromstring(data)
    joints = {n.get("name"): n for n in tree.findall("joint")}
    links = {n.get("name"): n for n in tree.findall("link")}
    model = dict(schema=SCHEMA, dds_order=list(ORDER), control_authorized=False,
                 source_urdf_sha256=hashlib.sha256(data).hexdigest(), legs=[],
                 urdf_total_mass=sum(float(n.get("value")) for n in tree.findall("link/inertial/mass")))
    for name in ORDER:
        leg, parent = dict(name=name, joints=[], bodies=[]), "base_link"
        for part in ("hip", "thigh", "calf", "foot"):
            joint = joints[name + "_" + part + "_joint"]
            if joint.find("parent").get("link") != parent:
                raise ValueError("Unexpected chain topology")
            parent = joint.find("child").get("link")
            origin = joint.find("origin")
            link = links[parent]
            inertial = link.find("inertial")
            mass = float(inertial.find("mass").get("value"))
            if not math.isfinite(mass) or mass <= 0.:
                raise ValueError("Invalid link mass")
            leg["bodies"].append(dict(mass=mass, com=vector(inertial.find("origin"), "xyz").tolist()))
            if part != "foot":
                if joint.get("type") != "revolute":
                    raise ValueError("Expected revolute leg joint")
                limit = joint.find("limit")
                leg["joints"].append(dict(origin=vector(origin, "xyz").tolist(),
                    rpy=vector(origin, "rpy").tolist(), axis=vector(joint.find("axis"), "xyz").tolist(),
                    lower=float(limit.get("lower")), upper=float(limit.get("upper"))))
            else:
                collision = link.findall("collision")
                if (joint.get("type") != "fixed" or len(collision) != 1 or
                        collision[0].find("geometry/sphere") is None or
                        np.linalg.norm(vector(origin, "rpy")) > 1e-12 or
                        np.linalg.norm(vector(collision[0].find("origin"), "xyz")) > 1e-12):
                    raise ValueError("Unexpected spherical foot topology")
                leg.update(tip=vector(origin, "xyz").tolist(),
                           radius=float(collision[0].find("geometry/sphere").get("radius")))
        model["legs"].append(leg)
    return model


class Candidate:
    def __init__(self, model):
        if (model.get("schema") != SCHEMA or model.get("dds_order") != list(ORDER)
                or model.get("control_authorized") is not False
                or len(model.get("source_urdf_sha256", "")) != 64 or len(model["legs"]) != 4):
            raise ValueError("Invalid model provenance/order/authority")
        for index, leg in enumerate(model["legs"]):
            if (leg["name"] != ORDER[index] or len(leg["joints"]) != 3 or len(leg["bodies"]) != 4
                    or not math.isfinite(leg["radius"]) or not 0. < leg["radius"] <= .1):
                raise ValueError("Invalid leg model")
            for body in leg["bodies"]:
                if not math.isfinite(body["mass"]) or body["mass"] <= 0. or not np.isfinite(body["com"]).all():
                    raise ValueError("Invalid link inertia")
        self.model = model

    def leg(self, index, q, gravity_base):
        leg = self.model["legs"][index]
        rotation, position = np.eye(3), np.zeros(3)
        origins, axes, centers = [], [], []
        for j, item in enumerate(leg["joints"]):
            if not math.isfinite(q[j]) or not item["lower"] <= q[j] <= item["upper"]:
                raise ValueError("Joint outside URDF limits")
            position = position + rotation @ np.array(item["origin"])
            rotation = rotation @ rpy_rotation(item["rpy"])
            origins.append(position.copy())
            axes.append(rotation @ np.array(item["axis"]))
            rotation = rotation @ axis_rotation(item["axis"], q[j])
            centers.append(position + rotation @ np.array(leg["bodies"][j]["com"]))
        foot = position + rotation @ np.array(leg["tip"])
        centers.append(foot + rotation @ np.array(leg["bodies"][3]["com"]))
        jacobian = np.column_stack([np.cross(axis, foot - origin) for axis, origin in zip(axes, origins)])
        gravity_torque = np.zeros(3)
        for j in range(3):
            for body in range(j, 4):
                gravity_torque[j] -= (np.cross(axes[j], centers[body] - origins[j])
                    @ (leg["bodies"][body]["mass"] * gravity_base))
        return foot, jacobian, gravity_torque

    def evaluate(self, row):
        result = dict(input_valid=False, model_valid=False, quasi_static=False,
                      control_authorized=CONTROL_AUTHORIZED, motion_qualified=False,
                      reason="missing/invalid input")
        try:
            def array(key, count):
                values = np.array([float(row[f"{key}_{i}"]) for i in range(count)])
                if not np.isfinite(values).all():
                    raise ValueError("Non-finite " + key)
                return values
            q, dq, tau = array("q", 12), array("dq", 12), array("tau", 12)
            rotation = quaternion_rotation(array("quat", 4))
            gyro = array("gyro", 3)
            age = float(row["low_age"])
            if not math.isfinite(age) or not 0. <= age <= .1:
                raise ValueError("Stale lowstate")
            if str(row.get("sequence_invalid", False)).lower() in ("true", "1"):
                raise ValueError("Invalid lowstate sequence")
            if max(np.abs(tau)) > 360.:
                raise ValueError("Torque outside conservative B2 envelope")
            result["input_valid"] = True
            forces, heights, velocities, conditions = [], [], [], []
            for i in range(4):
                foot, jac, g = self.leg(i, q[i*3:i*3+3], rotation.T @ np.array([0., 0., -9.81]))
                condition = float(np.linalg.cond(jac))
                if not math.isfinite(condition) or condition > 30.:
                    raise ValueError("Ill-conditioned foot Jacobian")
                # Positive world-Z means upward reaction, not actuator action.
                forces.append(rotation @ np.linalg.solve(jac.T, g - tau[i*3:i*3+3]))
                heights.append(self.model["legs"][i]["radius"] - (rotation @ foot)[2])
                velocities.append(-rotation @ (jac @ dq[i*3:i*3+3] + np.cross(gyro, foot)))
                conditions.append(condition)
            forces, velocities = np.array(forces), np.array(velocities)
            if not np.isfinite(forces).all() or not np.isfinite(velocities).all():
                raise ValueError("Nonfinite force/velocity reconstruction")
            velocity = np.median(velocities, axis=0)
            result.update(model_valid=True, force_equivalent_world=forces.tolist(),
                          gravity_only_compensation=True, jacobian_condition=conditions,
                          height_hypothesis=float(np.median(heights)),
                          height_spread=float(np.ptp(heights)),
                          no_slip_velocity_hypothesis=velocity.tolist(),
                          velocity_spread=float(np.max(np.linalg.norm(velocities-velocity, axis=1))),
                          max_joint_speed=float(max(np.abs(dq))), gyro_norm=float(np.linalg.norm(gyro)))
            # Old captures lack acceleration and fault fields: allow numerical
            # investigation, NEVER upgrade them to stationary qualification.
            accel = array("accel", 3)
            ddq = array("ddq", 12)
            def integer(key):
                value = float(row[key])
                if not math.isfinite(value) or value < 0. or not value.is_integer():
                    raise ValueError("Invalid integer field: " + key)
                return int(value)
            if any(integer(f"lost_{i}") != 0 for i in range(12)):
                raise ValueError("Motor loss counter nonzero; inspect transport")
            if any(integer(f"mode_{i}") != 10 for i in range(12)):
                raise ValueError("Unexpected motor mode")
            # Specific-force sign is checked too; a reversed IMU convention is
            # rejected rather than taking abs() of its vertical component.
            accel_residual = np.linalg.norm(rotation @ accel - np.array([0., 0., 9.81]))
            result["acceleration_residual"] = float(accel_residual)
            result["quasi_static"] = bool(max(np.abs(dq)) <= .25 and np.linalg.norm(gyro) <= .15
                and max(np.abs(ddq)) <= 3. and accel_residual <= 1.5)
            result["reason"] = ("quasi-static force hypothesis; NOT contact qualification"
                if result["quasi_static"] else "dynamic/IMU uncertainty; static approximation not applicable")
        except (KeyError, ValueError, TypeError, np.linalg.LinAlgError) as error:
            result["reason"] = str(error)
        return result


def load_candidate(path):
    return Candidate(json.loads(Path(path).read_text()))
