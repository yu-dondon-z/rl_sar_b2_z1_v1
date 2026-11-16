"""Explicit simulation-only candidate for joint speed constraints.

MuJoCo has no PhysX maxJointVelocity equivalent. Two native velocity actuators
per joint implement unilateral dissipative brakes: the upper brake can only
apply negative force, and the lower brake only positive force. Their combined
law is -D*sign(v)*max(abs(v)-limit, 0), with a finite constraint reaction cap.
They do not change the robot actuator gains, action, targets, or encoder data.

MuJoCo 3.2.3's implicit derivative includes a velocity actuator even when its
force is clipped to zero. Inactive brakes therefore have BOTH gain and bias
disabled; otherwise their ghost derivative would suppress ordinary motion.
This compliant approximation permits the first bound crossing and is not a
hard velocity limit or PhysX solver port. Keep it explicitly experimental.
"""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


PREFIX = "vd1_speed_"


def brake_names(joint_name):
    return PREFIX + "upper_" + joint_name, PREFIX + "lower_" + joint_name


def _limits(value, count):
    limits = np.asarray(value, dtype=float)
    if limits.shape != (count,) or not np.isfinite(limits).all() or np.any(limits <= 0):
        raise ValueError("Velocity constraint limits must be finite positive joint bounds")
    return limits


def add_native_velocity_brakes(scene_path, joint_names, velocity_limits, max_reaction_force=1e6):
    """Append independent brakes to a generated standalone MJCF, in place.

    The original motor definitions are retained. Placeholder kv=1 is replaced
    from the initial effective inertia before the helper advances physics.
    Keyframe controls are extended with the fixed +/-limit brake references.
    """
    names = tuple(joint_names)
    limits = _limits(velocity_limits, len(names))
    cap = float(max_reaction_force)
    if (not names or len(set(names)) != len(names) or not np.isfinite(cap) or cap <= 0):
        raise ValueError("Expected unique joints and a positive finite reaction cap")
    path = Path(scene_path)
    tree = ET.parse(str(path))
    root = tree.getroot()
    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")
    if any(str(item.get("name", "")).startswith(PREFIX) for item in actuator):
        raise ValueError("Native velocity brakes have already been added")
    old_count = len(actuator)
    references = []
    for name, limit in zip(names, limits):
        upper, lower = brake_names(name)
        for brake, bounds, reference in ((upper, (-cap, 0), limit), (lower, (0, cap), -limit)):
            ET.SubElement(actuator, "velocity", name=brake, joint=name, kv="1",
                          forcelimited="true", forcerange="%.12g %.12g" % bounds,
                          ctrllimited="false")
            references.append(float(reference))
    for key in root.findall("keyframe/key"):
        controls = np.fromstring(key.get("ctrl", ""), sep=" ").tolist()
        if len(controls) > old_count:
            raise ValueError("Keyframe has too many controls before adding velocity brakes")
        controls.extend([0.] * (old_count - len(controls)))
        key.set("ctrl", " ".join("%.12g" % v for v in controls + references))
    temporary = path.with_name(path.name + ".native_velocity.tmp")
    tree.write(str(temporary), encoding="utf-8", xml_declaration=True)
    temporary.replace(path)
    return path


def validate_native_velocity_brakes(model, joint_ids, velocity_limits):
    """Return upper/lower brake actuator IDs after checking unilateral force."""
    joints = np.asarray(joint_ids, dtype=int)
    limits = _limits(velocity_limits, len(joints))
    pairs = []
    expected_names = set()
    for joint, limit in zip(joints, limits):
        if model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_HINGE:
            raise ValueError("Velocity brakes support named scalar hinges only")
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(joint))
        pair = []
        for upper, brake in zip((True, False), brake_names(name)):
            actuator = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, brake)
            if actuator < 0:
                raise ValueError("Missing unilateral velocity brake: " + brake)
            expected_names.add(brake)
            force = model.actuator_forcerange[actuator]
            gain = model.actuator_gainprm[actuator, 0]
            if (model.actuator_trntype[actuator] != mujoco.mjtTrn.mjTRN_JOINT
                    or model.actuator_trnid[actuator, 0] != joint
                    or not np.array_equal(model.actuator_gear[actuator], [1, 0, 0, 0, 0, 0])
                    or model.actuator_dyntype[actuator] != mujoco.mjtDyn.mjDYN_NONE
                    or model.actuator_gaintype[actuator] != mujoco.mjtGain.mjGAIN_FIXED
                    or model.actuator_biastype[actuator] != mujoco.mjtBias.mjBIAS_AFFINE
                    or not np.isfinite(gain) or gain < 0
                    or not np.allclose(model.actuator_biasprm[actuator, :3], [0, 0, -gain])
                    or not model.actuator_forcelimited[actuator]
                    or model.actuator_ctrllimited[actuator]
                    or not np.isfinite(force).all()
                    or (upper and not (force[0] < 0 and force[1] == 0))
                    or (not upper and not (force[0] == 0 and force[1] > 0))):
                raise ValueError("Wrong unilateral velocity brake contract: " + brake)
            pair.append(actuator)
        pairs.append(pair)
    actual_names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
                    for index in range(model.nu)
                    if str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)).startswith(PREFIX)}
    if actual_names != expected_names:
        raise ValueError("Unexpected extra native velocity brake actuator")
    return np.asarray(pairs, dtype=int)


class NativeVelocityLimitBrakes:
    """Integrate using unilateral brake active sets, separate from robot PD.

    D is initial effective inertia / regularization_s, not a robot motor gain.
    Constraint reaction is capped separately from the 120/200/320 Nm motors.
    The helper exposes diagnostics only; it does not supply actor observations.
    """

    def __init__(self, model, joint_ids, velocity_limits, regularization_s=1e-5):
        self.model = model
        self.joints = np.asarray(joint_ids, dtype=int)
        self.dofs = np.asarray(model.jnt_dofadr[self.joints], dtype=int)
        self.limits = _limits(velocity_limits, len(self.joints)).copy()
        self.regularization_s = float(regularization_s)
        if not np.isfinite(self.regularization_s) or self.regularization_s <= 0:
            raise ValueError("Constraint regularization time must be finite and positive")
        self.brake_ids = validate_native_velocity_brakes(model, self.joints, self.limits)
        self.references = np.column_stack((self.limits, -self.limits))
        self.damping = None
        self.substeps, self.active_substeps = 0, 0
        self.max_reaction_force, self.max_speed_ratio = 0., 0.
        self.max_positive_linearization_power = 0.
        self.last = {}

    def initialize(self, data):
        """Set constant brake compliance from initial mass, without stepping."""
        if self.damping is not None:
            return
        mujoco.mj_forward(self.model, data)
        mass = np.empty((self.model.nv, self.model.nv))
        mujoco.mj_fullM(self.model, mass, data.qM)
        inverse = np.linalg.inv(mass)
        effective = 1. / np.diag(inverse)[self.dofs]
        if not np.isfinite(effective).all() or np.any(effective <= 0):
            raise ValueError("Nonpositive initial effective joint inertia")
        self.damping = effective / self.regularization_s
        self.model.actuator_gainprm[self.brake_ids, 0] = 0.
        self.model.actuator_biasprm[self.brake_ids, 2] = 0.
        data.ctrl[self.brake_ids] = self.references

    def prepare(self, data):
        """Disable inactive derivatives; activate only an actual bound excess.

        Never preactivate from a predicted acceleration while the real joint
        is inside its bounds: a clipped-zero force still contributes a false
        derivative in MuJoCo's implicit integrator. First-crossing overshoot is
        an explicit limitation of this per-substep active-set approximation.
        """
        self.initialize(data)
        data.ctrl[self.brake_ids] = self.references
        velocity = data.qvel[self.dofs]
        active = np.column_stack((velocity > self.limits, velocity < -self.limits))
        damping = active*self.damping[:, None]
        self.model.actuator_gainprm[self.brake_ids, 0] = damping
        self.model.actuator_biasprm[self.brake_ids, 2] = -damping

    def step(self, data):
        self.prepare(data)
        before = data.qvel[self.dofs].copy()
        mujoco.mj_step(self.model, data)
        forces = data.actuator_force[self.brake_ids].copy()
        summed = forces.sum(axis=1)
        # MuJoCo reports the force at this implicit step's linearization state,
        # not an independently measured hardware torque or exact work integral.
        power = summed * before
        self.max_positive_linearization_power = max(self.max_positive_linearization_power,
                                                     float(max(0., power.max())))
        self.max_reaction_force = max(self.max_reaction_force, float(np.abs(forces).max()))
        after = data.qvel[self.dofs].copy()
        ratio = float(np.max(np.abs(after) / self.limits))
        self.max_speed_ratio = max(self.max_speed_ratio, ratio)
        self.substeps += 1
        self.active_substeps += int(np.any(np.abs(forces) > 1e-10))
        self.last = dict(brake_force_at_linearization=forces.tolist(),
                         linearization_power_w=power.tolist(),
                         max_velocity_ratio=ratio)

    def summary(self):
        return dict(model="native unilateral velocity brakes; compliant experimental approximation",
                    regularization_s=self.regularization_s, damping_from_initial_effective_inertia=True,
                    brake_damping=self.damping.tolist() if self.damping is not None else None,
                    substeps=self.substeps, active_substeps=self.active_substeps,
                    max_brake_reaction_force_at_linearization_nm=self.max_reaction_force,
                    max_positive_linearization_power_w=self.max_positive_linearization_power,
                    max_velocity_ratio_after_step=self.max_speed_ratio,
                    reaction_cap_independent_of_motor_effort=True,
                    inactive_gain_and_bias_disabled=True, activation_from_actual_velocity_only=True,
                    robot_pd_gains_changed=False, qpos_qvel_assigned=False, encoder_clipped=False,
                    hard_bound_guaranteed=False)
