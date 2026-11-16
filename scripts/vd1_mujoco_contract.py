"""VD1's named sensor boundary and unchanged joint control contract.

The adapter reads *only* joint encoder and ideal IMU sensor outputs from
``data.sensordata``. It does not retain ``MjData`` or expose root/world state.
The simulated orientation sensor is an ideal IMU attitude estimate; only its
unit gravity projection is used. It is not a calibrated hardware fusion model.

Call ``apply_targets`` once every 5 ms. It sets leg motor torques and Z1
position targets; it never advances physics. The runner may integrate five
1 ms MuJoCo substeps per control update, retaining the upstream 50 Hz actor.
"""

from __future__ import annotations

import numpy as np
import mujoco


ACTOR_JOINT_NAMES = (
    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
    "joint1", "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
    "joint2", "joint3", "joint4", "joint5", "joint6",
)
ARM_JOINT_NAMES = tuple("joint%d" % index for index in range(1, 7))
KP, KD = 160.0, 5.0
PD_PERIOD_S, ACTOR_PERIOD_S = 0.005, 0.02


def _finite_array(value, shape, description):
    result = np.asarray(value, dtype=np.float64)
    if result.shape != shape or not np.isfinite(result).all():
        raise ValueError(description + " must have finite shape " + str(shape))
    return result


def _named_id(model, kind, name):
    number = mujoco.mj_name2id(model, kind, name)
    if number < 0:
        raise ValueError("Missing VD1 model component: " + name)
    return number


class VD1MujocoAdapter:
    """Bind actor order by name, independently of MJCF's depth-first order.

    All public arrays ending in ``18`` use ``ACTOR_JOINT_NAMES`` order. The
    original velocity bounds are diagnostic: MuJoCo has no equivalent of the
    PhysX solver's joint velocity cap. No state, target, or timestamp is clipped
    to conceal that engine difference.
    """

    def __init__(self, model, *, physx_implicit_position=False):
        if (model.nq, model.nv) != (25, 24) or model.nu not in (18, 54):
            raise ValueError("VD1 requires a free base and 18 robot actuators, optionally 36 named velocity brakes")
        self.arm_mask18 = np.array([name in ARM_JOINT_NAMES for name in ACTOR_JOINT_NAMES])
        self.effort_limit18 = np.array([
            120.0 if arm else 320.0 if "calf" in name else 200.0
            for name, arm in zip(ACTOR_JOINT_NAMES, self.arm_mask18)
        ])
        self.velocity_limit18 = np.array([
            3.1415 if arm else 14.0 if "calf" in name else 23.0
            for name, arm in zip(ACTOR_JOINT_NAMES, self.arm_mask18)
        ])
        self.default_q18 = np.array([
            dict(zip(ARM_JOINT_NAMES, (0.0, 2.55, -1.6, -1.4, 0.0, 0.0)))[name]
            if arm else -1.5 if "calf" in name else
            (0.8 if name.startswith("F") else 1.0) if "thigh" in name else
            (0.1 if name[1] == "L" else -0.1)
            for name, arm in zip(ACTOR_JOINT_NAMES, self.arm_mask18)
        ])
        joint_ids, actuator_ids, q_sensor_ids, qd_sensor_ids = [], [], [], []
        for name, arm, effort in zip(ACTOR_JOINT_NAMES, self.arm_mask18, self.effort_limit18):
            joint = _named_id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if model.jnt_type[joint] != mujoco.mjtJoint.mjJNT_HINGE:
                raise ValueError("VD1 joint must be a scalar hinge: " + name)
            actuator = _named_id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name + "_motor")
            if (model.actuator_trntype[actuator] != mujoco.mjtTrn.mjTRN_JOINT
                    or model.actuator_trnid[actuator, 0] != joint
                    or not np.array_equal(model.actuator_gear[actuator], [1, 0, 0, 0, 0, 0])):
                raise ValueError("Wrong VD1 actuator joint transmission: " + name)
            if (model.actuator_dyntype[actuator] != mujoco.mjtDyn.mjDYN_NONE
                    or model.actuator_gaintype[actuator] != mujoco.mjtGain.mjGAIN_FIXED
                    or not model.actuator_forcelimited[actuator]
                    or not np.allclose(model.actuator_forcerange[actuator], [-effort, effort])
                    or model.actuator_ctrllimited[actuator]):
                raise ValueError("Wrong VD1 actuator dynamics or force limits: " + name)
            expected_bias = [0, -KP, -KD] if arm else [0, 0, 0]
            expected_type = mujoco.mjtBias.mjBIAS_AFFINE if arm else mujoco.mjtBias.mjBIAS_NONE
            if (model.actuator_biastype[actuator] != expected_type
                    or not np.isclose(model.actuator_gainprm[actuator, 0], KP if arm else 1)
                    or not np.allclose(model.actuator_biasprm[actuator, :3], expected_bias)):
                raise ValueError("Wrong VD1 explicit/implicit PD contract: " + name)
            dof = model.jnt_dofadr[joint]
            if (not np.isclose(model.dof_damping[dof], 0)
                    or not np.isclose(model.dof_armature[dof], 0)
                    or not np.isclose(model.dof_frictionloss[dof], 0.01)):
                raise ValueError("VD1 joint has extra damping, armature, or friction: " + name)
            q_sensor_ids.append(self._sensor(model, name + "_pos", mujoco.mjtSensor.mjSENS_JOINTPOS,
                                             1, mujoco.mjtObj.mjOBJ_JOINT, joint))
            qd_sensor_ids.append(self._sensor(model, name + "_vel", mujoco.mjtSensor.mjSENS_JOINTVEL,
                                              1, mujoco.mjtObj.mjOBJ_JOINT, joint))
            joint_ids.append(joint)
            actuator_ids.append(actuator)
        if len(set(actuator_ids)) != 18:
            raise ValueError("VD1 actuators must each control a unique named joint")
        self.joint_ids18 = np.array(joint_ids, dtype=int)
        self.actuator_ids18 = np.array(actuator_ids, dtype=int)
        if model.nu == 54:
            from vd1_mujoco_velocity_solver import validate_native_velocity_brakes
            validate_native_velocity_brakes(model, self.joint_ids18, self.velocity_limit18)
        self._q_addresses = np.array(q_sensor_ids, dtype=int)
        self._qd_addresses = np.array(qd_sensor_ids, dtype=int)
        site = _named_id(model, mujoco.mjtObj.mjOBJ_SITE, "imu")
        base = _named_id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        if (model.site_bodyid[site] != base
                or not np.allclose(model.site_quat[site], [1, 0, 0, 0])):
            raise ValueError("VD1 IMU axes must coincide with base_link axes")
        self._gyro_address = self._sensor(model, "imu_gyro", mujoco.mjtSensor.mjSENS_GYRO,
                                          3, mujoco.mjtObj.mjOBJ_SITE, site)
        self._orientation_address = self._sensor(model, "imu_orientation", mujoco.mjtSensor.mjSENS_FRAMEQUAT,
                                                 4, mujoco.mjtObj.mjOBJ_SITE, site)
        orientation = _named_id(model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_orientation")
        if model.sensor_refid[orientation] != -1:
            raise ValueError("VD1 ideal IMU orientation must use the inertial reference")
        # Validate the unmodified 160/5 source contract above before applying
        # this explicit numerical conversion. For a linear unsaturated drive,
        # PhysX implicit Euler includes h**2*kp in the denominator; MuJoCo's
        # implicitfast includes only velocity derivatives. kv=kd+h*kp supplies
        # that position term without changing the source continuous PD gains.
        self.physx_implicit_position = bool(physx_implicit_position)
        self.source_kp18 = np.full(18, KP)
        self.source_kd18 = np.full(18, KD)
        self.arm_discrete_kd18 = np.full(18, KD)
        if self.physx_implicit_position:
            if model.opt.integrator not in (mujoco.mjtIntegrator.mjINT_IMPLICIT,
                                            mujoco.mjtIntegrator.mjINT_IMPLICITFAST):
                raise ValueError("Implicit position conversion requires MuJoCo implicit integration")
            discrete = KD + model.opt.timestep*KP
            self.arm_discrete_kd18[self.arm_mask18] = discrete
            model.actuator_biasprm[self.actuator_ids18[self.arm_mask18], 2] = -discrete

    @staticmethod
    def _sensor(model, name, kind, dimension, object_type, object_id):
        sensor = _named_id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if (model.sensor_type[sensor] != kind or model.sensor_dim[sensor] != dimension
                or model.sensor_objtype[sensor] != object_type
                or model.sensor_objid[sensor] != object_id):
            raise ValueError("Wrong VD1 sensor type, size, or measured object: " + name)
        return int(model.sensor_adr[sensor])

    def _read_encoders(self, data):
        q = _finite_array(data.sensordata[self._q_addresses], (18,), "joint encoders").copy()
        qd = _finite_array(data.sensordata[self._qd_addresses], (18,), "joint velocities").copy()
        return q, qd

    def read_proprioception(self, data):
        """Return only base-frame IMU and measured joints, in actor order."""
        q, qd = self._read_encoders(data)
        address = self._gyro_address
        gyro = _finite_array(data.sensordata[address:address + 3], (3,), "IMU gyro").copy()
        address = self._orientation_address
        attitude = _finite_array(data.sensordata[address:address + 4], (4,), "IMU attitude")
        norm = float(np.linalg.norm(attitude))
        if abs(norm - 1.0) > 1e-3:
            raise ValueError("IMU attitude must be a unit wxyz quaternion")
        w, x, y, z = attitude / norm
        # R(attitude).T @ [0, 0, -1]; no yaw or world position is returned.
        gravity = np.array((2*(w*y - x*z), -2*(y*z + w*x), 2*(x*x + y*y) - 1))
        return {"gyro_b": gyro, "gravity_b": gravity, "q": q, "qd": qd}

    def apply_targets(self, data, target18):
        """Set controls; return diagnostics without stepping or editing state.

        ``effort18`` contains the actual commanded leg torque and an estimated
        instantaneous arm PD torque. MuJoCo's native position servo is integrated
        implicitly; reading its actual force belongs in independent evaluation.
        No diagnostic feeds back into the actor or its targets.
        """
        target = _finite_array(target18, (18,), "joint targets")
        q, qd = self._read_encoders(data)
        source_requested = KP*(target - q) - KD*qd
        requested = KP*(target - q) - self.arm_discrete_kd18*qd
        effort = np.clip(requested, -self.effort_limit18, self.effort_limit18)
        controls = np.where(self.arm_mask18, target, effort)
        data.ctrl[self.actuator_ids18] = controls
        ratio = np.abs(qd) / self.velocity_limit18
        return {
            "effort18": effort,
            "effort_unclipped18": requested,
            "source_pd_effort_unclipped18": source_requested,
            "effort_saturated18": np.abs(requested) > self.effort_limit18,
            "leg_torque18": np.where(self.arm_mask18, 0.0, effort),
            "actuator_ctrl18": controls,
            "velocity_limit_exceeded18": ratio > 1,
            "max_velocity_limit_ratio": float(ratio.max()),
            "arm_effort_is_estimate": True,
            "velocity_bounds_are_diagnostic": True,
        }
