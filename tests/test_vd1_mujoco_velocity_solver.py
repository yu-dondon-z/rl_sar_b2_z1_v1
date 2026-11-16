"""Tests of an explicit native velocity-brake simulation candidate.

These tests establish passivity and wiring, not VD1 locomotion success or an
exact reproduction of PhysX joint velocity constraints.
"""

import ast
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from vd1_mujoco_velocity_solver import add_native_velocity_brakes, NativeVelocityLimitBrakes
from vd1_mujoco_contract import ACTOR_JOINT_NAMES, VD1MujocoAdapter
from vd1_mujoco_scene import build_scene, validate_model


def fixture_xml(free=False):
    root = ET.Element("mujoco")
    ET.SubElement(root, "option", timestep=".001", gravity="0 0 0", integrator="implicitfast")
    world = ET.SubElement(root, "worldbody")
    if free:
        base = ET.SubElement(world, "body", name="base")
        ET.SubElement(base, "freejoint")
        ET.SubElement(base, "geom", type="sphere", size=".2", mass="2", contype="0", conaffinity="0")
    else:
        base = world
    rotor = ET.SubElement(base, "body", name="rotor")
    ET.SubElement(rotor, "joint", name="rotor_joint", axis="0 0 1", damping="0")
    ET.SubElement(rotor, "geom", type="sphere", size=".1", mass="1", contype="0", conaffinity="0")
    actuators = ET.SubElement(root, "actuator")
    ET.SubElement(actuators, "motor", name="rotor_motor", joint="rotor_joint")
    return ET.ElementTree(root)


class NativeBrakeTests(unittest.TestCase):
    def setUp(self):
        self.cache = tempfile.TemporaryDirectory(prefix="vd1_brake_test_", dir="/dev/shm")
        self.addCleanup(self.cache.cleanup)

    def model(self, free=False):
        path = Path(self.cache.name) / "simple.xml"
        fixture_xml(free).write(str(path))
        add_native_velocity_brakes(path, ["rotor_joint"], [3.0])
        model = mujoco.MjModel.from_xml_path(str(path))
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "rotor_joint")
        brakes = NativeVelocityLimitBrakes(model, [joint], [3.0], regularization_s=1e-5)
        return model, data, brakes

    def test_inactive_brake_has_zero_force_and_does_not_accelerate_toward_limit(self):
        for speed in (-2.0, 0.0, 1.0, 2.9):
            with self.subTest(speed=speed):
                model, data, brakes = self.model()
                data.qvel[brakes.dofs] = speed
                brakes.step(data)
                np.testing.assert_allclose(data.qvel[brakes.dofs], speed, atol=1e-14)
                np.testing.assert_array_equal(data.actuator_force[brakes.brake_ids], [[0., 0.]])
                self.assertEqual(brakes.summary()["active_substeps"], 0)

    def test_inactive_brake_preserves_nonzero_torque_response_against_no_brake_baseline(self):
        baseline_path = Path(self.cache.name) / "baseline.xml"
        fixture_xml().write(str(baseline_path))
        baseline_model = mujoco.MjModel.from_xml_path(str(baseline_path))
        baseline = mujoco.MjData(baseline_model)
        for speed, torque in ((1.0, .02), (-1.0, -.02), (0.0, .01)):
            with self.subTest(speed=speed, torque=torque):
                model, data, brakes = self.model()
                mujoco.mj_resetData(baseline_model, baseline)
                baseline.qvel[0] = speed
                baseline.ctrl[0] = torque
                data.qvel[brakes.dofs] = speed
                data.ctrl[0] = torque
                mujoco.mj_step(baseline_model, baseline)
                brakes.step(data)
                self.assertNotEqual(float(baseline.qvel[0]), speed)
                np.testing.assert_array_equal(data.qvel, baseline.qvel)
                np.testing.assert_array_equal(data.qpos, baseline.qpos)
                np.testing.assert_array_equal(model.actuator_gainprm[brakes.brake_ids, 0], [[0., 0.]])
                np.testing.assert_array_equal(model.actuator_biasprm[brakes.brake_ids, 2], [[0., 0.]])

    def test_upper_and_lower_reactions_are_dissipative_and_reduce_kinetic_energy(self):
        for speed in (-6.0, 6.0):
            with self.subTest(speed=speed):
                model, data, brakes = self.model()
                data.qvel[brakes.dofs] = speed
                mujoco.mj_forward(model, data)
                before_energy = .5*data.qvel @ (np.array([[.004]]) @ data.qvel)
                brakes.step(data)
                after_energy = .5*data.qvel @ (np.array([[.004]]) @ data.qvel)
                self.assertLess(after_energy, before_energy)
                force = data.actuator_force[brakes.brake_ids].sum()
                self.assertLessEqual(force*speed, 0)
                self.assertLess(abs(data.qvel[0]), abs(speed))
                self.assertGreater(abs(data.qvel[0]), 3.0)
                self.assertEqual(brakes.summary()["max_positive_linearization_power_w"], 0)
                self.assertFalse(brakes.summary()["hard_bound_guaranteed"])

    def test_reported_external_force_ghost_regression_keeps_v1_equal_1_point_25(self):
        model, data, brakes = self.model()
        data.qvel[0] = 1.0
        data.qfrc_applied[0] = 1.0
        brakes.step(data)
        # A solid sphere with m=1 and radius=.1 has I=.004. The old inactive
        # kv derivative incorrectly reduced this response despite zero force.
        self.assertAlmostEqual(float(data.qvel[0]), 1.25, places=12)
        self.assertEqual(data.qfrc_applied[0], 1.0)
        np.testing.assert_array_equal(data.actuator_force[brakes.brake_ids], [[0., 0.]])

    def test_free_base_reacts_and_total_angular_momentum_is_conserved(self):
        model, data, brakes = self.model(free=True)
        data.qvel[brakes.dofs] = 6.0
        mujoco.mj_forward(model, data)
        mujoco.mj_subtreeVel(model, data)
        base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
        momentum = data.subtree_angmom[base].copy()
        brakes.step(data)
        self.assertGreater(abs(data.qvel[5]), 0)
        mujoco.mj_forward(model, data)
        mujoco.mj_subtreeVel(model, data)
        np.testing.assert_allclose(data.subtree_angmom[base], momentum, atol=1e-12)

    def test_constant_brake_compliance_does_not_change_robot_motor_or_passive_gain(self):
        model, data, brakes = self.model()
        robot_gain = model.actuator_gainprm[0].copy()
        robot_bias = model.actuator_biasprm[0].copy()
        damping = model.dof_damping.copy()
        ctrl = data.ctrl[0]
        data.qvel[brakes.dofs] = 6.0
        brakes.step(data)
        brake_compliance = brakes.damping.copy()
        brakes.step(data)
        np.testing.assert_array_equal(model.actuator_gainprm[0], robot_gain)
        np.testing.assert_array_equal(model.actuator_biasprm[0], robot_bias)
        np.testing.assert_array_equal(model.dof_damping, damping)
        np.testing.assert_array_equal(brakes.damping, brake_compliance)
        self.assertEqual(data.ctrl[0], ctrl)

    def test_bound_crossing_is_reported_without_claiming_hard_constraint(self):
        model, data, brakes = self.model()
        data.qvel[brakes.dofs] = 2.9
        data.ctrl[0] = 20.0
        brakes.step(data)
        self.assertGreater(abs(data.qvel[0]), 3.0)
        self.assertGreater(brakes.summary()["max_velocity_ratio_after_step"], 1.0)
        self.assertFalse(brakes.summary()["hard_bound_guaranteed"])

    def test_actual_vd1_model_accepts_only_named_brakes_and_keeps_actor_motors(self):
        path = build_scene(0.0, Path(self.cache.name) / "actual", timestep=.001)
        original = mujoco.MjModel.from_xml_path(str(path))
        original_adapter = VD1MujocoAdapter(original)
        original_gain = original.actuator_gainprm[original_adapter.actuator_ids18].copy()
        original_bias = original.actuator_biasprm[original_adapter.actuator_ids18].copy()
        add_native_velocity_brakes(path, ACTOR_JOINT_NAMES, original_adapter.velocity_limit18)
        model = mujoco.MjModel.from_xml_path(str(path))
        self.assertEqual(model.nu, 54)
        self.assertEqual(validate_model(model)["nu"], 54)
        adapter = VD1MujocoAdapter(model)
        data = mujoco.MjData(model)
        mujoco.mj_resetDataKeyframe(model, data, 0)
        mujoco.mj_forward(model, data)
        brakes = NativeVelocityLimitBrakes(model, adapter.joint_ids18, adapter.velocity_limit18)
        brakes.initialize(data)
        # Exercise the true source model at the instant before physics:
        # velocities at half limits must produce no brake force anywhere.
        data.qvel[brakes.dofs] = brakes.limits*.5
        brakes.prepare(data)
        mujoco.mj_forward(model, data)
        np.testing.assert_array_equal(data.actuator_force[brakes.brake_ids], np.zeros((18, 2)))
        data.qvel[brakes.dofs] = brakes.limits*1.2
        brakes.prepare(data)
        mujoco.mj_forward(model, data)
        force = data.actuator_force[brakes.brake_ids].sum(axis=1)
        self.assertTrue(np.all(force * data.qvel[brakes.dofs] <= 0))
        np.testing.assert_array_equal(model.actuator_gainprm[adapter.actuator_ids18], original_gain)
        np.testing.assert_array_equal(model.actuator_biasprm[adapter.actuator_ids18], original_bias)
        np.testing.assert_allclose(adapter.default_q18, original_adapter.default_q18)

    def test_source_never_assigns_qpos_qvel_or_sensor_buffer(self):
        source = Path(__file__).resolve().parents[1] / "scripts/vd1_mujoco_velocity_solver.py"
        tree = ast.parse(source.read_text())
        for node in ast.walk(tree):
            if isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)):
                targets = node.targets if isinstance(node, ast.Assign) else [node.target]
                for target in targets:
                    for part in ast.walk(target):
                        if isinstance(part, ast.Attribute):
                            self.assertNotIn(part.attr, {"qpos", "qvel", "sensordata"})


if __name__ == "__main__":
    unittest.main()
