"""Physical constraint projection tests, independent of control observations."""

import ast
from pathlib import Path
import sys
import tempfile
import unittest

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from vd1_mujoco_velocity_impulse import VelocityImpulseConstraints
from vd1_mujoco_scene import build_scene
from vd1_mujoco_contract import VD1MujocoAdapter


class VelocityImpulseTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cache = tempfile.TemporaryDirectory(prefix="vd1_impulse_test_", dir="/dev/shm")
        cls.path = build_scene(0.0, cls.cache.name, timestep=.001)
        cls.model = mujoco.MjModel.from_xml_path(str(cls.path))

    @classmethod
    def tearDownClass(cls):
        cls.cache.cleanup()

    def actual(self):
        data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, data, 0)
        mujoco.mj_forward(self.model, data)
        adapter = VD1MujocoAdapter(self.model)
        solver = VelocityImpulseConstraints(self.model, adapter.joint_ids18, adapter.velocity_limit18)
        return data, adapter, solver

    def free_rotor(self):
        model = mujoco.MjModel.from_xml_string("""
          <mujoco><option timestep=".001" gravity="0 0 0" integrator="implicitfast"/>
            <worldbody><body name="base"><freejoint/>
              <geom type="sphere" size=".2" mass="2" contype="0" conaffinity="0"/>
              <body name="rotor"><joint name="hinge" axis="0 0 1"/>
                <geom type="sphere" size=".1" mass="1" contype="0" conaffinity="0"/>
              </body></body></worldbody>
          </mujoco>""")
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "hinge")
        return model, data, VelocityImpulseConstraints(model, [joint], [3.0])

    def test_within_bounds_projection_is_identity_without_position_or_time_change(self):
        data, adapter, solver = self.actual()
        data.qvel[solver.dofs] = adapter.velocity_limit18*.3
        qpos, qvel, timestamp = data.qpos.copy(), data.qvel.copy(), data.time
        solver.project(data)
        np.testing.assert_array_equal(data.qvel, qvel)
        np.testing.assert_array_equal(data.qpos, qpos)
        self.assertEqual(data.time, timestamp)
        self.assertFalse(solver.last["active"])

    def test_free_base_responds_and_momentum_is_conserved_instead_of_component_clipping(self):
        model, data, solver = self.free_rotor()
        data.qvel[solver.dofs] = 6.0
        mujoco.mj_forward(model, data)
        mujoco.mj_subtreeVel(model, data)
        base = model.body("base").id
        angular = data.subtree_angmom[base].copy()
        linear = data.subtree_linvel[base].copy()
        independent_clip = data.qvel.copy()
        independent_clip[solver.dofs] = 3.0
        solver.project(data)
        self.assertGreater(abs(data.qvel[5]), .1)
        self.assertFalse(np.allclose(data.qvel, independent_clip))
        np.testing.assert_allclose(data.qvel[solver.dofs], 3.0, atol=1e-10)
        mujoco.mj_subtreeVel(model, data)
        np.testing.assert_allclose(data.subtree_angmom[base], angular, atol=1e-12)
        np.testing.assert_allclose(data.subtree_linvel[base], linear, atol=1e-12)
        self.assertLess(solver.last["kinetic_energy_change"], 0)
        np.testing.assert_array_equal(solver.last["generalized_impulse"][:6], np.zeros(6))

    def test_actual_full_mass_projection_converges_and_never_increases_total_kinetic_energy(self):
        random = np.random.RandomState(15)
        for trial in range(5):
            with self.subTest(trial=trial):
                data, adapter, solver = self.actual()
                data.qvel[:6] = random.normal(0, .3, 6)
                data.qvel[solver.dofs] = adapter.velocity_limit18 * random.uniform(-2.5, 2.5, 18)
                before = data.qvel.copy()
                qpos = data.qpos.copy()
                gains = self.model.actuator_gainprm.copy()
                targets = data.ctrl.copy()
                solver.project(data)
                self.assertTrue(np.all(np.abs(data.qvel[solver.dofs]) <= adapter.velocity_limit18 + 2e-8))
                self.assertLessEqual(solver.last["kinetic_energy_change"], 1e-10)
                delta = data.qvel-before
                np.testing.assert_allclose(solver.mass @ delta, solver.last["generalized_impulse"], atol=1e-9)
                np.testing.assert_allclose((solver.mass @ delta)[:6], 0., atol=1e-9)
                np.testing.assert_array_equal(data.qpos, qpos)
                np.testing.assert_array_equal(self.model.actuator_gainprm, gains)
                np.testing.assert_array_equal(data.ctrl, targets)

    def test_real_sensor_outputs_are_recomputed_from_physical_constrained_state(self):
        data, adapter, solver = self.actual()
        data.qvel[solver.dofs] = adapter.velocity_limit18 * 1.5
        mujoco.mj_forward(self.model, data)
        before = adapter.read_proprioception(data)
        solver.project(data)
        measured = adapter.read_proprioception(data)
        self.assertFalse(np.array_equal(before["qd"], measured["qd"]))
        np.testing.assert_allclose(measured["qd"], data.qvel[solver.dofs], atol=0)
        snapshot = data.sensordata.copy()
        mujoco.mj_forward(self.model, data)
        # Re-solving contact acceleration may differ at roundoff level; the
        # gyro/quaternion/encoders still derive from the exact same state.
        np.testing.assert_allclose(data.sensordata, snapshot, rtol=1e-12, atol=1e-11)

    def test_step_advances_once_then_projects_actual_velocity(self):
        model, data, solver = self.free_rotor()
        data.qvel[solver.dofs] = -6.0
        solver.step(data)
        self.assertAlmostEqual(data.time, .001)
        np.testing.assert_allclose(data.qvel[solver.dofs], -3.0, atol=1e-10)
        summary = solver.summary()
        self.assertTrue(summary["state_velocity_impulse_applied"])
        self.assertFalse(summary["independent_component_clip"])
        self.assertFalse(summary["exact_physx_solver"])
        self.assertEqual(summary["physics_steps"], 1)

    def test_hinge_displacement_uses_constrained_velocity_at_1ms_and_5ms(self):
        for dt in (.001, .005):
            for speed in (-6., 6.):
                with self.subTest(dt=dt, speed=speed):
                    model, data, solver = self.free_rotor()
                    model.opt.timestep = dt
                    data.qvel[solver.dofs] = speed
                    before = data.qpos.copy()
                    solver.step(data)
                    qpos_address = model.jnt_qposadr[solver.joints[0]]
                    displacement = data.qpos[qpos_address]-before[qpos_address]
                    self.assertLessEqual(abs(displacement), 3.*dt + 1e-12)
                    self.assertAlmostEqual(displacement, np.sign(speed)*3.*dt, places=12)
                    self.assertTrue(solver.summary()["position_integrated_from_constrained_velocity"])

    def test_evaluation_observer_saves_native_step_without_changing_response(self):
        model, data, solver = self.free_rotor()
        other = mujoco.MjData(model)
        reference = VelocityImpulseConstraints(model, solver.joints, solver.limits)
        data.qvel[solver.dofs] = 6.
        other.qvel[:] = data.qvel
        snapshots = []
        for _ in range(5):
            solver.step(data, evaluation_observer=lambda d: snapshots.append(
                (float(d.time), d.qvel.copy(), d.qfrc_constraint.copy())))
            reference.step(other)
            np.testing.assert_array_equal(data.qpos, other.qpos)
            np.testing.assert_array_equal(data.qvel, other.qvel)
        self.assertEqual(len(snapshots), 5)
        self.assertAlmostEqual(snapshots[0][0], model.opt.timestep)
        self.assertGreater(abs(snapshots[0][1][solver.dofs[0]]), solver.limits[0])

    def test_code_has_one_explicit_coupled_velocity_impulse_write_and_no_sensor_writes(self):
        path = Path(__file__).resolve().parents[1] / "scripts/vd1_mujoco_velocity_impulse.py"
        tree = ast.parse(path.read_text())
        velocity_writes = []
        position_writes = []
        for node in ast.walk(tree):
            if isinstance(node, (ast.Assign, ast.AnnAssign, ast.AugAssign)):
                targets = node.targets if isinstance(node, ast.Assign) else [node.target]
                for target in targets:
                    for part in ast.walk(target):
                        if isinstance(part, ast.Attribute):
                            self.assertNotEqual(part.attr, "sensordata")
                            if part.attr == "qvel":
                                velocity_writes.append(node)
                            if part.attr == "qpos":
                                position_writes.append(node)
        self.assertEqual(len(velocity_writes), 1)
        self.assertIsInstance(velocity_writes[0], ast.AugAssign)
        self.assertEqual(len(position_writes), 1)
        self.assertIsInstance(position_writes[0], ast.Assign)


if __name__ == "__main__":
    unittest.main()
