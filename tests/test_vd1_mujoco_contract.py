"""Sensor-only VD1 boundaries, using a real compiled MuJoCo model and actor.

Run with the already installed crack_vision Python. No renderer, hardware
communication, policy training, or robot motion is started by these tests.
"""

import math
from pathlib import Path
import sys
import tempfile
import unittest

import mujoco
import numpy as np
import torch


DEPLOY_ROOT = Path(__file__).resolve().parents[1]
TRAINING_ROOT = Path("/home/dong/b2_z1")
sys.path.insert(0, str(DEPLOY_ROOT / "scripts"))
sys.path.insert(0, str(TRAINING_ROOT / "scripts/vision"))
from vd1_mujoco_contract import ACTOR_JOINT_NAMES, VD1MujocoAdapter
from vd1_mujoco_scene import build_scene, DEFAULT_JOINT_POSITION, EXPECTED_JOINT_NAMES
from vd1_control import SCHEMA, VD1VisualController, build_actor_observation


def visual_fixture():
    """Fixed synthetic camera measurements; no model or scene access."""
    front_x = np.linspace(.3, 2., 40)
    rear_x = np.linspace(-1.3, -.8, 30)
    capture = dict(valid=True, sequence=4, capture_generation=7, capture_timestamp=100.)
    return dict(schema=SCHEMA, valid=True, capture_generation=7,
                front=dict(capture, points_b=np.column_stack((front_x, .03*front_x,
                                                              np.full_like(front_x, -.5))).tolist()),
                rear=dict(capture, points_b=np.column_stack((rear_x, np.zeros_like(rear_x),
                                                             np.full_like(rear_x, -.5))).tolist(),
                          tcp_measured_b=[-1.02, .01, -.30], ground_plane_b=[0., 0., 1., .5]))


class SensorOnlyData:
    """Fails if a control input tries to access anything except sensor/ctrl."""
    def __init__(self, data):
        self.sensordata = data.sensordata.copy()
        self.ctrl = data.ctrl.copy()

    def __getattr__(self, name):
        raise AssertionError("Forbidden simulator field accessed: " + name)


class VD1MujocoContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cache = tempfile.TemporaryDirectory(prefix="vd1_contract_test_", dir="/dev/shm")
        cls.path = build_scene(0.0, cls.cache.name, timestep=.001)
        cls.model = mujoco.MjModel.from_xml_path(str(cls.path))
        cls.actor = torch.jit.load(str(TRAINING_ROOT / ".pretrained_checkpoints/VD1/exported/policy.pt"),
                                   map_location="cpu").eval()
        torch.set_num_threads(1)

    @classmethod
    def tearDownClass(cls):
        cls.cache.cleanup()

    def setUp(self):
        self.data = mujoco.MjData(self.model)
        key = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "vd1_default")
        mujoco.mj_resetDataKeyframe(self.model, self.data, key)
        mujoco.mj_forward(self.model, self.data)
        self.adapter = VD1MujocoAdapter(self.model)

    def observation_action(self, data):
        controller = VD1VisualController([-1.02, 0., -.30], ramp_s=0)
        command = controller.update(visual_fixture(), now=100., dt=.02)
        measured = self.adapter.read_proprioception(data)
        obs = build_actor_observation(command["base_command_b"], command["tool_pose_b"],
                                      measured["gyro_b"], measured["gravity_b"], measured["q"],
                                      measured["qd"], self.adapter.default_q18, np.linspace(-.1, .1, 18))
        with torch.inference_mode():
            action = self.actor(torch.as_tensor(obs).unsqueeze(0))[0].numpy()
        return obs, action

    def test_names_remap_depth_first_model_to_upstream_actor_order(self):
        self.assertEqual(ACTOR_JOINT_NAMES, EXPECTED_JOINT_NAMES)
        physical = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, index)
                    for index in range(1, self.model.njnt)]
        self.assertNotEqual(tuple(physical), ACTOR_JOINT_NAMES)
        measured = self.adapter.read_proprioception(self.data)
        np.testing.assert_allclose(measured["q"], [DEFAULT_JOINT_POSITION[n] for n in ACTOR_JOINT_NAMES])
        np.testing.assert_allclose(measured["q"], self.adapter.default_q18)
        for index, name in enumerate(ACTOR_JOINT_NAMES):
            joint = self.adapter.joint_ids18[index]
            self.data.qpos[self.model.jnt_qposadr[joint]] += .001*(index + 1)
            self.data.qvel[self.model.jnt_dofadr[joint]] = -.02*(index + 1)
        mujoco.mj_forward(self.model, self.data)
        measured = self.adapter.read_proprioception(self.data)
        np.testing.assert_allclose(measured["q"] - self.adapter.default_q18, .001*np.arange(1, 19))
        np.testing.assert_allclose(measured["qd"], -.02*np.arange(1, 19))

    def test_sensor_only_proxy_permits_read_and_control_without_world_state(self):
        proxy = SensorOnlyData(self.data)
        measured = self.adapter.read_proprioception(proxy)
        self.assertEqual(set(measured), {"gyro_b", "gravity_b", "q", "qd"})
        diagnostics = self.adapter.apply_targets(proxy, self.adapter.default_q18)
        self.assertTrue(np.isfinite(diagnostics["effort18"]).all())
        obs, action = self.observation_action(proxy)
        self.assertEqual(obs.shape, (70,))
        self.assertEqual(action.shape, (18,))

    def test_fixed_sensors_make_world_pose_tcp_and_contact_counterfactual_invariant(self):
        obs_before, action_before = self.observation_action(self.data)
        sensors_before = self.data.sensordata.copy()
        site_before = self.data.site_xpos.copy()
        contact_before = self.data.ncon
        # Intentionally change simulator truth without changing the delivered
        # sensor/image packet, as if those measurements were held in a buffer.
        self.data.qpos[:3] = [3.0, -2.0, .28]
        self.data.qpos[3:7] = [math.cos(.18), math.sin(.18), 0, 0]
        self.data.qvel[:6] = [.4, -.3, .2, .1, .2, -.1]
        mujoco.mj_forward(self.model, self.data)
        self.assertFalse(np.allclose(site_before, self.data.site_xpos))
        self.assertNotEqual(contact_before, self.data.ncon)
        self.assertFalse(np.array_equal(sensors_before, self.data.sensordata))
        self.data.sensordata[:] = sensors_before
        obs_after, action_after = self.observation_action(self.data)
        np.testing.assert_array_equal(obs_before, obs_after)
        np.testing.assert_array_equal(action_before, action_after)

    def test_imu_yaw_is_discarded_but_roll_changes_gravity(self):
        proxy = SensorOnlyData(self.data)
        address = self.adapter._orientation_address
        yaw = 1.2
        proxy.sensordata[address:address+4] = [math.cos(yaw/2), 0, 0, math.sin(yaw/2)]
        np.testing.assert_allclose(self.adapter.read_proprioception(proxy)["gravity_b"], [0, 0, -1], atol=1e-15)
        roll = .3
        proxy.sensordata[address:address+4] = [math.cos(roll/2), math.sin(roll/2), 0, 0]
        gravity = self.adapter.read_proprioception(proxy)["gravity_b"]
        np.testing.assert_allclose(gravity, [0, -math.sin(roll), -math.cos(roll)])
        self.assertAlmostEqual(np.linalg.norm(gravity), 1.)

    def test_nonfinite_and_nonunit_sensor_values_are_rejected(self):
        proxy = SensorOnlyData(self.data)
        proxy.sensordata[self.adapter._orientation_address:self.adapter._orientation_address+4] = 0
        with self.assertRaisesRegex(ValueError, "unit"):
            self.adapter.read_proprioception(proxy)
        proxy = SensorOnlyData(self.data)
        proxy.sensordata[self.adapter._gyro_address] = float("nan")
        with self.assertRaisesRegex(ValueError, "finite"):
            self.adapter.read_proprioception(proxy)

    def test_leg_torque_saturates_and_arm_control_remains_native_position(self):
        target = self.adapter.default_q18 + np.where(np.arange(18) % 2, -10, 10)
        diagnostics = self.adapter.apply_targets(self.data, target)
        controls = self.data.ctrl[self.adapter.actuator_ids18]
        arms = self.adapter.arm_mask18
        np.testing.assert_array_equal(controls[arms], target[arms])
        expected = np.sign(target - self.adapter.default_q18) * self.adapter.effort_limit18
        np.testing.assert_allclose(controls[~arms], expected[~arms])
        np.testing.assert_allclose(diagnostics["effort18"], expected)
        self.assertTrue(diagnostics["arm_effort_is_estimate"])

    def test_leg_velocity_feedback_and_named_actuator_mapping(self):
        proxy = SensorOnlyData(self.data)
        velocity = np.linspace(-1, 1, 18)
        proxy.sensordata[self.adapter._qd_addresses] = velocity
        target = self.adapter.default_q18 + np.linspace(-.1, .1, 18)
        self.adapter.apply_targets(proxy, target)
        for index, name in enumerate(ACTOR_JOINT_NAMES):
            actuator = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name + "_motor")
            expected = target[index] if self.adapter.arm_mask18[index] else (
                160*(target[index] - self.adapter.default_q18[index]) - 5*velocity[index])
            self.assertAlmostEqual(proxy.ctrl[actuator], expected)

    def test_velocity_excess_is_reported_without_changing_measured_state(self):
        proxy = SensorOnlyData(self.data)
        proxy.sensordata[self.adapter._qd_addresses] = self.adapter.velocity_limit18*1.2
        sensors_before = proxy.sensordata.copy()
        diagnostics = self.adapter.apply_targets(proxy, self.adapter.default_q18)
        self.assertTrue(diagnostics["velocity_limit_exceeded18"].all())
        self.assertAlmostEqual(diagnostics["max_velocity_limit_ratio"], 1.2)
        self.assertTrue(diagnostics["velocity_bounds_are_diagnostic"])
        np.testing.assert_array_equal(proxy.sensordata, sensors_before)

    def test_apply_targets_does_not_step_or_mutate_physics_state(self):
        qpos, qvel, sensors, timestamp = self.data.qpos.copy(), self.data.qvel.copy(), self.data.sensordata.copy(), self.data.time
        self.adapter.apply_targets(self.data, self.adapter.default_q18 + .01)
        np.testing.assert_array_equal(self.data.qpos, qpos)
        np.testing.assert_array_equal(self.data.qvel, qvel)
        np.testing.assert_array_equal(self.data.sensordata, sensors)
        self.assertEqual(self.data.time, timestamp)
        for invalid in ([0.0]*17, [float("nan")]*18, [float("inf")]*18):
            ctrl_before = self.data.ctrl.copy()
            with self.assertRaises(ValueError):
                self.adapter.apply_targets(self.data, invalid)
            np.testing.assert_array_equal(self.data.ctrl, ctrl_before)

    def test_constructor_rejects_encoder_wired_to_another_joint(self):
        sensor = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, "FL_hip_joint_pos")
        previous = int(self.model.sensor_objid[sensor])
        try:
            self.model.sensor_objid[sensor] = self.adapter.joint_ids18[1]
            with self.assertRaisesRegex(ValueError, "sensor"):
                VD1MujocoAdapter(self.model)
        finally:
            self.model.sensor_objid[sensor] = previous

    def test_constructor_rejects_arm_gain_change_and_extra_passive_damping(self):
        actuator = self.adapter.actuator_ids18[np.flatnonzero(self.adapter.arm_mask18)[0]]
        gain = self.model.actuator_gainprm[actuator, 0]
        try:
            self.model.actuator_gainprm[actuator, 0] = 60.
            with self.assertRaisesRegex(ValueError, "PD contract"):
                VD1MujocoAdapter(self.model)
        finally:
            self.model.actuator_gainprm[actuator, 0] = gain
        dof = self.model.jnt_dofadr[self.adapter.joint_ids18[-1]]
        damping = self.model.dof_damping[dof]
        try:
            self.model.dof_damping[dof] = 1.
            with self.assertRaisesRegex(ValueError, "extra damping"):
                VD1MujocoAdapter(self.model)
        finally:
            self.model.dof_damping[dof] = damping


if __name__ == "__main__":
    unittest.main()
