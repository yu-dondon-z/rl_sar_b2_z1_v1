"""An explicitly massless fixed tool changes loads without changing geometry."""
from pathlib import Path
import sys
import tempfile
import unittest

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from vd1_mujoco_scene import build_scene, validate_model


class ZeroGunMassTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cache = tempfile.TemporaryDirectory(prefix='vd1_zero_gun_', dir='/dev/shm')
        cls.models = {}
        for mode in ('original', 'zero'):
            scene = build_scene(.3, Path(cls.cache.name)/mode, road_surface='tape',
                                tool_markers=False, gun_mass_mode=mode)
            cls.models[mode] = mujoco.MjModel.from_xml_path(str(scene))

    @classmethod
    def tearDownClass(cls):
        cls.cache.cleanup()

    def test_exact_zero_mass_and_inertia_only_for_gun(self):
        original, zero = self.models['original'], self.models['zero']
        gun = zero.body('link_gun').id
        np.testing.assert_array_equal(zero.body_mass[gun], 0.)
        np.testing.assert_array_equal(zero.body_inertia[gun], np.zeros(3))
        others = np.arange(zero.nbody) != gun
        for field in ('body_mass', 'body_inertia', 'body_iquat'):
            np.testing.assert_array_equal(getattr(zero, field)[others], getattr(original, field)[others])
        self.assertAlmostEqual(original.body_mass.sum()-zero.body_mass.sum(), 3.25904698235386)
        self.assertAlmostEqual(zero.body_mass.sum(), 78.9990202)

    def test_collision_visuals_frames_and_actor_dimensions_are_preserved(self):
        original, zero = self.models['original'], self.models['zero']
        self.assertEqual((zero.nq, zero.nv, zero.nu), (25, 24, 18))
        # The model title labels the experiment; all object names stay equal.
        self.assertEqual(zero.names.split(b'\0')[1:], original.names.split(b'\0')[1:])
        for field in ('body_pos', 'body_quat', 'jnt_axis', 'jnt_pos', 'jnt_range',
                      'geom_bodyid', 'geom_type', 'geom_pos', 'geom_quat', 'geom_size',
                      'geom_contype', 'geom_conaffinity', 'geom_group', 'site_bodyid', 'site_pos'):
            np.testing.assert_array_equal(getattr(zero, field), getattr(original, field))
        self.assertGreater(zero.geom('link_gun_collision_0').contype[0], 0)
        self.assertGreaterEqual(zero.site('tool_tcp').id, 0)

    def test_validation_requires_the_correct_explicit_mass_mode(self):
        self.assertEqual(validate_model(self.models['original'])['gun_mass_mode'], 'original')
        result = validate_model(self.models['zero'], gun_mass_mode='zero')
        self.assertEqual(result['gun_mass_kg'], 0.)
        self.assertTrue(result['gun_mass_experiment'])
        with self.assertRaises(AssertionError):
            validate_model(self.models['zero'])
        with self.assertRaises(AssertionError):
            validate_model(self.models['original'], gun_mass_mode='zero')

    def test_mass_matrix_is_positive_and_gravity_load_changes(self):
        loads = {}
        for mode, model in self.models.items():
            data = mujoco.MjData(model)
            mujoco.mj_resetDataKeyframe(model, data, model.key('vd1_default').id)
            mujoco.mj_forward(model, data)
            matrix = np.zeros((model.nv, model.nv))
            mujoco.mj_fullM(model, matrix, data.qM)
            self.assertGreater(np.linalg.eigvalsh(matrix).min(), 0.)
            self.assertTrue(np.isfinite(data.qfrc_bias).all())
            loads[mode] = data.qfrc_bias.copy()
        self.assertAlmostEqual(loads['original'][2]-loads['zero'][2], 3.25904698235386*9.81)
        self.assertGreater(np.linalg.norm(loads['original'][6:]-loads['zero'][6:]), .1)


if __name__ == '__main__':
    unittest.main()
