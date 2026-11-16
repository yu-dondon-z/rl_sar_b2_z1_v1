"""CPU adapter tests with independent URDF FK fixtures; no simulator stepping."""

import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np
import torch


sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from vd12_geometry_guard import ASSETS, URDF, MujocoGunGeometryGuard, clearance_decision, load_pure_module
from vd1_mujoco_scene import ARM_JOINT_NAMES, EXPECTED_JOINT_NAMES, DEFAULT_JOINT_POSITION, quaternion_from_matrix


class ModelFixture:
    def __init__(self, names):
        self.names, self.nbody = ['world'] + list(names), len(names) + 1

    def body(self, index):
        return SimpleNamespace(name=self.names[index])


class GeometryContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cpu = load_pure_module('vd12_independent_test_audit', ASSETS / 'vd11_geometry_audit.py').GunRobotGeometryAudit(URDF)
        cls.names = list(cls.cpu._children)
        cls.q = dict(DEFAULT_JOINT_POSITION)
        cls.q.update(zip(ARM_JOINT_NAMES, (0., .66647356, -1.25178768, .88531411, 0., 0.)))
        cls.model = ModelFixture(cls.names)

    def fixture_data(self, q=None, names=None):
        names = self.names if names is None else names
        transforms = self.cpu._fk(self.q if q is None else q)
        return SimpleNamespace(xpos=np.array([[0., 0., 0.]] + [transforms[name][1].tolist() for name in names]),
                               xquat=np.array([[1., 0., 0., 0.]] + [quaternion_from_matrix(transforms[name][0]).tolist() for name in names]))

    def test_nominal_all_44_pairs_match_cpu_and_shared_profile(self):
        adapter = MujocoGunGeometryGuard(self.model, dtype=torch.float64)
        result = adapter.evaluate(self.fixture_data())
        reference = self.cpu.audit(EXPECTED_JOINT_NAMES, [self.q[n] for n in EXPECTED_JOINT_NAMES])
        expected = {row['other_collision']: row['clearance_lower_bound_m'] - 1e-6 for row in reference['pair_results']}
        np.testing.assert_allclose(result['clearance_lower_bound_m'], [expected[n] for n in result['pair_names']], atol=1e-10)
        self.assertEqual(len(result['pair_names']), 44)
        self.assertFalse(result['conservative_keepout_violated'])
        self.assertEqual(result['soft_clearance_cost'], 0.)
        self.assertEqual(adapter.profile['soft_margin_m'], .03)
        self.assertEqual(adapter.profile['hard_threshold_m'], 0.)
        contract = load_pure_module('vd12_test_contract_factory', ASSETS / 'vd12_contract.py')
        self.assertEqual(adapter.profile, contract.geometry_profile(URDF))
        json.dumps(adapter.metadata, allow_nan=False)
        json.dumps(result, allow_nan=False)

    def test_hard_rule_is_strict_negative_and_soft_rule_matches_training(self):
        profile = MujocoGunGeometryGuard(self.model).profile
        for value, bad, cost in ((.03, False, 0.), (.015, False, .25), (0., False, 1.), (-.03, True, 4.)):
            result = clearance_decision(value, True, profile)
            self.assertEqual(result['conservative_keepout_violated'], bad)
            self.assertAlmostEqual(result['soft_clearance_cost'], cost)
        self.assertEqual(clearance_decision(float('nan'), True, profile)['soft_clearance_cost'], 9.)
        self.assertTrue(clearance_decision(.1, False, profile)['conservative_keepout_violated'])

    def test_bad_historical_arm_is_conservative_keepout_without_force_claim(self):
        q = dict(self.q)
        q.update(zip(ARM_JOINT_NAMES, (-.14608973264694214, .0001682684087427333, .0003486844361759722,
                                     .7668989896774292, .08212888240814209, -.27894189953804016)))
        result = MujocoGunGeometryGuard(self.model).evaluate(self.fixture_data(q))
        self.assertTrue(result['conservative_keepout_violated'])
        self.assertNotIn('confirmed_collision', result)
        self.assertNotIn('penetration_depth_m', result)
        self.assertFalse(result['physics_advanced'])

    def test_runtime_body_order_is_real_name_driven(self):
        reverse = list(reversed(self.names))
        original = MujocoGunGeometryGuard(self.model, dtype=torch.float64).evaluate(self.fixture_data())
        result = MujocoGunGeometryGuard(ModelFixture(reverse), dtype=torch.float64).evaluate(self.fixture_data(names=reverse))
        np.testing.assert_allclose(result['clearance_lower_bound_m'], original['clearance_lower_bound_m'], atol=1e-12)

    def test_fixed_child_fold_keeps_all_pairs(self):
        # Root plus movable children is a strict subset of actual named bodies.
        import xml.etree.ElementTree as ET
        root = ET.parse(URDF).getroot()
        subset = ['base_link'] + [joint.find('child').get('link') for joint in root.findall('joint') if joint.get('type') != 'fixed']
        full = MujocoGunGeometryGuard(self.model, dtype=torch.float64)
        folded = MujocoGunGeometryGuard(self.model, body_names=subset, dtype=torch.float64)
        self.assertEqual(folded.geometry.num_pairs, 44)
        self.assertTrue(any(row['source_link'] != row['carrier_body'] for row in folded.geometry.metadata['pairs']))
        np.testing.assert_allclose(folded.evaluate(self.fixture_data())['clearance_lower_bound_m'],
                                   full.evaluate(self.fixture_data())['clearance_lower_bound_m'], atol=1e-10)

    def test_float32_default_agrees_with_strict_float64(self):
        first = MujocoGunGeometryGuard(self.model).evaluate(self.fixture_data())
        second = MujocoGunGeometryGuard(self.model, dtype=torch.float64).evaluate(self.fixture_data())
        np.testing.assert_allclose(first['clearance_lower_bound_m'], second['clearance_lower_bound_m'], atol=2e-6)

    def test_invalid_actual_pose_fails_closed_finitely(self):
        adapter = MujocoGunGeometryGuard(self.model)
        data = self.fixture_data()
        data.xquat[self.names.index('link06') + 1] = 0.
        result = adapter.evaluate(data)
        self.assertFalse(result['input_valid'])
        self.assertTrue(result['conservative_keepout_violated'])
        self.assertEqual(result['minimum_pair_index'], -1)
        self.assertIsNone(result['minimum_pair_name'])
        json.dumps(result, allow_nan=False)

    def test_evaluation_does_not_write_actual_pose_arrays(self):
        data = self.fixture_data()
        before = (data.xpos.copy(), data.xquat.copy())
        MujocoGunGeometryGuard(self.model).evaluate(data)
        np.testing.assert_array_equal(data.xpos, before[0])
        np.testing.assert_array_equal(data.xquat, before[1])

    def test_missing_movable_body_or_unknown_selected_name_rejected(self):
        with self.assertRaises(ValueError):
            MujocoGunGeometryGuard(self.model, body_names=[name for name in self.names if name != 'link02'])
        with self.assertRaises(ValueError):
            MujocoGunGeometryGuard(self.model, body_names=self.names + ['not_a_runtime_body'])


if __name__ == '__main__':
    torch.set_num_threads(1)
    unittest.main()
