"""Nearby guidance regressions using partial, synchronized camera paths."""
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from vd1_visual_path import paired_ground_guidance


def sections(function, normal_rotation=None):
    result = []
    for x in (np.linspace(1.5, 2.5, 40), np.linspace(-1.3, -.65, 50)):
        points = np.column_stack((x, function(x), np.full_like(x, -.5)))
        result.append(points if normal_rotation is None else points @ normal_rotation.T)
    normal = [0., 0., 1.] if normal_rotation is None else normal_rotation[:, 2]
    return result+[np.r_[normal, .5]]


class PairedPathTests(unittest.TestCase):
    def test_interpolates_between_cameras_instead_of_pursuing_far_visible_point(self):
        fn = lambda x: .015*x**3+.02*x*x-.04*x-.06
        geometry, status = paired_ground_guidance(*sections(fn))
        self.assertAlmostEqual(geometry[-1][0], .6)
        self.assertAlmostEqual(geometry[-1][1], fn(.6), delta=1e-4)
        self.assertAlmostEqual(geometry[1], .06, delta=1e-4)
        self.assertTrue(status['guidance_gap_is_interpolated'])
        self.assertLess(status['guidance_fit_p95_m'], 1e-5)

    def test_rear_offset_changes_near_guidance_with_same_front_path(self):
        data = sections(lambda x: np.zeros_like(x))
        straight, _ = paired_ground_guidance(*data)
        data[1][:, 1] = -.12
        correction, _ = paired_ground_guidance(*data)
        self.assertAlmostEqual(straight[-1][1], 0.)
        self.assertLess(correction[-1][1], -.02)

    def test_sharp_bend_shortens_horizon_and_strengthens_lateral_correction(self):
        data = sections(lambda x: .2*x*x+.025)
        fixed, _ = paired_ground_guidance(*data)
        near, status = paired_ground_guidance(*data, adaptive_lookahead=True)
        yaw = lambda geometry: .8*geometry[-1][1]/np.sum(geometry[-1][:2]**2)
        self.assertAlmostEqual(fixed[-1][0], .6)
        self.assertAlmostEqual(near[-1][0], .4, delta=1e-4)
        self.assertGreater(yaw(near), yaw(fixed))
        self.assertAlmostEqual(status['guidance_local_curvature_m_inv'], .4, delta=1e-4)

    def test_gentle_bend_preserves_original_horizon(self):
        data = sections(lambda x: .04*x*x+.02*x-.01)
        fixed, _ = paired_ground_guidance(*data)
        adaptive, _ = paired_ground_guidance(*data, adaptive_lookahead=True)
        np.testing.assert_allclose(adaptive[-1], fixed[-1], atol=1e-12)

    def test_ground_plane_removes_roll_from_steering(self):
        angle = .12
        rot = np.array([[1., 0., 0.], [0., np.cos(angle), -np.sin(angle)],
                        [0., np.sin(angle), np.cos(angle)]])
        flat, _ = paired_ground_guidance(*sections(lambda x: -.04+.025*x))
        rolled, status = paired_ground_guidance(*sections(lambda x: -.04+.025*x, rot))
        np.testing.assert_allclose(rolled[-1], flat[-1], atol=1e-12)
        # The displayed point is converted back to the capture base frame.
        expected = rot @ [.6, -.04+.025*.6, -.5]
        np.testing.assert_allclose(status['front_lookahead_b'], expected, atol=1e-12)

    def test_fit_is_not_weighted_by_front_pixel_density(self):
        data = sections(lambda x: .012*x*x-.03)
        reference, _ = paired_ground_guidance(*data)
        data[0] = np.repeat(data[0], 20, axis=0)
        dense, _ = paired_ground_guidance(*data)
        np.testing.assert_allclose(dense[-1], reference[-1], atol=1e-12)

    def test_unbracketed_gap_and_excessive_bend_are_rejected(self):
        for fn in (lambda x: 2*x*x, lambda x: np.zeros_like(x)):
            data = sections(fn)
            if np.max(np.abs(data[0][:, 1])) == 0:
                data[0][:, 0] += 4.
            with self.assertRaises(ValueError):
                paired_ground_guidance(*data)

    def test_insufficient_or_nonfinite_samples_are_rejected(self):
        data = sections(lambda x: np.zeros_like(x))
        for front in (data[0][:2], np.full((5, 3), np.nan)):
            with self.assertRaises(ValueError):
                paired_ground_guidance(front, *data[1:])


if __name__ == '__main__':
    unittest.main()
