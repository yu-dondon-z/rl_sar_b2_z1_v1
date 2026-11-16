"""Known image geometry distinguishes tape centres from skeleton cap corners."""
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, '/home/dong/RHACrackNet/code')
from crack_trajectory import CenterlineConfig, extract_centerline
from vd5_rear_centerline import recenter_visible_sections


class RearCrossSectionTests(unittest.TestCase):
    def extract(self, mask):
        raw = extract_centerline(mask, CenterlineConfig(morphology_kernel=0, waypoint_spacing_px=6.),
                                 (320., 310.))
        return raw, recenter_visible_sections(raw, 6.)

    def test_thick_clipped_tape_removes_skeleton_corner_hooks(self):
        mask = np.zeros((480, 640), np.uint8)
        mask[130:472, 300:341] = 255
        raw, (centered, status) = self.extract(mask)
        np.testing.assert_allclose(centered.waypoints_uv[:, 0], 320., atol=.01)
        self.assertEqual(status['rear_centerline_method'], 'visible_mask_cross_sections')
        pixels = np.rint(centered.waypoints_uv).astype(int)
        self.assertTrue(np.all(mask[pixels[:, 1], pixels[:, 0]]))

    def test_diagonal_tape_keeps_its_observed_direction(self):
        mask = np.zeros((480, 640), np.uint8)
        cv2.line(mask, (250, 80), (320, 430), 255, 25)
        _, (centered, status) = self.extract(mask)
        uv = centered.waypoints_uv
        interior = uv[(uv[:, 1] > 100) & (uv[:, 1] < 410)]
        np.testing.assert_allclose(interior[:, 0], 250.+.2*(interior[:, 1]-80.), atol=1.)
        self.assertEqual(status['rear_centerline_method'], 'visible_mask_cross_sections')

    def test_curved_strip_is_not_replaced_by_a_straight_line(self):
        mask = np.zeros((480, 640), np.uint8)
        v = np.arange(80, 431)
        u = 280.+.0005*(v-250.)**2
        cv2.polylines(mask, [np.rint(np.c_[u, v]).astype(np.int32)], False, 255, 21)
        _, (centered, status) = self.extract(mask)
        uv = centered.waypoints_uv
        interior = uv[(uv[:, 1] > 100) & (uv[:, 1] < 410)]
        np.testing.assert_allclose(interior[:, 0], 280.+.0005*(interior[:, 1]-250.)**2, atol=1.)
        self.assertEqual(status['rear_centerline_method'], 'visible_mask_cross_sections')

    def test_multiple_runs_are_not_bridged_across_unobserved_pixels(self):
        mask = np.zeros((480, 640), np.uint8)
        cv2.rectangle(mask, (280, 100), (300, 440), 255, -1)
        cv2.rectangle(mask, (330, 100), (350, 440), 255, -1)
        cv2.rectangle(mask, (280, 420), (350, 440), 255, -1)
        raw, (centered, status) = self.extract(mask)
        self.assertIs(centered, raw)
        self.assertEqual(status['rear_cross_section_fallback'], 'too few observed cross sections')


if __name__ == '__main__':
    unittest.main()
