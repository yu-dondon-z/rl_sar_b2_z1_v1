"""Front crack selection with a bend and a competing central false line."""
from pathlib import Path
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
sys.path.insert(0, '/home/dong/b2_z1/scripts/vision')
sys.path.insert(0, '/home/dong/RHACrackNet/code')
from crack_trajectory import CenterlineConfig, extract_centerline
from vd1_visual_front import supported_front_centerline


class FrontComponentTests(unittest.TestCase):
    def setUp(self):
        self.mask = np.zeros((480, 640), dtype=np.uint8)
        self.config = CenterlineConfig(waypoint_spacing_px=6.)
        self.intrinsic = np.array([[460., 0., 320.], [0., 460., 240.], [0., 0., 1.]])
        self.transform = np.array([[0., 0., 1., .39], [-1., 0., 0., 0.],
                                   [0., -1., 0., 0.], [0., 0., 0., 1.]])

    def extract(self):
        return supported_front_centerline(self.mask, self.config, (320., 311.),
            extract_centerline, self.intrinsic, self.transform, [0., 0., 1., .55])

    def test_off_centre_bend_wins_over_central_false_component(self):
        cv2.line(self.mask, (550, 339), (550, 465), 255, 6)
        cv2.line(self.mask, (330, 338), (330, 375), 255, 6)
        saved = self.mask.copy()
        original = extract_centerline(self.mask, self.config, (320., 311.))
        self.assertAlmostEqual(float(original.waypoints_uv[:, 0].mean()), 330., delta=1.)
        selected = self.extract()
        self.assertAlmostEqual(float(selected.waypoints_uv[:, 0].mean()), 550., delta=1.)
        np.testing.assert_array_equal(self.mask, saved)

    def test_long_transverse_false_edge_does_not_outvote_forward_path(self):
        cv2.line(self.mask, (540, 350), (540, 465), 255, 6)
        cv2.line(self.mask, (40, 339), (400, 339), 255, 6)
        selected = self.extract()
        self.assertAlmostEqual(float(selected.waypoints_uv[:, 0].mean()), 540., delta=1.)

    def test_short_or_missing_components_remain_invalid(self):
        with self.assertRaises(ValueError):
            self.extract()
        cv2.line(self.mask, (330, 338), (330, 345), 255, 10)
        with self.assertRaises(ValueError):
            self.extract()

    def test_near_ground_mode_prefers_measured_near_tape_to_longer_side_edge(self):
        cv2.line(self.mask,(330,340),(330,465),255,6)
        cv2.line(self.mask,(30,335),(30,465),255,6)
        selected=supported_front_centerline(self.mask,self.config,(320.,311.),
            extract_centerline,self.intrinsic,self.transform,[0.,0.,1.,.55],nearest_ground=True)
        self.assertAlmostEqual(float(selected.waypoints_uv[:,0].mean()),330.,delta=1.)


if __name__ == '__main__':
    unittest.main()
