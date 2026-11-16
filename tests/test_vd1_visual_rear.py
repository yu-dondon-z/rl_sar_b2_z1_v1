"""Rear RGB-D processing must work with no visible TCP or colored markers."""
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import cv2
import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
sys.path.insert(0, '/home/dong/b2_z1/scripts/vision')
from dual_camera_perception import PerceptionConfig
from vd1_visual_rear import SeamOnlyDualPerception
from vd1_mujoco_scene import build_scene, validate_model


class RearSeamOnlyTests(unittest.TestCase):
    def setUp(self):
        self.perception = SeamOnlyDualPerception(PerceptionConfig(mode='classical'))
        self.rgb = np.full((480, 640, 3), 200, dtype=np.uint8)
        cv2.line(self.rgb, (320, 70), (320, 410), (0, 0, 0), 7)
        self.depth = np.full((480, 640), 550, dtype=np.uint16)
        self.metadata = dict(depth_unit_m=.001, depth_aligned_to_rgb=True,
            intrinsic_matrix=[[460., 0., 320.], [0., 460., 240.], [0., 0., 1.]],
            T_base_camera=[[0., -1., 0., -1.02], [-1., 0., 0., 0.],
                           [0., 0., -1., 0.], [0., 0., 0., 1.]])

    def test_no_markers_needed_for_measured_rear_seam(self):
        with patch.object(self.perception, '_markers', side_effect=AssertionError('TCP detector called')):
            result = self.perception.process_rear(self.metadata, self.rgb, self.depth)
        self.assertTrue(result['valid'], result['reason'])
        self.assertNotIn('tcp_measured_b', result)
        self.assertNotIn('tcp_error_m', result)
        self.assertNotIn('markers', result)
        points = np.array(result['points_b'])
        self.assertLess(points[:, 0].min(), -1.02)
        self.assertGreater(points[:, 0].max(), -1.02)
        np.testing.assert_allclose(points[:, 1], 0., atol=.003)
        np.testing.assert_allclose(points[:, 2], -.55, atol=.003)

    def test_missing_depth_or_missing_seam_is_still_invalid(self):
        for rgb, depth in ((self.rgb, np.zeros_like(self.depth)),
                           (np.full_like(self.rgb, 200), self.depth)):
            result = self.perception.process_rear(self.metadata, rgb, depth)
            self.assertFalse(result['valid'])

    def test_foreground_tool_edge_does_not_become_seam(self):
        self.depth[:, 313:328] = 350  # Dark edge elevated 20 cm above the road.
        result = self.perception.process_rear(self.metadata, self.rgb, self.depth)
        self.assertFalse(result['valid'])

    def test_removing_rendered_markers_keeps_robot_dynamics(self):
        with tempfile.TemporaryDirectory(prefix='vd1_no_markers_', dir='/dev/shm') as cache:
            path = build_scene(.3, cache, tool_markers=True)
            original = mujoco.MjModel.from_xml_path(str(path))
            path = build_scene(.3, cache, tool_markers=False)
            markerless = mujoco.MjModel.from_xml_path(str(path))
        self.assertEqual(original.ngeom-markerless.ngeom, 2)
        for name in ('tool_marker_cyan', 'tool_marker_magenta'):
            self.assertEqual(mujoco.mj_name2id(markerless, mujoco.mjtObj.mjOBJ_GEOM, name), -1)
        self.assertGreaterEqual(mujoco.mj_name2id(markerless, mujoco.mjtObj.mjOBJ_SITE, 'tool_tcp'), 0)
        np.testing.assert_array_equal(markerless.body_mass, original.body_mass)
        np.testing.assert_array_equal(markerless.body_inertia, original.body_inertia)
        self.assertEqual(validate_model(markerless)['nu'], 18)


if __name__ == '__main__':
    unittest.main()
