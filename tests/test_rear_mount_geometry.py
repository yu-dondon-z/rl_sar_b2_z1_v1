"""Small CPU-only checks of the static mounting study's projection math."""
from pathlib import Path
import sys
import unittest

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts/diagnostics'))
from check_rear_mount_candidate import ground_footprint, project, true_intervals


class MountProjection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        item = yaml.safe_load((ROOT/'config/vd5_measured_cameras.yaml').read_text())['rear_d435i']
        cls.camera = np.array(item['T_base_camera'], float)
        cls.intrinsic = np.array(item['intrinsic_matrix'], float)

    def test_recorded_camera_nominal_footprint(self):
        footprint = ground_footprint(self.camera, self.intrinsic, (640, 480))
        np.testing.assert_allclose(footprint[:, 2], 0., atol=1e-12)
        self.assertAlmostEqual(float(footprint[:, 0].max()), -.61275073697, places=8)
        self.assertAlmostEqual(float(footprint[:, 0].min()), -1.41285130693, places=8)

    def test_camera_height_is_base_height_plus_installation_offset(self):
        footprint = ground_footprint(self.camera, self.intrinsic, (640, 480), base_height=.45)
        self.assertAlmostEqual(float(footprint[:, 0].max()), -.595831924, places=6)

    def test_moving_camera_rearward_reduces_forward_coverage(self):
        shifted = self.camera.copy()
        shifted[0, 3] -= .1
        original = ground_footprint(self.camera, self.intrinsic, (640, 480))
        actual = ground_footprint(shifted, self.intrinsic, (640, 480))
        np.testing.assert_allclose(actual[:, 0], original[:, 0]-.1)

    def test_workpoint_projection_inside_image(self):
        camera = self.camera.copy()
        camera[2, 3] += .5
        uv, visible = project(np.array([[-.8, 0, 0], [0, 0, 0]]), camera, self.intrinsic, (640, 480))
        self.assertEqual(visible.tolist(), [True, False])
        np.testing.assert_allclose(uv[0], [323.6682434082, 298.3385465375], atol=1e-8)

    def test_intervals_never_bridge_unseen_gap(self):
        intervals = true_intervals(np.arange(6)*.01, np.array([True, True, False, True, True, False]))
        self.assertEqual(len(intervals), 2)
        self.assertAlmostEqual(intervals[0]['length_m'], .01)
        self.assertAlmostEqual(intervals[1]['x_start_m'], .03)
        self.assertEqual(true_intervals(np.arange(3), np.zeros(3, bool)), [])

    def test_reject_above_horizon_camera(self):
        with self.assertRaises(ValueError):
            ground_footprint(np.eye(4), self.intrinsic, (640, 480))


if __name__ == '__main__':
    unittest.main()
