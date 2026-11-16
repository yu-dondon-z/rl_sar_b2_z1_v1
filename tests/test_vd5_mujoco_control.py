import math
import unittest

import numpy as np

from vd5_mujoco_control import (front_camera_command, quaternion_from_euler_xyz,
                               quaternion_matrix, sample_polyline_behind_position,
                               seam_target_pose, startup_fraction)


class VD5MuJoCoControlTests(unittest.TestCase):
    def test_startup_quintic_has_flat_endpoints(self):
        self.assertEqual(startup_fraction(0.0), 0.0)
        self.assertAlmostEqual(startup_fraction(1.0), 0.5)
        self.assertEqual(startup_fraction(2.0), 1.0)

    def test_polyline_target_is_arc_length_behind_pose(self):
        path = np.array(((0., 0., 0.), (1., 0., 0.), (1., 1., 0.)))
        target = sample_polyline_behind_position(path, np.array((1., 0.5, 0.)), 0.25)
        np.testing.assert_allclose(target, (1., 0.25, 0.))

    def test_startup_target_starts_at_actual_tcp(self):
        path = np.array(((-1., 0., 0.), (2., 0., 0.)))
        start = np.array((0.2, 0.1, 0.3))
        q_start = quaternion_from_euler_xyz(0., 0., 0.)
        pose, world, blend = seam_target_pose(np.zeros(3), q_start, start, q_start, path, 0.0)
        np.testing.assert_allclose(world, start)
        np.testing.assert_allclose(pose[:3], start)
        self.assertEqual(blend, 0.0)

    def test_front_command_uses_straight_path_without_yaw(self):
        path = np.column_stack((np.linspace(-2., 6., 161), np.zeros(161), np.zeros(161)))
        command, diagnostic = front_camera_command(np.array((0., 0., 0.5)),
                                                   quaternion_from_euler_xyz(0., 0., 0.), path)
        self.assertAlmostEqual(command[0], 0.2)
        self.assertEqual(command[1], 0.0)
        self.assertAlmostEqual(command[2], 0.0, places=10)
        self.assertTrue(diagnostic["path_visible"])
        self.assertGreater(diagnostic["camera_visible_points"], 1)

    def test_camera_command_is_finite_for_curved_path(self):
        x = np.linspace(-2., 6., 161)
        y = 0.04*np.sin((x-2.)*math.pi/8.0)**3
        path = np.column_stack((x, y, np.zeros_like(x)))
        command, diagnostic = front_camera_command(np.array((2.5, 0.0, 0.5)),
                                                   quaternion_from_euler_xyz(0., 0., 0.), path)
        self.assertTrue(np.isfinite(command).all())
        self.assertTrue(math.isfinite(diagnostic["curvature_m_inv"]))
        self.assertLessEqual(abs(command[2]), 0.8)

    def test_euler_quaternion_rotation_matches_z_yaw(self):
        matrix = quaternion_matrix(quaternion_from_euler_xyz(0., 0., math.pi/2))
        np.testing.assert_allclose(matrix @ np.array((1., 0., 0.)), (0., 1., 0.), atol=1e-12)


if __name__ == "__main__":
    unittest.main()
