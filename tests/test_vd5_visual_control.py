import copy
import unittest

import numpy as np

from vd5_visual_control import (SCHEMA, VD5VisualController, front_visual_command,
                                rear_visual_target)


class VD5VisualControlTests(unittest.TestCase):
    def setUp(self):
        self.plane = np.array((0.0, 0.0, 1.0, 0.5))

    def test_front_rgb_points_drive_constant_speed_and_zero_yaw_on_straight(self):
        x = np.linspace(0.15, 2.5, 80)
        points = np.column_stack((x, np.zeros_like(x), np.full_like(x, -0.5)))
        command, status = front_visual_command(points, self.plane, 0.2)
        np.testing.assert_allclose(command, (0.2, 0.0, 0.0), atol=1e-10)
        self.assertEqual(status["command_source"], "front_rgb_centerline_plus_rear_depth_ground_plane")

    def test_front_visual_curve_changes_yaw_from_centerline_points(self):
        x = np.linspace(0.15, 2.5, 100)
        points = np.column_stack((x, 0.01 * x * x, np.full_like(x, -0.5)))
        command, status = front_visual_command(points, self.plane, 0.2)
        self.assertEqual(command[0], 0.2)
        self.assertGreater(command[2], 0.0)
        self.assertTrue(np.isfinite(command).all())
        self.assertGreater(status["curvature_m_inv"], 0.0)

    def test_rear_target_is_measured_arc_length_behind_base(self):
        x = np.linspace(-1.5, -0.2, 100)
        points = np.column_stack((x, np.zeros_like(x), np.full_like(x, -0.5)))
        target, status = rear_visual_target(points, self.plane, 0.70)
        self.assertAlmostEqual(target[0], -0.70, places=2)
        self.assertAlmostEqual(target[2], -0.5)
        self.assertAlmostEqual(status["rear_target_distance_behind_base_m"], 0.70)

    def test_work_distance_changes_target_without_changing_the_observed_path(self):
        x = np.linspace(-1.5, -.4, 100)
        points = np.column_stack((x, .02*x, np.full_like(x, -.5)))
        original = points.copy()
        targets = []
        for distance in (.70, .80, .85):
            target, status = rear_visual_target(points, self.plane, distance)
            self.assertAlmostEqual(float(np.linalg.norm(target[:2])), distance, delta=.002)
            self.assertAlmostEqual(status['rear_target_distance_behind_base_m'], distance)
            self.assertAlmostEqual(target[2], -.5)
            targets.append(target[0])
        self.assertGreater(targets[0], targets[1])
        self.assertGreater(targets[1], targets[2])
        np.testing.assert_array_equal(points, original)

    def test_rear_target_refuses_unseen_extrapolation(self):
        x = np.linspace(-0.5, -0.2, 24)
        points = np.column_stack((x, np.zeros_like(x), np.full_like(x, -0.5)))
        with self.assertRaisesRegex(ValueError, "support is too short"):
            rear_visual_target(points, self.plane, 0.70)

    def test_occluded_target_requires_explicit_bounded_estimate(self):
        x = np.linspace(-.77, -.62, 24)
        points = np.column_stack((x, np.zeros_like(x), np.full_like(x, -.5)))
        with self.assertRaisesRegex(ValueError, 'support is too short'):
            rear_visual_target(points, self.plane, .80)
        target, status = rear_visual_target(points, self.plane, .80, .05)
        np.testing.assert_allclose(target, (-.80, 0., -.5), atol=1.e-10)
        self.assertFalse(status['rear_target_directly_observed'])
        self.assertEqual(status['rear_target_source'], 'rear_rgbd_local_occlusion_estimate')
        self.assertAlmostEqual(status['rear_target_occlusion_extension_m'], .03)
        self.assertGreaterEqual(status['rear_occlusion_fit_points'], 8)
        self.assertLess(status['rear_occlusion_fit_residual_p90_m'], 1.e-10)
        with self.assertRaisesRegex(ValueError, 'support is too short'):
            rear_visual_target(points, self.plane, .85, .05)

    def test_occlusion_estimate_refuses_short_or_curled_visible_support(self):
        x = np.linspace(-.77, -.69, 24)
        points = np.column_stack((x, np.zeros_like(x), np.full_like(x, -.5)))
        with self.assertRaisesRegex(ValueError, 'spanning at least'):
            rear_visual_target(points, self.plane, .80, .05)
        x = np.linspace(-.77, -.62, 24)
        points = np.column_stack((x, .03*np.sin(np.linspace(0., 2.*np.pi, len(x))),
                                  np.full_like(x, -.5)))
        with self.assertRaisesRegex(ValueError, 'not reliable enough'):
            rear_visual_target(points, self.plane, .80, .05)

    def test_occlusion_estimate_rejects_unsupported_limit(self):
        x = np.linspace(-.77, -.62, 24)
        points = np.column_stack((x, np.zeros_like(x), np.full_like(x, -.5)))
        for limit in (-.01, .051, np.nan):
            with self.assertRaisesRegex(ValueError, 'between zero and 0.05'):
                rear_visual_target(points, self.plane, .80, limit)

    def test_rear_endpoint_pixel_hook_does_not_rotate_the_whole_path(self):
        x = np.linspace(-.836, -.50, 40)
        points = np.column_stack((x, np.zeros_like(x), np.full_like(x, -.5)))
        # Reproduce the tiny, nearly lateral endpoint segment from seq 1123.
        points[-1, 1] = -.013
        target, status = rear_visual_target(points, self.plane, .70)
        self.assertAlmostEqual(target[0], -.70, delta=.01)
        self.assertAlmostEqual(target[1], 0., delta=.005)
        self.assertGreater(status['rear_tangent_fit_points'], 3)
        self.assertGreater(status['rear_tangent_b'][0], .99)

    def test_rear_target_bridges_only_the_small_body_occluded_near_field(self):
        x = np.linspace(-2.0, -0.85, 60)
        points = np.column_stack((x, np.zeros_like(x), np.full_like(x, -0.5)))
        target, status = rear_visual_target(points, self.plane, 0.70)
        self.assertAlmostEqual(target[0], -0.70, places=2)
        self.assertAlmostEqual(status["rear_target_near_field_extension_m"], 0.15, places=2)

        x = np.linspace(-2.0, -1.2, 60)
        points = np.column_stack((x, np.zeros_like(x), np.full_like(x, -0.5)))
        with self.assertRaisesRegex(ValueError, "body-occluded near field"):
            rear_visual_target(points, self.plane, 0.70)

    def test_rear_target_rejects_far_side_false_centerline(self):
        x = np.linspace(-2.0, -0.7, 60)
        points = np.column_stack((x, np.full_like(x, 0.48), np.full_like(x, -0.5)))
        with self.assertRaisesRegex(ValueError, "working envelope"):
            rear_visual_target(points, self.plane, 0.70)

    def test_controller_requires_paired_fresh_visual_measurements(self):
        x_front = np.linspace(0.15, 2.0, 40)
        x_rear = np.linspace(-1.5, -0.2, 40)
        packet = {
            "schema": SCHEMA, "valid": True, "capture_generation": 4,
            "front": {"valid": True, "capture_generation": 4, "sequence": 2,
                      "capture_timestamp": 10.0,
                      "points_b": np.column_stack((x_front, np.zeros_like(x_front),
                                                    np.full_like(x_front, -0.5))).tolist(),
                      "ground_plane_b": self.plane.tolist()},
            "rear": {"valid": True, "capture_generation": 4, "sequence": 2,
                     "capture_timestamp": 10.0,
                     "points_b": np.column_stack((x_rear, np.zeros_like(x_rear),
                                                   np.full_like(x_rear, -0.5))).tolist(),
                     "ground_plane_b": self.plane.tolist()},
        }
        controller = VD5VisualController((0.2, 0.0, -0.3), (1.0, 0.0, 0.0, 0.0))
        output = controller.update(packet, now=10.01, dt=0.02)
        self.assertTrue(output["status"]["valid"])
        self.assertFalse(output["status"]["uses_simulator_seam"])
        self.assertAlmostEqual(output["status"]["rear_target_b"][0], -0.7, places=2)
        stale = controller.update(packet, now=11.1, dt=0.02)
        self.assertFalse(stale["status"]["valid"])
        self.assertIn("stale", stale["status"]["reason"])

    def test_rejected_new_image_reports_the_actual_reason_and_its_own_age(self):
        packet = dict(schema=SCHEMA, valid=False, reason='rear: geometric seam path is too short',
                      front=dict(capture_timestamp=10.), rear=dict(capture_timestamp=10.))
        controller = VD5VisualController((.2, 0., -.3), (1., 0., 0., 0.))
        output = controller.update(packet, now=10.1, dt=.02)
        self.assertFalse(output['status']['valid'])
        self.assertIn('geometric seam path is too short', output['status']['reason'])
        self.assertAlmostEqual(output['status']['latest_packet_capture_age_s'], .1)
        self.assertIsNone(output['status']['capture_age_s'])

    def hold_fixture(self):
        def stream(x):
            return dict(valid=True, capture_generation=4, sequence=2, capture_timestamp=10.,
                        points_b=np.c_[x, np.zeros_like(x), np.full_like(x, -.5)].tolist(),
                        ground_plane_b=self.plane.tolist())
        packet=dict(schema=SCHEMA, valid=True, capture_generation=4,
                    front=stream(np.linspace(.15, 2., 40)),
                    rear=stream(np.linspace(-1.5, -.2, 40)))
        controller=VD5VisualController((-.7, 0., -.5), (1., 0., 0., 0.),
                                       ramp_s=.01, rear_hold_s=.30)
        controller.update(packet, now=10.01, dt=.02)
        missing=copy.deepcopy(packet)
        missing.update(valid=False, reason='rear: geometric seam path is too short')
        for stream in (missing['front'], missing['rear']):
            stream.update(sequence=3, capture_timestamp=10.10)
        missing['rear'].update(valid=False, reason='geometric seam path is too short', points_b=[])
        return controller, missing

    def test_short_rear_loss_holds_without_renewing_measurement(self):
        controller, missing=self.hold_fixture()
        output=controller.update(missing, now=10.11, dt=.02)
        status=output['status']
        self.assertTrue(status['valid'])
        self.assertFalse(status['measurement_valid'])
        self.assertEqual(status['control_state'], 'rear_hold')
        self.assertEqual(status['rear_held_source_sequence'], 2)
        self.assertEqual(status['front_source_sequence'], 3)
        self.assertAlmostEqual(status['capture_age_s'], .11)
        self.assertAlmostEqual(status['rear_hold_elapsed_s'], .10)
        self.assertAlmostEqual(controller._accepted_at, 10.01)
        output=controller.update(missing, now=10.32, dt=.02)
        self.assertFalse(output['status']['valid'])
        self.assertEqual(output['status']['control_state'], 'braking')
        recovered=copy.deepcopy(missing)
        recovered.update(valid=True)
        recovered['rear'].update(valid=True, reason='ok', points_b=np.c_[
            np.linspace(-1.5,-.2,40), np.zeros(40), np.full(40,-.5)].tolist())
        for stream in (recovered['front'], recovered['rear']):
            stream.update(sequence=4, capture_timestamp=10.33)
        output=controller.update(recovered, now=10.34, dt=.02)
        self.assertTrue(output['status']['measurement_valid'])
        self.assertEqual(output['status']['control_state'], 'tracking')
        self.assertEqual(output['status']['source_sequence'], 4)

    def test_rear_hold_refuses_freshness_identity_and_depth_failures(self):
        for case in ('future', 'old', 'generation', 'pair', 'front', 'depth', 'plane', 'forged'):
            controller, packet=self.hold_fixture()
            if case=='future': packet['rear']['capture_timestamp']=11.
            if case=='old': packet['rear']['capture_timestamp']=8.
            if case=='generation': packet['capture_generation']=5
            if case=='pair': packet['rear']['sequence']=4
            if case=='front': packet['front']['valid']=False
            if case=='depth': packet['rear']['reason']='rear seam depth support too low (0.0)'
            if case=='plane':
                packet['rear']['ground_plane_b']=[0.,0.,1.,.6]
                packet['front']['ground_plane_b']=[0.,0.,1.,.6]
            if case=='forged':
                packet['front']['sequence']=packet['rear']['sequence']=2
            output=controller.update(packet, now=10.11, dt=.02)
            self.assertFalse(output['status']['valid'], case)

    def test_hold_refuses_large_curvature_and_is_disabled_by_default(self):
        controller, packet=self.hold_fixture()
        x=np.linspace(.15, 2., 40)
        packet['front']['points_b']=np.c_[x, .3*x*x, np.full_like(x,-.5)].tolist()
        self.assertFalse(controller.update(packet, now=10.11, dt=.02)['status']['valid'])
        controller, packet=self.hold_fixture()
        controller.rear_hold_s=0.
        self.assertFalse(controller.update(packet, now=10.11, dt=.02)['status']['valid'])


if __name__ == "__main__":
    unittest.main()
