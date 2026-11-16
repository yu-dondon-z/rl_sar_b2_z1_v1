"""CPU tests for measured RGB-D geometry; no Isaac or simulator state input."""

from dataclasses import replace
import unittest

import cv2
import numpy as np

from dual_camera_perception import (
    DualCameraPerception, PerceptionConfig, camera_geometry, fit_ground_plane,
    fit_known_radius_sphere, intersect_pixels_with_plane, nearest_point_on_polyline,
    project_to_plane, unproject,
)


def synthetic_rgbd():
    """Ray-traced images of a plane, a tape, and two *surface-depth* spheres."""
    height, width = 360, 480
    intrinsic = np.array([[220., 0, 240.], [0, 220., 180.], [0, 0, 1.]])
    transform = np.diag([1., -1., -1., 1.])
    yy, xx = np.mgrid[:height, :width]
    rays = np.stack(((xx - 240) / 220, (yy - 180) / 220, np.ones_like(xx)), axis=-1)
    depth = np.full((height, width), .6)
    image = np.full((height, width, 3), 170, dtype=np.uint8)
    # Plane y=-.10 in base -> optical y=.10, so the seam has an independently
    # known 4 cm lateral error from the measured TCP (base y=-.14).
    ground_y_b = -rays[..., 1] * depth
    image[np.abs(ground_y_b + .10) < .007] = 5
    for center_b, color in (([-.15, -.14, -.35], [255, 255, 0]),
                             ([-.05, -.14, -.35], [255, 0, 255])):
        center = transform[:3, :3].T @ np.asarray(center_b)
        a = np.sum(rays*rays, axis=-1)
        b = np.sum(rays*center, axis=-1)
        discriminant = b*b - a * (center @ center - .035**2)
        inside = discriminant >= 0
        z = (b - np.sqrt(np.maximum(discriminant, 0))) / a
        visible = inside & (z > 0) & (z < depth)
        depth[visible] = z[visible]
        image[visible] = color
    metadata = {"sequence": 42, "capture_timestamp": 321.125,
                "capture_generation": 9, "camera_name": "rear_d435i",
                "intrinsic_matrix": intrinsic.tolist(), "T_base_camera": transform.tolist(),
                "depth_unit_m": .001, "depth_aligned_to_rgb": True}
    return image, np.rint(depth*1000).astype(np.uint16), metadata


class GeometryTests(unittest.TestCase):
    def test_plane_intersection_includes_camera_translation_and_tilt(self):
        intrinsic = np.array([[500., 0, 320.], [0, 480., 240.], [0, 0, 1.]])
        transform = np.eye(4)
        transform[:3, :3] = np.diag([1, -1, -1])
        transform[:3, 3] = [.2, -.1, .15]
        plane = np.array([.1, -.05, 1., .6])
        uv = np.array([[320., 240.], [410., 280.], [260., 200.]])
        points, valid = intersect_pixels_with_plane(uv, intrinsic, transform, plane)
        self.assertTrue(valid.all())
        self.assertLess(np.max(np.abs(points @ plane[:3] + plane[3])), 1e-10)
        rays_in_camera = (points - transform[:3, 3]) @ transform[:3, :3]
        reprojection = rays_in_camera @ intrinsic.T
        np.testing.assert_allclose(reprojection[:, :2] / reprojection[:, 2:], uv, atol=1e-10)

    def test_ground_ransac_rejects_foreground_and_reports_measured_tilt(self):
        generator = np.random.default_rng(16)
        xy = generator.uniform(-1, 1, (1800, 2))
        z = -.6 - .07*xy[:, 0] + .04*xy[:, 1] + generator.normal(0, .001, len(xy))
        ground = np.column_stack((xy, z))
        foreground = generator.uniform([-.5, -.5, -.4], [.5, .5, .3], (500, 3))
        plane, diagnostics = fit_ground_plane(np.vstack((ground, foreground)), PerceptionConfig())
        expected = np.array([.07, -.04, 1., .6])
        expected /= np.linalg.norm(expected[:3])
        np.testing.assert_allclose(plane, expected, atol=.001)
        self.assertGreater(diagnostics["ground_inlier_ratio"], .75)
        self.assertLess(diagnostics["ground_rms_m"], .002)

    def test_ground_plane_cannot_be_inferred_from_no_depth(self):
        with self.assertRaisesRegex(ValueError, "not enough valid"):
            fit_ground_plane(np.empty((0, 3)), PerceptionConfig())

    def test_nearest_polyline_uses_segments_not_discrete_waypoints(self):
        path = np.array([[-1., 0, -.6], [0, 0, -.6], [1, 0, -.6]])
        projected = project_to_plane(np.array([.41, .04, -.35]), [0, 0, 1, .6])
        nearest = nearest_point_on_polyline(projected, path)
        np.testing.assert_allclose(nearest, [.41, 0, -.6])
        self.assertAlmostEqual(np.linalg.norm(nearest - projected), .04)


class ImageClosedLoopTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.perception = DualCameraPerception(PerceptionConfig(mode="classical"))

    def test_surface_depth_recovers_sphere_centers_and_four_cm_tcp_error(self):
        rgb, depth, metadata = synthetic_rgbd()
        result = self.perception.process_rear(metadata, rgb, depth)
        self.assertTrue(result["valid"], result)
        np.testing.assert_allclose(result["tcp_measured_b"], [-.25, -.14, -.35], atol=.002)
        self.assertAlmostEqual(result["tcp_error_m"], .04, delta=.003)
        self.assertEqual(result["capture_timestamp"], 321.125)
        self.assertEqual(result["capture_generation"], 9)
        self.assertEqual(result["confidence_kind"], "dark_pixel_contrast")
        # Surface median is >2 cm away from the sphere center: fitting radius
        # matters and cannot be replaced with a center-pixel depth probe.
        cyan = result["markers"]["cyan"]
        self.assertLess(cyan["fit_rms_m"], .001)
        self.assertGreater(cyan["surface_depth_samples"], 100)

    def test_front_uses_rgb_and_rear_measured_plane_without_depth_argument(self):
        rgb, depth, metadata = synthetic_rgbd()
        rear = self.perception.process_rear(metadata, rgb, depth)
        front_metadata = {**metadata, "camera_name": "front_rgb", "ground_plane_b": rear["ground_plane_b"]}
        front = self.perception.process_front(front_metadata, rgb)
        self.assertTrue(front["valid"], front)
        points = np.asarray(front["points_b"])
        self.assertLess(np.max(np.abs(points[:, 1] + .10)), .003)
        self.assertEqual(front["geometry_source"], "front_rgb_rays_intersect_rear_depth_plane")
        self.assertNotIn("tcp_measured_b", front)

    def test_front_keeps_measured_local_path_when_horizon_points_are_ill_conditioned(self):
        rgb = np.full((480, 640, 3), 170, np.uint8)
        cv2.line(rgb, (320, 280), (320, 479), (5, 5, 5), 7)
        # A bottom padding/border false positive touches the real line. It
        # must not become a metre-long lateral branch in the extracted path.
        cv2.line(rgb, (0, 478), (639, 478), (5, 5, 5), 3)
        transform = np.eye(4)
        transform[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
        transform[:3, 3] = [.4, 0, 0]
        metadata = {"intrinsic_matrix": [[460, 0, 320], [0, 460, 240], [0, 0, 1]],
                    "T_base_camera": transform.tolist(), "ground_plane_b": [0, 0, 1, .6]}
        result = self.perception.process_front(metadata, rgb)
        self.assertTrue(result["valid"], result)
        points = np.asarray(result["points_b"])
        self.assertLessEqual(np.max(points[:, 0]), 3.)
        self.assertGreater(np.min(points[:, 0]), 1.)
        self.assertGreater(result["path_length_m"], 1.)
        self.assertLess(np.max(np.abs(points[:, 1])), .005)

    def test_missing_plane_is_invalid_instead_of_nominal_ground_fallback(self):
        rgb, _, metadata = synthetic_rgbd()
        result = self.perception.process_front(metadata, rgb)
        self.assertFalse(result["valid"])
        self.assertIn("synchronized rear-measured", result["reason"])

    def test_missing_marker_does_not_fallback_to_robot_tcp(self):
        rgb, depth, metadata = synthetic_rgbd()
        rgb[(rgb[..., 0] > 200) & (rgb[..., 2] > 200)] = 170
        result = self.perception.process_rear(metadata, rgb, depth)
        self.assertFalse(result["valid"])
        self.assertIn("magenta TCP marker", result["reason"])
        self.assertIn("ground_plane_b", result)
        self.assertNotIn("tcp_measured_b", result)

    def test_invalid_depth_rejects_without_fabricating_measurement(self):
        rgb, depth, metadata = synthetic_rgbd()
        result = self.perception.process_rear(metadata, rgb, np.zeros_like(depth))
        self.assertFalse(result["valid"])
        self.assertIn("not enough valid rear depth", result["reason"])
        self.assertNotIn("ground_plane_b", result)
        self.assertNotIn("tcp_measured_b", result)

    def test_misaligned_depth_rejects_before_geometry(self):
        rgb, depth, metadata = synthetic_rgbd()
        result = self.perception.process_rear({**metadata, "depth_aligned_to_rgb": False}, rgb, depth)
        self.assertFalse(result["valid"])
        self.assertIn("not aligned", result["reason"])

    def test_neural_mode_preserves_real_confidence_instead_of_inventing_it(self):
        rgb, depth, metadata = synthetic_rgbd()
        class LowConfidenceSegmenter:
            def predict_bgr(self, image):
                from types import SimpleNamespace
                mask = cv2.inRange(image, (0, 0, 0), (10, 10, 10))
                return SimpleNamespace(mask=mask, probability=np.full(mask.shape, .2), inference_ms=7.)
        perception = DualCameraPerception(PerceptionConfig(), segmenter=LowConfidenceSegmenter())
        result = perception.process_rear(metadata, rgb, depth)
        self.assertFalse(result["valid"])
        self.assertIn("confidence too low", result["reason"])
        self.assertEqual(result["confidence_kind"], "neural_probability")

    def test_stronger_neural_gun_outline_cannot_be_projected_into_a_fake_ground_seam(self):
        rgb, depth, metadata = synthetic_rgbd()
        # This broad elevated gun surface is nearer the image anchor and has
        # stronger neural support than the real thin ground seam. Its depth
        # is 25 cm above ground; it must be excluded before skeleton selection.
        rgb[230:243, 30:450] = 0
        depth[230:243, 30:450] = 350
        class GunBiasedSegmenter:
            def predict_bgr(self, image):
                from types import SimpleNamespace
                mask = cv2.inRange(image, (0, 0, 0), (10, 10, 10))
                probability = np.full(mask.shape, .95)
                probability[230:243, 30:450] = .999
                return SimpleNamespace(mask=mask, probability=probability, inference_ms=1.)
        perception = DualCameraPerception(PerceptionConfig(), segmenter=GunBiasedSegmenter())
        unfiltered_uv, _, _ = perception._centerline(rgb, .65)
        self.assertGreater(float(np.median(unfiltered_uv[:, 1])), 225.)
        result = perception.process_rear(metadata, rgb, depth)
        self.assertTrue(result["valid"], result)
        self.assertAlmostEqual(result["tcp_error_m"], .04, delta=.003)
        np.testing.assert_allclose(np.asarray(result["points_b"])[:, 1], -.10, atol=.003)
        self.assertGreater(result["rear_elevated_foreground_ratio"], .03)
        self.assertLess(max(point[1] for point in result["waypoints_uv"]), 225.)


if __name__ == "__main__":
    unittest.main(verbosity=2)
