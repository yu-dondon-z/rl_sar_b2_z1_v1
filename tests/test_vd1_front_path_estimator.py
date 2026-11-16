"""Front-only estimation regressions, with synthetic camera observations only."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
sys.path.insert(0, '/home/dong/b2_z1/scripts/vision')
from vd1_front_path_estimator import FrontPathEstimator, nominal_front_plane
from vd1_visual_guard import GuardedVD1VisualController
from run_vd1_dual_camera_bridge import process_pair
from vd1_control import SCHEMA
from vd1_frames import VD1FramePublisher


def observation(function, sequence=0, timestamp=10., generation=1):
    x = np.linspace(1.5, 3., 80)
    return dict(valid=True, capture_generation=generation, sequence=sequence,
        capture_timestamp=timestamp, ground_plane_b=[0., 0., 1., .55],
        geometry_source='front_rgb_imu_nominal_height_flat_floor',
        points_b=np.column_stack((x, function(x), np.full_like(x, -.55))).tolist())


class FrontPathTests(unittest.TestCase):
    def test_distant_s_bend_does_not_pull_the_nearest_tangent_into_blind_region(self):
        # A straight near section followed by an S bend used to generate a
        # large cubic intercept even though the observed near tangent is clear.
        x=np.linspace(1.2,3.,150)
        y=-.05-.14*(x-1.2)-.4*np.maximum(x-1.7,0)**2+.4*np.maximum(x-2.3,0)**2
        stream=observation(lambda t: t*0)
        stream['points_b']=np.c_[x,y,0*x-.55].tolist()
        geometry,status=FrontPathEstimator().update(stream)
        expected=-.05-.14*(geometry[-1][0]-1.2)
        self.assertLess(abs(geometry[-1][1]-expected),.02)
        self.assertLessEqual(np.ptp(status['front_fitting_x_range_m']),.55000001)

    def test_steering_reference_remains_inside_far_visible_samples(self):
        fn = lambda x: .025 + .04*x + .06*x*x
        geometry, diagnostic = FrontPathEstimator().update(observation(fn))
        self.assertLess(abs(geometry[-1][1]-fn(geometry[-1][0])), .015)
        self.assertFalse(diagnostic['front_near_field_is_extrapolated'])
        self.assertEqual(diagnostic['front_extrapolation_distance_m'], 0.)
        self.assertGreaterEqual(geometry[-1][0], diagnostic['front_fitting_x_range_m'][0])
        self.assertLessEqual(geometry[-1][0], diagnostic['front_fitting_x_range_m'][1])
        self.assertFalse(diagnostic['historical_points_registered'])
        self.assertFalse(diagnostic['body_motion_estimation'])

    def test_curvature_cannot_shorten_target_into_blind_region(self):
        for curvature in (-.4, -.2, 0., .2, .4):
            geometry, diagnostic = FrontPathEstimator().update(
                observation(lambda x: .03 + curvature*(x-1.75)**2))
            self.assertAlmostEqual(geometry[-1][0], 1.775)
            self.assertEqual(diagnostic['front_extrapolation_distance_m'], 0.)

    def test_negative_blind_intercept_cannot_reverse_visible_left_target(self):
        geometry, diagnostic = FrontPathEstimator().update(observation(
            lambda x: .05 + .2*(x-1.5) - .3*(x-1.5)**2))
        self.assertLess(diagnostic['front_estimated_coefficients'][0], 0.)
        self.assertGreater(geometry[-1][1], 0.)
        self.assertTrue(diagnostic['front_estimator_reliable'])

    def test_near_supported_turns_keep_the_nominal_lookahead(self):
        for sign in (-1., 1.):
            stream = observation(lambda x: 0*x)
            x = np.linspace(.25, 1., 80)
            stream['points_b'] = np.c_[x, sign*(.02+.1*x*x), 0*x-.55].tolist()
            geometry, diagnostic = FrontPathEstimator().update(stream)
            self.assertAlmostEqual(geometry[-1][0], .6)
            self.assertGreater(sign*geometry[-1][1], 0.)
            self.assertFalse(diagnostic['front_near_field_is_extrapolated'])

    def test_mirrored_observations_reverse_steering(self):
        fn = lambda x: -.03 + .04*x + .08*x*x
        left, _ = FrontPathEstimator().update(observation(fn))
        right, _ = FrontPathEstimator().update(observation(lambda x: -fn(x)))
        np.testing.assert_allclose(left[-1]*[1., -1., 1.], right[-1], atol=1e-10)

    def test_repeated_capture_cannot_shrink_uncertainty_or_refresh_time(self):
        estimator = FrontPathEstimator()
        stream = observation(lambda x: .02 + 0*x)
        first = estimator.update(stream)
        for _ in range(100):
            repeated = estimator.update(stream)
        self.assertIs(first, repeated)
        stream['capture_timestamp'] += .01
        with self.assertRaisesRegex(ValueError, 'cannot refresh'):
            estimator.update(stream)

    def test_frame_reset_discards_old_curve(self):
        estimator = FrontPathEstimator()
        estimator.update(observation(lambda x: .1 + 0*x))
        reset = observation(lambda x: -.1 + 0*x, generation=2, timestamp=11.)
        actual, status = estimator.update(reset)
        expected, _ = FrontPathEstimator().update(reset)
        np.testing.assert_allclose(actual[-1], expected[-1])
        self.assertTrue(status['front_estimator_reset'])

    def test_temporal_filter_reduces_alternating_segmentation_noise(self):
        filtered, independent = [], []
        estimator = FrontPathEstimator()
        for i in range(60):
            shift = .012 if i % 2 else -.012
            stream = observation(lambda x: .04*x + shift, i, 10. + .1*i)
            filtered.append(estimator.update(stream)[0][-1][1])
            independent.append(FrontPathEstimator().update(stream)[0][-1][1])
        self.assertLess(np.std(filtered[20:]), np.std(independent[20:]) * .85)

    def test_invalid_points_do_not_generate_a_path(self):
        for points in ([], [[float('nan'), 0., 0.]]*8, [[1., 0., -.55]]*8):
            stream = observation(lambda x: 0*x)
            stream['points_b'] = points
            with self.assertRaises(ValueError):
                FrontPathEstimator().update(stream)

    def test_rear_geometry_does_not_change_base_command(self):
        front = observation(lambda x: .015 + .02*x + .03*x*x)
        x = np.linspace(-1.3, -.8, 30)
        outputs = []
        for rear_y, rear_plane in ((-.1, [0., 0., 1., .45]), (.1, [.05, 0., 1., .65])):
            rear = dict(front, ground_plane_b=rear_plane,
                points_b=np.column_stack((x, np.full_like(x, rear_y), np.full_like(x, -.55))).tolist())
            packet = dict(schema=SCHEMA, capture_generation=1, front=front, rear=rear, valid=True)
            controller = GuardedVD1VisualController([-1.02, 0., -.3], ramp_s=0,
                                                    forward_speed=.2, front_path_estimation=True)
            outputs.append(controller.update(packet, 10.1, .02))
        np.testing.assert_array_equal(outputs[0]['base_command_b'], outputs[1]['base_command_b'])
        self.assertNotEqual(outputs[0]['tool_pose_b'], outputs[1]['tool_pose_b'])
        self.assertFalse(outputs[0]['status']['rear_used_for_steering'])

    def test_stale_images_still_expire(self):
        front = observation(lambda x: 0*x)
        x = np.linspace(-1.3, -.8, 30)
        rear = dict(front, points_b=np.column_stack((x, 0*x, 0*x-.55)).tolist())
        packet = dict(schema=SCHEMA, capture_generation=1, front=front, rear=rear, valid=True)
        controller = GuardedVD1VisualController([-1.02, 0., -.3], front_path_estimation=True)
        result = controller.update(packet, 12., .02)
        self.assertFalse(result['status']['valid'])
        self.assertIn('stale', result['status']['reason'])

    def test_front_projection_works_without_a_rear_plane(self):
        class Perception:
            def process_rear(self, *args):
                return dict(valid=False, reason='depth missing')

            def process_front(self, metadata, image):
                return dict(valid=True, ground_plane_b=metadata['ground_plane_b'])

        front, rear = process_pair(Perception(), {'imu_gravity_b': [0., 0., -1.]},
                                   None, {}, None, None, front_nominal_height_m=.55)
        self.assertTrue(front['valid'])
        self.assertFalse(rear['valid'])
        self.assertFalse(front['rear_depth_used'])
        np.testing.assert_allclose(front['ground_plane_b'], [0., 0., 1., .55])

    def test_nominal_plane_requires_imu_and_valid_height(self):
        for gravity, height in ((None, .55), ([0., 0., -2.], .55), ([0., 0., -1.], float('nan'))):
            with self.assertRaises(ValueError):
                nominal_front_plane(gravity, height)

    def test_capture_publishes_only_optional_imu_gravity_for_front_projection(self):
        calibration = dict(position=[0., 0., 0.], quaternion_wxyz=[1., 0., 0., 0.],
                           intrinsic_matrix=[[300., 0., 8.], [0., 300., 6.], [0., 0., 1.]])
        with tempfile.TemporaryDirectory() as directory:
            publisher = VD1FramePublisher(directory, calibration, calibration, {}, generation=3)
            rgb, depth = np.zeros((12, 16, 3), dtype=np.uint8), np.ones((12, 16))
            publisher.publish(rgb, rgb, depth, front_frame=0, rear_frame=0,
                              capture_timestamp=10., sim_time_s=0., front_gravity_b=[0., 0., -1.])
            front = json.loads((Path(directory)/'front/latest_frame.json').read_text())
            rear = json.loads((Path(directory)/'rear/latest_frame.json').read_text())
            self.assertEqual(front['imu_gravity_b'], [0., 0., -1.])
            self.assertNotIn('imu_gravity_b', rear)
            self.assertNotIn('base_position_w', front)
            self.assertNotIn('base_velocity_b', front)


if __name__ == '__main__':
    unittest.main()
