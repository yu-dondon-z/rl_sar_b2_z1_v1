import copy
import json
import math
import unittest

import numpy as np

from vd5_mujoco_control import (_tracking_errors, quaternion_from_euler_xyz, quaternion_matrix,
                               startup_fraction)
from vd5_visual_control import SCHEMA, front_visual_command
from vd12_visual_control import (VD12VisualController, _ObservedEnvelopeRearPathMemory,
                                cylinder_endface_height, supported_front_geometry)


def packet(now=10., sequence=0, generation=4, plane=(0., 0., 1., .5)):
    plane = np.asarray(plane, dtype=float)
    def stream(xs):
        xs = np.asarray(xs)
        zs = -(plane[0] * xs + plane[3]) / plane[2]
        return dict(valid=True, capture_generation=generation, sequence=sequence,
                    capture_timestamp=now, ground_plane_b=plane.tolist(),
                    points_b=np.c_[xs, np.zeros_like(xs), zs].tolist())
    return dict(schema=SCHEMA, valid=True, capture_generation=generation,
                front=stream(np.linspace(.15, 2., 60)),
                rear=stream(np.linspace(-1.5, -.2, 60)))


class VD12VisualControlTests(unittest.TestCase):
    def controller(self, **kwargs):
        return VD12VisualController((-.95, 0., .35),
                                    quaternion_from_euler_xyz(0., .3, math.pi), **kwargs)

    def advance(self, controller, elapsed=2.):
        output = None
        for tick in range(round(elapsed / .02) + 1):
            output = controller.update(packet(10. + tick * .02, tick),
                                       now=10. + tick * .02, dt=.02)
            self.assertTrue(output['status']['valid'], output['status']['reason'])
        return output

    def test_initial_command_preserves_fk_pose_and_zero_base(self):
        controller = self.controller()
        output = controller.update(packet(), now=10., dt=.02)
        np.testing.assert_allclose(output['tool_pose_b'][:3], [-.95, 0., .35])
        np.testing.assert_allclose(output['tool_pose_b'][3:], controller._start_quaternion)
        np.testing.assert_array_equal(output['base_command_b'], [0., 0., 0.])
        self.assertEqual(output['status']['startup_blend'], 0.)
        self.assertFalse(output['status']['uses_world_pose'])
        self.assertFalse(output['status']['uses_simulator_seam'])

    def test_two_second_startup_reaches_rim_goal_without_extra_filter(self):
        controller = self.controller()
        output = self.advance(controller)
        status = output['status']
        self.assertAlmostEqual(status['startup_blend'], 1.)
        height = .028 * math.sin(.45) + .005
        np.testing.assert_allclose(output['tool_pose_b'][:3], [-.8, 0., -.5 + height], atol=1.e-10)
        np.testing.assert_allclose(output['base_command_b'], [.2, 0., 0.], atol=1.e-10)
        self.assertAlmostEqual(np.linalg.norm(output['tool_pose_b'][3:]), 1.)
        self.assertEqual(status['target_filter'], 'none')
        self.assertTrue(status['rear_target_directly_observed'])
        self.assertEqual(status['capture_generation'], 4)
        self.assertEqual(status['source_sequence'], 100)
        self.assertAlmostEqual(status['capture_age_s'], 0.)

    def test_mid_startup_matches_teacher_blend_and_quaternion_normalization(self):
        controller = self.controller()
        output = self.advance(controller, 1.)
        fraction = startup_fraction(1., 2.)
        self.assertAlmostEqual(output['status']['startup_blend'], fraction)
        goal = np.asarray(output['status']['tool_goal_b'])
        expected = (1. - fraction) * np.array([-.95, 0., .35]) + fraction * goal
        np.testing.assert_allclose(output['tool_pose_b'][:3], expected, atol=1.e-12)
        self.assertAlmostEqual(output['base_command_b'][0], .2 * fraction)
        self.assertAlmostEqual(np.linalg.norm(output['tool_pose_b'][3:]), 1.)

    def test_plane_normal_rim_formula_matches_horizontal_teacher(self):
        for axis_z in (-1., -.5, 0., .5, 1.):
            axis = np.array([math.sqrt(1. - axis_z**2), 0., axis_z])
            expected = max(1.1 * axis_z, 0.) + .028 * math.sqrt(1. - axis_z**2) + .005
            self.assertAlmostEqual(cylinder_endface_height(axis, [0., 0., 1., .5]), expected)
        for axis in ([0., 0., 0.], [math.nan, 0., 1.]):
            with self.assertRaises(ValueError):
                cylinder_endface_height(axis, [0., 0., 1., .5])

    def test_tilted_measured_plane_keeps_full_tube_surface_clearance(self):
        controller = self.controller()
        plane = np.array([.10, -.07, 1., .5])
        plane /= np.linalg.norm(plane[:3])
        for tick in range(101):
            output = controller.update(packet(10. + tick * .02, tick, plane=plane),
                                       10. + tick * .02, .02)
        self.assertTrue(output['status']['valid'], output['status']['reason'])
        pose = np.array(output['tool_pose_b'])
        axis = quaternion_matrix(pose[3:])[:, 0]
        projected = axis @ plane[:3]
        extent = max(1.1 * projected, 0.) + .028 * math.sqrt(max(1. - projected**2, 0.))
        minimum = pose[:3] @ plane[:3] + plane[3] - extent
        self.assertAlmostEqual(minimum, .005, places=10)

    def test_identity_freshness_and_clock_failures_stop_immediately(self):
        for failure in ('stale', 'future', 'missing', 'pair', 'generation',
                        'backwards_sequence', 'new_seq_old_stamp',
                        'same_seq_new_stamp', 'same_seq_changed_path', 'clock'):
            with self.subTest(failure=failure):
                controller = self.controller()
                first = packet(10., 3)
                controller.update(first, 10., .02)
                before = controller.update(first, 10.02, .02)['tool_pose_b']
                candidate = copy.deepcopy(first)
                now = 10.04
                if failure == 'stale': now = 10.3
                if failure == 'future':
                    for stream in ('front', 'rear'): candidate[stream]['capture_timestamp'] = 11.
                if failure == 'missing': candidate = None
                if failure == 'pair': candidate['rear']['sequence'] = 4
                if failure == 'generation':
                    candidate['capture_generation'] = 5
                    for stream in ('front', 'rear'): candidate[stream]['capture_generation'] = 5
                if failure == 'backwards_sequence':
                    for stream in ('front', 'rear'): candidate[stream]['sequence'] = 2
                if failure == 'new_seq_old_stamp':
                    for stream in ('front', 'rear'): candidate[stream]['sequence'] = 4
                if failure == 'same_seq_new_stamp':
                    for stream in ('front', 'rear'): candidate[stream]['capture_timestamp'] = 10.03
                if failure == 'same_seq_changed_path': candidate['rear']['points_b'][0][1] = .001
                if failure == 'clock': now = 10.01
                output = controller.update(candidate, now, .02)
                self.assertFalse(output['status']['valid'])
                self.assertTrue(output['status']['failure_latched'])
                np.testing.assert_array_equal(output['base_command_b'], [0., 0., 0.])
                np.testing.assert_array_equal(output['tool_pose_b'], before)
                self.assertEqual(output['status']['tool_pose_source'], 'last_setpoint_not_reobserved')
                self.assertFalse(output['status']['rear_target_directly_observed'])

    def test_invalid_latches_and_fresh_frames_cannot_restart_episode(self):
        controller = self.controller()
        output = controller.update(None, 10., .02)
        self.assertFalse(output['status']['valid'])
        output = controller.update(packet(10.02, 1), 10.02, .02)
        self.assertFalse(output['status']['valid'])
        self.assertIn('latched', output['status']['reason'])
        self.assertIsNone(output['status']['capture_generation'])

    def test_repeated_fresh_frame_keeps_identity_without_renewing_capture(self):
        controller = self.controller()
        image = packet(10., 2)
        first = controller.update(image, 10., .02)
        repeat = controller.update(image, 10.10, .02)
        self.assertTrue(first['status']['measurement_is_new'])
        self.assertTrue(repeat['status']['valid'])
        self.assertFalse(repeat['status']['measurement_is_new'])
        self.assertAlmostEqual(repeat['status']['capture_age_s'], .10)
        self.assertEqual(repeat['status']['capture_timestamps'], [10., 10.])
        stale = controller.update(image, 10.16, .02)
        self.assertFalse(stale['status']['valid'])
        self.assertAlmostEqual(stale['status']['capture_age_s'], .16)

    def test_small_near_field_gap_uses_explicitly_labelled_bounded_estimate(self):
        controller = self.controller()
        image = packet()
        xs = np.linspace(-1.5, -.9, 40)
        image['rear']['points_b'] = np.c_[xs, np.zeros_like(xs), np.full_like(xs, -.5)].tolist()
        output = controller.update(image, 10.04, .02)
        status = output['status']
        self.assertTrue(status['valid'], status['reason'])
        self.assertTrue(status['measurement_valid'])
        self.assertFalse(status['rear_target_directly_observed'])
        self.assertTrue(status['target_estimated'])
        self.assertTrue(status['rear_target_estimated'])
        self.assertEqual(status['rear_target_source'], 'rear_rgbd_near_field_tangent_estimate')
        self.assertAlmostEqual(status['rear_target_near_field_extension_m'], .10)
        self.assertEqual(status['rear_target_max_near_field_extension_m'], .25)
        self.assertAlmostEqual(status['rear_target_source_age_s'], .04)
        self.assertEqual(status['rear_target_source_capture_timestamp'], 10.)
        self.assertEqual(status['rear_target_source_sequence'], 0)
        self.assertEqual(status['rear_target_source_capture_generation'], 4)
        self.assertEqual(status['rear_target_occlusion_extension_m'], 0.)
        # Regression: the inherited helper emits np.bool_ for an estimated
        # work point, which previously broke strict JSON actor-log output.
        json.dumps(output, allow_nan=False)
        repeat = controller.update(image, 10.08, .02)
        self.assertFalse(repeat['status']['measurement_is_new'])
        self.assertAlmostEqual(repeat['status']['rear_target_source_age_s'], .08)
        self.assertFalse(repeat['status']['rear_target_directly_observed'])

    def test_near_field_bound_and_strict_direct_only_mode(self):
        for near_endpoint, bound in ((-1.06, .25), (-.91, .10), (-.9, 0.)):
            controller = self.controller(max_near_field_extension_m=bound)
            image = packet()
            xs = np.linspace(-1.5, near_endpoint, 40)
            image['rear']['points_b'] = np.c_[xs, np.zeros_like(xs), np.full_like(xs, -.5)].tolist()
            output = controller.update(image, 10., .02)
            self.assertFalse(output['status']['valid'])
            self.assertFalse(output['status']['rear_target_directly_observed'])
            np.testing.assert_array_equal(output['base_command_b'], [0., 0., 0.])
        direct = self.controller(max_near_field_extension_m=0.)
        output = direct.update(packet(), 10., .02)
        self.assertTrue(output['status']['valid'])
        self.assertTrue(output['status']['rear_target_directly_observed'])
        self.assertFalse(output['status']['target_estimated'])

    def test_far_end_target_is_not_extrapolated(self):
        for xs in (np.linspace(-.77, -.2, 40), np.linspace(-.5, -.2, 40)):
            controller = self.controller()
            image = packet()
            image['rear']['points_b'] = np.c_[xs, np.zeros_like(xs), np.full_like(xs, -.5)].tolist()
            output = controller.update(image, 10., .02)
            self.assertFalse(output['status']['valid'])
            self.assertFalse(output['status']['rear_target_directly_observed'])
            np.testing.assert_array_equal(output['base_command_b'], [0., 0., 0.])

    def test_explicit_far_end_fit_uses_old_support_bounds_and_honest_labels(self):
        controller = self.controller(max_occlusion_extension=.05)
        image = packet()
        xs = np.linspace(-.77, -.62, 40)
        image['rear']['points_b'] = np.c_[xs, np.zeros_like(xs), np.full_like(xs, -.5)].tolist()
        output = controller.update(image, 10.02, .02)
        status = output['status']
        self.assertTrue(status['valid'], status['reason'])
        self.assertTrue(status['target_estimated'])
        self.assertFalse(status['rear_target_directly_observed'])
        self.assertEqual(status['rear_target_source'], 'rear_rgbd_local_occlusion_estimate')
        self.assertAlmostEqual(status['rear_target_occlusion_extension_m'], .03)
        self.assertEqual(status['rear_target_max_occlusion_extension_m'], .05)
        self.assertGreaterEqual(status['rear_occlusion_fit_points'], 8)
        self.assertGreaterEqual(status['rear_occlusion_fit_span_m'], .12)
        self.assertAlmostEqual(status['rear_target_source_age_s'], .02)
        self.assertEqual(status['rear_target_source_sequences'], [0])
        for xs in (np.linspace(-.74, -.60, 40), np.linspace(-.77, -.69, 24)):
            controller = self.controller(max_occlusion_extension=.05)
            short = packet()
            short['rear']['points_b'] = np.c_[xs, np.zeros_like(xs), np.full_like(xs, -.5)].tolist()
            self.assertFalse(controller.update(short, 10., .02)['status']['valid'])

    def memory_fixture(self):
        controller = self.controller(rear_path_memory_s=.6, max_occlusion_extension=.05)
        for tick in range(101):
            image = packet(10. + tick * .02, tick)
            xs = np.linspace(-1.0, -.60, 60)
            image['rear']['points_b'] = np.c_[xs, np.zeros_like(xs), np.full_like(xs, -.5)].tolist()
            output = controller.update(image, 10. + tick * .02, .02)
            self.assertTrue(output['status']['valid'], output['status']['reason'])
        missing = packet(12.02, 101)
        missing.update(valid=False, reason='rear: geometric seam path is too short')
        missing['rear'].update(valid=False, reason='geometric seam path is too short', points_b=[])
        return controller, missing

    def test_memory_is_optional_and_preserves_original_capture_provenance(self):
        self.assertIsNone(self.controller()._path_memory)
        controller, missing = self.memory_fixture()
        output = controller.update(missing, 12.02, .02)
        status = output['status']
        self.assertTrue(status['valid'], status['reason'])
        self.assertFalse(status['measurement_valid'])
        self.assertTrue(status['measurement_is_new'])
        self.assertEqual(status['control_state'], 'rear_memory')
        self.assertTrue(status['rear_memory_used'])
        self.assertTrue(status['target_estimated'])
        self.assertFalse(status['rear_target_directly_observed'])
        self.assertFalse(status['rear_target_held'])
        self.assertEqual(status['rear_target_source'], 'rear_rgbd_observed_path_command_propagation')
        self.assertEqual(status['rear_memory_motion_source'], 'sent_base_commands_only')
        self.assertEqual(status['rear_target_source_sequences'], status['rear_memory_source_sequences'])
        self.assertEqual(status['source_sequence'], 101)  # fresh packet, not historical target
        self.assertNotIn(101, status['rear_target_source_sequences'])
        self.assertEqual(status['rear_target_source_capture_timestamp'],
                         status['rear_memory_source_capture_timestamp_min'])
        self.assertAlmostEqual(status['rear_target_source_age_s'],
                               12.02 - status['rear_target_source_capture_timestamp'])
        self.assertAlmostEqual(status['capture_age_s'], 0.)
        self.assertGreater(status['rear_target_source_age_s'], status['capture_age_s'])
        self.assertLessEqual(status['rear_target_source_age_s'], .6 + 1.e-12)
        self.assertGreater(status['rear_memory_command_displacement_max_m'], 0.)
        json.dumps(output, allow_nan=False)

    def test_memory_is_preferred_to_far_fit_when_current_images_do_not_cover_workpoint(self):
        controller, _ = self.memory_fixture()
        image = packet(12.02, 101)
        xs = np.linspace(-.77, -.62, 40)
        image['rear']['points_b'] = np.c_[xs, np.zeros_like(xs), np.full_like(xs, -.5)].tolist()
        output = controller.update(image, 12.02, .02)
        self.assertTrue(output['status']['valid'], output['status']['reason'])
        self.assertTrue(output['status']['rear_memory_used'])
        self.assertFalse(output['status']['measurement_valid'])
        self.assertEqual(output['status']['rear_target_occlusion_extension_m'], 0.)

    def test_historical_tangent_is_opt_in_bounded_and_not_a_new_measurement(self):
        xs = np.linspace(-1.3, -.9, 80)
        points = np.c_[xs, np.zeros_like(xs), np.full_like(xs, -.5)]
        self.assertIsNone(self.controller()._memory_visual_target(points, [0., 0., 1., .5]))
        controller = self.controller(rear_path_memory_s=.6, historical_tangent_estimate=True)
        for tick in range(101):
            image = packet(10.+tick*.02, tick)
            image['rear']['points_b'] = points.tolist()
            output = controller.update(image, 10.+tick*.02, .02)
            self.assertTrue(output['status']['valid'], output['status']['reason'])
        missing = packet(12.02, 101)
        missing.update(valid=False, reason='rear: geometric seam path is too short')
        missing['rear'].update(valid=False, reason='geometric seam path is too short', points_b=[])
        output = controller.update(missing, 12.02, .02)
        status = output['status']
        self.assertTrue(status['valid'], status['reason'])
        self.assertTrue(status['rear_memory_tangent_estimated'])
        self.assertTrue(status['target_estimated'])
        self.assertFalse(status['measurement_valid'])
        self.assertFalse(status['rear_target_directly_observed'])
        self.assertFalse(status['rear_target_held'])
        self.assertEqual(status['rear_target_source'], 'rear_rgbd_historical_near_field_tangent_estimate')
        self.assertGreater(status['rear_target_near_field_extension_m'], .10)
        self.assertLessEqual(status['rear_target_near_field_extension_m'], .25)
        self.assertGreaterEqual(status['rear_memory_tangent_fit_span_m'], .12)
        self.assertGreaterEqual(status['rear_memory_tangent_fit_points'], 8)
        self.assertLessEqual(status['rear_memory_tangent_fit_residual_p90_m'], .008)
        self.assertNotIn(101, status['rear_target_source_sequences'])
        self.assertAlmostEqual(status['rear_target_source_age_s'],
                               12.02-status['rear_target_source_capture_timestamp'])
        json.dumps(output, allow_nan=False)
        # Fresh missing frames cannot renew a historical observation.
        for tick in range(102, 136):
            now = 10.+tick*.02
            for name in ('front', 'rear'):
                missing[name].update(capture_timestamp=now, sequence=tick)
            output = controller.update(missing, now, .02)
            if not output['status']['valid']:
                break
        self.assertFalse(output['status']['valid'])
        self.assertTrue(output['status']['failure_latched'])

    def test_historical_tangent_needs_original_length_count_and_residual_bounds(self):
        controller = self.controller(historical_tangent_estimate=True)
        for count, span, noise in ((7, .4, 0.), (40, .10, 0.), (40, .4, .03)):
            xs = np.linspace(-.9-span, -.9, count)
            ys = noise*np.where(np.arange(count) % 2, -1., 1.)
            self.assertIsNone(controller._memory_visual_target(
                np.c_[xs, ys, np.full_like(xs, -.5)], [0., 0., 1., .5]))
        xs = np.linspace(-1.5, -1.06, 60)
        with self.assertRaisesRegex(ValueError, '0.25 m'):
            controller._memory_visual_target(np.c_[xs, np.zeros_like(xs), np.full_like(xs, -.5)],
                                             [0., 0., 1., .5])

    def test_historical_tangent_target_uses_validated_fit_not_raw_endpoint_hook(self):
        controller = self.controller(historical_tangent_estimate=True)
        xs = np.linspace(-1.3, -.9, 80)
        ys = np.zeros_like(xs)
        ys[-1] = .014
        points = np.c_[xs, ys, np.full_like(xs, -.5)]
        target, diagnostics = controller._memory_visual_target(points, [0., 0., 1., .5])
        endpoint = np.asarray(diagnostics['rear_memory_tangent_fitted_endpoint_b'])
        direction = np.asarray(diagnostics['rear_tangent_b'])
        expected = endpoint + diagnostics['rear_memory_tangent_fit_extension_m']*direction
        np.testing.assert_allclose(target, expected, atol=1.e-12)
        np.testing.assert_allclose(target, diagnostics['rear_target_b'], atol=1.e-12)
        # The old raw-endpoint anchor produced y=16.28mm despite the accepted
        # fit's P90 residual being only 2.11mm. Do not preserve that hook.
        self.assertLess(abs(target[1]), .008)
        self.assertAlmostEqual(diagnostics['rear_target_near_field_extension_m'],
                               float(np.linalg.norm(target-points[-1])))

    def test_optional_cache_far_tail_cannot_veto_valid_direct_workpoint(self):
        controller = self.controller(rear_path_memory_s=.6)
        image = packet()
        xs = np.linspace(-1.5, -.60, 80)
        ys = np.where(xs < -1.2, .40, .20)
        points = np.c_[xs, ys, np.full_like(xs, -.5)]
        image['rear']['points_b'] = points.tolist()
        original = copy.deepcopy(image)
        output = controller.update(image, 10., .02)
        status = output['status']
        self.assertTrue(status['valid'], status['reason'])
        self.assertTrue(status['rear_target_directly_observed'])
        self.assertFalse(status['rear_memory_used'])
        self.assertEqual(image, original)  # current geometry is never cropped
        self.assertEqual(len(controller._path_memory.frames), 1)
        cached = controller._path_memory.frames[0]['points']
        self.assertTrue(np.all(np.abs(cached[:, 1]) <= .35))
        self.assertEqual(len(cached), int(np.count_nonzero(ys <= .35)))
        self.assertTrue(np.all(np.isin(cached[:, 0], xs[ys <= .35])))

    def test_unsupported_cache_frame_does_not_get_an_identity_or_support(self):
        memory = _ObservedEnvelopeRearPathMemory(.6)
        memory.advance(10., .02, [0., 0., 0.])
        plane = np.array([0., 0., 1., .5])
        for xs, ys in ((np.linspace(-1., -.6, 40), np.full(40, .40)),
                       (np.linspace(-.8, -.72, 40), np.zeros(40)),
                       (np.linspace(-1., -.6, 5), np.zeros(5))):
            memory.ingest(np.c_[xs, ys, np.full_like(xs, -.5)], plane, np.eye(3),
                          4, 9, 10., 10.)
            self.assertIsNone(memory.identity)
            self.assertEqual(len(memory.frames), 0)
            self.assertEqual(memory.paths(plane, np.eye(3), 10.), [])

    def test_clipped_cache_gap_still_uses_original_gap_split_not_interpolation(self):
        memory = _ObservedEnvelopeRearPathMemory(.6)
        memory.advance(10., .02, [0., 0., 0.])
        plane = np.array([0., 0., 1., .5])
        xs = np.linspace(-1.2, -.6, 121)
        ys = np.where((xs > -1.0) & (xs < -.8), .40, 0.)
        memory.ingest(np.c_[xs, ys, np.full_like(xs, -.5)], plane, np.eye(3),
                      4, 9, 10., 10.)
        paths = memory.paths(plane, np.eye(3), 10.)
        self.assertEqual(len(paths), 2)
        for points, _ in paths:
            self.assertFalse(np.any((points[:, 0] > -.99) & (points[:, 0] < -.81)))
            self.assertLessEqual(np.max(np.linalg.norm(np.diff(points, axis=0), axis=1)), .025)

    def test_parallel_visual_offsets_match_training_lateral_heading_correction(self):
        for offset in (-.20, -.10, 0., .10, .20):
            with self.subTest(offset=offset):
                controller = self.controller()
                self.advance(controller)
                image = packet(12.02, 101)
                points = np.asarray(image['front']['points_b'])
                points[:, 1] = offset
                image['front']['points_b'] = points.tolist()
                lateral, heading, curvature, valid = _tracking_errors(points[:, :2],
                    np.ones(len(points), dtype=bool), .6)
                self.assertTrue(valid)
                expected = float(np.clip(1.5 * heading + .2 * curvature, -.8, .8))
                output = controller.update(image, 12.02, .02)
                status = output['status']
                self.assertTrue(status['valid'], status['reason'])
                self.assertAlmostEqual(status['lateral_error_m'], lateral, places=10)
                self.assertAlmostEqual(status['tangent_heading_rad'], 0., places=10)
                self.assertAlmostEqual(status['heading_error_rad'], heading, places=10)
                self.assertAlmostEqual(output['base_command_b'][2], expected, places=10)
                self.assertAlmostEqual(status['yaw_rate_rps'], expected, places=10)
                if offset:
                    self.assertEqual(math.copysign(1., output['base_command_b'][2]), math.copysign(1., offset))
                else:
                    self.assertAlmostEqual(output['base_command_b'][2], 0., places=10)

    def test_front_candidate_with_short_support_does_not_veto_other_valid_fits(self):
        # Actual RHANet frame 89 from the rejected 20261114_190436 run,
        # expressed in its measured ground plane. The best-median candidate
        # spans 0.296 m, but the next candidate has valid 0.912 m support.
        xy = np.array([
            [1.153312334, .115063969], [1.153784084, .115156001],
            [1.171652109, .118641726], [1.190435712, .122306153],
            [1.210207122, .126163289], [1.231046364, .130228744],
            [1.253042349, .134519864], [1.276294137, .139055975],
            [1.300912435, .143858673], [1.327021353, .148952171],
            [1.354760504, .154363705], [1.384287494, .160124023],
            [1.415780922, .166267966], [1.449391278, .171027704],
            [1.485381624, .175684789], [1.524104477, .183085172],
            [1.565802012, .191054050], [1.610830690, .199659548],
            [1.659606300, .208981128], [1.712616830, .219112050],
            [1.770438852, .230162503], [1.833758580, .242263631],
            [1.903399231, .255572760], [1.980357012, .270280277],
            [2.065849080, .286618794],
        ])
        image = packet()
        image['front']['points_b'] = np.c_[xy, np.full(len(xy), -.5)].tolist()
        original = copy.deepcopy(image)
        with self.assertRaisesRegex(ValueError, 'robust inlier support'):
            front_visual_command(image['front']['points_b'], [0., 0., 1., .5], .2)
        output = self.controller().update(image, 10., .02)
        status = output['status']
        self.assertTrue(status['valid'], status['reason'])
        self.assertTrue(status['front_fit_candidate_fallback'])
        self.assertGreater(status['front_fit_candidate_rank'], 1)
        self.assertGreaterEqual(status['front_fit_inlier_span_m'], .30)
        self.assertGreaterEqual(status['front_fit_inlier_count'], 13)
        self.assertAlmostEqual(status['front_fit_residual_tolerance_m'], .0025)
        self.assertEqual(image, original)

    def test_front_candidate_fallback_keeps_bounds_and_legacy_success(self):
        for offset in (-.1, 0., .1):
            points = np.c_[np.linspace(.2, 1.1, 40), np.full(40, offset), np.full(40, -.5)]
            old = front_visual_command(points, [0., 0., 1., .5], .2)
            new = front_visual_command(points, [0., 0., 1., .5], .2,
                                       geometry_fn=supported_front_geometry)
            np.testing.assert_array_equal(old[0], new[0])
            self.assertEqual(old[1], new[1])
        # Enough raw span, but the only robustly supported part spans 0.28m:
        # ranking alternatives must not relax the 0.30m or residual gates.
        x = np.r_[np.linspace(.2, .48, 20), np.linspace(.6, 1.1, 5)]
        y = np.r_[np.zeros(20), [.0507979026, .2081478226, -.2090952611,
                               .0108276052, .3929469543]]
        with self.assertRaisesRegex(ValueError, 'robust inlier support'):
            supported_front_geometry(np.c_[x, y, np.zeros_like(x)])

    def test_memory_cannot_override_front_pair_depth_or_capture_failures(self):
        for failure in ('front', 'pair', 'depth', 'generation', 'stale'):
            with self.subTest(failure=failure):
                controller, missing = self.memory_fixture()
                if failure == 'front': missing['front']['valid'] = False
                if failure == 'pair': missing['rear']['sequence'] += 1
                if failure == 'depth': missing['rear']['reason'] = 'rear seam depth support too low (0.0)'
                if failure == 'generation':
                    missing['capture_generation'] = 5
                    for name in ('front', 'rear'): missing[name]['capture_generation'] = 5
                if failure == 'stale': missing['rear']['capture_timestamp'] = 10.
                status = controller.update(missing, 12.02, .02)['status']
                self.assertFalse(status['valid'])
                self.assertTrue(status['failure_latched'])

    def test_memory_expires_by_original_capture_not_fresh_missing_packets(self):
        controller, missing = self.memory_fixture()
        last_status = None
        for tick in range(101, 136):
            now = 10. + tick * .02
            for name in ('front', 'rear'):
                missing[name].update(sequence=tick, capture_timestamp=now)
            last_status = controller.update(missing, now, .02)['status']
            if not last_status['valid']:
                break
        self.assertFalse(last_status['valid'])
        self.assertTrue(last_status['failure_latched'])
        self.assertFalse(last_status['measurement_valid'])

    def test_no_truth_fallback_from_extra_packet_keys(self):
        controller = self.controller()
        image = packet()
        image['rear']['points_b'] = []
        image.update(world_pose=[0., 0., .5], seam_points_w=[[0., 0., 0.]] * 10,
                     target_world=[-.8, 0., 0.], simulator_truth_valid=True)
        output = controller.update(image, 10., .02)
        self.assertFalse(output['status']['valid'])
        self.assertFalse(output['status']['uses_world_pose'])
        self.assertFalse(output['status']['uses_simulator_seam'])

    def test_bad_initialization_is_rejected(self):
        for kwargs in (dict(speed=math.nan), dict(trailing_distance=-.8),
                       dict(stale_timeout_s=math.inf), dict(ramp_s=.3),
                       dict(tube_length_m=0.), dict(tube_outer_radius_m=-.028),
                       dict(working_clearance_m=-.005),
                       dict(max_near_field_extension_m=-.01),
                       dict(max_near_field_extension_m=.251),
                       dict(max_near_field_extension_m=math.nan),
                       dict(rear_path_memory_s=-.01), dict(rear_path_memory_s=.81),
                       dict(rear_path_memory_s=math.nan),
                       dict(max_occlusion_extension=-.01), dict(max_occlusion_extension=.051)):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.controller(**kwargs)


if __name__ == '__main__':
    unittest.main()
