"""CPU-only behavioral checks of VD1's visual controller and actor boundary."""

import copy
import inspect
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from vd1_control import SCHEMA, VD1VisualController, build_actor_observation


def packet(now=10., lateral=0., slope=0., tcp_y=0., generation=3, sequence=8):
    forward = np.linspace(.3, 2., 40)
    rearward = np.linspace(-1.3, -.8, 30)
    capture = dict(sequence=sequence, capture_generation=generation, capture_timestamp=now,
                   valid=True)
    return dict(schema=SCHEMA, valid=True, capture_generation=generation,
                front=dict(capture, points_b=np.column_stack((forward, lateral + slope*forward,
                                                             np.full_like(forward, -.50))).tolist()),
                rear=dict(capture, points_b=np.column_stack((rearward, np.zeros_like(rearward),
                                                            np.full_like(rearward, -.50))).tolist(),
                          ground_plane_b=[0., 0., 1., .5], tcp_measured_b=[-1.02, tcp_y, -.30]))


def settled(controller, observation, iterations=100):
    result = None
    for _ in range(iterations):
        result = controller.update(observation, now=10., dt=.02)
    return result


class VisualControllerTests(unittest.TestCase):
    def controller(self, **kwargs):
        return VD1VisualController([-1.02, 0., -.30], **kwargs)

    def test_no_visual_measurement_stands_and_retains_initial_encoder_target(self):
        controller = self.controller()
        result = controller.update(None, 10., .02)
        self.assertFalse(result["status"]["valid"])
        np.testing.assert_array_equal(result["base_command_b"], [0., 0., 0.])
        np.testing.assert_allclose(result["tool_pose_b"][:3], [-1.02, 0., -.30])

    def test_front_left_and_right_seams_steer_in_the_correct_direction(self):
        left = settled(self.controller(ramp_s=0), packet(lateral=.10))
        right = settled(self.controller(ramp_s=0), packet(lateral=-.10))
        self.assertGreater(left["base_command_b"][2], 0)
        self.assertLess(right["base_command_b"][2], 0)
        self.assertAlmostEqual(left["base_command_b"][0], .2, places=4)
        self.assertEqual(left["base_command_b"][1], 0)
        self.assertLess(left["status"]["robot_lateral_error_m"], 0)

    def test_front_heading_and_curvature_are_image_derived(self):
        data = packet()
        points = np.asarray(data["front"]["points_b"])
        points[:, 1] = .05 * points[:, 0]**2
        data["front"]["points_b"] = points.tolist()
        result = settled(self.controller(ramp_s=0), data)
        self.assertGreater(result["status"]["front_curvature_m_inv"], .08)
        self.assertGreater(result["status"]["front_heading_error_rad"], 0)

    def test_visible_left_and_right_arcs_pursue_correctly_without_double_feedforward(self):
        for curvature in (-.10, -.05, .05, .10):
            with self.subTest(curvature=curvature):
                data = packet()
                x = np.linspace(1.8, 3., 40)
                # Independently construct a circle through the base origin,
                # with a camera-visible section beginning 1.8 m in front.
                y = (1 - np.sqrt(1 - (curvature*x)**2)) / curvature
                data["front"]["points_b"] = np.column_stack((x, y, np.full_like(x, -.5))).tolist()
                result = self.controller(ramp_s=0).update(data, 10., .02)
                status = result["status"]
                self.assertTrue(status["valid"], status)
                self.assertGreater(status["requested_yaw_rate_rps"] * curvature, 0)
                self.assertAlmostEqual(status["requested_yaw_rate_rps"], .2*curvature, delta=.001)
                self.assertGreaterEqual(status["front_lookahead_b"][0], x.min())
                self.assertLessEqual(status["front_lookahead_b"][0], x.max())
                self.assertEqual(status["front_diagnostics_reference"], "visible_tangent_not_base_pose")

    def test_real_straight_front_frame_endpoint_hook_cannot_dominate_steering(self):
        # Image-derived xy from session_20261102_153842_qqvy16jd.
        # The first 0.482 mm segment bends 0.397 mm sideways; the remaining
        # visible line spans >1 m.  No scene path or robot pose is a fixture.
        xy = np.array([
            [1.75990039, -.01786303], [1.76038248, -.01746635],
            [1.78101489, -.00290948], [1.81547867, .00309182],
            [1.85510736, .00317782], [1.89700372, .00326874],
            [1.94136814, .00336502], [1.98842530, .00346714],
            [2.03842805, .00357565], [2.09166189, .00369118],
            [2.14845045, .00381442], [2.20916210, .00394618],
            [2.27421789, .00408736], [2.34410140, .00423902],
            [2.41937071, .00440236], [2.50067339, .00457880],
            [2.58876515, .00476998], [2.68453342, .00497781],
            [2.78902733, .00520458], [2.90349631, .00545316],
        ])
        data = packet()
        data["front"]["points_b"] = np.column_stack((xy, np.full(len(xy), -.701))).tolist()
        result = self.controller(ramp_s=0).update(data, 10., .02)
        self.assertTrue(result["status"]["valid"], result["status"])
        self.assertLess(abs(result["status"]["front_heading_error_rad"]), .01)
        self.assertLess(abs(result["status"]["robot_lateral_error_m"]), .01)
        self.assertLess(abs(result["status"]["front_curvature_m_inv"]), .005)
        self.assertLess(abs(result["status"]["requested_yaw_rate_rps"]), .02)

    def test_short_front_support_is_rejected_instead_of_extrapolated_from_tiny_segments(self):
        data = packet()
        points = np.asarray(data["front"]["points_b"])
        points[:, 0] = np.linspace(1.7, 1.8, len(points))
        data["front"]["points_b"] = points.tolist()
        result = self.controller().update(data, 10., .02)
        self.assertFalse(result["status"]["valid"])
        self.assertIn("0.30", result["status"]["reason"])

    def test_observed_tcp_moves_rear_target_in_opposite_direction(self):
        left_tcp = settled(self.controller(), packet(tcp_y=.04))
        right_tcp = settled(self.controller(), packet(tcp_y=-.04))
        self.assertLess(left_tcp["tool_pose_b"][1], -.01)
        self.assertGreater(right_tcp["tool_pose_b"][1], .01)
        self.assertAlmostEqual(left_tcp["status"]["tcp_projection_error_m"], .04)
        self.assertAlmostEqual(left_tcp["tool_pose_b"][0], -1.02)
        self.assertAlmostEqual(left_tcp["tool_pose_b"][2], -.30)

    def test_rear_target_uses_visible_seam_and_bounded_feedback(self):
        data = packet(tcp_y=1.)
        points = np.asarray(data["rear"]["points_b"])
        points[:, 1] = .2
        data["rear"]["points_b"] = points.tolist()
        result = settled(self.controller(), data)
        self.assertTrue(result["status"]["target_clipped"])
        self.assertLessEqual(result["tool_pose_b"][1], .15)
        self.assertLessEqual(np.linalg.norm(result["status"]["rear_feedback_b"]), .030000001)
        data["rear"]["points_b"] = (points + [3., 0., 0.]).tolist()
        rejected = self.controller().update(data, 10., .02)
        self.assertFalse(rejected["status"]["valid"])
        self.assertIn("does not cover", rejected["status"]["reason"])

    def test_stale_capture_stops_smoothly_without_changing_target_or_timestamp(self):
        controller = self.controller(ramp_s=0)
        data = packet(lateral=.1, tcp_y=.04)
        active = settled(controller, data)
        previous_target = active["tool_pose_b"]
        stale = controller.update(data, 11.01, .02)
        self.assertFalse(stale["status"]["valid"])
        self.assertLess(stale["base_command_b"][0], active["base_command_b"][0])
        self.assertGreater(stale["base_command_b"][0], 0)
        self.assertGreater(stale["status"]["capture_age_s"], 1)
        self.assertEqual(data["front"]["capture_timestamp"], 10.)
        np.testing.assert_array_equal(stale["tool_pose_b"], previous_target)
        for _ in range(180):
            stale = controller.update(data, 11.01, .02)
        np.testing.assert_array_equal(stale["base_command_b"], [0., 0., 0.])

    def test_same_sequence_cannot_rejuvenate_capture_clock(self):
        controller = self.controller()
        controller.update(packet(), 10., .02)
        refreshed = packet(now=10.5)
        result = controller.update(refreshed, 10.5, .02)
        self.assertFalse(result["status"]["valid"])
        self.assertIn("refresh", result["status"]["reason"])
        self.assertAlmostEqual(result["status"]["capture_age_s"], .5)

    def test_pair_generation_mismatch_and_replayed_generation_are_rejected(self):
        controller = self.controller()
        controller.update(packet(), 10., .02)
        mismatched = packet()
        mismatched["rear"]["capture_generation"] = 4
        self.assertFalse(controller.update(mismatched, 10., .02)["status"]["valid"])
        self.assertFalse(controller.update(packet(generation=2), 10., .02)["status"]["valid"])

    def test_privileged_fields_are_rejected_at_every_packet_level(self):
        for key in ("base_position_w", "base_quaternion_w", "true_tcp", "body_state_w",
                    "evaluation_only_tcp_ground_truth_b", "path_world", "root_pos_w",
                    "capture_base_quaternion_wxyz", "base_quaternion_wxyz", "actual_tcp_b"):
            for location in ("top", "front", "rear", "nested"):
                with self.subTest(key=key, location=location):
                    data = packet()
                    if location == "nested":
                        data["rear"]["metadata"] = {key: [1, 2, 3]}
                    else:
                        container = data if location == "top" else data[location]
                        container[key] = [1, 2, 3]
                    result = self.controller().update(data, 10., .02)
                    self.assertFalse(result["status"]["valid"])
                    self.assertIn("privileged", result["status"]["reason"])

    def test_legal_inputs_are_invariant_to_external_truth_and_not_modified(self):
        observation = packet(lateral=.04, tcp_y=.03)
        original = copy.deepcopy(observation)
        first, second = self.controller(), self.controller()
        # External state is deliberately absent from the API and controller.
        external_truth = np.zeros((30, 3))
        for step in range(30):
            external_truth[:] = (step + 1) * 12345.
            a = first.update(observation, 10., .02)
            b = second.update(copy.deepcopy(observation), 10., .02)
            self.assertEqual(a, b)
        self.assertEqual(observation, original)
        self.assertNotIn("robot", inspect.signature(first.update).parameters)
        self.assertFalse(a["status"]["uses_world_pose"])

    def test_nonfinite_geometry_and_future_frames_are_rejected(self):
        for change in ("points", "tcp", "plane", "future"):
            with self.subTest(change=change):
                data = packet()
                if change == "points":
                    data["front"]["points_b"][0][0] = float("nan")
                elif change == "tcp":
                    data["rear"]["tcp_measured_b"][1] = float("inf")
                elif change == "plane":
                    data["rear"]["ground_plane_b"] = [0, 0, 0, .5]
                else:
                    data["front"]["capture_timestamp"] = 10.1
                self.assertFalse(self.controller().update(data, 10., .02)["status"]["valid"])

    def test_target_transition_is_continuous_from_encoder_fk(self):
        controller = VD1VisualController([-.8, .1, .05])
        before = np.array([-.8, .1, .05])
        result = controller.update(packet(), 10., .02)
        self.assertLessEqual(np.linalg.norm(np.asarray(result["tool_pose_b"][:3]) - before), .003000001)
        self.assertTrue(result["status"]["target_transition_outside_workspace"])

    def test_measured_tilted_plane_changes_projection_without_changing_nominal_height(self):
        data = packet(tcp_y=.04)
        # A tilted measured ground normal; no known world elevation exists.
        data["rear"]["ground_plane_b"] = [0., .2, 1., .5]
        points = np.asarray(data["rear"]["points_b"])
        points[:, 2] = -.5 - .2*points[:, 1]
        data["rear"]["points_b"] = points.tolist()
        result = settled(self.controller(), data)
        self.assertTrue(result["status"]["valid"])
        self.assertAlmostEqual(result["status"]["tcp_projection_error_m"], 0., delta=1.e-9)
        self.assertAlmostEqual(result["tool_pose_b"][2], -.30)

    def test_new_generation_resets_sequence_and_startup_ramp(self):
        controller = self.controller()
        settled(controller, packet())
        result = controller.update(packet(now=10.1, generation=4, sequence=0), 10.1, .02)
        self.assertTrue(result["status"]["valid"])
        self.assertEqual(result["status"]["source_sequence"], 0)
        self.assertLess(result["status"]["startup_blend"], .001)


class ActorObservationTests(unittest.TestCase):
    def build(self, **overrides):
        tool = VD1VisualController([-1.02, 0., -.3]).update(None, 10., .02)["tool_pose_b"]
        args = dict(base_command_b=[.2, 0, .3], tool_pose_b=tool, omega_b=[1., 2., 3.],
                    gravity_b=[0., 0., -1.], q=np.arange(18) / 10,
                    qd=np.arange(18), default_q=np.ones(18), last_action=np.arange(18) / 20)
        args.update(overrides)
        return build_actor_observation(**args)

    def test_exact_upstream_70_dimension_order_and_scales(self):
        result = self.build()
        self.assertEqual(result.shape, (70,))
        self.assertEqual(result.dtype, np.float32)
        np.testing.assert_allclose(result[:3], [.2, 0, .3])
        np.testing.assert_allclose(result[10:13], [.2, .4, .6])
        np.testing.assert_allclose(result[13:16], [0, 0, -1])
        np.testing.assert_allclose(result[16:34], np.arange(18)/10 - 1, atol=1.e-7)
        np.testing.assert_allclose(result[34:52], np.arange(18)*.05)
        np.testing.assert_allclose(result[52:], np.arange(18)/20)

    def test_clip_happens_before_angular_and_encoder_speed_scaling(self):
        result = self.build(omega_b=[1000, -1000, 0], qd=np.full(18, 1000.))
        np.testing.assert_allclose(result[10:13], [20, -20, 0])
        np.testing.assert_allclose(result[34:52], 5.)

    def test_wrong_encoder_dimensions_and_nonfinite_inputs_fail(self):
        for field, bad in (("q", np.zeros(19)), ("qd", np.zeros(17)),
                           ("omega_b", [0, float("nan"), 0])):
            with self.subTest(field=field), self.assertRaises(ValueError):
                self.build(**{field: bad})


if __name__ == "__main__":
    unittest.main(verbosity=2)
