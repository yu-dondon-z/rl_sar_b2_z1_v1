"""Startup handover uses camera evidence, never elapsed time alone."""
import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'python/vision'))
sys.path.insert(0, '/home/dong/b2_z1/scripts/vision')
from vd1_visual_guard import FrontStartupGuidance


def diagnostics():
    return dict(front_estimator_reset=False, guidance_fit_p95_m=.005,
        front_estimator_reliable=True,
        front_estimated_target_std_m=.03, front_visible_line_target_xy_m=[1.4, .05],
        front_visible_line_fit_p95_m=.005, front_visible_line_heading_rad=.05)


class StartupTests(unittest.TestCase):
    def test_single_capture_never_qualifies_even_when_repeated(self):
        controller = FrontStartupGuidance()
        d = diagnostics()
        for _ in range(250):
            yaw = controller.update(d, (1, 1, (10., 10.)), .4, .2, .02)
        self.assertEqual(d['front_estimator_blend'], 0.)
        self.assertLess(abs(yaw), .12)
        for _ in range(100):
            controller.update(d, (1, 1, (10., 10.)), .4, .2, .02)
        self.assertEqual(d['front_estimator_blend'], 0.)

    def test_unreliable_estimate_falls_back_after_complete_handover(self):
        controller = FrontStartupGuidance()
        d = diagnostics()
        for tick in range(220):
            seq = tick//15
            controller.update(d, (1, seq, (10.+seq*.3,)*2), .04, .2, .02)
        self.assertEqual(d['front_startup_phase'], 'estimator')
        d['front_estimator_reliable'] = False
        yaw = controller.update(d, (1, 15, (14.5,)*2), -.3, .2, .02)
        self.assertEqual(d['front_estimator_blend'], 0.)
        self.assertGreater(yaw, 0.)
        self.assertTrue(d['front_estimator_fallback'])
        d['front_estimator_reliable'] = True
        controller.update(d, (1, 16, (14.8,)*2), .04, .2, .02)
        self.assertEqual(d['front_estimator_blend'], 0.)

    def test_yaw_jump_after_handover_returns_to_visible_segment(self):
        controller = FrontStartupGuidance()
        d = diagnostics()
        for tick in range(220):
            seq = tick//15
            controller.update(d, (1, seq, (10.+seq*.3,)*2), .04, .2, .02)
        yaw = controller.update(d, (1, 15, (14.5,)*2), -.3, .2, .02)
        self.assertEqual(d['front_estimator_blend'], 0.)
        self.assertGreater(yaw, 0.)

    def test_stable_captures_blend_without_a_yaw_step(self):
        controller = FrontStartupGuidance()
        d, yaws = diagnostics(), []
        for tick in range(220):
            seq = tick//15
            yaws.append(controller.update(d, (1, seq, (10.+seq*.3,)*2), .4, .2, .02))
        self.assertEqual(d['front_startup_phase'], 'estimator')
        self.assertAlmostEqual(yaws[-1], .4)
        self.assertLess(max(abs(b-a) for a,b in zip(yaws, yaws[1:])), .01)

    def test_instability_cannot_be_promoted_by_waiting(self):
        controller = FrontStartupGuidance()
        d = diagnostics()
        for seq in range(25):
            controller.update(d, (1, seq, (10.+seq*.2,)*2), (-1)**seq*.2, .2, .2)
        self.assertEqual(d['front_estimator_blend'], 0.)

    def test_curved_or_misaligned_start_is_not_blind_straight_motion(self):
        for field, value in [('front_visible_line_heading_rad', .5),
                             ('front_visible_line_fit_p95_m', .04)]:
            d = diagnostics(); d[field] = value
            with self.assertRaisesRegex(ValueError, 'straight/aligned'):
                FrontStartupGuidance().update(d, (1, 1, (10., 10.)), .4, .2, .02)


if __name__ == '__main__':
    unittest.main()
