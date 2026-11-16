"""Missing rear work-point coverage pauses travel, not physics or source clocks."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'scripts'), '/home/dong/b2_z1/scripts/vision']
from deploy_vd12_vision import rear_support_wait_can_step
from vd12_visual_control import VD12VisualController
from vd14_motion_compensation import MotionCompensatedVisualController, SensorMotionHistory
from vd5_mujoco_control import quaternion_from_euler_xyz, quaternion_matrix
from test_vd12_vision_entry import packet
from test_vd14_motion_compensation import SyntheticLegs, proprio


class RearSupportWait(unittest.TestCase):
    def setup_controller(self, velocity=(0., 0., 0.)):
        self.motion = SensorMotionHistory(SyntheticLegs(velocity))
        self.motion.sample(0., proprio(), [100.]*4)
        self.controller = MotionCompensatedVisualController([-.95, 0., .35],
            quaternion_from_euler_xyz(0., .3, np.pi), motion_history=self.motion,
            trailing_distance=.95, max_occlusion_extension=.05, stale_timeout_s=.15)
        result = self.update(0., 0)
        self.assertTrue(result['status']['valid'], result['status']['reason'])
        return result

    def image(self, clock, sequence, missing=False):
        image = packet(stamp=10.+clock, sequence=sequence)
        for name in ('front', 'rear'):
            image[name]['capture_sim_time_s'] = clock
        if missing:
            xs = np.linspace(-.8964, -.65, 60)
            image['rear']['points_b'] = np.c_[xs, np.zeros_like(xs), np.full_like(xs, -.5)].tolist()
        return image

    def update(self, clock, sequence, missing=False, image=None):
        while self.motion.samples[-1]['time_s'] < clock-1.e-9:
            self.motion.sample(round(self.motion.samples[-1]['time_s']+.005, 9), proprio(), [100.]*4)
        return self.controller.update(self.image(clock, sequence, missing) if image is None else image,
            now=10.+clock, dt=.02, control_time_s=clock)

    def test_53_point_6mm_shortfall_waits_without_relaxing_50mm_bound(self):
        self.setup_controller()
        result = self.update(.02, 1, missing=True)
        status = result['status']
        self.assertFalse(status['valid'])
        self.assertEqual(status['control_state'], 'rear_support_wait')
        self.assertFalse(status['failure_latched'])
        self.assertFalse(status['runtime_must_terminate_on_invalid'])
        self.assertAlmostEqual(status['rear_support_required_extension_m'], .0536)
        self.assertEqual(status['rear_support_allowed_extension_m'], .05)
        self.assertFalse(status['measurement_valid'])
        self.assertFalse(status['measurement_is_new'])
        self.assertFalse(status['rear_target_directly_observed'])
        self.assertTrue(status['rear_target_held'])
        np.testing.assert_array_equal(result['base_command_b'], [0., 0., 0.])
        self.assertTrue(rear_support_wait_can_step(result, version=14, motion_compensation='leg_imu'))

    def test_hold_transports_the_command_with_sensor_motion_not_base_motion_commands(self):
        first = self.setup_controller((.1, .03, 0.))
        anchor = np.array(first['tool_pose_b'])
        result = self.update(.02, 1, missing=True)
        np.testing.assert_allclose(result['tool_pose_b'][:3], anchor[:3]-[.002, .0006, 0.], atol=1.e-12)
        np.testing.assert_allclose(quaternion_matrix(result['tool_pose_b'][3:]),
                                   quaternion_matrix(anchor[3:]), atol=1.e-12)
        self.assertGreater(np.dot(anchor[3:], result['tool_pose_b'][3:]), .999)

    def test_wait_can_persist_but_old_command_never_becomes_a_fresh_measurement(self):
        self.setup_controller()
        for tick in range(1, 111):
            result = self.update(tick*.02, tick, missing=True)
            self.assertEqual(result['status']['control_state'], 'rear_support_wait')
        status = result['status']
        self.assertGreater(status['held_setpoint_age_s'], 2.)
        self.assertEqual(status['held_setpoint_source']['sequences'], [0])
        self.assertEqual(status['held_setpoint_source']['capture_timestamp'], 10.)
        self.assertAlmostEqual(status['capture_age_s'], 0.)
        self.assertFalse(status['measurement_valid'])
        self.assertFalse(status['rear_memory_used'])

    def test_supported_continuous_target_resumes_without_reset_or_faster_startup(self):
        self.setup_controller()
        elapsed = self.controller._elapsed
        for tick in range(1, 11):
            self.update(tick*.02, tick, missing=True)
        self.assertEqual(self.controller._elapsed, elapsed)
        result = self.update(.22, 11)
        self.assertTrue(result['status']['valid'], result['status']['reason'])
        self.assertTrue(result['status']['resumed_from_rear_support_wait'])
        self.assertAlmostEqual(self.controller._elapsed, elapsed+.02)

    def test_loss_of_camera_freshness_is_still_latched_fatal(self):
        self.setup_controller()
        image = self.image(.02, 1, missing=True)
        self.update(.02, 1, image=image)
        result = self.update(.20, 1, image=image)
        self.assertFalse(result['status']['valid'])
        self.assertTrue(result['status']['failure_latched'])
        self.assertIn('stale', result['status']['reason'])
        self.assertFalse(rear_support_wait_can_step(result, version=14, motion_compensation='leg_imu'))
        self.assertFalse(self.update(.22, 2)['status']['valid'])

    def test_repeated_rejected_frame_cannot_change_geometry_or_timestamp(self):
        for kind in ('geometry', 'timestamp'):
            with self.subTest(kind=kind):
                self.setup_controller()
                image = self.image(.02, 1, missing=True)
                self.update(.02, 1, image=image)
                changed = deepcopy(image)
                if kind == 'geometry': changed['rear']['points_b'][0][1] = .001
                else:
                    for name in ('front', 'rear'): changed[name]['capture_timestamp'] += .001
                result = self.update(.04, 1, image=changed)
                self.assertFalse(result['status']['valid'])
                self.assertTrue(result['status']['failure_latched'])

    def test_shortfall_cannot_hide_changed_generation_or_mutated_accepted_frame(self):
        for kind in ('generation', 'accepted_geometry'):
            with self.subTest(kind=kind):
                self.setup_controller()
                if kind == 'generation':
                    image = self.image(.02, 1, missing=True)
                    image['capture_generation'] = 43
                    for name in ('front', 'rear'): image[name]['capture_generation'] = 43
                else:
                    image = self.image(0., 0, missing=True)
                result = self.update(.02, 1, image=image)
                self.assertFalse(result['status']['valid'])
                self.assertTrue(result['status']['failure_latched'])
                self.assertNotEqual(result['status']['control_state'], 'rear_support_wait')

    def test_recovery_does_not_increase_jump_allowance_with_wait_time(self):
        self.setup_controller()
        for tick in range(1, 11): self.update(tick*.02, tick, missing=True)
        image = self.image(.22, 11)
        for point in image['rear']['points_b']: point[1] = .2
        result = self.update(.22, 11, image=image)
        self.assertFalse(result['status']['valid'])
        self.assertIn('continuity bound', result['status']['reason'])
        self.assertTrue(result['status']['failure_latched'])

    def test_no_wait_before_any_supported_command_and_legacy_controller_still_stops(self):
        self.setup_controller()
        for cls, kwargs in ((VD12VisualController, {}),
                            (MotionCompensatedVisualController, dict(motion_history=self.motion))):
            controller = cls([-.95, 0., .35], quaternion_from_euler_xyz(0., .3, np.pi),
                trailing_distance=.95, max_occlusion_extension=.05, **kwargs)
            args = dict(control_time_s=0.) if kwargs else {}
            result = controller.update(self.image(0., 0, missing=True), 10., .02, **args)
            self.assertFalse(result['status']['valid'])
            self.assertTrue(result['status']['failure_latched'])
            self.assertFalse(rear_support_wait_can_step(result, version=14, motion_compensation='leg_imu'))

    def test_runtime_exception_is_narrow_and_rejects_nonzero_travel(self):
        self.setup_controller()
        result = self.update(.02, 1, missing=True)
        for version, mode in ((12, 'none'), (13, 'none'), (14, 'none')):
            self.assertFalse(rear_support_wait_can_step(result, version=version, motion_compensation=mode))
        changed = deepcopy(result); changed['base_command_b'][0] = .01
        self.assertFalse(rear_support_wait_can_step(changed, version=14, motion_compensation='leg_imu'))
        changed = deepcopy(result); changed['tool_pose_b'][0] = float('nan')
        self.assertFalse(rear_support_wait_can_step(changed, version=14, motion_compensation='leg_imu'))


if __name__ == '__main__': unittest.main()
