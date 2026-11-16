"""Camera-only state-machine regressions, including the recorded false path."""
import copy
import json
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
sys.path.insert(0, '/home/dong/b2_z1/scripts/vision')
from vd1_control import SCHEMA
from vd1_visual_guard import GuardedVD1VisualController


def packet(t=10., sequence=0, rear_y=0., front_y=0., rear_valid=True, front_valid=True, generation=1):
    front_x, rear_x = np.linspace(.4, 2., 20), np.linspace(-1.3, -.8, 20)
    shared = dict(capture_timestamp=t, sequence=sequence, capture_generation=generation)
    return dict(schema=SCHEMA, valid=front_valid and rear_valid, capture_generation=generation,
        front=dict(shared, valid=front_valid, reason='front test',
            points_b=np.column_stack((front_x, np.full_like(front_x, front_y), np.full_like(front_x, -.5))).tolist()),
        rear=dict(shared, valid=rear_valid, reason='rear test',
            ground_plane_b=[0., 0., 1., .5],
            points_b=np.column_stack((rear_x, np.full_like(rear_x, rear_y), np.full_like(rear_x, -.5))).tolist()))


class VisualGuardTests(unittest.TestCase):
    def controller(self, **kwargs):
        return GuardedVD1VisualController([-1.02, 0., -.30], ramp_s=0, **kwargs)

    def settled(self, controller, data=None):
        data = packet() if data is None else data
        for _ in range(100):
            result = controller.update(data, data['front']['capture_timestamp']+.1, .02)
        return result

    def test_no_images_do_not_start(self):
        result = self.controller().update(None, 10., .02)
        self.assertFalse(result['status']['valid'])
        self.assertEqual(result['base_command_b'], [0., 0., 0.])

    def test_markerless_target_is_the_seam_without_tcp_error_amplification(self):
        result = self.settled(self.controller(), packet(rear_y=.08))
        self.assertEqual(result['status']['state'], 'tracking')
        np.testing.assert_allclose(result['tool_pose_b'][:3], [-1.02, .08, -.30], atol=.0002)
        self.assertEqual(result['status']['tool_target_mode'], 'rear_seam_direct')
        self.assertFalse(result['status']['visual_tcp_feedback'])
        self.assertFalse(result['status']['requires_tcp_markers'])
        self.assertIsNone(result['status']['tcp_projection_error_m'])

    def test_tcp_measurement_is_ignored_even_if_malformed(self):
        data = packet(rear_y=.08)
        expected = self.settled(self.controller(), data)
        data['rear']['tcp_measured_b'] = 'not a measurement'
        actual = self.settled(self.controller(), data)
        np.testing.assert_array_equal(actual['tool_pose_b'], expected['tool_pose_b'])
        self.assertEqual(actual['status'], expected['status'])

    def test_missing_seam_or_ground_still_prevents_startup(self):
        for field in ('points_b', 'ground_plane_b'):
            data = packet()
            del data['rear'][field]
            result = self.controller().update(data, 10.1, .02)
            self.assertFalse(result['status']['valid'])
            self.assertEqual(result['base_command_b'], [0., 0., 0.])

    def test_recorded_sequence_252_is_rejected_before_workspace_clipping(self):
        recorded = json.loads((ROOT/'tests/fixtures/vd1_rear_discontinuity.json').read_text())['packets']
        controller = self.controller()
        before = self.settled(controller, recorded[0])
        # This older fixture already starts outside the actor workspace.
        # It must no longer start motion by silently clipping that target.
        self.assertFalse(before['status']['valid'])
        self.assertIn('outside actor workspace',before['status']['reason'])
        bad = recorded[1]
        result = controller.update(bad, bad['front']['capture_timestamp']+.15, .02)
        self.assertEqual(bad['rear']['sequence'], 252)
        self.assertFalse(result['status']['rear_valid'])
        self.assertIn('discontinuity', result['status']['rear_reason'])
        self.assertEqual(result['status']['state'], 'waiting')
        np.testing.assert_array_equal(result['tool_pose_b'], before['tool_pose_b'])
        self.assertAlmostEqual(result['base_command_b'][0], before['base_command_b'][0], places=5)
        after = recorded[2]
        recovered = controller.update(after, after['front']['capture_timestamp']+.15, .02)
        self.assertTrue(recovered['status']['rear_valid'])

    def test_short_rear_dropout_freezes_tool_keeps_fresh_front_steering(self):
        controller = self.controller()
        before = self.settled(controller)
        result = controller.update(packet(10.2, 1, rear_valid=False, front_y=.1), 10.3, .02)
        self.assertTrue(result['status']['valid'])
        self.assertEqual(result['status']['state'], 'rear_hold')
        self.assertGreater(result['base_command_b'][2], 0)
        self.assertAlmostEqual(result['base_command_b'][0], before['base_command_b'][0], places=5)
        np.testing.assert_array_equal(result['tool_pose_b'], before['tool_pose_b'])

    def test_short_dropout_does_not_restart_two_second_ramp(self):
        controller = GuardedVD1VisualController([-1.02, 0., -.30])
        for index in range(120):
            t=10.+index*.02
            active=controller.update(packet(t, index), t+.05, .02)
        lost=controller.update(packet(t+.02,120,rear_valid=False),t+.07,.02)
        recovered=controller.update(packet(t+.04,121),t+.09,.02)
        self.assertEqual(active['status']['startup_blend'],1.)
        self.assertEqual(lost['status']['startup_blend'],1.)
        self.assertEqual(recovered['status']['startup_blend'],1.)

    def test_persistent_rear_loss_brakes_and_repeated_frames_cannot_rearm(self):
        controller=self.controller()
        before=self.settled(controller)
        controller.update(packet(10.7,1,rear_valid=False),10.75,.02)
        stopped=controller.update(packet(11.31,2,rear_valid=False),11.36,.02)
        self.assertEqual(stopped['status']['state'],'stopped')
        self.assertLess(stopped['base_command_b'][0],before['base_command_b'][0])
        fresh=packet(11.4,3)
        for _ in range(50):
            result=controller.update(fresh,11.45,.02)
        self.assertEqual(result['status']['state'],'stopped')
        self.assertFalse(result['status']['valid'])

    def test_restart_needs_distinct_consistent_reacquisition_and_recovery_frames(self):
        controller=self.controller()
        self.settled(controller)
        controller.update(packet(10.7,1,rear_valid=False),10.75,.02)
        controller.update(packet(11.31,2,rear_valid=False),11.36,.02)
        states=[]
        for index in range(5):
            t=11.4+index*.04
            result=controller.update(packet(t,index+3),t+.05,.02)
            states.append(result['status']['state'])
        self.assertEqual(states[:4],['stopped']*4)
        self.assertEqual(states[4],'tracking')

    def test_front_loss_still_brakes_even_with_recent_good_rear(self):
        controller=self.controller()
        active=self.settled(controller)
        result=controller.update(packet(10.2,1,front_valid=False),10.25,.02)
        self.assertFalse(result['status']['valid'])
        self.assertLess(result['base_command_b'][0],active['base_command_b'][0])
        np.testing.assert_array_equal(result['tool_pose_b'],active['tool_pose_b'])

    def test_stale_frames_are_not_used_to_extend_rear_hold(self):
        controller=self.controller()
        self.settled(controller)
        result=controller.update(packet(),11.01,.02)
        self.assertFalse(result['status']['valid'])
        self.assertIn('stale',result['status']['reason'])

    def test_same_sequence_cannot_refresh_capture_timestamp(self):
        controller=self.controller()
        self.settled(controller)
        result=controller.update(packet(10.2,0),10.25,.02)
        self.assertFalse(result['status']['valid'])
        self.assertIn('refresh',result['status']['reason'])

    def test_repeated_bad_frame_does_not_become_accepted_as_clock_advances(self):
        controller=self.controller()
        self.settled(controller,packet(rear_y=-.14))
        bad=packet(10.1,1,rear_y=.17)
        for age in [.15,.3,.5,.7]:
            result=controller.update(bad,10.1+age,.02)
            self.assertFalse(result['status']['rear_valid'])
            self.assertIn('discontinuity',result['status']['rear_reason'])

    def test_new_generation_discards_old_path_and_requires_recovery(self):
        controller=self.controller()
        self.settled(controller)
        result=controller.update(packet(10.2,0,generation=2),10.25,.02)
        self.assertEqual(result['status']['state'],'stopped')
        self.assertEqual(result['status']['capture_generation'],2)

    def test_guard_rejects_privileged_fields_and_leaves_packets_unmodified(self):
        controller=self.controller()
        data=packet()
        saved=copy.deepcopy(data)
        result=controller.update(data,10.1,.02)
        self.assertTrue(result['status']['valid'])
        self.assertEqual(data,saved)
        data['rear']['hidden']={'base_position_w':[0,0,.5]}
        result=controller.update(data,10.1,.02)
        self.assertFalse(result['status']['valid'])
        self.assertIn('forbidden',result['status']['reason'])

    def test_workspace_and_target_rate_limit_are_preserved(self):
        controller=self.controller()
        self.settled(controller)
        previous=np.array([-1.02,0.,-.30])
        for index in range(1,40):
            t=10.+index*.02
            result=controller.update(packet(t,index,rear_y=index*.005),t+.05,.02)
            target=np.array(result['tool_pose_b'][:3])
            self.assertLessEqual(np.linalg.norm(target-previous),.15*.02+1e-12)
            self.assertLessEqual(abs(target[1]),.15)
            previous=target


if __name__=='__main__':
    unittest.main()
