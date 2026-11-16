"""Stop reference regressions, including the recorded 0.444 rad target reversal."""
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'python/vision')]
from control import ARM,LEGS,DEFAULT,STAND,Session,blend

class StopArmTests(unittest.TestCase):
    def test_early_stop_does_not_continue_startup_arm_blend(self):
        s=Session();s.mode='ready';s.target=DEFAULT.copy();s.prone=DEFAULT.copy()
        s.event('track',10,DEFAULT,True)
        raw=DEFAULT.copy();raw[13]=2.4
        s.actor_target(raw,10.46);reference=s.target[ARM].copy()
        s.event('stop',10.48,DEFAULT)
        for now in (10.48,10.8,11.0,11.46):
            raw[13]=2.671740174;s.actor_target(raw,now)
            np.testing.assert_array_equal(s.target[ARM],reference)
        self.assertEqual(s.mode,'braking')

    def test_recorded_braking_reversal_is_not_sent_but_leg_output_is_preserved(self):
        s=Session();s.mode='tracking';s.target=DEFAULT.copy();s.policy_started=0
        reference=s.target[ARM].copy();s.event('down',23.810379,DEFAULT)
        self.assertTrue(s.pending_down)
        for q2 in (2.671740174,2.227769628):
            raw=DEFAULT.copy();raw[13]=q2;raw[0]=.14
            s.actor_target(raw,24.79)
            np.testing.assert_array_equal(s.target[ARM],reference)
            np.testing.assert_array_equal(s.target[LEGS],raw[LEGS])
            self.assertEqual(raw[13],q2) # raw policy output was not overwritten

    def test_repeated_stop_does_not_rebase_reference_and_stand_return_keeps_it(self):
        s=Session();s.mode='tracking';s.target=DEFAULT.copy();s.target[13]=2.48
        s.event('stop',10,DEFAULT);reference=s.target.copy()
        s.event('down',10.2,STAND)
        np.testing.assert_array_equal(s.brake_arm_target,reference[ARM])
        for now in np.arange(10.2,12,.02):
            s.advance(float(now),DEFAULT,np.zeros(18),np.zeros(3),np.array([0,0,-1]))
            if s.mode=='return_stand':break
        self.assertEqual(s.mode,'return_stand')
        s.advance(s.entered+1.,DEFAULT,np.zeros(18),np.zeros(3),np.array([0,0,-1]))
        np.testing.assert_array_equal(s.target[ARM],reference[ARM])

    def test_tracking_entry_preserves_leg_blend_and_bounds_arm_reference(self):
        s=Session();s.mode='ready';s.target=DEFAULT.copy()
        s.event('track',10,DEFAULT,True);raw=DEFAULT+.05
        actual=s.actor_target(raw,10.5)
        np.testing.assert_array_equal(actual[LEGS],blend(DEFAULT,raw,.5)[LEGS])
        self.assertLessEqual(np.max(np.abs(actual[ARM]-DEFAULT[ARM])),.020000001)
        previous=actual.copy();actual=s.actor_target(raw,11.1)
        np.testing.assert_array_equal(actual[LEGS],raw[LEGS])
        self.assertLessEqual(np.max(np.abs(actual[ARM]-previous[ARM])),.020000001)
        s.brake_arm_target=DEFAULT[ARM].copy();s.mode='ready'
        s.event('track',12,DEFAULT,True);self.assertIsNone(s.brake_arm_target)

if __name__=='__main__':unittest.main()
