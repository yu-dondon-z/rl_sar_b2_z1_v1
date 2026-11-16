"""Reference slew and frozen-feedback regressions; no hardware IO."""
from pathlib import Path
import sys
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'python/vision')]
from control import ARM,LEGS,DEFAULT,Session
from feedback import Z1MotorResponse
from native_io import State


class ArmHandoverTests(unittest.TestCase):
    def session(self):
        s=Session();s.mode='ready';s.target=DEFAULT.copy()
        s.event('track',10.,DEFAULT,True)
        return s

    def test_reversals_delayed_ticks_and_input_immutability(self):
        s=self.session();previous=s.target.copy();last=10.
        for now,q2 in ((10.,2.1),(10.02,2.9),(10.04,1.9),(11.5,2.9),(11.51,1.9)):
            raw=DEFAULT.copy();raw[13]=q2;original=raw.copy()
            actual=s.actor_target(raw,now)
            self.assertLessEqual(np.max(np.abs(actual[ARM]-previous[ARM])),min(now-last,.02)+1e-12)
            np.testing.assert_array_equal(raw,original)
            if now>=11:np.testing.assert_array_equal(actual[LEGS],raw[LEGS])
            previous=actual.copy();last=now

    def test_constant_target_eventually_arrives_without_overshoot(self):
        s=self.session();raw=DEFAULT.copy();raw[13]=1.9
        for now in np.arange(10.,13.,.02):
            actual=s.actor_target(raw,float(now))
            self.assertGreaterEqual(actual[13],1.9)
            self.assertLessEqual(actual[13],2.55)
        self.assertAlmostEqual(actual[13],1.9)

    def test_moving_feedback_freeze_latches_without_rebasing_support_or_gains(self):
        s=self.session();state=State();state.q[:]=DEFAULT;state.dq[13]=2.36
        monitor=Z1MotorResponse();monitor.observe(state,10.)
        for now in (10.02,10.1,10.19):
            self.assertFalse(monitor.observe(state,now)['z1_motor_vector_repeated_while_moving'])
        self.assertTrue(monitor.observe(state,10.21)['z1_motor_vector_repeated_while_moving'])
        reference=s.target.copy();kp=s.kp.copy();kd=s.kd.copy()
        s.block_motor_feedback(10.21)
        for event in ('down','track','stand','prepare','stop'):
            s.event(event,10.3,DEFAULT,True)
            s.advance(10.3,DEFAULT,np.zeros(18),np.zeros(3),np.array([0,0,-1]))
            self.assertEqual(s.mode,'held')
            np.testing.assert_array_equal(s.target,reference)
            np.testing.assert_array_equal(s.kp,kp);np.testing.assert_array_equal(s.kd,kd)
        # A subsequent changing vector is not permission to resume movement.
        state.q[13]+=.01;monitor.observe(state,10.4)
        self.assertTrue(s.motor_feedback_blocked)
        s.event('emergency',10.5,DEFAULT)
        self.assertEqual(s.mode,'fault')

    def test_existing_native_fault_is_not_masked(self):
        s=self.session();s.transition('fault',10.1,'native IO fault 4')
        s.block_motor_feedback(10.2)
        self.assertEqual(s.mode,'fault');self.assertEqual(s.message,'native IO fault 4')

if __name__=='__main__':unittest.main()
