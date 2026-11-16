"""Legacy get-up and B interruption regression; no hardware transports."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock
import ctypes as ct
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'python/vision')]
from control import (ARM,LEGS,DEFAULT,STAND,PRE_STAND,Session,
                     FIXED_KP,FIXED_KD,STAND_KP,STAND_KD,POLICY_KP,POLICY_KD)
from native_io import Command,send,library
GRAVITY=np.array([0.,0.,-1.]);ZERO=np.zeros(18)

def poses():
    prone=DEFAULT.copy();prone[:8]=[.294,-.285,.286,-.289,1.214,1.212,1.211,1.212]
    prone[9:13]=[-2.803,-2.804,-2.790,-2.800];prone[ARM]=0
    actual=prone.copy();actual[:8]=[.0647,-.0677,.0829,-.0874,.614,.642,.681,.669]
    actual[9:13]=[-1.687,-1.638,-1.905,-1.849]
    return prone,actual

def advance(s,now,q,qd=ZERO):return s.advance(now,q,qd,np.zeros(3),GRAVITY)

class StandRecoveryTests(unittest.TestCase):
    def stalled(self):
        prone,actual=poses();s=Session();s.event('stand',0,prone)
        advance(s,1.,actual);advance(s,3.,actual);advance(s,9.02,actual)
        self.assertEqual(s.mode,'held');return s,actual

    def test_first_second_uses_folded_prepose_not_direct_extension(self):
        prone,_=poses();s=Session();s.event('stand',0,prone)
        self.assertEqual(s.mode,'pre_stand');advance(s,.5,prone)
        np.testing.assert_allclose(s.target[LEGS],.5*(prone[LEGS]+PRE_STAND[LEGS]))
        np.testing.assert_array_equal(s.target[ARM],prone[ARM]);np.testing.assert_array_equal(s.kp,FIXED_KP)
        advance(s,1.,prone);self.assertEqual(s.mode,'standing_up')
        np.testing.assert_allclose(s.target[LEGS],PRE_STAND[LEGS]);advance(s,2.,prone)
        np.testing.assert_allclose(s.target[LEGS],.5*(PRE_STAND[LEGS]+STAND[LEGS]))
        np.testing.assert_array_equal(s.target[ARM],prone[ARM])

    def test_real_stand_gain_ramp_does_not_modify_position_target(self):
        prone,_=poses();s=Session();s.event('stand',0,prone)
        advance(s,1,prone);advance(s,3,prone);goal=s.target.copy()
        self.assertEqual(s.mode,'stand_settle');np.testing.assert_allclose(s.kd[LEGS],8.)
        advance(s,4,goal);np.testing.assert_allclose(s.kp,.5*(FIXED_KP+STAND_KP))
        np.testing.assert_allclose(s.kd[LEGS],11.5);advance(s,5,goal)
        self.assertEqual(s.mode,'standing');np.testing.assert_array_equal(s.target,goal)
        np.testing.assert_allclose(s.kp[[0,4,9]],[450,450,675]);np.testing.assert_allclose(s.kd[LEGS],15.)

    def test_half_stand_not_success_no_bias_and_blocked_joint_named(self):
        s,actual=self.stalled();self.assertIn('RL_calf_joint',s.message);self.assertIn('0.405',s.message)
        np.testing.assert_array_equal(s.kp,STAND_KP);goal=s.target.copy();advance(s,30,actual)
        np.testing.assert_array_equal(s.target,goal)

    def test_b_half_stand_preserves_reference_and_skips_unrelated_arrival_gate(self):
        s,actual=self.stalled();q=s.target.copy();kp=s.kp.copy();kd=s.kd.copy()
        s.event('down',10,actual);self.assertEqual(s.mode,'lowering');advance(s,10,actual)
        np.testing.assert_array_equal(s.target,q)
        np.testing.assert_array_equal(s.kp,kp);np.testing.assert_array_equal(s.kd,kd)
        advance(s,15.1,s.prone);self.assertEqual(s.mode,'idle');np.testing.assert_array_equal(s.kp,FIXED_KP)

    def test_b_interrupts_preparation_getup_and_settling(self):
        for t in (.5,2.,4.):
            prone,_=poses();s=Session();s.event('stand',0,prone)
            for now in np.arange(0,t+.001,.02):advance(s,float(now),prone)
            before=s.target.copy();s.event('down',t+.001,prone)
            self.assertEqual(s.mode,'lowering');np.testing.assert_array_equal(s.start,before)

    def test_arm_return_checks_only_arm_and_failure_remains_visible(self):
        for arrived in (False,True):
            s,q=self.stalled();s.target[ARM]=DEFAULT[ARM];q[ARM]=DEFAULT[ARM]
            s.event('down',10,q);self.assertEqual(s.mode,'arm_lower')
            if arrived:q[ARM]=s.prone[ARM]
            advance(s,20.1,q);self.assertEqual(s.mode,'lowering' if arrived else 'held')

    def test_real_position_and_stable_window_required(self):
        prone,_=poses();s=Session();s.event('stand',0,prone)
        advance(s,1,prone);advance(s,3,prone);goal=s.target.copy();advance(s,4,goal)
        moving=goal.copy();moving[4]+=.04;advance(s,4.9,moving);advance(s,5.,goal)
        self.assertEqual(s.mode,'stand_settle');advance(s,5.51,goal);self.assertEqual(s.mode,'standing')

    def test_policy_and_stop_blend_from_actual_stand_gains(self):
        s=Session();s.mode='ready';s.target=STAND.copy();s.prone=STAND.copy()
        s.kp=STAND_KP.copy();s.kd=STAND_KD.copy()
        s.event('track',10,STAND,True);s.policy_gains(10);np.testing.assert_array_equal(s.kp,STAND_KP)
        s.policy_gains(10.5);np.testing.assert_allclose(s.kp,.5*(STAND_KP+POLICY_KP))
        s.policy_gains(11);np.testing.assert_array_equal(s.kp,POLICY_KP);np.testing.assert_array_equal(s.kd,POLICY_KD)
        s.move('return_stand',12,STAND,'stop');advance(s,12,STAND)
        np.testing.assert_array_equal(s.kp,POLICY_KP);advance(s,15.1,STAND)
        np.testing.assert_array_equal(s.kp,STAND_KP)

    def test_native_receives_explicit_gains_and_rejects_corrupt_values(self):
        lib=Mock();lib.vd2_command.return_value=0
        send(lib,STAND,leg_kp=STAND_KP,leg_kd=STAND_KD)
        value=ct.cast(lib.vd2_command.call_args[0][0],ct.POINTER(Command)).contents
        np.testing.assert_array_equal(value.leg_kp,STAND_KP);np.testing.assert_array_equal(value.leg_kd,STAND_KD)
        real=library();self.assertEqual(real.vd2_command(ct.byref(value)),0)
        for field,index,bad in (('leg_kp',0,451.),('leg_kp',9,676.),('leg_kd',0,15.1),
                                ('leg_kd',0,float('nan')),('leg_kp',8,1.)):
            command=Command.from_buffer_copy(value);getattr(command,field)[index]=bad
            self.assertNotEqual(real.vd2_command(ct.byref(command)),0)

    def test_lowering_checks_actual_fold_and_velocity_and_keeps_hip_residual(self):
        s,q=self.stalled();s.event('down',10,q)
        folded=s.prone.copy();folded[:2]+=[.17,-.17]
        advance(s,15.1,folded);self.assertEqual(s.mode,'idle')
        self.assertAlmostEqual(np.max(np.abs(folded[:4]-s.target[:4])),.17)
        for moving in (False,True):
            s,q=self.stalled();s.event('down',10,q);folded=s.prone.copy()
            if not moving:folded[9]-=.3
            advance(s,15.1,folded,np.ones(18) if moving else ZERO);self.assertEqual(s.mode,'lowering')

if __name__=='__main__':unittest.main()
