import ctypes as ct
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'python/vision')]
from control import DEFAULT,Session,position_target,limits
from native_io import Command,library


class RealStartTargetTests(unittest.TestCase):
    def passive_pose(self):
        q=DEFAULT.copy()
        q[:4]=[.29060,-.29845,.27846,-.29531]
        q[4:8]=[1.21210,1.21141,1.21118,1.21323]
        q[[9,10,11,12]]=[-2.79826,-2.80430,-2.78918,-2.79998]
        q[[8,13,14,15,16,17]]=[-.02107,-.003239,.002053,-.151316,.215212,.042533]
        return q

    def test_actual_passive_zero_offsets_no_longer_reject_first_stand_command(self):
        measured=self.passive_pose();original=measured.copy()
        session=Session();session.event('stand',10.,measured)
        session.advance(10.,measured,np.zeros(18),np.zeros(3),np.array([0.,0.,-1.]))
        lib=library()
        raw=Command((ct.c_double*18)(*session.target),0.,1)
        self.assertNotEqual(lib.vd2_command(ct.byref(raw)),0)
        target,applied=position_target(session.target,limits())
        self.assertTrue(applied)
        self.assertEqual(target[13],0.);self.assertEqual(target[14],0.)
        command=Command((ct.c_double*18)(*target),0.,1)
        self.assertEqual(lib.vd2_command(ct.byref(command)),0)
        np.testing.assert_array_equal(measured,original)
        np.testing.assert_array_equal(session.prone,original)

    def test_large_position_error_and_nonfinite_targets_are_still_rejected(self):
        bad=self.passive_pose();bad[13]=-.02
        with self.assertRaises(ValueError):position_target(bad,limits())
        bad[13]=float('nan')
        with self.assertRaises(ValueError):position_target(bad,limits())


if __name__=='__main__':unittest.main()
