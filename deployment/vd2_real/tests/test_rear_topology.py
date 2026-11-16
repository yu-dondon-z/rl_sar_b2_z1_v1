"""RGB continuity must not turn unknown depth into measured support."""
import sys
from pathlib import Path
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'python/vision')]
from realtime_perception import rear_topology_roi,rear_work_section
from dual_camera_perception import PerceptionConfig,rear_ground_roi


class RearTopologyTests(unittest.TestCase):
    def setUp(self):
        self.k=np.array([[600.,0,80],[0,600.,60],[0,0,1]])
        self.tf=np.diag([1.,-1.,-1.,1.])
        self.plane=np.array([0.,0.,1.,1.])
        self.cfg=PerceptionConfig()

    def roi(self,depth):
        allowed,_=rear_ground_roi(depth,self.k,self.tf,self.plane,self.cfg)
        return rear_topology_roi(depth,allowed,self.k,self.tf,self.plane,self.cfg)

    def test_unknown_strip_preserves_rgb_topology_without_filling_depth(self):
        depth=np.ones((120,160));depth[50:70]=np.nan
        original=depth.copy()
        self.assertTrue(self.roi(depth)[60,80])
        np.testing.assert_array_equal(depth,original)

    def test_foreground_and_adjacent_holes_remain_excluded(self):
        depth=np.ones((120,160));depth[50:70,70:90]=.8
        depth[59:62,79:82]=np.nan
        roi=self.roi(depth)
        self.assertFalse(roi[55,75])
        self.assertFalse(roi[60,80])

    def test_rgb_continuity_does_not_relax_measured_work_bracket(self):
        x=np.r_[np.linspace(-1.1,-1.04,8),np.linspace(-1.,-.94,8)]
        points=np.c_[x,np.zeros((16,2))]
        with self.assertRaisesRegex(ValueError,'2.5 cm'):
            rear_work_section({},points,np.zeros((16,2)),np.ones(16,bool),1.,self.cfg)

    def test_missing_one_side_cannot_be_invented(self):
        x=np.linspace(-1.08,-.96,20);points=np.c_[x,np.zeros((20,2))]
        with self.assertRaisesRegex(ValueError,'both sides'):
            rear_work_section({},points,np.zeros((20,2)),x<-1.02,1.,self.cfg)


if __name__=='__main__':unittest.main()
