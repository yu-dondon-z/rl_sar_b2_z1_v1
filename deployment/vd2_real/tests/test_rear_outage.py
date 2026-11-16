"""Capture deadlines and time spent rejecting a rear sample are distinct."""
from pathlib import Path
import sys,unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'python/vision')]
from vd1_visual_guard import GuardedVD1VisualController
from vd1_control import SCHEMA

def packet(t,seq,y=0):
    shared=dict(capture_timestamp=t,sequence=seq,capture_generation=1,valid=True,reason='ok')
    def points(xs):return np.c_[xs,np.full(len(xs),y),np.full(len(xs),-.5)].tolist()
    return dict(schema=SCHEMA,capture_generation=1,
        front=dict(shared,points_b=points(np.linspace(.4,2,20))),
        rear=dict(shared,points_b=points(np.linspace(-1.3,-.8,20)),ground_plane_b=[0,0,1,.5]))

class RearOutageTests(unittest.TestCase):
    def test_observed_out_of_workspace_is_not_reported_as_vision_loss_or_commanded(self):
        c=GuardedVD1VisualController([-1.02,-.12,-.3],paired_guidance=False)
        initial=c.update(packet(10,1,-.12),10.1,.02)
        result=c.update(packet(10.2,2,-.174),10.3,.02)
        self.assertTrue(result['status']['rear_valid'])
        self.assertFalse(result['status']['valid'])
        self.assertIn('outside actor workspace',result['status']['reason'])
        self.assertFalse(result['status']['target_updated'])
        np.testing.assert_array_equal(result['tool_pose_b'],initial['tool_pose_b'])

    def test_one_outlier_does_not_spend_processing_age_as_outage(self):
        c=GuardedVD1VisualController([-1.02,0,-.3],paired_guidance=False)
        self.assertTrue(c.update(packet(10,1),10.55,.02)['status']['valid'])
        result=c.update(packet(10.35,2,.277),10.74,.02)
        self.assertEqual(result['status']['state'],'rear_hold')
        self.assertAlmostEqual(result['status']['rear_trusted_capture_age_s'],.74)
        self.assertEqual(result['status']['rear_outage_s'],0)
        self.assertFalse(result['status']['target_updated'])
        result=c.update(packet(10.55,3,.005),10.95,.02)
        self.assertEqual(result['status']['state'],'tracking')

    def test_fresh_bad_packets_cannot_refresh_last_trusted_capture(self):
        c=GuardedVD1VisualController([-1.02,0,-.3],paired_guidance=False)
        c.update(packet(10,1),10.55,.02)
        c.update(packet(10.35,2,.277),10.74,.02)
        held=c.update(packet(10.7,3,.277),11.01,.02)
        self.assertEqual(held['status']['state'],'rear_hold')
        self.assertFalse(held['status']['target_updated'])
        result=c.update(packet(11.,4,.277),11.35,.02)
        self.assertFalse(result['status']['valid'])
        self.assertGreater(result['status']['rear_trusted_capture_age_s'],1.)
        self.assertEqual(result['status']['rear_source_sequence'],1)

    def test_single_missing_frame_recovers_after_processing_delay_without_reacquisition_lock(self):
        c=GuardedVD1VisualController([-1.02,0,-.3],paired_guidance=False)
        c.update(packet(10,1),10.6,.02)
        c.update(packet(10.365,2,.277),10.733,.02)
        held=c.update(packet(10.365,2,.277),11.02,.02)
        self.assertEqual(held['status']['state'],'rear_hold')
        result=c.update(packet(10.73,3,.005),11.1,.02)
        self.assertEqual(result['status']['state'],'tracking')
        self.assertTrue(result['status']['target_updated'])
