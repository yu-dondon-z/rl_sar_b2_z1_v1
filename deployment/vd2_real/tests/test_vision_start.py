import unittest
from pathlib import Path
import sys
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'python/vision'),str(ROOT/'thirdparty/rhacracknet')]
from vision_start import packet_status,VisionStartRequest,StableVisualGeometry
from control import Session,DEFAULT
from vd1_visual_guard import GuardedVD1VisualController


def packet():
    front=dict(valid=True,reason='ok',sequence=3,capture_generation=1,capture_timestamp=10.,
               points_b=np.column_stack([np.linspace(.4,2.,12),np.zeros(12),np.full(12,-.58)]).tolist())
    rear=dict(front,points_b=np.column_stack([np.linspace(-1.2,-.6,12),np.zeros(12),np.full(12,-.58)]).tolist(),
              ground_plane_b=[0,0,1,.58])
    return dict(kind='packet',schema='b2_z1_dual_vision_v1',valid=True,capture_generation=1,
                front=front,rear=rear,processing_ms=250.)


class VisualReadinessTests(unittest.TestCase):
    def test_one_accidentally_valid_packet_cannot_trigger_policy_entry(self):
        gate=StableVisualGeometry();p=packet()
        status=dict(valid=True,rear_valid=True,front_lookahead_b=[.6,.02,-.55])
        for _ in range(100):self.assertFalse(gate.update(p,status))
        self.assertEqual(gate.count,1)
        for seq in (4,5):
            p['front']['sequence']=seq
            accepted=gate.update(p,status)
        self.assertTrue(accepted)
        self.assertFalse(gate.update(p,dict(status,valid=False)))
        self.assertEqual(gate.count,0)

    def test_start_geometry_jump_generation_change_or_rear_hold_resets_stability(self):
        status=dict(valid=True,rear_valid=True,front_lookahead_b=[.6,.02,-.55])
        for alteration in ('jump','generation','hold'):
            p=packet();gate=StableVisualGeometry();gate.update(p,status)
            p['front']['sequence']=4;gate.update(p,status)
            p['front']['sequence']=5;changed=dict(status)
            if alteration=='jump':changed['front_lookahead_b']=[.6,.3,-.55]
            if alteration=='generation':p['front']['capture_generation']=2
            if alteration=='hold':changed['rear_valid']=False
            self.assertFalse(gate.update(p,changed))

    def test_old_valid_packets_are_rejected_with_real_age_and_processing_cost(self):
        p=packet();p['processing_ms']=2317.
        status=packet_status(p,1,12.317)
        self.assertFalse(status['valid']);self.assertIn('stale',status['reason'])
        self.assertAlmostEqual(status['rear_age_s'],2.317)
        self.assertEqual(p['rear']['capture_timestamp'],10.)

    def test_independent_clock_skew_future_generation_and_invalid_detection_fail(self):
        changes=[('capture_timestamp',10.05),('capture_timestamp',11.),('capture_timestamp',float('nan')),
                 ('sequence',4),('valid',False)]
        for field,value in changes:
            p=packet();p['rear'][field]=value
            with self.subTest(field=field,value=value):self.assertFalse(packet_status(p,1,10.3)['valid'])
        self.assertFalse(packet_status(packet(),2,10.3)['valid'])
        self.assertTrue(packet_status(packet(),1,10.3)['valid'])

    def test_visible_segment_must_cover_the_trained_rear_target(self):
        p=packet();p['rear']['points_b']=[[-.9+.3*i/11,0,-.58] for i in range(12)]
        status=GuardedVD1VisualController([-1.02,0,-.3],forward_speed=.2).update(p,10.3,.02)['status']
        self.assertFalse(status['valid']);self.assertIn('x=-1.02',status['rear_reason'])
        np.testing.assert_allclose(packet_status(p,1,10.3)['rear_observed_x_m'],[-.9,-.6])
        status=GuardedVD1VisualController([-1.02,0,-.3],forward_speed=.2).update(packet(),10.3,.02)['status']
        self.assertTrue(status['valid'],status)

    def test_pending_request_waits_in_ready_and_enters_tracking_only_after_fresh_geometry(self):
        start=VisionStartRequest();session=Session();session.mode='ready';session.target=DEFAULT.copy()
        event=start.update('track',session.mode,10.,False);session.event(event,10.,DEFAULT,False)
        self.assertTrue(start.pending);self.assertEqual(session.mode,'ready')
        event=start.update(None,session.mode,10.5,False);session.event(event,10.5,DEFAULT,False)
        self.assertEqual(session.mode,'ready')
        event=start.update(None,session.mode,10.7,True);session.event(event,10.7,DEFAULT,True)
        self.assertEqual(event,'track');self.assertEqual(session.mode,'tracking');self.assertFalse(start.pending)
        # Subsequent stop/recovery never automatically issues another start.
        self.assertEqual(start.update('stop','tracking',11.,True),'stop')
        self.assertIsNone(start.update(None,'ready',12.,True))

    def test_timeout_and_all_cancel_events_cannot_start_on_a_later_valid_frame(self):
        for cancel in ('stop','down','emergency','prepare','stand','track'):
            start=VisionStartRequest();start.update('track','ready',10.,False)
            returned=start.update(cancel,'ready',10.1,True)
            self.assertEqual(returned,None if cancel=='track' else cancel)
            self.assertFalse(start.pending);self.assertIsNone(start.update(None,'ready',10.2,True))
        start=VisionStartRequest();start.update('track','ready',10.,False)
        self.assertIsNone(start.update(None,'ready',15.,True));self.assertIn('timed out',start.message)
        self.assertIsNone(start.update(None,'ready',15.1,True))
        start.update('track','ready',16.,False);start.update(None,'held',16.1,False)
        self.assertFalse(start.pending);self.assertIsNone(start.update(None,'ready',16.2,True))


if __name__=='__main__':unittest.main()
