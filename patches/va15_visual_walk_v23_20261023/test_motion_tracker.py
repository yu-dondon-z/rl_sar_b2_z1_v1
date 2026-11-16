import json
import tempfile
import unittest
from pathlib import Path
import cv2
import numpy as np
from camera_context import Context
from near_tracker import NearTracker
from long_tracker import LongTracker
from test_tracker import scene


def textured(center=935):
    image, mask, probability=scene(center=center)
    small=np.full((360,640,3),220,np.uint8)
    rng=np.random.RandomState(42)
    for y in range(179,350,14):
        for x in range(60,581,18):
            x,y1=x+int(rng.randint(-3,4)),y+int(rng.randint(-2,3))
            value=int(rng.choice([195,245]))
            cv2.rectangle(small,(x,y1),(x+3,y1+3),(value,)*3,-1)
    background=cv2.resize(small,(1920,1080),interpolation=cv2.INTER_NEAREST)
    background[mask>0]=image[mask>0]
    return background,mask,probability


def warp(data,dx,dy=0.,angle=0.):
    transform=cv2.getRotationMatrix2D((960.,972.),angle,1.)
    transform[:,2]+=[dx,dy]
    image,mask,_=data
    image=cv2.warpAffine(image,transform,(1920,1080),borderValue=(220,)*3)
    mask=cv2.warpAffine(mask,transform,(1920,1080),flags=cv2.INTER_NEAREST)
    return image,mask,np.where(mask,.995,.001).astype(np.float32)


class MotionTests(unittest.TestCase):
    def test_background_motion_does_not_look_like_a_new_line(self):
        data=textured();tracker=LongTracker()
        tracker.detect(*data,timestamp_ns=1_000_000_000)
        result=tracker.detect(*warp(data,21.,6.),timestamp_ns=1_200_000_000)
        self.assertTrue(result['motion_compensation_valid'])
        self.assertGreaterEqual(result['motion_inliers'],20)
        self.assertLess(result['association_delta_px'],4.)
        self.assertLess(abs(result['near_u']-956),4.)
        self.assertTrue(result['long_valid'])

    def test_stride_period_motion_keeps_the_same_line(self):
        data=textured();tracker=LongTracker()
        for n in range(28):
            phase=n*np.pi/3.
            result=tracker.detect(*warp(data,18*np.sin(phase),6*np.sin(phase),.6*np.sin(phase)),
                                  timestamp_ns=1_000_000_000+n*180_000_000)
            self.assertEqual(result['stable_frames'],n+1)
            self.assertTrue(result['long_valid'])

    def test_independent_line_switch_is_still_rejected(self):
        tracker=NearTracker();tracker.detect(*textured(),timestamp_ns=1_000_000_000)
        with self.assertRaisesRegex(ValueError,'Near tape changed'):
            tracker.detect(*textured(center=1020),timestamp_ns=1_200_000_000)
        self.assertGreater(tracker.debug['association_delta_px'],60.)

    def test_no_texture_is_not_permission_to_open_the_gate(self):
        tracker=NearTracker();tracker.detect(*scene(),timestamp_ns=1_000_000_000)
        with self.assertRaises(ValueError):
            tracker.detect(*warp(scene(),21.),timestamp_ns=1_200_000_000)

    def test_excessive_motion_and_stale_registration_rejected(self):
        for dx,stamp in [(110.,1_200_000_000),(21.,2_000_000_000)]:
            tracker=NearTracker();tracker.detect(*textured(),timestamp_ns=1_000_000_000)
            with self.assertRaises(ValueError):tracker.detect(*warp(textured(),dx),timestamp_ns=stamp)

    def test_context_automatic_standing_acquisition_not_moving_reacquisition(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'context.json';c=Context();nonce='a'*32;now=1_000_000_000
            self.assertFalse(c.update(path,nonce,now,True)['valid'])
            for phase,reset,reacquire in [(0,True,True),(3,False,False),(1,True,True),
                                          (2,False,False),(3,False,False),(1,True,True)]:
                path.write_text(json.dumps(dict(session_id=nonce,phase=phase,monotonic_ns=now)))
                result=c.update(path,nonce,now,True)
                self.assertEqual((result['reset'],result['reacquire']),(reset,reacquire))
            self.assertFalse(c.update(path,nonce,now+1_000_000_001,True)['valid'])
            self.assertFalse(c.update(path,'b'*32,now,True)['valid'])


if __name__=='__main__':unittest.main()
