import tempfile
import threading
import time
import unittest
from pathlib import Path
import cv2
import numpy as np
from camera_diagnostics import DiagnosticWriter

class DiagnosticTests(unittest.TestCase):
    def test_worker_is_bounded_and_never_changes_source_pixels(self):
        with tempfile.TemporaryDirectory() as d:
            started=threading.Event();release=threading.Event()
            def blocked(path,image,options):
                started.set();release.wait(3.)
                return cv2.imwrite(path,image,options)
            writer=DiagnosticWriter(d,blocked)
            try:
                image=np.zeros((12,16,3),dtype=np.uint8)
                self.assertTrue(writer.overlay(image,image));self.assertTrue(started.wait(1.))
                image[:]=255
                self.assertTrue(writer.overlay(image,image));self.assertTrue(writer.overlay(image,image))
                before=time.monotonic();self.assertFalse(writer.overlay(image,image))
                self.assertLess(time.monotonic()-before,.2);self.assertEqual(writer.dropped,1)
                self.assertFalse(writer.close(.01))
                release.set();self.assertTrue(writer.close(2.))
                self.assertEqual(writer.errors,0)
                self.assertEqual(cv2.imread(str(Path(d)/'near_overlay.png')).shape,image.shape)
            finally:
                release.set();writer.close(2.)

    def test_rejection_owns_pixel_and_packet_copies(self):
        with tempfile.TemporaryDirectory() as d:
            writer=DiagnosticWriter(d)
            a=np.zeros((10,10,3),dtype=np.uint8);packet={'valid':False,'debug':{'count':1}}
            event=Path(d)/'rejection_1'
            self.assertTrue(writer.rejection(event,a,a,packet));a[:]=200;packet['debug']['count']=2
            self.assertTrue(writer.close(2.));self.assertEqual(writer.errors,0)
            self.assertIn('"count": 1',event.with_suffix('.json').read_text())
            self.assertLess(float(cv2.imread(str(event)+'_before.jpg').mean()),1.)

if __name__=='__main__':unittest.main()
