"""GUI backpressure and failures must not change camera/control evidence."""
import importlib.util
import multiprocessing as mp
import queue
from pathlib import Path
import sys
import tempfile
import threading
import unittest
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
import visual_preview as view
spec=importlib.util.spec_from_file_location('preview_launch',ROOT/'tools/launch_real.py')
launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)


def publish_without_reader(output):
    sample=dict(image=np.zeros((480,640,3),np.uint8),received=10.,number=1)
    view.PreviewPublisher(output).submit(sample,sample)


class VisualPreviewTests(unittest.TestCase):
    def test_masks_are_visible_without_claiming_valid_path_or_changing_source(self):
        sample=self.sample();original=sample['image'].copy()
        sample['raw_mask']=np.full((480,640),255,np.uint8)
        sample['filtered_mask']=np.zeros((480,640),np.uint8)
        sample['filtered_mask'][:,320:]=255
        result=view.small_frame(sample,dict(valid=False,reason='missing depth'))
        self.assertFalse(result['valid']);self.assertEqual(len(result['waypoints_uv']),0)
        self.assertFalse(np.array_equal(result['image'][100,200],result['image'][100,440]))
        np.testing.assert_array_equal(sample['image'],original)

    def sample(self):
        return dict(image=np.full((480,640,3),100,np.uint8),received=10.,number=25)

    def test_reduced_copy_preserves_identity_aspect_ratio_and_source_pixels(self):
        sample=self.sample();original=sample['image'].copy()
        uv=[[0,0],[639,479]];result=dict(valid=True,reason='ok',waypoints_uv=uv)
        reduced=view.small_frame(sample,result)
        self.assertEqual((reduced['received'],reduced['number']),(10.,25))
        self.assertEqual(reduced['image'].shape,(360,640,3))
        np.testing.assert_array_equal(reduced['waypoints_uv'],np.array(uv)*.75+[80,0])
        np.testing.assert_array_equal(sample['image'],original)
        self.assertEqual(result['waypoints_uv'],uv)
        self.assertTrue(np.all(reduced['image'][:,:80]==0))

    def test_full_queue_drops_without_resizing_or_waiting(self):
        output=queue.Queue(maxsize=1);output.put('busy')
        publisher=view.PreviewPublisher(output)
        with patch.object(view,'small_frame') as resize:
            self.assertFalse(publisher.submit(self.sample(),self.sample()))
        resize.assert_not_called();self.assertEqual(output.get_nowait(),'busy')

    def test_rate_limit_and_original_timestamps_are_retained(self):
        output=queue.Queue(maxsize=1);publisher=view.PreviewPublisher(output)
        with patch.object(view.time,'monotonic',return_value=20.):
            self.assertTrue(publisher.submit(self.sample(),self.sample()))
        item=output.get_nowait()
        self.assertEqual(item['front']['received'],10.)
        with patch.object(view.time,'monotonic',return_value=20.1):
            self.assertFalse(publisher.submit(self.sample(),self.sample()))
        self.assertTrue(output.empty())

    def test_stale_label_uses_receipt_time_without_modifying_frame(self):
        frame=view.small_frame(self.sample(),dict(valid=True,reason='ok',waypoints_uv=[[200,200],[220,220]]))
        original=frame['image'].copy();labels=[]
        with patch.object(view,'text_line',side_effect=lambda panel,text,*args:labels.append(text)):
            view.render_panel('front',frame,{},dict(state='ready'),11.1)
        self.assertIn('VALID / STALE',labels[0])
        self.assertIn('1.100 s',labels[1]);self.assertEqual(frame['received'],10.)
        np.testing.assert_array_equal(frame['image'],original)

    def test_display_error_is_contained_in_viewer_and_recorded(self):
        closing=threading.Event()
        with tempfile.TemporaryDirectory() as directory,\
             patch.object(view.cv2,'namedWindow',side_effect=RuntimeError('desktop unavailable')),\
             patch.object(view.cv2,'destroyAllWindows'):
            view.viewer_worker(queue.Queue(),queue.Queue(),closing,directory)
            self.assertIn('desktop unavailable',(Path(directory)/'display_error.txt').read_text())
        self.assertFalse(closing.is_set())

    def test_future_timestamp_cannot_show_a_fresh_valid_detection(self):
        frame=view.small_frame(self.sample(),dict(valid=True,reason='ok'))
        labels=[]
        with patch.object(view,'text_line',side_effect=lambda panel,text,*args:labels.append(text)):
            view.render_panel('front',frame,{},dict(state='ready'),9.)
        self.assertIn('CLOCK MISMATCH',labels[0]);self.assertEqual(frame['received'],10.)

    def test_q_only_exits_viewer_and_does_not_signal_control_stop(self):
        closing=threading.Event()
        with tempfile.TemporaryDirectory() as directory,\
             patch.object(view.cv2,'namedWindow'),patch.object(view.cv2,'moveWindow'),\
             patch.object(view.cv2,'imshow'),patch.object(view.cv2,'destroyAllWindows'),\
             patch.object(view.cv2,'waitKey',return_value=ord('q')):
            view.viewer_worker(queue.Queue(),queue.Queue(),closing,directory)
        self.assertFalse(closing.is_set())

    def test_no_display_does_not_start_a_child_or_guess_desktop(self):
        self.assertFalse(view.display_available({}))
        with patch.dict(view.os.environ,{},clear=True):
            preview=view.VisualPreview(None,True,'.')
        self.assertIsNone(preview.frames);preview.close()

    def test_unsupported_visibility_property_cannot_close_live_windows(self):
        closing=threading.Event()
        with tempfile.TemporaryDirectory() as directory,\
             patch.object(view.cv2,'namedWindow'),patch.object(view.cv2,'moveWindow'),\
             patch.object(view.cv2,'imshow') as show,patch.object(view.cv2,'destroyAllWindows'),\
             patch.object(view.cv2,'getWindowProperty',return_value=-1) as visible,\
             patch.object(view.cv2,'waitKey',side_effect=[-1]*5+[ord('q')]),\
             patch.object(closing,'wait'):
            view.viewer_worker(queue.Queue(),queue.Queue(),closing,directory)
        self.assertEqual(show.call_count,12)
        visible.assert_not_called()
        self.assertFalse(closing.is_set())

    def test_launcher_defaults_to_windows_and_allows_opt_out(self):
        with patch.object(sys,'argv',['start_real.sh']):self.assertTrue(launch.parse_args().display)
        with patch.object(sys,'argv',['start_real.sh','--no-display']):self.assertFalse(launch.parse_args().display)

    def test_camera_publisher_exits_even_when_viewer_has_no_reader(self):
        context=mp.get_context('spawn');output=context.Queue(maxsize=1)
        process=context.Process(target=publish_without_reader,args=(output,))
        process.start()
        try:
            process.join(timeout=5)
            self.assertFalse(process.is_alive(),'diagnostic pipe blocked camera-worker exit')
            self.assertEqual(process.exitcode,0)
        finally:
            if process.is_alive():process.terminate();process.join(timeout=2)
            output.cancel_join_thread();output.close()


if __name__=='__main__':unittest.main()
