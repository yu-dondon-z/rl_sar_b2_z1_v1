"""Independent publication never renews the other camera's evidence."""
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'python'), str(ROOT/'python/vision'), str(ROOT/'thirdparty/rhacracknet')]
from cameras import Capture
from independent_vision import run
from vision_start import packet_status, StableVisualGeometry
from vd1_visual_guard import GuardedVD1VisualController


def packet():
    def stream(seq, stamp, x):
        return dict(valid=True, sequence=seq, capture_generation=1, capture_timestamp=stamp,
            ground_plane_b=[0., 0., 1., .55],
            points_b=np.c_[x, .02+0*x, -.55+0*x].tolist())
    return dict(kind='packet', schema='b2_z1_dual_vision_v1', valid=True,
        capture_generation=1, publication_sequence=0, stream_mode='independent_latest',
        front_path_estimation=True, front=stream(3, 10., np.linspace(1.2, 2., 40)),
        rear=stream(7, 9.8, np.linspace(-1.3, -.8, 40)))


class IndependentVisionTests(unittest.TestCase):
    def controller(self):
        return GuardedVD1VisualController([-1.02, 0., -.3], forward_speed=.2,
                                         front_path_estimation=True)

    def test_independent_clocks_are_valid_but_never_renew_each_other(self):
        p = packet(); c = self.controller()
        self.assertTrue(packet_status(p, 1, 10.2)['valid'])
        self.assertTrue(c.update(p, 10.2, .02)['status']['valid'])
        p['publication_sequence'] += 1
        p['front'].update(sequence=4, capture_timestamp=10.4)
        status = c.update(p, 10.6, .02)['status']
        self.assertTrue(status['valid'], status)
        self.assertEqual(status['rear_source_sequence'], 7)
        self.assertAlmostEqual(status['rear_trusted_capture_age_s'], .8)
        self.assertEqual(status['front_startup_distinct_stable_frames'], 2)
        self.assertFalse(packet_status(p, 1, 10.81)['valid'])
        self.assertFalse(c.update(p, 10.81, .02)['status']['valid'])

    def test_rear_updates_do_not_count_as_front_stable_captures(self):
        p = packet(); c = self.controller()
        for i in range(6):
            p['publication_sequence'] = i
            p['rear'].update(sequence=7+i, capture_timestamp=9.8+.02*i)
            status = c.update(p, 10.2+.02*i, .02)['status']
            self.assertTrue(status['valid'], status)
            self.assertEqual(status['front_startup_distinct_stable_frames'], 1)

    def test_repeated_stream_cannot_refresh_timestamp_or_reverse_sequence(self):
        for name in ('front', 'rear'):
            for change in ({'capture_timestamp': 10.1}, {'sequence': 1}, {'sequence': 20}):
                p = packet(); c = self.controller(); c.update(p, 10.2, .02)
                p['publication_sequence'] += 1; p[name].update(change)
                self.assertFalse(c.update(p, 10.3, .02)['status']['valid'])

    def test_independent_streams_require_front_only_mode(self):
        c = GuardedVD1VisualController([-1.02, 0., -.3])
        self.assertFalse(c.update(packet(), 10.2, .02)['status']['valid'])

    def test_fresh_rear_does_not_renew_front_capture(self):
        p = packet(); c = self.controller(); c.update(p, 10.2, .02)
        p['publication_sequence'] += 1
        p['rear'].update(sequence=8, capture_timestamp=10.9)
        self.assertFalse(packet_status(p, 1, 11.01)['valid'])
        status = c.update(p, 11.01, .02)['status']
        self.assertFalse(status['valid'])
        self.assertIn('front capture is stale', status['reason'])

    def test_front_updates_cannot_requalify_one_rear_outlier(self):
        p = packet(); c = self.controller(); c.update(p, 10.2, .02)
        p['rear'].update(sequence=8, capture_timestamp=9.81)
        for point in p['rear']['points_b']:
            point[1] = .14
        for i in range(4):
            p['publication_sequence'] = i+1
            p['front'].update(sequence=4+i, capture_timestamp=10.2+.02*i)
            status = c.update(p, 10.3+.02*i, .02)['status']
            self.assertFalse(status['rear_valid'])
            self.assertIn('discontinuity', status['rear_reason'])
            self.assertEqual(c._rear_anchor['key'], (1, 7))

    def test_stop_recovery_requires_distinct_captures_from_both_streams(self):
        p = packet(); c = self.controller(); c.update(p, 10.2, .02)
        c._mode = 'stopped'
        for i in range(4):
            p['publication_sequence'] = i+1
            p['front'].update(sequence=4+i, capture_timestamp=10.2+.02*i)
            status = c.update(p, 10.3+.02*i, .02)['status']
            self.assertFalse(status['valid'])
            self.assertEqual(status['recovery_frames'], 1)
        p['publication_sequence'] += 1
        p['rear'].update(sequence=8, capture_timestamp=9.9)
        status = c.update(p, 10.4, .02)['status']
        self.assertEqual(status['recovery_frames'], 2)
        p['publication_sequence'] += 1
        p['front'].update(sequence=8, capture_timestamp=10.4)
        p['rear'].update(sequence=9, capture_timestamp=10.)
        self.assertTrue(c.update(p, 10.5, .02)['status']['valid'])

    def test_start_requires_new_evidence_from_both_cameras(self):
        gate = StableVisualGeometry(); p = packet()
        status = dict(valid=True, rear_valid=True, front_lookahead_b=[1.4, .02, -.55])
        for seq in range(3, 10):
            p['front']['sequence'] = seq
            self.assertFalse(gate.update(p, status))
        self.assertEqual(gate.count, 1)
        p['rear']['sequence'] = 8
        self.assertFalse(gate.update(p, status))
        p['front']['sequence'] = 10; p['rear']['sequence'] = 9
        self.assertTrue(gate.update(p, status))

    def test_front_publishes_while_rear_processing_is_blocked(self):
        front, rear = Capture(), Capture()
        stop, blocked, release = threading.Event(), threading.Event(), threading.Event()
        output = queue.Queue(maxsize=8)
        calls = [0]
        def rear_process(meta, image, depth):
            calls[0] += 1
            if calls[0] > 1:
                blocked.set(); release.wait(3.)
            return dict(valid=True, points_b=[[-1.1, 0., -.55], [-.9, 0., -.55]])
        detector = Mock(process_rear=rear_process)
        detector.segmenter.last_diagnostic_mask = None
        detector.last_filtered_mask = None
        def front_process(detector, meta, *args):
            return dict(valid=True, points_b=[]), meta
        def sample(number):
            return dict(number=number, received=time.monotonic(), image=np.zeros((2, 2, 3)),
                k=np.eye(3), device_time_ms=100., depth_device_time_ms=100.,
                timestamp_domain='device', depth_timestamp_domain='device',
                timestamp_basis='host_receive', depth=np.ones((2, 2)))
        front.put(sample(1)); rear.put(sample(1))
        config = dict(T_base_camera=np.eye(4).tolist())
        writer = Mock(); writer.submit.return_value = True
        failures = Mock(); failures.poll.return_value = False
        decoder = Mock(); decoder.decode.side_effect = lambda value: value
        with tempfile.TemporaryDirectory() as directory, \
                patch('realtime_perception.ObservableRearPerception', return_value=detector), \
                patch('front_projection.process_front', side_effect=front_process):
            thread = threading.Thread(target=run, args=(output, stop, front, rear, decoder,
                config, config, 1, Path(directory), None, .55, Mock(), writer, failures, Mock(), None))
            thread.start()
            try:
                first = output.get(timeout=3.)
                while first.get('kind') != 'packet':
                    first = output.get(timeout=3.)
                rear.put(sample(2)); self.assertTrue(blocked.wait(3.))
                fresh = sample(2); front.put(fresh)
                deadline = time.monotonic()+3.
                while time.monotonic() < deadline:
                    updated = output.get(timeout=3.)
                    if updated.get('front_source_frame') == 2:
                        break
                self.assertEqual(updated['front_source_frame'], 2)
                self.assertEqual(updated['front']['capture_timestamp'], fresh['received'])
                self.assertEqual(updated['rear'], first['rear'])
                self.assertGreater(updated['publication_sequence'], first['publication_sequence'])
            finally:
                stop.set(); release.set(); thread.join(timeout=5.)
                self.assertFalse(thread.is_alive())

    def check_rear_clock_rejection(self, bad_clock):
        front, rear = Capture(), Capture()
        stop = threading.Event()
        output = queue.Queue(maxsize=32)
        def sample(number):
            return dict(number=number, received=time.monotonic(), image=np.zeros((2, 2, 3)),
                k=np.eye(3), device_time_ms=100., depth_device_time_ms=100.,
                timestamp_domain='device', depth_timestamp_domain='device',
                timestamp_basis='host_receive', depth=np.ones((2, 2)))
        detector = Mock()
        detector.process_rear.return_value = dict(valid=True, points_b=[])
        decoder = Mock(); decoder.decode.side_effect = lambda value: value
        writer = Mock(); writer.submit.return_value = True
        failures = Mock(); failures.poll.return_value = False
        viewer = Mock()
        config = dict(T_base_camera=np.eye(4).tolist())
        bad = sample(1); bad.update(bad_clock)
        front.put(sample(1)); rear.put(bad)
        def wait_packet(predicate):
            deadline = time.monotonic()+3.
            while time.monotonic() < deadline:
                value = output.get(timeout=3.)
                if value.get('kind') == 'packet' and predicate(value):
                    return value
            self.fail('no matching packet')
        with tempfile.TemporaryDirectory() as directory, \
                patch('realtime_perception.ObservableRearPerception', return_value=detector), \
                patch('front_projection.process_front',
                      side_effect=lambda detector, meta, *args: (dict(valid=True, points_b=[]), meta)):
            thread = threading.Thread(target=run, args=(output, stop, front, rear, decoder,
                config, config, 1, Path(directory), None, .55, Mock(), writer, failures, viewer, None))
            thread.start()
            try:
                first = wait_packet(lambda p: True)
                self.assertFalse(first['valid'])
                self.assertTrue(first['front']['valid'])
                self.assertEqual(first['rear']['points_b'], [])
                self.assertIn('timestamps differ', first['rear']['reason'])
                self.assertEqual(first['rear']['capture_timestamp'], bad['received'])
                detector.process_rear.assert_not_called()
                # Repeated bad pairs remain invalid, but do not block the front stream.
                bad2 = sample(2); bad2.update(bad_clock); rear.put(bad2)
                second = wait_packet(lambda p: p['rear_source_frame'] == 2)
                self.assertFalse(second['rear']['valid'])
                front.put(sample(2))
                updated = wait_packet(lambda p: p['front_source_frame'] == 2)
                self.assertEqual(updated['rear'], second['rear'])
                self.assertFalse(packet_status(updated, 1, time.monotonic())['valid'])
                good = sample(3); rear.put(good)
                recovered = wait_packet(lambda p: p['rear_source_frame'] == 3)
                self.assertTrue(recovered['valid'])
                self.assertEqual(recovered['rear']['capture_timestamp'], good['received'])
                self.assertEqual(recovered['rear']['sequence'], 2)
                detector.process_rear.assert_called_once()
            finally:
                stop.set(); thread.join(timeout=5.)
                self.assertFalse(thread.is_alive())
            self.assertTrue(viewer.submit.called)
            shown = viewer.submit.call_args_list[0].args
            self.assertEqual(shown[1]['number'], 1)
            self.assertFalse(shown[2]['rear']['valid'])
            self.assertIn('rgb_timestamp_domain', (Path(directory)/'vision.jsonl').read_text())

    def test_rear_clock_domain_mismatch_is_rejected_then_recovers(self):
        self.check_rear_clock_rejection(dict(depth_timestamp_domain='system'))

    def test_rear_timestamp_skew_is_rejected_then_recovers(self):
        self.check_rear_clock_rejection(dict(depth_device_time_ms=141.))


if __name__ == '__main__':
    unittest.main()
