"""CPU tests of real paired image files and private VD1 capture provenance."""

import copy
import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from run_dual_camera_bridge import read_frame
from vd1_frames import FRAME_SCHEMA, SCHEMA, VD1FramePublisher


CALIBRATION = {"position": [-.445, 0., .005], "quaternion_wxyz": [1., 0., 0., 0.],
               "intrinsic_matrix": [[392.5, 0., 6.], [0., 392.5, 4.], [0., 0., 1.]]}


class FramePublisherTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="vd1_frames_test_")
        self.directory = Path(self.temporary.name)
        self.publisher = VD1FramePublisher(self.directory, CALIBRATION, CALIBRATION,
                                            {"marker_radius_m": .035}, generation=17)
        self.rgb = np.zeros((8, 12, 3), dtype=np.uint8)
        self.rgb[..., 0] = 255  # RGB red must survive PNG's BGR encoding.
        self.depth = np.full((8, 12), .6124)

    def tearDown(self):
        self.temporary.cleanup()

    def publish(self, front_frame=1, rear_frame=1, timestamp=123.):
        return self.publisher.publish(self.rgb, self.rgb, self.depth,
                                      front_frame=front_frame, rear_frame=rear_frame,
                                      capture_timestamp=timestamp, sim_time_s=.2)

    def packet(self):
        result = dict(schema=SCHEMA, valid=True, capture_generation=17)
        for stream in ("front", "rear"):
            metadata = json.loads((self.directory / stream / "latest_frame.json").read_text())
            result[stream] = {key: metadata[key] for key in ("sequence", "capture_generation", "capture_timestamp")}
            result[stream]["valid"] = True
        return result

    def test_pair_files_are_readable_by_existing_bridge_and_have_no_world_pose(self):
        self.assertTrue(self.publish())
        front, rgb, depth = read_frame(self.directory, "front", False)
        rear, rear_rgb, rear_depth = read_frame(self.directory, "rear", True)
        self.assertEqual(front["schema"], FRAME_SCHEMA)
        self.assertIsNone(depth)
        np.testing.assert_array_equal(rgb[0, 0], [0, 0, 255])
        np.testing.assert_array_equal(rear_rgb, rgb)
        self.assertEqual(int(rear_depth[0, 0]), 612)
        self.assertEqual(front["sequence"], rear["sequence"])
        self.assertEqual(front["capture_timestamp"], rear["capture_timestamp"])
        self.assertNotIn("base_position_w", front)
        self.assertNotIn("base_quaternion_w", rear)
        self.assertEqual(rear["marker_radius_m"], .035)
        self.assertEqual(self.publisher.validate_packet(self.packet()), (True, ""))

    def test_reused_or_one_sided_frame_does_not_refresh_capture(self):
        self.publish()
        before = (self.directory / "front/latest_frame.json").read_bytes()
        self.assertFalse(self.publish(front_frame=1, rear_frame=2, timestamp=124.))
        self.assertFalse(self.publish(front_frame=2, rear_frame=1, timestamp=125.))
        self.assertFalse(self.publish(timestamp=126.))
        self.assertEqual((self.directory / "front/latest_frame.json").read_bytes(), before)
        self.assertTrue(self.publish(front_frame=2, rear_frame=2, timestamp=127.))
        self.assertEqual(self.packet()["front"]["sequence"], 1)

    def test_forged_capture_identity_is_rejected_using_private_history(self):
        self.publish()
        valid = self.packet()
        for location, field, value in (("front", "sequence", 99),
                                       ("rear", "capture_timestamp", 123.1),
                                       ("rear", "capture_generation", 18),
                                       ("top", "capture_generation", 18)):
            with self.subTest(location=location, field=field):
                forged = copy.deepcopy(valid)
                container = forged if location == "top" else forged[location]
                container[field] = value
                accepted, reason = self.publisher.validate_packet(forged)
                self.assertFalse(accepted)
                self.assertTrue(reason)
        # Editing the public metadata file does not alter private provenance.
        path = self.directory / "rear/latest_frame.json"
        metadata = json.loads(path.read_text())
        metadata["capture_timestamp"] = 999.
        path.write_text(json.dumps(metadata))
        self.assertFalse(self.publisher.validate_packet(self.packet())[0])

    def test_privileged_keys_in_packet_or_calibration_are_rejected(self):
        self.publish()
        for location in ("top", "front", "rear"):
            forged = self.packet()
            container = forged if location == "top" else forged[location]
            container["base_position_w"] = [0, 0, .5]
            accepted, reason = self.publisher.validate_packet(forged)
            self.assertFalse(accepted)
            self.assertIn("privileged", reason)
        bad_calibration = dict(CALIBRATION, base_quaternion_w=[1, 0, 0, 0])
        with self.assertRaisesRegex(ValueError, "privileged"):
            VD1FramePublisher(self.directory, bad_calibration, CALIBRATION, {})

    def test_invalid_depth_pixels_remain_invalid_and_alignment_is_checked(self):
        self.depth[0, :4] = [np.nan, np.inf, -.2, 0.]
        self.publish()
        rear = json.loads((self.directory / "rear/latest_frame.json").read_text())
        measured = cv2.imread(str(self.directory / "rear" / rear["depth_file"]), cv2.IMREAD_UNCHANGED)
        np.testing.assert_array_equal(measured[0, :4], 0)
        with self.assertRaisesRegex(ValueError, "aligned"):
            self.publisher.publish(self.rgb, self.rgb, self.depth[:2], front_frame=2, rear_frame=2,
                                   capture_timestamp=124., sim_time_s=.3)

    def test_cache_is_bounded_and_old_capture_cannot_be_reconstructed_from_files(self):
        self.publish()
        old_packet = self.packet()
        for frame in range(2, 130):
            self.publish(front_frame=frame, rear_frame=frame, timestamp=123. + frame)
        self.assertEqual(len(self.publisher._captures), 128)
        self.assertFalse(self.publisher.validate_packet(old_packet)[0])
        self.assertTrue(self.publisher.validate_packet(self.packet())[0])

    def test_invalid_detector_result_can_keep_valid_capture_provenance(self):
        self.publish()
        observed = self.packet()
        observed["valid"] = False
        observed["rear"]["valid"] = False
        observed["rear"]["reason"] = "TCP marker occluded"
        self.assertEqual(self.publisher.validate_packet(observed), (True, ""))

    def test_zero_tip_extension_is_valid_but_radius_and_spacing_remain_positive(self):
        publisher = VD1FramePublisher(self.directory, CALIBRATION, CALIBRATION,
                                       {"tip_extension_m": 0.}, generation=18)
        self.assertEqual(publisher.marker_calibration["marker_tip_extension_m"], 0.)
        for key in ("radius_m", "axial_spacing_m"):
            with self.subTest(key=key), self.assertRaises(ValueError):
                VD1FramePublisher(self.directory, CALIBRATION, CALIBRATION, {key: 0.})


if __name__ == "__main__":
    unittest.main(verbosity=2)
