"""Bounded file-protocol tests for paired capture timing and measured geometry."""

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

import cv2

from run_dual_camera_bridge import SCHEMA, read_frame
from test_dual_camera_perception import synthetic_rgbd


class BridgeTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="va16_bridge_test_")
        self.exchange = Path(self.temporary.name)

    def tearDown(self):
        self.temporary.cleanup()

    def export(self, capture_timestamp):
        rgb, depth, source = synthetic_rgbd()
        for camera in ("front", "rear"):
            directory = self.exchange / camera
            directory.mkdir()
            cv2.imwrite(str(directory / "rgb.png"), rgb)
            metadata = {**source, "capture_timestamp": capture_timestamp,
                        "rgb_file": "rgb.png", "camera_name": camera,
                        "camera_to_base": {"position": [0, 0, 0], "quaternion_wxyz": [0, 1, 0, 0]}}
            metadata.pop("T_base_camera")
            if camera == "rear":
                cv2.imwrite(str(directory / "depth_mm.png"), depth)
                metadata["depth_file"] = "depth_mm.png"
            (directory / "latest_frame.json").write_text(json.dumps(metadata))

    def bridge(self, maximum_age):
        result = subprocess.run([sys.executable, str(Path(__file__).with_name("run_dual_camera_bridge.py")),
            "--exchange-dir", str(self.exchange), "--mode", "classical", "--max-frames", "1",
            "--max-frame-age-s", str(maximum_age), "--save-overlay"],
            text=True, capture_output=True, timeout=15)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        return json.loads((self.exchange / "latest_dual_trajectory.json").read_text())

    def test_front_stream_requires_no_depth_image(self):
        self.export(time.monotonic())
        metadata, rgb, depth = read_frame(self.exchange, "front", False)
        self.assertIsNone(depth)
        self.assertEqual(rgb.shape, (360, 480, 3))
        self.assertEqual(metadata["camera_position_b"], [0, 0, 0])

    def test_paired_packet_preserves_capture_and_measured_tcp_error(self):
        captured = time.monotonic()
        self.export(captured)
        packet = self.bridge(5)
        self.assertTrue(packet["valid"], packet)
        self.assertEqual(packet["schema"], SCHEMA)
        self.assertEqual(packet["front"]["capture_timestamp"], captured)
        self.assertEqual(packet["rear"]["capture_timestamp"], captured)
        self.assertEqual(packet["capture_generation"], 9)
        self.assertAlmostEqual(packet["rear"]["tcp_error_m"], .04, delta=.003)
        self.assertEqual(packet["front"]["ground_plane_b"], packet["rear"]["ground_plane_b"])
        self.assertTrue((self.exchange / "latest_dual_overlay.png").is_file())
        self.assertEqual(len((self.exchange / "dual_vision_packets.jsonl").read_text().splitlines()), 1)

    def test_stale_capture_is_rejected_without_refreshing_capture_time(self):
        captured = time.monotonic() - 5
        self.export(captured)
        packet = self.bridge(1)
        self.assertFalse(packet["valid"])
        self.assertFalse(packet["front"]["valid"])
        self.assertFalse(packet["rear"]["valid"])
        self.assertEqual(packet["rear"]["capture_timestamp"], captured)
        self.assertNotIn("tcp_measured_b", packet["rear"])
        self.assertGreater(packet["capture_age_s"], 5)


if __name__ == "__main__":
    unittest.main(verbosity=2)
