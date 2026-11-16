import ast
import math
from pathlib import Path
import unittest
from sample_b2_stop_feedback import describe, summarize


class ReadOnlyCaptureTests(unittest.TestCase):
    def test_nan_does_not_become_zero(self):
        self.assertEqual(describe([math.nan, math.inf]), {"count": 0})
        self.assertEqual(describe([1, math.nan, 3])["median"], 2)

    def test_no_auto_calibration(self):
        row = {"raw_FR": 101, "raw_FL": 202, "raw_RR": 303, "raw_RL": 404,
               "sport_height": .52, "sport_speed": 0, "sport_age": .01, "sport_error": 0}
        result = summarize([row], "custom_stand")
        self.assertFalse(result["calibration_generated"])
        self.assertFalse(result["motion_qualified"])
        self.assertEqual(result["raw_bridge"]["RL"]["median"], 404)
        self.assertEqual(result["fresh_sport_rows"], 1)

    def test_zero_stamp_arrival_does_not_prove_source_freshness(self):
        row = {"raw_FR": 1, "raw_FL": 2, "raw_RR": 3, "raw_RL": 4,
               "sport_height": .52, "sport_speed": 0, "sport_age": math.inf,
               "sport_receive_age": .01, "sport_error": 0}
        result = summarize([row], "custom_stand")
        self.assertEqual(result["fresh_sport_rows"], 0)
        self.assertEqual(result["fresh_sport_receive_rows"], 1)
        self.assertFalse(result["motion_qualified"])

    def test_only_subscriber_sdk_imports(self):
        tree = ast.parse(Path(__file__).with_name("sample_b2_stop_feedback.py").read_text())
        imports = [n for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and
                   n.module and n.module.startswith("unitree_sdk2py")]
        imported = {item.name for node in imports for item in node.names}
        self.assertEqual(imported, {"ChannelFactoryInitialize", "ChannelSubscriber", "LowState_", "SportModeState_"})


if __name__ == "__main__":
    unittest.main()
