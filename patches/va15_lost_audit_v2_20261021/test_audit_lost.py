import ast
import math
from pathlib import Path
import unittest
from audit_lost import motor_reasons, summarize


def sample(tick=0, lost=0):
    row = dict(elapsed=tick*.001, low_tick=tick, low_age=0.)
    for i in range(12):
        for name, value in (("q", .1), ("dq", 0.), ("tau", 1.), ("mode", 10), ("lost", lost if i == 1 else 0)):
            row[f"{name}_{i}"] = value
    return row


class AuditTests(unittest.TestCase):
    def test_nonzero_is_still_rejected(self):
        rows = [sample(10*i, 5) for i in range(100)]
        r = summarize(rows, "standing")
        self.assertEqual(r["basic_motor_checks_rejected_rows"], 100)
        self.assertEqual(r["motors"][1]["changed"], 0)
        self.assertEqual(r["motors"][1]["first"], 5)
        self.assertFalse(r["control_authorized"])
        self.assertFalse(r["hardware_qualified"])
        self.assertFalse(r["raw_lost_semantics_verified"])

    def test_changes_and_decreases_not_hidden_by_unsigned_math(self):
        r = summarize([sample(10*i, n) for i, n in enumerate([0, 5, 5, 6, 1])], "prone")
        self.assertEqual((r["motors"][1]["changed"], r["motors"][1]["rises"], r["motors"][1]["decreases_or_wraps"]), (3, 2, 1))
        self.assertEqual(r["motors"][1]["events"][-1]["raw_difference"], -5)

    def test_basic_faults_rejected(self):
        for key, value in (("q_3", math.nan), ("dq_2", 31), ("tau_11", 361), ("mode_4", 0), ("lost_6", 1)):
            row = sample()
            row[key] = value
            self.assertTrue(motor_reasons(row))

    def test_channel_changes_do_not_certify_transport(self):
        rows=[sample(10*i,5) for i in range(3)]
        rows[1]["q_1"]+=.0001
        r=summarize(rows,"prone")
        self.assertEqual(r["motors"][1]["observed_channel_variation_not_freshness_proof"]["q"]["distinct"],2)
        self.assertFalse(r["raw_lost_semantics_verified"])
        self.assertFalse(r["source_tick_alone_proves_each_motor_fresh"])

    def test_no_missing_data_acceptance(self):
        with self.assertRaises(ValueError):
            summarize([], "prone")
        row = sample(100)
        row["low_age"] = .2
        r = summarize([sample(100), row, sample(20)], "unknown")
        self.assertEqual(r["source_age_over_100ms_rows"], 1)
        self.assertEqual(r["tick_duplicate_sample_pairs"], 1)
        self.assertEqual(r["tick_backwards_pairs"], 1)

    def test_allowed_sdk_imports_only(self):
        tree = ast.parse(Path(__file__).with_name("audit_lost.py").read_text())
        names = [item.id for item in ast.walk(tree) if isinstance(item, ast.Name)]
        self.assertNotIn("ChannelPublisher", names)
        imports = [item.module for item in ast.walk(tree) if isinstance(item, ast.ImportFrom) and item.module.startswith("unitree")]
        self.assertEqual(imports, ["unitree_sdk2py.core.channel", "unitree_sdk2py.idl.unitree_go.msg.dds_"])


if __name__ == "__main__":
    unittest.main()
