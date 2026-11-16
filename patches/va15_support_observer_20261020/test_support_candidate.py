import ast
import copy
import unittest
from pathlib import Path

import numpy as np
from va15_support_candidate import Candidate, ORDER, export_model, load_candidate
from probe_b2_support import assess

URDF = Path(__file__).parent / "../va15_stop_feedback_20261020/board_results/model_sources/board_b2_z1.urdf"


class SupportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = export_model(URDF) if URDF.is_file() else load_candidate(
            Path(__file__).with_name("b2_support_model.json")).model
        cls.candidate = Candidate(cls.model)

    def row(self, force=180.):
        result = dict(low_age=.001, sequence_invalid=False)
        for key, values in (("q", [0., .8, -1.6]*4), ("dq", [0.]*12),
                ("quat", [1.,0.,0.,0.]), ("gyro", [0.]*3), ("accel", [0.,0.,9.81]),
                ("ddq", [0.]*12), ("mode", [10]*12), ("lost", [0]*12)):
            result.update({f"{key}_{i}":v for i,v in enumerate(values)})
        for i, name in enumerate(ORDER):
            q = [result[f"q_{i*3+j}"] for j in range(3)]
            _, jac, g = self.candidate.leg(i, q, np.array([0.,0.,-9.81]))
            tau = g-jac.T@np.array([0.,0.,force])
            result.update({f"tau_{i*3+j}":float(tau[j]) for j in range(3)})
            result["raw_"+name] = 500.
        return result

    def test_upward_sign_and_gravity(self):
        r = self.candidate.evaluate(self.row())
        self.assertTrue(r["quasi_static"])
        np.testing.assert_allclose(r["force_equivalent_world"], [[0.,0.,180.]]*4, atol=1e-10)
        self.assertFalse(r["control_authorized"])

    def test_unloaded_leg_gravity_not_contact(self):
        r = self.candidate.evaluate(self.row(0.))
        np.testing.assert_allclose(r["force_equivalent_world"], 0., atol=1e-10)
        self.assertFalse(r["motion_qualified"])

    def test_downward_force_not_absolute_value(self):
        r = self.candidate.evaluate(self.row(-80.))
        self.assertTrue(all(v[2] < 0. for v in r["force_equivalent_world"]))

    def test_each_unloaded_leg_is_preserved(self):
        for leg in range(4):
            row, unloaded = self.row(), self.row(0.)
            for j in range(3): row[f"tau_{leg*3+j}"] = unloaded[f"tau_{leg*3+j}"]
            result = self.candidate.evaluate(row)
            self.assertAlmostEqual(result["force_equivalent_world"][leg][2], 0., places=8)
            self.assertFalse(result["control_authorized"])

    def test_missing_acceleration_does_not_qualify_old_capture(self):
        row = self.row(); row.pop("accel_2")
        r = self.candidate.evaluate(row)
        self.assertTrue(r["model_valid"])
        self.assertFalse(r["quasi_static"])

    def test_refuse_dynamic_and_reversed_accel(self):
        for key,value in (("dq_0",.3), ("ddq_0",4.), ("gyro_0",.2), ("accel_2",-9.81), ("accel_0",2.)):
            row = self.row(); row[key] = value
            self.assertFalse(self.candidate.evaluate(row)["quasi_static"], key)

    def test_refuse_invalid_inputs(self):
        for key,value in (("q_0",2.), ("tau_0",400.), ("tau_0",float("nan")),
                ("dq_0",float("inf")), ("quat_0",0.), ("low_age",.2), ("low_age",-.1)):
            row = self.row(); row[key] = value
            self.assertFalse(self.candidate.evaluate(row)["model_valid"], key)

    def test_refuse_missing_torque(self):
        row = self.row(); row.pop("tau_0")
        self.assertFalse(self.candidate.evaluate(row)["model_valid"])

    def test_refuse_transport_faults(self):
        for key,value in (("lost_3",1), ("mode_4",0), ("sequence_invalid",True)):
            row = self.row(); row[key] = value
            self.assertFalse(self.candidate.evaluate(row)["quasi_static"])

    def test_reject_singular_straight_knee(self):
        row = self.row()
        # Native knee upper bound is -0.4, so extension outside the URDF is
        # refused before reaching an ill-conditioned solve.
        row["q_2"] = 0.
        self.assertFalse(self.candidate.evaluate(row)["model_valid"])

    def test_model_cannot_enable_authority(self):
        model = copy.deepcopy(self.model); model["control_authorized"] = True
        with self.assertRaises(ValueError): Candidate(model)

    def test_model_order_provenance(self):
        for key, value in (("dds_order", ["FL","FR","RL","RR"]), ("source_urdf_sha256", "")):
            model = copy.deepcopy(self.model); model[key] = value
            with self.assertRaises(ValueError): Candidate(model)

    def test_summary_never_grants_control(self):
        report = assess([self.row()]*3, self.candidate)
        self.assertEqual(report["quasi_static_rows"], 3)
        for key in ("control_authorized","motion_qualified","contact_calibration_generated"):
            self.assertIs(report[key], False)

    def test_probe_has_no_publishers_or_motion_imports(self):
        tree = ast.parse(Path(__file__).with_name("probe_b2_support.py").read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom):
                self.assertNotIn("sport_client", node.module or "")
                self.assertNotIn("motion_switcher", node.module or "")
                self.assertNotIn("z1", node.module or "")
                self.assertNotIn("ChannelPublisher", [n.name for n in node.names])

    def test_serialized_float_counters_roundtrip(self):
        row = {key:str(float(value)) if isinstance(value,(int,float)) else value
               for key,value in self.row().items() if key != "sequence_invalid"}
        row["sequence_invalid"] = "False"
        self.assertTrue(self.candidate.evaluate(row)["quasi_static"])

    def test_reject_fractional_or_negative_motor_counters(self):
        for key,value in (("mode_0","10.5"),("lost_0","-1"),("mode_0","nan")):
            row=self.row();row[key]=value
            self.assertFalse(self.candidate.evaluate(row)["quasi_static"])

    def test_identical_pose_without_torque_does_not_create_support(self):
        row=self.row(0.)
        r=self.candidate.evaluate(row)
        # Same standing geometry and perfect IMU still do not imply contact.
        self.assertGreater(r["height_hypothesis"],.48)
        self.assertLess(max(v[2] for v in r["force_equivalent_world"]),1e-8)
        self.assertIs(r["motion_qualified"],False)


if __name__ == "__main__": unittest.main()
