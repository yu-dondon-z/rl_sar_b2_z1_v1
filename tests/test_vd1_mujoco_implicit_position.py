"""Analytic one-step tests of implicit position stiffness discretization.

PhysX force drive: implicitDrives.pdf, 2024-04-23, equation denominator
1 + inverse_inertia*(h*kd + h*h*kp). MuJoCo 3.2.3 implicitfast linearizes
velocity only. The kv=kd+h*kp conversion is exact here for an unsaturated
linear hinge; these tests make no whole-articulation equivalence claim.
"""

from pathlib import Path
import sys
import tempfile
import unittest

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from vd1_mujoco_contract import VD1MujocoAdapter
from vd1_mujoco_scene import build_scene


class ImplicitPositionTests(unittest.TestCase):
    def test_native_kv5_differs_and_converted_kv_matches_analytic_implicit_euler(self):
        for h in (.001, .005):
            for inertia in (.004, .08):
                with self.subTest(h=h, inertia=inertia):
                    q, v, target, kp, kd = .1, .2, .2, 160., 5.
                    analytic_v = (inertia*v+h*kp*(target-q))/(inertia+h*kd+h*h*kp)
                    values = []
                    for kv in (kd, kd+h*kp):
                        xml = ('<mujoco><option timestep="%s" gravity="0 0 0" integrator="implicitfast"/>'
                               '<worldbody><body><joint name="j" axis="0 0 1"/>'
                               '<inertial pos="0 0 0" mass="1" diaginertia="%s %s %s"/>'
                               '<geom type="sphere" size=".1" mass="0"/></body></worldbody>'
                               '<actuator><position joint="j" kp="160" kv="%s"/></actuator></mujoco>'
                               % (h, inertia, inertia, inertia, kv))
                        model = mujoco.MjModel.from_xml_string(xml)
                        data = mujoco.MjData(model)
                        data.qpos[0], data.qvel[0], data.ctrl[0] = q, v, target
                        mujoco.mj_step(model, data)
                        values.append(float(data.qvel[0]))
                        if kv != kd:
                            self.assertAlmostEqual(data.qvel[0], analytic_v, places=12)
                            self.assertAlmostEqual(data.qpos[0], q+h*analytic_v, places=12)
                    self.assertGreater(abs(values[0]-analytic_v), 1e-5)

    def test_adapter_requires_source_gains_before_explicit_discrete_conversion(self):
        with tempfile.TemporaryDirectory(prefix="vd1_implicit_test_", dir="/dev/shm") as cache:
            path = build_scene(0., cache, timestep=.005)
            model = mujoco.MjModel.from_xml_path(str(path))
            baseline = VD1MujocoAdapter(model)
            gains, bias = model.actuator_gainprm.copy(), model.actuator_biasprm.copy()
            adapted = VD1MujocoAdapter(model, physx_implicit_position=True)
            np.testing.assert_array_equal(model.actuator_gainprm, gains)
            arms = adapted.actuator_ids18[adapted.arm_mask18]
            legs = adapted.actuator_ids18[~adapted.arm_mask18]
            np.testing.assert_allclose(model.actuator_biasprm[arms, 2], -5.8, atol=0)
            np.testing.assert_array_equal(model.actuator_biasprm[legs], bias[legs])
            np.testing.assert_array_equal(adapted.source_kp18, np.full(18, 160.))
            np.testing.assert_array_equal(adapted.source_kd18, np.full(18, 5.))
            np.testing.assert_array_equal(adapted.default_q18, baseline.default_q18)
            self.assertTrue(adapted.physx_implicit_position)
            # A previously modified model cannot silently pass source validation.
            with self.assertRaisesRegex(ValueError, "PD contract"):
                VD1MujocoAdapter(model, physx_implicit_position=True)


if __name__ == "__main__":
    unittest.main()
