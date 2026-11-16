"""Compiled candidate mass/COM/inertia, tube collision and TCP consistency."""

from pathlib import Path
import sys
import tempfile
import unittest

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from vd1_mujoco_scene import build_scene, validate_vd13_model, VD13_URDF_PATH
from vd12_geometry_guard import MujocoGunGeometryGuard


class Mount75CompiledModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cache = tempfile.TemporaryDirectory(prefix="vd13_model_")
        scene = build_scene(
            0.0, cls.cache.name, robot_version="vd13", tool_markers=False
        )
        cls.model = mujoco.MjModel.from_xml_path(str(scene))

    @classmethod
    def tearDownClass(cls):
        cls.cache.cleanup()

    def test_mass_com_full_inertia(self):
        result = validate_vd13_model(self.model)
        self.assertEqual(result["status"], "passed")
        self.assertAlmostEqual(result["gun_mass_kg"], 2.93)

    def test_geometry_guard_endpoint_matches_actual_tcp(self):
        data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, data, self.model.key("vd1_default").id)
        mujoco.mj_forward(self.model, data)
        guard = MujocoGunGeometryGuard(self.model, urdf_path=VD13_URDF_PATH)
        result = guard.evaluate(data)
        ends = np.array(result["tube_endpoints_w_m"])
        self.assertLess(
            np.linalg.norm(
                ends - data.site_xpos[self.model.site("tool_tcp").id], axis=1
            ).min(),
            2e-6,
        )
        self.assertAlmostEqual(np.linalg.norm(ends[1] - ends[0]), 1.1, places=6)

    def test_source_geometry_not_legacy_link06_offset(self):
        np.testing.assert_allclose(
            self.model.site("tool_tcp").pos, [0.075 + 0.75, 0, 0.1], atol=1e-10
        )
        geom = self.model.geom("link06_collision_0")
        np.testing.assert_allclose(geom.pos, [0.275, 0, 0.1], atol=1e-10)
        np.testing.assert_allclose(geom.size[:2], [0.028, 0.55], atol=1e-10)
        self.assertEqual(geom.type, mujoco.mjtGeom.mjGEOM_CYLINDER)


if __name__ == "__main__":
    unittest.main()
