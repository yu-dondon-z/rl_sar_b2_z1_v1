"""Physical contact regressions for VD1; no controller or actor inputs.

Run with the crack_vision Python environment, which provides MuJoCo 3.2.3.
Actual sphere contacts are checked: ray hits alone missed a severe convex-mesh
penetration error in the original very large, thin road prisms.
"""

import tempfile
from pathlib import Path
import sys
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from vd1_mujoco_scene import build_scene, validate_model


class RoadPhysicalContactTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cache = tempfile.TemporaryDirectory(prefix="vd1_road_contact_test_", dir="/dev/shm")
        path = build_scene(0.15, cls.cache.name, timestep=.001)
        cls.robot_model = mujoco.MjModel.from_xml_path(str(path))
        root = ET.parse(str(path)).getroot()
        world = root.find("worldbody")
        world.remove(world.find("body"))
        for tag in ("actuator", "sensor", "keyframe"):
            root.remove(root.find(tag))
        probe = ET.SubElement(world, "body", name="contact_probe", pos=".25 -.2 .03")
        ET.SubElement(probe, "freejoint")
        ET.SubElement(probe, "inertial", pos="0 0 0", mass="1", diaginertia=".001 .001 .001")
        ET.SubElement(probe, "geom", name="probe_sphere", type="sphere", size=".032",
                      contype="2", conaffinity="1")
        cls.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))

    @classmethod
    def tearDownClass(cls):
        cls.cache.cleanup()

    def contacts(self, x, y, z):
        data = mujoco.MjData(self.model)
        data.qpos[:3] = [x, y, z]
        data.qpos[3:7] = [1, 0, 0, 0]
        mujoco.mj_forward(self.model, data)
        return [(float(c.dist), np.array(c.frame[:3])) for c in data.contact[:data.ncon]]

    def test_original_robot_dynamics_still_match(self):
        self.assertEqual(validate_model(self.robot_model)["nu"], 18)

    def test_straight_sphere_contact_depth_and_normal_including_boundaries(self):
        for x, y in ((.25, -.2), (0., -.2), (1., .2)):
            for z in (.03, .032, .04):
                with self.subTest(x=x, y=y, z=z):
                    contacts = self.contacts(x, y, z)
                    if z == .04:
                        self.assertFalse(contacts)
                    else:
                        self.assertTrue(contacts)
                        for distance, normal in contacts:
                            self.assertAlmostEqual(distance, z-.032, places=10)
                            self.assertLess(np.linalg.norm(normal[:2]), 1e-10)

    def test_curved_small_mesh_bounds_numerical_error(self):
        # MuJoCo's MPR reports at most 0.2 mm error / 0.03 lateral normal at
        # these small convex-mesh junctions.  Keep that limitation explicit;
        # the original 97 x 95 m prism was wrong by >100 mm at the same depth.
        for x in (7., 7.025, 7.05):
            with self.subTest(x=x):
                contacts = self.contacts(x, .8, .03)
                self.assertTrue(contacts)
                for distance, normal in contacts:
                    self.assertLess(abs(distance+.002), .00025)
                    self.assertLess(np.linalg.norm(normal[:2]), .035)
                self.assertFalse(self.contacts(x, .8, .04))

    def test_open_gap_has_no_support_at_the_road_surface(self):
        # Probe the real model by a downward collision ray, restricted to the
        # road group.  Actual sphere contacts are independently tested above.
        data = mujoco.MjData(self.model)
        data.qpos[:3] = [5, 5, 5]
        data.qpos[3:7] = [1, 0, 0, 0]
        mujoco.mj_forward(self.model, data)
        groups = np.ones(6, dtype=np.uint8)
        groups[2:4] = 0
        hit = np.zeros(1, dtype=np.int32)
        distance = mujoco.mj_ray(self.model, data, np.array([.25, 0., 1.]),
                                  np.array([0., 0., -1.]), groups, 1, -1, hit)
        self.assertAlmostEqual(1-distance, -.05, places=8)


if __name__ == "__main__":
    unittest.main()
