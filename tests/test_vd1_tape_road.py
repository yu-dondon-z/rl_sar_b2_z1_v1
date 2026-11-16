"""The visual line must not form a trench or introduce tape contact geometry."""
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import cv2
import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from vd1_mujoco_scene import build_scene, validate_model, _road_geometry


class TapeRoadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cache = tempfile.TemporaryDirectory(prefix='vd1_tape_road_', dir='/dev/shm')
        scene = build_scene(.5, cls.cache.name, road_surface='tape', tool_markers=False)
        cls.robot = mujoco.MjModel.from_xml_path(str(scene))
        cls.root = ET.parse(str(scene)).getroot()
        cls.stations = _road_geometry().road_stations(.5)
        ground = ET.fromstring(ET.tostring(cls.root))
        world = ground.find('worldbody')
        world.remove(world.find('body'))
        for tag in ('actuator', 'sensor', 'keyframe'):
            ground.remove(ground.find(tag))
        body = ET.SubElement(world, 'body', name='probe')
        ET.SubElement(body, 'freejoint')
        ET.SubElement(body, 'inertial', pos='0 0 0', mass='1', diaginertia='.001 .001 .001')
        ET.SubElement(body, 'geom', type='sphere', size='.032', mass='1', contype='2', conaffinity='1')
        cls.probe = mujoco.MjModel.from_xml_string(ET.tostring(ground, encoding='unicode'))

    @classmethod
    def tearDownClass(cls):
        cls.cache.cleanup()

    def test_line_is_only_texture_and_ground_has_one_flat_contact(self):
        world = self.root.find('worldbody')
        contacts = [g for g in world.findall('geom') if g.get('contype') != '0']
        self.assertEqual([g.get('name') for g in contacts], ['tape_ground_contact'])
        self.assertEqual(contacts[0].get('type'), 'plane')
        self.assertEqual(contacts[0].get('pos'), '0 0 0')
        self.assertIsNone(world.find("geom[@name='crack_floor']"))
        self.assertFalse(self.root.findall('asset/hfield'))
        self.assertEqual(validate_model(self.robot)['nu'], 18)

    def test_contact_depth_and_normals_on_line_edges_and_both_bends(self):
        data = mujoco.MjData(self.probe)
        for x in (-3., 0., 5., 7., 12., 17., 22., 28.):
            y = np.interp(x, self.stations[:, 0], self.stations[:, 1])
            for offset in (-.1, -.01, 0., .01, .1):
                data.qpos[:3], data.qpos[3:7] = [x, y+offset, .018], [1., 0., 0., 0.]
                mujoco.mj_forward(self.probe, data)
                self.assertEqual(data.ncon, 1)
                self.assertAlmostEqual(data.contact[0].dist, -.014, places=9)
                self.assertLess(np.linalg.norm(data.contact[0].frame[:2]), 1e-12)

    def test_texture_follows_curve_with_two_centimetre_width(self):
        texture = cv2.imread(str(Path(self.cache.name)/'black_tape_curve.png'), cv2.IMREAD_GRAYSCALE)
        height, width = texture.shape
        dy = 6./height
        for x in (0., 5., 7., 12., 17., 22., 24.):
            col = int((x+2)/28*width)
            sample_x = -2+(col+.5)*28/width
            expected_y = np.interp(sample_x, self.stations[:, 0], self.stations[:, 1])
            rows = np.flatnonzero(texture[:, col] < 54)
            self.assertTrue(len(rows))
            measured_y = 3.-(rows.mean()+.5)*dy
            self.assertAlmostEqual(measured_y, expected_y, delta=dy)
            self.assertAlmostEqual(len(rows)*dy, .02, delta=dy)

    def test_visible_ground_continues_flat_beyond_old_six_metre_strip(self):
        data = mujoco.MjData(self.probe)
        mujoco.mj_forward(self.probe, data)
        groups = np.zeros(6, dtype=np.uint8)
        groups[2] = 1  # Only the rendered surfaces, not the invisible contact plane.
        for x in (-20., 7., 17., 35.):
            for y in (-40., -10., -3.01, -2.99, 2.99, 3.01, 10., 40.):
                hit = np.full(1, -1, dtype=np.int32)
                distance = mujoco.mj_ray(self.probe, data, np.array([x, y, 1.]),
                    np.array([0., 0., -1.]), groups, 1, -1, hit)
                self.assertGreaterEqual(hit[0], 0)
                self.assertAlmostEqual(distance, 1., places=10)


if __name__ == '__main__':
    unittest.main()
