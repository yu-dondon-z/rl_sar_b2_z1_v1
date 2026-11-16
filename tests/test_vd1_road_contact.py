"""Physical surface regressions at actual foot penetration depths."""
from pathlib import Path
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from vd1_mujoco_scene import build_scene, _road_geometry, validate_model
from vd1_mujoco_road_contact import continuous_road_contact


class ContinuousRoadTests(unittest.TestCase):
    curvature = .098

    @classmethod
    def setUpClass(cls):
        cls.cache = tempfile.TemporaryDirectory(prefix='vd1_continuous_contact_', dir='/tmp')
        # .098 also places an outer tile entirely on a shallow lip, covering
        # MuJoCo's per-file height normalization as well as the near-.10 bend.
        original = build_scene(cls.curvature, cls.cache.name, timestep=.005)
        cls.original = mujoco.MjModel.from_xml_path(str(original))
        path = continuous_road_contact(original, cls.curvature)
        cls.robot = mujoco.MjModel.from_xml_path(str(path))
        root = ET.parse(str(path)).getroot()
        world = root.find('worldbody')
        world.remove(world.find('body'))
        for tag in ('actuator', 'sensor', 'keyframe'):
            root.remove(root.find(tag))
        body = ET.SubElement(world, 'body', name='probe', pos='0 0 .03')
        ET.SubElement(body, 'freejoint')
        ET.SubElement(body, 'inertial', pos='0 0 0', mass='1', diaginertia='.001 .001 .001')
        ET.SubElement(body, 'geom', type='sphere', size='.032', mass='1', contype='2', conaffinity='1')
        cls.model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
        cls.stations = _road_geometry().road_stations(cls.curvature)

    @classmethod
    def tearDownClass(cls):
        cls.cache.cleanup()

    def contacts(self, x, y, z):
        data = mujoco.MjData(self.model)
        data.qpos[:3], data.qpos[3:7] = [x, y, z], [1., 0., 0., 0.]
        mujoco.mj_forward(self.model, data)
        return [(float(c.dist), np.array(c.frame[:3])) for c in data.contact[:data.ncon]]

    def test_contact_normals_and_depth_away_from_crack_at_strip_junctions(self):
        # The old 5cm convex strips produced ~0.82 horizontal normals and
        # ~13.6mm depth error at z=.018, even 20cm away from the crack.
        for x in [2.926, 3.074, 4.776, 4.924, 6.626, 6.774, 7., 7.025, 7.05]:
            center = np.interp(x, self.stations[:, 0], self.stations[:, 1])
            for side in [-1, 1]:
                for z in [.018, .025, .030]:
                    with self.subTest(x=x, side=side, z=z):
                        contacts = self.contacts(x, center+side*.2, z)
                        self.assertTrue(contacts)
                        distance, normal = min(contacts, key=lambda c: c[0])
                        self.assertLess(abs(distance-(z-.032)), .0005)
                        self.assertLess(np.linalg.norm(normal[:2]), .15)

    def test_trench_depth_curved_alignment_and_lips_are_preserved(self):
        data = mujoco.MjData(self.model)
        data.qpos[:3], data.qpos[3:7] = [10., 2., 1.], [1., 0., 0., 0.]
        mujoco.mj_forward(self.model, data)
        groups = np.zeros(6, dtype=np.uint8)
        groups[3] = 1  # Only the alternative collision surface.
        # MuJoCo rays skip fully transparent geoms; make only the test model's
        # collision surface opaque for this independent geometry measurement.
        self.model.geom_rgba[self.model.geom_group == 3, 3] = 1.
        # Avoid grid-line ray degeneracies in MuJoCo 3.2.3; collision probes
        # above separately include exact strip junctions.
        for x in [.253, 3.003, 5.003, 7.003, 13.003, 17.003, 21.003]:
            center = np.interp(x, self.stations[:, 0], self.stations[:, 1])
            for offset, expected in [(0., -.05), (-.0125, 0.), (.0125, 0.), (.10, 0.)]:
                hit = np.zeros(1, dtype=np.int32)
                ray = mujoco.mj_ray(self.model, data, np.array([x, center+offset, 1.]),
                                    np.array([0., 0., -1.]), groups, 1, -1, hit)
                self.assertAlmostEqual(1-ray, expected, delta=.0001,
                                       msg='x=%s offset=%s curvature=%s' % (x, offset, self.curvature))

    def test_robot_dynamics_and_rendered_road_geometry_are_unchanged(self):
        validate_model(self.robot)
        for field in ('body_mass', 'body_inertia', 'jnt_range', 'actuator_gainprm', 'actuator_biasprm'):
            np.testing.assert_array_equal(getattr(self.robot, field), getattr(self.original, field))
        for geom in range(self.original.ngeom):
            name = mujoco.mj_id2name(self.original, mujoco.mjtObj.mjOBJ_GEOM, geom)
            if not name or not name.startswith('road_'):
                continue
            other = self.robot.geom(name).id
            for field in ('geom_type', 'geom_size', 'geom_pos', 'geom_quat', 'geom_rgba'):
                np.testing.assert_array_equal(getattr(self.original, field)[geom], getattr(self.robot, field)[other])
            self.assertEqual(self.robot.geom_contype[other], 0)

    def test_compiler_normalization_does_not_deepen_shallow_outer_tiles(self):
        # Per-file normalization must retain absolute heights. This includes
        # outer lip tiles whose lowest sample is above the trench floor.
        hfields = self.robot.hfield_data
        shallow_tiles = 0
        for gid in range(self.robot.ngeom):
            if self.robot.geom_type[gid] != mujoco.mjtGeom.mjGEOM_HFIELD:
                continue
            field = self.robot.geom_dataid[gid]
            nr, nc = self.robot.hfield_nrow[field], self.robot.hfield_ncol[field]
            start = self.robot.hfield_adr[field]
            raw = hfields[start:start+nr*nc].reshape(nr, nc)
            x = np.linspace(-25.,35.,nc)
            center = np.interp(x,self.stations[:,0],self.stations[:,1])
            ycenter = self.robot.geom_pos[gid,1]
            y = np.linspace(ycenter-.005,ycenter+.005,nr)
            expected = .05*np.clip((np.abs(y[:,None]-center[None,:])-.0075)/.0025,0,1)-.05
            compiled = self.robot.geom_pos[gid,2]+self.robot.hfield_size[field,2]*raw
            np.testing.assert_allclose(compiled,expected,atol=1e-7,rtol=0)
            shallow_tiles += int(expected.min()>-.0499)
        self.assertGreater(shallow_tiles,0)


class SharpContinuousRoadTests(ContinuousRoadTests):
    # Recheck real compiled contact normals, depth, lips and render/dynamics
    # at .40/m. The +/-1.351m crests must not be filled by outer flat boxes.
    curvature = .40


if __name__ == '__main__':
    unittest.main()
