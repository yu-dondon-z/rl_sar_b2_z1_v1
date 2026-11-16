"""Sensor-only transport, time provenance, kinematics and nonphysical sites."""
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
sys.path.insert(0, '/home/dong/b2_z1/scripts/vision')
from vd14_motion_compensation import (LegKinematics, SensorMotionHistory, FootLoadSensors,
    exp_rotation, matrix_quaternion, MotionCompensatedVisualController, SensorRearPathMemory)
from vd5_mujoco_control import quaternion_matrix, quaternion_from_euler_xyz
from test_vd12_vision_entry import packet


class SyntheticLegs:
    def __init__(self, velocity=(0.,0.,0.)):
        self.velocity = np.array(velocity, dtype=float)

    def evaluate(self, q, qd):
        return (np.array([[.4,.25,-.5],[.4,-.25,-.5],[-.4,.25,-.5],[-.4,-.25,-.5]]),
                np.tile(-self.velocity, (4,1)), np.zeros((4,3)))


def proprio(gravity=(0.,0.,-1.), gyro=(0.,0.,0.)):
    return dict(gravity_b=np.array(gravity), gyro_b=np.array(gyro), q=np.zeros(18), qd=np.zeros(18),
                accelerometer_b_m_s2=-9.81*np.array(gravity))


class SensorTransport(unittest.TestCase):
    def history(self, velocity=(0.,0.,0.), steps=40):
        h = SensorMotionHistory(SyntheticLegs(velocity))
        for i in range(steps+1):
            h.sample(i*.005, proprio(), [100.,100.,100.,100.])
        return h

    def test_local_origin_and_encoder_velocity_integration(self):
        h = self.history((.1, .03, .01))
        r,p = h.pose_at(.2)
        np.testing.assert_allclose(r,np.eye(3))
        np.testing.assert_allclose(p,[.02,.006,.002],atol=1.e-12)
        self.assertFalse(h.diagnostic['simulator_pose_input'])
        self.assertFalse(h.diagnostic['sent_command_input'])

    def test_rigid_points_and_plane_transform_preserves_distances(self):
        h = self.history((.1,.03,.01))
        image = packet(stamp=10.)
        for stream in ('front','rear'):
            image[stream]['capture_sim_time_s']=0.
        original=deepcopy(image)
        out = h.transport(image,.2)
        np.testing.assert_allclose(np.asarray(out['rear']['points_b']),
                                   np.asarray(image['rear']['points_b'])-[.02,.006,.002])
        np.testing.assert_allclose(out['rear']['ground_plane_b'], [0.,0.,1.,.502])
        self.assertEqual(image,original)
        self.assertEqual(out['rear']['capture_timestamp'],10.)
        self.assertEqual(out['rear']['sequence'],0)

    def test_rotation_and_quaternion_sign_convention(self):
        for vector in ([.1,.2,.3],[-.2,0.,0.],[0.,np.pi,0.],[0.,0.,0.]):
            r=exp_rotation(np.array(vector))
            np.testing.assert_allclose(quaternion_matrix(matrix_quaternion(r)),r,atol=1.e-12)
        omega=np.array([0.,.4,0.])
        class RotatingLegs(SyntheticLegs):
            def evaluate(self,q,qd):
                feet,_,_=super().evaluate(q,qd)
                return feet,-np.cross(omega,feet),np.tile(-omega,(4,1))
        h=SensorMotionHistory(RotatingLegs())
        for i in range(41):
            r=exp_rotation(omega*i*.005)
            h.sample(i*.005,proprio(r.T @ np.array([0.,0.,-1.]),omega),[100.]*4)
        np.testing.assert_allclose(h.pose_at(.2)[0],exp_rotation(omega*.2),atol=1.e-12)

    def test_missing_reversed_or_future_sensor_clocks_rejected(self):
        h=self.history()
        for cap,now in ((.001,.2),(.2,.1),(-.005,.2),(0.,.201)):
            with self.subTest(cap=cap,now=now),self.assertRaises(ValueError):
                h.relative(cap,now)
        for stamp in (.2,.199,.22):
            with self.subTest(stamp=stamp),self.assertRaises(ValueError):
                h.sample(stamp,proprio(),[100.]*4)

    def test_load_loss_prediction_is_bounded_and_not_fabricated_contact(self):
        h=self.history(steps=0)
        for i in range(1,11):
            h.sample(i*.005,proprio(),[0.]*4)
        self.assertEqual(h.diagnostic['support_sensor_count'],0)
        self.assertTrue(h.diagnostic['short_velocity_prediction'])
        with self.assertRaises(ValueError):
            h.sample(.055,proprio(),[0.]*4)
        with self.assertRaises(ValueError):
            self.history().sample(.205,proprio(),[float('nan')]*4)

    def test_repeated_raw_image_can_move_without_renewing_or_mutating_it(self):
        h=self.history((.2,0.,0.),steps=0)
        c=MotionCompensatedVisualController([-.95,0.,.35],
            quaternion_from_euler_xyz(0.,.3,np.pi),motion_history=h,stale_timeout_s=.15)
        image=packet(stamp=10.)
        for name in ('front','rear'): image[name]['capture_sim_time_s']=0.
        first=c.update(image,10.,.02,control_time_s=0.)
        for i in range(1,21):
            h.sample(i*.005,proprio(),[100.]*4)
            if i % 4 == 0:
                repeat=c.update(image,10.+i*.005,.02,control_time_s=i*.005)
        self.assertTrue(first['status']['valid'],first['status']['reason'])
        self.assertTrue(repeat['status']['valid'],repeat['status']['reason'])
        self.assertFalse(repeat['status']['measurement_is_new'])
        self.assertAlmostEqual(repeat['status']['capture_age_s'],.1)
        self.assertEqual(repeat['status']['capture_timestamps'],[10.,10.])
        for i in range(21,25): h.sample(i*.005,proprio(),[100.]*4)
        changed=deepcopy(image);changed['rear']['points_b'][0][1]=.001
        rejected=c.update(changed,10.12,.02,control_time_s=.12)
        self.assertFalse(rejected['status']['valid'])
        self.assertIn('changed its image-derived geometry',rejected['status']['reason'])
        self.assertFalse(c.update(image,10.12,.02,control_time_s=.1)['status']['valid'])

    def test_motion_does_not_make_stale_frames_fresh(self):
        h=self.history()
        c=MotionCompensatedVisualController([-.95,0.,.35],
            quaternion_from_euler_xyz(0.,.3,np.pi),motion_history=h,stale_timeout_s=.15)
        image=packet(stamp=10.)
        for name in ('front','rear'): image[name]['capture_sim_time_s']=0.
        result=c.update(image,10.2,.02,control_time_s=.2)
        self.assertFalse(result['status']['valid'])
        self.assertIn('stale',result['status']['reason'])

    def test_compensation_cannot_accept_privileged_metadata(self):
        h=self.history(steps=0)
        for key in ('world_pose','actual_tcp','ground_truth'):
            c=MotionCompensatedVisualController([-.95,0.,.35],
                quaternion_from_euler_xyz(0.,.3,np.pi),motion_history=h)
            image=packet(stamp=10.)
            for name in ('front','rear'): image[name]['capture_sim_time_s']=0.
            image['rear']['metadata']={key:[0.,0.,0.]}
            result=c.update(image,10.,.02,control_time_s=0.)
            self.assertFalse(result['status']['valid'])
            self.assertIn('privileged',result['status']['reason'])

    def test_prediction_bound_cannot_be_loosened(self):
        for bound in (0.,.051,float('nan'),float('inf')):
            with self.assertRaises(ValueError):
                SensorMotionHistory(SyntheticLegs(),max_prediction_s=bound)

    def test_initial_no_load_uses_labelled_inertial_prior_not_fake_support(self):
        h=SensorMotionHistory(SyntheticLegs())
        s=proprio();s['accelerometer_b_m_s2']=np.zeros(3)
        h.sample(0.,s,[0.]*4)
        h.sample(.005,s,[0.]*4)
        self.assertEqual(h.diagnostic['support_sensor_count'],0)
        self.assertAlmostEqual(h.diagnostic['estimated_velocity_b_m_s'][2],-9.81*.005)
        self.assertIn('NOT_measured',h.diagnostic['initialization'])

    def test_cache_uses_sensor_pose_not_sent_command(self):
        h=self.history(steps=0)
        memory=SensorRearPathMemory(h,.6)
        from vd5_visual_control import _ground_basis
        plane=np.array([0.,0.,1.,.5]);basis=_ground_basis(plane)
        points=np.c_[np.linspace(-1.2,-.5,100),np.zeros(100),np.full(100,-.5)]
        memory.ingest(points,plane,basis,1,0,10.,10.)
        memory.advance(10.1,.1,[100.,100.,100.])
        paths=memory.paths(plane,basis,10.1)
        self.assertTrue(paths)
        self.assertAlmostEqual(paths[0][1]['rear_memory_sensor_displacement_max_m'],0.)
        self.assertAlmostEqual(paths[0][1]['rear_memory_source_age_max_s'],.1)
        self.assertIn('leg_IMU',paths[0][1]['rear_memory_motion_source'])


class NamedMotionSensorBoundary(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import mujoco
        from vd1_mujoco_scene import build_scene,VD13_URDF_PATH
        from vd1_mujoco_contract import ACTOR_JOINT_NAMES,VD1MujocoAdapter
        cls.mujoco=mujoco
        cls.cache=tempfile.TemporaryDirectory(prefix='vd14_sensors_')
        scene=build_scene(0.,cls.cache.name,robot_version='vd13',odometry_sensors=True)
        cls.model=mujoco.MjModel.from_xml_path(str(scene))
        cls.adapter=VD1MujocoAdapter(cls.model)
        cls.kinematics=LegKinematics(VD13_URDF_PATH,ACTOR_JOINT_NAMES)

    @classmethod
    def tearDownClass(cls):
        cls.cache.cleanup()

    def test_encoder_fk_and_jacobian_match_independent_engine(self):
        m=self.model;d=self.mujoco.MjData(m)
        q=self.adapter.default_q18.copy()
        ids=m.jnt_qposadr[self.adapter.joint_ids18]
        d.qpos[ids]=q;d.qpos[3:7]=[1.,0.,0.,0.]
        self.mujoco.mj_forward(m,d)
        qd=np.linspace(-.3,.5,18)
        feet,velocity,omega=self.kinematics.evaluate(q,qd)
        for i,name in enumerate(('FL','FR','RL','RR')):
            np.testing.assert_allclose(feet[i],d.xpos[m.body(name+'_foot').id]-d.xpos[m.body('base_link').id],atol=1.e-12)
        shifted=self.kinematics.evaluate(q+qd*1.e-7,qd)[0]
        np.testing.assert_allclose((shifted-feet)/1.e-7,velocity,atol=1.e-7)

    def test_touch_reads_only_sensor_buffer_and_measures_real_contact(self):
        m=self.model;d=self.mujoco.MjData(m)
        self.mujoco.mj_resetDataKeyframe(m,d,m.key('vd1_default').id)
        d.qpos[2]=.45;self.mujoco.mj_forward(m,d)
        sensor=FootLoadSensors(m)
        load=sensor.read(d)
        self.assertGreater(float(load.sum()),0.)
        class SensorOnly:
            sensordata=d.sensordata
            def __getattr__(self,name): raise AssertionError('Forbidden simulator state '+name)
        np.testing.assert_array_equal(sensor.read(SensorOnly()),load)

    def test_added_sensing_sites_do_not_change_mass_or_dynamics(self):
        from vd1_mujoco_scene import build_scene,validate_vd13_model
        scene=build_scene(0.,self.cache.name,robot_version='vd13',odometry_sensors=False)
        original=self.mujoco.MjModel.from_xml_path(str(scene))
        for name in ('body_mass','body_inertia','body_ipos','geom_size','geom_contype','geom_conaffinity',
                     'actuator_gainprm','actuator_biasprm','dof_damping','dof_frictionloss'):
            np.testing.assert_array_equal(getattr(original,name),getattr(self.model,name))
        self.assertEqual(validate_vd13_model(self.model)['status'],'passed')


if __name__=='__main__': unittest.main()
