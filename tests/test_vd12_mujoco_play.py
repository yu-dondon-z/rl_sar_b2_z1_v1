import copy
import importlib.util
from pathlib import Path
import sys
import unittest
import contextlib
import io
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'scripts'))
import play_vd12_mujoco as play

class Contracts(unittest.TestCase):
    def fixture(self):
        profile={'version':1,'hash':'test'}
        checkpoint=dict(training_experiment_name='b2_z1_vd12_geometry_guard_amp',iter=5999,vd12_geometry_profile=profile,
            vd5_curriculum_state=dict(version=1,startup_profile=copy.deepcopy(play.shared.PROFILE),ground_blend=1.,ground_transition_steps=20000,seam_trailing_distance_m=.8))
        agent=dict(experiment_name=checkpoint['training_experiment_name'],empirical_normalization=False,clip_actions=None,
            policy=dict(activation='elu',actor_hidden_dims=[512,256,128]))
        return checkpoint,agent,dict(vd12_geometry_profile=profile),profile
    def test_valid(self):
        play.validate_checkpoint(*self.fixture())
    def test_vd11_rejected(self):
        values=list(self.fixture()); values[0]['training_experiment_name']='b2_z1_vd11_motion_quality_amp'
        with self.assertRaises(ValueError): play.validate_checkpoint(*values)
    def test_profile_mismatch(self):
        values=list(self.fixture()); values[3]={'version':1,'different':True}
        with self.assertRaises(ValueError): play.validate_checkpoint(*values)
    def test_wrong_startup(self):
        values=list(self.fixture()); values[0]['vd5_curriculum_state']['startup_profile']['duration_s']=1
        with self.assertRaises(ValueError): play.validate_checkpoint(*values)
    def test_incomplete_curriculum(self):
        values=list(self.fixture()); values[0]['vd5_curriculum_state']['ground_blend']=.9
        with self.assertRaises(ValueError): play.validate_checkpoint(*values)
    def test_norm_rejected(self):
        values=list(self.fixture()); values[1]['empirical_normalization']=True
        with self.assertRaises(ValueError): play.validate_checkpoint(*values)
    def test_latch_survives_safe_sample(self):
        latch=play.GeometryLatch()
        self.assertFalse(latch.sample(dict(conservative_keepout_violated=True),1))
        for tick in (2,3): self.assertFalse(latch.sample(dict(conservative_keepout_violated=False),tick))
        self.assertTrue(latch.sample(dict(conservative_keepout_violated=False),4))
        self.assertEqual(latch.first['physics_step'],1)
    def test_reset_bad(self):
        self.assertTrue(play.GeometryLatch().sample(dict(conservative_keepout_violated=True),0))
    def test_defaults(self):
        args=play.parse([])
        self.assertEqual(args.checkpoint.name,'model_5999.pt')
        self.assertFalse(args.viewer)
    def test_saved_settings_real_snapshot(self):
        saved=play.load_yaml(play.DEFAULT_CHECKPOINT.parent/'params/env.yaml')
        result=play.validate_saved_settings(saved)
        self.assertTrue(result['clock_checked'])
        saved['sim']['dt']=.001
        with self.assertRaises(ValueError): play.validate_saved_settings(saved)
    def test_saved_camera_rejected(self):
        saved=play.load_yaml(play.DEFAULT_CHECKPOINT.parent/'params/env.yaml')
        saved['commands']['base_velocity']['lookahead_distance']=.8
        with self.assertRaises(ValueError): play.validate_saved_settings(saved)
    def test_mutated_supported_settings_rejected(self):
        original=play.load_yaml(play.DEFAULT_CHECKPOINT.parent/'params/env.yaml')
        mutations=[
            (('episode_length_s',),.01),
            (('actions','JointPositionAction','offset'),1.),
            (('actions','JointPositionAction','clip'),{'.*':[-1.,1.]}),
            (('scene','robot','init_state','pos'),[0.,0.,.6]),
            (('scene','robot','init_state','joint_vel'),{'.*':.1}),
            (('scene','robot','init_state','joint_pos','joint2'),.5),
        ]
        for group in original['scene']['robot']['actuators']:
            for field,value in [('stiffness',100.),('damping',4.),('effort_limit_sim',100.),('velocity_limit_sim',1.),('friction',.02),('armature',.01)]:
                mutations.append((('scene','robot','actuators',group,field),value))
        for term in ('velocity_commands','target_commands','base_ang_vel','projected_gravity','joint_pos_rel','joint_vel_rel','actions'):
            mutations.extend([(('observations','policy',term,'scale'),2.),(('observations','policy',term,'clip'),[-.1,.1])])
        for path,value in mutations:
            with self.subTest(path=path):
                changed=copy.deepcopy(original)
                node=changed
                for key in path[:-1]: node=node[key]
                node[path[-1]]=value
                with self.assertRaises(ValueError): play.validate_saved_settings(changed)
    def test_duration_control_grid(self):
        for duration in ('0','.00001','.005','.019','.021','30.02','nan','inf'):
            with self.subTest(duration=duration),contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit): play.parse(['--duration',duration])
        for duration in ('.02','.06','30'):
            self.assertEqual(play.parse(['--duration',duration]).duration,float(duration))
    def test_processed_target_clip_not_raw_actor_clip(self):
        default=np.linspace(-1,1,18)
        action=np.linspace(-1000,1000,18)
        np.testing.assert_allclose(play.processed_joint_target(default,action),np.clip(default+.25*action,-100,100))
        moderate=np.full(18,200.)
        np.testing.assert_allclose(play.processed_joint_target(default,moderate),default+50.)

if __name__=='__main__': unittest.main()
