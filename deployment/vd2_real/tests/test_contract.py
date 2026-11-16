import ctypes as ct
import json
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np
import torch
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'python'),str(ROOT/'python/vision'),str(ROOT/'thirdparty/rhacracknet')]
from native_io import State,library,Command
from control import Actor,Session,Remote,DEFAULT,STAND,ARM,LEGS,A,B,X,Y,R1,L1,limits
from vd1_control import build_actor_observation,_tool_quaternion
from cameras import calibration,paired_metadata
from dual_vision_kinematics import URDFToolKinematics
from run import packet_ok

class ContractTests(unittest.TestCase):
    def state(self):
        s=State();s.q[:]=DEFAULT;s.quaternion[0]=1
        return s

    def test_policy_uses_raw_history_and_exact_observation(self):
        torch.set_num_threads(1);actor=Actor();state=self.state()
        state.dq[:]=np.linspace(-1,1,18);state.omega[:]=[.2,-.3,.4]
        tool=np.r_[[-1.02,.02,-.30],_tool_quaternion()]
        raw=np.linspace(-.7,.5,18);actor.last=raw.copy()
        target,detail=actor.step(state,[.2,0,.08],tool)
        expected=np.r_[[.2,0,.08],tool,[.04,-.06,.08],[0,0,-1],np.zeros(18),np.linspace(-1,1,18)*.05,raw].astype(np.float32)
        np.testing.assert_allclose(detail['observation'],expected,atol=1e-7)
        with torch.inference_mode():expected_action=actor.model(torch.from_numpy(expected)[None]).numpy()[0]
        np.testing.assert_array_equal(actor.last,expected_action)
        bounds=limits()
        np.testing.assert_allclose(target,np.clip(DEFAULT+.25*expected_action,bounds[:,0],bounds[:,1]),atol=1e-7)

    def test_native_rejects_bad_target_without_opening_transports(self):
        lib=library();self.assertEqual(lib.vd2_offline_check(),0)
        command=Command((ct.c_double*18)(*DEFAULT),1.,1)
        self.assertEqual(lib.vd2_command(ct.byref(command)),0)
        command.q[13]=-1
        self.assertNotEqual(lib.vd2_command(ct.byref(command)),0)
        command.q[13]=float('nan')
        self.assertNotEqual(lib.vd2_command(ct.byref(command)),0)

    def test_reading_native_snapshot_matches_ctypes_without_io(self):
        lib=library();s=State();self.assertEqual(lib.vd2_snapshot(ct.byref(s)),0)
        self.assertEqual(s.b2_time,0);self.assertEqual(s.z1_time,0);self.assertEqual(s.fault,0)
        loaded=Path('/proc/self/maps').read_text()
        dds_paths=[line.split()[-1] for line in loaded.splitlines() if '/libddsc.so' in line]
        self.assertTrue(dds_paths)
        self.assertTrue(all(str(ROOT/'thirdparty/unitree_sdk2') in path for path in dds_paths))

    def test_button_priorities_and_no_held_start(self):
        remote=Remote();self.assertIsNone(remote.event(A));self.assertIsNone(remote.event(A))
        remote.event(0);self.assertEqual(remote.event(A),'stand')
        remote.event(0);self.assertEqual(remote.event(R1|Y),'track')
        self.assertIsNone(remote.event(R1|Y))
        self.assertEqual(remote.event(R1|Y|B),'down')
        self.assertEqual(remote.event(L1|X|B),'emergency')

    def test_stopping_returns_whole_stance_then_b_is_preserved(self):
        session=Session();session.mode='tracking';session.prone=DEFAULT.copy();session.prone[LEGS]=np.tile([0],12)
        session.prone[ARM]=0
        session.target=DEFAULT+np.linspace(-.03,.03,18)
        session.brake_velocity=np.array([.2,0,.1])
        session.event('down',0,DEFAULT);self.assertEqual(session.mode,'braking')
        session.advance(1.1,DEFAULT,np.zeros(18),np.zeros(3),np.array([0,0,-1]))
        for i in range(1,300):
            now=1.1+i*.02
            session.advance(now,session.target.copy(),np.zeros(18),np.zeros(3),np.array([0,0,-1]))
            if session.mode=='return_stand':break
        self.assertEqual(session.mode,'return_stand');self.assertTrue(session.pending_down)
        now=session.entered+3.1
        session.advance(now,STAND,np.zeros(18),np.zeros(3),np.array([0,0,-1]))
        self.assertEqual(session.mode,'arm_lower')
        np.testing.assert_allclose(session.end[LEGS],STAND[LEGS])

    def test_stop_is_latched_when_vision_returns(self):
        s=Session();s.mode='tracking';s.prone=DEFAULT.copy();s.event('stop',10,DEFAULT)
        for now in (10.1,10.2):
            s.event(None,now,DEFAULT,visual_valid=True)
            self.assertEqual(s.mode,'braking')

    def test_entire_preparation_and_get_down_cycle_uses_encoder_arrival(self):
        s=Session();prone=DEFAULT.copy();prone[[9,10,11,12]]=-2.6;prone[ARM]=0
        s.event('stand',0,prone)
        s.advance(1.,prone,np.zeros(18),np.zeros(3),np.array([0,0,-1]))
        s.advance(3.,prone,np.zeros(18),np.zeros(3),np.array([0,0,-1]))
        self.assertEqual(s.mode,'stand_settle') # elapsed time alone is insufficient
        q=s.end.copy()
        for now in (4.,4.5,5.):s.advance(now,q,np.zeros(18),np.zeros(3),np.array([0,0,-1]))
        self.assertEqual(s.mode,'standing')
        s.event('prepare',5.1,q)
        q=s.end.copy();s.advance(11.2,q,np.zeros(18),np.zeros(3),np.array([0,0,-1]))
        self.assertEqual(s.mode,'ready')
        s.event('track',11.3,DEFAULT,visual_valid=False);self.assertEqual(s.mode,'ready')
        s.event('down',11.4,DEFAULT);self.assertEqual(s.mode,'arm_lower')
        q=s.end.copy();s.advance(17.5,q,np.zeros(18),np.zeros(3),np.array([0,0,-1]))
        self.assertEqual(s.mode,'lowering')
        s.advance(22.6,prone,np.zeros(18),np.zeros(3),np.array([0,0,-1]))
        self.assertEqual(s.mode,'idle');np.testing.assert_allclose(s.target,prone)

    def test_moving_encoders_cannot_be_declared_stopped(self):
        s=Session();s.mode='tracking';s.event('stop',0,DEFAULT)
        s.advance(6,DEFAULT,np.ones(18),np.zeros(3),np.array([0,0,-1]))
        self.assertEqual(s.mode,'fault')

    def test_encoder_fk_is_not_a_simulator_link_pose(self):
        kinematics=URDFToolKinematics(ROOT/'assets/encoder_fk.urdf')
        result=kinematics.position(torch.tensor(DEFAULT[ARM],dtype=torch.float32)[None])
        self.assertEqual(tuple(result.shape),(1,3));self.assertTrue(torch.isfinite(result).all())
        self.assertLess(float(result[0,0]),-.8)
        # Nominal encoder FK before the controller's first 3 mm tool transition.
        np.testing.assert_allclose(result.numpy()[0],[-1.022242,0,-.396327],atol=1e-6)

    def test_selected_policy_matches_recorded_golden_observations(self):
        actor=Actor();golden=np.load(ROOT/'assets/actor_golden.npz')
        with torch.inference_mode():result=actor.model(torch.from_numpy(golden['observations'])).numpy()
        np.testing.assert_allclose(result,golden['actions'],atol=2e-6,rtol=1e-6)

    def test_real_segmentation_weights_load_and_infer(self):
        from crack_inference import CrackSegmenter
        segmenter=CrackSegmenter(ROOT/'assets/rhanet.pth',device='cpu',threshold=.90,input_size=(360,640))
        result=segmenter.predict_bgr(np.zeros((480,640,3),np.uint8))
        self.assertEqual(result.mask.shape,(480,640));self.assertTrue(np.isfinite(result.probability).all())

    def test_camera_times_remain_independent_and_stale_pairs_fail(self):
        fc=calibration(ROOT/'config/front_rgb.yaml')
        rc=dict(calibrated=True,T_base_camera=np.eye(4).tolist())
        front=dict(k=np.eye(3),number=8,received=20.,timestamp_basis='host_receive',device_time_ms=None)
        rear=dict(front,number=120,received=20.021,device_time_ms=200.,depth_device_time_ms=201.,
                  timestamp_domain='hardware_clock',depth_timestamp_domain='hardware_clock')
        fm,rm=paired_metadata(front,rear,fc,rc,3,4,20.1)
        self.assertEqual(fm['capture_timestamp'],20.)
        self.assertEqual(rm['capture_timestamp'],20.021)
        self.assertNotEqual(fm['source_frame_number'],rm['source_frame_number'])
        with self.assertRaises(ValueError):paired_metadata(front,rear,fc,rc,3,4,22.)
        rear['received']=20.05
        with self.assertRaises(ValueError):paired_metadata(front,rear,fc,rc,3,4,20.1)

    def test_unmeasured_rear_extrinsics_cannot_enable_tracking(self):
        with patch('cameras.yaml.safe_load',return_value=dict(calibrated=False,T_base_camera=None)):
            with self.assertRaises(ValueError):calibration(ROOT/'config/rear_d435i.yaml')
        self.assertFalse(packet_ok(None,1,2))

    def test_run_refuses_missing_extrinsics_before_any_robot_io(self):
        import run
        with patch.object(sys,'argv',['run.py','--run']), patch.object(run,'library') as lib:
            with patch.object(run,'calibration',side_effect=ValueError('unmeasured extrinsics')):
                with self.assertRaises(ValueError):run.main()
            lib.assert_not_called()

    def test_all_packaged_hashes_are_verified(self):
        import hashlib
        manifest=json.loads((ROOT/'MANIFEST.json').read_text())
        for name,digest in manifest['files_sha256'].items():
            if name.startswith('config/'):
                # Installation necessarily changes camera calibration/serial and
                # interface settings. Immutable program/models still match hashes.
                continue
            self.assertEqual(hashlib.sha256((ROOT/name).read_bytes()).hexdigest(),digest,name)

if __name__=='__main__':unittest.main()
