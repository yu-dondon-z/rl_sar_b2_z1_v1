import importlib.util
from pathlib import Path
import unittest
from unittest.mock import patch
import cv2
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('calibrate_rear',ROOT/'tools/calibrate_rear.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)

class CalibrationTests(unittest.TestCase):
    def test_known_checkerboard_pose_recovers_optical_to_base_transform(self):
        k=np.array([[610.,0,320],[0,612.,240],[0,0,1.]])
        points=np.zeros((54,3));points[:,:2]=np.mgrid[0:9,0:6].T.reshape(-1,2)*.025
        rvec=np.array([.15,-.12,.03]);tvec=np.array([-.1,-.06,.8])
        corners=cv2.projectPoints(points,rvec,tvec,k,np.zeros(5))[0].astype(np.float32)
        board_pose=np.eye(4);board_pose[:3,3]=[-.8,0,-.5]
        expected=np.eye(4);expected[:3,:3]=cv2.Rodrigues(rvec)[0];expected[:3,3]=tvec
        with patch.object(cv2,'findChessboardCornersSB',return_value=(True,corners)):
            result,rms,_=module.solve(np.zeros((480,640,3),np.uint8),k,np.zeros(5),board_pose,9,6,.025)
        np.testing.assert_allclose(result,board_pose@np.linalg.inv(expected),atol=5e-6)
        self.assertLess(rms,1e-4)

    def test_invalid_measured_board_transform_is_rejected(self):
        bad=np.eye(4);bad[0,0]=2
        with self.assertRaises(ValueError):module.rigid(bad)

if __name__=='__main__':unittest.main()
