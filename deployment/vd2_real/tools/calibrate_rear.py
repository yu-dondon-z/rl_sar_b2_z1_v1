#!/usr/bin/env python3
"""Estimate camera extrinsics from a checkerboard whose pose in B2 is measured."""
import argparse
import json
from pathlib import Path
import sys
import cv2
import numpy as np
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'python'))
from realsense_geometry import intrinsics_from_record,rectification_maps

def prepare_image(image,intrinsics):
    """New raw RealSense records are rectified using their recorded SDK model.

    Old records without a model retain the historical OpenCV Brown contract.
    No inverse Brown coefficients are sent to OpenCV solvePnP.
    """
    k=np.asarray(intrinsics['intrinsic_matrix'],dtype=float)
    if 'distortion_model' not in intrinsics:
        return image,k,np.asarray(intrinsics['distortion_coefficients'],dtype=float)
    sensor=intrinsics_from_record(intrinsics)
    if image.shape[:2]!=(sensor.height,sensor.width):
        raise ValueError('raw image size does not match recorded RealSense intrinsics')
    image=cv2.remap(image,*rectification_maps(sensor),interpolation=cv2.INTER_LINEAR)
    return image,k,np.zeros(5)

def rigid(value):
    t=np.asarray(value,dtype=float)
    if (t.shape!=(4,4) or not np.isfinite(t).all() or not np.allclose(t[3],[0,0,0,1])
            or not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-4)
            or not np.isclose(np.linalg.det(t[:3,:3]),1,atol=1e-4)):
        raise ValueError('T_base_board must be a measured rigid 4x4 transform')
    return t

def solve(image,k,distortion,board_pose,columns,rows,square):
    if columns<3 or rows<3 or square<=0:raise ValueError('invalid checkerboard dimensions')
    gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    found,corners=cv2.findChessboardCornersSB(gray,(columns,rows))
    if not found:raise ValueError('checkerboard not detected')
    object_points=np.zeros((columns*rows,3),dtype=np.float64)
    object_points[:,:2]=np.mgrid[0:columns,0:rows].T.reshape(-1,2)*square
    # Corner origin/order must match the physically measured board transform.
    success,rvec,tvec=cv2.solvePnP(object_points,corners,np.asarray(k),np.asarray(distortion))
    if not success:raise ValueError('PnP failed')
    projection=cv2.projectPoints(object_points,rvec,tvec,np.asarray(k),np.asarray(distortion))[0]
    rms=float(np.sqrt(np.mean(np.sum((projection-corners)**2,axis=2))))
    if rms>1.:raise ValueError('reprojection RMS exceeds 1 pixel: '+str(rms))
    t_camera_board=np.eye(4);t_camera_board[:3,:3]=cv2.Rodrigues(rvec)[0];t_camera_board[:3,3]=tvec[:,0]
    if np.any((object_points@t_camera_board[:3,:3].T+t_camera_board[:3,3])[:,2]<=0):raise ValueError('board behind camera')
    result=rigid(board_pose)@np.linalg.inv(t_camera_board)
    return result,rms,corners

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--image',required=True);p.add_argument('--intrinsics',required=True)
    p.add_argument('--board-pose',required=True,help='JSON with measured T_base_board, first inner corner origin')
    p.add_argument('--columns',type=int,default=9);p.add_argument('--rows',type=int,default=6)
    p.add_argument('--square-m',type=float,default=.025)
    p.add_argument('--output',required=True,help='new YAML file; never overwrites an existing calibration')
    args=p.parse_args();target=Path(args.output)
    if target.exists():raise FileExistsError('choose a new output path to preserve existing calibration')
    intr=json.loads(Path(args.intrinsics).read_text());board=json.loads(Path(args.board_pose).read_text())
    image=cv2.imread(args.image)
    if image is None:raise ValueError('cannot read raw camera image')
    image,k,distortion=prepare_image(image,intr)
    t,rms,corners=solve(image,k,distortion,
        board['T_base_board'],args.columns,args.rows,args.square_m)
    cv2.drawChessboardCorners(image,(args.columns,args.rows),corners,True)
    # Mark first/last detected corners to make corner-order ambiguity reviewable.
    for label,corner in (('origin',corners[0]),('last',corners[-1])):
        cv2.putText(image,label,tuple(np.rint(corner[0]).astype(int)),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,0,255),2)
    cv2.imwrite(str(target.with_suffix('.corners.png')),image)
    target.write_text(yaml.safe_dump(dict(calibrated=True,camera_name='rear_d435i',
        serial=intr['serial'],T_base_camera=t.tolist(),reprojection_rms_px=rms,
        source_image=str(Path(args.image).resolve()),source_board_pose=str(Path(args.board_pose).resolve()),
        source_distortion_model=intr.get('distortion_model','legacy_opencv_brown_conrady'),
        calibration_image_space='rectified_pinhole' if 'distortion_model' in intr else 'legacy_raw_opencv_brown',
        caveat='PnP reprojection does not validate measured board pose; verify corner origin and repeat independently'),sort_keys=False))
    print('Wrote '+str(target)+'; RMS %.4f px. Check the labelled origin before installing.'%rms)

if __name__=='__main__':main()
