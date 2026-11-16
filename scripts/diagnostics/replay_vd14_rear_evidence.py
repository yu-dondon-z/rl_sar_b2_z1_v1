#!/usr/bin/env python3
"""Replay retained RGB-D slots through old/new perception on the SAME network mask.

No physics, world poses, actor, publication clocks, hardware I/O or old logs
are changed. Capture age is intentionally not deployment acceptance here.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import tempfile

import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'scripts'),'/home/dong/b2_z1/scripts/vision','/home/dong/RHACrackNet/code']
from crack_inference import CrackSegmenter
from dual_camera_perception import PerceptionConfig
from vd12_gun_mask import NearClippedEncoderGunMask
from vd12_visual_rear import VD12RearSupportPerception
from vd14_visual_rear import VD14RearEvidencePerception
from vd5_visual_control import rear_visual_target


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directories',type=Path,nargs='+')
    parser.add_argument('--output-root',type=Path,default=ROOT/'outputs/vd14_rear_evidence')
    args=parser.parse_args()
    records=[];segmenter=None
    for directory in args.directories:
        directory=directory.resolve(strict=True)
        manifest=json.loads((directory/'manifest.json').read_text())
        if segmenter is None:
            detector=Path(manifest['detector_checkpoint'])
            detector_hash=hashlib.sha256(detector.read_bytes()).hexdigest()
            if detector_hash!=manifest['detector_sha256']:raise ValueError('Detector identity changed')
            segmenter=CrackSegmenter(detector,'RHANet_dep','cuda:0',.90,input_size=(360,640))
            segmenter.warmup(360,640)
        if manifest['detector_sha256']!=detector_hash:raise ValueError('Replay detector differs across runs')
        for meta_path in sorted((directory/'rear').glob('frame_slot_*.json')):
            metadata=json.loads(meta_path.read_text())
            rgb_path=directory/'rear'/metadata['rgb_file'];depth_path=directory/'rear'/metadata['depth_file']
            rgb=cv2.imread(str(rgb_path));depth=cv2.imread(str(depth_path),cv2.IMREAD_UNCHANGED)
            inferred=segmenter.predict_bgr(rgb)
            class SameMask:
                def predict_bgr(self,_):return inferred
            kwargs=dict(config=PerceptionConfig(mode='rhanet'),segmenter=SameMask(),
                gun_mask=NearClippedEncoderGunMask(
                    '/home/dong/b2_z1_VD2/source/b2_z1/b2_z1/assets/b2_z1/b2_z1_vd13_mount75.urdf',
                    near_m=manifest['render_near_plane_m']),
                rear_centerline_mode='cross_sections',trailing_distance=.95,max_occlusion_extension=.05)
            record=dict(directory=str(directory),sequence=metadata['sequence'],
                capture_sim_time_s=metadata['capture_sim_time_s'],
                rgb_sha256=hashlib.sha256(rgb_path.read_bytes()).hexdigest(),
                depth_sha256=hashlib.sha256(depth_path.read_bytes()).hexdigest())
            for name,cls in (('old',VD12RearSupportPerception),('new',VD14RearEvidencePerception)):
                result=cls(**kwargs).process_rear(metadata,rgb,depth)
                diagnostic=dict(valid=result['valid'],reason=result['reason'])
                if result['valid']:
                    uv=np.rint(result['waypoints_uv']).astype(int)
                    diagnostic['selected_pixel_gray_median']=float(np.median(cv2.cvtColor(rgb,cv2.COLOR_BGR2GRAY)[uv[:,1],uv[:,0]]))
                    try:
                        target,details=rear_visual_target(result['points_b'],result['ground_plane_b'],.95,.05)
                        diagnostic.update(target_b=target.tolist(),tangent_b=details['rear_tangent_b'],target_source=details['rear_target_source'])
                    except ValueError as error:
                        diagnostic['target_rejection']=str(error)
                record[name]=diagnostic
            records.append(record)
            print(json.dumps(record),flush=True)
    args.output_root.mkdir(parents=True,exist_ok=True)
    output=Path(tempfile.mkdtemp(prefix='replay_',dir=args.output_root))/'comparison.json'
    output.write_text(json.dumps(dict(detector_sha256=detector_hash,records=records,
        same_neural_output_for_both=True,simulation_advanced=False,world_pose_input=False,
        source_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
            (Path(__file__),ROOT/'scripts/vd14_visual_rear.py',ROOT/'scripts/vd12_visual_rear.py')}),indent=2,allow_nan=False))
    print(output)
    return 0


if __name__=='__main__':raise SystemExit(main())
