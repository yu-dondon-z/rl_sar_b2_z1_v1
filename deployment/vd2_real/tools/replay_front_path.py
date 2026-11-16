#!/usr/bin/env python3
"""Replay saved real images and associated B2 IMU; no robot/camera IO."""
import argparse
import gzip
import json
import multiprocessing as mp
import os
os.environ['OMP_WAIT_POLICY']='PASSIVE'
os.environ['OPENBLAS_NUM_THREADS']='1'
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'python/vision'),str(ROOT/'thirdparty/rhacracknet')]
import cv2
import numpy as np
import torch
import yaml
from control import quaternion_wxyz_to_matrix
from front_projection import HISTORY_SIZE,publish_gravity,process_front
from vd1_front_path_estimator import FrontPathEstimator
from vd1_visual_guard import GuardedVD1VisualController
from realtime_segmenter import RealtimeSegmenter
from realtime_perception import ObservableRearPerception
from dual_camera_perception import PerceptionConfig


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--prefix',default='working_pair_')
    parser.add_argument('--iterations',type=int,default=3)
    args=parser.parse_args()
    if args.iterations<1:raise ValueError('positive iteration count required')
    def source(name):return args.session/(args.prefix+name)
    pair=json.loads(source('pair_metadata.json').read_text())
    front=cv2.imread(str(source('front_latest.png')));rear=cv2.imread(str(source('rear_latest.png')))
    exact_depth=source('rear_depth_m.npy')
    depth=(np.load(exact_depth,allow_pickle=False) if exact_depth.is_file() else
           cv2.imread(str(source('rear_depth_mm.png')),cv2.IMREAD_UNCHANGED))
    if front is None or rear is None or depth is None:raise ValueError('incomplete saved pair')
    if not exact_depth.is_file():depth=depth.astype(np.float32)*.001;depth[depth<=0]=np.nan
    capture=pair['front']['capture_timestamp'];imu=[]
    with gzip.open(args.session/'control.jsonl.gz','rt') as log:
        for line in log:
            row=json.loads(line)
            if 'quaternion' not in row or row.get('b2_age_s') is None:continue
            receipt=row['monotonic']-row['b2_age_s']
            if capture-.10<=receipt<=capture:
                gravity=quaternion_wxyz_to_matrix(row['quaternion']).T@np.array([0.,0.,-1.])
                imu.append((receipt,gravity))
    history=mp.get_context('spawn').Array('d',HISTORY_SIZE*4,lock=True)
    for timestamp,gravity in imu[-HISTORY_SIZE:]:publish_gravity(history,timestamp,gravity)
    height=yaml.safe_load((ROOT/'config/deployment.yaml').read_text())['front_nominal_base_height_m']
    cv2.setNumThreads(1);torch.set_num_threads(2)
    segmenter=RealtimeSegmenter(ROOT/'assets/rhanet.pth',device='cpu',threshold=.90,input_size=(360,640))
    segmenter.warmup(360,640,2)
    perception=ObservableRearPerception(PerceptionConfig(rhacracknet_root=str(ROOT/'thirdparty/rhacracknet')),segmenter)
    for _ in range(args.iterations):
        start=time.perf_counter();r=perception.process_rear(pair['rear'],rear,depth)
        f,fm=process_front(perception,pair['front'],front,r,history,height)
        estimation=None;error=None
        if f.get('valid'):
            try:_,estimation=FrontPathEstimator().update(f)
            except ValueError as exc:error=str(exc)
        packet=dict(schema='b2_z1_dual_vision_v1',capture_generation=pair['front']['capture_generation'],
                    front=f,rear=r,valid=bool(f.get('valid') and r.get('valid')))
        ctrl=GuardedVD1VisualController([-1.02,0,-.3],forward_speed=.2,front_path_estimation=True)
        # Original receipt times retained. Evaluate at the historical capture clock,
        # never present saved images to the live-control clock as fresh frames.
        status=ctrl.update(packet,max(pair[n]['capture_timestamp'] for n in ('front','rear'))+.02,.02)['status']
        print(json.dumps(dict(offline_only=True,opens_robot_io=False,prefix=args.prefix,
            total_ms=1000*(time.perf_counter()-start),front_valid=f.get('valid'),front_reason=f.get('reason'),
            rear_valid=r.get('valid'),rear_reason=r.get('reason'),front_estimator_error=error,
            rear_work_bracket_m=r.get('rear_observed_work_bracket_m'),
            rear_path_length_m=r.get('path_length_m'),exact_depth=exact_depth.is_file(),
            geometry_status=status,estimation=estimation,
            imu_host_receive_timestamp=fm.get('imu_host_receive_timestamp'),
            source_capture_timestamp=capture,
            note='fixed height model; real logs replayed at historical clock; no live freshness/accuracy acceptance'),
            allow_nan=False,ensure_ascii=False),flush=True)


if __name__=='__main__':main()
