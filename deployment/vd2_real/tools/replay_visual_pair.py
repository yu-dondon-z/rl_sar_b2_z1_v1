#!/usr/bin/env python3
"""Time a saved RGB/RGB-D pair offline; never open robot/camera IO."""
import argparse
import json
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
from realtime_segmenter import RealtimeSegmenter
from dual_camera_perception import PerceptionConfig
from realtime_perception import ObservableRearPerception


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--iterations',type=int,default=3)
    parser.add_argument('--prefix',default=None,help='e.g. working_pair_, latest_valid_ or valid_pair_')
    parser.add_argument('--compare-reference',action='store_true',help='compare original CPU masks and probabilities')
    parser.add_argument('--reference',action='store_true',help='time the previously installed perception/layout for comparison')
    args=parser.parse_args()
    if args.iterations<1:raise ValueError('iterations must be positive')
    choices=[args.prefix] if args.prefix is not None else ['working_pair_','latest_valid_','valid_pair_']
    chosen=next((name for name in choices if (args.session/(name+'pair_metadata.json')).is_file()),None)
    if chosen is None:raise FileNotFoundError('no saved paired capture found')
    def source(name):return args.session/(chosen+name)
    meta_path=source('pair_metadata.json')
    if not meta_path.is_file():
        raise FileNotFoundError('no preserved valid pair in this session; older latest images can be prone views')
    pair=json.loads(meta_path.read_text())
    front=cv2.imread(str(source('front_latest.png')));rear=cv2.imread(str(source('rear_latest.png')))
    exact_depth=source('rear_depth_m.npy')
    depth=(np.load(exact_depth,allow_pickle=False) if exact_depth.is_file() else
           cv2.imread(str(source('rear_depth_mm.png')),cv2.IMREAD_UNCHANGED))
    if front is None or rear is None or depth is None:raise ValueError('saved pair is incomplete')
    if not exact_depth.is_file():depth=depth.astype(np.float32)*.001;depth[depth<=0]=np.nan
    if not args.reference:cv2.setNumThreads(1)
    torch.set_num_threads(2)
    if args.reference:
        from crack_inference import CrackSegmenter
        from vd1_visual_rear import SeamOnlyDualPerception
        model_class,perception_class=CrackSegmenter,SeamOnlyDualPerception
    else:model_class,perception_class=RealtimeSegmenter,ObservableRearPerception
    segmenter=model_class(ROOT/'assets/rhanet.pth',device='cpu',threshold=.90,input_size=(360,640))
    segmenter.warmup(360,640,2)
    perception=perception_class(PerceptionConfig(rhacracknet_root=str(ROOT/'thirdparty/rhacracknet')),segmenter)
    if args.compare_reference:
        from crack_inference import CrackSegmenter
        reference=CrackSegmenter(ROOT/'assets/rhanet.pth',device='cpu',threshold=.90,input_size=(360,640))
        for name,image in [('front',front),('rear',rear)]:
            a=reference.predict_bgr(image);b=segmenter.predict_bgr(image)
            print(json.dumps(dict(comparison=name,max_probability_difference=float(np.max(np.abs(a.probability-b.probability))),
                different_mask_pixels=int(np.count_nonzero(a.mask!=b.mask)),input_size=[360,640],threshold=.90)))
    for _ in range(args.iterations):
        start=time.perf_counter();r=perception.process_rear(pair['rear'],rear,depth);middle=time.perf_counter()
        f=dict(valid=False,reason='rear measured ground plane unavailable')
        if r.get('ground_plane_b') is not None:
            fm=dict(pair['front'],ground_plane_b=r['ground_plane_b'],ground_plane_source='paired_rear_depth')
            f=perception.process_front(fm,front)
        end=time.perf_counter();xs=[p[0] for p in r.get('points_b',[])]
        print(json.dumps(dict(offline_replay_only=True,opens_robot_io=False,prefix=chosen,reference=args.reference,
            source_capture_times=[pair[n]['capture_timestamp'] for n in ('front','rear')],
            rear_ms=1000*(middle-start),front_ms=1000*(end-middle),total_ms=1000*(end-start),
            front_valid=f['valid'],front_reason=f['reason'],rear_valid=r['valid'],rear_reason=r['reason'],
            rear_x_range_m=[min(xs),max(xs)] if xs else None,
            rear_covers_target=bool(xs and min(xs)<=-1.02<=max(xs)),
            valid_3d_ratio=r.get('valid_3d_ratio'),direct_depth_point_count=r.get('direct_depth_point_count'),
            plane_intersection_point_count=r.get('plane_intersection_point_count'),
            rear_plane_supported_hole_pixels=r.get('rear_plane_supported_hole_pixels'),
            rear_work_bracket_m=r.get('rear_observed_work_bracket_m'),
            rear_path_length_m=r.get('path_length_m'),exact_depth=exact_depth.is_file(),
            note='historical images; depth PNG quantized to 1 mm unless exact_depth=true; not live validation'),ensure_ascii=False))


if __name__=='__main__':main()
