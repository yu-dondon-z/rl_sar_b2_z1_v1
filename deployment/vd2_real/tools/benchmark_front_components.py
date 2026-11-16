#!/usr/bin/env python3
"""Compare exhaustive/pruned front extraction on saved pairs; never open hardware.

Both modes use original images, weights, mask threshold, camera geometry and
rear depth. Timings exclude camera acquisition, SDK IO and live scheduling.
"""
import argparse
import functools
import gzip
import json
import os
os.environ['OMP_WAIT_POLICY']='PASSIVE'
os.environ['OPENBLAS_NUM_THREADS']='1'
from pathlib import Path
import sys
import time
from unittest.mock import patch
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'python/vision'),str(ROOT/'thirdparty/rhacracknet')]
import cv2
import numpy as np
import torch
from control import quaternion_wxyz_to_matrix
from vd1_front_path_estimator import nominal_front_plane
import vd1_visual_front
from realtime_segmenter import RealtimeSegmenter
from realtime_perception import ObservableRearPerception
from dual_camera_perception import PerceptionConfig
import yaml

def front_meta(meta,rows,height):
    # Reconstruct the same recorded IMU/fixed-height front projection if an old
    # diagnostic pair predates saving its projected front metadata.
    capture=meta['capture_timestamp']
    candidates=[r for r in rows if 'quaternion' in r and r.get('b2_age_s') is not None
                and capture-.1<=r['monotonic']-r['b2_age_s']<=capture]
    if not candidates:raise ValueError('saved front has no preceding real IMU')
    row=max(candidates,key=lambda r:r['monotonic']-r['b2_age_s'])
    gravity=quaternion_wxyz_to_matrix(row['quaternion']).T@np.array([0.,0.,-1.])
    return dict(meta,ground_plane_b=nominal_front_plane(gravity,height).tolist(),
        ground_plane_source='imu_gravity_and_configured_nominal_height')

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sessions',type=Path,nargs='+',required=True)
    parser.add_argument('--iterations',type=int,default=3)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    if args.iterations<1:raise ValueError('positive iteration count required')
    cv2.setNumThreads(1);torch.set_num_threads(2)
    segmenter=RealtimeSegmenter(ROOT/'assets/rhanet.pth',device='cpu',threshold=.90,input_size=(360,640))
    segmenter.warmup(360,640,2)
    perception=ObservableRearPerception(PerceptionConfig(rhacracknet_root=str(ROOT/'thirdparty/rhacracknet')),segmenter)
    original=vd1_visual_front.supported_front_centerline;results=[];pairs=0
    height=yaml.safe_load((ROOT/'config/deployment.yaml').read_text())['front_nominal_base_height_m']
    for session in args.sessions:
        with gzip.open(session/'control.jsonl.gz','rt') as stream:rows=[json.loads(line)for line in stream]
        for path in sorted(session.glob('*pair_metadata.json')):
            prefix=path.name[:-len('pair_metadata.json')];meta=json.loads(path.read_text())
            fm=front_meta(meta['front'],rows,height)
            front=cv2.imread(str(session/(prefix+'front_latest.png')))
            rear=cv2.imread(str(session/(prefix+'rear_latest.png')))
            exact=session/(prefix+'rear_depth_m.npy')
            depth=(np.load(exact,allow_pickle=False) if exact.is_file() else
                   cv2.imread(str(session/(prefix+'rear_depth_mm.png')),cv2.IMREAD_UNCHANGED))
            if front is None or rear is None or depth is None:raise ValueError('incomplete '+str(path))
            if not exact.is_file():depth=depth.astype(np.float32)*.001;depth[depth<=0]=np.nan
            for iteration in range(args.iterations):
                reference=None
                for prune in (False,True):
                    with patch.object(vd1_visual_front,'supported_front_centerline',functools.partial(original,prune=prune)):
                        start=time.perf_counter();r=perception.process_rear(meta['rear'],rear,depth);middle=time.perf_counter()
                        f=perception.process_front(fm,front);end=time.perf_counter()
                    geometry=(f['valid'],f['reason'],f['points_b'],f['waypoints_uv'],f.get('confidence'))
                    if reference is None:reference=geometry
                    equal=geometry==reference
                    result=dict(session=str(session),prefix=prefix,iteration=iteration,pruned=prune,
                        total_ms=1000*(end-start),front_ms=1000*(end-middle),rear_ms=1000*(middle-start),
                        front_valid=f['valid'],rear_valid=r['valid'],front_exactly_matches_exhaustive=equal,
                        exact_depth=exact.is_file())
                    results.append(result)
                    if not equal:raise RuntimeError('front geometry changed: '+str(path))
            pairs+=1
    if not pairs:raise ValueError('no saved pairs')
    summary={}
    for prune,name in ((False,'exhaustive'),(True,'pruned')):
        group=[r for r in results if r['pruned']==prune];times=np.array([r['total_ms']for r in group])
        fronts=np.array([r['front_ms']for r in group])
        summary[name]=dict(samples=len(group),valid_pairs=sum(r['front_valid'] and r['rear_valid']for r in group),
            total_ms=dict(p50=float(np.median(times)),p95=float(np.percentile(times,95)),max=float(times.max())),
            front_ms=dict(p50=float(np.median(fronts)),p95=float(np.percentile(fronts,95)),max=float(fronts.max())),
            adjacent_compute_sum_max_ms=float(np.max(times[:-1]+times[1:])) if len(times)>1 else None)
    report=dict(offline_only=True,hardware_io_opened=False,saved_pairs=pairs,summary=summary,rows=results,
        limitation='developer CPU, saved pairs, no camera/SDK concurrency; no live freshness or tracking accuracy acceptance')
    args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps({k:v for k,v in report.items()if k!='rows'},indent=2))

if __name__=='__main__':main()
