"""Offline saved-JPEG/network geometry audit; never imports a camera/robot SDK.

Saved JPEGs are recompressed and not the original stream/probabilities. A pair
does not recreate the complete motion-association history. No motion validity,
live timing or hardware qualification is granted by this analysis.
"""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--recording',type=Path,default=Path('/home/dong/rl_sar_b2_z1/outputs/diagnosis_v251_yumjt3ry_20261027/exchange'))
    p.add_argument('--rha',type=Path,default=Path('/home/dong/RHACrackNet/code'))
    p.add_argument('--sequences',type=int,nargs='+',default=[489])
    p.add_argument('--tracker',choices=['current','baseline'],default='current')
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    sys.path.insert(0,str(args.rha))
    import torch
    torch.set_num_threads(2);torch.set_num_interop_threads(1);cv2.setNumThreads(1)
    from crack_inference import CrackSegmenter
    from camera_cpu import configure_cpu_layout
    tracker_source=Path(__file__).resolve().with_name('long_tracker.py')
    if args.tracker=='baseline':
        tracker_source=Path(__file__).resolve().parent.parent/'va15_stop_recovery_v252_20261027/long_tracker.py'
        spec=importlib.util.spec_from_file_location('archived_long_tracker',tracker_source)
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        LongTracker=module.LongTracker
    else:
        from long_tracker import LongTracker
    source_sha=hashlib.sha256(tracker_source.read_bytes()).hexdigest()
    weight=args.rha/'exps/rhanet_dep_CamCrack789_b2_stage1/ckpt/best_ckpt.pth'
    expected='f1eea726c85773c3ddb71649bdae097eff6ca7693c7f36ad488fa9f4643bdb80'
    if hashlib.sha256(weight.read_bytes()).hexdigest()!=expected:
        raise RuntimeError('Recorded deployment weight mismatch')
    network=CrackSegmenter(weight,'RHANet_dep','cpu',.90,input_size=(360,640))
    layout=configure_cpu_layout(network)
    network.warmup(1080,1920,iterations=1)
    reports=[]
    archived={}
    for line in (args.recording/'guide_packets.jsonl').read_text().splitlines():
        try:
            packet=json.loads(line);archived[packet['sequence']]=packet
        except (json.JSONDecodeError,KeyError):
            continue # incomplete shutdown tail is not an accepted camera packet
    for sequence in args.sequences:
        tracker=LongTracker()
        for j,side in enumerate(['before','after']):
            file=args.recording/f'rejection_{sequence:06d}_{side}.jpg'
            image=cv2.imread(str(file))
            if image is None:raise RuntimeError('Missing saved JPEG: '+str(file))
            result=network.predict_bgr(image)
            np.savez_compressed(args.output/f'{sequence}_{side}_prob.npz',probability=cv2.resize(result.probability,(640,360)))
            started=time.perf_counter()
            report=dict(sequence=sequence,side=side,image_sha256=hashlib.sha256(file.read_bytes()).hexdigest(),
                        inference_ms=result.inference_ms)
            original=archived[sequence-(side=='before')]
            recomputed=dict(original);recomputed['valid']=False
            try:
                # Explicit artificial OFFLINE association clock for this pair,
                # never written as a camera capture or fed into motion control.
                features=tracker.detect(image,result.mask,result.probability,1_000_000_000+j*170_000_000)
                report.update(near_detected=True,long_valid=features['long_valid'],
                    span_px=features['path_vertical_span_px'],point_count=features['path_point_count'],
                    look_v=features['look_v'],endpoint_candidate=features['endpoint_candidate'])
                recomputed.update({k:v for k,v in features.items() if k!='points'})
                # Geometry review ONLY. No claim that this two-frame replay
                # establishes live six-frame acquisition or timing budget.
                recomputed['valid']=bool(features['long_valid'])
                for u,v in features['points']:cv2.circle(image,(round(u),round(v)),3,(0,220,255),-1)
            except ValueError as exc:
                report.update(near_detected=False,error=str(exc))
            artifact=dict(schema='va15_offline_geometry_review_v1',motion_permission=False,
                          acquisition_history_complete=False,timing_budget_qualified=False,
                          original_packet=original,recomputed_packet=recomputed,
                          expected_geometry_valid=bool(recomputed['valid']))
            (args.output/f'{sequence}_{side}_geometry.json').write_text(json.dumps(artifact,allow_nan=False)+'\n')
            report.update(postprocess_ms=(time.perf_counter()-started)*1000,diagnostics=tracker.debug)
            cv2.imwrite(str(args.output/f'{sequence}_{side}_overlay.jpg'),image)
            reports.append(report);print(json.dumps(report),flush=True)
    if hashlib.sha256(tracker_source.read_bytes()).hexdigest()!=source_sha:
        raise RuntimeError('Tracker changed during image audit')
    summary=dict(reports=reports,cpu_layout=layout,tracker=args.tracker,tracker_sha256=source_sha,
                 checkpoint_sha256=expected,recompressed_jpeg=True,complete_history=False,
                 hardware_qualified=False,live_timing_qualified=False,policy_control_invoked=False)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')


if __name__=='__main__':main()
