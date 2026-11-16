"""Saved RGB only: compare fixed/observed look rows; no SDK or robot access."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

from camera_cpu import configure_cpu_layout
from camera_near import CALIBRATION_SHA
from long_tracker import LongTracker


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--frames',type=Path,required=True)
    p.add_argument('--rha',type=Path,required=True)
    p.add_argument('--previous',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise RuntimeError('Use a new output name')
    cv2.setNumThreads(1)
    import torch
    torch.set_num_threads(2);torch.set_num_interop_threads(1)
    sys.path.insert(0,str(args.rha))
    from crack_inference import CrackSegmenter
    spec=importlib.util.spec_from_file_location('old_fixed_row',args.previous/'long_tracker.py')
    previous=importlib.util.module_from_spec(spec);spec.loader.exec_module(previous)
    network=CrackSegmenter(args.rha/'exps/rhanet_dep_CamCrack789_b2_stage1/ckpt/best_ckpt.pth',
        'RHANet_dep','cpu',.90,input_size=(360,640))
    golden=configure_cpu_layout(network)
    results=[];packets=[]
    for image_path in sorted(args.frames.glob('*.jpg')):
        rgb=cv2.imread(str(image_path));capture=time.monotonic_ns();result=network.predict_bgr(rgb)
        def detect(cls):
            try:
                f=cls().detect(rgb,result.mask,result.probability)
                return dict(valid=f['long_valid'],look_v=f['look_v'],span=f['path_vertical_span_px'],
                            look_conf=f['look_confidence'],points=f['path_point_count']),f
            except ValueError as e:return dict(valid=False,reason=str(e)),None
        old,_=detect(previous.LongTracker);new,features=detect(LongTracker)
        if features is not None:
            packet={k:v for k,v in features.items() if k!='points'}
            packet.update(schema='va15_long_visual_v1',session_id='a'*32,
                calibration_sha256=CALIBRATION_SHA,timestamp_basis='host_receive_monotonic',
                sequence=len(packets)+1,capture_monotonic_ns=capture,
                publish_monotonic_ns=time.monotonic_ns(),image_width=1920,image_height=1080,
                valid=bool(new['valid']),offline_saved_image_fixture=True)
            packets.append(packet)
        results.append(dict(frame=image_path.name,old=old,new=new))
    report=dict(saved_file_only=True,hardware_qualified=False,frames=len(results),golden=golden,
                old_supported=sum(x['old']['valid'] for x in results),
                new_supported=sum(x['new']['valid'] for x in results),
                newly_supported=sum(x['new']['valid'] and not x['old']['valid'] for x in results),
                regressions=[x['frame'] for x in results if x['old']['valid'] and not x['new']['valid']],
                results=results)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    args.output.with_suffix('.packets.jsonl').write_text(''.join(json.dumps(x,allow_nan=False)+'\n' for x in packets))
    print(json.dumps({k:v for k,v in report.items() if k!='results'},indent=2))
    return 0 if report['frames']>=10 and not report['regressions'] and report['newly_supported']>0 else 2


if __name__=='__main__':raise SystemExit(main())
