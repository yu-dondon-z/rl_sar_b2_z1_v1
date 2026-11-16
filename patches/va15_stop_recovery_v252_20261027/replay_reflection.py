"""Saved RGB comparison, not a full session or hardware acceptance."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import time

os.environ['OMP_NUM_THREADS']='2'
os.environ['MKL_NUM_THREADS']='2'
import cv2
import numpy as np
from camera_cpu import configure_cpu_layout
from camera_near import CALIBRATION_SHA
from long_tracker import LongTracker


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--frames',type=Path,required=True)
    p.add_argument('--previous',type=Path,required=True)
    p.add_argument('--rha',type=Path,default=Path.home()/'RHACrackNet/code')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--require-rescue',action='append',default=[])
    args=p.parse_args()
    fixture=args.output.with_suffix('.packets.jsonl')
    if args.output.exists() or fixture.exists():raise RuntimeError('New output names required')
    import torch
    cv2.setNumThreads(1);torch.set_num_threads(2);torch.set_num_interop_threads(1)
    sys.path.insert(0,str(args.rha))
    from crack_inference import CrackSegmenter
    spec=importlib.util.spec_from_file_location('previous_reflection_tracker',args.previous/'long_tracker.py')
    old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)
    weight=args.rha/'exps/rhanet_dep_CamCrack789_b2_stage1/ckpt/best_ckpt.pth'
    net=CrackSegmenter(weight,'RHANet_dep','cpu',.90,input_size=(360,640))
    layout=configure_cpu_layout(net)
    records=[];packets=[]
    for path in sorted(args.frames.glob('*.jpg')):
        rgb=cv2.imread(str(path));result=net.predict_bgr(rgb)
        def detect(cls):
            begin=time.perf_counter()
            try:
                f=cls().detect(rgb,result.mask,result.probability)
                return dict(valid=f['long_valid'],near_u=f['near_u'],look_u=f['look_u'],look_v=f['look_v'],
                    span=f['path_vertical_span_px'],recovered=f.get('reflection_recovered_rows',0),
                    endpoint=f['endpoint_candidate'],post_ms=(time.perf_counter()-begin)*1000.),f
            except ValueError as error:return dict(valid=False,reason=str(error)),None
        before,_=detect(old.LongTracker);after,features=detect(LongTracker)
        if features is not None:
            packet={k:v for k,v in features.items() if k!='points'}
            now=time.monotonic_ns()
            packet.update(schema='va15_long_visual_v1',session_id='a'*32,calibration_sha256=CALIBRATION_SHA,
                timestamp_basis='host_receive_monotonic',sequence=len(packets)+1,capture_monotonic_ns=now-100_000_000,
                publish_monotonic_ns=now,image_width=1920,image_height=1080,image_received=True,
                valid=bool(after['valid']),offline_saved_image_fixture=True,frame=path.name)
            packets.append(packet)
        records.append(dict(frame=path.name,old=before,new=after))
    needed={r['frame']:r for r in records if r['frame'] in args.require_rescue}
    missing=[name for name in args.require_rescue if name not in needed or not needed[name]['new']['valid']]
    regressions=[r['frame'] for r in records if r['old']['valid'] and not r['new']['valid']]
    report=dict(frames=len(records),old_supported=sum(r['old']['valid'] for r in records),
        new_supported=sum(r['new']['valid'] for r in records),regressions=regressions,
        required_rescues_failed=missing,results=records,full_session_replay=False,hardware_qualified=False,
        weight_sha256=hashlib.sha256(weight.read_bytes()).hexdigest(),layout=layout,
        source_sha256={f:hashlib.sha256((Path(__file__).parent/f).read_bytes()).hexdigest()
                       for f in ['near_tracker.py','long_tracker.py','endpoint_vision.py']})
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    fixture.write_text(''.join(json.dumps(p,allow_nan=False)+'\n' for p in packets))
    print(json.dumps({k:v for k,v in report.items() if k!='results'},indent=2))
    for r in records:
        print(r['frame'],r['old'],r['new'])
    return 0 if records and not missing and not regressions else 2


if __name__=='__main__':raise SystemExit(main())
