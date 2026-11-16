"""OFFLINE saved-frame equivalence and timing. No SDK, sockets or commands."""
import argparse
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch

import cv2
import numpy as np

from camera_cpu import configure_cpu_layout
from long_tracker import LongTracker


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--frames',type=Path,required=True)
    p.add_argument('--rha',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    if args.output.exists():raise RuntimeError('Use a new report path')
    cv2.setNumThreads(1)
    import torch
    torch.set_num_threads(2);torch.set_num_interop_threads(1)
    sys.path.insert(0,str(args.rha))
    from crack_inference import CrackSegmenter
    model=CrackSegmenter(args.rha/'exps/rhanet_dep_CamCrack789_b2_stage1/ckpt/best_ckpt.pth',
                         'RHANet_dep','cpu',.90,input_size=(360,640))
    paths=sorted(args.frames.glob('*.jpg'))
    images=[cv2.imread(str(x)) for x in paths]
    if len(images)<3 or any(x is None for x in images):raise RuntimeError('Need saved decoded RGB frames')
    results=[]
    packets=[]
    def detect(image,result,capture=None):
        calls=[];original=np.polyfit
        def fit(x,y,deg,*a,**kw):
            calls.append((np.asarray(x).copy(),np.asarray(y).copy()))
            return original(x,y,deg,*a,**kw)
        try:
            with patch('near_tracker.np.polyfit',side_effect=fit):
                f=LongTracker().detect(image,result.mask,result.probability)
            if capture is not None:
                # Parser fixtures only: these are saved-file inferences, not
                # live camera packets or independent frames for real entry.
                from camera_near import CALIBRATION_SHA
                packet={k:v for k,v in f.items() if k!='points'}
                packet.update(schema='va15_long_visual_v1',session_id='a'*32,
                    timestamp_basis='host_receive_monotonic',calibration_sha256=CALIBRATION_SHA,
                    image_width=1920,image_height=1080,sequence=len(packets)+1,
                    capture_monotonic_ns=capture,publish_monotonic_ns=time.monotonic_ns(),
                    valid=bool(f['long_valid']),offline_saved_image_fixture=True)
                packets.append(packet)
            return dict(valid=f['long_valid'],u=f['near_u'],look=f['look_u'])
        except ValueError as e:
            out=dict(valid=False,reason=str(e))
            if calls:
                x,y=calls[-1]
                out['line_rms']=float(np.sqrt(np.mean((y-original(x,y,1)[0]*x-original(x,y,1)[1])**2)))
                out['quadratic_rms']=float(np.sqrt(np.mean((y-np.polyval(original(x,y,2),x))**2)))
            return out
    model.warmup(1080,1920,iterations=3)
    baseline=[]
    for path,image in zip(paths,images):
        start=time.perf_counter();r=model.predict_bgr(image);elapsed=(time.perf_counter()-start)*1000
        baseline.append((r,detect(image,r),elapsed))
    golden=configure_cpu_layout(model)
    model.warmup(1080,1920,iterations=3)
    for path,image,(old,old_path,old_ms) in zip(paths,images,baseline):
        capture=time.monotonic_ns()
        start=time.perf_counter();r=model.predict_bgr(image);elapsed=(time.perf_counter()-start)*1000
        new_path=detect(image,r,capture)
        error=float(np.max(np.abs(old.probability-r.probability)))
        equivalent=bool(np.allclose(old.probability,r.probability,atol=5.e-5,rtol=1.e-4))
        same=old_path['valid']==new_path['valid']
        if old_path['valid'] and new_path['valid']:
            same=same and abs(old_path['u']-new_path['u'])<1. and abs(old_path['look']-new_path['look'])<1.
        results.append(dict(frame=path.name,baseline_ms=old_ms,candidate_ms=elapsed,
                            max_probability_error=error,equivalent=equivalent,same_path=same,
                            old_path=old_path,new_path=new_path))
    # Repeated warmed full-wrapper measurements, not just the neural forward.
    samples=[]
    for i in range(40):
        start=time.perf_counter();model.predict_bgr(images[i%len(images)])
        samples.append((time.perf_counter()-start)*1000)
    report=dict(pass_equivalence=all(x['equivalent'] and x['same_path'] for x in results),
                golden=golden,frames=results,
                baseline_median_ms=float(np.median([x['baseline_ms'] for x in results])),
                candidate_median_ms=float(np.median([x['candidate_ms'] for x in results])),
                repeated_wrapper_p95_ms=float(np.percentile(samples,95)),
                repeated_wrapper_max_ms=max(samples),hardware_qualified=False,
                saved_frames_only=True)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    args.output.with_suffix('.packets.jsonl').write_text(''.join(json.dumps(x,allow_nan=False)+'\n' for x in packets))
    print(json.dumps(report,indent=2))
    return 0 if report['pass_equivalence'] else 2


if __name__=='__main__':raise SystemExit(main())
