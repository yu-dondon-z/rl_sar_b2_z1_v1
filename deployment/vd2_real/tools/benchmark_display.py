#!/usr/bin/env python3
"""Measure GUI copy/overlay cost on saved frames only; never open hardware."""
import argparse
import json
from pathlib import Path
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'python'))
import cv2
import numpy as np
from visual_preview import small_frame,render_panel


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',required=True,type=Path)
    parser.add_argument('--prefix',default='latest_valid_')
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--iterations',type=int,default=100)
    args=parser.parse_args()
    if args.output.exists():raise FileExistsError(args.output)
    if args.iterations<2:raise ValueError('at least two iterations required')
    cv2.setNumThreads(1)
    meta=json.loads((args.session/(args.prefix+'pair_metadata.json')).read_text())
    packet=meta.get('failure_diagnostic',{}).get('packet',{})
    if not packet and (args.session/'vision.jsonl').is_file():
        for line in (args.session/'vision.jsonl').read_text().splitlines():
            item=json.loads(line)
            if (item.get('capture_generation')==meta['front']['capture_generation'] and
                    item.get('front',{}).get('sequence')==meta['front']['sequence']):
                packet=item;break
    samples={}
    for key in ('front','rear'):
        image=cv2.imread(str(args.session/(args.prefix+key+'_latest.png')))
        if image is None:raise ValueError('missing saved '+key+' image')
        samples[key]=dict(image=image,received=meta[key]['capture_timestamp'],
                          number=meta[key]['source_frame_number'])
    prepare=[];render=[]
    for iteration in range(args.iterations+5):
        begin=time.perf_counter()
        reduced={key:small_frame(samples[key],packet.get(key)) for key in samples}
        middle=time.perf_counter()
        panels={key:render_panel(key,reduced[key],packet,dict(state='OFFLINE_REPLAY',
                reason='Saved images only; no live control'),time.monotonic()) for key in samples}
        end=time.perf_counter()
        if iteration>=5:prepare.append((middle-begin)*1000);render.append((end-middle)*1000)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    result=dict(offline_only=True,hardware_io_opened=False,iterations=args.iterations,
        session=str(args.session),prefix=args.prefix,
        source_frames={key:dict(number=samples[key]['number'],received=samples[key]['received']) for key in samples},
        includes='both image reductions and text/path overlay; excludes GUI backend, IPC and concurrent load',
        prepare_ms={key:float(value) for key,value in zip(('p50','p95','max'),np.percentile(prepare,[50,95,100]))},
        render_ms={key:float(value) for key,value in zip(('p50','p95','max'),np.percentile(render,[50,95,100]))})
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    for key,panel in panels.items():cv2.imwrite(str(args.output.with_name(args.output.stem+'_'+key+'.png')),panel)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
