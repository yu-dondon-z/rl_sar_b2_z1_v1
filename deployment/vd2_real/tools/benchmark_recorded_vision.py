#!/usr/bin/env python3
"""Offline saved-image timing with optional capture/control CPU load. No hardware IO.

Repeated saved images are NOT fresh sensor observations. This estimates compute
headroom, not live-camera freshness, robot stability or tracking accuracy.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import gzip
import json
import multiprocessing as mp
import os
from pathlib import Path
import sys
import threading
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'python/vision'),str(ROOT/'thirdparty/rhacracknet')]


def control_load(stop,session,root,all_cpus):
    os.sched_setaffinity(0,all_cpus)
    sys.path[:0]=[str(root/'python'),str(root/'python/vision'),str(root/'thirdparty/rhacracknet')]
    import numpy as np
    import torch
    from control import Actor,ARM
    from native_io import State
    from dual_vision_kinematics import URDFToolKinematics
    from vd1_visual_guard import GuardedVD1VisualController
    torch.set_num_threads(1)
    with gzip.open(session/'control.jsonl.gz','rt') as history:
        row=next(json.loads(line) for line in history if '"state": "ready"' in line)
    packets=[json.loads(line) for line in (session/'vision.jsonl').read_text().splitlines()]
    packet=next(p for p in packets if p.get('valid') and p['rear']['points_b']
                and min(p[0] for p in p['rear']['points_b'])<=-1.02<=max(p[0] for p in p['rear']['points_b']))
    state=State()
    for name in ('q','dq','tau','quaternion','omega'):getattr(state,name)[:]=row[name]
    actor=Actor();fk=URDFToolKinematics(root/'assets/encoder_fk.urdf')
    deadline=time.perf_counter()
    # Discard compressed log bytes in memory; preserve all original run files.
    import io
    while not stop.is_set():
        with torch.inference_mode():tcp=fk.position(torch.tensor(np.asarray(state.q)[ARM],dtype=torch.float32)[None])[0].numpy()
        controller=GuardedVD1VisualController(tcp,forward_speed=.20)
        # Historical time is confined to this offline fixture. No packet is
        # published, stamped as new, or sent to a real control loop.
        controller.update(packet,packet['front']['capture_timestamp']+.8,.02)
        actor.step(state,np.array([.20,0.,0.]),np.r_[tcp,row['tool_pose_b'][3:]])
        with gzip.GzipFile(fileobj=io.BytesIO(),mode='wb') as log:log.write(json.dumps(row).encode())
        deadline+=.02
        stop.wait(max(0.,deadline-time.perf_counter()))


def capture_load(stop,front,rear,depth,pair,front_demand=False,all_cpus=None):
    import cv2
    import numpy as np
    encoded=cv2.imencode('.jpg',front)[1]
    k=np.asarray(pair['front']['intrinsic_matrix'])
    maps=cv2.initUndistortRectifyMap(k,np.zeros(5),None,k,(front.shape[1],front.shape[0]),cv2.CV_32FC1)
    v,u=np.indices(depth.shape,dtype=np.float32)
    def front_task():
        if all_cpus is not None:os.sched_setaffinity(0,all_cpus)
        deadline=time.perf_counter()
        while not stop.is_set():
            frame=cv2.imdecode(encoded,cv2.IMREAD_COLOR)
            cv2.remap(frame,*maps,interpolation=cv2.INTER_LINEAR)
            deadline+=1/30;stop.wait(max(0.,deadline-time.perf_counter()))
    def rear_task():
        if all_cpus is not None:os.sched_setaffinity(0,all_cpus)
        deadline=time.perf_counter()
        while not stop.is_set():
            cv2.remap(rear,u,v,interpolation=cv2.INTER_LINEAR)
            cv2.remap(depth.copy(),u,v,interpolation=cv2.INTER_NEAREST)
            deadline+=1/30;stop.wait(max(0.,deadline-time.perf_counter()))
    tasks=(rear_task,) if front_demand else (front_task,rear_task)
    threads=[threading.Thread(target=task,daemon=True) for task in tasks]
    for thread in threads:thread.start()
    return threads


def main():
    global ROOT
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=ROOT,help='bundle to benchmark; never modified')
    parser.add_argument('--session',type=Path,required=True)
    parser.add_argument('--prefix',default='latest_valid_')
    parser.add_argument('--iterations',type=int,default=50)
    parser.add_argument('--load',action='store_true',help='add 30 Hz JPEG/remap and 50 Hz recorded control/actor work')
    parser.add_argument('--front-demand',action='store_true',help='decode/remap one selected JPEG per pair instead of at 30 Hz')
    parser.add_argument('--inference-cpus',default='',help='optional offline scheduling experiment, e.g. 0,2')
    parser.add_argument('--passive-numerics',action='store_true',help='use deployment PASSIVE OpenMP waiting and one BLAS thread')
    parser.add_argument('--independent',action='store_true',help='separate geometry workers with one locked network; saved front plane required')
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.passive_numerics:
        os.environ['OMP_WAIT_POLICY']='PASSIVE';os.environ['OPENBLAS_NUM_THREADS']='1'
    ROOT=args.root.resolve()
    sys.path[:0]=[str(ROOT/'python'),str(ROOT/'python/vision'),str(ROOT/'thirdparty/rhacracknet')]
    if args.iterations<2:raise ValueError('at least two iterations required')
    if args.output.exists():raise FileExistsError(args.output)
    all_cpus=set(os.sched_getaffinity(0))
    if args.inference_cpus:os.sched_setaffinity(0,{int(v) for v in args.inference_cpus.split(',')})
    import cv2,numpy as np,torch
    from realtime_segmenter import RealtimeSegmenter
    from realtime_perception import ObservableRearPerception
    from dual_camera_perception import PerceptionConfig
    cv2.setNumThreads(1);torch.set_num_threads(2)
    metadata=json.loads((args.session/(args.prefix+'pair_metadata.json')).read_text())
    front=cv2.imread(str(args.session/(args.prefix+'front_latest.png')))
    rear=cv2.imread(str(args.session/(args.prefix+'rear_latest.png')))
    depth=cv2.imread(str(args.session/(args.prefix+'rear_depth_mm.png')),cv2.IMREAD_UNCHANGED)
    if front is None or rear is None or depth is None:raise ValueError('saved capture is incomplete')
    depth=depth.astype(np.float32)*.001;depth[depth<=0]=np.nan
    segmenter=RealtimeSegmenter(ROOT/'assets/rhanet.pth',device='cpu',threshold=.90,input_size=(360,640))
    segmenter.warmup(360,640,2)
    encoded=cv2.imencode('.jpg',front)[1]
    k=np.asarray(metadata['front']['intrinsic_matrix'])
    maps=cv2.initUndistortRectifyMap(k,np.zeros(5),None,k,(front.shape[1],front.shape[0]),cv2.CV_32FC1)
    perception=ObservableRearPerception(PerceptionConfig(rhacracknet_root=str(ROOT/'thirdparty/rhacracknet')),segmenter)
    pool=None
    if args.independent:
        from independent_vision import LockedSegmenter
        if metadata['front'].get('ground_plane_b') is None:
            raise ValueError('independent offline test requires the recorded front projection plane')
        lock=threading.Lock()
        detectors=[ObservableRearPerception(PerceptionConfig(rhacracknet_root=str(ROOT/'thirdparty/rhacracknet')),
                   LockedSegmenter(segmenter,lock)) for _ in range(2)]
        pool=ThreadPoolExecutor(max_workers=2)
    def independent_call(name,start):
        if name=='front':result=detectors[0].process_front(metadata['front'],front)
        else:result=detectors[1].process_rear(metadata['rear'],rear,depth)
        return result,time.perf_counter()-start
    ctx=mp.get_context('spawn');stop=ctx.Event();process=None;threads=[];rows=[]
    try:
        if args.load:
            process=ctx.Process(target=control_load,args=(stop,args.session,ROOT,all_cpus),daemon=True);process.start()
            threads=capture_load(stop,front,rear,depth,metadata,args.front_demand,all_cpus)
            # Allow lazy actor compilation before measuring sustained load.
            time.sleep(3.)
        with args.output.open('x') as history:
            for iteration in range(args.iterations):
                start=time.perf_counter()
                if args.front_demand:
                    # Deliberately discarded: same already-rectified saved
                    # image remains the perception input in both variants.
                    decoded=cv2.imdecode(encoded,cv2.IMREAD_COLOR)
                    cv2.remap(decoded,*maps,interpolation=cv2.INTER_LINEAR)
                if args.independent:
                    tasks=[pool.submit(independent_call,name,start) for name in ('front','rear')]
                    f,front_elapsed=tasks[0].result();r,rear_elapsed=tasks[1].result()
                else:
                    r=perception.process_rear(metadata['rear'],rear,depth);middle=time.perf_counter()
                    f=dict(valid=False,reason='no rear ground plane')
                    if r.get('ground_plane_b') is not None:
                        f=perception.process_front(dict(metadata['front'],ground_plane_b=r['ground_plane_b']),front)
                end=time.perf_counter()
                row=dict(iteration=iteration,total_ms=1000*(end-start),
                    rear_ms=1000*(rear_elapsed if args.independent else middle-start),
                    front_ms=1000*(front_elapsed if args.independent else end-middle),
                    front_valid=f['valid'],rear_valid=r['valid'],
                    rear_neural_ms=r.get('inference_ms'),front_neural_ms=f.get('inference_ms'))
                rows.append(row);history.write(json.dumps(row)+'\n');history.flush()
                if process is not None and not process.is_alive():raise RuntimeError('offline load process exited')
        total=np.array([row['total_ms'] for row in rows])
        # Two sequential completions bound the lifetime of the previous result.
        # Acquisition/delivery time must be added in a separate live test.
        summary=dict(offline_only=True,robot_or_camera_io_opened=False,recorded_load=args.load,front_demand=args.front_demand,
            independent_workers=args.independent,
            stream_timing_scope=('completion since concurrent dispatch' if args.independent else
                                 'individual serial stage duration, not publication latency'),
            inference_cpus=args.inference_cpus,
            prefix=args.prefix,iterations=len(rows),valid_pairs=sum(r['front_valid'] and r['rear_valid'] for r in rows),
            processing_ms=dict(p50=float(np.percentile(total,50)),p95=float(np.percentile(total,95)),max=float(total.max())),
            adjacent_processing_sum_ms_max=float(np.max(total[:-1]+total[1:])),
            stream_completion_ms={name:dict(p50=float(np.percentile([r[name+'_ms'] for r in rows],50)),
                p95=float(np.percentile([r[name+'_ms'] for r in rows],95))) for name in ('front','rear')},
            limitations='recorded images; no camera acquisition, device alignment, SDK IO or actual robot motion')
        args.output.with_suffix('.summary.json').write_text(json.dumps(summary,indent=2)+'\n')
        print(json.dumps(summary,indent=2),flush=True)
    finally:
        stop.set()
        if pool is not None:pool.shutdown(wait=True)
        for thread in threads:thread.join(timeout=2.)
        if process is not None:
            process.join(timeout=5.)
            if process.is_alive():process.terminate();process.join(timeout=2.)


if __name__=='__main__':main()
