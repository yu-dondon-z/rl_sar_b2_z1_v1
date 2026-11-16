#!/usr/bin/env python3
"""Capture/infer only: no Z1 controller, motor channel, takeover or commands."""
import argparse
from datetime import datetime
import json
import multiprocessing as mp
import os
os.environ['OMP_WAIT_POLICY']='PASSIVE'
os.environ['OPENBLAS_NUM_THREADS']='1'
from pathlib import Path
import queue
import sys
import time
ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'python/vision'),str(ROOT/'thirdparty/rhacracknet')]
from cameras import worker,rear_covers_work_point
from vision_start import packet_status


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface',default='eth0')
    parser.add_argument('--seconds',type=float,default=60.)
    args=parser.parse_args()
    if args.seconds<=0:raise ValueError('seconds must be positive')
    destination=ROOT/'runs'/('vision_only_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f'))
    destination.mkdir(parents=True,exist_ok=False)
    ctx=mp.get_context('spawn');output=ctx.Queue(maxsize=1);stop=ctx.Event();generation=time.time_ns()
    process=ctx.Process(target=worker,args=(output,stop,args.interface,str(destination),generation,False,False,'cpu'),daemon=True)
    last=None;latest=None;records=[];start=last_print=time.monotonic()
    print('Camera/perception only; no motor control or Z1 controller. Logs: '+str(destination),flush=True)
    process.start()
    try:
        with (destination/'stream_timing.jsonl').open('w') as history:
            while time.monotonic()-start<args.seconds:
                try:item=output.get(timeout=.02)
                except queue.Empty:item=None
                now=time.monotonic()
                if item:
                    if item.get('kind')=='fatal':raise RuntimeError(item['reason'])
                    if item.get('kind')=='packet':
                        latest=item
                        row=dict(monotonic=now,processing_ms=item['processing_ms'],
                            delivery_gap_s=None if last is None else now-last,
                            front_age_s=now-item['front']['capture_timestamp'],
                            rear_age_s=now-item['rear']['capture_timestamp'],
                            detection_valid=item['valid'],rear_covers_work_point=rear_covers_work_point(item['rear']))
                        records.append(row);history.write(json.dumps(row)+'\n');history.flush();last=now
                    else:latest=item
                if now-last_print>=1:
                    status=packet_status(latest,generation,now)
                    if latest and latest.get('kind')=='status':status['reason']=latest.get('reason',status['reason'])
                    print(json.dumps(status,ensure_ascii=False),flush=True);last_print=now
                if not process.is_alive():raise RuntimeError('camera worker exited')
    except KeyboardInterrupt:pass
    finally:
        stop.set();process.join(timeout=6.)
        if process.is_alive():process.terminate();process.join(timeout=2.)
        import numpy as np
        summary=dict(camera_only=True,motor_control_started=False,packet_count=len(records),
            detection_valid_count=sum(r['detection_valid'] for r in records),
            target_covering_count=sum(r['rear_covers_work_point'] for r in records),
            note='host receipt age only; stationary camera test is not a walking/accuracy acceptance')
        for key in ('processing_ms','delivery_gap_s','front_age_s','rear_age_s'):
            values=[r[key] for r in records if r[key] is not None]
            if values:summary[key]=dict(p50=float(np.percentile(values,50)),p95=float(np.percentile(values,95)),max=max(values))
        (destination/'stream_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
        print(json.dumps(summary,ensure_ascii=False,indent=2),flush=True)


if __name__=='__main__':main()
