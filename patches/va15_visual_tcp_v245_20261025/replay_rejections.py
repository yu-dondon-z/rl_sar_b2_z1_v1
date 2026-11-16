"""Offline real-RGB rejection-pair replay; not an exact full-session replay."""
import argparse
import json
import os
from pathlib import Path
import sys
os.environ['OMP_NUM_THREADS']='2'
os.environ['MKL_NUM_THREADS']='2'
import cv2
import torch
from long_tracker import LongTracker

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('images',type=Path)
    p.add_argument('--rha',type=Path,default=Path.home()/'RHACrackNet/code')
    args=p.parse_args();sys.path.insert(0,str(args.rha))
    cv2.setNumThreads(1);torch.set_num_threads(2);torch.set_num_interop_threads(1)
    from crack_inference import CrackSegmenter
    net=CrackSegmenter(args.rha/'exps/rhanet_dep_CamCrack789_b2_stage1/ckpt/best_ckpt.pth','RHANet_dep','cpu',.90,input_size=(360,640))
    reports=[]
    for sequence in [145,215]:
        tracker=LongTracker()
        for n,side in enumerate(['before','after']):
            image=cv2.imread(str(args.images/f'rejection_{sequence:06d}_{side}.jpg'))
            output=net.predict_bgr(image)
            result=tracker.detect(image,output.mask,output.probability,1_000_000_000+n*180_000_000)
            assert result['long_valid']
        reports.append(dict(sequence=sequence,pass_=True,association_delta=tracker.debug.get('association_delta_px'),
                            association_limit=tracker.debug.get('association_limit_px')))
    print(json.dumps(dict(reports=reports,full_session_replay=False),indent=2))

if __name__=='__main__':main()
