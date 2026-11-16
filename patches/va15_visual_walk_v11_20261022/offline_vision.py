"""Replay saved images through the real segmentation model. NO camera/DDS/UDP."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time

os.environ['OMP_NUM_THREADS'] = '2'
os.environ['MKL_NUM_THREADS'] = '2'
import cv2
import numpy as np
from near_tracker import NearTracker


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        raise RuntimeError('New output directory required')
    root = Path(os.environ.get('RL_SAR_RHACRACKNET_ROOT', str(Path.home()/'RHACrackNet/code')))
    weight = root/'exps/rhanet_dep_CamCrack789_b2_stage1/ckpt/best_ckpt.pth'
    if hashlib.sha256(weight.read_bytes()).hexdigest() != 'f1eea726c85773c3ddb71649bdae097eff6ca7693c7f36ad488fa9f4643bdb80':
        raise RuntimeError('Vision weight mismatch')
    sys.path.insert(0, str(root))
    import torch
    torch.set_num_threads(2);torch.set_num_interop_threads(1);cv2.setNumThreads(1)
    from crack_inference import CrackSegmenter
    network = CrackSegmenter(weight, 'RHANet_dep', 'cpu', .90, input_size=(360,640))
    image = cv2.imread(str(args.image))
    if image is None or image.shape[:2] != (1080,1920):
        raise RuntimeError('Native saved camera image required')
    network.warmup(1080,1920,1)
    tracker = NearTracker();times = [];accepted = 0;features = None;reason = ''
    for _ in range(8):
        start = time.monotonic_ns()
        result = network.predict_bgr(image)
        try:
            features = tracker.detect(image,result.mask,result.probability);accepted += 1
        except ValueError as e:
            reason = str(e)
        times.append((time.monotonic_ns()-start)/1e6)
    args.output.mkdir(parents=True)
    overlay = image.copy()
    if features is not None:
        for u,v in features['points']:
            cv2.circle(overlay,(round(u),round(v)),3,(0,230,255),-1)
    cv2.imwrite(str(args.output/'near_replay.png'),overlay)
    report = dict(saved_image=str(args.image),image_sha256=hashlib.sha256(args.image.read_bytes()).hexdigest(),
                  processing_ms=times,processing_median_ms=float(np.median(times)),
                  accepted_repetitions=accepted,last_features=features,last_rejection=reason,
                  hardware_initialized=False,recognition_accuracy_qualified=False,
                  note='Repeating one archived/possibly annotated image is NOT independent-frame lock evidence, live freshness, or real-time qualification.')
    (args.output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__ == '__main__':
    main()
