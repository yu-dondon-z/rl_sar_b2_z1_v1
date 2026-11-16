"""B2 camera + RHACrackNet near-tape packets; never imports motor control."""
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
import yaml
from near_tracker import NearTracker

ROOT = Path(__file__).resolve().parent
CALIBRATION_SHA = '525a898c45693a2b00839a6d0bc2cc3e4c9d629f6687ff80dde9a850cbfdb4e0'
WEIGHT_SHA = 'f1eea726c85773c3ddb71649bdae097eff6ca7693c7f36ad488fa9f4643bdb80'


def atomic(path, value):
    temporary = path.with_name('.' + path.name + '.' + str(os.getpid()))
    temporary.write_text(json.dumps(value, allow_nan=False))
    temporary.replace(path)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('interface')
    p.add_argument('--exchange', type=Path, required=True)
    p.add_argument('--session-id', required=True)
    p.add_argument('--display', action='store_true')
    args = p.parse_args()
    cv2.setNumThreads(1)
    home = Path.home()
    rha = Path(os.environ.get('RL_SAR_RHACRACKNET_ROOT', str(home/'RHACrackNet/code')))
    sdk = Path(os.environ.get('RL_SAR_UNITREE_SDK2_PYTHON', str(home/'unitree_sdk2_python')))
    calibration = ROOT/'config/b2_builtin_front_camera.yaml'
    weight = rha/'exps/rhanet_dep_CamCrack789_b2_stage1/ckpt/best_ckpt.pth'
    for path, expected in ((calibration, CALIBRATION_SHA), (weight, WEIGHT_SHA)):
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError('Camera/weight fingerprint mismatch: ' + str(path))
    cfg = yaml.safe_load(calibration.read_text())
    intr = cfg['intrinsics']
    K = np.array([[intr['fx'],0,intr['cx']],[0,intr['fy'],intr['cy']],[0,0,1]], dtype=np.float64)
    D = np.array(intr['distortion'], dtype=np.float64)
    rotation = np.array(cfg['T_base_camera'], dtype=np.float64)[:3,:3]
    for directory in (rha, sdk):
        sys.path.insert(0, str(directory))
    import torch
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    from crack_inference import CrackSegmenter
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    from unitree_sdk2py.b2.front_video.front_video_client import FrontVideoClient
    # Only the camera RPC client, no lowcmd publisher or factory release.
    ChannelFactoryInitialize(0, args.interface)
    client = FrontVideoClient();client.SetTimeout(1.0);client.Init()
    network = CrackSegmenter(weight, 'RHANet_dep', 'cpu', .90, input_size=(360,640))
    network.warmup(1080,1920,iterations=1)
    tracker = NearTracker();sequence = 0;last_overlay_ns = 0
    args.exchange.mkdir(parents=True, exist_ok=True)
    print('NEAR CAMERA ONLY: bearing guide; no metric path, TCP updates, or motor commands. R resets association; Q closes.', flush=True)
    try:
        while True:
            begin = time.monotonic_ns()
            code, encoded = client.GetImageSample()
            capture = time.monotonic_ns();sequence += 1
            packet = dict(schema='va15_near_visual_v1', session_id=args.session_id,
                          timestamp_basis='host_receive_monotonic', calibration_sha256=CALIBRATION_SHA,
                          sequence=sequence, capture_monotonic_ns=capture, valid=False,
                          image_width=1920, image_height=1080, image_received=False)
            image = None
            try:
                if code != 0 or not encoded:
                    raise ValueError('Camera fetch failed')
                image = cv2.imdecode(np.frombuffer(bytes(encoded), dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError('JPEG decode failed')
                packet['image_received'] = image.shape[:2] == (1080,1920)
                result = network.predict_bgr(image)
                inference_end = time.monotonic_ns()
                features = tracker.detect(image, result.mask, result.probability)
                points = features.pop('points')
                uv = np.array([[[features['near_u'],features['near_v']]]], dtype=np.float64)
                xy = cv2.undistortPoints(uv,K,D).reshape(2)
                ray = rotation @ np.r_[xy,1.]
                bearing = float(np.arctan2(ray[1],ray[0]))
                packet.update(features, valid=features['stable_frames'] >= 6, bearing_display_rad=bearing,
                              inference_ms=float(result.inference_ms), fetch_ms=(capture-begin)/1e6,
                              postprocess_ms=(time.monotonic_ns()-inference_end)/1e6)
                for u,v in points:
                    cv2.circle(image,(round(u),round(v)),3,(0,230,255),-1)
                label = ('NEAR LINE' if packet['valid'] else 'ACQUIRING NEAR LINE')
                if abs(bearing) > .06:
                    label += ' | NOT CENTERED - NO START'
                label += f' | bearing={bearing:+.3f} rad | TCP UNCHANGED'
                color = (0,220,0) if packet['valid'] and abs(bearing)<=.06 else (0,180,255)
            except (ValueError, RuntimeError) as e:
                tracker.streak = 0
                label = 'INVALID NEAR GUIDE: '+str(e);color = (0,0,240)
                packet['reason'] = str(e)
            packet['publish_monotonic_ns'] = time.monotonic_ns()
            if packet['publish_monotonic_ns']-capture > 400_000_000:
                packet['valid']=False;packet['reason']='Processing exceeded 400 ms; no guide permission'
                label=packet['reason'];color=(0,0,240)
            atomic(args.exchange/'near_guide.json',packet)
            if image is not None:
                cv2.putText(image,label,(12,30),cv2.FONT_HERSHEY_SIMPLEX,.6,color,2)
                # Image encoding is diagnostic, not in the guide publication
                # path. Bound its rate so saving 1080p frames cannot dominate.
                if time.monotonic_ns()-last_overlay_ns >= 500_000_000:
                    temporary=args.exchange/f'.near_overlay_{os.getpid()}.png'
                    if cv2.imwrite(str(temporary),image):temporary.replace(args.exchange/'near_overlay.png')
                    last_overlay_ns = time.monotonic_ns()
                if args.display:
                    cv2.imshow('B2 near visual guide - not TCP trajectory',image)
                    key = cv2.waitKey(1)&255
                    if key in (27,ord('q')):break
                    if key in (ord('r'),ord('R')):
                        # Explicit reacquisition after standing. It emits an
                        # INVALID immediately; a running request stops/latches,
                        # never silently jumps from the old line to a new one.
                        tracker = NearTracker();sequence += 1
                        packet.update(valid=False,sequence=sequence,reason='Operator reset near association',
                                      publish_monotonic_ns=time.monotonic_ns())
                        atomic(args.exchange/'near_guide.json',packet)
    except KeyboardInterrupt:
        pass
    finally:
        atomic(args.exchange/'near_guide.json',dict(schema='va15_near_visual_v1',session_id=args.session_id,
            timestamp_basis='host_receive_monotonic',calibration_sha256=CALIBRATION_SHA,sequence=sequence+1,
            capture_monotonic_ns=time.monotonic_ns(),publish_monotonic_ns=time.monotonic_ns(),
            image_width=1920,image_height=1080,valid=False,reason='Camera stopped'))
        cv2.destroyAllWindows()


if __name__=='__main__':
    main()
