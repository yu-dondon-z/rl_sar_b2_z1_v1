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
from long_tracker import LongTracker
from camera_context import Context

ROOT = Path(__file__).resolve().parent
CALIBRATION_SHA = '525a898c45693a2b00839a6d0bc2cc3e4c9d629f6687ff80dde9a850cbfdb4e0'
WEIGHT_SHA = 'f1eea726c85773c3ddb71649bdae097eff6ca7693c7f36ad488fa9f4643bdb80'
NEAR_BEARING_LIMIT=.60
LOOK_BEARING_LIMIT=.65


def image_steering_in_range(near,look):
    return bool(np.isfinite(near) and np.isfinite(look) and
                abs(near)<=NEAR_BEARING_LIMIT and abs(look)<=LOOK_BEARING_LIMIT)


def required_stable_frames(phase):
    # Six independent frames for acquisition. Once explicitly following,
    # one associated good frame may recover from a single segmentation miss;
    # otherwise the six-frame restart would outlast the controller's 500 ms
    # expiry even though the same tape was already visible again.
    return 1 if phase == 2 else 6


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
    p.add_argument('--require-context', action='store_true')
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
    tracker = LongTracker();sequence = 0;last_overlay_ns = 0
    context_reader=Context();failures=0;previous_raw=None;rejection_count=0
    last_reason = None;reason_counts = {}
    args.exchange.mkdir(parents=True, exist_ok=True)
    print('V2.4 CAMERA ONLY: width-scaled walking association + supported long RGB curve; no R required. Q closes.', flush=True)
    try:
        while True:
            begin = time.monotonic_ns()
            code, encoded = client.GetImageSample()
            capture = time.monotonic_ns();sequence += 1
            packet = dict(schema='va15_long_visual_v1', session_id=args.session_id,
                          timestamp_basis='host_receive_monotonic', calibration_sha256=CALIBRATION_SHA,
                          sequence=sequence, capture_monotonic_ns=capture, valid=False,
                          image_width=1920, image_height=1080, image_received=False)
            image = None;raw_image = None
            context=context_reader.update(args.exchange/'vision_context.json',args.session_id,capture,args.require_context)
            if context['reset']:
                tracker=LongTracker();failures=0
                print(json.dumps(dict(sequence=sequence,reason='Automatic pose-phase reacquisition',phase=context['phase'])),flush=True)
            try:
                if code != 0 or not encoded:
                    raise ValueError('Camera fetch failed')
                image = cv2.imdecode(np.frombuffer(bytes(encoded), dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None:
                    raise ValueError('JPEG decode failed')
                raw_image = image.copy()
                packet['image_received'] = image.shape[:2] == (1080,1920)
                result = network.predict_bgr(image)
                inference_end = time.monotonic_ns()
                if not context['valid']:
                    raise ValueError('Controller camera context missing or stale')
                features = tracker.detect(image, result.mask, result.probability,capture)
                failures=0
                points = features.pop('points')
                uv = np.array([[[features['near_u'],features['near_v']]]], dtype=np.float64)
                xy = cv2.undistortPoints(uv,K,D).reshape(2)
                ray = rotation @ np.r_[xy,1.]
                bearing = float(np.arctan2(ray[1],ray[0]))
                look_uv=np.array([[[features['look_u'],features['look_v']]]],dtype=np.float64)
                look_xy=cv2.undistortPoints(look_uv,K,D).reshape(2)
                look_ray=rotation @ np.r_[look_xy,1.]
                look_bearing=float(np.arctan2(look_ray[1],look_ray[0]))
                angle_ok=image_steering_in_range(bearing,look_bearing)
                packet.update(features, valid=features['stable_frames'] >= required_stable_frames(context['phase']) and features['long_valid'], bearing_display_rad=bearing,
                              look_bearing_display_rad=look_bearing,image_steering_envelope_ok=angle_ok,
                              inference_ms=float(result.inference_ms), fetch_ms=(capture-begin)/1e6,
                              postprocess_ms=(time.monotonic_ns()-inference_end)/1e6)
                for u,v in points:
                    cv2.circle(image,(round(u),round(v)),3,(0,230,255) if v>=.83*1080 else (255,220,0),-1)
                label = ('PATH IN RANGE - R1+Y to start/stop' if packet['valid'] and angle_ok else
                         'PATH OUTSIDE ANGLE RANGE' if packet['valid'] else 'ACQUIRING / SHORT PATH')
                label += f" | {features['path_point_count']} pts | span={features['path_vertical_span_px']:.0f}px"
                label += f' | near={np.degrees(bearing):+.1f}/{np.degrees(NEAR_BEARING_LIMIT):.1f}deg'
                label += f' look={np.degrees(look_bearing):+.1f}/{np.degrees(LOOK_BEARING_LIMIT):.1f}deg'
                label += ' | '+{0:'POSE ACQUIRING',1:'STANDBY',2:'FOLLOWING',3:'SETTLING'}.get(context['phase'],'PREVIEW')
                if not features['long_valid']:packet['reason']=features['path_reason']
                color = (0,220,0) if packet['valid'] and angle_ok else (0,180,255)
                if features['long_valid']:
                    cv2.circle(image,(round(features['look_u']),round(features['look_v'])),7,(220,0,220),2)
            except (ValueError, RuntimeError) as e:
                tracker.streak = 0
                failures+=1
                label = 'INVALID NEAR GUIDE: '+str(e);color = (0,0,240)
                packet['reason'] = str(e)
            packet['near_diagnostics'] = tracker.debug
            packet['controller_phase']=context['phase']
            packet['publish_monotonic_ns'] = time.monotonic_ns()
            if packet['publish_monotonic_ns']-capture > 400_000_000:
                packet['valid']=False;packet['reason']='Processing exceeded 400 ms; no guide permission'
                label=packet['reason'];color=(0,0,240)
            reason = packet.get('reason','VALID' if packet['valid'] else 'Acquiring independent frames')
            reason_counts[reason] = reason_counts.get(reason,0)+1
            packet['reason_counts'] = reason_counts.copy()
            if reason != last_reason:
                print(json.dumps(dict(sequence=sequence,reason=reason,near_diagnostics=tracker.debug)),flush=True)
                # Preserve the actual before/after images of bounded rejection
                # events. Never paint these or feed overlays to the network.
                if last_reason=='VALID' and not packet['valid'] and rejection_count<20:
                    event=args.exchange/f'rejection_{sequence:06d}'
                    if previous_raw is not None:cv2.imwrite(str(event)+'_before.jpg',previous_raw)
                    if raw_image is not None:cv2.imwrite(str(event)+'_after.jpg',raw_image)
                    atomic(event.with_suffix('.json'),packet);rejection_count+=1
                last_reason = reason
            atomic(args.exchange/'near_guide.json',packet)
            if failures>=3 and context['reacquire']:
                tracker=LongTracker();failures=0
                print(json.dumps(dict(sequence=sequence,reason='Automatic stationary re-acquisition after rejection')),flush=True)
            if raw_image is not None:previous_raw=raw_image
            if image is not None:
                # Display-only diagnostics, applied AFTER prediction. Saved
                # raw frames above are never painted/fed back to the network.
                anchor_y=round(.90*1080)
                cv2.line(image,(384,anchor_y),(1536,anchor_y),(255,200,0),2)
                cv2.rectangle(image,(384,round(.57*1080)),(1536,round(.97*1080)),(180,120,0),1)
                cv2.putText(image,'NEAR ANCHOR | automatic acquisition while stationary',
                    (390,anchor_y-12),cv2.FONT_HERSHEY_SIMPLEX,.45,(255,200,0),1)
                cv2.putText(image,label,(12,30),cv2.FONT_HERSHEY_SIMPLEX,.6,color,2)
                cv2.putText(image,'R1+Y / T: START or STOP | X: stop | B: down | no R required',
                    (12,55),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,200,255),1)
                # Image encoding is diagnostic, not in the guide publication
                # path. Bound its rate so saving 1080p frames cannot dominate.
                if time.monotonic_ns()-last_overlay_ns >= 500_000_000:
                    temporary=args.exchange/f'.near_overlay_{os.getpid()}.png'
                    if cv2.imwrite(str(temporary),image):temporary.replace(args.exchange/'near_overlay.png')
                    if raw_image is not None:
                        temporary_raw=args.exchange/f'.near_raw_{os.getpid()}.jpg'
                        if cv2.imwrite(str(temporary_raw),raw_image,[cv2.IMWRITE_JPEG_QUALITY,95]):
                            temporary_raw.replace(args.exchange/'near_latest_raw.jpg')
                    last_overlay_ns = time.monotonic_ns()
                if args.display:
                    cv2.imshow('B2 V2.4.1 body guide + cached TCP',image)
                    key = cv2.waitKey(1)&255
                    if key in (27,ord('q')):break
    except KeyboardInterrupt:
        pass
    finally:
        atomic(args.exchange/'near_guide.json',dict(schema='va15_long_visual_v1',session_id=args.session_id,
            timestamp_basis='host_receive_monotonic',calibration_sha256=CALIBRATION_SHA,sequence=sequence+1,
            capture_monotonic_ns=time.monotonic_ns(),publish_monotonic_ns=time.monotonic_ns(),
            image_width=1920,image_height=1080,valid=False,reason='Camera stopped'))
        cv2.destroyAllWindows()


if __name__=='__main__':
    main()
