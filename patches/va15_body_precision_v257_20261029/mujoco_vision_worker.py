"""MuJoCo RGB adapter for the unchanged real-camera network and tracker."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import struct
import sys
import time

os.environ['OMP_NUM_THREADS'] = '2'
os.environ['MKL_NUM_THREADS'] = '2'
import cv2
import numpy as np
import yaml

from camera_cpu import configure_cpu_layout
from camera_near import CALIBRATION_SHA, WEIGHT_SHA, required_stable_frames
from long_tracker import LongTracker

ROOT = Path(__file__).resolve().parent
SESSION = 'c' * 32


def receive(connection, size):
    chunks = bytearray()
    while len(chunks) < size:
        part = connection.recv(size - len(chunks))
        if not part:
            raise EOFError
        chunks.extend(part)
    return bytes(chunks)


def send(connection, packet, overlay=None):
    encoded = json.dumps(packet, allow_nan=False).encode()
    pixels = b'' if overlay is None else np.ascontiguousarray(overlay[::-1, :, ::-1]).tobytes()
    connection.sendall(struct.pack('<QQ', len(encoded), len(pixels)) + encoded + pixels)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fd', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    connection = socket.socket(fileno=args.fd)
    args.output.mkdir(exist_ok=False)
    cv2.setNumThreads(1)
    import torch
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    rha = Path(os.environ.get('RL_SAR_RHACRACKNET_ROOT', str(Path.home()/'RHACrackNet/code')))
    weight = rha/'exps/rhanet_dep_CamCrack789_b2_stage1/ckpt/best_ckpt.pth'
    calibration = ROOT/'config/b2_builtin_front_camera.yaml'
    for path, expected in ((calibration, CALIBRATION_SHA), (weight, WEIGHT_SHA)):
        if hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError('Camera/weight fingerprint mismatch: ' + str(path))
    sys.path.insert(0, str(rha))
    from crack_inference import CrackSegmenter
    network = CrackSegmenter(weight, 'RHANet_dep', 'cpu', .90, input_size=(360, 640))
    layout = configure_cpu_layout(network)
    network.warmup(1080, 1920, iterations=1)
    cfg = yaml.safe_load(calibration.read_text())['intrinsics']
    native_k = np.array([[cfg['fx'], 0., cfg['cx']], [0., cfg['fy'], cfg['cy']], [0., 0., 1.]])
    distortion = np.asarray(cfg['distortion'])
    # MuJoCo supplies a centered, undistorted pinhole. Resample it to the
    # actual camera's intrinsics/distortion before running production vision.
    small_k = native_k.copy()
    small_k[:2] /= 3.
    small_k[0, 2] = (cfg['cx'] + .5)/3. - .5
    small_k[1, 2] = (cfg['cy'] + .5)/3. - .5
    xx, yy = np.meshgrid(np.arange(640), np.arange(360))
    normalized = cv2.undistortPoints(np.stack((xx, yy), axis=-1).astype(np.float32), small_k, distortion)
    map_x = (normalized[..., 0] * cfg['fy']/3. + 319.5).astype(np.float32)
    map_y = (normalized[..., 1] * cfg['fy']/3. + 179.5).astype(np.float32)
    tracker = LongTracker()
    log = (args.output/'packets.jsonl').open('x')
    send(connection, dict(ready=True, network='RHANet_dep', weight_sha256=WEIGHT_SHA,
                          calibration_sha256=CALIBRATION_SHA, layout=layout))
    try:
        while True:
            width, height, sequence, capture, phase = struct.unpack('<QQQQQ', receive(connection, 40))
            if (width, height) != (640, 360):
                raise ValueError('Expected MuJoCo 640x360 RGB frame')
            raw = np.frombuffer(receive(connection, width*height*3), dtype=np.uint8).reshape(height, width, 3)
            begin = time.perf_counter_ns()
            pinhole = raw[::-1, :, ::-1]
            distorted = cv2.remap(pinhole, map_x, map_y, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
            image = cv2.resize(distorted, (1920, 1080), interpolation=cv2.INTER_LINEAR)
            packet = dict(schema='va15_long_visual_v1', session_id=SESSION,
                          calibration_sha256=CALIBRATION_SHA, timestamp_basis='host_receive_monotonic',
                          sequence=sequence, capture_monotonic_ns=capture, valid=False,
                          image_width=1920, image_height=1080, image_received=True,
                          simulation_clock=True, controller_phase=phase)
            result = network.predict_bgr(image)
            overlay = cv2.resize(result.overlay_bgr, (640, 360))
            try:
                features = tracker.detect(image, result.mask, result.probability, capture)
                points = features.pop('points')
                packet.update(features)
                packet['valid'] = bool(features['long_valid'] and features['stable_frames'] >= required_stable_frames(phase))
                if not packet['valid']:
                    packet['reason'] = features.get('path_reason', 'Acquiring independent frames')
                polyline = np.round(np.asarray(points)/3.).astype(np.int32)
                cv2.polylines(overlay, [polyline], False, (0, 240, 255), 1)
                for key, color in (('near', (0, 0, 255)), ('look', (255, 0, 255)):
                    cv2.circle(overlay, (round(features[key+'_u']/3.), round(features[key+'_v']/3.)), 4, color, 1)
            except (ValueError, RuntimeError) as error:
                tracker.streak = 0
                packet['reason'] = str(error)
            packet['inference_ms'] = float(result.inference_ms)
            packet['near_diagnostics'] = tracker.debug
            processing = time.perf_counter_ns() - begin
            packet['processing_ms'] = processing/1.e6
            packet['publish_monotonic_ns'] = capture + processing
            if processing > 400_000_000:
                packet['valid'] = False
                packet['reason'] = 'Processing exceeded 400 ms; no guide permission'
            label = 'RHANet + LongTracker | ' + ('VALID' if packet['valid'] else packet.get('reason', 'ACQUIRING'))
            cv2.putText(overlay, label[:90], (8, 18), cv2.FONT_HERSHEY_SIMPLEX, .4,
                        (0, 230, 0) if packet['valid'] else (0, 100, 255), 1)
            send(connection, packet, overlay)
            log.write(json.dumps(packet, allow_nan=False) + '\n')
            log.flush()
            if sequence == 1 or sequence % 40 == 0 or (not packet['valid'] and sequence <= 20):
                cv2.imwrite(str(args.output/f'frame_{sequence:05d}.jpg'), image)
                cv2.imwrite(str(args.output/f'overlay_{sequence:05d}.jpg'), overlay)
    except EOFError:
        pass
    finally:
        log.close()
        connection.close()


if __name__ == '__main__':
    main()
