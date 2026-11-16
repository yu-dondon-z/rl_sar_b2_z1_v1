"""Offline dependency/calibration/weight checks. No camera or robot initialization."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
BRIDGE_HASH = 'd391eb85d1a9eb1354b5f5fa4b35b00bb5e5fe74b5fd54475d8cd412bbad6a5b'
CALIBRATION_HASH = '525a898c45693a2b00839a6d0bc2cc3e4c9d629f6687ff80dde9a850cbfdb4e0'
CHECKPOINT_HASH = 'f1eea726c85773c3ddb71649bdae097eff6ca7693c7f36ad488fa9f4643bdb80'


def paths():
    home = Path.home()
    sdk = Path(os.environ.get('RL_SAR_UNITREE_SDK2_PYTHON', str(home/'unitree_sdk2_python')))
    rha = Path(os.environ.get('RL_SAR_RHACRACKNET_ROOT', str(home/'RHACrackNet/code')))
    checkpoint = Path(os.environ.get('RL_SAR_CRACK_CHECKPOINT', str(
        rha/'exps/rhanet_dep_CamCrack789_b2_stage1/ckpt/best_ckpt.pth')))
    return sdk, rha, checkpoint


def load_bridge():
    spec = importlib.util.spec_from_file_location('restored_vision_bridge',
        ROOT/'scripts/run_b2_builtin_front_rhacracknet_bridge.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model-smoke', action='store_true')
    args = parser.parse_args()
    sdk, rha, checkpoint = paths()
    for item in (sdk, rha, checkpoint):
        if not item.exists():
            raise RuntimeError('Missing dependency: '+str(item))
    for file, expected in [(ROOT/'scripts/run_b2_builtin_front_rhacracknet_bridge.py', BRIDGE_HASH),
                           (ROOT/'config/b2_builtin_front_camera.yaml', CALIBRATION_HASH),
                           (checkpoint, CHECKPOINT_HASH)]:
        if hashlib.sha256(file.read_bytes()).hexdigest() != expected:
            raise RuntimeError('Restored vision asset fingerprint mismatch: '+str(file))
    for directory in (sdk, rha):
        sys.path.insert(0, str(directory))
    from unitree_sdk2py.b2.front_video.front_video_client import FrontVideoClient
    from crack_inference import CrackSegmenter
    from crack_trajectory import CenterlineConfig, PinholeCamera, extract_centerline
    import numpy as np
    import torch
    bridge = load_bridge()
    config, intrinsics, transform, _, _ = bridge.load_calibration(ROOT/'config/b2_builtin_front_camera.yaml')
    camera = PinholeCamera(*intrinsics, distortion=tuple(config['intrinsics']['distortion']))
    rays = bridge.calibrated_ray_payload([[820, 1000], [850, 900], [880, 800]], camera, transform)
    assert rays['schema_version'] == 2 and rays['projection_method'] == 'calibrated_rays_pose_ground_v2'
    mask = np.zeros((360, 640), dtype=np.uint8)
    assert extract_centerline is not None and FrontVideoClient is not None
    result = dict(offline_check_pass=True, sensor_initialized=False, robot_initialized=False,
                  control_authorized=False, hardware_qualified=False, visual_control_enabled=False,
                  restored_calibration_sha256=CALIBRATION_HASH, checkpoint_sha256=CHECKPOINT_HASH,
                  python=sys.executable, torch=torch.__version__,
                  note='2D path lock only; calibrated rays still require synchronized live-pose projection')
    if args.model_smoke:
        torch.set_num_threads(2)
        torch.set_num_interop_threads(1)
        segmenter = CrackSegmenter(checkpoint, 'RHANet_dep', 'cpu', .90, input_size=(360, 640))
        image = np.zeros((360, 640, 3), dtype=np.uint8)
        segmenter.warmup(360, 640, iterations=1)
        prediction = segmenter.predict_bgr(image)
        assert prediction.probability.shape == mask.shape and np.isfinite(prediction.probability).all()
        result['synthetic_model_smoke_pass'] = True
        result['synthetic_inference_ms'] = prediction.inference_ms
        result['note'] += '; synthetic image is NOT recognition accuracy or real-time qualification'
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
