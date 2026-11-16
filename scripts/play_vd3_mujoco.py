#!/usr/bin/env python3
"""Run the selected VD3 actor in a local MuJoCo scene.

The actor receives the same 70 values used by VD3: base command, the fixed
base-frame TCP target, IMU values, joint encoders and the previous raw action.
World pose, contact and seam measurements are written to a separate log only.
This is a sim2sim check; it does not connect to a robot, board or camera feed.
"""

from __future__ import annotations

import argparse
import gzip
import json
import math
import os
from pathlib import Path
import signal
import sys
import tempfile
import time

import numpy as np

from vd3_mujoco_policy import VD3_POLICY


DEPLOY_ROOT = Path(__file__).resolve().parents[1]
VD3_ROOT = Path('/home/dong/b2_z1_VD2')
DEFAULT_POLICY = VD3_POLICY
sys.path.insert(0, str(DEPLOY_ROOT / 'scripts'))
sys.path.insert(0, str(Path('/home/dong/b2_z1/scripts/vision')))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy', type=Path, default=DEFAULT_POLICY)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--speed', type=float, default=0.2)
    parser.add_argument('--curvature', type=float, default=0.05,
                        help='Rendered road curvature in 1/m; the road is long enough for this run.')
    parser.add_argument('--target', nargs=3, type=float, default=(-1.02, 0.0, -0.30))
    parser.add_argument('--physics-dt', type=float, choices=(0.001, 0.005), default=0.005)
    parser.add_argument('--output-root', type=Path, default=Path('/dev/shm/vd3_mujoco_runs'))
    parser.add_argument('--cache-dir', type=Path, default=Path('/dev/shm/vd3_mujoco_cache'))
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    if not 0.0 < args.duration <= 600.0 or not 0.0 < args.speed <= 1.0:
        parser.error('duration and speed are outside the supported range')
    if not 0.0 <= args.curvature <= 0.15:
        parser.error('curvature must be in [0, 0.15] 1/m')
    return args


def target_pose(position):
    from vd1_control import _tool_quaternion
    return np.concatenate((np.asarray(position, dtype=np.float64), _tool_quaternion()))


def actor_from_jit(path):
    import torch
    actor = torch.jit.load(str(path), map_location='cpu').eval()
    with torch.inference_mode():
        shape = tuple(actor(torch.zeros(1, 70)).shape)
    if shape != (1, 18):
        raise ValueError(f'VD3 policy must map (1,70) to (1,18), got {shape}')
    return actor


def validate_policy(path):
    if not path.is_file():
        raise FileNotFoundError(path)
    actor = actor_from_jit(path)
    return actor


def main():
    args = parse_args()
    if args.headless:
        os.environ.setdefault('MUJOCO_GL', 'egl')
    import mujoco
    import mujoco.viewer
    import torch
    from vd1_mujoco_contract import VD1MujocoAdapter, ACTOR_JOINT_NAMES
    from vd1_mujoco_scene import build_scene, validate_vd3_model
    from vd1_mujoco_velocity_impulse import VelocityImpulseConstraints
    from vd1_control import build_actor_observation

    actor = validate_policy(args.policy.resolve())
    stamp = time.strftime('%Y%m%d_%H%M%S')
    args.output_root.mkdir(parents=True, exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix=f'vd3_{stamp}_', dir=str(args.output_root.resolve())))
    scene = build_scene(args.curvature, args.cache_dir / str(output.name), timestep=args.physics_dt,
                        tool_markers=True, road_surface='tape', robot_version='vd3')
    model = mujoco.MjModel.from_xml_path(str(scene))
    validation = validate_vd3_model(model)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('vd1_default').id)
    mujoco.mj_forward(model, data)
    adapter = VD1MujocoAdapter(model, physx_implicit_position=args.physics_dt == 0.005)
    velocity_constraint = VelocityImpulseConstraints(model, adapter.joint_ids18, adapter.velocity_limit18)
    target = target_pose(args.target)
    command = np.asarray((args.speed, 0.0, 0.0), dtype=np.float64)
    previous_action = np.zeros(18, dtype=np.float32)
    stopped = [False]
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopped.__setitem__(0, True))

    trace = gzip.open(str(output / 'actor_trace.jsonl.gz'), 'wt', compresslevel=3)
    truth = gzip.open(str(output / 'evaluation_truth.jsonl.gz'), 'wt', compresslevel=3)
    viewer = None
    if not args.headless:
        viewer = mujoco.viewer.launch_passive(model, data)
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 3.4, 135.0, -25.0
    print(f'[VD3 MuJoCo] output: {output}', flush=True)
    print(f'[VD3 MuJoCo] policy: {args.policy.resolve()}', flush=True)
    print(f'[VD3 MuJoCo] speed={args.speed:.3f} m/s curvature={args.curvature:.3f}/m target={target[:3].tolist()}', flush=True)
    print('[VD3 MuJoCo] Close the viewer or Ctrl-C to exit.', flush=True)

    def viewer_running():
        return viewer is None or viewer.is_running()

    try:
        steps = int(math.ceil(args.duration / 0.02))
        for step in range(steps):
            if stopped[0] or not viewer_running():
                break
            tick = time.monotonic()
            measured = adapter.read_proprioception(data)
            observation = build_actor_observation(command, target, measured['gyro_b'], measured['gravity_b'],
                                                  measured['q'], measured['qd'], adapter.default_q18, previous_action)
            with torch.inference_mode():
                action = actor(torch.from_numpy(observation).unsqueeze(0)).numpy()[0]
            if action.shape != (18,) or not np.isfinite(action).all():
                raise ValueError('nonfinite VD3 actor action')
            joint_target = adapter.default_q18 + 0.25 * action
            for _ in range(4):
                mujoco.mj_forward(model, data)
                adapter.apply_targets(data, joint_target)
                for _ in range(max(1, int(round(0.005 / args.physics_dt)))):
                    velocity_constraint.step(data)
            mujoco.mj_forward(model, data)
            base = model.body('base_link').id
            tcp = model.site('tool_tcp').id
            rotation = data.xmat[base].reshape(3, 3)
            tilt = math.degrees(math.acos(float(np.clip(rotation[2, 2], -1., 1.))))
            row = dict(step=step, sim_time_s=float(data.time), base_command_b=command.tolist(),
                       tool_target_b=target.tolist(), observation=observation.tolist(), action=action.tolist(),
                       joint_target=joint_target.tolist(), privileged_actor_inputs=[])
            trace.write(json.dumps(row, allow_nan=False) + '\n')
            velocity = np.empty(6)
            mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_XBODY, base, velocity, 1)
            truth.write(json.dumps(dict(step=step, sim_time_s=float(data.time), base_position_w=data.xpos[base].tolist(),
                                       body_velocity_b=velocity[3:].tolist(),
                                       tcp_position_w=data.site_xpos[tcp].tolist(), tilt_deg=tilt,
                                       evaluation_only=True, truth_never_returned_to_actor=True)) + '\n')
            previous_action = action.copy()
            if viewer is not None:
                with viewer.lock():
                    viewer.cam.lookat[:] = data.xpos[base]
                viewer.sync()
            if step == 0 or (step + 1) % 250 == 0:
                print('[VD3 MuJoCo] step=%d sim=%.2fs vx=%.3f tilt=%.2f deg' %
                      (step + 1, data.time, velocity[3], tilt), flush=True)
            if viewer is not None:
                time.sleep(max(0.0, 0.02 - (time.monotonic() - tick)))
    finally:
        trace.close()
        truth.close()
        if viewer is not None:
            viewer.close()
        summary = dict(policy=str(args.policy.resolve()), policy_actor='70_to_18_jit',
                       robot='VD3 zero-added-mass gun', validation=validation,
                       speed_mps=args.speed, curvature_per_m=args.curvature, target_b=target.tolist(),
                       scene=str(scene), output=str(output), actor_inputs=['base command', 'fixed TCP target',
                       'IMU gyro', 'projected gravity', 'joint encoders', 'previous raw action'],
                       privileged_actor_inputs=[], no_robot_io=True)
        (output / 'runtime_contract.json').write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
    print(f'[VD3 MuJoCo] logs retained: {output}', flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
