"""Compare native measured TCP FK to MuJoCo model sites, without robot clients.

Example:
  python check_tcp_fk.py --fk-binary build/test_tcp_fk --samples 1000

This is an exact model geometry check, not physical tool/odometry calibration.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import mujoco
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fk-binary', type=Path, required=True)
    parser.add_argument('--scene', type=Path, default=Path(
        '/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml'))
    parser.add_argument('--samples', type=int, default=1000)
    parser.add_argument('--seed', type=int, default=246)
    args = parser.parse_args()
    if args.samples < 10 or args.samples > 100000:
        parser.error('--samples must be between 10 and 100000')
    binary, scene = args.fk_binary.resolve(strict=True), args.scene.resolve(strict=True)
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, 'joint' + str(i))
                 for i in range(1, 7)]
    site = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, 'ee_site')
    dummy = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'dummy_target')
    if min(joint_ids + [site, dummy]) < 0:
        raise RuntimeError('VA15 model must provide six Z1 joints, ee_site and dummy_target')
    rng = np.random.default_rng(args.seed)
    bounds = model.jnt_range[joint_ids]
    cases = [np.zeros(6), np.array([0., 2.18, -1.60, -.90, 0., 0.]),
             np.array([0., 2.24, -1.27, -1.33, .04, 0.]),
             bounds[:, 0].copy(), bounds[:, 1].copy()]
    cases.extend(rng.uniform(bounds[:, 0], bounds[:, 1], size=(args.samples, 6)))
    inputs, expected_base, expected_world, expected_dummy = [], [], [], []
    for arm in cases:
        position = rng.uniform([-10., -10., .3], [10., 10., .8])
        quaternion = rng.normal(size=4)
        quaternion /= np.linalg.norm(quaternion)
        data.qpos[:] = model.qpos0
        data.qpos[:3] = 0.
        data.qpos[3:7] = [1., 0., 0., 0.]
        for joint, value in zip(joint_ids, arm):
            data.qpos[model.jnt_qposadr[joint]] = value
        mujoco.mj_forward(model, data)
        expected_base.append(data.site_xpos[site].copy())
        data.qpos[:3], data.qpos[3:7] = position, quaternion
        mujoco.mj_forward(model, data)
        expected_world.append(data.site_xpos[site].copy())
        expected_dummy.append(data.xpos[dummy].copy())
        inputs.append(' '.join(format(float(x), '.17g')
                               for x in np.r_[arm, position, quaternion]))
    result = subprocess.run([str(binary), '--cli'], input='\n'.join(inputs) + '\n',
                            text=True, capture_output=True, check=True, timeout=60)
    rows = result.stdout.strip().splitlines()
    if len(rows) != len(inputs) or any(row == 'INVALID' for row in rows):
        raise RuntimeError('Native FK rejected a legal model pose or emitted the wrong number of rows')
    native = np.array([[float(x) for x in row.split()] for row in rows])
    if native.shape != (len(inputs), 6) or not np.isfinite(native).all():
        raise RuntimeError('Native FK result must contain six finite values per sample')
    base_error = np.linalg.norm(native[:, :3] - np.asarray(expected_base), axis=1)
    world_error = np.linalg.norm(native[:, 3:] - np.asarray(expected_world), axis=1)
    site_dummy_error = np.linalg.norm(np.asarray(expected_world) - np.asarray(expected_dummy), axis=1)
    passed = bool(max(base_error.max(), world_error.max(), site_dummy_error.max()) < 1.e-10)
    report = dict(
        pass_check=passed, samples=len(inputs), randomized_samples=args.samples, seed=args.seed,
        max_base_fk_error_m=float(base_error.max()), max_world_fk_error_m=float(world_error.max()),
        max_site_dummy_error_m=float(site_dummy_error.max()), policy_arm_indices=[8, 13, 14, 15, 16, 17],
        virtual_tcp_in_link6_m=[.105, 0., -.820], binary=str(binary),
        binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest(), model_scene=str(scene),
        scene_sha256=hashlib.sha256(scene.read_bytes()).hexdigest(),
        exact_model_geometry_only=True, physical_tool_calibrated=False, hardware_qualified=False,
        robot_clients_initialized=False)
    print(json.dumps(report, indent=2))
    raise SystemExit(0 if passed else 2)


if __name__ == '__main__':
    main()
