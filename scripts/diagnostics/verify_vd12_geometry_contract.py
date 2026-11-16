#!/usr/bin/env python3
"""Compile a new MuJoCo cache and verify shared VD12 geometry with mj_forward.

No mj_step, actor inference, optimizer, renderer or robot IO is used. Historical
Isaac keyframes contain only six actual arm angles: nominal legs are explicitly
retained rather than presented as historical whole-robot encoder measurements.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import tempfile

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'scripts'))

import numpy as np
import torch

from vd12_geometry_guard import ASSETS, URDF, MujocoGunGeometryGuard, load_pure_module
from vd1_mujoco_scene import (ARM_JOINT_NAMES, EXPECTED_JOINT_NAMES, DEFAULT_JOINT_POSITION,
                             build_scene, quaternion_matrix, validate_vd10_model)


DEFAULT_ISAAC_MANIFEST = Path('/home/dong/b2_z1_VD2/outputs/vd12_guard_probe_20261114/probe_20261114_144644_vqgpf9_x/manifest.json')
DEFAULT_KEYFRAMES = Path('/home/dong/b2_z1_VD2/outputs/vd11_existing_pose_geometry_audit_20261114_486e10c3/env0_actual_keyframes.json')
RESET_ARM = (0., .66647356, -1.25178768, .88531411, 0., 0.)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def model_fingerprint(model):
    names = ('body_pos', 'body_quat', 'jnt_range', 'geom_contype', 'geom_conaffinity',
             'actuator_gainprm', 'actuator_biasprm', 'dof_damping', 'dof_frictionloss')
    return {name: hashlib.sha256(np.asarray(getattr(model, name)).tobytes()).hexdigest() for name in names}


def load_cases(path):
    source = json.loads(path.read_text(encoding='utf-8'))
    frames = {row['physics_step']: row for row in source['keyframes']}
    nominal = dict(DEFAULT_JOINT_POSITION)
    nominal.update(zip(ARM_JOINT_NAMES, RESET_ARM))
    cases = [dict(label='nominal_reset', joint_positions=dict(nominal), root_position_m=[0., 0., .5],
                  root_quaternion_wxyz=[1., 0., 0., 0.], source='Authored VD10/VD11 nominal reset',
                  actual_historical_arm=False)]
    for step in (248, 301, 400):
        row = frames[step]
        positions = dict(nominal)
        positions.update(zip(ARM_JOINT_NAMES, row['sample_arm_joint_rad']))
        cases.append(dict(label='isaac_env0_arm_at_%.3fs' % row['elapsed_physics_time_s'],
                          joint_positions=positions,
                          root_position_m=[0., 0., row['sample_root_height_m']],
                          root_quaternion_wxyz=row['sample_root_quat_wxyz'], actual_historical_arm=True,
                          source=str(path.resolve()), source_physics_step=step,
                          source_time_s=row['elapsed_physics_time_s'],
                          measurement_scope='Actual env0 six-arm-joint q and base height/attitude, nominal unrecorded leg q. Root XY set to zero by common-world-translation invariance; this is not a complete recorded robot state.'))
    return cases


def verify_case(mujoco, model, data, case, adapters, cpu):
    mujoco.mj_resetData(model, data)
    data.qpos[:3] = case['root_position_m']
    data.qpos[3:7] = case['root_quaternion_wxyz']
    for name, value in case['joint_positions'].items():
        joint = model.joint(name)
        data.qpos[model.jnt_qposadr[joint.id]] = value
    before_qpos, before_qvel, before_time = data.qpos.copy(), data.qvel.copy(), float(data.time)
    mujoco.mj_forward(model, data)
    np.testing.assert_array_equal(data.qpos, before_qpos)
    np.testing.assert_array_equal(data.qvel, before_qvel)
    if float(data.time) != before_time:
        raise AssertionError('Static mj_forward unexpectedly advanced simulation time')
    outputs = {name: adapter.evaluate(data) for name, adapter in adapters.items()}
    ordered_q = [case['joint_positions'][name] for name in EXPECTED_JOINT_NAMES]
    reference = cpu.audit(EXPECTED_JOINT_NAMES, ordered_q)
    reference_pairs = {pair['other_collision']: pair for pair in reference['pair_results']}
    if len(reference_pairs) != 44:
        raise AssertionError('Independent source scope no longer contains the expected 44 pairs')
    pair_evidence, errors = [], {name: [] for name in outputs}
    for pair_index, pair_name in enumerate(outputs['runtime_all_float64']['pair_names']):
        raw = reference_pairs[pair_name]['clearance_lower_bound_m']
        expected = raw - adapters['runtime_all_float64'].profile['numerical_guard_m']
        row = dict(pair_index=pair_index, pair_name=pair_name, independent_cpu_raw_lower_bound_m=raw,
                   expected_after_shared_numerical_guard_m=expected)
        for name, output in outputs.items():
            other_index = output['pair_names'].index(pair_name)
            actual = output['clearance_lower_bound_m'][other_index]
            error = abs(actual - expected)
            errors[name].append(error)
            row[name + '_lower_bound_m'] = actual
            row[name + '_absolute_error_m'] = error
            tolerance = 2e-6 if 'float32' in name else 1e-9
            if error > tolerance:
                raise AssertionError('%s %s differs by %.12g m' % (name, pair_name, error))
        pair_evidence.append(row)
    root_r = quaternion_matrix(case['root_quaternion_wxyz'])
    root_p = np.array(case['root_position_m'])
    expected_endpoints = np.asarray(reference['tube_endpoints_root_m']) @ root_r.T + root_p
    np.testing.assert_allclose(outputs['runtime_all_float64']['tube_endpoints_w_m'], expected_endpoints, atol=1e-10, rtol=1e-9)
    transforms = cpu._fk(case['joint_positions'])
    body_errors = []
    for body_id in range(1, model.nbody):
        name = model.body(body_id).name
        rotation, position = transforms[name]
        position_error = float(np.linalg.norm(data.xpos[body_id] - (root_r @ position + root_p)))
        rotation_error = float(np.linalg.norm(data.xmat[body_id].reshape(3, 3) - root_r @ rotation))
        body_errors.append(dict(name=name, position_error_m=position_error, rotation_matrix_error=rotation_error))
    if max(row['position_error_m'] for row in body_errors) > 1e-9 or max(row['rotation_matrix_error'] for row in body_errors) > 1e-9:
        raise AssertionError('Compiled MuJoCo body-link FK does not match independent source FK')
    if len({output['conservative_keepout_violated'] for output in outputs.values()}) != 1:
        raise AssertionError('Runtime/full/fixed-child registries disagree on conservative decision')
    return dict(case=case, runtime_engine_fk='MuJoCo data.xpos/xquat/xmat after mj_forward; not synthesized FK poses',
                physics_advanced=False, mj_step_calls=0, simulation_time_before_s=before_time,
                simulation_time_after_s=float(data.time), state_arrays_unchanged=True,
                pair_count=44, max_pair_error_m={name: max(values) for name, values in errors.items()},
                independent_cpu_status=reference['status'],
                independent_cpu_confirmed_intersections=reference['confirmed_intersections'],
                adapter_results=outputs, pair_evidence=pair_evidence, runtime_body_fk_errors=body_errors)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--isaac-manifest', type=Path, default=DEFAULT_ISAAC_MANIFEST)
    parser.add_argument('--keyframes', type=Path, default=DEFAULT_KEYFRAMES)
    args = parser.parse_args()
    import mujoco

    manifest = json.loads(args.isaac_manifest.read_text(encoding='utf-8'))
    actual_isaac_names = manifest['geometry_registry']['body_names']
    if manifest['geometry_registry']['source_urdf_sha256'] != sha256(URDF):
        raise ValueError('Isaac runtime registry and current MuJoCo source URDF differ')
    output_root = ROOT / 'outputs'
    output_root.mkdir(exist_ok=True)
    output = Path(tempfile.mkdtemp(prefix='vd12_geometry_contract_', dir=output_root))
    # All generated XML/meshes/textures are new-cache outputs of the existing
    # builder. No scene model, collision mask, gains or default actor is edited.
    scene = build_scene(0., output / 'new_scene_cache', timestep=.005,
                        tool_markers=False, road_surface='tape', robot_version='vd10')
    model = mujoco.MjModel.from_xml_path(str(scene))
    asset_validation = validate_vd10_model(model)
    original_model = model_fingerprint(model)
    data = mujoco.MjData(model)
    adapters = {
        'runtime_all_float64': MujocoGunGeometryGuard(model, dtype=torch.float64),
        'runtime_all_float32': MujocoGunGeometryGuard(model),
        'isaac_runtime_24body_float64': MujocoGunGeometryGuard(model, body_names=actual_isaac_names, dtype=torch.float64),
        'isaac_runtime_24body_float32': MujocoGunGeometryGuard(model, body_names=actual_isaac_names),
    }
    cpu_source = ASSETS / 'vd11_geometry_audit.py'
    cpu = load_pure_module('vd12_independent_source_cpu_audit', cpu_source).GunRobotGeometryAudit(URDF)
    for adapter in adapters.values():
        if adapter.geometry.num_pairs != 44 or set(adapter.geometry.pair_names) != set(manifest['geometry_registry']['pair_names']):
            raise AssertionError('Runtime and Isaac body-folding scopes differ')
    results = []
    for case in load_cases(args.keyframes):
        result = verify_case(mujoco, model, data, case, adapters, cpu)
        results.append(result)
        primary = result['adapter_results']['runtime_all_float64']
        print('%s: pairs=44 min_lb=%.9f m keepout=%s max64_error=%.3g m' % (
            case['label'], primary['minimum_clearance_lower_bound_m'], primary['conservative_keepout_violated'],
            result['max_pair_error_m']['runtime_all_float64']), flush=True)
    if model_fingerprint(model) != original_model:
        raise AssertionError('Contract verification changed model masks, gains or dynamics')
    report = dict(schema_version=1, contract_verified=True, runtime_engine='MuJoCo ' + mujoco.__version__,
                  python=sys.executable, torch_version=torch.__version__, dtype_primary='float64 strict comparison; float32 training precision also compared',
                  physics_advanced=False, mj_step_calls=0, optimizer_called=False, actor_loaded=False,
                  default_deployment_changed=False, model_properties_unchanged=True,
                  source_urdf=str(URDF), source_urdf_sha256=sha256(URDF),
                  current_shared_profile=adapters['runtime_all_float64'].profile,
                  isaac_registry_manifest=str(args.isaac_manifest.resolve()), isaac_manifest_sha256=sha256(args.isaac_manifest),
                  isaac_runtime_body_names=actual_isaac_names, isaac_registry_recorded_profile=manifest['geometry_profile'],
                  provenance_note='Isaac manifest supplies its actual 24-body registry. Its recorded historical algorithm SHA may differ; all current numerical checks use the current shared geometry source and contract SHA, reported separately.',
                  keyframe_source=str(args.keyframes.resolve()), keyframe_sha256=sha256(args.keyframes),
                  new_cache_scene=str(scene), asset_validation=asset_validation, original_model_fingerprint=original_model,
                  adapter_metadata={name: adapter.metadata for name, adapter in adapters.items()}, results=results,
                  limitations=['Static engine-FK contract verification only: no dynamics, controller behavior, convergence or deployment acceptance.',
                      'Historical keyframes have actual six-arm-joint q only. Nominal legs and translation-normalized root XY are explicitly declared, not fabricated encoder data.',
                      'A negative conservative lower bound is a keep-out violation, not a contact-force measurement or general physical-intersection proof.',
                      'A single-tick adapter does not implement the four-physics-sample training termination latch.',
                      'Only authored URDF collision primitives and sampled configurations are covered; no continuous swept-motion or unmodelled visual-surface certificate.'])
    target = output / 'mujoco_isaac_geometry_contract.json'
    target.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf-8')
    print('report=' + str(target))
    return 0


if __name__ == '__main__':
    torch.set_num_threads(1)
    raise SystemExit(main())
