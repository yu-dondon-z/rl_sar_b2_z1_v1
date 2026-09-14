#!/usr/bin/env python3
"""Export the Isaac v4 actor and an isolated MuJoCo playback asset bundle."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import torch
import yaml
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CHECKPOINT = Path('/home/dong/b2_z1/logs/rsl_rl/b2_z1_amp/2026-09-13_14-31-05_rear_curve_gait_heading_v5_from10000/model_10300.pt')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', type=Path, default=DEFAULT_CHECKPOINT)
    parser.add_argument('--output', type=Path, default=ROOT / 'policy/b2_z1_no_gun/v4')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    checkpoint = torch.load(str(args.checkpoint), map_location='cpu', weights_only=False)
    if checkpoint.get('obs_norm_state_dict'):
        raise ValueError('This export expects the v4 unnormalised 70-D observations')
    state = {k[len('actor.'):]: v for k, v in checkpoint['model_state_dict'].items() if k.startswith('actor.')}
    actor = torch.nn.Sequential(
        torch.nn.Linear(70, 512), torch.nn.ELU(),
        torch.nn.Linear(512, 256), torch.nn.ELU(),
        torch.nn.Linear(256, 128), torch.nn.ELU(), torch.nn.Linear(128, 18),
    ).eval()
    actor.load_state_dict(state, strict=True)
    scripted = torch.jit.script(actor)
    scripted.save(str(args.output / 'policy.pt'))
    torch.manual_seed(42)
    with torch.inference_mode():
        samples = torch.randn(256, 70)
        error = (torch.jit.load(str(args.output / 'policy.pt'))(samples) - actor(samples)).abs().max().item()
    assert error < 1e-6, error
    base = yaml.safe_load((ROOT / 'policy/b2_z1_no_gun/base.yaml').read_text())['b2_z1_no_gun']
    config = {
        'version': 'v4', 'checkpoint': str(args.checkpoint),
        'checkpoint_sha256': hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
        'joint_names': base['joint_names'], 'default_dof_pos': base['default_dof_pos'],
        'deployment_stand_dof_pos': base['deployment_stand_dof_pos'],
        'deployment_lying_arm_dof_pos': base['deployment_lying_arm_dof_pos'],
        'welding_gun_requires_upright_install': base['welding_gun_requires_upright_install'],
        'policy_standby_arm_pos': base['policy_standby_arm_pos'],
        'deployment_stand_kp': [450]*8 + [60] + [675]*4 + [60]*5,
        'deployment_stand_kd': [15]*8 + [1] + [15]*4 + [1]*5,
        'deployment_stand_max_joint_error': base['deployment_stand_max_joint_error'],
        'physics_dt': 0.001, 'control_dt': 0.005, 'policy_dt': 0.02,
        'action_scale': 0.25, 'action_filter_alpha': 0.2, 'action_clip': 5.0,
        'kp': 160.0, 'kd': 5.0, 'cruise_speed': 0.35,
        'torque_limits': [200]*8 + [120] + [320]*4 + [120]*5,
        'target_orientation': [0.0, -2**-0.5, 0.0, 2**-0.5],
    }
    (args.output / 'config.json').write_text(json.dumps(config, indent=2) + '\n')

    source = ROOT / 'src/rl_sar_zoo/b2_z1_no_gun_description/mjcf'
    model = ET.parse(source / 'b2.xml')
    model.find('compiler').set('meshdir', '.')
    for mesh in model.findall('./asset/mesh'):
        mesh.set('file', os.path.relpath(source/'assets'/mesh.get('file'), args.output))
    model.find('option').set('integrator', 'implicitfast')
    model.find(".//body[@name='base_link']").set('pos', '0 0 0.7')
    for joint in model.findall('.//joint'):
        if joint.get('name', '').startswith('joint'):
            # The implicit position actuator already supplies kd=5.
            joint.set('damping', '0')
    flange = model.find(".//body[@name='link06']")
    mount = ET.SubElement(flange, 'body', name='link_mounter', pos='0.075 0 0')
    ET.SubElement(mount, 'inertial', pos='0 0 0', mass='0.001', diaginertia='0.000001 0.000001 0.000001')
    gun = ET.SubElement(mount, 'body', name='link_gun', pos='0 0 -0.2', quat='0.7071067811865476 0 0 0.7071067811865476')
    ET.SubElement(gun, 'inertial', pos='0 0 0', mass='0.001', diaginertia='0.000001 0.000001 0.000001')
    ET.SubElement(gun, 'geom', name='welding_gun', type='cylinder', pos='0 -0.03 -0.07',
                  size='0.028 0.55', rgba='0.65 0.68 0.72 1', contype='2', conaffinity='1',
                  group='0', mass='0', margin='0.002')
    # PhysX assigns the zero-mass dummy_target URDF link a 1 kg fallback.
    # This mass was present during training, even in the no-stick model.
    tcp = ET.SubElement(gun, 'body', name='dummy_target', pos='0 -0.03 -0.62')
    ET.SubElement(tcp, 'inertial', pos='0 0 0', mass='1', diaginertia='0.004 0.004 0.004')
    ET.indent(model, space='  ') if hasattr(ET, 'indent') else None
    model.write(args.output / 'b2.xml', encoding='unicode')
    # Match the v4 Isaac scene's gentle cubic-sine crack. Generate a separate
    # map with explicit bounds: the legacy PNG and XML bounds no longer agree.
    # Keep the interactive deployment scene compact.  Sixteen metres is long
    # enough for a normal visual-following trial, while reducing the hfield
    # from about 5 million to 1.2 million samples.
    x = np.linspace(-2., 14., 801)
    y = np.where(abs(x) <= 20., .4*np.sin(2*np.pi*x/20.)**3, 0.)
    road = Image.new('L', (801, 1501), color=107)
    ImageDraw.Draw(road).line(
        [(i, round((1.5-v)/3.*1500)) for i, v in enumerate(y)],
        fill=0, width=10)
    road.save(args.output/'cracked_ground.png')
    scene = ET.parse(source / 'scene_vision.xml')
    for elem in scene.iter():
        if elem.tag in ('texture', 'hfield') and elem.get('file'):
            elem.set('file', 'cracked_ground.png')
        if elem.tag == 'hfield':
            elem.set('size', '8 1.5 0.05 0.10')
        if elem.tag == 'geom' and elem.get('name') == 'floor':
            elem.set('pos', '6 0 -0.05')
        if elem.tag == 'light':
            elem.set('castshadow', 'false')
            elem.set('diffuse', '0.35 0.35 0.35')
            elem.set('ambient', '0.65 0.65 0.65')
    scene.write(args.output / 'scene_vision.xml', encoding='unicode')
    flat = ET.Element('mujoco', model='b2_z1_v4_flat')
    ET.SubElement(flat, 'include', file='b2.xml')
    world = ET.SubElement(flat, 'worldbody')
    ET.SubElement(world, 'light', pos='0 0 3', castshadow='false', ambient='0.6 0.6 0.6')
    ET.SubElement(world, 'geom', name='floor', type='plane', size='100 100 0.1',
                  rgba='0.42 0.42 0.42 1', friction='1.0 0.005 0.0001')
    ET.ElementTree(flat).write(args.output / 'scene_flat.xml', encoding='unicode')
    print(f'Exported {args.output}; 256-sample actor max error={error:g}', flush=True)


if __name__ == '__main__':
    main()
