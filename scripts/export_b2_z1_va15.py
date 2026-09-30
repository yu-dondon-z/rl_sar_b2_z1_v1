#!/usr/bin/env python3
"""Export VA15 into an isolated MuJoCo profile; leave older profiles intact."""
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import yaml

ROOT = Path(__file__).resolve().parents[1]
RUN = Path('/home/dong/b2_z1/logs/rsl_rl/b2_z1_amp/2026-09-28_15-07-31_va15_curriculum_from2500')
OUT = ROOT / 'policy/b2_z1_no_gun/VA15'


def main():
    if OUT.exists():
        raise SystemExit(f'Refusing to overwrite existing profile: {OUT}')
    env = yaml.load((RUN/'params/env.yaml').read_text(), Loader=yaml.BaseLoader)
    agent = yaml.load((RUN/'params/agent.yaml').read_text(), Loader=yaml.BaseLoader)
    action = env['actions']['JointPositionAction']
    assert float(action['scale']) == .25 and float(action['alpha']) == .2
    assert float(env['sim']['dt']) == .005
    subprocess.run([sys.executable, str(ROOT/'scripts/export_b2_z1_v4.py'),
                    '--checkpoint', str(RUN/'model_5000.pt'), '--output', str(OUT),
                    '--version-label', 'VA15'], check=True)
    cfg = json.loads((OUT/'config.json').read_text())
    cfg.update(tcp_min=[-1., -.15, -.55], tcp_max=[-.55, .15, -.30],
               tcp_path_height_offset=.06, tcp_height_world_vertical=True,
               vision_bridge_args=['--path-height-offset', '0'],
               cruise_speed=.22, action_clip=float(agent['clip_actions']),
               training_env=str(RUN/'params/env.yaml'),
               notes='VA15 no-stick training model; extended virtual TCP; simulation only')
    (OUT/'config.json').write_text(json.dumps(cfg, indent=2)+'\n')
    model = ET.parse(OUT/'b2.xml')
    gun = model.find(".//geom[@name='welding_gun']")
    # Compile the collider enabled; the player hides/disables it while lying
    # and attaches it only when the operator presses 1 after standing.
    gun.set('contype', '2')
    gun.set('conaffinity', '1')
    gun.set('rgba', '0.65 0.68 0.72 1')
    model.write(OUT/'b2.xml', encoding='unicode')
    print('VA15 simulation profile:', OUT)


if __name__ == '__main__':
    main()
