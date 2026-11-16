#!/usr/bin/env python3
"""MuJoCo dual-camera debug on a 0.30/m black tape curve with a massless gun.

Uses front RGB / rear RGBD RHANet seam measurements without TCP markers or
visual TCP feedback, with the same actor and experimental physics as before.
Only the generated gun body's mass/inertia are zero; the source URDF is intact.
The line is a ground texture with zero thickness; ground contact is flat.
Visible ground is 190 x 100 m; the tape texture retains its original resolution.
Additional arguments override the preset and are passed to play_vd1_mujoco.
"""
from pathlib import Path
import os
import sys

from play_vd1_mujoco import main as launch


ROOT = Path(__file__).resolve().parents[1]
PYTHON = '/home/dong/anaconda3/envs/crack_vision/bin/python'


def main(argv=None):
    os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
    os.environ.setdefault('OMP_NUM_THREADS', '1')
    preset = [
        '--mujoco-python', PYTHON, '--vision-python', PYTHON,
        '--speed', '0.4', '--curvature-stage', '5', '--max-steps', '3000',
        '--gun-mass-mode', 'zero',
        '--visual-controller', 'guarded',
        '--road-surface', 'tape',
        '--road-contact', 'heightfield',
        '--velocity-constraint', 'impulse', '--physics-dt', '0.005',
        '--physx-implicit-position',
        '--cache-dir', '/tmp/vd1_dual_vision_debug_cache',
        '--output-root', str(ROOT / 'outputs/VD1_DUAL_VISION_DEBUG'),
    ]
    return launch(preset + list(sys.argv[1:] if argv is None else argv))


if __name__ == '__main__':
    raise SystemExit(main())
