"""Bound CPU use in the camera process only; leave the restored bridge intact."""
import os
from pathlib import Path
import runpy
import sys

if __name__ == '__main__':
    os.environ['OMP_NUM_THREADS'] = '2'
    os.environ['MKL_NUM_THREADS'] = '2'
    import torch
    torch.set_num_threads(2)
    torch.set_num_interop_threads(1)
    bridge = Path(__file__).with_name('run_b2_builtin_front_rhacracknet_bridge.py')
    sys.argv[0] = str(bridge)
    runpy.run_path(str(bridge), run_name='__main__')
