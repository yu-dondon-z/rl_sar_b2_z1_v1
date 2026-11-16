#!/usr/bin/env python3
"""Export a standalone figure from the two 200 Hz gait logs."""
import argparse
import gzip
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter, sosfiltfilt


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mujoco', type=Path)
    p.add_argument('isaac', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--first-label', default='MuJoCo')
    p.add_argument('--second-label', default='Isaac Lab / PhysX')
    p.add_argument('--title', default='VD5 model_4498: fixed body TCP target, 0.2 m/s command; evaluation only')
    args = p.parse_args()
    fig, axes = plt.subplots(3,2,figsize=(12,8),constrained_layout=True)
    for col, (name, path) in enumerate(((args.first_label,args.mujoco),(args.second_label,args.isaac))):
        with gzip.open(str(path/'physics_truth.jsonl.gz'),'rt') as f:
            rows = [json.loads(s) for s in f]
        t = np.array([r['time_s'] for r in rows]); window = (t>=10)&(t<=12)
        z = np.array([r['tcp_position_w'][2] for r in rows])
        residual = sosfiltfilt(butter(3,3.,fs=200,btype='highpass',output='sos'),z)
        axes[0,col].plot(t[window],residual[window]*1000)
        axes[0,col].set(title=name, ylabel='TCP Z >3 Hz (mm)')
        for j,label in enumerate(('FL','FR','RL','RR')):
            axes[1,col].plot(t[window],np.array([r['foot_velocity_w'][j][2] for r in rows])[window],label=label,lw=.8)
            axes[2,col].plot(t[window],np.array([r['foot_force_w'][j][2] for r in rows])[window],label=label,lw=.8)
        axes[1,col].set_ylabel('Foot vertical velocity (m/s)')
        axes[2,col].set(ylabel='Foot vertical force (N)',xlabel='Simulation time (s)')
        axes[1,col].legend(ncol=4,fontsize=8)
        for ax in axes[:,col]:
            ax.grid(alpha=.25)
    fig.suptitle(args.title)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(str(args.output),dpi=160)
    print(args.output)


if __name__ == '__main__':
    main()
