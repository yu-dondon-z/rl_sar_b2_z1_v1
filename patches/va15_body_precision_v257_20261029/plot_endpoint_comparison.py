"""Plot saved independent physics scores; no simulator/control is launched."""
import csv
import math
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def rows(file):
    with file.open() as source:
        return list(csv.DictReader(source))


def main():
    root=Path(__file__).resolve().parent
    before=root.parent/'va15_vision_completion_v253_20261027/outputs/rear_endpoint_final'
    after=root/'outputs/rear_endpoint_final_v2'
    output=after/'comparison.png'
    if output.exists():
        raise RuntimeError('Refusing to overwrite plot')
    figure,axes=plt.subplots(2,2,figsize=(11,7))
    for col,(scenario,mu) in enumerate([('straight',.7),('curve',.8)]):
        old=rows(before/f'{scenario}_mu{mu}.csv')
        new=rows(after/f'{scenario}_mu{mu}_end5.0_noise0.0.csv')
        goal_y=.035*math.sin(4.) if scenario=='curve' else 0.
        for data,label,color in [(old,'V253 baseline','#b35454'),(new,'V254 candidate','#177e50')]:
            trace=[r for r in data if float(r['time'])>=12.]
            axes[0,col].plot([float(r['tcp_truth_x']) for r in trace],
                             [float(r['tcp_truth_y']) for r in trace],color=color,label=label)
            axes[0,col].scatter([float(trace[-1]['tcp_truth_x'])],[float(trace[-1]['tcp_truth_y'])],color=color,s=30)
            axes[1,col].plot([float(r['time']) for r in trace],
                [100.*math.hypot(float(r['tcp_truth_x'])-5.,float(r['tcp_truth_y'])-goal_y) for r in trace],
                color=color,label=label)
        x=[3.8+i*.01 for i in range(121)] # the actual crack ENDS at x=5 m
        axes[0,col].plot(x,[.035*math.sin(.8*v) if scenario=='curve' else 0. for v in x],
                         color='black',ls='--',label='Observed crack geometry')
        axes[0,col].scatter([5.],[goal_y],color='black',marker='*',s=100,label='Physical endpoint')
        axes[0,col].set(xlim=(3.8,5.5),ylim=(-.35,.12),xlabel='Physical TCP x [m]',ylabel='Physical TCP y [m]',title=scenario.capitalize())
        axes[1,col].set(xlim=(35,60),ylim=(0,65),xlabel='Simulation time [s]',ylabel='TCP endpoint XY error [cm]')
        for row in range(2):
            axes[row,col].grid(alpha=.25)
            axes[row,col].legend(fontsize=8)
    figure.suptitle('Rear virtual welding TCP: independent physical truth scoring\nSynthetic camera + MuJoCo; no hardware qualification',fontsize=12)
    figure.tight_layout(rect=(0,0,1,.93))
    figure.savefig(output,dpi=150)
    plt.close(figure)
    print(output)


if __name__=='__main__':
    main()
