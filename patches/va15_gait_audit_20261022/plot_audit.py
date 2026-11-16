"""Plot recorded startup PD transition; never connects to hardware."""
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parent
record = ROOT.with_name('va15_motor_contract_v2_20261021')/'board_results/motor_contract_v2_forward_20261022_W7SnQf'
with (record/'commands.csv').open() as stream:
    rows = list(csv.DictReader(stream))
origin = min(int(r['control_start_ns']) for r in rows if r['fsm'] == '2' and r['trial_stage'] == '3')
rows = [r for r in rows if -.12 <= (int(r['control_start_ns'])-origin)/1e9 <= .3]
t = np.array([(int(r['control_start_ns'])-origin)/1e9 for r in rows])
fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, constrained_layout=True)
axes[0].plot(t, [float(r['sent_kp_dds_2']) for r in rows], marker='.', label='FR calf Kp')
axes[0].set_ylabel('Kp (N m/rad)')
for leg, index in [('FR', 2), ('FL', 5), ('RR', 8), ('RL', 11)]:
    q = np.array([float(r[f'actual_q_dds_{index}']) for r in rows])
    target = np.array([float(r[f'sent_q_dds_{index}']) for r in rows])
    dq = np.array([float(r[f'actual_dq_dds_{index}']) for r in rows])
    kp = np.array([float(r[f'sent_kp_dds_{index}']) for r in rows])
    kd = np.array([float(r[f'sent_kd_dds_{index}']) for r in rows])
    pd = kp*(target-q)-kd*dq
    axes[1].plot(t, pd, marker='.', label=leg+' calf requested PD')
    axes[2].plot(t, q, marker='.', label=leg+' calf feedback')
axes[1].set_ylabel('Requested PD (N m)')
axes[2].set_ylabel('Feedback angle (rad)')
axes[2].set_xlabel('Seconds from first recorded forward command')
axes[0].set_title('Recorded startup: target continuity does not preserve PD support across a gain jump')
for ax in axes:
    ax.axvline(0, color='black', linestyle=':', label='motion entry')
    ax.grid(alpha=.25)
    ax.legend(fontsize=8, loc='best')
fig.savefig(ROOT/'startup_pd_transition.png', dpi=150)
plt.close(fig)
print(ROOT/'startup_pd_transition.png')
