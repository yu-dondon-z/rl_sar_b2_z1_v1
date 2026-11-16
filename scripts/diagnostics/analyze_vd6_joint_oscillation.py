#!/usr/bin/env python3
"""Read existing gait sessions; locate joint bands and touchdown associations.

All results are evaluation only. 50 Hz data cannot establish sub-20 ms causal
ordering or rule out aliased vibration above 25 Hz. No simulation rollout.
"""
import argparse
import gzip
import json
from pathlib import Path
import tempfile

import numpy as np
from scipy.signal import butter, sosfiltfilt, welch

ARM = [8, 13, 14, 15, 16, 17]
ROOT = Path(__file__).resolve().parents[2]


def read_rows(path):
    with gzip.open(path, 'rt') as handle:
        return [json.loads(line) for line in handle]


def hp(values, fs):
    return sosfiltfilt(butter(3, 3., fs=fs, btype='highpass', output='sos'), values, axis=0)


def rms(values):
    return np.sqrt(np.mean(np.asarray(values)**2, axis=0))


def spectrum(values, fs):
    frequency, power = welch(values, fs=fs, nperseg=min(len(values), int(4*fs)), axis=0)
    band = (frequency >= 3) & (frequency <= .48*fs)
    return frequency[band][np.argmax(power[band], axis=0)].tolist()


def touchdowns(physics):
    t = np.array([r['time_s'] for r in physics])
    f = np.array([r['foot_force_w'] for r in physics])[:, :, 2]
    result = []
    for j, foot in enumerate(('FL', 'FR', 'RL', 'RR')):
        on = f[:, j] > 10
        for k in range(12, len(t)-8):
            if not on[k-12:k].any() and on[k:k+2].all():
                result.append(dict(foot=foot, time_s=float(t[k]),
                                   peak_force_n=float(f[k:k+9, j].max())))
    return sorted(result, key=lambda e: e['time_s'])


def analyze(path, jacobian):
    manifest = json.loads((path/'manifest.json').read_text())
    actor = [r for r in read_rows(path/'actor_trace.jsonl.gz') if 5 <= r['time_s'] <= 30]
    physics = [r for r in read_rows(path/'physics_truth.jsonl.gz') if 5 <= r['time_s'] <= 30]
    t = np.array([r['time_s'] for r in actor])
    tp = np.array([r['time_s'] for r in physics])
    from vd1_mujoco_scene import EXPECTED_JOINT_NAMES, DEFAULT_JOINT_POSITION
    default = np.array([DEFAULT_JOINT_POSITION[n] for n in EXPECTED_JOINT_NAMES])
    # Older VD5 traces contain encoders only inside the actor observation:
    # q-default at 16:34 and qd*0.05 at 34:52. Recover those recorded sensors.
    q = np.array([r['joint_position18'] if 'joint_position18' in r else
                  np.asarray(r['observation'][16:34])+default for r in actor])[:, ARM]
    qd = np.array([r['joint_velocity18'] if 'joint_velocity18' in r else
                   np.asarray(r['observation'][34:52])/.05 for r in actor])[:, ARM]
    target = np.array([r['joint_target18'] for r in actor])[:, ARM]
    y = np.interp(t, tp, [r['tcp_position_b'][1] for r in physics])
    gyro = np.array([r['observation'][10:13] for r in actor])/.2
    qhp, thp, yhp = hp(q, 50), hp(target, 50), hp(y, 50)
    gyrohp=hp(gyro,50)
    keep = slice(50, -50)
    # Local differential FK at the mean posture, with an explicit fit check.
    jy = jacobian(np.mean(q, axis=0))[1]
    contributions = qhp*jy
    prediction = contributions.sum(axis=1)
    error = yhp[keep]-prediction[keep]
    events = [e for e in touchdowns(physics) if 6.2 <= e['time_s'] <= 28.8]
    lags = np.arange(-5, 11)*.02
    event_report = {}
    for foot in ('FL', 'FR', 'RL', 'RR'):
        chosen = [e for e in events if e['foot'] == foot]
        if not chosen:
            continue
        times = np.array([e['time_s'] for e in chosen])
        # Raw qd magnitude avoids acausal filtering when comparing event timing.
        ev = np.array([[np.interp(times+lag, t, np.abs(qd[:, j])).mean()
                        for j in range(6)] for lag in lags])
        yvel = np.gradient(np.array([r['tcp_position_b'][1] for r in physics]), .005)
        ev_y = [float(np.interp(times+lag, tp, np.abs(yvel)).mean()) for lag in lags]
        event_report[foot] = dict(count=len(chosen), lags_s=lags.tolist(),
            mean_abs_joint_velocity_radps=ev.tolist(), mean_abs_tcp_y_velocity_mps=ev_y,
            joint_peak_lags_s=lags[np.argmax(ev, axis=0)].tolist(),
            tcp_y_velocity_peak_lag_s=float(lags[np.argmax(ev_y)]))
    joints = []
    for j in range(6):
        # Target at t is NEW; q(t) responds to previous held targets.
        joints.append(dict(joint='joint'+str(j+1), actor_index=ARM[j],
            q_hp_rms_rad=float(rms(qhp[keep, j])), target_hp_rms_rad=float(rms(thp[keep, j])),
            velocity_rms_radps=float(rms(qd[keep, j])),
            velocity_abs_max_radps=float(np.max(np.abs(qd[keep, j]))),
            q_peak_hz=spectrum(qhp[keep, j], 50), target_peak_hz=spectrum(thp[keep, j], 50),
            mean_jacobian_y_m_per_rad=float(jy[j]),
            linear_tcp_y_contribution_hp_rms_mm=float(rms(contributions[keep, j])*1000),
            correlation_q_tcp_y=float(np.corrcoef(qhp[keep, j], yhp[keep])[0, 1]),
            correlation_q_roll_rate=float(np.corrcoef(qhp[keep,j],gyrohp[keep,0])[0,1]),
            tcp_y_covariance_fraction=float(np.mean(contributions[keep,j]*yhp[keep])/
                                            np.mean(yhp[keep]**2)),
            held_target_tracking_rms_rad=float(rms(q[1:, j]-target[:-1, j]))))
    result = dict(path=str(path), manifest=manifest, window_s=[5,30],
        sample_rates_hz=dict(joints=50, physics=200), joints=joints,
        encoder_source=('explicit joint fields' if 'joint_position18' in actor[0] else
                        'recorded observation: q-default, qd*0.05; no resimulation'),
        tcp_y_peak_hz=spectrum(yhp[keep], 50),
        tcp_y_hp_rms_mm=float(rms(yhp[keep])*1000),
        fk_linearization_residual_rms_mm=float(rms(error)*1000),
        fk_linearization_explained_variance=float(1-np.var(error)/np.var(yhp[keep])),
        gyro_hp_rms_radps=rms(hp(gyro,50)[keep]).tolist(),
        correlation_roll_rate_tcp_y=float(np.corrcoef(gyrohp[keep,0],yhp[keep])[0,1]),
        touchdown_association=event_report,
        largest_impacts=sorted(events,key=lambda e:e['peak_force_n'],reverse=True)[:8])
    return result, (t, qhp, thp, yhp, gyro, events)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--comparison', type=Path, default=ROOT/'outputs/VD6_REVIEW_20261111/comparison.json')
    parser.add_argument('--output-root', type=Path, required=True)
    args = parser.parse_args()
    import mujoco
    from vd1_mujoco_scene import build_scene
    scene = build_scene(0., Path(tempfile.mkdtemp(prefix='vd6_fk_', dir='/dev/shm')),
                        timestep=.005, tool_markers=False, road_surface='tape', robot_version='vd3')
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model,data,model.key('vd1_default').id)
    joints = [model.joint('joint'+str(j)).id for j in range(1,7)]
    def jacobian(q):
        data.qpos[model.jnt_qposadr[joints]] = q
        mujoco.mj_forward(model,data)
        jac = np.zeros((3,model.nv))
        mujoco.mj_jacSite(model,data,jac,None,model.site('tool_tcp').id)
        return jac[:,model.jnt_dofadr[joints]]
    args.output_root.mkdir(parents=True,exist_ok=True)
    sessions, traces = [], []
    for row in json.loads(args.comparison.read_text())['fixed']:
        m = row['manifest']
        if m.get('checkpoint_iteration',4498) not in (4498,4997) or m['speed'] != .2:
            continue
        result, trace = analyze(ROOT/row['path'], jacobian)
        sessions.append(result)
        traces.append(trace)
        print(m['engine'],m.get('checkpoint_iteration',4498),-m['fixed_body_tool_target'][0],
              'Y peak',result['tcp_y_peak_hz'], 'contributions mm',
              np.round([j['linear_tcp_y_contribution_hp_rms_mm'] for j in result['joints']],2))
    output = dict(sessions=sessions, notes=[
        'Associations in a closed loop are not causal attribution; event averages include gait phase and other feet.',
        '50 Hz joint samples have Nyquist 25 Hz and may alias higher frequency physics.',
        'Highpass is zero-phase, excludes 1 s at edges; timing comparisons use raw velocity magnitudes.',
        'At actor t, q/qd precede the new target; held-target error uses target(t-20ms).',
        'Jacobian decomposition is local; contribution RMS values do not add because joints correlate.'])
    (args.output_root/'existing_joint_analysis.json').write_text(json.dumps(output,indent=2,allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(len(sessions),3,figsize=(17,3*len(sessions)),squeeze=False)
    for row,(r,trace) in enumerate(zip(sessions,traces)):
        t,q,tar,y,gyro,events = trace
        # Around largest impact with room on both sides.
        center = r['largest_impacts'][0]['time_s']
        take = (t>=center-.35)&(t<=center+.45)
        axes[row,0].plot(t[take],y[take]*1000,label='TCP Y mm')
        axes[row,0].plot(t[take],gyro[take,0]*10,label='roll rate x10 rad/s')
        for j in (0,4,5):
            axes[row,1].plot(t[take],q[take,j],label='q joint'+str(j+1))
            axes[row,2].plot(t[take],tar[take,j],label='target joint'+str(j+1))
        for e in events:
            if center-.35<=e['time_s']<=center+.45:
                for ax in axes[row]:
                    ax.axvline(e['time_s'],color='gray',alpha=.4)
                axes[row,0].text(e['time_s'],0,e['foot'],fontsize=8)
        m=r['manifest']
        axes[row,0].set_ylabel('%s %s %.2fm' % (m['engine'],m.get('checkpoint_iteration',4498),-m['fixed_body_tool_target'][0]))
        for ax in axes[row]:
            ax.legend(fontsize=7); ax.grid(alpha=.2); ax.set_xlabel('time s')
    fig.tight_layout()
    fig.savefig(args.output_root/'existing_joint_touchdown.png',dpi=160)


if __name__ == '__main__':
    main()
