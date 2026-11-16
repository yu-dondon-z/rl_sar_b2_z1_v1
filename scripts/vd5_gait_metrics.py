"""Offline, engine-independent VD5 gait measurements (evaluation only)."""
import numpy as np
from scipy.signal import butter, sosfiltfilt


def distribution(values):
    a = np.asarray(values, dtype=float)
    return None if not a.size else dict(median=float(np.median(a)), p95=float(np.percentile(a, 95)),
                                      max=float(np.max(a)))


def high_frequency_rms(values, dt, cutoff=3.):
    """Remove frequencies below cutoff; exclude filter edges from RMS."""
    a = np.asarray(values, dtype=float)
    if len(a) < int(4 / dt):
        return None
    residual = sosfiltfilt(butter(3, cutoff, fs=1/dt, btype='highpass', output='sos'), a, axis=0)
    residual = residual[int(1/dt):-int(1/dt)]
    return float(np.sqrt(np.mean(np.sum(residual**2, axis=1)))) if a.ndim == 2 else float(np.sqrt(np.mean(residual**2)))


def summarize(physics, actor, dt=.005, settle=5.):
    rows = [r for r in physics if r['time_s'] >= settle]
    commands = [r for r in actor if r['time_s'] >= settle]
    if not rows:
        return dict(post_startup_samples=0)
    feet_v = np.array([r['foot_velocity_w'] for r in rows])
    force = np.array([r['foot_force_w'] for r in rows])
    contacts = force[:, :, 2] > 10.
    landing = {}
    for j, name in enumerate(('FL', 'FR', 'RL', 'RR')):
        events = []
        off = int(round(.06 / dt))
        on = max(2, int(round(.01 / dt)))
        for k in range(off, len(rows)-max(on, int(.04/dt))):
            if (not contacts[k-off:k, j].any() and contacts[k:k+on, j].all()):
                events.append(dict(time_s=rows[k]['time_s'],
                    descending_speed_mps=max(0., float(-feet_v[k-1, j, 2])),
                    peak_force_next40ms_n=float(np.max(force[k:k+int(.04/dt)+1, j, 2]))))
        landing[name] = dict(count=len(events),
            descending_speed_mps=distribution([e['descending_speed_mps'] for e in events]),
            peak_force_next40ms_n=distribution([e['peak_force_next40ms_n'] for e in events]))
    targets = np.array([r['joint_target18'] for r in commands])
    arm = [8, 13, 14, 15, 16, 17]
    legs = [i for i in range(18) if i not in arm]
    def target_delta(ids):
        return distribution(np.abs(np.diff(targets[:, ids], axis=0))) if len(targets) > 1 else None
    return dict(post_startup_samples=len(rows), settle_s=settle,
        base_speed_mean_mps=float(np.mean([r['base_velocity_b'][0] for r in rows])),
        tilt_max_deg=max(r['tilt_deg'] for r in rows),
        base_z_highpass_3hz_rms_m=high_frequency_rms([r['base_position_w'][2] for r in rows], dt),
        tcp_world_z_highpass_3hz_rms_m=high_frequency_rms([r['tcp_position_w'][2] for r in rows], dt),
        tcp_body_highpass_3hz_vector_rms_m=high_frequency_rms([r['tcp_position_b'] for r in rows], dt),
        tcp_body_target_error_m=distribution([np.linalg.norm(np.array(r['tcp_position_b'])-
            np.array(r['tool_target_b'])) for r in rows]),
        arm_target_delta_per20ms_rad=target_delta(arm), leg_target_delta_per20ms_rad=target_delta(legs),
        touchdowns=landing,
        measurement_note='200 Hz link-origin velocity just before first >10 N contact; >=60 ms flight and >=10 ms contact; 40 ms sampled peak, not continuous peak. Fixed body target is diagnostic, not world seam tracking.')
