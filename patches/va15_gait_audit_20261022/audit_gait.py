"""Offline phase/action/PD audit of the recorded VA15 trial; no robot SDKs."""
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import torch
import yaml

ROOT = Path(__file__).resolve().parent
RECORD = ROOT.with_name('va15_motor_contract_v2_20261021')/'board_results/motor_contract_v2_forward_20261022_W7SnQf'
TRAIN = Path('/home/dong/b2_z1/logs/rsl_rl/b2_z1_amp/2026-09-28_15-07-31_va15_curriculum_from2500')
LEGS = [0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12]
CALVES = {'FR': 2, 'FL': 5, 'RR': 8, 'RL': 11}


def read(name):
    with (RECORD/name).open(newline='') as stream:
        return list(csv.DictReader(stream))


def matrix(rows, key, count):
    return np.array([[float(r[key+str(i)]) for i in range(count)] for r in rows])


def pd(row, i, q_target=None, kp=None, kd=None):
    target = float(row[f'sent_q_dds_{i}']) if q_target is None else q_target
    p = float(row[f'sent_kp_dds_{i}']) if kp is None else kp
    d = float(row[f'sent_kd_dds_{i}']) if kd is None else kd
    return (p*(target-float(row[f'actual_q_dds_{i}'])) +
            d*(float(row[f'sent_dq_dds_{i}'])-float(row[f'actual_dq_dds_{i}'])) +
            float(row[f'sent_ff_tau_dds_{i}']))


def time_of(row):
    return int(row['control_start_ns'])


def phase_name(row):
    if row['fsm'] != '2':
        return None
    if row['trial_stage'] == '2':
        return 'armed_static'
    if row['trial_stage'] == '3':
        if int(row['front_handoff_cycles']) or int(row['rear_handoff_cycles']):
            return 'leg_handoff'
        if int(row['arm_blend_cycles']):
            return 'legs_released_arm_handoff'
        return 'all_handoffs_released_not_steady_state_qualification'
    if row['trial_stage'] == '4' and row['policy_running'] == '1':
        return 'actor_deceleration'
    return None


def phase_summary(rows, origin):
    joints = {}
    for name, i in CALVES.items():
        q = np.array([float(r[f'actual_q_dds_{i}']) for r in rows])
        target = np.array([float(r[f'sent_q_dds_{i}']) for r in rows])
        e = target-q
        joints[name] = dict(mean_error_rad=float(e.mean()), demeaned_error_rms_rad=float(e.std()),
            abs_error_p95_rad=float(np.quantile(abs(e), .95)),
            target_range_rad=float(np.ptp(target)), actual_range_rad=float(np.ptp(q)),
            mean_tau_est_nm=float(np.mean([float(r[f'actual_tau_est_dds_{i}']) for r in rows])))
    return dict(rows=len(rows), start_s=(time_of(rows[0])-origin)/1e9,
                end_s=(time_of(rows[-1])-origin)/1e9, calves=joints)


def main():
    cfg = next(iter(yaml.safe_load((ROOT/'inputs/deployment_config.yaml').read_text()).values()))
    base = next(iter(yaml.safe_load((ROOT/'inputs/deployment_base.yaml').read_text()).values()))
    # BaseLoader treats archived Python YAML tags as text/containers; it does
    # not instantiate objects or import the training environment.
    train = yaml.load((TRAIN/'params/env.yaml').read_text(), Loader=yaml.BaseLoader)
    agent = yaml.safe_load((TRAIN/'params/agent.yaml').read_text())
    manifest = json.loads((ROOT/'inputs/deployment_manifest.json').read_text())
    paths = [TRAIN/'model_5000.pt', ROOT/'inputs/board_policy.pt', ROOT/'inputs/deployment_config.yaml']
    expected = [manifest['checkpoint_sha256'], manifest['model_sha256'], manifest['config_sha256']]
    hashes = {str(path): hashlib.sha256(path.read_bytes()).hexdigest() for path in paths}
    assert list(hashes.values()) == expected

    commands, controls, actors = read('commands.csv'), read('control.csv'), read('actor.csv')
    moving = [r for r in commands if r['fsm'] == '2' and r['trial_stage'] == '3']
    origin = time_of(moving[0])
    phases = {}
    for row in commands:
        name = phase_name(row)
        if name:
            phases.setdefault(name, []).append(row)
    phases = {name: phase_summary(rows, origin) for name, rows in phases.items()}

    raw, applied = matrix(actors, 'raw_action_', 18), matrix(actors, 'filtered_action_', 18)
    obs, target = matrix(actors, 'observation_', 70), matrix(actors, 'target_q_policy_', 18)
    lower, upper = np.array(cfg['clip_actions_lower']), np.array(cfg['clip_actions_upper'])
    ema = np.zeros(18)
    predicted = []
    clipping = []
    for action in raw:
        ema = cfg['action_filter_alpha']*np.clip(action, -cfg['policy_input_clip'], cfg['policy_input_clip'])+(1-cfg['action_filter_alpha'])*ema
        output = np.clip(ema, lower, upper)
        predicted.append(output)
        clipping.append(abs(output-ema) > 1e-6)

    torch.set_num_threads(2)
    model = torch.nn.Sequential(torch.nn.Linear(70, 512), torch.nn.ELU(),
        torch.nn.Linear(512, 256), torch.nn.ELU(), torch.nn.Linear(256, 128), torch.nn.ELU(), torch.nn.Linear(128, 18))
    state = torch.load(TRAIN/'model_5000.pt', map_location='cpu', weights_only=False)['model_state_dict']
    model.load_state_dict({key[6:]: value for key, value in state.items() if key.startswith('actor.')})
    exported = torch.jit.load(str(ROOT/'inputs/board_policy.pt'), map_location='cpu')
    model.eval(); exported.eval()
    with torch.inference_mode():
        inputs = torch.tensor(obs, dtype=torch.float32)
        trained, jit = model(inputs).numpy(), exported(inputs).numpy()
    chain = dict(actor_rows=len(actors),
        original_checkpoint_vs_native_raw_max_abs=float(np.max(abs(trained-raw))),
        board_export_vs_native_raw_max_abs=float(np.max(abs(jit-raw))),
        original_checkpoint_vs_board_export_max_abs=float(np.max(abs(trained-jit))),
        previous_bounded_raw_observation_max_abs=float(np.max(abs(obs[1:, 52:]-np.clip(raw[:-1], -5, 5)))),
        ema_replay_max_abs=float(np.max(abs(np.array(predicted)-applied))),
        scaled_target_formula_max_abs_rad=float(np.max(abs(np.array(cfg['default_dof_pos'])+np.array(cfg['action_scale'])*applied-target))),
        raw_clip_hits_per_joint=np.sum(abs(raw) >= cfg['policy_input_clip'], axis=0).tolist(),
        additional_filtered_clip_hits_per_joint=np.sum(clipping, axis=0).tolist())

    before = [r for r in commands if phase_name(r) == 'armed_static'][-1]
    first = moving[0]
    handoff = {}
    for name, i in CALVES.items():
        old_q, old_kp, old_kd = [float(before[f'{key}_dds_{i}']) for key in ('sent_q', 'sent_kp', 'sent_kd')]
        new_kp, new_kd = [float(first[f'{key}_dds_{i}']) for key in ('sent_kp', 'sent_kd')]
        q, dq = float(first[f'actual_q_dds_{i}']), float(first[f'actual_dq_dds_{i}'])
        # Counterfactual math only, evaluated at the SAME measured state.
        old_request_same_state = old_kp*(old_q-q)-old_kd*dq
        equivalent_target = q+(old_request_same_state+new_kd*dq)/new_kp
        handoff[name] = dict(old_kp=old_kp, new_kp=new_kp,
            last_static_record_pd_request_nm=pd(before, i), first_motion_record_pd_request_nm=pd(first, i),
            last_static_tau_est_nm=float(before[f'actual_tau_est_dds_{i}']),
            first_motion_tau_est_nm=float(first[f'actual_tau_est_dds_{i}']),
            old_pd_request_at_first_motion_state_nm=old_request_same_state,
            requested_pd_ratio_at_same_state=pd(first, i)/old_request_same_state,
            first_target_rad=float(first[f'sent_q_dds_{i}']),
            counterfactual_torque_equivalent_target_rad=equivalent_target,
            counterfactual_pd_continuity_error_nm=abs(new_kp*(equivalent_target-q)-new_kd*dq-old_request_same_state))

    actions = train['actions']['JointPositionAction']
    actuators = train['scene']['robot']['actuators']
    training_comparison = dict(
        training_dt=float(train['sim']['dt']), deployment_dt=base['dt'],
        training_decimation=int(train['decimation']), deployment_decimation=base['decimation'],
        training_action_scale=float(actions['scale']), deployment_action_scale=cfg['action_scale'],
        training_action_ema=float(actions['alpha']), deployment_action_ema=cfg['action_filter_alpha'],
        training_action_clip=agent['clip_actions'], deployment_action_clip=cfg['policy_input_clip'],
        training_leg_pd={name: dict(kp=float(value['stiffness']), kd=float(value['damping']))
                         for name, value in actuators.items() if name in ('M107-24-2', '2')},
        deployment_leg_kp=[cfg['rl_kp'][i] for i in LEGS], deployment_leg_kd=[cfg['rl_kd'][i] for i in LEGS],
        training_arm_kd=5., deployment_arm_kd=[cfg['rl_kd'][i] for i in (8, 13, 14, 15, 16, 17)],
        warning='Arm damping and extra arm filtering intentionally differ; no evidence here to remove them.')
    result = dict(schema='va15_phase_audit_v1', hardware_qualified=False, real_control_modified=False,
                  source_hashes=hashes, phase_summaries=phases, action_chain=chain,
                  startup_pd_transition=handoff, training_comparison=training_comparison,
                  limitations=['One approximately 4 s trial; only 1.7 s with all handoffs released.',
                    'Position ranges and target error are not walking success/foot clearance measures.',
                    'PD math and tau_est are not independent motor acknowledgement or ground-truth torque.',
                    'Torque-equivalent targets above are a numerical counterfactual, NOT executable commands.',
                    'The startup request drop does not by itself explain later gait tracking differences.'])
    output = ROOT/'phase_audit.json'
    with output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(output)
    print(json.dumps(dict(action_chain=chain, startup_pd_transition=handoff), indent=2))


if __name__ == '__main__':
    main()
