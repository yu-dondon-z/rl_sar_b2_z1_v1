#!/usr/bin/env python3
"""Read-only analysis of diagnostic CSVs. Never imports robot/network SDKs."""
import argparse
import csv
import json
import math
from pathlib import Path

def finite(values):
    return [v for v in values if math.isfinite(v)]

def stats(values):
    values=sorted(finite(values))
    if not values:return dict(count=0,min=None,median=None,p95=None,max=None)
    return dict(count=len(values),min=values[0],median=values[(len(values)-1)//2],
                p95=values[int((len(values)-1)*.95)],max=values[-1])

def load(path):
    with path.open(newline='') as f:
        rows=list(csv.DictReader(f))
    # Keep monotonic timestamps as integers: converting their nanoseconds to
    # float before subtraction discards resolution unnecessarily.
    return [{key:(int(value) if key.endswith('_ns') else float(value))
             for key,value in row.items()} for row in rows]

def analyze(folder):
    folder=Path(folder)
    commands=load(folder/'commands.csv');controls=load(folder/'control.csv');actors=load(folder/'actor.csv')
    moving=[r for r in commands if r['fsm']==2 and r['trial_stage']==3]
    sourced=[r for r in commands if r['low_callback_received_ns']>0]
    prepared=[r for r in commands if r['fsm']==2 and r['trial_stage'] in (1,2)]
    actor_motion=[r for r in actors if abs(r['policy_command_0'])>.001]
    active_control=[r for r in controls if r['fsm']==2 and r['trial_stage']==3]
    joints=[]
    for i in range(12):
        actual=finite([r[f'actual_q_dds_{i}'] for r in moving])
        target=finite([r[f'sent_q_dds_{i}'] for r in moving])
        errors=[r[f'sent_q_dds_{i}']-r[f'actual_q_dds_{i}'] for r in moving]
        pd=[];residual=[]
        for r in moving:
            tau=r[f'sent_kp_dds_{i}']*(r[f'sent_q_dds_{i}']-r[f'actual_q_dds_{i}'])
            tau+=r[f'sent_kd_dds_{i}']*(r[f'sent_dq_dds_{i}']-r[f'actual_dq_dds_{i}'])+r[f'sent_ff_tau_dds_{i}']
            pd.append(tau);residual.append(r[f'actual_tau_est_dds_{i}']-tau)
        valid_errors=finite(errors)
        joints.append(dict(dds_index=i,mean_signed_q_error_rad=(sum(valid_errors)/len(valid_errors) if valid_errors else None),
            abs_q_error_rad=stats([abs(v) for v in errors]),
            actual_q_range_rad=(max(actual)-min(actual) if actual else None),
            sent_q_range_rad=(max(target)-min(target) if target else None),
            actual_abs_dq_rad_s=stats([abs(r[f'actual_dq_dds_{i}']) for r in moving]),
            standing_kp=stats([r[f'sent_kp_dds_{i}'] for r in prepared]),
            moving_kp=stats([r[f'sent_kp_dds_{i}'] for r in moving]),
            moving_kd=stats([r[f'sent_kd_dds_{i}'] for r in moving]),
            requested_pd_tau_nm=stats(pd),tau_est_minus_requested_pd_nm=stats(residual)))
    summary_path=folder/'telemetry_summary.json'
    integrity=json.loads(summary_path.read_text()) if summary_path.is_file() else {'missing_shutdown_summary':True}
    return dict(schema='va15_diagnostic_analysis_v1',directory=str(folder.resolve()),
        hardware_qualified=False,dds_write_is_motor_ack=False,recording_integrity=integrity,
        command_rows=len(commands),control_rows=len(controls),actor_rows=len(actors),moving_rows=len(moving),
        requested_x_m_s=stats([r['requested_0'] for r in moving]),
        limited_x_m_s=stats([r['limited_0'] for r in moving]),
        actor_command_x_m_s=stats([r['policy_command_0'] for r in actor_motion]),
        source_age_at_dds_return_ms=stats([(r['publish_end_ns']-r['low_callback_received_ns'])/1e6
             for r in commands if r['low_callback_received_ns']>0]),
        command_generation_age_at_dds_return_ms=stats([(r['publish_end_ns']-r['command_generated_ns'])/1e6
             for r in commands if r['command_generated_ns']>0]),
        max_publish_interval_ms=stats([r['max_publish_interval_ns']/1e6 for r in commands]),
        max_control_interval_ms=stats([r['max_control_interval_ns']/1e6 for r in controls]),
        control_record_point_execution_ms=stats([(r['control_end_ns']-r['control_start_ns'])/1e6 for r in controls]),
        actor_interval_ms=stats([(r['actor_begin_ns']-r['previous_actor_begin_ns'])/1e6
             for r in actors if r['previous_actor_begin_ns']>0]),
        actor_execution_ms=stats([(r['actor_end_ns']-r['actor_begin_ns'])/1e6 for r in actors]),
        cached_action_frames=sum(r['model_executed']==0 for r in actors),
        dds_write_failures=sum(r['dds_write_return']!=1 for r in commands),
        unsourced_command_rows=len(commands)-len(sourced),
        mode_or_lost_bad_rows=sum(any(r[f'mode_dds_{i}']!=10 or r[f'lost_dds_{i}']!=0 for i in range(12)) for r in sourced),
        front_handoff_fraction=(sum(r['front_handoff_cycles']>0 for r in active_control)/len(active_control) if active_control else None),
        rear_handoff_fraction=(sum(r['rear_handoff_cycles']>0 for r in active_control)/len(active_control) if active_control else None),
        arm_handoff_fraction=(sum(r['arm_blend_cycles']>0 for r in active_control)/len(active_control) if active_control else None),
        joints=joints,
        limitations=[
            'No independent real ground-truth displacement or foot clearance; this does not assess walking success.',
            'Feedback is the exact lowstate snapshot used by the producing control frame, not a motor response to that same command.',
            'Latest actor completion is a counter, not an exact association of an output queue item with a DDS command.',
            'PD torque is a requested estimate, not a proof of motor torque limits, saturation or acceptance.',
            'Static load-bearing PD can require nonzero q error; increasing gains is not inferred automatically.',
        ])

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);p.add_argument('--output',type=Path)
    args=p.parse_args();result=analyze(args.directory)
    text=json.dumps(result,indent=2,allow_nan=False)
    if args.output:
        with args.output.open('x') as f:f.write(text+'\n')
    print(text)

if __name__=='__main__':main()
