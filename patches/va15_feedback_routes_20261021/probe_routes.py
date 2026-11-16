#!/usr/bin/env python3
"""Bounded read-only DDS feedback-route survey. Never grants motion authority."""
import argparse
import json
import math
from pathlib import Path
import sys
import threading
import time


def finite_tree(value):
    if isinstance(value, dict):
        return all(finite_tree(x) for x in value.values())
    if isinstance(value, (tuple, list)):
        return all(finite_tree(x) for x in value)
    return not isinstance(value, float) or math.isfinite(value)


def clean(value):
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [clean(x) for x in value]
    return None if isinstance(value, float) and not math.isfinite(value) else value


def summarize(rows, now):
    positive = [r['source'] for r in rows if r['source'] > 0]
    changes = regressions = repeats = 0
    last_advance = None
    for i, row in enumerate(rows):
        if row['source'] > 0 and (i == 0 or row['source'] > rows[i-1]['source']):
            changes += 1
            last_advance = row['received']
        if i:
            regressions += row['source'] < rows[i-1]['source']
            repeats += row['source'] == rows[i-1]['source']
    age = now-last_advance if last_advance is not None else None
    good_values = all(r['values_finite'] for r in rows)
    unique = len(set(positive))
    return dict(messages=len(rows), distinct_positive_source_times=unique,
        source_advances=changes, source_repeats=repeats, source_regressions=regressions,
        last_arrival_age_s=now-rows[-1]['received'] if rows else None,
        last_source_advance_age_s=age, all_values_finite=good_values,
        source_time_advancing_at_end=bool(unique >= 2 and not regressions and good_values and age is not None and 0 <= age <= .15),
        first=clean(rows[0]['data']) if rows else None, last=clean(rows[-1]['data']) if rows else None,
        physical_frame_height_velocity_validated=False, control_authorized=False)


def stamp(s):
    return int(s.sec)*1000000000+int(s.nanosec)


def xyz(v):
    return [float(v.x), float(v.y), float(v.z)]


def low(msg):
    return int(msg.tick)*1000000, dict(tick=int(msg.tick), raw_foot_force_dds=list(msg.foot_force),
        raw_foot_force_est_dds=list(msg.foot_force_est),
        q=[float(m.q) for m in msg.motor_state[:12]], dq=[float(m.dq) for m in msg.motor_state[:12]],
        tau=[float(m.tau_est) for m in msg.motor_state[:12]],
        mode=[int(m.mode) for m in msg.motor_state[:12]], lost=[int(m.lost) for m in msg.motor_state[:12]],
        quaternion=list(msg.imu_state.quaternion), gyro=list(msg.imu_state.gyroscope))


def sport(msg):
    return stamp(msg.stamp), dict(error_code=int(msg.error_code), mode=int(msg.mode),
        position=list(msg.position), body_height=float(msg.body_height), velocity=list(msg.velocity),
        yaw_speed=float(msg.yaw_speed), raw_foot_force_dds=list(msg.foot_force),
        foot_position_body=list(msg.foot_position_body), foot_speed_body=list(msg.foot_speed_body))


def odometry(msg):
    pose=msg.pose.pose; twist=msg.twist.twist
    return stamp(msg.header.stamp), dict(frame_id=msg.header.frame_id, child_frame_id=msg.child_frame_id,
        position=xyz(pose.position), quaternion=[float(getattr(pose.orientation,k)) for k in ('w','x','y','z')],
        linear=xyz(twist.linear), angular=xyz(twist.angular),
        pose_covariance=list(msg.pose.covariance), twist_covariance=list(msg.twist.covariance),
        note='position.z is an odometry coordinate, not validated ground clearance')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('interface'); p.add_argument('--seconds', type=float, default=10.)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--sdk-root', type=Path, default=Path.home()/'unitree_sdk2_python')
    args=p.parse_args()
    if not math.isfinite(args.seconds) or not .5 <= args.seconds <= 30:
        p.error('Read-only survey must be 0.5..30 seconds')
    if not (Path('/sys/class/net')/args.interface).is_dir():
        p.error('No such network interface')
    if args.output.exists(): p.error('Refusing to overwrite output')
    sys.path.insert(0,str(args.sdk_root))
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_, SportModeState_
    from unitree_sdk2py.idl.nav_msgs.msg.dds_ import Odometry_
    definitions=[('rt/lowstate',LowState_,low)]
    definitions += [(name,SportModeState_,sport) for name in (
        'rt/sportmodestate','rt/lf/sportmodestate','rt/odommodestate','rt/lf/odommodestate')]
    definitions += [(name,Odometry_,odometry) for name in (
        'rt/lio_sam_ros2/mapping/odometry','rt/lio_sam_ros2/mapping/re_location_odometry')]
    lock=threading.Lock(); rows={name:[] for name,_,_ in definitions}; errors=[]; subscribers=[]
    def callback(name, extract):
        def receive(msg):
            try:
                source,data=extract(msg)
                row=dict(received=time.monotonic(),source=source,data=data,values_finite=finite_tree(data))
                with lock:
                    # Bound memory even if a publisher unexpectedly floods data.
                    if len(rows[name]) < 20000: rows[name].append(row)
                    elif not any(e.get('topic')==name for e in errors):
                        errors.append(dict(topic=name,error='capture limit reached; no acceptance'))
            except Exception as e:
                with lock:
                    if len(errors) < 100: errors.append(dict(topic=name,error=repr(e)))
        return receive
    print('READ ONLY: lowstate/odometry subscribers; no publisher, no service calls, no B2/Z1 control',flush=True)
    ChannelFactoryInitialize(0,args.interface)
    start=time.monotonic(); interrupted=False
    try:
        for name,kind,extract in definitions:
            sub=ChannelSubscriber(name,kind); subscribers.append(sub); sub.Init(callback(name,extract),10)
        time.sleep(args.seconds)
    except KeyboardInterrupt:
        interrupted=True
    finally:
        for sub in reversed(subscribers): sub.Close()
    now=time.monotonic()
    with lock:
        report=dict(read_only=True,control_authorized=False,hardware_qualified=False,
            physical_posture='unknown',elapsed_s=now-start,interrupted=interrupted,errors=list(errors),
            topics={name:summarize(value,now) for name,value in rows.items()},raw=clean(rows))
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as stream: json.dump(report,stream,indent=2,allow_nan=False)
    print(json.dumps({k:v for k,v in report.items() if k!='raw'},indent=2,allow_nan=False),flush=True)


if __name__ == '__main__': main()
