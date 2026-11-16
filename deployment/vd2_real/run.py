#!/usr/bin/env python3
"""Board entry point; --offline-check and --camera-preview never acquire motion."""
from pathlib import Path
import os
os.environ['OMP_WAIT_POLICY']='PASSIVE'
os.environ['OPENBLAS_NUM_THREADS']='1'
import sys
ROOT=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'python'),str(ROOT/'python/vision'),str(ROOT/'thirdparty/rhacracknet')]
import argparse
import ctypes as ct
from datetime import datetime
import gzip
import json
import multiprocessing as mp
import queue
import time
import numpy as np
import torch
import yaml
from native_io import library,check,snapshot,send
from feedback import FeedbackWait,Z1MotorResponse,sample_feedback
from control import Actor,Session,Remote,measured,position_target,DEFAULT,LEGS,ARM
from cameras import calibration,worker,offer
from vd1_visual_guard import GuardedVD1VisualController
from dual_vision_kinematics import URDFToolKinematics
from vision_start import packet_status,VisionStartRequest,StableVisualGeometry
from front_projection import HISTORY_SIZE,publish_gravity
from visual_preview import VisualPreview

def parse_args():
    parser=argparse.ArgumentParser(description=__doc__)
    modes=parser.add_mutually_exclusive_group(required=True)
    modes.add_argument('--offline-check',action='store_true')
    modes.add_argument('--camera-preview',action='store_true')
    modes.add_argument('--preflight',action='store_true',help='B2 states only; Z1 PASSIVE exchanges; no B2 takeover')
    modes.add_argument('--run',action='store_true',help='enable physical takeover on gamepad A')
    parser.add_argument('--interface',default='eth0',help='B2 SDK network interface; verify with ip -br addr')
    parser.add_argument('--seconds',type=float,default=0,help='preview/preflight duration; 0 means until Ctrl+C')
    parser.add_argument('--display',action='store_true')
    parser.add_argument('--vision-device',default='auto',choices=['auto','cpu','cuda:0'])
    return parser.parse_args()

def offline():
    import subprocess
    subprocess.run([str(ROOT/'build/bin/vd2_native_check')],check=True)
    subprocess.run([sys.executable,'-m','unittest','discover','-s',str(ROOT/'tests'),'-v'],check=True,cwd=str(ROOT))

def packet_ok(packet,generation,now):
    return packet_status(packet,generation,now)['valid']

def state_log(state):
    result={name:list(getattr(state,name)) for name in
            ('q','dq','tau','quaternion','omega','foot_force','motor_fault','motor_lost','z1_error','z1_connected')}
    result['z1_transport']=dict(sent_monotonic=state.z1_sent_time,
        source_submitted_monotonic=state.z1_sent_source_time,sent_q=list(state.z1_sent_q),
        sends=state.z1_sends,suppressed_sends=state.z1_suppressed,replies=state.z1_replies,
        scope='native_to_local_controller; reply pacing is not a command acknowledgement')
    return result

def main():
    args=parse_args()
    if args.offline_check:offline();return
    if args.seconds<0:raise ValueError('--seconds must be nonnegative')
    cfg=yaml.safe_load((ROOT/'config/deployment.yaml').read_text())
    if float(cfg['forward_speed_mps'])!=.20 or float(cfg['policy_period_s'])!=.02:
        raise ValueError('selected actor uses the validated 0.20 m/s, 50 Hz deployment contract')
    front_estimation=cfg.get('front_path_estimation',False)
    front_height=cfg.get('front_nominal_base_height_m') if front_estimation else None
    if front_estimation:
        from vd1_front_path_estimator import nominal_front_plane
        nominal_front_plane([0.,0.,-1.],front_height)
    # Known missing hardware calibration is checked before initializing any control.
    if args.run:
        calibration(ROOT/'config/rear_d435i.yaml')
        calibration(ROOT/'config/front_rgb.yaml')
    torch.set_num_threads(1)
    actor=Actor() if args.run else None
    fk=URDFToolKinematics(ROOT/'assets/encoder_fk.urdf') if args.run else None
    run_dir=ROOT/'runs'/datetime.now().strftime('%Y%m%d_%H%M%S_%f');run_dir.mkdir(parents=True)
    (run_dir/'run_metadata.json').write_text(json.dumps(dict(arguments=vars(args),config=cfg,
        hardware_tested=False,policy='VD3 mirror model_2000',
        note='no base velocity, global pose, simulated contacts, or seam truth in actor observations'),indent=2))
    print('Logs and latest camera images: '+str(run_dir),flush=True)
    context=mp.get_context('spawn');output=context.Queue(maxsize=1);stop=context.Event()
    failure_requests=context.Queue(maxsize=1)
    imu_history=context.Array('d',HISTORY_SIZE*4,lock=True)
    process=None;lib=None;io_open=False;acquired=False;session=Session();remote=Remote()
    generation=time.time_ns();packet=None;controller=None;event=None
    start=last_notice=deadline=time.monotonic();last_mode=None;last_vision=None;camera_failed=False
    feedback_wait=FeedbackWait();last_feedback_notice=None
    motor_response=Z1MotorResponse();motor_repeat_notice=False
    visual_start=VisionStartRequest();last_visual_notice=0.
    readiness=StableVisualGeometry();ready_filter=None
    last_ready_failure=-float('inf');last_ready_failure_key=None
    viewer=None
    try:
        if not args.preflight:
            viewer=VisualPreview(context,args.display,run_dir)
            process=context.Process(target=worker,args=(output,stop,args.interface,str(run_dir),generation,
                                   args.camera_preview,args.display,args.vision_device,imu_history,front_height,
                                   failure_requests,viewer.frames),daemon=True)
            process.start()
        if not args.camera_preview:
            lib=library()
            check(lib,lib.vd2_open(args.interface.encode(),cfg['z1_controller_ip'].encode(),cfg['z1_to_port'],cfg['z1_own_port']))
            io_open=True
            print('B2 low commands disabled until A; Z1 PASSIVE IO active. '
                  'Z1 reply age measures the local controller; motor sample age is unavailable.',flush=True)
        with gzip.open(run_dir/'control.jsonl.gz','wt') as log:
            while not stop.is_set():
                now=time.monotonic()
                if args.seconds and now-start>=args.seconds:break
                if process is not None and not process.is_alive():
                    if not args.run:raise RuntimeError('camera worker exited; see camera_failure.json and vision logs')
                    if not camera_failed:print('Camera worker exited: stopping vision tracking and retaining controller ownership.',flush=True)
                    camera_failed=True;packet=None
                try:
                    while True:
                        item=output.get_nowait()
                        if item.get('kind')=='fatal':
                            if not args.run:raise RuntimeError(item['reason'])
                            camera_failed=True;packet=None
                            print(item['reason'],flush=True)
                        if item.get('kind')=='packet' and not camera_failed:packet=item
                        if item.get('kind')=='status':last_vision=item
                except queue.Empty:pass
                if args.camera_preview:
                    viewer.publish_control('camera_preview',(last_vision or {}).get('reason','waiting for cameras'))
                    if last_vision and now-last_notice>1:print(json.dumps(last_vision,ensure_ascii=False),flush=True);last_notice=now
                    time.sleep(.02);continue
                state,now,feedback=sample_feedback(lib,snapshot,time.monotonic)
                feedback.update(motor_response.observe(state,now))
                repeated=feedback['z1_motor_vector_repeated_while_moving']
                if repeated and not motor_repeat_notice:
                    print('[Z1 diagnostic] identical motor q/dq/tau still report motion for %.3f s; '
                          'controller reply freshness does not prove motor feedback updates.' %
                          feedback['z1_motor_vector_unchanged_s'],flush=True)
                motor_repeat_notice=repeated
                fresh=feedback['fresh']
                if args.preflight:
                    if now-last_notice>1:
                        print(json.dumps(dict(**feedback,
                            rear_calibrated=calibration(ROOT/'config/rear_d435i.yaml',False)['calibrated'],
                            **state_log(state))),flush=True);last_notice=now
                    log.write(json.dumps(dict(monotonic=now,**feedback,**state_log(state)))+'\n')
                    log.flush();time.sleep(.02);continue
                feedback_failure=feedback_wait.update(feedback,now,acquired)
                if not fresh:
                    if viewer is not None:viewer.publish_control(session.mode,'waiting for fresh B2/Z1 feedback')
                    if (last_feedback_notice is None or now-last_feedback_notice>=1 or feedback_failure):
                        report=dict(kind='feedback_wait',monotonic=now,state=session.mode,
                            waiting_s=feedback_wait.gap(now),**feedback)
                        print('Feedback: '+json.dumps(report),flush=True)
                        log.write(json.dumps(dict(**report,**state_log(state)),allow_nan=False)+'\n')
                        log.flush();last_feedback_notice=now
                    if feedback_failure:
                        raise RuntimeError(feedback_failure+'; '+json.dumps(feedback))
                    time.sleep(.005);continue
                q,qd,omega,gravity=measured(state)
                if front_estimation:publish_gravity(imu_history,state.b2_time,gravity)
                event=remote.event(state.keys)
                if state.fault:session.transition('fault',now,'native IO fault '+str(state.fault))
                if repeated:session.block_motor_feedback(now)
                if event=='stand' and session.mode=='idle':
                    # The initial pose is the later get-down destination. Do not
                    # accidentally store an already standing pose as "prone".
                    if not (np.all(q[[9,10,11,12]]<-2.) and np.max(np.abs(qd))<.2):
                        print('A ignored: start from the stationary prone pose.',flush=True);event=None
                    elif not acquired:
                        check(lib,lib.vd2_acquire());acquired=True
                        # ReleaseMode RPC can take seconds. Session's first
                        # target must use the post-RPC sample, just like native IO.
                        state,now,feedback=sample_feedback(lib,snapshot,time.monotonic)
                        feedback.update(motor_response.observe(state,now))
                        failure=feedback_wait.update(feedback,now,acquired=True)
                        if failure or state.fault:
                            raise RuntimeError(failure or 'native IO fault after takeover '+str(state.fault))
                        q,qd,omega,gravity=measured(state);deadline=now
                visual_input=packet_status(packet,generation,now)
                if packet is None and last_vision:visual_input['reason']=last_vision.get('reason',visual_input['reason'])
                usable=visual_input['valid']
                previous_mode=session.mode
                remote_event=event
                if previous_mode!='ready':
                    readiness.reset();ready_filter=None
                elif not usable:readiness.reset()
                if previous_mode=='ready' and usable:
                    if front_estimation and ready_filter is None:
                        from vd1_front_path_estimator import FrontPathEstimator
                        ready_filter=FrontPathEstimator()
                    with torch.inference_mode():initial_tcp=fk.position(torch.tensor(q[ARM],dtype=torch.float32)[None])[0].numpy()
                    candidate=GuardedVD1VisualController(initial_tcp,forward_speed=float(cfg['forward_speed_mps']),
                                                        front_path_estimation=front_estimation,
                                                        front_path_filter=ready_filter)
                    geometry=candidate.update(packet,now,.02)['status']
                    usable=readiness.update(packet,geometry);visual_input['valid']=usable
                    visual_input['geometry_valid']=geometry['valid']
                    visual_input['stable_distinct_frames']=readiness.count
                    visual_input['reason']=(geometry['reason'] if not geometry['valid'] else
                        'fresh paired geometry ready' if usable else
                        'waiting for stable distinct geometry (%d/3)'%readiness.count)
                    if not usable and geometry.get('rear_reason'):visual_input['reason']+='; rear: '+geometry['rear_reason']
                event=visual_start.update(event,previous_mode,now,usable)
                if previous_mode=='ready' and not usable and packet is not None:
                    failure_key=(packet['capture_generation'],packet.get('publication_sequence',packet['front']['sequence']))
                    if failure_key!=last_ready_failure_key and now-last_ready_failure>=2.:
                        offer(failure_requests,dict(state='ready',capture_generation=failure_key[0],
                            sequence=failure_key[1],reported_monotonic=now,
                            reason=visual_input['reason'],visual_status=visual_input.copy()))
                        last_ready_failure=now;last_ready_failure_key=failure_key
                if visual_start.message:print('[vision] '+visual_start.message,flush=True)
                if event=='track' and previous_mode=='ready' and usable:
                    controller=GuardedVD1VisualController(initial_tcp,forward_speed=float(cfg['forward_speed_mps']),
                                                         front_path_estimation=front_estimation,
                                                         front_path_filter=ready_filter)
                    session.tool=np.r_[initial_tcp,session.tool[3:]];actor.reset()
                if previous_mode=='ready' and (now-last_visual_notice>=1 or remote_event=='track'):
                    print('[vision] '+json.dumps(dict(pending=visual_start.pending,**visual_input)),flush=True)
                    last_visual_notice=now
                session.event(event,now,q,usable)
                detail={};base=np.zeros(3)
                if session.mode=='tracking':
                    result=controller.update(packet,now,.02)
                    base=np.array(result['base_command_b']);session.tool=np.array(result['tool_pose_b'])
                    detail['visual_status']=result['status'];session.brake_velocity=base.copy()
                    if not result['status']['valid']:
                        if packet is not None:
                            offer(failure_requests,dict(capture_generation=packet['capture_generation'],
                                sequence=packet.get('publication_sequence',packet['front']['sequence']),reported_monotonic=now,
                                reason=result['status']['reason'],visual_status=result['status']))
                        session.event('stop',now,q)
                        session.message='vision stop latched: '+result['status']['reason']
                session.advance(now,q,qd,omega,gravity)
                if session.mode in ('tracking','braking'):
                    if session.mode=='braking':base=session.brake_velocity.copy()
                    target,actor_detail=actor.step(state,base,session.tool)
                    # Keep raw actor history intact. The actuator reference
                    # applies entry blending and latches arm joints on stop.
                    detail['actor_requested_target_q']=target.tolist()
                    session.actor_target(target,now);detail.update(actor_detail)
                    detail['brake_arm_reference_held']=session.mode=='braking'
                    detail['arm_reference_rate_limited']=session.arm_reference_limited
                    detail['arm_reference_rate_limit_rad_s']=session.arm_reference_rate_rad_s
                policy_blend=(min(max(now-session.policy_started,0),1) if session.mode in ('tracking','braking') else
                              max(1-(now-session.entered)/3,0) if session.mode=='return_stand' else 0.)
                if session.mode!='fault' and (session.mode!='idle' or session.prone is not None):
                    bounded,applied=position_target(session.target,actor.bounds)
                    if applied:detail['requested_position_target_q']=session.target.tolist()
                    detail['position_limit_applied']=applied
                    session.target=bounded
                if session.mode in ('tracking','braking'):session.policy_gains(now)
                command=session.target.copy()
                detail['nominal_target_q']=command.tolist()
                preparation=session.preparation_status(now,q)
                if preparation:detail['preparation']=preparation
                if session.prone is not None:
                    from control import FOLD
                    detail['prone_fold_error_rad']=float(np.max(np.abs(q[FOLD]-session.prone[FOLD])))
                    detail['hip_target_error_rad']=float(np.max(np.abs(q[:4]-session.target[:4])))
                gains=dict(leg_kp=session.kp,leg_kd=session.kd)
                # After returning prone, retain the measured initial prone target
                # until the process exits; do not let the watchdog trip silently.
                detail['command_submitted']=(session.mode!='idle' or previous_mode=='lowering'
                                             or session.prone is not None)
                if detail['command_submitted']:
                    detail['command_submit_begin_monotonic']=time.monotonic()
                    send(lib,command,mode=0 if session.mode=='fault' else 1,policy_blend=policy_blend,**gains)
                    detail['command_submit_end_monotonic']=time.monotonic()
                if session.mode!=last_mode:
                    print(session.mode+': '+session.message,flush=True);last_mode=session.mode
                if now-last_notice>1:
                    print('state=%s vision=%s vx_cmd=%.3f z1_mode=%d max_arm_error=%.3f rad max_leg_error=%.3f rad leg_kp=%.1f/%.1f leg_kd=%.1f b2_age_ms=%.3f z1_reply_age_ms=%.3f motor_age=unknown' % (
                        session.mode,usable,base[0],state.z1_mode[0],np.max(np.abs(command[ARM]-q[ARM])),
                        np.max(np.abs(session.target[LEGS]-q[LEGS])),session.kp[0],session.kp[9],session.kd[0],
                        1000*feedback['b2_age_s'],1000*feedback['z1_age_s']),flush=True)
                    last_notice=now
                record=dict(monotonic=now,state=session.mode,event=event,message=session.message,
                    motor_feedback_blocked=session.motor_feedback_blocked,
                    remote_event=remote_event,visual_start_pending=visual_start.pending,visual_input=visual_input,
                    base_command_b=base.tolist(),tool_pose_b=session.tool.tolist(),target_q=command.tolist(),
                    actuator_policy_blend=policy_blend,
                    leg_kp=session.kp.tolist(),leg_kd=session.kd.tolist(),stand_reference='legacy_real_v256',
                    **feedback,
                    **state_log(state),**detail)
                viewer.publish_control(session.mode,session.message+'; '+visual_input['reason'],base[0])
                log.write(json.dumps(record,allow_nan=False)+'\n');log.flush()
                if session.mode=='fault':raise RuntimeError(session.message)
                deadline+=.02;delay=deadline-time.monotonic()
                if delay>0:time.sleep(delay)
                else:deadline=time.monotonic()
        if not args.run and (run_dir/'camera_failure.json').is_file():
            raise RuntimeError((run_dir/'camera_failure.json').read_text())
    except KeyboardInterrupt:
        print('Interrupted; '+('native IO sends damping/PASSIVE before exit.' if io_open
                              else 'camera preview stopping; robot control was not opened.'),flush=True)
    finally:
        if io_open:lib.vd2_close()
        stop.set()
        if process is not None:
            process.join(timeout=4)
            if process.is_alive():process.terminate();process.join(timeout=2)
        if viewer is not None:viewer.close()
        print('Artifacts retained: '+str(run_dir),flush=True)

if __name__=='__main__':main()
