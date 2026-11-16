"""Real camera worker: SDK front JPEG + aligned/rectified D435i RGB-D.

Host receipt timestamps are independent. They are not exposure timestamps and
cannot establish physical camera synchronization. No simulation pose is used.
"""
import ctypes as ct
from collections import deque
import json
import os
from pathlib import Path
import queue
import signal
import sys
import threading
import time
import traceback
import cv2
import numpy as np
import yaml
from native_io import ROOT, library, check
from realsense_geometry import rectification_maps
from visual_preview import PreviewPublisher

def calibration(path,required=True):
    result=yaml.safe_load(Path(path).read_text())
    if result.get('calibrated') is not True:
        if required:raise ValueError(str(path)+': rear camera extrinsics have not been calibrated')
        return result
    transform=np.asarray(result.get('T_base_camera'),dtype=float)
    if (transform.shape!=(4,4) or not np.isfinite(transform).all()
        or not np.allclose(transform[3],[0,0,0,1])
        or not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-4)
        or not np.isclose(np.linalg.det(transform[:3,:3]),1,atol=1e-4)):
        raise ValueError('invalid measured T_base_camera: '+str(path))
    return result

def offer(output,item):
    try:output.put_nowait(item)
    except queue.Full:
        try:output.get_nowait()
        except queue.Empty:pass
        try:output.put_nowait(item)
        except queue.Full:pass

class Capture:
    def __init__(self):self.latest=None;self.error='waiting for camera';self.lock=threading.Lock()
    def put(self,item):
        with self.lock:self.latest=item;self.error=''
    def get(self):
        with self.lock:return self.latest,self.error

def front_loop(capture,stop,interface,config):
    try:
        lib=library();check(lib,lib.vd2_front_open(interface.encode()))
        raw=(ct.c_ubyte*(4*1024*1024))();counter=0
        intrinsic=config['intrinsics']
        k=np.array([[intrinsic['fx'],0,intrinsic['cx']],[0,intrinsic['fy'],intrinsic['cy']],[0,0,1.]])
        while not stop.is_set():
            requested_at=time.monotonic()
            requested,received=ct.c_double(),ct.c_double()
            count=lib.vd2_front_image(raw,len(raw),ct.byref(requested),ct.byref(received))
            if count<0:
                capture.error='B2 front GetImageSample failed';time.sleep(.05);continue
            counter+=1
            # Copy before the SDK reuses raw. Decode/rectify only the selected
            # pair, instead of every full-HD JPEG that perception will discard.
            capture.put(dict(jpeg=ct.string_at(raw,count),k=k,number=counter,received=received.value,
                             requested=requested.value,device_time_ms=None,
                             timestamp_basis='host_receive_monotonic_no_exposure_timestamp'))
            # The sensor stream is 30 Hz. Removing JPEG work must not turn the
            # SDK request loop into hundreds of RPCs per second.
            stop.wait(max(0.,1/30-(time.monotonic()-requested_at)))
    except Exception as error:capture.error=str(error)


class FrontDecoder:
    """Materialize a selected JPEG without changing its identity or receipt time."""
    def __init__(self,config):
        self.distortion=np.asarray(config['intrinsics']['distortion']);self.maps=None
        self.cached=None

    def decode(self,sample):
        if self.cached is not None and self.cached['number']==sample['number']:
            return self.cached
        image=cv2.imdecode(np.frombuffer(sample['jpeg'],dtype=np.uint8),cv2.IMREAD_COLOR)
        if image is None:raise ValueError('B2 front JPEG decode failed')
        if image.shape[:2]!=(1080,1920):
            raise ValueError('front calibration is for 1920x1080; recalibrate changed resolution')
        if self.maps is None:
            self.maps=cv2.initUndistortRectifyMap(sample['k'],self.distortion,None,sample['k'],
                (1920,1080),cv2.CV_32FC1)
        self.cached=dict(sample,image=cv2.remap(image,*self.maps,interpolation=cv2.INTER_LINEAR))
        return self.cached

def rear_loop(capture,stop,config):
    pipeline=None
    try:
        import pyrealsense2 as rs
        pipeline=rs.pipeline();settings=rs.config()
        devices=list(rs.context().query_devices())
        serial=config.get('serial','')
        if not serial:
            d435=[d for d in devices if 'D435' in d.get_info(rs.camera_info.name)]
            if len(d435)!=1:raise ValueError('select one D435i by setting serial in config/rear_d435i.yaml')
            serial=d435[0].get_info(rs.camera_info.serial_number)
        settings.enable_device(serial)
        settings.enable_stream(rs.stream.color,640,480,rs.format.bgr8,30)
        settings.enable_stream(rs.stream.depth,640,480,rs.format.z16,30)
        profile=pipeline.start(settings);align=rs.align(rs.stream.color)
        scale=profile.get_device().first_depth_sensor().get_depth_scale()
        sensor=profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        k=np.array([[sensor.fx,0,sensor.ppx],[0,sensor.fy,sensor.ppy],[0,0,1.]])
        # D435i can report inverse Brown-Conrady; its coefficients cannot simply
        # be passed to OpenCV as ordinary Brown-Conrady. Use the SDK projection.
        distortion=np.asarray(sensor.coeffs)
        maps=rectification_maps(sensor,rs)
        while not stop.is_set():
            frames=pipeline.wait_for_frames(1000);received=time.monotonic()
            aligned=align.process(frames);color=aligned.get_color_frame();depth=aligned.get_depth_frame()
            if not color or not depth:continue
            image=np.asanyarray(color.get_data()).copy()
            raw_depth=np.asanyarray(depth.get_data()).astype(np.float32)*scale
            raw_depth[raw_depth<=0]=np.nan
            capture.put(dict(image=cv2.remap(image,*maps,interpolation=cv2.INTER_LINEAR),
                raw_image=image,depth=cv2.remap(raw_depth,*maps,interpolation=cv2.INTER_NEAREST,
                    borderMode=cv2.BORDER_CONSTANT,borderValue=float('nan')),
                k=k,distortion=distortion.tolist(),distortion_model=str(sensor.model),serial=serial,
                number=int(color.get_frame_number()),received=received,
                device_time_ms=float(color.get_timestamp()),
                depth_device_time_ms=float(depth.get_timestamp()),
                timestamp_domain=str(color.get_frame_timestamp_domain()),
                depth_timestamp_domain=str(depth.get_frame_timestamp_domain()),
                timestamp_basis='host_receive_monotonic_device_timestamps_retained'))
    except Exception as error:capture.error=str(error)
    finally:
        if pipeline is not None:
            try:pipeline.stop()
            except RuntimeError:pass

def metadata(sample,config,name,generation,sequence):
    return dict(camera_name=name,sequence=sequence,capture_generation=generation,
        capture_timestamp=sample['received'],capture_timestamp_basis=sample['timestamp_basis'],
        source_frame_number=sample['number'],device_timestamp_ms=sample['device_time_ms'],
        intrinsic_matrix=sample['k'].tolist(),T_base_camera=config['T_base_camera'],
        distortion_coefficients=[0.]*5,depth_aligned_to_rgb=True,depth_unit_m=1.)

def paired_metadata(front,rear,front_config,rear_config,generation,sequence,now):
    # No replacing receipt times by inference time, minimum of two times, or a shared timestamp.
    if abs(front['received']-rear['received'])>.04:raise ValueError('host receipt skew exceeds 40 ms')
    if not -.002<=now-min(front['received'],rear['received'])<=1.:raise ValueError('camera sample expired')
    if rear['timestamp_domain']!=rear['depth_timestamp_domain']:
        raise ValueError('D435i RGB/depth clocks differ')
    if abs(rear['device_time_ms']-rear['depth_device_time_ms'])>40:
        raise ValueError('D435i RGB/depth device timestamps differ by over 40 ms')
    return (metadata(front,front_config,'front_rgb',generation,sequence),
            metadata(rear,rear_config,'rear_d435i',generation,sequence))

def save_images(run_dir,front,rear,prefix='',pair=None):
    """Write after publishing a packet so PNG encoding cannot age that packet."""
    options=[cv2.IMWRITE_PNG_COMPRESSION,1]
    cv2.imwrite(str(run_dir/(prefix+'front_latest.png')),front['image'],options)
    cv2.imwrite(str(run_dir/(prefix+'rear_latest.png')),rear['image'],options)
    cv2.imwrite(str(run_dir/(prefix+'rear_raw_for_calibration.png')),rear['raw_image'],options)
    depth_mm=np.nan_to_num(np.clip(rear['depth']*1000,0,65535)).astype(np.uint16)
    cv2.imwrite(str(run_dir/(prefix+'rear_depth_mm.png')),depth_mm,options)
    if pair is not None and 'failure_diagnostic' in pair:
        # PNG quantizes depth to millimetres. Preserve the exact float input
        # as well when explaining a control rejection.
        np.save(run_dir/(prefix+'rear_depth_m.npy'),rear['depth'],allow_pickle=False)
        for key in ('raw_mask','filtered_mask'):
            if rear.get(key) is not None:
                cv2.imwrite(str(run_dir/(prefix+'rear_'+key+'.png')),rear[key],options)
        if front.get('jpeg') is not None:
            (run_dir/(prefix+'front_source.jpg')).write_bytes(front['jpeg'])
    (run_dir/(prefix+'rear_intrinsics.json')).write_text(json.dumps(dict(
        intrinsic_matrix=rear['k'].tolist(),distortion_coefficients=rear['distortion'],
        distortion_model=rear['distortion_model'],image_space='raw_colour_sensor',
        serial=rear['serial'],image_size=[640,480],raw_image_file=prefix+'rear_raw_for_calibration.png'),indent=2))
    if pair is not None:
        (run_dir/(prefix+'pair_metadata.json')).write_text(json.dumps(pair,allow_nan=False,indent=2))

class ImageWriter:
    """Bounded diagnostic IO, independent of capture and perception cadence."""
    def __init__(self):
        self.pending=queue.Queue(maxsize=1);self.closing=threading.Event();self.error=None
        self.thread=threading.Thread(target=self._run,daemon=True);self.thread.start()
    def submit(self,*args,**kwargs):
        if self.error:raise RuntimeError('camera image writer: '+str(self.error))
        try:self.pending.put_nowait((args,kwargs));return True
        except queue.Full:return False
    def _run(self):
        while not self.closing.is_set() or not self.pending.empty():
            try:args,kwargs=self.pending.get(timeout=.1)
            except queue.Empty:continue
            try:save_images(*args,**kwargs)
            except Exception as error:self.error=error;return
    def close(self):
        self.closing.set();self.thread.join(timeout=3)


class FailureFrames:
    """Keep four published pairs so a control rejection saves its exact input.

    Requests affect diagnostic IO only. A busy writer cannot block control or
    perception, and the pending pair remains retained until submitted.
    """
    def __init__(self):
        self.recent=deque(maxlen=4);self.pending=None;self.number=0

    def remember(self,front,rear,pair,packet):
        key=(packet['capture_generation'],packet.get('publication_sequence',packet['front']['sequence']))
        self.recent.append((key,front,rear,pair,packet))

    def poll(self,requests,writer,run_dir):
        if requests is None:return False
        if self.pending is None:
            try:request=requests.get_nowait()
            except queue.Empty:return False
            key=(request['capture_generation'],request['sequence'])
            entry=next((e for e in reversed(self.recent) if e[0]==key),None)
            if entry is None:
                # Explain missing evidence; never substitute another capture.
                (run_dir/('visual_stop_%03d_missing.json'%self.number)).write_text(json.dumps(
                    dict(request=request,reason='requested capture is outside the four-pair diagnostic cache'),indent=2))
                self.number+=1;return False
            self.pending=(request,entry)
        request,(_,front,rear,pair,packet)=self.pending
        prefix=('ready_failure_' if request.get('state')=='ready' else 'visual_stop_')+'%03d_'%self.number
        saved_pair=dict(pair,failure_diagnostic=dict(request=request,packet=packet,
                        exact_depth_file=prefix+'rear_depth_m.npy'))
        if writer.submit(run_dir,front,rear,prefix=prefix,pair=saved_pair):
            self.pending=None;self.number+=1
        return True

def rear_covers_work_point(result):
    points=result.get('points_b',[])
    return bool(result.get('valid') and points
                and min(p[0] for p in points)<=-1.02<=max(p[0] for p in points))

def worker(output,stop,interface,run_dir,generation,preview=False,display=False,device='auto',
           imu_history=None,front_nominal_height_m=None,failure_requests=None,preview_frames=None):
    # The parent handles Ctrl+C and sets the event; avoid a second traceback in
    # this spawned process while it releases the camera streams.
    signal.signal(signal.SIGINT,signal.SIG_IGN)
    # A spawned worker inherits this before its OpenMP runtime initializes.
    # Also set it for direct diagnostic callers before importing torch.
    os.environ['OMP_WAIT_POLICY']='PASSIVE'
    os.environ['OPENBLAS_NUM_THREADS']='1'
    # Avoid twelve OpenCV workers competing with torch and the capture threads
    # on the board's CPU. This changes scheduling, not image dimensions.
    cv2.setNumThreads(1)
    run_dir=Path(run_dir);front_config=calibration(ROOT/'config/front_rgb.yaml')
    rear_config=calibration(ROOT/'config/rear_d435i.yaml',required=not preview)
    decoder=FrontDecoder(front_config)
    front,rear=Capture(),Capture()
    threads=[threading.Thread(target=front_loop,args=(front,stop,interface,front_config),daemon=True),
             threading.Thread(target=rear_loop,args=(rear,stop,rear_config),daemon=True)]
    for thread in threads:thread.start()
    writer=ImageWriter()
    failures=FailureFrames()
    viewer=PreviewPublisher(preview_frames)
    try:
        perception=None
        if not preview:
            import torch
            torch.set_num_threads(2)
            sys.path.insert(0,str(ROOT/'thirdparty/rhacracknet'))
            from realtime_segmenter import RealtimeSegmenter
            from dual_camera_perception import PerceptionConfig
            from realtime_perception import ObservableRearPerception
            segmenter=RealtimeSegmenter(ROOT/'assets/rhanet.pth',device=device,threshold=.90,input_size=(360,640))
            segmenter.warmup(360,640,2)
            perception=ObservableRearPerception(PerceptionConfig(rhacracknet_root=str(ROOT/'thirdparty/rhacracknet')),segmenter)
        if not preview and front_nominal_height_m is not None:
            from independent_vision import run
            run(output,stop,front,rear,decoder,front_config,rear_config,generation,
                run_dir,imu_history,front_nominal_height_m,segmenter,writer,failures,viewer,failure_requests)
            return
        sequence=0;last=(-1,-1);last_save=last_notice=last_valid_save=0
        saved_valid=saved_working=False
        with (run_dir/'vision.jsonl').open('a') as history:
            while not stop.is_set():
                failures.poll(failure_requests,writer,run_dir)
                f,fe=front.get();r,re=rear.get();now=time.monotonic()
                if f is None or r is None:
                    if now-last_notice>1:
                        offer(output,dict(kind='status',valid=False,reason='front: '+(fe or 'ok')+'; rear: '+(re or 'ok')));last_notice=now
                    time.sleep(.01);continue
                if preview and now-last_save>2:
                    if writer.submit(run_dir,decoder.decode(f),r):last_save=now
                if preview:
                    if viewer.due(now):viewer.submit(decoder.decode(f),r)
                    if now-last_notice>1:
                        offer(output,dict(kind='status',valid=False,reason='camera preview; rear calibrated='+str(rear_config['calibrated']),
                            front_age_s=now-f['received'],rear_age_s=now-r['received'],rear_serial=r['serial'],
                            rear_distortion_model=r['distortion_model']))
                        last_notice=now
                    time.sleep(.03);continue
                if f['number']<=last[0] or r['number']<=last[1]:time.sleep(.005);continue
                try:fm,rm=paired_metadata(f,r,front_config,rear_config,generation,sequence,now)
                except ValueError as error:
                    if now-last_notice>1:offer(output,dict(kind='status',valid=False,reason=str(error)));last_notice=now
                    time.sleep(.005);continue
                last=(f['number'],r['number']);sequence+=1
                try:f=decoder.decode(f)
                except ValueError as error:
                    offer(output,dict(kind='status',valid=False,reason=str(error)))
                    continue
                rear_begin=time.monotonic()
                rear_result=perception.process_rear(rm,r['image'],r['depth'])
                # Exact processed rear frame only; capture before front inference
                # replaces the segmenter's diagnostic mask. No extra inference.
                r=dict(r,raw_mask=getattr(segmenter,'last_diagnostic_mask',None),
                       filtered_mask=getattr(perception,'last_filtered_mask',None))
                front_begin=time.monotonic()
                from front_projection import process_front
                front_result,fm=process_front(perception,fm,f['image'],rear_result,
                                             imu_history,front_nominal_height_m)
                processed=time.monotonic()
                # Retain original independent capture identities even on detector failure.
                for item,meta in ((front_result,fm),(rear_result,rm)):
                    item.update({key:meta[key] for key in ('sequence','capture_generation','capture_timestamp')})
                packet=dict(kind='packet',schema='b2_z1_dual_vision_v1',capture_generation=generation,
                    front=front_result,rear=rear_result,
                    valid=bool(front_result['valid'] and rear_result['valid']),
                    receipt_skew_s=abs(f['received']-r['received']),
                    processing_ms=1000*(processed-now),
                    rear_processing_ms=1000*(front_begin-rear_begin),
                    front_processing_ms=1000*(processed-front_begin),
                    front_source_frame=f['number'],rear_source_frame=r['number'],
                    front_rpc_duration_s=f['received']-f['requested'],
                    front_preparation_ms=1000*(rear_begin-now),
                    timestamp_limit='host receipt only; exposure synchronization is not established',
                    inference_layout='channels_last' if segmenter.channels_last else 'default',
                    front_path_estimation=front_nominal_height_m is not None,
                    body_motion_estimation=False,
                    tool_target_mode='rear_seam_direct',visual_tcp_feedback=False)
                pair=dict(front=fm,rear=rm)
                failures.remember(f,r,pair,packet)
                offer(output,packet);history.write(json.dumps(packet,allow_nan=False)+'\n');history.flush()
                # Publish control geometry before any optional GUI copy. Reuse
                # this exact decoded pair and never repeat inference for display.
                viewer.submit(f,r,packet)
                # The normal latest files can later be overwritten by a prone
                # view. Preserve one genuine paired detection for offline replay.
                if failures.poll(failure_requests,writer,run_dir):continue
                if packet['valid'] and not saved_valid:
                    saved_valid=writer.submit(run_dir,f,r,prefix='valid_pair_',pair=pair)
                elif packet['valid'] and rear_covers_work_point(rear_result) and not saved_working:
                    saved_working=writer.submit(run_dir,f,r,prefix='working_pair_',pair=pair)
                elif packet['valid'] and now-last_valid_save>5:
                    if writer.submit(run_dir,f,r,prefix='latest_valid_',pair=pair):last_valid_save=now
                elif now-last_save>2:
                    if writer.submit(run_dir,f,r,pair=pair):last_save=now
    except Exception:
        failure=dict(kind='fatal',valid=False,reason=traceback.format_exc())
        (run_dir/'camera_failure.json').write_text(json.dumps(failure,indent=2))
        print(failure['reason'],file=sys.stderr,flush=True)
        offer(output,failure)
    finally:
        if preview:stop.set()
        failures.poll(failure_requests,writer,run_dir)
        writer.close()
        for thread in threads:thread.join(timeout=2)
