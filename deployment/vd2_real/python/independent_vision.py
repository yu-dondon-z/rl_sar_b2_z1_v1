"""Independent latest-only camera processing, with original per-stream clocks.

Inference is serialized to bound CPU contention; geometry/postprocessing and
publication are independent. No old sample is relabelled as a new capture.
"""
import json
import queue
import threading
import time
import traceback


class LockedSegmenter:
    """Only the network shares a lock; each detector retains its own mask."""
    def __init__(self, segmenter, lock):
        self.segmenter, self.lock = segmenter, lock
        self.last_diagnostic_mask = None

    def predict_bgr(self, image):
        with self.lock:
            result = self.segmenter.predict_bgr(image)
            self.last_diagnostic_mask = result.mask.copy()
            return result


def latest_packet(front, rear, generation, publication):
    return dict(kind='packet', schema='b2_z1_dual_vision_v1',
        stream_mode='independent_latest', publication_sequence=publication,
        capture_generation=generation, front=front['result'], rear=rear['result'],
        valid=bool(front['result']['valid'] and rear['result']['valid']),
        receipt_skew_s=abs(front['meta']['capture_timestamp']-rear['meta']['capture_timestamp']),
        front_processing_ms=front['processing_ms'], rear_processing_ms=rear['processing_ms'],
        processing_ms=max(front['processing_ms'], rear['processing_ms']),
        processing_ms_scope='maximum_independent_stream_latency_not_pair_duration',
        front_source_frame=front['sample']['number'], rear_source_frame=rear['sample']['number'],
        front_path_estimation=True, body_motion_estimation=False,
        timestamp_limit='independent host receipts; no exposure synchronization',
        tool_target_mode='rear_seam_direct', visual_tcp_feedback=False)


def run(output, stop, front, rear, decoder, front_config, rear_config, generation,
        run_dir, imu_history, height, segmenter, writer, failures, viewer, requests):
    from cameras import metadata, offer, rear_covers_work_point, ROOT
    from dual_camera_perception import PerceptionConfig
    from realtime_perception import ObservableRearPerception
    from front_projection import process_front

    lock = threading.Lock()
    cancelled = threading.Event()
    results = {name: queue.Queue(maxsize=1) for name in ('front', 'rear')}

    def process(name, capture, config):
        last = -1
        sequence = 0
        try:
            detector = ObservableRearPerception(PerceptionConfig(
                rhacracknet_root=str(ROOT/'thirdparty/rhacracknet')), LockedSegmenter(segmenter, lock))
            while not stop.is_set() and not cancelled.is_set():
                sample, error = capture.get()
                if sample is None or sample['number'] <= last:
                    stop.wait(.005)
                    continue
                last = sample['number']
                begin = time.monotonic()
                if not -.002 <= begin-sample['received'] <= 1.:
                    continue
                meta = metadata(sample, config, 'front_rgb' if name == 'front' else 'rear_d435i',
                                generation, sequence)
                sequence += 1
                if name == 'front':
                    sample = decoder.decode(sample)
                    result, meta = process_front(detector, meta, sample['image'], {}, imu_history, height)
                else:
                    clocks = dict(rgb_timestamp_domain=sample['timestamp_domain'],
                        depth_timestamp_domain=sample['depth_timestamp_domain'],
                        rgb_device_timestamp_ms=sample['device_time_ms'],
                        depth_device_timestamp_ms=sample['depth_device_time_ms'],
                        rgb_depth_timestamp_skew_ms=abs(
                            sample['device_time_ms']-sample['depth_device_time_ms']))
                    if (clocks['rgb_timestamp_domain'] != clocks['depth_timestamp_domain'] or
                            clocks['rgb_depth_timestamp_skew_ms'] > 40):
                        # Reject only this pair; publish raw images for diagnosis, not geometry.
                        result = dict(valid=False, points_b=[],
                            reason='D435i RGB/depth clocks or device timestamps differ')
                    else:
                        result = detector.process_rear(meta, sample['image'], sample['depth'])
                        sample = dict(sample, raw_mask=detector.segmenter.last_diagnostic_mask,
                                      filtered_mask=getattr(detector, 'last_filtered_mask', None))
                    result.update(clocks)
                    meta.update(clocks)
                result.update({key: meta[key] for key in
                               ('sequence', 'capture_generation', 'capture_timestamp')})
                offer(results[name], dict(sample=sample, meta=meta, result=result,
                    processing_ms=1000*(time.monotonic()-begin)))
        except Exception:
            offer(results[name], dict(error=traceback.format_exc()))

    threads = [threading.Thread(target=process, args=(name, capture, config), daemon=True)
               for name, capture, config in (('front', front, front_config), ('rear', rear, rear_config))]
    for thread in threads:
        thread.start()
    latest = {}
    publication = 0
    saved_valid = saved_working = False
    last_save = last_notice = 0.
    try:
        with (run_dir/'vision.jsonl').open('a') as history:
            while not stop.is_set():
                failures.poll(requests, writer, run_dir)
                changed = False
                for name in ('front', 'rear'):
                    try:
                        item = results[name].get_nowait()
                    except queue.Empty:
                        continue
                    if 'error' in item:
                        raise RuntimeError(name+': '+item['error'])
                    latest[name] = item
                    changed = True
                now = time.monotonic()
                if len(latest) < 2:
                    if now-last_notice > 1.:
                        offer(output, dict(kind='status', valid=False,
                            reason='waiting for independent front/rear results: '+','.join(latest)))
                        last_notice = now
                    stop.wait(.005)
                    continue
                if not changed:
                    stop.wait(.005)
                    continue
                f, r = latest['front'], latest['rear']
                packet = latest_packet(f, r, generation, publication)
                publication += 1
                # Publish before log encoding, GUI copies or diagnostic storage.
                offer(output, packet)
                history.write(json.dumps(packet, allow_nan=False)+'\n')
                history.flush()
                pair = dict(front=f['meta'], rear=r['meta'])
                failures.remember(f['sample'], r['sample'], pair, packet)
                viewer.submit(f['sample'], r['sample'], packet)
                if failures.poll(requests, writer, run_dir):
                    continue
                prefix = None
                if packet['valid'] and not saved_valid:
                    prefix = 'valid_pair_'
                elif packet['valid'] and rear_covers_work_point(r['result']) and not saved_working:
                    prefix = 'working_pair_'
                elif now-last_save > 2.:
                    prefix = ''
                if prefix is not None and writer.submit(run_dir, f['sample'], r['sample'], prefix=prefix, pair=pair):
                    last_save = now
                    saved_valid |= prefix == 'valid_pair_'
                    saved_working |= prefix == 'working_pair_'
    finally:
        cancelled.set()
        for thread in threads:
            thread.join(timeout=2.)
