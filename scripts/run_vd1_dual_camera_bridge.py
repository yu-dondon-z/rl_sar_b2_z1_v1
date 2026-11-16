#!/usr/bin/env python3
"""Paired RGB/RGB-D bridge for seam targets without visual TCP markers.

Reuse the upstream frame protocol and image overlays. The original bridge
remains available via --visual-controller original in the launcher.
"""
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import sys
import time


sys.path.insert(0, str(Path(__file__).resolve().parent / 'vision'))
sys.path.insert(0, str(Path(__file__).resolve().parent / 'vision/rhacracknet'))


def process_pair(perception, front_metadata, front_rgb, rear_metadata, rear_rgb, rear_depth,
                 front_nominal_height_m=None):
    """Use either the legacy rear plane or the independent front assumption.

    The optional front mode needs only the front image, its fixed calibration,
    capture-time IMU gravity, and configured height. Rear depth is tool-only.
    """
    try:
        rear = perception.process_rear(rear_metadata, rear_rgb, rear_depth)
    except (KeyError, RuntimeError, TypeError, ValueError) as error:
        rear = dict(valid=False, reason='rear perception: '+str(error))
    try:
        if front_nominal_height_m is None:
            if rear.get('ground_plane_b') is None:
                return dict(valid=False, reason='rear depth ground plane unavailable'), rear
            front_metadata = dict(front_metadata, ground_plane_b=rear['ground_plane_b'],
                                  ground_plane_source='paired_rear_depth')
        else:
            from vd1_front_path_estimator import nominal_front_plane
            plane = nominal_front_plane(front_metadata.get('imu_gravity_b'), front_nominal_height_m)
            front_metadata = dict(front_metadata, ground_plane_b=plane.tolist(),
                                  ground_plane_source='imu_gravity_and_configured_nominal_height')
        front = perception.process_front(front_metadata, front_rgb)
        if front_nominal_height_m is not None:
            front.update(geometry_source='front_rgb_imu_nominal_height_flat_floor',
                         ground_plane_source=front_metadata['ground_plane_source'],
                         nominal_base_height_m=front_nominal_height_m, rear_depth_used=False)
    except (KeyError, RuntimeError, TypeError, ValueError) as error:
        front = dict(valid=False, reason='front perception: '+str(error))
    return front, rear


def main():
    import cv2
    import numpy as np
    from dual_camera_perception import PerceptionConfig
    from vd1_visual_rear import SeamOnlyDualPerception
    from run_dual_camera_bridge import (SCHEMA, capture_key, depth_panel, overlay_frame,
                                        parse_args, read_frame, result_with_capture,
                                        write_json_atomic)

    # Extend the existing parser without changing the training/board bridge.
    import argparse
    extra = argparse.ArgumentParser(add_help=False)
    extra.add_argument('--front-nominal-height-m', type=float)
    extra.add_argument('--gun-mask-urdf', type=Path,
                       help='Exclude installed gun using capture-time arm encoders and this URDF.')
    extra.add_argument('--gun-mask-near-clip', action='store_true',
                       help='VD12: clip the long gun at the camera near plane before self-exclusion.')
    extra.add_argument('--gun-mask-near-plane-m', type=float, default=.08,
                       help='Fixed render near clipping depth, in metres; used only with --gun-mask-near-clip.')
    extra.add_argument('--rear-centerline-mode', choices=('skeleton', 'cross_sections'), default='skeleton')
    extra.add_argument('--rear-component-mode', choices=('anchor', 'workpoint_support', 'vd14_rgb_evidence'), default='anchor',
                       help='VD12: validate separate visible components against measured work-point geometry.')
    extra.add_argument('--rear-work-distance-m', type=float, default=.8)
    extra.add_argument('--rear-target-occlusion-extension-m', type=float, default=.05)
    front_args, remaining = extra.parse_known_args()
    sys.argv[1:] = remaining
    args = parse_args()
    if front_args.front_nominal_height_m is not None:
        from vd1_front_path_estimator import nominal_front_plane
        nominal_front_plane([0., 0., -1.], front_args.front_nominal_height_m)
    if args.max_frames < 0 or args.max_frame_age_s <= 0 or args.poll_period <= 0:
        raise ValueError('frame count must be nonnegative; timing limits must be positive')
    if args.input_height <= 0 or args.input_width <= 0:
        raise ValueError('inference image dimensions must be positive')
    root = args.rhacracknet_root.expanduser().resolve()
    sys.path.insert(0, str(root))
    segmenter = None
    if args.mode == 'rhanet':
        from crack_inference import CrackSegmenter
        segmenter = CrackSegmenter(args.checkpoint, args.model, args.device, args.threshold,
                                   input_size=(args.input_height, args.input_width))
        segmenter.warmup(args.input_height, args.input_width)
    gun_mask = None
    if front_args.gun_mask_near_clip and front_args.gun_mask_urdf is None:
        raise ValueError('--gun-mask-near-clip requires --gun-mask-urdf')
    if front_args.gun_mask_urdf is not None:
        from vd5_gun_mask import EncoderGunMask
        mask_type = EncoderGunMask
        if front_args.gun_mask_near_clip:
            from vd12_gun_mask import NearClippedEncoderGunMask
            mask_type = NearClippedEncoderGunMask
            gun_mask = mask_type(front_args.gun_mask_urdf, near_m=front_args.gun_mask_near_plane_m)
        else:
            gun_mask = mask_type(front_args.gun_mask_urdf)
    perception_type = SeamOnlyDualPerception
    selection_arguments = {}
    if front_args.rear_component_mode in ('workpoint_support', 'vd14_rgb_evidence'):
        from vd12_visual_rear import VD12RearSupportPerception
        perception_type = VD12RearSupportPerception
        if front_args.rear_component_mode == 'vd14_rgb_evidence':
            from vd14_visual_rear import VD14RearEvidencePerception
            perception_type = VD14RearEvidencePerception
        selection_arguments = dict(trailing_distance=front_args.rear_work_distance_m,
            max_occlusion_extension=front_args.rear_target_occlusion_extension_m)
    perception = perception_type(
        config=PerceptionConfig(mode=args.mode, rhacracknet_root=str(root)), segmenter=segmenter,
        gun_mask=gun_mask, rear_centerline_mode=front_args.rear_centerline_mode, **selection_arguments)
    exchange = args.exchange_dir.expanduser().resolve()
    exchange.mkdir(parents=True, exist_ok=True)
    last_key = status_key = None
    processed = 0
    print('[Dual vision] rear seam directly sets the tool target; TCP markers/feedback disabled.', flush=True)
    write_json_atomic(exchange/'vision_ready.json', dict(
        ready=True, mode=args.mode, model=args.model if segmenter else None,
        tool_target_mode='rear_seam_direct', requires_tcp_markers=False,
        visual_tcp_feedback=False, timestamp=time.time()))
    with (exchange/'dual_vision_packets.jsonl').open('a', encoding='utf-8') as history:
        try:
            while args.max_frames == 0 or processed < args.max_frames:
                try:
                    front_metadata, front_rgb, _ = read_frame(exchange, 'front', False)
                    rear_metadata, rear_rgb, rear_depth = read_frame(exchange, 'rear', True)
                    key = capture_key(front_metadata)
                    if key != capture_key(rear_metadata) or key == last_key:
                        time.sleep(args.poll_period)
                        continue
                    if abs(front_metadata['capture_timestamp']-rear_metadata['capture_timestamp']) > .04:
                        raise ValueError('paired camera captures differ by more than 40 ms')
                except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
                    time.sleep(args.poll_period)
                    continue
                last_key = key
                processed += 1
                start = time.monotonic()
                oldest_capture = min(front_metadata['capture_timestamp'], rear_metadata['capture_timestamp'])
                if start-oldest_capture > args.max_frame_age_s or oldest_capture > start+.01:
                    rear = dict(valid=False, reason='capture timestamp is stale or in the future')
                    front = dict(rear)
                else:
                    front, rear = process_pair(perception, front_metadata, front_rgb,
                                                rear_metadata, rear_rgb, rear_depth,
                                                front_args.front_nominal_height_m)
                front = result_with_capture(front, front_metadata)
                rear = result_with_capture(rear, rear_metadata)
                now = time.monotonic()
                age = now-oldest_capture
                if age > args.max_frame_age_s:
                    for result in (front, rear):
                        result.update(valid=False, reason='capture expired during processing')
                valid = bool(front['valid'] and rear['valid'])
                reason = 'paired front steering and rear seam measurements valid' if valid else (
                    "front: %s; rear: %s" % (front['reason'], rear['reason']))
                packet = dict(schema=SCHEMA, timestamp=time.time(), capture_generation=key[0],
                              mode=args.mode, model=args.model if segmenter else None,
                              front=front, rear=rear, valid=valid, reason=reason,
                              processing_ms=1000*(now-start), capture_age_s=age,
                              tool_target_mode='rear_seam_direct', visual_tcp_feedback=False,
                              requires_tcp_markers=False)
                write_json_atomic(exchange/'latest_dual_trajectory.json', packet)
                write_json_atomic(exchange/'latest_vision_status.json', dict(
                    schema=SCHEMA, valid=valid, reason=reason,
                    front_sequence=front['sequence'], rear_sequence=rear['sequence'],
                    mode=args.mode, processing_ms=packet['processing_ms'], capture_age_s=age,
                    tool_target_mode='rear_seam_direct', requires_tcp_markers=False))
                history.write(json.dumps(packet, allow_nan=False)+'\n')
                history.flush()
                if (valid, reason) != status_key or processed % 25 == 0:
                    print('[Dual vision] pair=%s %s age=%.3fs processing=%.1fms %s' % (
                        key, 'VALID' if valid else 'WAIT', age, packet['processing_ms'], reason), flush=True)
                    status_key = (valid, reason)
                if args.save_overlay or args.display:
                    rear_overlay = overlay_frame(rear_rgb, rear, 'Rear RGB-D / observed seam')
                    # Perception VALID means a measured segment, not that the
                    # control-time work point is covered. Display that state
                    # separately; this file is display-only, never control input.
                    try:
                        control = json.loads((exchange/'latest_control_status.json').read_text())
                        control_age = time.monotonic()-control['control_update_monotonic_s']
                        if 0. <= control_age <= 2.:
                            waiting = control['control_state'] == 'rear_support_wait'
                            text = 'Control: '+('WAIT - work point not supported' if waiting else control['control_state'])
                            required = control.get('rear_support_required_extension_m')
                            allowed = control.get('rear_support_allowed_extension_m')
                            if waiting and required is not None and allowed is not None:
                                text += ' (%.1f > %.1f mm)' % (1000*required, 1000*allowed)
                            cv2.putText(rear_overlay, text, (10, 111), cv2.FONT_HERSHEY_SIMPLEX,
                                        .43, (0, 190, 255) if waiting else (0, 220, 0), 1, cv2.LINE_AA)
                    except (OSError, KeyError, TypeError, ValueError):
                        pass
                    if not rear.get('valid') and rear.get('rear_candidate_waypoints_uv'):
                        candidates = np.rint(rear['rear_candidate_waypoints_uv']).astype(np.int32)
                        cv2.polylines(rear_overlay, [candidates], False, (0, 180, 255), 2, cv2.LINE_AA)
                        cv2.putText(rear_overlay, 'Amber: rejected visible fragment %.1f cm' % (
                            100*rear['rear_candidate_path_length_m']), (10, 108),
                            cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 180, 255), 1, cv2.LINE_AA)
                    exclusion = np.asarray(rear.get('gun_exclusion_contour_uv', []), dtype=np.int32)
                    if exclusion.ndim == 2 and exclusion.shape[1] == 2 and len(exclusion) >= 3:
                        cv2.polylines(rear_overlay, [exclusion], True, (255, 100, 20), 2, cv2.LINE_AA)
                        cv2.putText(rear_overlay, 'Blue: gun excluded (encoders)', (10, 84),
                                    cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 100, 20), 1, cv2.LINE_AA)
                    panels = [overlay_frame(front_rgb, front, 'Front RGB / steering'),
                              rear_overlay,
                              depth_panel(rear_depth, float(rear_metadata.get('depth_unit_m', .001)))]
                    height = min(360, front_rgb.shape[0], rear_rgb.shape[0])
                    panel = np.hstack([cv2.resize(item, (640, height)) for item in panels])
                    if args.save_overlay:
                        temporary = exchange/('.latest_dual_overlay.%d.tmp.png' % os.getpid())
                        if cv2.imwrite(str(temporary), panel):
                            temporary.replace(exchange/'latest_dual_overlay.png')
                    if args.display:
                        window = 'B2+Z1: front RGB | rear seam target | rear depth'
                        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
                        if processed == 1:
                            cv2.resizeWindow(window, 1280, 300)
                            cv2.moveWindow(window, 40, 80)
                        cv2.imshow(window, panel)
                        if cv2.waitKey(1) & 0xFF in (27, ord('q')):
                            break
        except KeyboardInterrupt:
            pass
        finally:
            if args.display:
                cv2.destroyAllWindows()
            write_json_atomic(exchange/'vision_finished.json', dict(
                processed_pairs=processed, timestamp=datetime.now(timezone.utc).isoformat(),
                mode=args.mode, tool_target_mode='rear_seam_direct'))


if __name__ == '__main__':
    main()
