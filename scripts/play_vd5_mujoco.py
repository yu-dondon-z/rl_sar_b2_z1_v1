#!/usr/bin/env python3
"""Run VD5 Startup model_4498 from rendered front RGB and rear RGB-D perception."""

from __future__ import annotations

import argparse
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import sys
import subprocess
import tempfile
import time

import numpy as np

from vd5_mujoco_policy import (VD5_CHECKPOINT_SHA256, VD5_MANIFEST, VD5_POLICY,
                               VD5_POLICY_SHA256)


DEPLOY_ROOT = Path(__file__).resolve().parents[1]
TRAINING_ROOT = Path("/home/dong/b2_z1")
sys.path.insert(0, str(DEPLOY_ROOT / "scripts"))
sys.path.insert(0, str(TRAINING_ROOT / "scripts/vision"))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--policy", type=Path, default=VD5_POLICY)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--speed", type=float, default=0.20)
    parser.add_argument("--tcp-trailing-distance", type=float, default=0.70,
                        help="TCP work-point arc distance behind the base, in metres (default: 0.70).")
    parser.add_argument('--rear-occlusion-extension', type=float, default=0.0,
                        help='Explicit diagnostic: estimate up to 0.05 m past a reliable visible rear endpoint; default 0.')
    parser.add_argument('--rear-centerline-mode', choices=('skeleton', 'cross_sections'), default='skeleton')
    parser.add_argument('--rear-hold-s', type=float, default=0.,
                        help='Explicit body-relative target hold for short rear geometry loss, at most 0.30 s.')
    parser.add_argument('--rear-path-memory-s', type=float, default=0.,
                        help='Experimental observed rear path buffer, moved by sent commands; at most 0.8 s.')
    parser.add_argument("--curvature", type=float, default=0.05,
                        help="Rendered road curvature, in 1/m; never sent to the visual controller.")
    parser.add_argument("--physics-dt", type=float, choices=(0.001, 0.005), default=0.005)
    parser.add_argument("--output-root", type=Path, default=DEPLOY_ROOT / "outputs/VD5_MUJOCO")
    parser.add_argument("--cache-dir", type=Path, default=Path("/dev/shm/vd5_mujoco_cache"))
    parser.add_argument("--velocity-constraint", choices=("off", "impulse"), default="impulse")
    parser.add_argument("--vision-mode", choices=("rhanet", "classical"), default="rhanet",
                        help="RHANet is the real detector; classical is an explicit dark-tape diagnostic.")
    parser.add_argument("--vision-device", default="cuda:0")
    parser.add_argument("--vision-max-age", type=float, default=1.0)
    parser.add_argument("--no-vision-window", action="store_true")
    parser.add_argument("--headless", action="store_true", help="Hide MuJoCo and perception windows; keep camera-time pacing.")
    args = parser.parse_args()
    if not 0.0 < args.duration <= 600.0 or not 0.0 < args.speed <= 0.4:
        parser.error("duration or speed is outside the supported range")
    if not 0.0 <= args.curvature <= 0.15:
        parser.error("curvature must be in [0, 0.15] 1/m")
    if not 0.10 <= args.vision_max_age <= 2.0:
        parser.error("vision maximum age must be between 0.10 and 2.0 seconds")
    if not 0.60 <= args.tcp_trailing_distance <= 1.0:
        parser.error("TCP trailing distance must be between 0.60 and 1.0 metres")
    if not 0. <= args.rear_occlusion_extension <= .05:
        parser.error('rear occlusion extension must be between zero and 0.05 metres')
    if not 0. <= args.rear_hold_s <= .30:
        parser.error('rear hold must be between zero and 0.30 seconds')
    if not 0. <= args.rear_path_memory_s <= .8:
        parser.error('rear path memory must be between zero and 0.8 seconds')
    if args.rear_path_memory_s and args.rear_hold_s:
        parser.error('choose rear path memory or rear target hold separately')
    return args


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def save_json(path: Path, value):
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def json_default(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError("Object is not JSON serializable: " + type(value).__name__)


def load_visual_packet(path, publisher):
    try:
        packet = json.loads(path.read_text(encoding="utf-8"))
        accepted, reason = publisher.validate_packet(packet)
        return (packet, "") if accepted else (None, reason)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        return None, str(error)


class EvaluationRecorder:
    """Simulation truth output, separate from all actor and command inputs."""

    def __init__(self, mujoco, model, seam_points):
        self.mujoco = mujoco
        self.model = model
        self.seam = np.asarray(seam_points, dtype=np.float64)
        self.base_id = model.body("base_link").id
        self.tcp_id = model.site("tool_tcp").id
        self.foot_ids = np.array([model.body(name + "_foot").id
                                  for name in ("FL", "FR", "RL", "RR")], dtype=int)
        self.rows = []

    def _distance(self, point):
        delta = self.seam[1:] - self.seam[:-1]
        length2 = np.maximum(np.sum(delta*delta, axis=1), 1e-12)
        fraction = np.clip(np.sum((point[:2] - self.seam[:-1, :2]) * delta[:, :2], axis=1)
                           / length2, 0.0, 1.0)
        projection = self.seam[:-1] + fraction[:, None]*delta
        return float(np.linalg.norm(projection[:, :2] - point[:2], axis=1).min())

    def record(self, data, step):
        base = data.xpos[self.base_id].copy()
        tcp = data.site_xpos[self.tcp_id].copy()
        feet = data.xpos[self.foot_ids].copy()
        rotation = data.xmat[self.base_id].reshape(3, 3)
        tilt = math.degrees(math.acos(float(np.clip(rotation[2, 2], -1.0, 1.0))))
        foot_force = np.zeros((4, 3), dtype=np.float64)
        contact_force = np.zeros(6, dtype=np.float64)
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            body1 = self.model.geom_bodyid[contact.geom1]
            body2 = self.model.geom_bodyid[contact.geom2]
            mujoco = self.mujoco
            mujoco.mj_contactForce(self.model, data, contact_index, contact_force)
            force_w = contact.frame.reshape(3, 3).T @ contact_force[:3]
            for foot_index, foot_id in enumerate(self.foot_ids):
                if body1 == foot_id:
                    foot_force[foot_index] -= force_w
                if body2 == foot_id:
                    foot_force[foot_index] += force_w
        velocity = np.zeros(6, dtype=np.float64)
        self.mujoco.mj_objectVelocity(self.model, data, self.mujoco.mjtObj.mjOBJ_XBODY,
                                      self.base_id, velocity, 1)
        row = {
            "step": int(step), "sim_time_s": float(data.time),
            "base_position_w": base.tolist(),
            "base_cross_track_m": self._distance(base),
            "base_yaw_rad": float(math.atan2(rotation[1, 0], rotation[0, 0])),
            "base_velocity_b": velocity[3:].tolist(), "tilt_deg": tilt,
            "tcp_position_w": tcp.tolist(),
            "tcp_projection_error_m": self._distance(tcp),
            "tcp_height_above_ground_m": float(tcp[2]),
            "foot_position_w": feet.tolist(),
            "foot_seam_center_distance_m": [self._distance(p) for p in feet],
            "foot_net_force_w": foot_force.tolist(),
            "foot_support_gt10n": (foot_force[:, 2] > 10.0).tolist(),
            "foot_seam_buffer_m": 0.052,
            "note": "evaluation only; never returned to actor or command generator",
        }
        self.rows.append(row)
        return row

    def summary(self):
        def summarize(key, rows):
            values = np.asarray([row[key] for row in rows], dtype=np.float64)
            if values.size == 0:
                return None
            return {"mean": float(np.mean(values)), "p95": float(np.percentile(values, 95)),
                    "max": float(np.max(values))}

        settled = [row for row in self.rows if row["sim_time_s"] >= 2.0]
        feet = [row for row in settled if any(
            contact and distance < row["foot_seam_buffer_m"]
            for contact, distance in zip(row["foot_support_gt10n"], row["foot_seam_center_distance_m"]))]
        support_samples = sum(sum(row["foot_support_gt10n"]) for row in settled)
        under_seam_samples = sum(sum(
            bool(contact and distance < row["foot_seam_buffer_m"])
            for contact, distance in zip(row["foot_support_gt10n"], row["foot_seam_center_distance_m"]))
            for row in settled)
        return {
            "sample_count": len(self.rows), "settled_sample_count_after_2s": len(settled),
            "tcp_projection_error_m": summarize("tcp_projection_error_m", settled),
            "base_cross_track_m": summarize("base_cross_track_m", settled),
            "tilt_deg": summarize("tilt_deg", settled),
            "base_forward_speed_mps": summarize("base_velocity_b", [
                dict(base_velocity_b=[row["base_velocity_b"][0]]) for row in settled]),
            "tcp_height_above_ground_m": summarize("tcp_height_above_ground_m", settled),
            "support_foot_samples": support_samples,
            "support_foot_samples_inside_52mm_seam_buffer": under_seam_samples,
            "support_foot_seam_buffer_fraction": (under_seam_samples/support_samples
                                                    if support_samples else None),
            "foot_rows_with_support_near_seam": len(feet),
            "interpretation": "5.2 cm buffer is half the 4 cm tape width plus 3.2 cm foot radius; tape has no collision.",
        }


def main():
    args = parse_args()
    if args.headless:
        os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    import torch
    if sha256(args.policy) != VD5_POLICY_SHA256:
        raise ValueError("VD5 policy SHA256 mismatch; run export_vd5_policy.py and review the manifest")
    if not VD5_MANIFEST.is_file():
        raise FileNotFoundError("Missing VD5 policy manifest: " + str(VD5_MANIFEST))
    manifest = json.loads(VD5_MANIFEST.read_text(encoding="utf-8"))
    if manifest.get("checkpoint_sha256") != VD5_CHECKPOINT_SHA256:
        raise ValueError("VD5 manifest does not identify the reviewed model_4498 checkpoint")
    actor = torch.jit.load(str(args.policy), map_location="cpu").eval()
    with torch.inference_mode():
        if tuple(actor(torch.zeros(1, 70)).shape) != (1, 18):
            raise ValueError("VD5 actor must map observations (1,70) to actions (1,18)")
    torch.set_num_threads(1)

    from vd1_frames import VD1FramePublisher
    from vd1_mujoco_contract import VD1MujocoAdapter
    from vd1_mujoco_scene import (MARKER_CALIBRATION, VD3_TCP_POSITION, VD3_TCP_ROTATION,
                                  VD3_URDF_PATH, build_scene, validate_vd3_model)
    from vd5_camera_geometry import measured_camera_calibration
    from vd1_mujoco_velocity_impulse import VelocityImpulseConstraints
    from vd1_control import build_actor_observation
    from vd1_scene import road_stations
    from vd5_visual_control import VD5VisualController, encoder_tcp_pose

    args.output_root.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    output = Path(tempfile.mkdtemp(prefix="vd5_" + stamp + "_", dir=str(args.output_root.resolve())))
    cache = args.cache_dir.expanduser().resolve() / output.name
    camera_calibration = measured_camera_calibration()
    scene = build_scene(args.curvature, cache, timestep=args.physics_dt, tool_markers=False,
                        road_surface="tape", robot_version="vd3",
                        camera_calibration=camera_calibration)
    model = mujoco.MjModel.from_xml_path(str(scene))
    model_validation = validate_vd3_model(model)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("vd1_default").id)
    # VD5 Startup trains its ordinary-standing reset at 0.50 m, not the legacy 0.70 m keyframe.
    data.qpos[2] = 0.50
    mujoco.mj_forward(model, data)
    adapter = VD1MujocoAdapter(model, physx_implicit_position=args.physics_dt == 0.005)
    velocity_constraint = (VelocityImpulseConstraints(model, adapter.joint_ids18, adapter.velocity_limit18)
                          if args.velocity_constraint == "impulse" else None)
    seam = np.column_stack((road_stations(args.curvature), np.zeros(len(road_stations(args.curvature)))))
    base_id = model.body("base_link").id
    evaluation = EvaluationRecorder(mujoco, model, seam)
    trace = gzip.open(str(output / "actor_trace.jsonl.gz"), "wt", compresslevel=3)
    truth = gzip.open(str(output / "evaluation_truth.jsonl.gz"), "wt", compresslevel=3)

    def calibration(name):
        camera = camera_calibration[name]
        return {"position": list(camera["position_b"]),
                "quaternion_wxyz": list(camera["quaternion_ros_wxyz"]),
                "intrinsic_matrix": camera['intrinsic_matrix']}

    publisher = VD1FramePublisher(output, calibration("front_rgb"),
                                  calibration("rear_d435i"), MARKER_CALIBRATION)
    renderer = mujoco.Renderer(model, height=480, width=640)
    scene_options = mujoco.MjvOption()
    scene_options.geomgroup[3] = 0
    camera_frame = [0]

    def capture():
        frame = camera_frame[0]
        renderer.disable_depth_rendering()
        renderer.update_scene(data, camera="front_rgb", scene_option=scene_options)
        front_rgb = renderer.render().copy()
        renderer.update_scene(data, camera="rear_d435i", scene_option=scene_options)
        rear_rgb = renderer.render().copy()
        renderer.enable_depth_rendering()
        rear_depth = renderer.render().copy()
        renderer.disable_depth_rendering()
        publisher.publish(front_rgb, rear_rgb, rear_depth,
                          front_frame=frame, rear_frame=frame,
                          capture_timestamp=time.monotonic(), sim_time_s=float(data.time),
                          rear_arm_joint_positions=adapter.read_proprioception(data)['q'][arm_indices])
        camera_frame[0] += 1

    bridge = DEPLOY_ROOT / "scripts/run_vd1_dual_camera_bridge.py"
    if not bridge.is_file():
        raise FileNotFoundError("Missing deployed dual-camera perception bridge: " + str(bridge))
    initial = adapter.read_proprioception(data)
    arm_indices = [8, 13, 14, 15, 16, 17]
    tcp_start_b, quat_start_b = encoder_tcp_pose(initial["q"][arm_indices], VD3_URDF_PATH,
                                                 VD3_TCP_POSITION, VD3_TCP_ROTATION)
    controller = VD5VisualController(tcp_start_b, quat_start_b, speed=args.speed,
                                     trailing_distance=args.tcp_trailing_distance,
                                     stale_timeout_s=args.vision_max_age,
                                     max_occlusion_extension=args.rear_occlusion_extension,
                                     rear_hold_s=args.rear_hold_s,
                                     rear_path_memory_s=args.rear_path_memory_s)
    viewer = None
    if not args.headless:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(model, data)
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 3.4, 135.0, -25.0
        viewer.opt.geomgroup[3] = 0
    stopped = [False]
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: stopped.__setitem__(0, True))

    vision_command = [sys.executable, str(bridge), "--exchange-dir", str(output),
                      "--mode", args.vision_mode, "--device", args.vision_device,
                      "--gun-mask-urdf", str(VD3_URDF_PATH),
                      "--rear-centerline-mode", args.rear_centerline_mode,
                      "--save-overlay", "--max-frame-age-s", str(args.vision_max_age)]
    show_vision = not args.headless and not args.no_vision_window
    if show_vision:
        vision_command.append("--display")
    vision_log = (output / "vision_bridge.log").open("w", encoding="utf-8")
    vision_env = os.environ.copy()
    vision_env["PYTHONUNBUFFERED"] = "1"
    vision_process = subprocess.Popen(vision_command, stdout=vision_log,
                                      stderr=subprocess.STDOUT, env=vision_env)

    contract = {
        "strategy": "VD5 Startup model_4498", "checkpoint_sha256": VD5_CHECKPOINT_SHA256,
        "policy_sha256": VD5_POLICY_SHA256, "robot": "B2+Z1 VD3 zero-added-mass gun model",
        "robot_model_validation": model_validation, "scene": str(scene),
        "speed_mps": args.speed, "curve_max_curvature_per_m": args.curvature,
        "tape_physics": "flat ground; visual tape has no collision or height",
        "actor_input": "70 values: base command, 7D base-frame TCP target, gyro, gravity, q, qd, previous action",
        "actor_output": "18 joint offsets, scaled by 0.25 around default joint pose",
        "observation_normalizer": False, "startup_ramp_s": 2.0,
        "tcp_seam_trailing_distance_m": args.tcp_trailing_distance, "tool_target_orientation_euler_xyz_rad": [
            0.0, math.pi/2.0-0.45, math.pi],
        "front_command_source": "rendered front RGB -> RHANet centerline -> calibrated ray/paired rear-depth plane -> visible-path steering",
        "rear_command_source": ("rendered rear RGB-D -> RHANet centerline -> aligned depth projection -> "
                                f"{args.tcp_trailing_distance:.2f} m trailing target; endpoint estimates labelled separately"),
        "camera_calibration": camera_calibration,
        "perception_mode": args.vision_mode,
        "vision_process": str(bridge),
        "vision_capture_max_age_s": args.vision_max_age,
        "camera_simulation_limits": "MuJoCo perspective/occlusion/lighting and aligned metric depth; no lens distortion, sensor noise, or D435i depth artifacts",
        "capture_history_frame_provenance_validated": True,
        "target_frame_motion_estimation": bool(args.rear_path_memory_s),
        "rear_history_motion_source": "sent base command integration" if args.rear_path_memory_s else None,
        "rear_history_motion_is_measured_odometry": False,
        "rear_path_memory_s": args.rear_path_memory_s,
        "rear_target_near_field_method": "measured rear centerline tangent, max 0.25 m extension past visible endpoint",
        "rear_occlusion_extension_limit_m": args.rear_occlusion_extension,
        "rear_centerline_mode": args.rear_centerline_mode,
        "rear_hold_s": args.rear_hold_s,
        "rear_occlusion_extension_method": "explicit local visible-line estimate; >=8 points, >=0.12 m support, residual P90<=8 mm",
        "tcp_start_pose_source": "arm_encoder_forward_kinematics_from_source_URDF",
        "actor_privileged_state_inputs": [],
        "evaluation_truth_returned_to_control": False,
        "simulator_only": True, "hardware_io": False,
        "actor_hz": 50, "joint_pd_hz": 200, "physics_dt_s": args.physics_dt,
        "velocity_constraint": args.velocity_constraint,
    }
    save_json(output / "runtime_contract.json", contract)
    print(f"[VD5 MuJoCo] session: {output}", flush=True)
    print(f"[VD5 MuJoCo] strategy=model_4498 speed={args.speed:.2f} m/s curvature={args.curvature:.3f}/m "
          f"TCP trail={args.tcp_trailing_distance:.2f} m occlusion_estimate_limit={args.rear_occlusion_extension:.2f} m "
          f"rear_centerline={args.rear_centerline_mode} rear_hold={args.rear_hold_s:.2f} s "
          f"rear_path_memory={args.rear_path_memory_s:.2f} s",
          flush=True)
    print("[VD5 MuJoCo] rendered front RGB + rear RGB-D through dual-camera perception; no robot I/O.", flush=True)
    if viewer is not None:
        print("[VD5 MuJoCo] close viewer or press Ctrl-C to stop.", flush=True)

    last_action = np.zeros(18, dtype=np.float32)
    steps = int(math.ceil(args.duration / 0.02))
    stop_reason, runtime_error = "max_steps", None
    valid_commands = active_commands = held_commands = memory_commands = 0
    latest_reason = None
    try:
        warmup_deadline = time.monotonic() + 45.0
        packet = None
        rejection = "waiting for vision bridge startup"
        while not stopped[0]:
            if viewer is not None and not viewer.is_running():
                stop_reason = "viewer_closed_before_vision_ready"
                break
            if vision_process.poll() is not None:
                raise RuntimeError("vision bridge exited during startup; see " + str(output / "vision_bridge.log"))
            capture()
            packet, rejection = load_visual_packet(output / "latest_dual_trajectory.json", publisher)
            if packet is not None and packet.get("valid") is True:
                break
            if time.monotonic() >= warmup_deadline:
                raise RuntimeError("no valid paired camera packet within 45 s: " + rejection)
            if viewer is not None:
                viewer.sync()
            time.sleep(0.04)
        if stop_reason == "viewer_closed_before_vision_ready":
            raise RuntimeError("MuJoCo viewer closed before the first valid camera packet")
        print("[VD5 MuJoCo] first paired RGB/RGB-D frame accepted; starting actor.", flush=True)
        for step in range(steps):
            if stopped[0]:
                stop_reason = "interrupted"
                break
            if viewer is not None and not viewer.is_running():
                stop_reason = "viewer_closed"
                break
            if vision_process.poll() is not None:
                stop_reason = "vision_bridge_exit"
                runtime_error = "vision bridge exited; see " + str(output / "vision_bridge.log")
                break
            tick = time.monotonic()
            if step % 2 == 0:
                capture()
            packet, rejection = load_visual_packet(output / "latest_dual_trajectory.json", publisher)
            visual_command = controller.update(packet, now=time.monotonic(), dt=0.02)
            command = np.asarray(visual_command["base_command_b"], dtype=np.float64)
            target_pose_b = np.asarray(visual_command["tool_pose_b"], dtype=np.float64)
            status = visual_command["status"]
            measured = adapter.read_proprioception(data)
            observation = build_actor_observation(command, target_pose_b, measured["gyro_b"],
                                                  measured["gravity_b"], measured["q"], measured["qd"],
                                                  adapter.default_q18, last_action)
            with torch.inference_mode():
                action = actor(torch.from_numpy(observation).unsqueeze(0)).numpy()[0]
            if action.shape != (18,) or not np.isfinite(action).all():
                raise ValueError("VD5 actor emitted invalid 18-joint action")
            joint_target = adapter.default_q18 + 0.25*action
            actuator_diagnostics = None
            for _ in range(4):
                mujoco.mj_forward(model, data)
                actuator_diagnostics = adapter.apply_targets(data, joint_target)
                substeps = int(round(0.005 / args.physics_dt))
                for _ in range(substeps):
                    if velocity_constraint is None:
                        mujoco.mj_step(model, data)
                    else:
                        velocity_constraint.step(data)
            mujoco.mj_forward(model, data)
            truth_row = evaluation.record(data, step)
            truth.write(json.dumps(truth_row, allow_nan=False) + "\n")
            trace.write(json.dumps({
                "step": step, "sim_time_s": float(data.time),
                "base_command_b": command.tolist(),
                "visual_command_diagnostics": status,
                "startup_blend": status.get("startup_blend", 0.0),
                "tcp_target_b": target_pose_b.tolist(),
                "observation": observation.tolist(), "action": action.tolist(),
                "joint_target": joint_target.tolist(), "joint_position": measured["q"].tolist(),
                "joint_velocity": measured["qd"].tolist(),
                "packet_rejection": rejection,
                "packet_sequence": None if packet is None else packet.get("front", {}).get("sequence"),
                "actuator_diagnostics": actuator_diagnostics,
            }, allow_nan=False, default=json_default) + "\n")
            last_action = action.copy()
            active_commands += int(status.get('valid', False))
            valid_commands += int(status.get('measurement_valid', False))
            held_commands += int(status.get('control_state') == 'rear_hold')
            memory_commands += int(status.get('control_state') == 'rear_memory')
            if step == 0 or (step + 1) % 250 == 0:
                trace.flush()
                truth.flush()
                print("[VD5 MuJoCo] step=%d sim=%.2fs cmd_vx=%.3f cmd_wz=%.3f actual_vx=%.3f tilt=%.1f deg tcp=%.3f m" % (
                    step+1, data.time, command[0], command[2], truth_row["base_velocity_b"][0],
                    truth_row["tilt_deg"], truth_row["tcp_projection_error_m"]), flush=True)
            reason = status.get("reason")
            if step == 0 or (step + 1) % 100 == 0 or reason != latest_reason:
                print("[VD5 vision] step=%d measurement_valid=%s control=%s packet_age=%s memory_source_age=%s front_yaw=%s rear_target=%s reason=%s" % (
                    step + 1, status.get("measurement_valid"), status.get('control_state'), status.get("capture_age_s"),
                    status.get('rear_memory_source_age_max_s'),
                    status.get("yaw_rate_rps"), status.get("rear_target_b"), reason), flush=True)
                latest_reason = reason
            if truth_row["tilt_deg"] > 55.0 or truth_row["base_position_w"][2] < 0.20:
                stop_reason = "fall_guard"
                break
            if viewer is not None:
                with viewer.lock():
                    viewer.cam.lookat[:] = data.xpos[base_id]
                viewer.sync()
            time.sleep(max(0.0, 0.02 - (time.monotonic() - tick)))
    except Exception as error:
        stop_reason = "runtime_error"
        runtime_error = repr(error)
        raise
    finally:
        trace.close()
        truth.close()
        if vision_process.poll() is None:
            vision_process.terminate()
            try:
                vision_process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                vision_process.kill()
                vision_process.wait()
        vision_log.close()
        if viewer is not None:
            viewer.close()
        renderer.close()
        summary = {
            "stop_reason": stop_reason, "runtime_error": runtime_error,
            "actor_steps": len(evaluation.rows), "sim_time_s": float(data.time),
            "static_only": False, "policy": str(args.policy.resolve()),
            "policy_sha256": VD5_POLICY_SHA256, "checkpoint_sha256": VD5_CHECKPOINT_SHA256,
            "speed_command_mps": args.speed, "curvature_per_m": args.curvature,
            "startup_ramp_s": 2.0, "tcp_trailing_distance_m": args.tcp_trailing_distance,
            "rear_occlusion_extension_limit_m": args.rear_occlusion_extension,
            "rear_centerline_mode": args.rear_centerline_mode,
            "rear_hold_s": args.rear_hold_s,
            "rear_path_memory_s": args.rear_path_memory_s,
            "valid_visual_actor_steps": valid_commands,
            "active_control_actor_steps": active_commands,
            "rear_hold_actor_steps": held_commands,
            "rear_memory_actor_steps": memory_commands,
            "visual_mode": args.vision_mode,
            "visual_bridge_log": str(output / "vision_bridge.log"),
            "camera_frames_published": camera_frame[0],
            "evaluation": evaluation.summary(),
            "note": "Commands use rendered RGB/RGB-D perception; MuJoCo lacks real lens/depth sensor noise. Hardware validation is separate.",
        }
        save_json(output / "summary.json", summary)
    print(json.dumps(summary, indent=2), flush=True)
    print(f"[VD5 MuJoCo] logs retained: {output}", flush=True)
    return 1 if stop_reason in ("runtime_error", "fall_guard") else 0


if __name__ == "__main__":
    raise SystemExit(main())
