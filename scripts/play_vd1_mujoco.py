#!/usr/bin/env python3
"""Launch VD1's image-only external perception and MuJoCo actor deployment."""

from __future__ import annotations

import argparse
from datetime import datetime, timedelta, timezone
import gzip
import hashlib
import json
import os
from pathlib import Path
import select
import signal
import subprocess
import tempfile
import threading
import time
import xml.etree.ElementTree as ET

from vd3_mujoco_policy import VD3_POLICY, VD3_POLICY_SHA256


CURVATURE_STAGES = (0.0, 0.05, 0.10, 0.15, 0.40, 0.30, 0.50)
UPSTREAM_COMMIT = "b64b0faf8b4db3a9c1c2e05235521f61ab4dc47d"
POLICY_SHA256 = "86be6c2d1c7a0e56fe4bd93c2d62622469c030b0cdc6d10db7af001eae3f889a"


def sha256_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for block in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def source_manifest(deploy_repo, training_repo, robot_version='vd1'):
    urdf = (Path('/home/dong/b2_z1_VD2/source/b2_z1/b2_z1/assets/b2_z1/b2_z1_vd3_zero_gun.urdf')
            if robot_version == 'vd3' else training_repo / "source/b2_z1/b2_z1/assets/b2_z1/b2_z1_vd1.urdf")
    paths = [
        deploy_repo / "scripts/play_vd1_mujoco.py",
        deploy_repo / "scripts/run_vd1_mujoco.py",
        deploy_repo / "scripts/vd3_mujoco_policy.py",
        deploy_repo / "scripts/vd1_visual_guard.py",
        deploy_repo / "scripts/vd1_visual_path.py",
        deploy_repo / "scripts/vd1_visual_front.py",
        deploy_repo / "scripts/vd1_front_path_estimator.py",
        deploy_repo / "scripts/vd1_visual_rear.py",
        deploy_repo / "scripts/run_vd1_dual_camera_bridge.py",
        deploy_repo / "scripts/vd1_mujoco_road_contact.py",
        deploy_repo / "scripts/vd1_mujoco_scene.py",
        deploy_repo / "scripts/vd1_mujoco_contract.py",
        deploy_repo / "scripts/vd1_mujoco_velocity_constraint.py",
        deploy_repo / "scripts/vd1_mujoco_velocity_solver.py",
        deploy_repo / "scripts/vd1_mujoco_velocity_impulse.py",
        training_repo / "scripts/vision/vd1_control.py",
        training_repo / "scripts/vision/vd1_frames.py",
        training_repo / "scripts/vision/vd1_scene.py",
        training_repo / "scripts/vision/run_dual_camera_bridge.py",
        training_repo / "scripts/vision/dual_camera_perception.py",
        training_repo / "source/b2_z1/b2_z1/assets/b2_z1_vd1.py",
        urdf,
        training_repo / "source/b2_z1/b2_z1/tasks/manager_based/b2_z1/mdp/commands/dual_vision_kinematics.py",
    ]
    if robot_version == 'vd3':
        paths.append(VD3_POLICY.parent / 'provenance.json')
    if urdf.is_file():
        paths.extend(sorted({(urdf.parent / item.get("filename")).resolve()
                             for item in ET.parse(str(urdf)).findall(".//mesh")}))
    return {str(path): ({"sha256": sha256_file(path), "size_bytes": path.stat().st_size}
                        if path.is_file() else {"missing": True}) for path in paths}


def write_json(path, payload):
    temporary = path.with_name("." + path.name + ".tmp")
    temporary.write_text(json.dumps(payload, indent=2, allow_nan=False), encoding="utf-8")
    temporary.replace(path)


class CompressedBridgeHistory:
    """Let the unchanged bridge append to a pipe, retaining only lossless gzip.

    The bridge opens its standard history path. A private reader compresses
    every byte without editing, rotating or truncating the bridge's records.
    """

    def __init__(self, exchange):
        self.path = exchange / "dual_vision_packets.jsonl"
        self.output = exchange / "dual_vision_packets.jsonl.gz"
        self.error = None
        self.bytes_read = 0
        os.mkfifo(self.path, 0o600)
        self.reader = os.open(str(self.path), os.O_RDONLY | os.O_NONBLOCK)
        # Keep the reader alive during model warm-up, before the bridge opens
        # its writer. This descriptor is closed only after the bridge stops.
        self.keepalive = os.open(str(self.path), os.O_WRONLY | os.O_NONBLOCK)
        self.thread = threading.Thread(target=self._copy, name="vd1-history-gzip", daemon=True)
        self.thread.start()

    def _copy(self):
        try:
            with gzip.open(str(self.output), "wb", compresslevel=3) as destination:
                while True:
                    ready, _, _ = select.select([self.reader], [], [], 0.25)
                    if not ready:
                        continue
                    block = os.read(self.reader, 65536)
                    if not block:
                        break
                    destination.write(block)
                    destination.flush()
                    self.bytes_read += len(block)
        except Exception as error:
            self.error = str(error)

    def close(self):
        os.close(self.keepalive)
        self.thread.join(timeout=10)
        if self.thread.is_alive():
            self.error = self.error or "history compression did not finish after bridge shutdown"
        else:
            os.close(self.reader)
            self.path.unlink()


def stop_child(process):
    """Stop only a child process group created by this launcher."""
    if process is None:
        return
    for signum, timeout in ((signal.SIGINT, 8), (signal.SIGTERM, 4), (signal.SIGKILL, 2)):
        try:
            os.killpg(process.pid, signum)
        except ProcessLookupError:
            process.poll()
            return
        deadline = time.monotonic() + timeout
        # The group leader may already have exited while one of its children
        # remains alive. Reaping the leader alone is not enough to clean up.
        while time.monotonic() < deadline:
            process.poll()
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.1)


def main(argv=None):
    deploy_repo = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-repo", type=Path, default=Path("/home/dong/b2_z1"))
    parser.add_argument('--robot-version', choices=('vd1', 'vd3'), default='vd1')
    parser.add_argument("--policy", type=Path)
    parser.add_argument("--mujoco-python", type=Path,
                        default=Path("/home/dong/anaconda3/envs/crack_vision/bin/python"))
    parser.add_argument("--vision-python", type=Path,
                        default=Path("/home/dong/anaconda3/envs/crack_vision/bin/python"))
    parser.add_argument("--vision-device", default="cuda:0")
    parser.add_argument("--visual-controller", choices=("original", "guarded"), default="original")
    parser.add_argument("--front-only-guidance", action="store_true",
                        help="Ablation: keep the previous far visible front reference in guarded mode")
    parser.add_argument('--front-path-estimation', action='store_true',
                        help='Front-only curve filtering/extrapolation, with no robot motion estimator')
    parser.add_argument('--front-nominal-height-m', type=float, default=.55,
                        help='Constant base height used for front flat-floor projection; never simulator pose')
    parser.add_argument("--road-contact", choices=("prisms", "heightfield"), default="prisms",
                        help="Road collision model; rendered crack and robot parameters are preserved")
    parser.add_argument("--road-surface", choices=("crack", "tape"), default="crack",
                        help="Physical trench or zero-thickness black line painted on flat ground")
    parser.add_argument('--gun-mass-mode', choices=('original', 'zero'), default='original',
                        help='Debug mass experiment: zero removes gun mass/inertia, preserving geometry')
    parser.add_argument("--velocity-constraint", choices=("off", "implicit_brake", "native_velocity", "impulse"), default="off",
                        help="Experimental velocity-constraint surrogate; off preserves the native model")
    parser.add_argument("--physics-dt", type=float, choices=(0.001, 0.005), default=0.001,
                        help="Integration period; actor remains 50 Hz and leg PD remains 200 Hz")
    parser.add_argument("--physx-implicit-position", action="store_true",
                        help="Experimental PhysX joint-drive integration conversion; source gains stay 160/5")
    parser.add_argument("--curvature-stage", type=int, choices=range(len(CURVATURE_STAGES)), default=1,
                        help="Scene curve curvature 0/0.05/0.10/0.15/0.40/0.30/0.50 per metre")
    parser.add_argument("--speed", type=float, default=0.40,
                        help="Forward velocity command in m/s (0 < speed <= 1; default 0.40)")
    parser.add_argument("--max-steps", type=int, default=0,
                        help="Actor steps; 0 runs until the viewer closes or Ctrl-C")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-display", action="store_true", help="Hide perception windows")
    parser.add_argument("--static-only", action="store_true",
                        help="Physics-paused camera/model diagnostic; no integration or actor motion")
    parser.add_argument("--classical", action="store_true",
                        help="Explicit classical image diagnostic, not a learned-detector result")
    parser.add_argument("--output-root", type=Path, default=deploy_repo / "outputs/VD1_MUJOCO")
    parser.add_argument("--cache-dir", type=Path, default=Path("/dev/shm/vd1_mujoco"))
    parser.add_argument("--screenshot", type=Path)
    parser.add_argument("--dry-run", action="store_true",
                        help="Write metadata and print commands without starting processes")
    args = parser.parse_args(argv)
    if args.max_steps < 0:
        parser.error("--max-steps must be nonnegative")
    if not 0 < args.speed <= 1:
        parser.error("--speed must be finite and within (0, 1] m/s")
    if args.front_path_estimation and (args.visual_controller != 'guarded' or args.front_only_guidance):
        parser.error('front path estimation requires guarded mode and excludes the old front-only ablation')
    if not .3 <= args.front_nominal_height_m <= 1.:
        parser.error('front nominal height must be in [0.3, 1.0] metres')
    args.training_repo = args.training_repo.expanduser().resolve()
    default_policy = VD3_POLICY if args.robot_version == 'vd3' else args.training_repo / ".pretrained_checkpoints/VD1/exported/policy.pt"
    args.policy = (args.policy or default_policy).expanduser().resolve()
    for name in ("policy", "mujoco_python", "vision_python"):
        path = getattr(args, name).expanduser().resolve()
        if not path.is_file():
            parser.error("Missing {}: {}".format(name, path))
        setattr(args, name, path)
    runner = deploy_repo / "scripts/run_vd1_mujoco.py"
    bridge = (deploy_repo / "scripts/run_vd1_dual_camera_bridge.py" if args.visual_controller == 'guarded'
              else args.training_repo / "scripts/vision/run_dual_camera_bridge.py")
    for path in (bridge, runner):
        if not path.is_file() and not args.dry_run:
            parser.error("Missing entry point: {}".format(path))
    expected_policy_sha = VD3_POLICY_SHA256 if args.robot_version == 'vd3' else POLICY_SHA256
    if sha256_file(args.policy) != expected_policy_sha:
        parser.error('The selected actor SHA256 differs from the verified ' + args.robot_version.upper() + ' export')
    output_root = args.output_root.expanduser().resolve()
    output_root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime("%Y%m%d_%H%M%S")
    exchange = Path(tempfile.mkdtemp(prefix="session_{}_".format(stamp), dir=str(output_root)))
    screenshot = args.screenshot.expanduser().resolve() if args.screenshot else exchange / "viewport_robot.png"
    mode = "classical" if args.classical else "rhanet"
    vision_cmd = [str(args.vision_python), str(bridge), "--exchange-dir", str(exchange),
                  "--device", args.vision_device, "--mode", mode, "--save-overlay"]
    if args.front_path_estimation:
        vision_cmd.extend(['--front-nominal-height-m', str(args.front_nominal_height_m)])
    if not args.headless and not args.no_display:
        vision_cmd.append("--display")
    play_cmd = [str(args.mujoco_python), str(runner), "--policy", str(args.policy),
                '--robot-version', args.robot_version,
                "--output-dir", str(exchange), "--vision-result", str(exchange / "latest_dual_trajectory.json"),
                "--curvature-max", str(CURVATURE_STAGES[args.curvature_stage]),
                "--speed", str(args.speed), "--max-steps", str(args.max_steps),
                "--visual-controller", args.visual_controller,
                "--velocity-constraint", args.velocity_constraint,
                "--physics-dt", str(args.physics_dt),
                "--cache-dir", str(args.cache_dir.expanduser().resolve()), "--screenshot", str(screenshot)]
    if args.headless:
        play_cmd.append("--headless")
    if args.static_only:
        play_cmd.append("--static-only")
    if args.physx_implicit_position:
        play_cmd.append("--physx-implicit-position")
    if args.front_only_guidance:
        play_cmd.append("--front-only-guidance")
    if args.front_path_estimation:
        play_cmd.extend(['--front-path-estimation', '--front-nominal-height-m', str(args.front_nominal_height_m)])
    play_cmd.extend(["--road-contact", args.road_contact, "--road-surface", args.road_surface,
                     '--gun-mass-mode', args.gun_mass_mode])
    env = os.environ.copy()
    for inherited in ("B2_Z1_VISION_TRAJECTORY", "B2_Z1_DUAL_VISION_TRAJECTORY"):
        env.pop(inherited, None)
    env["PYTHONUNBUFFERED"] = "1"
    cache_root = args.cache_dir.expanduser().resolve()
    ram_cache_paths = {
        "CUDA_CACHE_PATH": str(cache_root / "cuda_cache"),
        "__GL_SHADER_DISK_CACHE_PATH": str(cache_root / "gl_cache"),
        "MESA_SHADER_CACHE_DIR": str(cache_root / "mesa_cache"),
    }
    env.update(ram_cache_paths)
    if args.headless:
        env.setdefault("MUJOCO_GL", "egl")
    starting_sources = source_manifest(deploy_repo, args.training_repo, args.robot_version)
    session = {
        "version": args.robot_version.upper() + "_MUJOCO", "simulator": "MuJoCo",
        "upstream_repository": "https://github.com/LiqinLu/b2_z1.git",
        "upstream_branch": "dual_cmd", "upstream_commit": UPSTREAM_COMMIT,
        "policy": str(args.policy), "policy_sha256": expected_policy_sha,
        "actor_observations": 70, "actor_actions": 18, "critic_used": False,
        "external_sensors": ["front_rgb", "rear_rgbd"],
        "proprioception": ["imu", "joint_encoders", "previous_actor_action"],
        "perception_mode": mode, "classical_diagnostic_only": args.classical,
        "curvature_stage": args.curvature_stage,
        "maximum_curvature_per_m": CURVATURE_STAGES[args.curvature_stage],
        "forward_speed_mps": args.speed, "static_only": args.static_only,
        "visual_controller": args.visual_controller,
        "tool_target_mode": ('rear_seam_direct' if args.visual_controller == 'guarded'
                             else 'visual_tcp_feedback'),
        "visual_tcp_feedback": args.visual_controller != 'guarded',
        "requires_tcp_markers": args.visual_controller != 'guarded',
        "tool_markers_in_scene": args.visual_controller != 'guarded',
        "front_component_selection": ('longest_measured_forward_support'
                                      if args.visual_controller == 'guarded' else 'fixed_image_anchor'),
        "front_only_guidance": args.front_only_guidance,
        "front_path_estimation": args.front_path_estimation, "body_motion_estimation": False,
        "front_nominal_height_m": args.front_nominal_height_m if args.front_path_estimation else None,
        "base_guidance_source": 'front_rgb_curve_estimator' if args.front_path_estimation else 'legacy_visual_controller',
        "road_surface": args.road_surface,
        "gun_mass_mode": args.gun_mass_mode,
        "gun_mass_experiment": args.gun_mass_mode != 'original',
        "road_contact": 'flat_plane' if args.road_surface == 'tape' else args.road_contact,
        "velocity_constraint": args.velocity_constraint,
        "velocity_constraint_experimental": args.velocity_constraint != "off",
        "physics_timestep_s": args.physics_dt,
        "leg_pd_period_s": 0.005, "actor_period_s": 0.02,
        "physics_substeps_per_pd": int(round(0.005 / args.physics_dt)),
        "physx_implicit_position": args.physx_implicit_position,
        "created_timestamp": time.time(), "timezone": "Asia/Shanghai",
        "source_sha256_at_launch": starting_sources,
        "play": play_cmd, "vision": vision_cmd, "hardware_started": False,
        "bridge_history": "dual_vision_packets.jsonl.gz",
        "runtime_cache_environment": ram_cache_paths,
        "state": "dry_run" if args.dry_run else "starting",
    }

    def save_session():
        write_json(exchange / "session.json", session)

    save_session()
    print("[VD1 MuJoCo] session: {}".format(exchange), flush=True)
    print("[VD1 MuJoCo] policy: {} (SHA256 {})".format(args.policy, expected_policy_sha), flush=True)
    print("[VD1 MuJoCo] {} vision; speed={:.2f} m/s; curvature stage={} ({:.2f}/m)".format(
        mode, args.speed, args.curvature_stage, CURVATURE_STAGES[args.curvature_stage]), flush=True)
    if args.dry_run:
        print(json.dumps({"play": play_cmd, "vision": vision_cmd}, indent=2), flush=True)
        return 0
    play = vision = history = None
    result = 0

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    try:
        for directory in ram_cache_paths.values():
            Path(directory).mkdir(parents=True, exist_ok=True)
        # Verify actual imports, not only importlib.find_spec: an environment
        # can contain torch's files while its shared libraries fail to load.
        probe = subprocess.run(
            [str(args.mujoco_python), "-c",
             "import json,sys,mujoco,torch,numpy,cv2; "
             "actor=torch.jit.load(sys.argv[1],map_location='cpu').eval(); "
             "action=actor(torch.zeros((1,70))); "
             "assert tuple(action.shape)==(1,18) and bool(torch.isfinite(action).all()), 'Invalid VD1 actor output'; "
             "print(json.dumps({'mujoco':mujoco.__version__,'torch':torch.__version__,'numpy':numpy.__version__,'cv2':cv2.__version__,'actor_input':70,'actor_output':18}))",
             str(args.policy)],
            env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=30)
        if probe.returncode:
            raise RuntimeError("MuJoCo Python environment cannot import its dependencies: " + probe.stdout[-3000:])
        session["runtime_versions"] = json.loads(probe.stdout.strip().splitlines()[-1])
        save_session()
        history = CompressedBridgeHistory(exchange)
        with (exchange / "vision.log").open("w") as vision_log, (exchange / "play.log").open("w") as play_log:
            vision = subprocess.Popen(vision_cmd, cwd=str(args.training_repo), env=env,
                                      stdout=vision_log, stderr=subprocess.STDOUT, start_new_session=True)
            session["vision_pid"] = vision.pid
            save_session()
            startup = time.monotonic()
            while True:
                if vision.poll() is not None:
                    raise RuntimeError("Vision warm-up exited ({}); see {}".format(vision.returncode, exchange / "vision.log"))
                if history.error:
                    raise RuntimeError("History compression failed: " + history.error)
                try:
                    ready = json.loads((exchange / "vision_ready.json").read_text())
                    if ready.get("ready") is True and ready.get("mode") == mode:
                        break
                except (OSError, ValueError, AttributeError):
                    pass
                if time.monotonic() - startup > 120:
                    raise RuntimeError("Vision warm-up timed out; see {}".format(exchange / "vision.log"))
                time.sleep(0.1)
            play = subprocess.Popen(play_cmd, cwd=str(deploy_repo), env=env, stdout=play_log,
                                    stderr=subprocess.STDOUT, start_new_session=True)
            session.update(play_pid=play.pid, state="running")
            save_session()
            last_status = None
            while play.poll() is None:
                if vision.poll() is not None:
                    raise RuntimeError("Vision exited ({}); see {}".format(vision.returncode, exchange / "vision.log"))
                if history.error:
                    raise RuntimeError("History compression failed: " + history.error)
                try:
                    status = json.loads((exchange / "latest_vision_status.json").read_text())
                    status_key = (status.get("valid"), status.get("reason"))
                    if status_key != last_status:
                        print("[VD1 vision] {}: {}".format("VALID" if status_key[0] else "WAIT", status_key[1]), flush=True)
                        last_status = status_key
                except (OSError, ValueError, AttributeError):
                    pass
                time.sleep(0.25)
            if play.returncode:
                raise RuntimeError("Simulation exited ({}); see {}".format(play.returncode, exchange / "play.log"))
            try:
                summary = json.loads((exchange / "summary.json").read_text())
                steps, reason = summary.get("actor_steps"), summary.get("stop_reason")
                if (not isinstance(steps, int) or isinstance(steps, bool) or steps < 0
                        or not isinstance(reason, str) or not reason.strip()):
                    raise ValueError("summary requires actor_steps and stop_reason")
            except (OSError, ValueError, AttributeError) as error:
                raise RuntimeError("Simulation exited without a valid summary.json: {}; see {}".format(error, exchange / "play.log"))
            session["simulation_summary"] = summary
            if reason in ("runtime_error", "nonfinite_actor_action", "IMU_tilt_stop"):
                raise RuntimeError("Simulation ended with {}; inspect summary.json".format(reason))
            session["state"] = "finished"
    except KeyboardInterrupt:
        print("[VD1 MuJoCo] Stopping this session.", flush=True)
        session["state"] = "interrupted"
        result = 130
    except Exception as error:
        session.update(state="failed", error=str(error))
        print("[VD1 MuJoCo] {}".format(error), flush=True)
        result = 1
    finally:
        stop_child(play)
        stop_child(vision)
        if history is not None:
            history.close()
            session.update(bridge_history_uncompressed_bytes=history.bytes_read,
                           bridge_history_compression_error=history.error)
            if history.error:
                session.update(state="failed", error=history.error)
                result = 1
        ending_sources = source_manifest(deploy_repo, args.training_repo, args.robot_version)
        session.update(finished_timestamp=time.time(), source_sha256_at_finish=ending_sources,
                       source_files_changed_during_session=sorted(
                           name for name in set(starting_sources) | set(ending_sources)
                           if starting_sources.get(name) != ending_sources.get(name)))
        save_session()
        print("[VD1 MuJoCo] Logs and camera images retained: {}".format(exchange), flush=True)
    return result


if __name__ == "__main__":
    raise SystemExit(main())
