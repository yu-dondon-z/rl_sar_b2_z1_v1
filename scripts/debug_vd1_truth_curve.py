#!/usr/bin/env python3
"""SIMULATION ONLY: privileged world-point curve guidance for the VD1 actor.

This deliberately bypasses the camera pipeline. World path points and the true
base pose generate the existing 3D velocity and 7D tool commands; the actor is
unchanged (70 observations -> 18 actions). Never use this as a deployment loop.
"""
from __future__ import annotations

import argparse
import csv
from datetime import datetime, timedelta, timezone
import gzip
import html
import json
import math
from pathlib import Path
import signal
import sys
import tempfile
import time

import numpy as np

from diagnose_vd1_base_response import (
    ACTOR_DT, PD_DT, DEFAULT_POLICY, POLICY_SHA256, TRAINING_ROOT,
    TruthRecorder, dynamics_hash, load_fixed_tool_sample, make_flat_ground,
    sha256, smoothstep, write_json,
)
from vd1_base_response_viewer import DiagnosticViewer

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SAMPLE = ROOT / 'scripts/fixtures/vd1_fixed_camera_tool_command.json'
PRIVILEGED_INPUTS = ['world-frame curve points', 'true base position', 'true base orientation']


class TruthPolyline:
    """Dense points connected by exact line segments, including straight ends."""

    def __init__(self, points):
        self.points = np.asarray(points, dtype=float).copy()
        if (self.points.ndim != 2 or self.points.shape[1] != 3 or len(self.points) < 3
                or not np.isfinite(self.points).all()):
            raise ValueError('Curve must contain at least three finite XYZ points')
        self.delta = np.diff(self.points[:, :2], axis=0)
        self.length = np.linalg.norm(self.delta, axis=1)
        if np.any(self.length <= 1e-8):
            raise ValueError('Curve has coincident adjacent XY points')
        self.arc = np.concatenate(([0.], np.cumsum(self.length)))

    def project(self, xy):
        xy = np.asarray(xy, dtype=float)
        if xy.shape != (2,) or not np.isfinite(xy).all():
            raise ValueError('Projection requires finite XY')
        fraction = np.clip(np.sum((xy-self.points[:-1, :2])*self.delta, axis=1)
                           / self.length**2, 0., 1.)
        nearest = self.points[:-1, :2] + fraction[:, None]*self.delta
        index = int(np.argmin(np.sum((xy-nearest)**2, axis=1)))
        point = self.points[index] + fraction[index]*(self.points[index+1]-self.points[index])
        tangent = self.delta[index]/self.length[index]
        offset = xy-point[:2]
        return dict(point=point, arc=float(self.arc[index]+fraction[index]*self.length[index]),
                    distance=float(np.linalg.norm(offset)),
                    signed_distance=float(tangent[0]*offset[1]-tangent[1]*offset[0]))

    def at(self, arc):
        arc = float(np.clip(arc, 0., self.arc[-1]))
        index = min(int(np.searchsorted(self.arc, arc, side='right')-1), len(self.length)-1)
        blend = (arc-self.arc[index])/self.length[index]
        return self.points[index] + blend*(self.points[index+1]-self.points[index])


def make_curve(curvature=.10, extent=50., step=.05):
    """Smooth sin^3 profile with configurable curvature; points at ground z=0."""
    if not np.isfinite([curvature, extent, step]).all() or not 0 <= curvature <= .50 or extent < 25 or step <= 0:
        raise ValueError('Invalid truth curve parameters')
    x = np.linspace(-5., extent, int(math.ceil((extent+5.)/step))+1)
    wave = 2*math.pi/20.
    amplitude = curvature/(3*wave**2)
    y = np.where((x > 2.) & (x < 22.), amplitude*np.sin(wave*(x-2.))**3, 0.)
    return TruthPolyline(np.column_stack((x, y, np.zeros_like(x))))


def rear_curve_target(path, base_position, rotation):
    """Intersect the true seam with actor rear working x=-1.02 in base axes.

    Preserve the trained nominal base-frame working z=-0.30, fixed orientation
    and workspace limits. No IK or true-TCP feedback changes the actor actions.
    """
    local = (path.points-np.asarray(base_position)) @ np.asarray(rotation)
    x = -1.02
    indices = np.flatnonzero((local[:-1, 0]-x)*(local[1:, 0]-x) <= 0)
    candidates = []
    for index in indices:
        dx = local[index+1, 0]-local[index, 0]
        if abs(dx) > 1e-10:
            fraction = (x-local[index, 0])/dx
            point = local[index] + fraction*(local[index+1]-local[index])
            candidates.append(point)
    if candidates:
        point = min(candidates, key=lambda p: abs(p[1]))
        supported = True
    else:
        point = local[int(np.argmin(np.abs(local[:, 0]-x)))]
        supported = False
    raw = np.array([x, point[1], -.30])
    target = np.clip(raw, [-1.08, -.15, -.34], [-.96, .15, -.26])
    return target, supported, bool(np.any(raw != target))


class PrivilegedCurveCommands:
    """Truth localization -> pure pursuit -> ordinary actor command slots."""

    def __init__(self, path, initial_tool, speed=.4, lookahead=.6, tool_mode='curve'):
        self.path = path
        self.tool = np.asarray(initial_tool, dtype=float).copy()
        if self.tool.shape != (7,) or not np.isfinite(self.tool).all() or abs(np.linalg.norm(self.tool[3:])-1) > 1e-3:
            raise ValueError('Initial tool must be finite XYZ and unit wxyz')
        if not np.isfinite([speed, lookahead]).all() or not 0 < speed <= 1 or not .1 <= lookahead <= 5:
            raise ValueError('Invalid command speed/lookahead')
        if tool_mode not in ('curve', 'fixed'):
            raise ValueError('Unknown tool mode')
        self.speed, self.lookahead, self.tool_mode = speed, lookahead, tool_mode

    def update(self, elapsed, base_position, rotation, dt=ACTOR_DT):
        position, rotation = np.asarray(base_position, dtype=float), np.asarray(rotation, dtype=float)
        if (position.shape != (3,) or rotation.shape != (3, 3)
                or not np.isfinite(position).all() or not np.isfinite(rotation).all()
                or not np.isfinite([elapsed, dt]).all() or elapsed < 0 or dt <= 0):
            raise ValueError('Invalid truth command state')
        projection = self.path.project(position[:2])
        lookahead = self.path.at(projection['arc']+self.lookahead)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
        c, s = math.cos(yaw), math.sin(yaw)
        delta = lookahead[:2]-position[:2]
        forward, lateral = c*delta[0]+s*delta[1], -s*delta[0]+c*delta[1]
        speed = self.speed*smoothstep(elapsed/2.)
        yaw_command = float(np.clip(2*speed*lateral/max(forward**2+lateral**2, .01), -.8, .8))
        target, supported, clipped = rear_curve_target(self.path, position, rotation)
        if self.tool_mode == 'curve':
            change = (1-math.exp(-dt/.30))*(target-self.tool[:3])
            length = float(np.linalg.norm(change))
            if length > .15*dt:
                change *= .15*dt/length
            self.tool[:3] += change
        status = dict(privileged=True, steering='true-pose pure pursuit',
                      base_cross_track_m=projection['distance'],
                      base_signed_cross_track_m=projection['signed_distance'],
                      path_arc_m=projection['arc'], lookahead_w=lookahead.tolist(),
                      closest_base_curve_point_w=projection['point'].tolist(),
                      rear_supported=supported, rear_target_clipped=clipped,
                      tool_mode=self.tool_mode, rear_workspace_target_b=target.tolist(),
                      tool_target_w=(position+rotation @ self.tool[:3]).tolist(),
                      lookahead_is_forward=bool(forward > 0))
        return np.array([speed, 0., yaw_command]), self.tool.copy(), status


def save_trajectory_svg(path, curve, rows, title='VD1 PRIVILEGED TRUTH CURVE'):
    """A dependency-free top view that VS Code can open as an image."""
    base = np.asarray([r['base_position_w'][:2] for r in rows], dtype=float).reshape(-1, 2)
    tcp = np.asarray([r['tcp_position_w'][:2] for r in rows], dtype=float).reshape(-1, 2)
    max_x = max(25., float(base[:, 0].max())+2 if len(base) else 25.)
    reference = curve.points[(curve.points[:, 0] >= -2) & (curve.points[:, 0] <= max_x), :2]
    all_points = np.vstack((reference, base, tcp))
    low, high = all_points.min(axis=0)-[.5, .05], all_points.max(axis=0)+[.5, .05]
    # This is an XY chart, with explicitly independent axis scales so a gentle
    # 17 cm curve remains visible across thirty metres of forward travel.
    scale = np.array([1000/(high[0]-low[0]), 390/(high[1]-low[1])])
    center = (low+high)/2
    screen_scale = scale*np.array([1., -1.])
    def coords(points):
        screen = (points-center)*screen_scale+[550, 330]
        return ' '.join('%.2f,%.2f' % tuple(p) for p in screen)
    elapsed = rows[-1]['sim_time_s'] if rows else 0.
    error = rows[-1].get('tcp_curve_error_m', 0.) if rows else 0.
    body = [f'<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="620" viewBox="0 0 1100 620">',
            '<rect width="1100" height="620" fill="#17212d"/>',
            f'<text x="35" y="42" fill="white" font-size="23">{html.escape(title)}</text>',
            '<text x="35" y="75" fill="#fbbf24" font-size="16">SIMULATION ONLY: true world path and base pose drive commands</text>',
            f'<text x="35" y="103" fill="white" font-size="16">t={elapsed:.2f}s | TCP ground-projection error={error:.3f} m | XY chart: independent axis scales</text>',
            '<text x="35" y="125" fill="#b9c5d2" font-size="12">Y [m]</text>',
            '<text x="530" y="568" fill="#b9c5d2" font-size="13">X [m]</text>',
            '<text x="35" y="603" fill="#56d9ff" font-size="16">CYAN: truth curve / points</text>',
            '<text x="370" y="603" fill="#ffad55" font-size="16">ORANGE: actual base</text>',
            '<text x="720" y="603" fill="#ff70c5" font-size="16">PINK: actual TCP projection</text>']
    for x in np.linspace(low[0], high[0], 7):
        pixel = (x-center[0])*scale[0]+550
        body.append(f'<line x1="{pixel:.2f}" y1="135" x2="{pixel:.2f}" y2="525" stroke="#354352"/>')
        body.append(f'<text x="{pixel:.2f}" y="545" fill="#b9c5d2" font-size="12" text-anchor="middle">{x:.1f}</text>')
    for y in np.linspace(low[1], high[1], 5):
        pixel = -(y-center[1])*scale[1]+330
        body.append(f'<line x1="50" y1="{pixel:.2f}" x2="1050" y2="{pixel:.2f}" stroke="#354352"/>')
        body.append(f'<text x="45" y="{pixel+4:.2f}" fill="#b9c5d2" font-size="11" text-anchor="end">{y:.2f}</text>')
    for points, color in [(reference, '#56d9ff'), (base, '#ffad55'), (tcp, '#ff70c5')]:
        if len(points):
            body.append(f'<polyline points="{coords(points)}" fill="none" stroke="{color}" stroke-width="2.5"/>')
            screen = (points[-1]-center)*screen_scale+[550, 330]
            body.append(f'<circle cx="{screen[0]:.2f}" cy="{screen[1]:.2f}" r="4" fill="{color}"/>')
    for point in reference[::10]:
        x, y = (point-center)*screen_scale+[550, 330]
        body.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="2" fill="#56d9ff"/>')
    body.append('</svg>')
    path = Path(path)
    temporary = path.with_suffix('.svg.tmp')
    temporary.write_text('\n'.join(body)+'\n', encoding='utf-8')
    temporary.replace(path)


class TruthCurveViewer(DiagnosticViewer):
    """Render truth-point reference and actual motion on a separate model."""

    def __init__(self, model, data, curve, **kwargs):
        self.curve = curve
        self.base_trail, self.tcp_trail = [], []
        self.last_time = None
        super().__init__(model, data, **kwargs)
        self.tcp = self.model.site('tool_tcp').id

    def update(self, source, command, evaluation=None):
        import mujoco
        if not self.is_running():
            return
        evaluation = evaluation or {}
        status = evaluation.get('guidance', {})
        velocity = evaluation.get('body_velocity_b', [0., 0., 0.])
        paused = 'PAUSED: SPACE to start/resume' if self.paused.is_set() else 'RUNNING: SPACE to pause'
        labels = [f'VD1 PRIVILEGED TRUTH CURVE | {paused}',
                  't=%.2fs | cmd vx=%.3f w=%.3f | actual vx=%.3f' %
                  (source.time, command[0], command[2], velocity[0]),
                  'base error=%.3fm | TCP error=%.3fm | tilt=%.1fdeg' %
                  (status.get('base_cross_track_m', 0.), evaluation.get('tcp_curve_error_m', 0.),
                   evaluation.get('tilt_deg', 0.)),
                  'CYAN truth | ORANGE base trail | GREEN target | PINK actual TCP']
        with self.handle.lock():
            self._copy_state(source)
            base = self.data.xpos[self.base].copy()
            tcp = self.data.site_xpos[self.tcp].copy()
            if self.last_time != float(source.time):
                self.base_trail.append([base[0], base[1], .018])
                self.tcp_trail.append([tcp[0], tcp[1], .024])
                self.last_time = float(source.time)
            self.handle.cam.lookat[:] = base + (-.3, 0., -.08)
            scene = self.handle.user_scn
            if scene is not None:
                scene.ngeom = 0
                def geom(kind, position, size, color, label=None):
                    if scene.ngeom >= len(scene.geoms):
                        return None
                    g = scene.geoms[scene.ngeom]
                    mujoco.mjv_initGeom(g, kind, np.asarray(size), np.asarray(position),
                                       np.eye(3).reshape(-1), np.asarray(color, dtype=np.float32))
                    if label is not None:
                        g.label = label[:99]
                    scene.ngeom += 1
                    return g
                def line(start, end, color, width=2.):
                    g = geom(mujoco.mjtGeom.mjGEOM_LINE, [0., 0., 0.], [0., 0., 0.], color)
                    if g is not None:
                        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_LINE, width,
                                             np.asarray(start), np.asarray(end))
                def sphere(point, color, radius=.025):
                    geom(mujoco.mjtGeom.mjGEOM_SPHERE, point, [radius]*3, color)
                for index, label in enumerate(labels):
                    geom(mujoco.mjtGeom.mjGEOM_LABEL, base+[-.3, 0., 1.12-index*.14],
                         [0., 0., 0.], [1., 1., 1., 1.], label)
                near = self.curve.points[np.linalg.norm(self.curve.points[:, :2]-base[:2], axis=1) < 8][::2].copy()
                near[:, 2] += .008
                for start, end in zip(near[:-1], near[1:]):
                    line(start, end, [0., .85, 1., 1.], 3.)
                for point in near[::5]:
                    sphere(point, [0., .85, 1., 1.], .013)
                for points, color in [(self.base_trail, [1., .55, .10, 1.]),
                                      (self.tcp_trail, [1., .2, .65, 1.])]:
                    trail = points[-1500::5]
                    for start, end in zip(trail[:-1], trail[1:]):
                        line(start, end, color)
                if 'lookahead_w' in status:
                    lookahead = np.asarray(status['lookahead_w'])+[0., 0., .025]
                    sphere(lookahead, [1., 1., .1, 1.], .045)
                    line([base[0], base[1], .025], lookahead, [1., 1., .1, 1.])
                if 'tool_target_w' in status:
                    target = status['tool_target_w']
                    sphere(target, [.2, 1., .25, 1.], .035)
                    line(tcp, target, [.2, 1., .25, 1.])
                sphere(tcp, [1., .2, .65, 1.], .025)
        self.handle.sync()


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60.)
    parser.add_argument('--speed', type=float, default=.4)
    parser.add_argument('--curvature-max', type=float, default=.10)
    parser.add_argument('--lookahead', type=float, default=.6)
    parser.add_argument('--tool-mode', choices=('curve', 'fixed'), default='curve')
    parser.add_argument('--headless', action='store_true', help='Generate SVG/CSV without opening a window')
    parser.add_argument('--physical-crack', action='store_true',
                        help='Isolation check: keep the physical trench with privileged path commands')
    parser.add_argument('--road-contact', choices=('prisms', 'heightfield'), default='prisms')
    parser.add_argument('--no-start-paused', action='store_true', help='Start the native viewer immediately')
    parser.add_argument('--realtime-factor', type=float, default=1.)
    parser.add_argument('--output-root', type=Path, default=ROOT/'outputs/VD1_TRUTH_CURVE_DEBUG')
    args = parser.parse_args(argv)
    values = [args.duration, args.speed, args.curvature_max, args.lookahead, args.realtime_factor]
    if (not np.isfinite(values).all() or not 0 < args.duration <= 600 or not 0 < args.speed <= 1
            or not 0 <= args.curvature_max <= .40 or not .1 <= args.lookahead <= 5
            or not 0 < args.realtime_factor <= 4):
        parser.error('Use finite duration (0,600], speed (0,1], curvature [0,.40], lookahead [.1,5], realtime (0,4]')
    return args


def main(argv=None):
    args = arguments(argv)
    sys.path.insert(0, str(TRAINING_ROOT/'scripts/vision'))
    import mujoco
    import torch
    from vd1_control import build_actor_observation
    from vd1_mujoco_contract import VD1MujocoAdapter
    from vd1_mujoco_scene import build_scene, validate_model, URDF_PATH
    from vd1_mujoco_velocity_impulse import VelocityImpulseConstraints

    torch.set_num_threads(1)
    if sha256(DEFAULT_POLICY) != POLICY_SHA256:
        raise ValueError('Original VD1 policy hash changed')
    actor = torch.jit.load(str(DEFAULT_POLICY), map_location='cpu').eval()
    with torch.inference_mode():
        if actor(torch.zeros(1, 70)).shape != (1, 18):
            raise ValueError('Expected the VD1 70 -> 18 actor')
    root = args.output_root.expanduser().resolve()
    root.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone(timedelta(hours=8))).strftime('%Y%m%d_%H%M%S')
    output = Path(tempfile.mkdtemp(prefix='truth_curve_'+stamp+'_', dir=str(root)))
    print('[VD1 TRUTH DEBUG] SIMULATION ONLY: privileged path and true base pose', flush=True)
    print('[VD1 TRUTH DEBUG] output: '+str(output), flush=True)
    print('[VD1 TRUTH DEBUG] VS Code image preview: '+str(output/'trajectory.svg'), flush=True)
    curve = make_curve(args.curvature_max, max(50., args.duration+10.))
    np.savetxt(output/'truth_curve.csv', curve.points, delimiter=',', header='x_w_m,y_w_m,z_w_m', comments='')
    scene = build_scene(args.curvature_max, output/'scene_cache', timestep=PD_DT)
    if not args.physical_crack:
        scene = make_flat_ground(scene)
    elif args.road_contact == 'heightfield':
        from vd1_mujoco_road_contact import continuous_road_contact
        scene = continuous_road_contact(scene, args.curvature_max)
    model = mujoco.MjModel.from_xml_path(str(scene))
    validation = validate_model(model)
    adapter = VD1MujocoAdapter(model, physx_implicit_position=True)
    initial_dynamics = dynamics_hash(model)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key('vd1_default').id)
    mujoco.mj_forward(model, data)
    base_id = model.body('base_link').id
    sample = load_fixed_tool_sample(DEFAULT_SAMPLE)
    controller = PrivilegedCurveCommands(curve, sample['tool_pose_b'], args.speed, args.lookahead, args.tool_mode)
    brake = VelocityImpulseConstraints(model, adapter.joint_ids18, adapter.velocity_limit18)
    paths = [Path(__file__).resolve(), ROOT/'scripts/vd1_base_response_viewer.py',
             ROOT/'scripts/diagnose_vd1_base_response.py', ROOT/'scripts/vd1_mujoco_scene.py',
             ROOT/'scripts/vd1_mujoco_road_contact.py',
             ROOT/'scripts/vd1_mujoco_contract.py', ROOT/'scripts/vd1_mujoco_velocity_impulse.py',
             TRAINING_ROOT/'scripts/vision/vd1_control.py', TRAINING_ROOT/'scripts/vision/vd1_scene.py',
             DEFAULT_SAMPLE, URDF_PATH]
    source_hashes = {str(p): sha256(p) for p in paths}
    write_json(output/'manifest.json', dict(
        simulation_only=True, privileged_debug=True, robot_communication=False, vision_processes=False,
        privileged_command_inputs=PRIVILEGED_INPUTS, actor_observations=70, actor_actions=18,
        actor_truth_usage='True path/base pose enter velocity and tool commands; no extra observation slots',
        policy=str(DEFAULT_POLICY), policy_sha256=POLICY_SHA256, speed_mps=args.speed,
        curvature_max_per_m=args.curvature_max, curve='sin^3: straight to x=2, wave to x=22, then straight',
        curve_point_spacing_m=.05, lookahead_m=args.lookahead, tool_mode=args.tool_mode,
        terrain='physical 2cm wide / 5cm deep trench' if args.physical_crack else
                'continuous flat plane; reference points have no collision', duration_s=args.duration,
        road_contact=args.road_contact if args.physical_crack else 'flat_plane',
        actor_hz=50, pd_hz=200, physics_dt=PD_DT, velocity_constraint='experimental impulse',
        physx_implicit_position=True, headless=args.headless, viewer_physics_isolated=True,
        working_target='base x=-1.02, z=-0.30; truth-curve lateral target clipped to trained workspace',
        mujoco_version=mujoco.__version__, torch_version=torch.__version__, source_hashes=source_hashes))
    truth = TruthRecorder(model, output)
    viewer = None
    stopped = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.__setitem__(0, True))
    previous_action = np.zeros(18, dtype=np.float32)
    reason, runtime_error, steps = 'duration_complete', None, 0
    metrics = []
    try:
        save_trajectory_svg(output/'trajectory.svg', curve, truth.rows)
        if not args.headless:
            viewer = TruthCurveViewer(model, data, curve, start_paused=not args.no_start_paused,
                                      realtime_factor=args.realtime_factor)
            print('[VD1 TRUTH DEBUG] Click window; SPACE starts/pauses; close or Ctrl+C exits', flush=True)
        with gzip.open(output/'actor_trace.jsonl.gz', 'xt', compresslevel=3) as trace, \
                (output/'tracking.csv').open('x', newline='', encoding='utf-8') as csv_handle:
            columns = ['sim_time_s', 'cmd_vx', 'actual_vx', 'cmd_w', 'actual_w', 'base_error_m',
                       'tcp_projection_error_m', 'tilt_deg', 'rear_target_clipped', 'base_x', 'base_y', 'tcp_x', 'tcp_y']
            writer = csv.DictWriter(csv_handle, fieldnames=columns)
            writer.writeheader()
            for step in range(int(math.ceil(args.duration/ACTOR_DT))):
                if stopped[0]:
                    reason = 'interrupted'
                    break
                command_time = float(data.time)
                command, tool, guidance = controller.update(command_time, data.xpos[base_id].copy(),
                                                              data.xmat[base_id].reshape(3, 3).copy())
                # VS Code: place a breakpoint here to inspect guidance, tool and command.
                display = dict(truth.rows[-1]) if truth.rows else {}
                display['guidance'] = guidance
                if viewer is not None and not viewer.wait_until_running(data, command, display, stopped):
                    reason = 'interrupted' if stopped[0] else 'viewer_closed'
                    break
                started = time.monotonic()
                measured = adapter.read_proprioception(data)
                observation = build_actor_observation(command, tool, measured['gyro_b'], measured['gravity_b'],
                                                       measured['q'], measured['qd'], adapter.default_q18, previous_action)
                with torch.inference_mode():
                    action = actor(torch.from_numpy(observation).unsqueeze(0)).numpy()[0]
                if action.shape != (18,) or not np.isfinite(action).all():
                    raise ValueError('Actor returned invalid actions')
                target = np.clip(adapter.default_q18+.25*action, -100., 100.)
                for _ in range(4):
                    mujoco.mj_forward(model, data)
                    adapter.apply_targets(data, target)
                    brake.step(data)
                mujoco.mj_forward(model, data)
                truth.write(data, step)
                row = truth.rows[-1]
                base_projection = curve.project(np.asarray(row['base_position_w'])[:2])
                tcp_projection = curve.project(np.asarray(row['tcp_position_w'])[:2])
                row['tcp_curve_error_m'] = tcp_projection['distance']
                metric = dict(sim_time_s=float(data.time), cmd_vx=float(command[0]),
                              actual_vx=row['body_velocity_b'][0], cmd_w=float(command[2]),
                              actual_w=row['body_angular_velocity_b'][2], base_error_m=base_projection['distance'],
                              tcp_projection_error_m=tcp_projection['distance'], tilt_deg=row['tilt_deg'],
                              rear_target_clipped=guidance['rear_target_clipped'],
                              base_x=row['base_position_w'][0], base_y=row['base_position_w'][1],
                              tcp_x=row['tcp_position_w'][0], tcp_y=row['tcp_position_w'][1])
                writer.writerow(metric)
                metrics.append(metric)
                trace.write(json.dumps(dict(step=step, command_time_s=command_time,
                    base_command_b=command.tolist(), tool_pose_b=tool.tolist(), guidance=guidance,
                    observation=observation.tolist(), action=action.tolist(), previous_action=previous_action.tolist(),
                    joint_target=target.tolist(), privileged_command_inputs=PRIVILEGED_INPUTS,
                    post_step_evaluation=metric), allow_nan=False)+'\n')
                previous_action = action.copy()
                steps += 1
                if viewer is not None:
                    display = dict(row, guidance=guidance)
                    viewer.update(data, command, display)
                if step == 0 or steps % 50 == 0:
                    save_trajectory_svg(output/'trajectory.svg', curve, truth.rows)
                    print('[VD1 TRUTH DEBUG] t=%.2f cmd_vx=%.3f actual_vx=%.3f cmd_w=%.3f base_err=%.3fm TCP_err=%.3fm tilt=%.1fdeg' %
                          (data.time, command[0], metric['actual_vx'], command[2], metric['base_error_m'],
                           metric['tcp_projection_error_m'], metric['tilt_deg']), flush=True)
                if adapter.read_proprioception(data)['gravity_b'][2] > -.5:
                    reason = 'IMU_tilt_stop'
                    break
                if viewer is not None:
                    viewer.pace(started, ACTOR_DT, stopped)
    except Exception as exc:
        reason, runtime_error = 'runtime_error', repr(exc)
        raise
    finally:
        if viewer is not None:
            viewer.close()
        truth.handle.close()
        save_trajectory_svg(output/'trajectory.svg', curve, truth.rows)
        steady = [r for r in metrics if r['sim_time_s'] >= 2.]
        def stats(key):
            values = np.asarray([r[key] for r in steady])
            return dict(mean=float(values.mean()), p95=float(np.percentile(values, 95)),
                        max=float(values.max())) if len(values) else None
        evaluation = truth.summary('straight', args.duration)
        evaluation.pop('truth_never_returned_to_actor', None)
        final_dynamics = dynamics_hash(model)
        write_json(output/'summary.json', dict(
            privileged_debug=True, sim_time_s=float(data.time), actor_steps=steps, stop_reason=reason,
            runtime_error=runtime_error, requested_duration_s=args.duration,
            base_cross_track_after_2s_m=stats('base_error_m'), tcp_projection_after_2s_m=stats('tcp_projection_error_m'),
            speed_after_2s_mps=stats('actual_vx'), tilt_max_deg=max([r['tilt_deg'] for r in metrics] or [0.]),
            rear_target_clipped_fraction=float(np.mean([r['rear_target_clipped'] for r in metrics])) if metrics else None,
            evaluation=evaluation, model_validation=validation, viewer_physics_isolated=True,
            initial_dynamics_sha256=initial_dynamics, final_dynamics_sha256=final_dynamics,
            dynamics_unchanged=initial_dynamics == final_dynamics,
            source_unchanged=source_hashes == {str(p): sha256(p) for p in paths},
            privileged_command_inputs=PRIVILEGED_INPUTS, experimental_velocity_constraint=brake.summary()))
    print('[VD1 TRUTH DEBUG] result=%s t=%.2fs | SVG: %s' % (reason, data.time, output/'trajectory.svg'), flush=True)
    return int(reason in ('runtime_error', 'IMU_tilt_stop'))


if __name__ == '__main__':
    sys.exit(main())
