#!/usr/bin/env python3
"""MuJoCo-only v4 playback with the training observation/action contract."""
import argparse
import json
import math
import os
from pathlib import Path
import select
import subprocess
import sys
import tempfile
import termios
import time

import mujoco
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]


class Playback:
    def __init__(self, profile, scene):
        self.cfg = json.loads((profile / 'config.json').read_text())
        self.model = mujoco.MjModel.from_xml_path(str(profile / scene))
        self.data = mujoco.MjData(self.model)
        self.actor = torch.jit.load(str(profile / 'policy.pt'), map_location='cpu').eval()
        self.joints = np.array([self.model.joint(n).id for n in self.cfg['joint_names']])
        self.qids = self.model.jnt_qposadr[self.joints]
        self.vids = self.model.jnt_dofadr[self.joints]
        assert np.array_equal(self.model.actuator_trnid[:, 0], self.joints), 'Actuator order mismatch'
        self.default = np.array(self.cfg['default_dof_pos'], dtype=np.float64)
        # Standing is a physical controller target, while ``default`` is the
        # normalization reference learned by the actor.  Keeping them
        # separate prevents the asymmetric training reference from becoming
        # an incomplete/crooked GetUp pose.
        self.stand = np.array(
            self.cfg.get('deployment_stand_dof_pos', self.cfg['default_dof_pos']),
            dtype=np.float64)
        if self.stand.shape != self.default.shape or not np.isfinite(self.stand).all():
            raise ValueError('deployment_stand_dof_pos must match default_dof_pos')
        self.arm = np.array([8, 13, 14, 15, 16, 17])
        self.legs = np.array([i for i in range(18) if i not in self.arm])
        self.policy_kp = np.broadcast_to(
            np.asarray(self.cfg['kp'], dtype=np.float64), self.default.shape).copy()
        self.policy_kd = np.broadcast_to(
            np.asarray(self.cfg['kd'], dtype=np.float64), self.default.shape).copy()
        self.stand_kp = np.broadcast_to(
            np.asarray(self.cfg.get('deployment_stand_kp', self.cfg['kp']),
                       dtype=np.float64), self.default.shape).copy()
        self.stand_kd = np.broadcast_to(
            np.asarray(self.cfg.get('deployment_stand_kd', self.cfg['kd']),
                       dtype=np.float64), self.default.shape).copy()
        self.lying_arm = np.asarray(
            self.cfg.get('deployment_lying_arm_dof_pos',
                         self.default[[8, 13, 14, 15, 16, 17]]),
            dtype=np.float64)
        if self.lying_arm.shape != (6,) or not np.isfinite(self.lying_arm).all():
            raise ValueError('deployment_lying_arm_dof_pos must contain 6 finite joint angles')
        self.lying = self.default.copy()
        self.lying[4:8] = 1.4
        self.lying[9:13] = -2.8
        self.lying[self.arm] = self.lying_arm
        self.control_kp = self.policy_kp.copy()
        self.control_kd = self.policy_kd.copy()
        joint_mid = self.model.jnt_range[self.joints].mean(axis=1)
        joint_half_range = np.diff(self.model.jnt_range[self.joints], axis=1)[:, 0]*.45
        self.data.qpos[self.qids] = np.clip(self.default, joint_mid-joint_half_range, joint_mid+joint_half_range)
        self.data.ctrl[self.arm] = self.default[self.arm]
        mujoco.mj_forward(self.model, self.data)
        self.base = self.model.body('base_link').id
        self.site = self.model.site('ee_site').id
        self.gun = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, 'welding_gun')
        self.gun_visual = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, 'ee_tracked_tcp_visual')
        self.gun_contype = int(self.model.geom_contype[self.gun]) if self.gun >= 0 else 0
        self.gun_conaffinity = int(self.model.geom_conaffinity[self.gun]) if self.gun >= 0 else 0
        self.gun_rgba = self.model.geom_rgba[self.gun].copy() if self.gun >= 0 else None
        self.gun_visual_rgba = (
            self.model.geom_rgba[self.gun_visual].copy()
            if self.gun_visual >= 0 else None)
        self.site_rgba = self.model.site_rgba[self.site].copy()
        self.tool_attached = True
        self.last_action = np.zeros(18, dtype=np.float32)
        self.filtered_action = self.last_action.copy()
        self.target = self.clamp_target(self.to_body(self.data.site_xpos[self.site]))
        self.velocity = np.zeros(3)
        self.joint_target = self.default.copy()
        self.heading = 0.0
        self.elapsed = 0.0
        self.tool_active = False
        self.path = np.empty((0, 3))
        self.path_mtime = None
        self.last_received = -math.inf

    def rotation(self):
        return self.data.xmat[self.base].reshape(3, 3)

    def reset_lying(self):
        """Fold B2 and Z1 in their no-tool transport/home configuration."""
        self.set_tool_attached(False)
        pose = self.lying.copy()
        self.data.qpos[self.qids] = pose
        self.data.qvel[:] = 0
        self.joint_target = pose.copy()
        self.data.ctrl[self.legs] = 0
        self.data.ctrl[self.arm] = pose[self.arm]
        mujoco.mj_forward(self.model, self.data)
        feet = [self.model.geom(name+'_foot_collision').id for name in ('FL', 'FR', 'RL', 'RR')]
        bottom = min(self.data.geom_xpos[g, 2]-self.model.geom_size[g, 0] for g in feet)
        self.data.qpos[2] -= bottom
        mujoco.mj_forward(self.model, self.data)
        print('Z1 folded to home pose for lying; welding gun is detached.', flush=True)

    def set_tool_attached(self, attached):
        """Represent the real install/remove operation in the MuJoCo model."""
        attached = bool(attached)
        if self.gun >= 0:
            self.model.geom_contype[self.gun] = self.gun_contype if attached else 0
            self.model.geom_conaffinity[self.gun] = self.gun_conaffinity if attached else 0
            self.model.geom_rgba[self.gun] = self.gun_rgba
            self.model.geom_rgba[self.gun, 3] = self.gun_rgba[3] if attached else 0.
        if self.gun_visual >= 0:
            self.model.geom_rgba[self.gun_visual] = self.gun_visual_rgba
            self.model.geom_rgba[self.gun_visual, 3] = (
                self.gun_visual_rgba[3] if attached else 0.)
        self.model.site_rgba[self.site] = self.site_rgba
        self.model.site_rgba[self.site, 3] = self.site_rgba[3] if attached else 0.
        # Keep the training-time 1 kg TCP payload surrogate in dynamics even
        # while the visible gun is removed. Runtime mass edits require a full
        # model-constant rebuild and can corrupt the free-base reference pose;
        # more importantly, retaining the payload gives the safer conservative
        # stand-up test before the real gun is installed.
        self.tool_attached = attached

    def to_body(self, points):
        return (points - self.data.xpos[self.base]) @ self.rotation()

    def clamp_target(self, target):
        return np.clip(target, self.cfg.get('tcp_min', [-1.0, -0.15, -0.38]),
                       self.cfg.get('tcp_max', [-0.55, 0.15, -0.30]))

    def command(self, speed):
        self.elapsed += self.cfg['policy_dt']
        blend = min(self.elapsed / 4.0, 1.0)
        desired = np.array([speed, 0.0, 0.0]) * blend**2 * (3 - 2*blend)
        self.velocity += (1-math.exp(-self.cfg['policy_dt']/.4)) * (desired-self.velocity)

    def vision_command(self, exchange, speed, enabled):
        path = exchange / 'latest_trajectory.csv'
        try:
            stamp = path.stat().st_mtime_ns
            if stamp != self.path_mtime:
                points = np.loadtxt(str(path), delimiter=',', comments='#', ndmin=2)
                if points.shape[1] == 3 and len(points) >= 3 and np.isfinite(points).all():
                    self.path = points
                    self.path_mtime = stamp
                    self.last_received = time.monotonic()
        except (OSError, ValueError):
            pass
        dt = self.cfg['policy_dt']
        desired = np.zeros(3)
        self.tool_active = False
        fresh = time.monotonic()-self.last_received <= 1.0
        points_b = self.to_body(self.path)
        forward = points_b[(points_b[:, 0] > .1) & (points_b[:, 0] < 2.5)]
        if enabled and fresh and len(forward) >= 3:
            tangent = forward[-1, :2]-forward[0, :2]
            if tangent[0] < 0:
                tangent = -tangent
            tangent /= max(np.linalg.norm(tangent), 1e-8)
            if tangent[0] >= .65:
                lookahead = forward[np.argmin(abs(forward[:, 0]-.9))]
                heading = np.clip(math.atan2(tangent[1], tangent[0]) +
                                  math.atan2(.5*lookahead[1], max(lookahead[0], .1)), -1.2, 1.2)
                self.heading += .25*(heading-self.heading)
                if abs(self.heading) <= .45:
                    desired[0] = speed*max(.25, math.cos(self.heading))
                desired[2] = np.clip(.9*self.heading, -.18, .18)
                self.elapsed += dt
                blend = min(self.elapsed/4., 1.)
                desired *= blend**2*(3-2*blend)
                rear = points_b[(points_b[:, 0] >= -1) & (points_b[:, 0] <= -.55)]
                if len(rear):
                    target = rear[np.argmin(abs(rear[:, 0]+.775))].copy()
                    height_offset = self.cfg.get('tcp_path_height_offset', .10)
                    if self.cfg.get('tcp_height_world_vertical', False):
                        target += self.rotation().T @ np.array([0., 0., height_offset])
                    else:
                        target[2] += height_offset
                    delta = (1-math.exp(-dt/.5))*(target-self.target)
                    delta *= min(1., .15*dt/max(np.linalg.norm(delta), 1e-8))
                    self.target = self.clamp_target(self.target+delta)
                    self.tool_active = True
        self.velocity += (1-math.exp(-dt/.4))*(desired-self.velocity)
        if not np.any(desired) and np.linalg.norm(self.velocity) <= .01:
            self.velocity[:] = 0
            self.elapsed = 0
            self.heading = 0

    def export_state(self, exchange, sequence, enabled):
        values = [sequence, self.data.time, self.model.nq, *self.data.qpos,
                  int(enabled), *self.velocity]
        atomic_text(exchange/'latest_state.txt', ' '.join(map(str, values))+'\n')

    def update_markers(self):
        # The renderer process has its own MjData; these markers are GUI-only.
        target_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, 'trajectory_target_visual')
        if target_id >= 0:
            self.data.mocap_pos[self.model.body_mocapid[target_id]] = (
                self.data.xpos[self.base]+self.rotation()@self.target)
        if len(self.path):
            local = self.to_body(self.path)
            visible = self.path[(local[:, 0] > -2) & (local[:, 0] < 4)]
            if len(visible):
                for i, index in enumerate(np.linspace(0, len(visible)-1, 24).astype(int)):
                    name = 'trajectory_path_visual_{:02d}'.format(i)
                    body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
                    if body >= 0:
                        self.data.mocap_pos[self.model.body_mocapid[body]] = visible[index]+[0, 0, .015]

    def observation(self):
        body_velocity = np.zeros(6)
        # mjOBJ_BODY local coordinates use the principal-inertia frame, which
        # is rotated relative to base_link. Isaac observes base_link gyro.
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, self.base, body_velocity, 0)
        body_velocity[:3] = self.rotation().T @ body_velocity[:3]
        obs = np.concatenate((
            self.velocity, self.target, self.cfg['target_orientation'],
            np.clip(body_velocity[:3], -100, 100)*.2,
            self.rotation().T @ [0, 0, -1],
            np.clip(self.data.qpos[self.qids]-self.default, -100, 100),
            np.clip(self.data.qvel[self.vids], -100, 100)*.05, self.last_action,
        )).astype(np.float32)
        assert obs.shape == (70,) and np.isfinite(obs).all()
        return obs

    def infer(self):
        obs = self.observation()
        with torch.inference_mode():
            action = self.actor(torch.from_numpy(obs).unsqueeze(0))[0].numpy()
        self.last_action = np.clip(action, -self.cfg['action_clip'], self.cfg['action_clip'])
        self.filtered_action += self.cfg['action_filter_alpha'] * (self.last_action-self.filtered_action)
        self.joint_target = self.default + self.cfg['action_scale']*self.filtered_action

    def step(self):
        # Explicit B2 PD at 200 Hz; Z1 uses MuJoCo implicit position drives.
        for _ in range(round(self.cfg['policy_dt']/self.cfg['control_dt'])):
            torque = (self.control_kp*(self.joint_target-self.data.qpos[self.qids])-
                      self.control_kd*self.data.qvel[self.vids])
            torque = np.clip(torque, -np.array(self.cfg['torque_limits']), self.cfg['torque_limits'])
            self.data.ctrl[self.legs] = torque[self.legs]
            self.data.ctrl[self.arm] = self.joint_target[self.arm]
            for _ in range(round(self.cfg['control_dt']/self.cfg['physics_dt'])):
                mujoco.mj_step(self.model, self.data)
        mujoco.mj_forward(self.model, self.data)

    def status(self):
        return dict(time_s=float(self.data.time), position=self.data.xpos[self.base].tolist(),
                    tilt_deg=math.degrees(math.acos(np.clip(self.rotation()[2, 2], -1, 1))),
                    command=self.velocity.tolist(), target=self.target.tolist(),
                    tcp=self.data.site_xpos[self.site].tolist(), tool_active=self.tool_active,
                    tool_attached=self.tool_attached)


def atomic_text(path, contents):
    temporary = path.with_name('.'+path.name+'.tmp')
    temporary.write_text(contents)
    temporary.replace(path)


class TerminalKeyboard:
    """Nonblocking single-key terminal input; preserve Ctrl+C and restore tty."""
    def __init__(self, stream=None):
        self.stream = sys.stdin if stream is None else stream
        self.fd = None
        self.saved = None
        self.escape = None

    def __enter__(self):
        if self.stream.isatty():
            self.fd = self.stream.fileno()
            self.saved = termios.tcgetattr(self.fd)
            settings = termios.tcgetattr(self.fd)
            settings[3] &= ~(termios.ICANON | termios.ECHO)
            settings[6][termios.VMIN] = 0
            settings[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, settings)
        return self

    def read(self):
        if self.fd is None or not select.select([self.fd], [], [], 0)[0]:
            return []
        keys = []
        for key in os.read(self.fd, 128):
            # Arrow/function-key escape sequences can contain digits such as
            # '1'. They must not accidentally activate a robot stage.
            if self.escape == 'start':
                self.escape = 'sequence' if key in (ord('['), ord('O')) else None
            elif self.escape == 'sequence':
                if 0x40 <= key <= 0x7e:
                    self.escape = None
            elif key == 27:
                self.escape = 'start'
            elif key in b'019tT ':
                keys.append(key)
        return keys

    def __exit__(self, exc_type, exc_value, traceback):
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSANOW, self.saved)


class StagedControl:
    """Keyboard transitions run on the simulation thread, including in tests."""
    def __init__(self, playback, exchange, mode, speed):
        self.p = playback
        self.exchange, self.mode, self.speed = exchange, mode, speed
        self.phase = 'idle'
        self.phase_time = 0.
        self.settled_time = 0.
        self.stand_ready = False
        self.stand_from_lying = False
        self.ready_uses_stand_gains = False
        self.after_stop = 'ready'
        # Keep the policy's standing TCP reference, independently of the
        # tucked startup arm pose (which is outside the tracking workspace).
        self.start_target = playback.target.copy()
        self.ready_target = playback.stand.copy()
        playback.reset_lying()
        self.stand_start = playback.joint_target.copy()
        self.down_start = playback.joint_target.copy()
        self.down_upright = playback.joint_target.copy()

    def transition(self, phase):
        previous = self.phase
        self.phase, self.phase_time, self.settled_time = phase, 0., 0.
        if phase == 'standing':
            self.stand_ready = False
        if phase == 'ready':
            self.ready_uses_stand_gains = previous in ('preparing', 'settling')
        print('Stage: '+phase, flush=True)
        if phase == 'ready':
            print('Default mode: trajectory OFF. Press T to enable tracking.', flush=True)
        elif phase == 'tracking':
            print('Trajectory mode: ON. Press T again to stop tracking.', flush=True)

    def stop_tracking(self, after_stop='ready'):
        self.after_stop = after_stop
        self.p.tool_active = False
        self.transition('stopping')
        destination = {
            'ready': 'default mode.',
            'standing': 'standing.',
            'getting_down': 'the gun-removal/get-down sequence.',
        }.get(after_stop, after_stop+'.')
        print('Trajectory OFF; slowing to '+destination, flush=True)

    def start_symmetric_settle(self, after_stop='ready'):
        """Return the legs to verified four-foot support before the next state."""
        self.after_stop = after_stop
        self.p.velocity[:] = 0.
        self.p.tool_active = False
        self.stand_start = self.p.joint_target.copy()
        self.ready_target = self.p.stand.copy()
        # The arm remains at its current work pose during the leg-only settle.
        # It is folded only after four-foot support has been confirmed.
        self.ready_target[self.p.arm] = self.p.joint_target[self.p.arm]
        self.transition('settling')
        if after_stop == 'getting_down':
            print('Get-down queued; first returning to verified symmetric four-foot support.',
                  flush=True)

    def start_get_down(self):
        """Remove the simulated tool, fold Z1, then lower B2."""
        self.p.velocity[:] = 0.
        self.p.tool_active = False
        self.p.set_tool_attached(False)
        self.down_start = self.p.joint_target.copy()
        self.down_upright = self.down_start.copy()
        self.down_upright[self.p.legs] = self.p.stand[self.p.legs]
        self.down_upright[self.p.arm] = self.p.lying_arm
        self.transition('getting_down')
        print('Welding gun removed in simulation; folding Z1 before lowering B2.',
              flush=True)

    def fallen(self):
        # A folded robot starts below the normal standing-height threshold.
        rising = self.phase in ('idle', 'getting_down') or (
            self.phase == 'standing' and self.stand_from_lying and self.phase_time < 4.5)
        status = self.p.status()
        return status['position'][2] < (.07 if rising else .25) or status['tilt_deg'] > 45

    def key(self, key):
        if key == ord('0'):
            if self.phase in ('tracking', 'stopping'):
                self.stop_tracking('standing')
            else:
                self.stand_from_lying = self.phase == 'idle'
                if self.stand_from_lying:
                    self.p.set_tool_attached(False)
                self.stand_start = self.p.joint_target.copy()
                self.transition('standing')
            return True
        if key == ord('9'):
            if self.phase == 'idle':
                print('B2 and Z1 are already in the lying/home state.', flush=True)
                return True
            if self.phase == 'tracking':
                self.stop_tracking('getting_down')
                return True
            if self.phase == 'stopping':
                self.after_stop = 'getting_down'
                print('Get-down requested; stopping and settling to four-foot support first.',
                      flush=True)
                return True
            if self.phase == 'settling':
                self.after_stop = 'getting_down'
                print('Get-down queued; waiting for four-foot support verification.',
                      flush=True)
                return True
            if self.phase == 'preparing':
                self.start_symmetric_settle('getting_down')
                return True
            if self.phase == 'standing' and not self.stand_ready:
                self.start_symmetric_settle('getting_down')
                return True
            if self.phase in ('standing', 'ready'):
                self.start_get_down()
                return True
            print('Wait for the current get-down transition to finish.', flush=True)
            return False
        if key == ord('1'):
            if self.phase != 'standing' or not self.stand_ready:
                error = np.max(np.abs(
                    self.p.data.qpos[self.p.qids[self.p.legs]]-
                    self.p.stand[self.p.legs])) if self.phase == 'standing' else math.inf
                print(f'Press 0 first and wait for standing to settle (leg error={error:.3f} rad).',
                      flush=True)
                return False
            if not self.p.tool_attached:
                self.p.set_tool_attached(True)
                print('Welding gun installed in simulation; starting policy handoff.',
                      flush=True)
            # Preserve the held actuator target across the policy handoff.
            seed = (self.p.joint_target-self.p.default)/self.p.cfg['action_scale']
            self.p.filtered_action[:] = seed
            self.p.last_action[:] = seed
            self.p.target = self.start_target.copy()
            self.p.velocity[:] = 0
            self.p.elapsed = 0.
            self.p.heading = 0.
            self.transition('preparing')
            return True
        if key in (ord('T'), ord('t')):
            if self.phase == 'tracking':
                self.stop_tracking()
                return True
            if self.phase == 'stopping':
                print('Trajectory OFF; wait for stopping to finish before T.', flush=True)
                return False
            if self.phase != 'ready':
                print('Press 1 and wait for the arm start pose before T.', flush=True)
                return False
            if self.mode == 'stand':
                print('Stand-only mode does not start trajectory motion.', flush=True)
                return False
            if self.mode == 'vision' and (time.monotonic()-self.p.last_received > 1. or len(self.p.path) < 3):
                print('Waiting for a fresh visual crack path; press T when VALID.', flush=True)
                return False
            self.p.elapsed = 0.
            self.p.heading = 0.
            self.transition('tracking')
            return True
        if key == 32 and self.phase == 'tracking':
            self.stop_tracking()
            return True
        return False

    def tick(self):
        p = self.p
        active = self.phase == 'tracking'
        if self.mode == 'vision':
            p.vision_command(self.exchange, self.speed, active)
        else:
            p.command(self.speed if active and self.mode == 'walk' else 0.)
        if self.phase == 'idle':
            return
        self.phase_time += p.cfg['policy_dt']
        if self.phase == 'standing':
            if self.stand_from_lying:
                # Raise B2 first with Z1 at its compact home pose. Only after
                # B2 is upright is Z1 deployed to the welding work pose. The
                # gun remains detached throughout both motions.
                standing_folded = p.stand.copy()
                standing_folded[p.arm] = p.lying_arm
                if self.phase_time <= 2.:
                    blend = min(self.phase_time/2., 1.)
                    blend = blend*blend*(3-2*blend)
                    p.joint_target = self.stand_start+blend*(
                        standing_folded-self.stand_start)
                else:
                    blend = min((self.phase_time-2.)/2., 1.)
                    blend = blend*blend*(3-2*blend)
                    p.joint_target = standing_folded+blend*(
                        p.stand-standing_folded)
            else:
                blend = min(self.phase_time/2., 1.)
                blend = blend*blend*(3-2*blend)
                p.joint_target = self.stand_start+blend*(p.stand-self.stand_start)
            # Returning from a moving policy can occur at any gait phase.
            # Blend both the target and gains so a high stand stiffness is
            # never applied instantaneously to an off-target swing leg.
            p.control_kp[:] = p.policy_kp+blend*(p.stand_kp-p.policy_kp)
            p.control_kd[:] = p.policy_kd+blend*(p.stand_kd-p.policy_kd)
        elif self.phase == 'preparing':
            p.control_kp[:] = p.stand_kp
            p.control_kd[:] = p.stand_kd
            p.infer()
            # Warm up only the arm. The zero-command AMP policy is not a
            # standing controller, so applying its leg outputs here makes B2
            # walk before the operator enables the trajectory.
            held_leg_action = (
                (p.stand[p.legs]-p.default[p.legs])/p.cfg['action_scale'])
            p.filtered_action[p.legs] = held_leg_action
            p.last_action[p.legs] = held_leg_action
            p.joint_target[p.legs] = p.stand[p.legs]
        elif self.phase == 'settling':
            # Finish a velocity stop at a symmetric four-foot support pose,
            # but leave Z1 at its current working pose so a subsequent T does
            # not redeploy the long gun. This prevents slow drift from holding
            # an arbitrary final gait phase.
            blend = min(self.phase_time/1.2, 1.)
            blend = blend*blend*(3-2*blend)
            p.joint_target = self.stand_start+blend*(
                self.ready_target-self.stand_start)
            p.control_kp[:] = p.policy_kp
            p.control_kd[:] = p.policy_kd
            p.control_kp[p.legs] = (
                p.policy_kp[p.legs]+blend*(p.stand_kp[p.legs]-p.policy_kp[p.legs]))
            p.control_kd[p.legs] = (
                p.policy_kd[p.legs]+blend*(p.stand_kd[p.legs]-p.policy_kd[p.legs]))
        elif self.phase == 'getting_down':
            # The real operator must remove the gun before key 9. In MuJoCo it
            # was detached on state entry. Fold Z1 while upright, then lower
            # B2 with the compact arm held fixed.
            if self.phase_time <= 2.:
                blend = min(self.phase_time/2., 1.)
                blend = blend*blend*(3-2*blend)
                p.joint_target = self.down_start+blend*(
                    self.down_upright-self.down_start)
            else:
                blend = min((self.phase_time-2.)/2., 1.)
                blend = blend*blend*(3-2*blend)
                p.joint_target = self.down_upright+blend*(
                    p.lying-self.down_upright)
            p.control_kp[:] = p.stand_kp
            p.control_kd[:] = p.stand_kd
        elif self.phase == 'ready':
            # Keep the verified end of the preparation transition. Tracking
            # resumes inference from this physically applied action history.
            # The initial ready pose has symmetric stand legs and benefits
            # from high stand gains. After a moving stop the held pose can
            # contain a gait-phase offset, so it must retain policy gains.
            p.control_kp[:] = p.stand_kp if self.ready_uses_stand_gains else p.policy_kp
            p.control_kd[:] = p.stand_kd if self.ready_uses_stand_gains else p.policy_kd
        else:
            p.control_kp[:] = p.policy_kp
            p.control_kd[:] = p.policy_kd
            p.infer()
        p.step()
        if self.phase == 'standing':
            leg_error = float(np.max(np.abs(
                p.data.qpos[p.qids[p.legs]]-p.stand[p.legs])))
            leg_speed = float(np.max(np.abs(p.data.qvel[p.vids[p.legs]])))
            status = p.status()
            upright = status['position'][2] >= .40 and status['tilt_deg'] <= 10.
            required_time = 4. if self.stand_from_lying else 2.
            max_error = float(p.cfg.get('deployment_stand_max_joint_error', .12))
            stable = (self.phase_time >= required_time and upright and
                      leg_error <= max_error and leg_speed <= .20)
            self.settled_time = self.settled_time+p.cfg['policy_dt'] if stable else 0.
            if self.settled_time >= .4 and not self.stand_ready:
                if self.stand_from_lying:
                    self.stand_from_lying = False
                    print('B2 upright and Z1 at the safe installation pose. '
                          'Install the real welding gun, then press 1.', flush=True)
                self.stand_ready = True
                print(f'Standing settled: max_leg_target_error={leg_error:.3f} rad, '
                      f'max_leg_speed={leg_speed:.3f} rad/s; policy handoff enabled.',
                      flush=True)
        if self.phase == 'preparing':
            quiet = np.max(np.abs(p.data.qvel[p.vids[p.arm]])) < .20
            self.settled_time = self.settled_time+p.cfg['policy_dt'] if quiet else 0.
            if self.phase_time >= 2. and self.settled_time >= .4:
                self.transition('ready')
                print('Arm start pose ready. Press T to start trajectory motion.', flush=True)
        if self.phase == 'settling':
            leg_error = float(np.max(np.abs(
                p.data.qpos[p.qids[p.legs]]-p.stand[p.legs])))
            leg_speed = float(np.max(np.abs(p.data.qvel[p.vids[p.legs]])))
            status = p.status()
            max_error = float(p.cfg.get('deployment_stand_max_joint_error', .12))
            stable = (
                self.phase_time >= 1.2
                and status['position'][2] >= .40
                and status['tilt_deg'] <= 10.
                and leg_error <= max_error
                and leg_speed <= .20
            )
            self.settled_time = self.settled_time+p.cfg['policy_dt'] if stable else 0.
            if self.settled_time >= .4:
                applied = (p.joint_target-p.default)/p.cfg['action_scale']
                p.filtered_action[:] = applied
                p.last_action[:] = applied
                if self.after_stop == 'getting_down':
                    print(
                        f'Four-foot support verified: leg_error={leg_error:.3f} rad, '
                        f'leg_speed={leg_speed:.3f} rad/s; starting get-down.',
                        flush=True,
                    )
                    self.start_get_down()
                else:
                    self.transition('ready')
                    print('Stable four-foot stop ready. Press T to resume trajectory motion.',
                          flush=True)
        if self.phase == 'getting_down' and self.phase_time >= 4.:
            p.joint_target = p.lying.copy()
            p.filtered_action[:] = (
                (p.lying-p.default)/p.cfg['action_scale'])
            p.last_action[:] = p.filtered_action
            self.transition('idle')
            print('B2 lying; Z1 is in its compact home pose. Press 0 to stand.',
                  flush=True)
        if self.phase == 'stopping' and np.linalg.norm(p.velocity) <= .01:
            p.velocity[:] = 0
            p.elapsed = 0.
            self.stand_start = p.joint_target.copy()
            if self.after_stop == 'standing':
                self.stand_from_lying = False
                self.transition('standing')
            else:
                self.start_symmetric_settle(self.after_stop)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', type=Path, default=ROOT/'policy/b2_z1_no_gun/v4')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--duration', type=float, default=0, help='Simulated seconds; 0 runs until the window closes')
    parser.add_argument('--mode', choices=('vision', 'walk', 'stand'), default='vision')
    parser.add_argument('--vision-device', default='cuda:0')
    parser.add_argument('--display-vision', action='store_true')
    parser.add_argument('--vision-rate-hz', type=float, default=10.0,
                        help='off-screen camera/RHACrackNet update rate')
    parser.add_argument('--speed', type=float, default=.35)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--auto-start', action='store_true',
                        help='For diagnostics: execute the same 0 -> 1 -> T sequence automatically')
    parser.add_argument(
        '--auto-get-down-at', type=float, default=0., metavar='SECONDS',
        help='Diagnostic only: press 9 automatically at this simulation time',
    )
    args = parser.parse_args()
    if not 0 <= args.speed <= .38:
        parser.error('--speed must be in [0, 0.38], within the trained range')
    if not 2.0 <= args.vision_rate_hz <= 25.0:
        parser.error('--vision-rate-hz must be in [2, 25]')
    if args.headless and args.duration <= 0:
        parser.error('Headless playback requires a positive --duration')
    if args.auto_get_down_at < 0:
        parser.error('--auto-get-down-at cannot be negative')
    if not args.auto_start and not sys.stdin.isatty():
        parser.error('Manual control requires an interactive terminal; use --auto-start for unattended diagnostics')
    torch.set_num_threads(1)
    scene = 'scene_vision.xml' if args.mode == 'vision' else 'scene_flat.xml'
    playback = Playback(args.profile, scene)
    # Per-run isolation allows diagnostics alongside the previous deployment.
    profile_label = str(playback.cfg.get('version', args.profile.name))
    checkpoint_label = Path(playback.cfg.get('checkpoint', 'unknown')).name
    exchange = Path(tempfile.mkdtemp(prefix=f'b2_z1_mujoco_{profile_label}_'))
    bridge = None
    bridge_args = None
    bridge_log = None
    if args.mode == 'vision':
        bridge_log = (exchange/'bridge.log').open('w')
        bridge_args = [sys.executable, '-u', str(ROOT/'scripts/run_mujoco_rhacracknet_bridge.py'),
                       '--exchange-dir', str(exchange), '--scene', str(args.profile/scene),
                       '--device', args.vision_device, '--segmentation-mode', 'network', '--threshold', '.90',
                       '--minimum-path-length', '.20', '--minimum-path-span', '.15',
                       '--max-forward', '1.8', '--cache-behind', '1.2',
                       '--cache-hold-seconds', '0.5', '--save-overlay']
        if args.display_vision:
            bridge_args.append('--display')
        bridge_args.extend(playback.cfg.get('vision_bridge_args', []))
    controller = StagedControl(playback, exchange, args.mode, args.speed)
    viewer = None
    if not args.headless:
        import mujoco.viewer
        viewer = mujoco.viewer.launch_passive(playback.model, playback.data)
        viewer.cam.distance = 4
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
    records = []
    steps = 0
    vision_stride = max(1, round(1.0/(playback.cfg['policy_dt']*args.vision_rate_hz)))
    start = time.monotonic()
    print(
        f'{profile_label} {checkpoint_label} | Initially lying with compact Z1 and no gun | '
        'TERMINAL keys (no Enter): 0 stand + Z1 installation pose, '
        '1 confirm gun + policy start pose, T toggle trajectory/default, '
        f'9 remove gun + get down, Space stop, Ctrl+C exit | diagnostics={exchange}',
        flush=True,
    )
    auto_stage = 0
    auto_get_down_requested = False
    keyboard = TerminalKeyboard()
    try:
        keyboard.__enter__()
        if bridge_args:
            bridge = subprocess.Popen(bridge_args, env=dict(os.environ, MUJOCO_GL='egl'),
                                      stdin=subprocess.DEVNULL, stdout=bridge_log, stderr=subprocess.STDOUT)
        while (args.duration <= 0 or playback.data.time < args.duration) and (viewer is None or viewer.is_running()):
            tick_start = time.monotonic()
            for key in keyboard.read():
                controller.key(key)
            if args.auto_start:
                if auto_stage == 0:
                    controller.key(ord('0'))
                    auto_stage = 1
                elif auto_stage == 1 and controller.phase == 'standing' and controller.stand_ready:
                    if args.mode == 'stand':
                        auto_stage = 3
                    else:
                        controller.key(ord('1'))
                        auto_stage = 2
                elif auto_stage == 2 and controller.phase == 'ready' and args.mode != 'stand':
                    if args.mode != 'vision' or time.monotonic()-playback.last_received <= 1.:
                        if controller.key(ord('T')):
                            auto_stage = 3
                if (
                    args.auto_get_down_at > 0
                    and not auto_get_down_requested
                    and playback.data.time >= args.auto_get_down_at
                ):
                    controller.key(ord('9'))
                    auto_get_down_requested = True
            if bridge is not None:
                if bridge.poll() is not None:
                    raise RuntimeError('Vision bridge exited; see '+str(exchange/'bridge.log'))
                if steps % vision_stride == 0:
                    playback.export_state(
                        exchange, steps//vision_stride,
                        controller.phase == 'tracking')
            controller.tick()
            status = playback.status()
            status['stage'] = controller.phase
            steps += 1
            if args.output:
                records.append(status)
            if steps % 250 == 0:
                print(json.dumps(status), flush=True)
            if steps % 25 == 0:
                status['real_time_factor'] = playback.data.time/max(time.monotonic()-start, 1e-6)
                atomic_text(exchange/'latest_controller_status.json', json.dumps(status, indent=2)+'\n')
            if controller.fallen():
                raise RuntimeError('Fall detected: '+json.dumps(status))
            if auto_get_down_requested and controller.phase == 'idle':
                print('Automatic direct-9 regression completed in the safe idle pose.',
                      flush=True)
                break
            if viewer:
                playback.update_markers()
                viewer.cam.lookat[:] = playback.data.xpos[playback.base]
                viewer.sync()
            if viewer or bridge or not args.auto_start:
                time.sleep(max(0, playback.cfg['policy_dt']-(time.monotonic()-tick_start)))
    except KeyboardInterrupt:
        print('\nPlayback stopped.', flush=True)
    finally:
        keyboard.__exit__(None, None, None)
        if bridge is not None:
            bridge.terminate()
            try:
                bridge.wait(timeout=5)
            except subprocess.TimeoutExpired:
                bridge.kill()
                bridge.wait(timeout=5)
            bridge_log.close()
        if viewer:
            viewer.close()
        if args.output:
            args.output.write_text(json.dumps(records, indent=2)+'\n')
        print(f'Simulated {playback.data.time:.2f} s in {time.monotonic()-start:.2f} wall seconds', flush=True)


if __name__ == '__main__':
    main()
