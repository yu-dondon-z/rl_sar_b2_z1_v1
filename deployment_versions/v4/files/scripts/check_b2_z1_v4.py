#!/usr/bin/env python3
"""Regression checks for the MuJoCo v4 interface, gun contact and startup."""
import json
from pathlib import Path
import tempfile

import mujoco
import numpy as np
import torch

from run_b2_z1_v4 import Playback, ROOT, StagedControl


def main():
    torch.set_num_threads(1)
    profile = ROOT/'policy/b2_z1_no_gun/v4'
    p = Playback(profile, 'scene_flat.xml')
    p.data.qvel[3:6] = [1, 2, 3]
    mujoco.mj_forward(p.model, p.data)
    np.testing.assert_allclose(p.observation()[10:13], [.2, .4, .6], atol=1e-6)
    p.data.qvel[:] = 0
    mujoco.mj_forward(p.model, p.data)
    gun = p.model.geom('welding_gun').id
    tip = p.data.geom_xpos[gun]-.55*p.data.geom_xmat[gun].reshape(3, 3)[:, 2]
    np.testing.assert_allclose(tip, p.data.site_xpos[p.site], atol=1e-7)
    assert p.model.body('link_gun').mass[0] == .001
    assert p.model.body('dummy_target').mass[0] == 1.
    assert p.model.geom_contype[gun] == 2 and p.model.geom_conaffinity[gun] == 1
    # Raise the test plane into the gun tip. Read a real contact force from
    # the solver, independently of visual appearance or collision flags.
    p.model.geom_pos[p.model.geom('floor').id, 2] = tip[2]+.02
    # The original plane was at the world origin; invalidate the compiler's
    # same-frame shortcut before moving this static geom for the test.
    p.model.geom_sameframe[p.model.geom('floor').id] = 0
    peak_force = 0.
    mujoco.mj_forward(p.model, p.data)
    for _ in range(20):
        mujoco.mj_step(p.model, p.data)
        for contact in range(p.data.ncon):
            c = p.data.contact[contact]
            if gun in (c.geom1, c.geom2):
                force = np.zeros(6)
                mujoco.mj_contactForce(p.model, p.data, contact, force)
                peak_force = max(peak_force, np.linalg.norm(force[:3]))
    assert peak_force > .01, 'Gun produced no contact force'

    results = {'gun_contact_force_n': peak_force}
    for mode, speed in [('stand', 0.), ('walk', .35)]:
        p = Playback(profile, 'scene_flat.xml')
        heights, tilts = [], []
        for _ in range(1500):
            p.command(speed)
            p.infer()
            p.step()
            status = p.status()
            if p.data.time >= 1:
                heights.append(status['position'][2])
                tilts.append(status['tilt_deg'])
        assert min(heights) > .45 and max(tilts) < 10, (mode, min(heights), max(tilts))
        if mode == 'walk':
            assert p.data.xpos[p.base, 0] > 5.
        results[mode] = dict(final_position=p.data.xpos[p.base].tolist(),
                             min_height=min(heights), max_tilt_deg=max(tilts))
    # A missing perception stream must decay the velocity without moving the
    # held TCP target. Check the actual production command-loss path.
    held = p.target.copy()
    with tempfile.TemporaryDirectory(prefix='b2_z1_v4_loss_check_') as exchange:
        for _ in range(150):
            p.vision_command(Path(exchange), .35, True)
    assert np.linalg.norm(p.velocity) == 0
    np.testing.assert_array_equal(p.target, held)
    with tempfile.TemporaryDirectory(prefix='b2_z1_v4_keys_check_') as exchange:
        p = Playback(profile, 'scene_flat.xml')
        control = StagedControl(p, Path(exchange), 'walk', .35)
        assert .08 < p.data.xpos[p.base, 2] < .20, 'Startup must be lying down'
        assert not control.fallen(), 'Intentional lying pose was classified as a fall'
        assert all(c.dist > -1e-5 for c in p.data.contact), 'Startup intersects the ground'
        np.testing.assert_allclose(
            p.data.qpos[p.qids[p.arm]], p.lying_arm, atol=1e-12,
            err_msg='Z1 did not start in the configured lying storage pose')
        assert not p.tool_attached
        assert p.model.geom_contype[gun] == 0
        assert p.model.geom_conaffinity[gun] == 0
        assert p.model.geom_rgba[gun, 3] == 0
        results['lying_storage'] = dict(
            arm_position=p.lying_arm.tolist(), welding_gun_attached=False)
        lying_qpos = p.data.qpos.copy()
        assert not control.key(ord('1')) and not control.key(ord('T'))
        for _ in range(20):
            control.tick()
        assert p.data.time == 0 and control.phase == 'idle'
        np.testing.assert_array_equal(p.data.qpos, lying_qpos)
        control.key(ord('0'))
        assert not control.key(ord('T'))
        rise_tilts = []
        for _ in range(400):
            control.tick()
            rise_tilts.append(p.status()['tilt_deg'])
            assert not control.fallen(), p.status()
            if control.stand_ready:
                break
        assert control.phase == 'standing' and np.linalg.norm(p.velocity) == 0
        assert control.stand_ready, 'GetUp did not reach the deployment stand target'
        assert not p.tool_attached, 'Gun must wait for operator confirmation/key 1'
        assert p.data.xpos[p.base, 2] > .40 and max(rise_tilts) < 10
        results['get_up'] = dict(initial_height=lying_qpos[2],
                                standing_height=p.data.xpos[p.base, 2],
                                max_tilt_deg=max(rise_tilts))
        assert control.key(ord('1'))
        assert p.tool_attached, 'Key 1 must confirm/install the simulated gun'
        assert not control.key(ord('T'))
        for _ in range(400):
            control.tick()
            assert np.linalg.norm(p.velocity) == 0
            if control.phase == 'ready':
                break
        assert control.phase == 'ready', 'Arm did not settle at its start pose'
        ready_x = p.data.xpos[p.base, 0]
        for _ in range(100):
            control.tick()
        assert control.phase == 'ready' and abs(p.data.xpos[p.base, 0]-ready_x) < .03
        assert control.key(ord('T'))
        for _ in range(500):
            control.tick()
            assert p.status()['tilt_deg'] < 10 and p.data.xpos[p.base, 2] > .45
        assert p.data.xpos[p.base, 0]-ready_x > 1.5
        assert control.key(ord('t')), 'Second T must stop tracking'
        assert not p.tool_active
        assert not control.key(ord('T')), 'Do not restart while still stopping'
        held = p.target.copy()
        for _ in range(150):
            control.tick()
            assert not control.fallen()
        assert control.phase == 'ready' and np.linalg.norm(p.velocity) == 0
        np.testing.assert_array_equal(p.target, held)
        stopped = p.data.xpos[p.base].copy()
        for _ in range(150):
            control.tick()
        # Loaded Z1/gun stance has a small contact-settling oscillation; bound
        # the three-second drift to 6 cm while still rejecting policy motion.
        assert np.linalg.norm(p.data.xpos[p.base, :2]-stopped[:2]) < .06
        assert control.key(ord('t'))
        for _ in range(150):
            control.tick()
        assert control.key(ord('T'))
        for _ in range(150):
            control.tick()
        assert control.phase == 'ready' and np.linalg.norm(p.velocity) == 0
        assert control.key(ord('T'))
        for _ in range(150):
            control.tick()
        assert control.key(32)
        for _ in range(150):
            control.tick()
        assert control.phase == 'ready'
        assert control.key(ord('t'))
        for _ in range(50):
            control.tick()
        assert control.key(ord('0'))
        for _ in range(250):
            control.tick()
        assert control.phase == 'standing' and np.linalg.norm(p.velocity) == 0
        assert p.data.xpos[p.base, 2] > .40 and p.status()['tilt_deg'] < 10
        assert control.key(ord('9'))
        for _ in range(250):
            control.tick()
            assert not control.fallen()
            if control.phase == 'idle':
                break
        assert control.phase == 'idle' and not p.tool_attached
        np.testing.assert_allclose(
            p.data.qpos[p.qids[p.arm]], p.lying_arm, atol=.02)
        results['keyboard_sequence'] = 'lying -> 0 -> 1 -> T -> stop/restart -> 0 -> 9 -> lying passed'
    print('PASS: '+json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
