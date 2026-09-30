#!/usr/bin/env python3
"""Verify VA15 key-1 tool visibility and real solver contacts after attachment."""
import argparse
import json
from pathlib import Path
import tempfile

import mujoco
import numpy as np
import torch

from run_b2_z1_v4 import Playback, ROOT, StagedControl


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images', type=Path, help='Optional before/after rendered PNG directory')
    args = parser.parse_args()
    torch.set_num_threads(1)
    results = {}
    for scene in ('scene_flat.xml', 'scene_vision.xml'):
        p = Playback(ROOT/'policy/b2_z1_no_gun/VA15', scene)
        with tempfile.TemporaryDirectory(prefix='va15_gun_check_') as directory:
            c = StagedControl(p, Path(directory), 'walk', .22)

            def check_attached(expected):
                assert p.tool_attached == expected
                assert (p.model.geom_rgba[p.gun, 3] > 0) == expected
                assert int(p.model.geom_contype[p.gun]) == (2 if expected else 0)
                assert int(p.model.geom_conaffinity[p.gun]) == (1 if expected else 0)

            def advance_until(predicate, limit=750):
                for _ in range(limit):
                    c.tick()
                    assert not c.fallen(), p.status()
                    if predicate():
                        return
                raise AssertionError('Stage did not complete: '+c.phase)

            check_attached(False)
            assert not c.key(ord('1')), 'Early key 1 must not attach the tool'
            check_attached(False)
            assert c.key(ord('0'))
            advance_until(lambda: c.stand_ready)
            check_attached(False)
            renderer = None
            try:
                if args.images and scene == 'scene_flat.xml':
                    from PIL import Image
                    args.images.mkdir(parents=True, exist_ok=True)
                    renderer = mujoco.Renderer(p.model, height=480, width=640)
                    camera = mujoco.MjvCamera()
                    camera.lookat[:] = p.data.xpos[p.base]+[-.4, 0, .1]
                    camera.distance, camera.azimuth, camera.elevation = 2.8, 90., -12.
                    renderer.update_scene(p.data, camera=camera)
                    before = renderer.render().copy()
                    Image.fromarray(before).save(args.images/'before_key1.png')
                assert c.key(ord('1'))
                check_attached(True)
                if renderer:
                    renderer.update_scene(p.data, camera=camera)
                    after = renderer.render().copy()
                    changed = int(np.count_nonzero(np.any(before != after, axis=2)))
                    assert changed > 100, 'Attached gun produced no visible image change'
                    Image.fromarray(after).save(args.images/'after_key1.png')
                    results['render_changed_pixels'] = changed
            finally:
                if renderer:
                    renderer.close()
            advance_until(lambda: c.phase == 'ready')
            check_attached(True)
            tip = p.data.geom_xpos[p.gun]-.55*p.data.geom_xmat[p.gun].reshape(3, 3)[:, 2]
            np.testing.assert_allclose(tip, p.data.site_xpos[p.site], atol=1e-7)
            results[scene] = dict(stage=c.phase, height=float(p.data.xpos[p.base, 2]),
                                  tilt_deg=p.status()['tilt_deg'], visible=True,
                                  collision_masks=[2, 1])
            if scene == 'scene_flat.xml':
                # Use a separate test model with an elevated plane intersecting
                # the tool tip. Never alter the active controller/model assets.
                forces = {}
                for attached in (False, True):
                    probe = Playback(ROOT/'policy/b2_z1_no_gun/VA15', scene)
                    probe.data.qpos[:] = p.data.qpos
                    probe.data.qvel[:] = 0
                    probe.joint_target[:] = p.joint_target
                    probe.set_tool_attached(attached)
                    floor = probe.model.geom('floor').id
                    probe.model.geom_pos[floor, 2] = tip[2]+.02
                    probe.model.geom_sameframe[floor] = 0
                    mujoco.mj_forward(probe.model, probe.data)
                    peak = 0.
                    for _ in range(20):
                        mujoco.mj_step(probe.model, probe.data)
                        for contact_id, contact in enumerate(probe.data.contact):
                            if probe.gun in (contact.geom1, contact.geom2):
                                force = np.zeros(6)
                                mujoco.mj_contactForce(probe.model, probe.data, contact_id, force)
                                peak = max(peak, float(np.linalg.norm(force[:3])))
                    forces[str(attached)] = peak
                assert forces['False'] == 0
                assert forces['True'] > .01, 'Enabled collider produced no contact force'
                results['test_obstacle_peak_force_n'] = forces
            assert c.key(ord('9'))
            check_attached(False)
            advance_until(lambda: c.phase == 'idle')
            check_attached(False)
    print('PASS: '+json.dumps(results, indent=2))


if __name__ == '__main__':
    main()
