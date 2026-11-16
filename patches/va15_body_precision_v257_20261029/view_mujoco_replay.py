#!/usr/bin/env python3
"""Replay a V257 MuJoCo CSV in an interactive viewer.

The CSV must contain qpos_0..qpos_N columns, as produced by
simulate_precision. A red capsule is the straight ground seam and a green
marker is the TCP's XY projection. This is a visualization only; it sends no
robot command.
"""
import argparse
import csv
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


def rotation_z_to(vector):
    """Return a rotation whose local +Z points along vector."""
    direction = np.asarray(vector, dtype=float)
    direction /= np.linalg.norm(direction)
    helper = np.array([0., 0., 1.])
    if abs(np.dot(direction, helper)) > .95:
        helper = np.array([0., 1., 0.])
    x = np.cross(helper, direction)
    x /= np.linalg.norm(x)
    y = np.cross(direction, x)
    return np.column_stack((x, y, direction)).reshape(-1)


def init_capsule(geom, a, b, radius, rgba):
    vec = np.asarray(b, dtype=float) - np.asarray(a, dtype=float)
    length = float(np.linalg.norm(vec))
    if length <= 1.e-9:
        return False
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        np.array([radius, radius, .5 * length]),
        .5 * (np.asarray(a) + np.asarray(b)),
        rotation_z_to(vec),
        np.asarray(rgba, dtype=float),
    )
    return True


def init_sphere(geom, position, radius, rgba):
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([radius, 0., 0.]),
        np.asarray(position, dtype=float),
        np.eye(3).reshape(-1),
        np.asarray(rgba, dtype=float),
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv", type=Path)
    parser.add_argument("--scene", type=Path, default=Path(
        "/home/dong/rl_sar_b2_z1_old/policy/b2_z1_no_gun/VA15/scene_flat.xml"))
    parser.add_argument("--speed", type=float, default=1.0,
                        help="replay speed multiplier; 1 is real recorded time")
    parser.add_argument("--no-seam", action="store_true")
    args = parser.parse_args()
    if args.speed <= 0.:
        raise SystemExit("--speed must be positive")
    rows = list(csv.DictReader(args.csv.open()))
    if not rows:
        raise SystemExit("empty CSV")
    qpos_names = sorted((name for name in rows[0] if name.startswith("qpos_")),
                        key=lambda name: int(name.split("_")[1]))
    if not qpos_names:
        raise SystemExit("CSV has no qpos columns; rerun simulate_precision from V257")
    model = mujoco.MjModel.from_xml_path(str(args.scene))
    data = mujoco.MjData(model)
    if len(qpos_names) != model.nq:
        raise SystemExit(f"CSV nq={len(qpos_names)} but scene nq={model.nq}")
    base = model.body("base_link").id
    tcp = model.site("ee_site").id
    with mujoco.viewer.launch_passive(model, data, show_left_ui=True,
                                      show_right_ui=True) as viewer:
        viewer.cam.distance = 3.0
        viewer.cam.azimuth = 135.
        viewer.cam.elevation = -22.
        last_t = None
        for row in rows:
            if not viewer.is_running():
                break
            data.qpos[:] = [float(row[name]) for name in qpos_names]
            data.qvel[:] = 0.
            mujoco.mj_forward(model, data)
            base_position = data.xpos[base].copy()
            if last_t is None:
                dt = .02
            else:
                dt = max(.005, float(row["time"]) - last_t)
            last_t = float(row["time"])
            with viewer.lock():
                viewer.user_scn.ngeom = 0
                if not args.no_seam:
                    # The synthetic_straight scenario's physical seam.
                    init_capsule(viewer.user_scn.geoms[0],
                                 [-1., 0., .004], [12., 0., .004],
                                 .012, [0.9, .05, .03, 1.])
                    viewer.user_scn.ngeom = 1
                tcp_xy = data.site_xpos[tcp].copy()
                tcp_xy[2] = .008
                init_sphere(viewer.user_scn.geoms[1], tcp_xy, .025,
                            [0.1, .9, .1, 1.])
                viewer.user_scn.ngeom = 2
                viewer.cam.lookat[:] = base_position + [0., 0., -.05]
            viewer.sync()
            time.sleep(dt / args.speed)


if __name__ == "__main__":
    main()
