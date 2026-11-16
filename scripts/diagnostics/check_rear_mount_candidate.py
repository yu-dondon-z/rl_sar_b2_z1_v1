#!/usr/bin/env python3
"""Static mounting/visibility study. No actor, mj_step, camera IO or robot IO.

Use each version's own source URDF, merged inertia and TCP. Flange centre is
link06 +75mm: the old .975m TCP means .900m clamp-to-tip, not .975m.
No dynamic response or drive-torque claims are made by this static study.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))


def ground_footprint(transform, intrinsic, image_size, base_height=0.5):
    """Ground intersections of image corner pixel centres, without occlusion."""
    width, height = image_size
    camera = np.asarray(transform, float).copy()
    camera[2, 3] += base_height
    uv = np.array([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]])
    directions = np.c_[uv, np.ones(4)] @ np.linalg.inv(intrinsic).T @ camera[:3, :3].T
    if np.any(directions[:, 2] >= -1e-8):
        raise ValueError(
            "This finite footprint diagnostic needs all four rays below the horizon"
        )
    return camera[:3, 3] - camera[2, 3] * directions / directions[:, 2:3]


def true_intervals(x, selected):
    """Sample-centre interval endpoints; lengths exclude any unseen gaps."""
    indices = np.flatnonzero(selected)
    if not len(indices):
        return []
    groups = np.split(indices, np.flatnonzero(np.diff(indices) != 1) + 1)
    return [
        dict(
            x_start_m=float(x[g[0]]),
            x_end_m=float(x[g[-1]]),
            length_m=float(x[g[-1]] - x[g[0]]),
        )
        for g in groups
    ]


def project(points, camera, intrinsic, image_size):
    optical = (points - camera[:3, 3]) @ camera[:3, :3]
    pixels = optical @ intrinsic.T
    with np.errstate(divide="ignore", invalid="ignore"):
        uv = pixels[:, :2] / pixels[:, 2:3]
    width, height = image_size
    valid = (
        np.isfinite(uv).all(1)
        & (optical[:, 2] >= 0.08)
        & (optical[:, 2] <= 4.5)
        & (uv[:, 0] >= 0)
        & (uv[:, 0] <= width - 1)
        & (uv[:, 1] >= 0)
        & (uv[:, 1] <= height - 1)
    )
    return uv, valid


def solve_ik(model, data, qids, limits, site, position, orientation, seeds):
    import mujoco
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation

    def residual(q):
        data.qpos[qids] = q
        mujoco.mj_forward(model, data)
        actual = data.site_xmat[site].reshape(3, 3)
        return np.r_[
            data.site_xpos[site] - position,
            0.3 * Rotation.from_matrix(orientation.T @ actual).as_rotvec(),
        ]

    lower, upper = limits.T
    best = None
    for seed in seeds:
        result = least_squares(
            residual,
            np.clip(seed, lower + 1e-7, upper - 1e-7),
            bounds=(lower + 1e-7, upper - 1e-7),
            max_nfev=200,
            ftol=1e-10,
            xtol=1e-10,
            gtol=1e-10,
        )
        if best is None or np.linalg.norm(result.fun) < np.linalg.norm(best.fun):
            best = result
        if np.linalg.norm(best.fun) < 1e-7:
            break
    residual(best.x)
    position_error = float(np.linalg.norm(best.fun[:3]))
    angle_error = float(np.linalg.norm(best.fun[3:]) / 0.3)
    return dict(
        q_rad=best.x.tolist(),
        position_error_m=position_error,
        orientation_error_deg=math.degrees(angle_error),
        minimum_limit_margin_deg=float(
            np.rad2deg(np.minimum(best.x - lower, upper - best.x)).min()
        ),
        solution_found=bool(position_error < 1e-4 and angle_error < 1e-3),
        scope="Numerical search within authored limits, not an impossibility proof or dynamic test",
    )


def visibility(
    model, data, camera, intrinsic, image_size, tube_vertices, target_x, near
):
    import cv2
    import mujoco
    from vd12_gun_mask import clip_convex_vertices

    # Flat, straight seam probe. These are evaluation rays, never perception.
    x = np.linspace(-1.6, -0.35, 1251)
    points = np.c_[x, np.zeros_like(x), np.zeros_like(x)]
    uv, in_image = project(points, camera, intrinsic, image_size)
    groups = np.ones(6, dtype=np.uint8)
    groups[3] = groups[5] = 0  # Invisible collision proxies and camera's own housing.
    visible = np.zeros(len(points), bool)
    blockers = Counter()
    for index in np.flatnonzero(in_image):
        direction = points[index] - camera[:3, 3]
        expected = np.linalg.norm(direction)
        direction /= expected
        optical_forward = direction @ camera[:3, 2]
        offset = near / optical_forward
        start = camera[:3, 3] + offset * direction
        hit = np.array([-1], dtype=np.int32)
        distance = mujoco.mj_ray(model, data, start, direction, groups, 1, -1, hit)
        # Road surface visual is slightly raised above the physical z=0 plane.
        visible[index] = distance >= 0 and abs(distance + offset - expected) <= 0.004
        if not visible[index]:
            name = model.geom(int(hit[0])).name if hit[0] >= 0 else "no_hit"
            blockers[name] += 1

    # Same geometric convex gun exclusion and 8px rim as the existing front end.
    # No fabricated capture metadata/timestamps are needed for this static study.
    vertices_c = (tube_vertices - camera[:3, 3]) @ camera[:3, :3]
    vertices_c = clip_convex_vertices(vertices_c, near)
    width, height = image_size
    mask = np.zeros((height, width), dtype=np.uint8)
    if len(vertices_c) >= 3:
        pixels = vertices_c @ intrinsic.T
        hull = cv2.convexHull(
            np.rint(np.clip(pixels[:, :2] / pixels[:, 2:3], -100000, 100000)).astype(
                np.int32
            )
        )
        cv2.fillConvexPoly(mask, hull, 255)
        mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (17, 17)))
    excluded = np.zeros(len(points), bool)
    ids = np.flatnonzero(in_image)
    pixel = np.rint(uv[ids]).astype(int)
    excluded[ids] = mask[pixel[:, 1], pixel[:, 0]] > 0
    usable = visible & ~excluded
    ahead = x >= target_x
    details = dict(
        image_ground_intervals=true_intervals(x, in_image),
        ray_visible_ground_intervals=true_intervals(x, visible),
        ray_visible_ahead_intervals=true_intervals(x, visible & ahead),
        after_gun_exclusion_ahead_intervals=true_intervals(x, usable & ahead),
        blocked_ray_counts=dict(blockers),
        ray_sample_spacing_m=0.001,
        ray_surface_tolerance_m=0.004,
        gun_exclusion_margin_px=8,
        scope="Static native visual-mesh rays and ideal ground line; NOT RHANet detection, depth validity, or frame timing",
    )
    return details, points, uv, in_image, visible, usable


def candidate_startup(model, data, qids, limits, site, guard):
    """Sample the inherited command ramp with a fixed nominal base, not dynamics."""
    import mujoco
    from scipy.spatial.transform import Rotation

    reset = np.array([0, 0.66647356, -1.25178768, 0.88531411, 0, 0])
    samples = []
    for blend in (0.0, 0.5, 1.0):
        mujoco.mj_resetDataKeyframe(model, data, model.key("vd1_default").id)
        data.qpos[2] = 0.5
        data.qpos[qids] = reset
        mujoco.mj_forward(model, data)
        start = data.site_xpos[site].copy()
        q0 = Rotation.from_matrix(data.site_xmat[site].reshape(3, 3)).as_quat()
        pitch = 0.3 + blend * (math.pi / 2 - 0.45 - 0.3)
        q1 = Rotation.from_euler("xyz", [0, pitch, math.pi]).as_quat()
        if np.dot(q0, q1) < 0:
            q1 = -q1
        end = np.array([-0.95, 0.0, (1 - blend) * 0.85])
        previous = reset.copy()
        for t in np.linspace(0.0, 1.0, 21):
            fraction = t**3 * (10 + t * (-15 + 6 * t))
            q = q0 + fraction * (q1 - q0)
            orientation = Rotation.from_quat(q / np.linalg.norm(q)).as_matrix()
            az = orientation[2, 0]
            rim_height = max(1.1 * az, 0.0) + 0.028 * math.sqrt(max(1 - az**2, 0.0))
            position = start + fraction * (end - start)
            position[2] += blend * fraction * (rim_height + 0.005)
            ik = solve_ik(
                model, data, qids, limits, site, position, orientation, [previous]
            )
            previous = np.array(ik["q_rad"])
            bound = guard.evaluate(data)["minimum_clearance_lower_bound_m"]
            samples.append(
                dict(
                    ground_blend=blend,
                    time_s=2 * t,
                    ik=ik,
                    minimum_clearance_lower_bound_m=bound,
                    target_tube_surface_min_z_m=float(position[2] - rim_height),
                )
            )
    return dict(
        scope="63 independent IK samples: fixed level base z=.5m; no moving-base, continuous-sweep or dynamic guarantee",
        all_samples_reachable=all(s["ik"]["solution_found"] for s in samples),
        minimum_keepout_bound_m=min(
            s["minimum_clearance_lower_bound_m"] for s in samples
        ),
        minimum_target_tube_surface_z_m=min(
            s["target_tube_surface_min_z_m"] for s in samples
        ),
        samples=samples,
    )


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-root", type=Path, default=ROOT / "outputs/rear_mount_candidate"
    )
    args = parser.parse_args(argv)
    import os

    os.environ.setdefault("MUJOCO_GL", "egl")
    import cv2
    import mujoco
    import torch
    from scipy.spatial.transform import Rotation
    from vd1_mujoco_scene import (
        build_scene,
        quaternion_matrix,
        VD10_URDF_PATH,
        VD13_URDF_PATH,
        validate_vd10_model,
        validate_vd13_model,
    )
    from vd5_camera_geometry import measured_camera_calibration, DEFAULT_CALIBRATION
    from vd5_gun_mask import EncoderGunMask
    from vd12_geometry_guard import MujocoGunGeometryGuard

    torch.set_num_threads(1)
    args.output_root.mkdir(parents=True, exist_ok=True)
    output = Path(
        tempfile.mkdtemp(
            prefix=time.strftime("%Y%m%d_%H%M%S") + "_", dir=args.output_root.resolve()
        )
    )
    cameras = measured_camera_calibration()
    calibration = cameras["rear_d435i"]
    camera = np.eye(4)
    camera[:3, :3] = quaternion_matrix(calibration["quaternion_ros_wxyz"])
    camera[:3, 3] = calibration["position_b"]
    intrinsic = np.array(calibration["intrinsic_matrix"])
    image_size = (calibration["width"], calibration["height"])
    footprint = ground_footprint(camera, intrinsic, image_size)
    camera[2, 3] += 0.5
    orientation = Rotation.from_euler(
        "xyz", [0, math.pi / 2 - 0.45, math.pi]
    ).as_matrix()
    goal_z = 0.005 + 0.028 * math.sqrt(1 - orientation[2, 0] ** 2)
    rng = np.random.default_rng(7)
    options = mujoco.MjvOption()
    options.geomgroup[3] = options.geomgroup[5] = 0
    cases = []
    sources = []
    startup = None
    for version, source, reach, distances, validate in [
        ("vd10", VD10_URDF_PATH, 0.9, [0.8, 1.0], validate_vd10_model),
        ("vd13", VD13_URDF_PATH, 0.75, [0.8, 0.95, 1.0, 1.1], validate_vd13_model),
    ]:
        scene = build_scene(
            0.0,
            output / version,
            tool_markers=False,
            road_surface="tape",
            robot_version=version,
            camera_calibration=cameras,
        )
        model = mujoco.MjModel.from_xml_path(str(scene))
        audit = validate(model)
        sources.append(
            dict(
                version=version,
                urdf=str(source),
                urdf_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                audit=audit,
            )
        )
        data = mujoco.MjData(model)
        model.geom_group[model.geom("rear_camera_housing").id] = 5
        joints = np.array([model.joint("joint%d" % i).id for i in range(1, 7)])
        qids, limits = model.jnt_qposadr[joints], model.jnt_range[joints].copy()
        site, link = model.site("tool_tcp").id, model.body("link06").id
        guard = MujocoGunGeometryGuard(model, urdf_path=source, dtype=torch.float64)
        gun = EncoderGunMask(source)
        seeds = [
            np.array([0, 0.66647356, -1.25178768, 0.88531411, 0, 0]),
            limits.mean(axis=1),
        ]
        seeds += list(
            rng.uniform(limits[:, 0] + 1e-6, limits[:, 1] - 1e-6, size=(16, 6))
        )
        near = float(model.vis.map.znear * model.stat.extent)
        renderer = mujoco.Renderer(model, height=image_size[1], width=image_size[0])
        try:
            for distance in distances:
                mujoco.mj_resetDataKeyframe(model, data, model.key("vd1_default").id)
                data.qpos[2], data.qvel[:] = 0.5, 0.0
                goal = np.array([-distance, 0, goal_z])
                ik = solve_ik(model, data, qids, limits, site, goal, orientation, seeds)
                name = f"clamp_tip_{round(reach*1000)}mm_work_{round(distance*1000)}mm"
                case = dict(
                    name=name,
                    robot_version=version,
                    clamp_to_tip_m=reach,
                    tail_to_clamp_m=1.1 - reach,
                    tcp_in_link06_m=model.site_pos[site].tolist(),
                    work_distance_m=distance,
                    target_world_m=goal.tolist(),
                    ik=ik,
                    ideal_forward_view_m=max(
                        0.0, float(footprint[:, 0].max()) + distance
                    ),
                )
                # Failed IK poses must not be described as valid operating visibility.
                if ik["solution_found"]:
                    geometry = guard.evaluate(data)
                    ends = np.array(geometry["tube_endpoints_w_m"])
                    expected_tip = data.site_xpos[site]
                    if min(np.linalg.norm(ends - expected_tip, axis=1)) > 1e-6:
                        raise AssertionError("Candidate geometry and TCP disagree")
                    case["geometry"] = dict(
                        minimum_clearance_lower_bound_m=geometry[
                            "minimum_clearance_lower_bound_m"
                        ],
                        minimum_pair_name=geometry["minimum_pair_name"],
                        conservative_keepout_violated=geometry[
                            "conservative_keepout_violated"
                        ],
                        semantics=geometry["semantics"],
                    )
                    vertices = (
                        gun.vertices @ data.xmat[link].reshape(3, 3).T + data.xpos[link]
                    )
                    result, points, uv, inside, visible, usable = visibility(
                        model,
                        data,
                        camera,
                        intrinsic,
                        image_size,
                        vertices,
                        -distance,
                        near,
                    )
                    case["visibility"] = result
                    renderer.update_scene(
                        data, camera="rear_d435i", scene_option=options
                    )
                    frame = renderer.render().copy()
                    for index in np.flatnonzero(inside)[::8]:
                        color = (
                            (0, 210, 0)
                            if usable[index]
                            else ((220, 0, 220) if visible[index] else (230, 30, 30))
                        )
                        cv2.circle(
                            frame, tuple(np.rint(uv[index]).astype(int)), 2, color, -1
                        )
                    cv2.putText(
                        frame,
                        name,
                        (8, 22),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.42,
                        (255, 255, 255),
                        1,
                    )
                    cv2.putText(
                        frame,
                        "Projected probes: green clear / red blocked / magenta excluded",
                        (8, 43),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.32,
                        (255, 255, 255),
                        1,
                    )
                    image = output / (name + ".png")
                    cv2.imwrite(str(image), cv2.cvtColor(frame, cv2.COLOR_RGB2BGR))
                    case["annotated_static_render"] = str(image)
                cases.append(case)
                print(json.dumps(case, ensure_ascii=False), flush=True)
        finally:
            renderer.close()
        if version == "vd13":
            startup = candidate_startup(model, data, qids, limits, site, guard)
    summary = dict(
        sources=sources,
        schema_version=2,
        source_camera_config=str(DEFAULT_CALIBRATION),
        camera_config_sha256=hashlib.sha256(
            DEFAULT_CALIBRATION.read_bytes()
        ).hexdigest(),
        diagnostic_source_sha256=hashlib.sha256(
            Path(__file__).read_bytes()
        ).hexdigest(),
        scope="Nominal level base z=.50m, nominal fixed legs, static straight ground seam, original working pitch",
        physics_steps=0,
        actor_loaded=False,
        hardware_io=False,
        training_assets_modified=False,
        candidate_geometry_only=False,
        inertias_updated=True,
        no_dynamic_claim="Source masses/COM/inertias validated, but static test is NOT dynamics or deployment approval",
        camera_extrinsics_quality="Existing approximate installation transform, not a new physical calibration",
        ground_footprint_world_corners_m=footprint.tolist(),
        cases=cases,
        candidate_startup=startup,
    )
    (output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n"
    )
    print("OUTPUT=" + str(output), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
