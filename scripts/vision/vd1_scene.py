"""VD1 rendered scene and independent road geometry.

Import ``VD1SceneCfg`` only after starting Isaac's ``AppLauncher``.  The class
is loaded lazily so the road geometry can also be checked on a CPU without
Isaac.  Road stations describe the physical mesh and are for scene generation
and independent evaluation only; they must not enter visual control inputs.

The checkpoint's imported URDF already contains its gun mass and collision.
Only optical, massless TCP fiducials are added here.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np


ROAD_HALF_LENGTH_M = 95.0
ROAD_HALF_WIDTH_M = 95.0
CRACK_WIDTH_M = 0.02
CRACK_DEPTH_M = 0.05
CURVE_START_X_M = 2.0
CURVE_WAVELENGTH_M = 20.0
ROAD_MESH_STEP_M = 0.05

TOOL_TCP_GUN_M = (0.0, -0.03, -0.62)
TOOL_MARKER_CYAN_GUN_M = TOOL_TCP_GUN_M
TOOL_MARKER_MAGENTA_GUN_M = (0.0, -0.03, -0.56)
MARKER_CALIBRATION = {
    "marker_radius_m": 0.035,
    "marker_axial_spacing_m": 0.06,
    "marker_tip_extension_m": 0.0,
}


def _validate_profile(curvature_max, start_x_m, wavelength_m):
    values = np.asarray([curvature_max, start_x_m, wavelength_m], dtype=float)
    if not np.isfinite(values).all() or curvature_max < 0 or wavelength_m <= 0:
        raise ValueError("road curvature must be finite/nonnegative and wavelength positive")


def road_profile(x, curvature_max=0.0, *, start_x_m=CURVE_START_X_M,
                 wavelength_m=CURVE_WAVELENGTH_M):
    """Local road Y: straight, one sin^3 cycle, then straight again.

    The joins have zero first and second derivative.  With k=2*pi/wavelength,
    the analytical maximum planar curvature is exactly ``3*A*k**2``.  This
    function exposes scene/evaluation geometry, never a steering reference.
    """
    _validate_profile(curvature_max, start_x_m, wavelength_m)
    x = np.asarray(x, dtype=np.float64)
    if not np.isfinite(x).all():
        raise ValueError("road coordinates must be finite")
    wave_number = 2.0 * math.pi / wavelength_m
    amplitude = curvature_max / (3.0 * wave_number**2)
    phase = wave_number * (x - start_x_m)
    curved = (x > start_x_m) & (x < start_x_m + wavelength_m)
    return np.where(curved, amplitude * np.sin(phase)**3, 0.0)


def road_stations(curvature_max=0.0, *, start_x_m=CURVE_START_X_M,
                  wavelength_m=CURVE_WAVELENGTH_M,
                  road_half_length_m=ROAD_HALF_LENGTH_M,
                  mesh_step_m=ROAD_MESH_STEP_M):
    """Return the exact physical mesh's local XY stations, including its ends.

    This is a static road: launch another run with a new ``curvature_max`` to
    increase difficulty.  Changing a controller variable cannot change it.
    """
    _validate_profile(curvature_max, start_x_m, wavelength_m)
    if (not np.isfinite([road_half_length_m, mesh_step_m]).all()
            or mesh_step_m <= 0
            or road_half_length_m <= max(abs(start_x_m), abs(start_x_m + wavelength_m))):
        raise ValueError("road ends must enclose the curve; mesh step must be positive")
    count = round(wavelength_m / mesh_step_m) + 1
    if count < 2:
        raise ValueError("road curve needs at least two mesh stations")
    curved_x = np.linspace(start_x_m, start_x_m + wavelength_m, count)
    x = np.concatenate(([-road_half_length_m], curved_x, [road_half_length_m]))
    return np.column_stack((x, road_profile(
        x, curvature_max, start_x_m=start_x_m, wavelength_m=wavelength_m)))


def road_half_mesh_data(stations, *, left_side=True,
                        half_width_m=ROAD_HALF_WIDTH_M,
                        crack_width_m=CRACK_WIDTH_M,
                        crack_depth_m=CRACK_DEPTH_M):
    """Build one closed, outward-facing road half without transverse seams.

    Both top surfaces have upward normals.  That matters because PhysX
    triangle-mesh collision is one-sided: an inverted half lets feet through.
    Returns NumPy vertices ``[N,3]`` and triangle indices ``[M,3]``.
    """
    stations = np.asarray(stations, dtype=np.float64)
    dimensions = np.asarray([half_width_m, crack_width_m, crack_depth_m], dtype=float)
    if (stations.ndim != 2 or stations.shape[1] != 2 or len(stations) < 2
            or not np.isfinite(stations).all() or np.any(np.diff(stations[:, 0]) <= 0)):
        raise ValueError("mesh stations must be finite, strictly ascending local XY")
    if not np.isfinite(dimensions).all() or np.any(dimensions <= 0):
        raise ValueError("road half-width, crack width and depth must be positive")
    if np.max(np.abs(stations[:, 1])) + 0.5 * crack_width_m >= half_width_m:
        raise ValueError("crack reaches the outside of the road")
    x, centre_y = stations.T
    sign = 1.0 if left_side else -1.0
    inner_y = centre_y + sign * 0.5 * crack_width_m
    outer_y = np.full_like(x, sign * half_width_m)
    top_z, bottom_z = np.zeros_like(x), np.full_like(x, -crack_depth_m)
    vertices = np.stack((
        np.column_stack((x, inner_y, top_z)),
        np.column_stack((x, outer_y, top_z)),
        np.column_stack((x, inner_y, bottom_z)),
        np.column_stack((x, outer_y, bottom_z)),
    ), axis=1).reshape(-1, 3)
    faces = []
    for station in range(len(x) - 1):
        a, b = 4 * station, 4 * (station + 1)
        faces.extend((
            (a, a + 1, b + 1), (a, b + 1, b),
            (a + 2, b + 3, a + 3), (a + 2, b + 2, b + 3),
            (a, b, b + 2), (a, b + 2, a + 2),
            (a + 1, a + 3, b + 3), (a + 1, b + 3, b + 1),
        ))
    last = 4 * (len(x) - 1)
    faces.extend(((0, 2, 3), (0, 3, 1),
                  (last, last + 1, last + 3), (last, last + 3, last + 2)))
    faces = np.asarray(faces, dtype=np.int64)
    triangles = vertices[faces]
    volume_times_six = np.einsum(
        "ij,ij->i", triangles[:, 0], np.cross(triangles[:, 1], triangles[:, 2])).sum()
    if volume_times_six < 0:
        faces = faces[:, [0, 2, 1]]
    return vertices, faces


def configure_scene(cfg, curvature_max=0.0, usd_cache_dir=None):
    """Configure and return ``cfg`` in place, retaining the upstream robot.

    ``usd_cache_dir`` only selects the generated USD cache.  It never changes
    the source URDF, joint limits, masses or actuator parameters.
    """
    if cfg.num_envs != 1:
        raise ValueError("VD1 external image feedback supports exactly one environment")
    # Validate geometry before any USD/PhysX resources are created.
    road_stations(curvature_max)
    for half in (cfg.road_left, cfg.road_right):
        half.spawn.curvature_max = float(curvature_max)
    if usd_cache_dir is not None:
        cfg.robot.spawn.usd_dir = str(Path(usd_cache_dir).expanduser().resolve())
        cfg.robot.spawn.usd_file_name = "b2_z1_vd1.usd"
    return cfg


def _build_scene_class():
    """Load Isaac-dependent definitions only after AppLauncher is running."""
    import isaaclab.sim as sim_utils
    from isaaclab.assets import ArticulationCfg, AssetBaseCfg
    from isaaclab.scene import InteractiveSceneCfg
    from isaaclab.sensors import ContactSensorCfg, ImuCfg
    from isaaclab.utils import configclass

    from b2_z1.assets.b2_z1_vd1 import B2_Z1_VD1_CFG
    from b2_z1.tasks.manager_based.b2_z1.b2_z1_amp_env_cfg import GROUND_MATERIAL_CFG
    from b2_z1.tasks.manager_based.b2_z1.b2_z1_va16_dual_vision_env_cfg import (
        VA16DualVisionSceneCfg,
    )

    @sim_utils.clone
    def _spawn_road_half(prim_path, cfg, translation=None, orientation=None, **kwargs):
        import isaacsim.core.utils.prims as prim_utils
        import trimesh
        from isaaclab.terrains.utils import create_prim_from_mesh

        stations = road_stations(
            cfg.curvature_max, start_x_m=cfg.curve_start_x_m,
            wavelength_m=cfg.wavelength_m, road_half_length_m=cfg.half_length_m,
            mesh_step_m=cfg.mesh_step_m)
        vertices, faces = road_half_mesh_data(
            stations, left_side=cfg.left_side, half_width_m=cfg.half_width_m,
            crack_width_m=cfg.crack_width_m, crack_depth_m=cfg.crack_depth_m)
        create_prim_from_mesh(
            prim_path, trimesh.Trimesh(vertices=vertices, faces=faces, process=False),
            translation=translation, orientation=orientation,
            visual_material=cfg.visual_material, physics_material=cfg.physics_material)
        return prim_utils.get_prim_at_path(prim_path)

    @configclass
    class VD1RoadHalfCfg(sim_utils.SpawnerCfg):
        func = _spawn_road_half
        left_side: bool = True
        curvature_max: float = 0.0
        curve_start_x_m: float = CURVE_START_X_M
        wavelength_m: float = CURVE_WAVELENGTH_M
        half_length_m: float = ROAD_HALF_LENGTH_M
        half_width_m: float = ROAD_HALF_WIDTH_M
        mesh_step_m: float = ROAD_MESH_STEP_M
        crack_width_m: float = CRACK_WIDTH_M
        crack_depth_m: float = CRACK_DEPTH_M
        visual_material = sim_utils.PreviewSurfaceCfg(
            diffuse_color=(0.42, 0.42, 0.42), roughness=0.95)
        physics_material = GROUND_MATERIAL_CFG

    def marker(name, position, color):
        return AssetBaseCfg(
            prim_path=f"{{ENV_REGEX_NS}}/Robot/link_gun/{name}",
            spawn=sim_utils.SphereCfg(
                radius=MARKER_CALIBRATION["marker_radius_m"],
                collision_props=sim_utils.CollisionPropertiesCfg(collision_enabled=False),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=color, metallic=0.0, roughness=0.4)),
            init_state=AssetBaseCfg.InitialStateCfg(pos=position))

    # Copy only the real camera configurations and their visual housing, not
    # any command, reward, trajectory or debug-visualization implementation.
    cameras = VA16DualVisionSceneCfg(num_envs=1, env_spacing=4.0)

    @configclass
    class VD1SceneCfg(InteractiveSceneCfg):
        num_envs: int = 1
        env_spacing: float = 4.0

        terrain = AssetBaseCfg(
            prim_path="/World/ground",
            spawn=sim_utils.CuboidCfg(
                size=(200.0, 200.0, 0.10),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=(0.003, 0.003, 0.003), roughness=1.0),
                physics_material=GROUND_MATERIAL_CFG),
            init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -0.10)),
            collision_group=-1)
        road_left = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/RoadLeft", spawn=VD1RoadHalfCfg(left_side=True),
            collision_group=-1)
        road_right = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/RoadRight", spawn=VD1RoadHalfCfg(left_side=False),
            collision_group=-1)
        robot: ArticulationCfg = B2_Z1_VD1_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")

        # Independent evaluation only.  Contact values are not actor inputs.
        contact_forces = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Robot/.*", update_period=0.005,
            history_length=3, track_air_time=True, debug_vis=False)
        imu = ImuCfg(
            prim_path="{ENV_REGEX_NS}/Robot/base_link", update_period=0.005,
            debug_vis=False)
        front_rgb = cameras.front_rgb.copy()
        rear_d435i = cameras.rear_d435i.copy()
        rear_d435i_model = cameras.rear_d435i_model.copy()

        tool_marker_cyan = marker("tool_marker_cyan", TOOL_MARKER_CYAN_GUN_M, (0.0, 1.0, 1.0))
        tool_marker_magenta = marker(
            "tool_marker_magenta", TOOL_MARKER_MAGENTA_GUN_M, (1.0, 0.0, 1.0))
        sky_light = AssetBaseCfg(
            prim_path="/World/skyLight",
            spawn=sim_utils.DomeLightCfg(
                intensity=750.0, color=(1.0, 1.0, 1.0), texture_file=None))

    VD1SceneCfg.__qualname__ = "VD1SceneCfg"
    return VD1SceneCfg


def __getattr__(name):
    if name == "VD1SceneCfg":
        scene_class = _build_scene_class()
        globals()[name] = scene_class
        return scene_class
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
