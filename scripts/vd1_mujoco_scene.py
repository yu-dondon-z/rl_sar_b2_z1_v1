"""Independent MuJoCo assets for the upstream VD1 B2 + six-axis Z1 actor.

The original VD1 URDF supplies robot dynamics and geometry by default. The
explicit zero-gun debug mode removes only link_gun mass and inertia in generated
MJCF, preserving its geometry and the source URDF. Generated assets use a cache.
Road geometry in this module is for building the physical scene and independent
evaluation only.  It must never be passed to the visual controller or actor.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


TRAINING_ROOT = Path("/home/dong/b2_z1")
URDF_PATH = TRAINING_ROOT / "source/b2_z1/b2_z1/assets/b2_z1/b2_z1_vd1.urdf"
URDF_SHA256 = "14ae3abccfa4d7e5f74f69347ced43f7b1ec5bed728376972260d8b18130c9c6"
VD3_URDF_PATH = Path('/home/dong/b2_z1_VD2/source/b2_z1/b2_z1/assets/b2_z1/b2_z1_vd3_zero_gun.urdf')
VD3_URDF_SHA256 = 'a6a1f7143d1ce33d6e25b3cbaf7809d17a971c1ec793e6055b365fda9bb39be0'
VD10_URDF_PATH = Path('/home/dong/b2_z1_VD2/source/b2_z1/b2_z1/assets/b2_z1/b2_z1_vd10_hollow_gun.urdf')
VD10_URDF_SHA256 = '975d949007339b063dd0bad11d099cd97a04dd11ec1f1e506acff86a684f9103'
VD10_TCP_POSITION = (0.975, 0.0, 0.1)
VD10_TCP_ROTATION = np.eye(3)  # TCP +X coincides with the physical tube and flange +X.
VD13_URDF_PATH = Path('/home/dong/b2_z1_VD2/source/b2_z1/b2_z1/assets/b2_z1/b2_z1_vd13_mount75.urdf')
VD13_URDF_SHA256 = 'b21de5407bfa1592d217e095eefa13e22af11179a36ec2e1175934529a10ce07'
VD13_TCP_POSITION = (.825, 0., .1)
VD3_TCP_POSITION = (0.10499999999979762, 1.1019615310039722e-07, -0.82)
VD3_TCP_ROTATION = np.array(((3.673205103247453e-06, -4.6928204133545015e-06, 0.9999999999822424),
                             (2.6984871462320583e-11, 0.9999999999889887, 4.692820413287039e-06),
                             (-0.9999999999932537, 9.747179571131628e-12, 3.6732051033336414e-06)))
EXPECTED_JOINT_NAMES = (
    "FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint",
    "FL_thigh_joint", "FR_thigh_joint", "RL_thigh_joint", "RR_thigh_joint",
    "joint1", "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
    "joint2", "joint3", "joint4", "joint5", "joint6",
)
ARM_JOINT_NAMES = tuple("joint%d" % index for index in range(1, 7))
DEFAULT_JOINT_POSITION = {
    **{leg + "_hip_joint": 0.1 if leg.endswith("L") else -0.1
       for leg in ("FL", "FR", "RL", "RR")},
    **{leg + "_thigh_joint": 0.8 if leg.startswith("F") else 1.0
       for leg in ("FL", "FR", "RL", "RR")},
    **{leg + "_calf_joint": -1.5 for leg in ("FL", "FR", "RL", "RR")},
    **dict(zip(ARM_JOINT_NAMES, (0.0, 2.55, -1.6, -1.4, 0.0, 0.0))),
}
TOOL_TCP_GUN_M = (0.0, -0.03, -0.62)
MARKER_CALIBRATION = {
    "marker_radius_m": 0.035,
    "marker_axial_spacing_m": 0.06,
    "marker_tip_extension_m": 0.0,
}
CAMERA_CALIBRATION = {
    "front_rgb": {
        "position_b": (0.39079, 0.0, -0.013433),
        "quaternion_ros_wxyz": (0.49999816, -0.5, 0.50000184, -0.5),
        "fx": 460.0, "fy": 460.0, "width": 640, "height": 480,
        "near_m": 0.07, "far_m": 6.0,
    },
    "rear_d435i": {
        "position_b": (-0.445, 0.0, 0.005),
        "quaternion_ros_wxyz": (0.27059805007309856, -0.6532814824381883,
                                -0.6532814824381882, 0.2705980500730985),
        "fx": 392.5, "fy": 392.5, "width": 640, "height": 480,
        "near_m": 0.10, "far_m": 6.0,
    },
}


def vd3_camera_calibration():
    """Convert the VD3 command cameras' world-frame RPY to ROS optical quats."""
    optical_to_gl = (0., 1., 0., 0.)
    forward_gl = (0.5, 0.5, -0.5, -0.5)
    result = {}
    for name, position, pitch, yaw in (
        ('front_rgb', (0.41251, 0.024997, 0.04765), math.pi/4, 0.),
        ('rear_d435i', (-0.4081, -0.025, 0.024861), math.pi/4, math.pi),
    ):
        gl = _quat_multiply(rpy_quaternion((0., pitch, yaw)), forward_gl)
        ros = _quat_multiply(gl, optical_to_gl)
        result[name] = dict(position_b=position, quaternion_ros_wxyz=tuple(ros),
                            fx=320., fy=320., width=640, height=480, near_m=.05, far_m=5.)
    return result

# MuJoCo's convex collision for a 97 x 95 x .05 m prism produces inaccurate
# sphere penetration depths.  Keep the actual robot workspace six metres wide
# and partition long straight road spans while retaining every curved station.
# This changes only the physical scene extent/tessellation, never perception.
ROAD_HALF_WIDTH_M = 3.0
ROAD_MAX_SEGMENT_LENGTH_M = 1.0
# Retain the central tape texture's 3 mm resolution while extending the plain
# visible ground far beyond the cameras' view. Physical contact stays a plane.
TAPE_VISUAL_HALF_WIDTH_M = 50.0


def _values(text, default=(0.0, 0.0, 0.0)):
    return np.asarray(default if text is None else list(map(float, text.split())), dtype=float)


def _fmt(values):
    return " ".join("%.12g" % value for value in values)


def _quat_multiply(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return np.array((w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
                     w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v))


def rpy_quaternion(rpy):
    r, p, y = np.asarray(rpy, dtype=float) / 2.0
    return _quat_multiply(_quat_multiply(
        (math.cos(y), 0, 0, math.sin(y)), (math.cos(p), 0, math.sin(p), 0)),
        (math.cos(r), math.sin(r), 0, 0))


def quaternion_matrix(quaternion):
    w, x, y, z = np.asarray(quaternion) / np.linalg.norm(quaternion)
    return np.array(((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
                     (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
                     (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y))))


def quaternion_from_matrix(matrix):
    """Return a unit wxyz quaternion for an orthonormal rotation matrix."""
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (3, 3) or not np.isfinite(matrix).all():
        raise ValueError('rotation matrix must be finite 3x3')
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = 2.0 * math.sqrt(trace + 1.0)
        quat = np.array((0.25 * scale, (matrix[2, 1] - matrix[1, 2]) / scale,
                         (matrix[0, 2] - matrix[2, 0]) / scale,
                         (matrix[1, 0] - matrix[0, 1]) / scale))
    else:
        index = int(np.argmax(np.diag(matrix)))
        next_index = (index + 1) % 3
        last_index = (index + 2) % 3
        scale = 2.0 * math.sqrt(max(1.0e-15, 1.0 + matrix[index, index]
                                    - matrix[next_index, next_index] - matrix[last_index, last_index]))
        values = np.zeros(4)
        values[index + 1] = 0.25 * scale
        values[0] = (matrix[last_index, next_index] - matrix[next_index, last_index]) / scale
        values[next_index + 1] = (matrix[next_index, index] + matrix[index, next_index]) / scale
        values[last_index + 1] = (matrix[last_index, index] + matrix[index, last_index]) / scale
        quat = values
    quat /= np.linalg.norm(quat)
    return quat if quat[0] >= 0 else -quat


def _origin(node):
    origin = node.find("origin")
    if origin is None:
        return {"pos": "0 0 0", "quat": "1 0 0 0"}
    return {"pos": _fmt(_values(origin.get("xyz"))),
            "quat": _fmt(rpy_quaternion(_values(origin.get("rpy"))))}


def _dae_to_obj(source, destination):
    """Convert the source's COLLADA visual triangles, retaining scene transforms.

    These upstream DAE files contain triangles or polygon lists in metre/Z-up
    coordinates.  Unsupported primitives/axes are rejected rather than silently
    generating a differently oriented robot mesh.  Normals/materials are visual
    only; MuJoCo recomputes normals and URDF colours are assigned to geoms.
    """
    root = ET.parse(str(source)).getroot()
    ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
    if root.find("c:asset/c:up_axis", ns).text != "Z_UP":
        raise ValueError("Unsupported COLLADA axis: " + str(source))
    unit = root.find("c:asset/c:unit", ns)
    unit_scale = float(unit.get("meter", "1")) if unit is not None else 1.0
    geometries = {}
    for geometry in root.findall("c:library_geometries/c:geometry", ns):
        mesh = geometry.find("c:mesh", ns)
        arrays = {}
        for data_source in mesh.findall("c:source", ns):
            accessor = data_source.find("c:technique_common/c:accessor", ns)
            stride = int(accessor.get("stride", "1"))
            arrays[data_source.get("id")] = np.fromstring(
                data_source.find("c:float_array", ns).text, sep=" ").reshape(-1, stride)
        vertices = {}
        for v in mesh.findall("c:vertices", ns):
            item = next(i for i in v.findall("c:input", ns) if i.get("semantic") == "POSITION")
            vertices[v.get("id")] = arrays[item.get("source")[1:]][:, :3]
        all_vertices, all_faces = [], []
        offset = 0
        for primitive in mesh:
            tag = primitive.tag.rsplit("}", 1)[-1]
            if tag in ("source", "vertices", "extra"):
                continue
            if tag not in ("triangles", "polylist"):
                raise ValueError("Unsupported COLLADA primitive: " + tag)
            inputs = primitive.findall("c:input", ns)
            stride = max(int(i.get("offset", "0")) for i in inputs) + 1
            index = next(i for i in inputs if i.get("semantic") == "VERTEX")
            positions = vertices[index.get("source")[1:]]
            indices = np.fromstring(primitive.find("c:p", ns).text, sep=" ", dtype=int)
            indices = indices.reshape(-1, stride)[:, int(index.get("offset", "0"))]
            counts = (np.full(int(primitive.get("count")), 3) if tag == "triangles"
                      else np.fromstring(primitive.find("c:vcount", ns).text, sep=" ", dtype=int))
            cursor = 0
            for count in counts:
                face = indices[cursor:cursor + count]
                for corner in range(1, count - 1):
                    all_faces.append((offset + face[0], offset + face[corner], offset + face[corner + 1]))
                cursor += count
            if cursor != len(indices):
                raise ValueError("Invalid COLLADA polygon indices")
            all_vertices.append(positions)
            offset += len(positions)
        geometries[geometry.get("id")] = (np.concatenate(all_vertices), np.asarray(all_faces))
    output_vertices, output_faces = [], []
    vertex_offset = 0

    def visit(node, parent_transform):
        nonlocal vertex_offset
        transform = np.eye(4)
        for item in node:
            tag = item.tag.rsplit("}", 1)[-1]
            if tag == "matrix":
                transform = transform @ np.fromstring(item.text, sep=" ").reshape(4, 4)
            elif tag in ("translate", "rotate", "scale", "skew", "lookat"):
                raise ValueError("Unsupported COLLADA node transform: " + tag)
        transform = parent_transform @ transform
        for instance in node.findall("c:instance_geometry", ns):
            verts, faces = geometries[instance.get("url")[1:]]
            output_vertices.append((verts @ transform[:3, :3].T + transform[:3, 3]) * unit_scale)
            output_faces.append(faces + vertex_offset)
            vertex_offset += len(verts)
        for child in node.findall("c:node", ns):
            visit(child, transform)

    scene_reference = root.find("c:scene/c:instance_visual_scene", ns).get("url")[1:]
    scene = next(s for s in root.findall("c:library_visual_scenes/c:visual_scene", ns)
                 if s.get("id") == scene_reference)
    for node in scene.findall("c:node", ns):
        visit(node, np.eye(4))
    if not output_vertices:
        raise ValueError("COLLADA has no instantiated visual geometry")
    with destination.open("w", encoding="utf-8") as handle:
        handle.write("# Converted from the VD1 source COLLADA visual mesh\n")
        for vertex in np.concatenate(output_vertices):
            handle.write("v " + _fmt(vertex) + "\n")
        for face in np.concatenate(output_faces):
            handle.write("f %d %d %d\n" % tuple(face + 1))


def _road_geometry():
    source = TRAINING_ROOT / "scripts/vision/vd1_scene.py"
    spec = importlib.util.spec_from_file_location("vd1_physical_road_geometry", str(source))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _add_road(asset, world, curvature_max):
    """Convex slab pieces preserve the gap in MuJoCo's convex mesh collision.

    A single nonconvex mesh per road half would be replaced by its convex hull
    by MuJoCo, closing the crack.  Each span here is a convex trapezoidal prism
    with exactly the same top boundary as the source VD1 physical mesh.
    """
    source_stations = _road_geometry().road_stations(curvature_max)
    segments = []
    for start, end in zip(source_stations[:-1], source_stations[1:]):
        count = max(1, int(math.ceil((end[0] - start[0]) / ROAD_MAX_SEGMENT_LENGTH_M)))
        segments.append(np.linspace(start, end, count + 1)[:-1])
    stations = np.concatenate(segments + [source_stations[-1:]], axis=0)
    ET.SubElement(world, "geom", name="crack_floor", type="plane", size="100 100 .1",
                  pos="0 0 -.05", rgba=".003 .003 .003 1", contype="1", conaffinity="2",
                  friction="1 .005 .0001", condim="3")
    faces = ((0, 1, 2), (0, 2, 3), (4, 6, 5), (4, 7, 6),
             (0, 4, 5), (0, 5, 1), (1, 5, 6), (1, 6, 2),
             (2, 6, 7), (2, 7, 3), (3, 7, 4), (3, 4, 0))
    for index, ((x0, y0), (x1, y1)) in enumerate(zip(stations[:-1], stations[1:])):
        for sign, side in ((1, "left"), (-1, "right")):
            top = np.array(((x0, y0 + sign*.01, 0), (x1, y1 + sign*.01, 0),
                            (x1, sign*ROAD_HALF_WIDTH_M, 0),
                            (x0, sign*ROAD_HALF_WIDTH_M, 0)))
            name = "road_%s_%03d" % (side, index)
            if abs(y1-y0) < 1e-15:
                inner = y0 + sign*.01
                outer = sign*ROAD_HALF_WIDTH_M
                ET.SubElement(world, "geom", name=name, type="box",
                              pos=_fmt(((x0+x1)/2, (inner+outer)/2, -.025)),
                              size=_fmt(((x1-x0)/2, abs(outer-inner)/2, .025)),
                              rgba=".42 .42 .42 1", contype="1", conaffinity="2", mass="0",
                              friction="1 .005 .0001", condim="3", solref=".005 1")
                continue
            vertices = np.concatenate((top, top + (0, 0, -.05)))
            oriented_faces = np.asarray(faces)
            if sign < 0:
                oriented_faces = oriented_faces[:, (0, 2, 1)]
            ET.SubElement(asset, "mesh", name=name, vertex=_fmt(vertices.ravel()),
                          face=_fmt(oriented_faces.ravel()))
            ET.SubElement(world, "geom", name=name, type="mesh", mesh=name,
                          rgba=".42 .42 .42 1", contype="1", conaffinity="2", mass="0",
                          friction="1 .005 .0001", condim="3", solref=".005 1")


def _add_tape_road(asset, world, curvature_max, cache):
    """Paint a 2 cm black stripe into a flat road's texture, with no tape geom.

    The central texture covers the complete bend at ~3 mm/pixel. Separate
    straight end textures avoid stretching that resolution across 190 metres.
    All visual slabs have their top at z=0; one invisible plane supplies every
    ground contact. The line never changes optical depth or collision height.
    """
    import cv2

    stations = _road_geometry().road_stations(curvature_max)
    height = 2048
    for name, lower, upper, width in (('curve', -2., 26., 8192), ('straight', -95., -2., 16)):
        x = lower+(np.arange(width)+.5)*(upper-lower)/width
        center_y = np.interp(x, stations[:, 0], stations[:, 1])
        dy = 2*ROAD_HALF_WIDTH_M/height
        center_row = (ROAD_HALF_WIDTH_M-center_y)/dy-.5
        image = np.full((height, width), 107, dtype=np.uint8)
        columns = np.arange(width)
        # Pixel coverage keeps the 2 cm width without jagged subpixel edges.
        for offset in range(-5, 7):
            rows = np.floor(center_row).astype(int)+offset
            coverage = np.clip(.5+.01/dy-np.abs(rows-center_row), 0., 1.)
            valid = (rows >= 0) & (rows < height)
            image[rows[valid], columns[valid]] = np.rint(107-106*coverage[valid]).astype(np.uint8)
        path = cache/('black_tape_'+name+'.png')
        if not cv2.imwrite(str(path), image):
            raise OSError('Cannot write road tape texture: '+str(path))
        ET.SubElement(asset, 'texture', name='tape_'+name, type='2d', file=str(path))
        ET.SubElement(asset, 'material', name='tape_'+name, texture='tape_'+name,
                      texuniform='false', texrepeat='1 1', specular='0', reflectance='0')
    ET.SubElement(world, 'geom', name='tape_ground_contact', type='plane', size='100 100 .1',
                  pos='0 0 0', rgba='0 0 0 0', group='3', contype='1', conaffinity='2',
                  friction='1 .005 .0001', condim='3', solref='.005 1', mass='0')
    for name, lower, upper, texture in (('start', -95., -2., 'straight'),
                                       ('bend', -2., 26., 'curve'),
                                       ('end', 26., 95., 'straight')):
        ET.SubElement(world, 'geom', name='tape_road_'+name, type='box',
                      pos=_fmt(((lower+upper)/2, 0., -.025)),
                      size=_fmt(((upper-lower)/2, ROAD_HALF_WIDTH_M, .025)),
                      material='tape_'+texture, group='2', contype='0', conaffinity='0', mass='0')
    # Use the same texture colour pipeline as the central road. A bare rgba
    # material has a different rendered brightness under the scene lighting.
    gray_path = cache/'tape_ground_gray.png'
    if not cv2.imwrite(str(gray_path), np.full((1, 1), 107, dtype=np.uint8)):
        raise OSError('Cannot write plain ground texture: '+str(gray_path))
    ET.SubElement(asset, 'texture', name='tape_ground_gray', type='2d', file=str(gray_path))
    ET.SubElement(asset, 'material', name='tape_ground_gray', texture='tape_ground_gray',
                  texuniform='false', texrepeat='1 1', specular='0', reflectance='0')
    # Add adjacent slabs instead of stretching the tape texture over 100 m.
    # Their tops and colour match the central strip exactly; no overlap or lip.
    side_half_width = (TAPE_VISUAL_HALF_WIDTH_M-ROAD_HALF_WIDTH_M)/2
    side_center = (TAPE_VISUAL_HALF_WIDTH_M+ROAD_HALF_WIDTH_M)/2
    for side in (-1, 1):
        ET.SubElement(world, 'geom', name='tape_ground_outer_%d' % side, type='box',
                      pos=_fmt((0., side*side_center, -.025)),
                      size=_fmt((95., side_half_width, .025)), material='tape_ground_gray',
                      group='2', contype='0', conaffinity='0', mass='0')


def build_scene(curvature_max, cache_dir, timestep=0.005, tool_markers=True,
                road_surface='crack', gun_mass_mode='original', robot_version='vd1',
                camera_calibration=None, odometry_sensors=False, light_background=False):
    """Return a generated, standalone MJCF path, without touching existing assets.

    Leg actuator controls are torques; Z1 controls are joint-position targets.
    The Z1 position actuators supply the original 160/5 implicit PD behavior.
    The runner applies the original leg explicit PD at 200 Hz.  Both kinds are
    named ``<joint>_motor`` and their controls must not be confused.
    """
    if not (math.isfinite(timestep) and timestep > 0):
        raise ValueError("MuJoCo timestep must be positive and finite")
    if road_surface not in ('crack', 'tape'):
        raise ValueError('Unknown road surface: '+str(road_surface))
    if gun_mass_mode not in ('original', 'zero'):
        raise ValueError('Unknown gun mass mode: '+str(gun_mass_mode))
    if robot_version not in ('vd1', 'vd3', 'vd10', 'vd13'):
        raise ValueError('Unknown robot version: ' + str(robot_version))
    urdf_path = (VD13_URDF_PATH if robot_version == 'vd13' else VD10_URDF_PATH if robot_version == 'vd10' else
                 VD3_URDF_PATH if robot_version == 'vd3' else URDF_PATH)
    expected_sha = (VD13_URDF_SHA256 if robot_version == 'vd13' else VD10_URDF_SHA256 if robot_version == 'vd10' else
                    VD3_URDF_SHA256 if robot_version == 'vd3' else URDF_SHA256)
    if hashlib.sha256(urdf_path.read_bytes()).hexdigest() != expected_sha:
        raise ValueError(robot_version.upper() + ' URDF changed; verify the actor asset contract first')
    if robot_version in ('vd3', 'vd10', 'vd13') and gun_mass_mode != 'original':
        raise ValueError('Non-VD1 gun mass is defined in its separate URDF asset')
    cache = Path(cache_dir).expanduser().resolve()
    cache.mkdir(parents=True, exist_ok=True)
    robot = ET.parse(str(urdf_path)).getroot()
    links = {link.get("name"): link for link in robot.findall("link")}
    children = {}
    for joint in robot.findall("joint"):
        children.setdefault(joint.find("parent").get("link"), []).append(joint)
    root = ET.Element("mujoco", model=("VD13_B2_Z1_mount75" if robot_version == 'vd13' else
                                      "VD10_B2_Z1_hollow_gun" if robot_version == 'vd10' else
                                      "VD3_B2_Z1_zero_load_gun" if robot_version == 'vd3'
                                      else "VD1_B2_Z1_zero_gun" if gun_mass_mode == 'zero'
                                      else "VD1_B2_Z1_original_heavy_gun"))
    ET.SubElement(root, "compiler", angle="radian", autolimits="true", fusestatic="false",
                  inertiafromgeom="false")
    ET.SubElement(root, "option", timestep=str(timestep), integrator="implicitfast",
                  cone="elliptic", impratio="100", iterations="100")
    # MuJoCo's visual clipping distances are multiplied by model.stat.extent.
    # The 190 m road must not push the camera's near plane several metres away.
    ET.SubElement(root, "statistic", extent="2", center="0 0 .7")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth="640", offheight="480")
    ET.SubElement(visual, "map", znear=".035", zfar="3")
    ET.SubElement(visual, "quality", shadowsize="0")
    ET.SubElement(visual, "headlight", ambient=".6 .6 .6", diffuse=".5 .5 .5",
                  specular="0 0 0")
    defaults = ET.SubElement(root, "default")
    ET.SubElement(defaults, "joint", damping="0", armature="0", frictionloss=".01")
    ET.SubElement(defaults, "geom", density="0", mass="0", friction="1 .005 .0001")
    asset = ET.SubElement(root, "asset")
    if light_background:
        # A spectator-only backdrop asset, not a wall/geom or collision object.
        # Camera capture explicitly disables mjRND_SKYBOX to preserve original
        # robot RGB/depth inputs; the passive viewer keeps it visible.
        ET.SubElement(asset, 'texture', name='viewer_light_background', type='skybox',
                      builtin='gradient', rgb1='.78 .81 .85', rgb2='.25 .38 .54',
                      width='32', height='192')
    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", name="overhead", pos="0 0 10", dir="0 0 -1",
                  directional="true", diffuse=".6 .6 .6", ambient=".4 .4 .4", castshadow="false")
    if road_surface == 'tape':
        _add_tape_road(asset, world, curvature_max, cache)
    else:
        _add_road(asset, world, curvature_max)
    cameras = (camera_calibration if camera_calibration is not None else
               vd3_camera_calibration() if robot_version in ('vd3', 'vd10', 'vd13') else CAMERA_CALIBRATION)
    mesh_ids = {}
    joint_order = []

    def shape(parent, element, collision, name):
        geometry = element.find("geometry")
        attributes = {"name": name, **_origin(element), "mass": "0", "density": "0"}
        if collision:
            attributes.update(contype="2", conaffinity="1", group="3", rgba="0 0 0 0")
        else:
            material = element.find("material/color")
            attributes.update(contype="0", conaffinity="0", group="2",
                              rgba=material.get("rgba") if material is not None else ".65 .65 .65 1")
        primitive = next(iter(geometry))
        if primitive.tag == "mesh":
            source = urdf_path.parent / primitive.get("filename")
            key = (str(source), primitive.get("scale", "1 1 1"))
            if key not in mesh_ids:
                mesh_name = "robot_mesh_%02d" % len(mesh_ids)
                converted = source
                if source.suffix.lower() == ".dae":
                    converted = cache / (source.stem + ".obj")
                    _dae_to_obj(source, converted)
                ET.SubElement(asset, "mesh", name=mesh_name, file=str(converted),
                              scale=key[1])
                mesh_ids[key] = mesh_name
            attributes.update(type="mesh", mesh=mesh_ids[key])
        elif primitive.tag == "box":
            attributes.update(type="box", size=_fmt(_values(primitive.get("size")) / 2))
        elif primitive.tag == "cylinder":
            # UnitreeUrdfFileCfg uses replace_cylinders_with_capsules=True.
            # In the actual converted PhysX USD, capsule.height is the original
            # cylinder length and its hemispheres extend by radius at both ends.
            # MuJoCo capsule.size[1] is the corresponding half segment length.
            attributes.update(type="capsule" if collision and robot_version not in ('vd10', 'vd13') else "cylinder",
                              size=_fmt((float(primitive.get("radius")),
                                         float(primitive.get("length"))/2)))
        elif primitive.tag == "sphere":
            attributes.update(type="sphere", size=primitive.get("radius"))
        else:
            raise ValueError("Unsupported URDF geometry " + primitive.tag)
        ET.SubElement(parent, "geom", **attributes)

    def body(parent, link_name, incoming=None):
        attributes = {"name": link_name}
        attributes.update(_origin(incoming) if incoming is not None else {"pos": "0 0 .7"})
        target = ET.SubElement(parent, "body", **attributes)
        if incoming is None:
            ET.SubElement(target, "freejoint", name="floating_base_joint")
        elif incoming.get("type") != "fixed":
            if incoming.get("type") != "revolute":
                raise ValueError("Unsupported joint type " + incoming.get("type"))
            name = incoming.get("name")
            limits = incoming.find("limit")
            ET.SubElement(target, "joint", name=name, type="hinge", axis=incoming.find("axis").get("xyz"),
                          range=limits.get("lower") + " " + limits.get("upper"))
            joint_order.append(name)
        link = links[link_name]
        inertial = link.find("inertial")
        if inertial is not None and float(inertial.find("mass").get("value")) > 0:
            tensor = inertial.find("inertia")
            inertia = np.array(((float(tensor.get("ixx")), float(tensor.get("ixy")), float(tensor.get("ixz"))),
                                (float(tensor.get("ixy")), float(tensor.get("iyy")), float(tensor.get("iyz"))),
                                (float(tensor.get("ixz")), float(tensor.get("iyz")), float(tensor.get("izz")))))
            origin = _origin(inertial)
            rotation = quaternion_matrix(_values(origin["quat"]))
            inertia = rotation @ inertia @ rotation.T
            if link_name == 'link_gun' and gun_mass_mode == 'zero':
                # This fixed body has no independent DOF; MuJoCo accepts exact
                # zero mass/inertia. Geometry and tool_tcp stay attached.
                ET.SubElement(target, 'inertial', pos=origin['pos'], mass='0', diaginertia='0 0 0')
            else:
                ET.SubElement(target, "inertial", pos=origin["pos"], mass=inertial.find("mass").get("value"),
                              fullinertia=_fmt((inertia[0,0], inertia[1,1], inertia[2,2],
                                                inertia[0,1], inertia[0,2], inertia[1,2])))
        for index, visual_element in enumerate(link.findall("visual")):
            shape(target, visual_element, False, "%s_visual_%d" % (link_name, index))
        for index, collision in enumerate(link.findall("collision")):
            shape(target, collision, True, "%s_collision_%d" % (link_name, index))
        if link_name == "base_link":
            ET.SubElement(target, "site", name="imu", pos="0 -.02341 .04927", size=".001",
                          rgba="0 0 0 0")
            for name, calibration in cameras.items():
                ros = np.array(calibration["quaternion_ros_wxyz"])
                ros = ros / np.linalg.norm(ros)
                gl = _quat_multiply(ros, (0, 1, 0, 0))
                attrs = dict(name=name, pos=_fmt(calibration["position_b"]), quat=_fmt(gl))
                if 'intrinsic_matrix' in calibration:
                    width, height = calibration['width'], calibration['height']
                    # MuJoCo principalpixel is an offset from sensor centre:
                    # positive x moves the image left; positive y moves it down.
                    attrs.update(sensorsize=_fmt((width, height)), resolution=_fmt((width, height)),
                                 focalpixel=_fmt((calibration['fx'], calibration['fy'])),
                                 principalpixel=_fmt((width/2-calibration['cx'],
                                                      calibration['cy']-height/2)))
                else:
                    attrs['fovy'] = str(math.degrees(2*math.atan(
                        calibration["height"]/(2*calibration["fy"]))))
                ET.SubElement(target, "camera", **attrs)
            camera_position = cameras['rear_d435i']['position_b']
            ET.SubElement(target, "geom", name="rear_camera_housing", type="box",
                          pos=_fmt(camera_position), size=".018 .045 .012", rgba=".1 .1 .12 1",
                          group="2", contype="0", conaffinity="0", mass="0")
        if odometry_sensors and link_name in ('FL_foot', 'FR_foot', 'RL_foot', 'RR_foot'):
            # A nonphysical sensing volume around the authored 32mm foot sphere.
            # touch measures real engine contacts, not inferred/fabricated contact.
            ET.SubElement(target, 'site', name=link_name+'_load_site', type='sphere',
                          pos='0 0 0', size='.033', rgba='0 0 0 0', group='3')
        if robot_version in ('vd3', 'vd10', 'vd13') and link_name == 'link06':
            tcp_position, tcp_rotation = ((VD13_TCP_POSITION, VD10_TCP_ROTATION) if robot_version == 'vd13' else
                                         (VD10_TCP_POSITION, VD10_TCP_ROTATION) if robot_version == 'vd10'
                                          else (VD3_TCP_POSITION, VD3_TCP_ROTATION))
            ET.SubElement(target, 'site', name='tool_tcp', pos=_fmt(tcp_position),
                          quat=_fmt(quaternion_from_matrix(tcp_rotation)), size='.001', rgba='0 0 0 0')
            if tool_markers:
                ET.SubElement(target, 'geom', name='tool_marker_cyan', type='sphere',
                              pos=_fmt(tcp_position), size='.035', rgba='0 1 1 1',
                              group='2', mass='0', contype='0', conaffinity='0')
                ET.SubElement(target, 'geom', name='tool_marker_magenta', type='sphere',
                              pos=_fmt(np.asarray(tcp_position) + .06*tcp_rotation[:, 0 if robot_version in ('vd10', 'vd13') else 2]),
                              size='.035', rgba='1 0 1 1', group='2', mass='0', contype='0', conaffinity='0')
        if link_name == "link_gun":
            ET.SubElement(target, "site", name="tool_tcp", pos=_fmt(TOOL_TCP_GUN_M), size=".001",
                          rgba="0 0 0 0")
            if tool_markers:
                for name, pos, color in (("cyan", TOOL_TCP_GUN_M, "0 1 1 1"),
                                         ("magenta", (0, -.03, -.56), "1 0 1 1")):
                    ET.SubElement(target, "geom", name="tool_marker_" + name, type="sphere",
                                  pos=_fmt(pos), size=".035", rgba=color, group="2", mass="0",
                                  contype="0", conaffinity="0")
        for joint in children.get(link_name, ()):
            body(target, joint.find("child").get("link"), joint)

    body(world, "base_link")
    if set(joint_order) != set(EXPECTED_JOINT_NAMES):
        raise ValueError("VD1 joint names do not match its 18-action actor")
    actuator = ET.SubElement(root, "actuator")
    sensors = ET.SubElement(root, "sensor")
    ET.SubElement(sensors, "gyro", name="imu_gyro", site="imu")
    ET.SubElement(sensors, "framequat", name="imu_orientation", objtype="site", objname="imu")
    ET.SubElement(sensors, "accelerometer", name="imu_acceleration", site="imu")
    if odometry_sensors:
        for foot in ('FL', 'FR', 'RL', 'RR'):
            ET.SubElement(sensors, 'touch', name=foot+'_foot_load', site=foot+'_foot_load_site')
    for name in joint_order:
        effort = 120 if name in ARM_JOINT_NAMES else 320 if "calf" in name else 200
        attributes = dict(name=name + "_motor", joint=name, forcelimited="true",
                          forcerange="-%d %d" % (effort, effort), ctrllimited="false")
        if name in ARM_JOINT_NAMES:
            ET.SubElement(actuator, "position", kp="160", kv="5", **attributes)
        else:
            ET.SubElement(actuator, "motor", **attributes)
        ET.SubElement(sensors, "jointpos", name=name + "_pos", joint=name)
        ET.SubElement(sensors, "jointvel", name=name + "_vel", joint=name)
    keyframe = ET.SubElement(root, "keyframe")
    ET.SubElement(keyframe, "key", name="vd1_default",
                  qpos=_fmt([0, 0, .7, 1, 0, 0, 0] + [DEFAULT_JOINT_POSITION[n] for n in joint_order]),
                  ctrl=_fmt([DEFAULT_JOINT_POSITION[n] if n in ARM_JOINT_NAMES else 0.0 for n in joint_order]))
    path = cache / ("vd13_scene.xml" if robot_version == 'vd13' else "vd10_scene.xml" if robot_version == 'vd10' else
                    "vd3_scene.xml" if robot_version == 'vd3' else "vd1_scene.xml")
    ET.ElementTree(root).write(str(path), encoding="utf-8", xml_declaration=True)
    return path


def validate_model(model, gun_mass_mode='original'):
    """Validate compiled dynamics; these checks do not feed control inputs."""
    import mujoco
    if gun_mass_mode not in ('original', 'zero'):
        raise ValueError('Unknown gun mass mode: '+str(gun_mass_mode))
    gun_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link_gun")
    gun_mass = 0. if gun_mass_mode == 'zero' else 3.25904698235386
    if gun_id < 0 or not math.isclose(model.body_mass[gun_id], gun_mass, abs_tol=1e-11):
        raise AssertionError("VD1 gun mass differs from the requested " + gun_mass_mode + " mode")
    if model.nq != 25 or model.nv != 24 or model.nu not in (18, 54):
        raise AssertionError("VD1 must have a free base, 18 robot actuators and optionally 36 velocity brakes")
    if model.nu == 54:
        from vd1_mujoco_velocity_solver import validate_native_velocity_brakes
        joint_ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in EXPECTED_JOINT_NAMES]
        limits = [3.1415 if n in ARM_JOINT_NAMES else 14.0 if "calf" in n else 23.0
                  for n in EXPECTED_JOINT_NAMES]
        validate_native_velocity_brakes(model, joint_ids, limits)
    original = ET.parse(str(URDF_PATH)).getroot()
    original_mass = sum(float(link.find("inertial/mass").get("value"))
                        for link in original.findall("link") if link.find("inertial/mass") is not None)
    expected_mass = original_mass - (3.25904698235386 if gun_mass_mode == 'zero' else 0.)
    if not math.isclose(model.body_mass.sum(), expected_mass, abs_tol=1e-8):
        raise AssertionError("Compiled robot mass differs from the requested VD1 mass mode")
    for link in original.findall("link"):
        inertial = link.find("inertial")
        if inertial is None or float(inertial.find("mass").get("value")) == 0:
            continue
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, link.get("name"))
        if body_id < 0:
            raise AssertionError("Missing VD1 body: " + link.get('name'))
        expected_link_mass = (0. if link.get('name') == 'link_gun' and gun_mass_mode == 'zero'
                              else float(inertial.find('mass').get('value')))
        np.testing.assert_allclose(model.body_mass[body_id], expected_link_mass, rtol=0, atol=1e-11)
        tensor = inertial.find("inertia")
        expected = np.array(((float(tensor.get("ixx")), float(tensor.get("ixy")), float(tensor.get("ixz"))),
                             (float(tensor.get("ixy")), float(tensor.get("iyy")), float(tensor.get("iyz"))),
                             (float(tensor.get("ixz")), float(tensor.get("iyz")), float(tensor.get("izz")))))
        origin = _origin(inertial)
        rotation = quaternion_matrix(_values(origin["quat"]))
        expected = rotation @ expected @ rotation.T
        if link.get('name') == 'link_gun' and gun_mass_mode == 'zero':
            expected = np.zeros((3, 3))
        compiled_rotation = quaternion_matrix(model.body_iquat[body_id])
        compiled = compiled_rotation @ np.diag(model.body_inertia[body_id]) @ compiled_rotation.T
        # MuJoCo diagonalises fullinertia with its own iterative eigensolver.
        np.testing.assert_allclose(compiled, expected, rtol=1e-5, atol=1e-8)
        np.testing.assert_allclose(model.body_ipos[body_id], _values(origin["pos"]), atol=1e-11)
    return {"nq": model.nq, "nv": model.nv, "nu": model.nu,
            "gun_mass_mode": gun_mass_mode, "gun_mass_experiment": gun_mass_mode != 'original',
            "robot_mass_kg": float(model.body_mass.sum()),
            "gun_mass_kg": float(model.body_mass[gun_id]), "camera_count": model.ncam}


def validate_vd3_model(model):
    """Check the zero-added-mass VD3 URDF and virtual TCP after compilation."""
    import mujoco
    if model.nq != 25 or model.nv != 24 or model.nu not in (18, 54):
        raise AssertionError('VD3 must have a free base and 18 joints')
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'link_gun') >= 0:
        raise AssertionError('VD3 must not have a separate gun rigid body')
    link = model.body('link06')
    tcp = model.site('tool_tcp')
    if tcp.bodyid != link.id:
        raise AssertionError('VD3 TCP must be attached to link06')
    np.testing.assert_allclose(tcp.pos, VD3_TCP_POSITION, rtol=0, atol=1e-10)
    np.testing.assert_allclose(quaternion_matrix(tcp.quat), VD3_TCP_ROTATION, rtol=0, atol=1e-8)
    if not math.isclose(float(model.body_mass.sum()), 78.9990202, abs_tol=1e-7):
        raise AssertionError('VD3 compiled nominal mass differs from training URDF')
    if not math.isclose(float(link.mass), .2887581, abs_tol=1e-8):
        raise AssertionError('VD3 link06 mass differs from training URDF')
    if not any(model.geom_bodyid[i] == link.id and model.geom_contype[i] != 0
               for i in range(model.ngeom)):
        raise AssertionError('VD3 gun collision must be attached to link06')
    return dict(nq=model.nq, nv=model.nv, nu=model.nu, robot_mass_kg=float(model.body_mass.sum()),
                gun_mass_kg=0.0, gun_mass_mode='zero_added_mass', camera_count=model.ncam,
                tcp_parent_body='link06')


def validate_vd10_model(model):
    """Check the isolated measured-tube candidate without validating old actors."""
    import mujoco
    if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'link_vd10_gun') >= 0:
        raise AssertionError('VD10 gun must not become a separate contact body')
    link_id = model.body('link06').id
    if not math.isclose(float(model.body_mass[link_id]), 3.2187581, abs_tol=1e-8):
        raise AssertionError('VD10 link06 must include the measured 2.93 kg tube')
    if model.nq != 25 or model.nv != 24 or model.nu not in (18, 54):
        raise AssertionError('VD10 must preserve the 18 controlled joints')
    if not math.isclose(float(model.body_mass.sum()), 81.9290202, abs_tol=1e-7):
        raise AssertionError('VD10 must add only the measured tube mass')
    np.testing.assert_allclose(model.body_ipos[link_id],
                               (.38903995318750123, -1.5564863463955246e-05, .09089982091096564), atol=1e-9)
    tcp = model.site('tool_tcp')
    if tcp.bodyid != model.body('link06').id:
        raise AssertionError('VD10 TCP must be attached to link06')
    np.testing.assert_allclose(tcp.pos, VD10_TCP_POSITION, atol=1e-10)
    np.testing.assert_allclose(quaternion_matrix(tcp.quat), VD10_TCP_ROTATION, atol=1e-10)
    collision_ids = [i for i in range(model.ngeom)
                     if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, i) == 'link06_collision_0']
    if len(collision_ids) != 1 or model.geom_type[collision_ids[0]] != mujoco.mjtGeom.mjGEOM_CYLINDER:
        raise AssertionError('VD10 tube collision must remain a 1.1 m outer cylinder')
    np.testing.assert_allclose(model.geom_size[collision_ids[0], :2], (.028, .55), atol=1e-10)
    return dict(nq=model.nq, nv=model.nv, nu=model.nu, robot_mass_kg=float(model.body_mass.sum()),
                gun_mass_kg=2.93, tcp_parent_body='link06', robot_version='vd10_candidate')


def validate_vd13_model(model):
    """Validate the rebuilt mass/COM/full inertia and TCP, not just mesh location."""
    import json
    manifest = json.loads(VD13_URDF_PATH.with_suffix('.manifest.json').read_text())
    link = model.body('link06').id
    if model.nq != 25 or model.nv != 24 or model.nu not in (18, 54):
        raise AssertionError('VD13 articulation dimensions differ')
    np.testing.assert_allclose(model.body_mass.sum(), manifest['nominal_robot_mass_kg'], atol=1e-7)
    np.testing.assert_allclose(model.body_mass[link], manifest['link06_combined_mass_kg'], atol=1e-9)
    np.testing.assert_allclose(model.body_ipos[link], manifest['link06_combined_com_m'], atol=1e-10)
    rotation = quaternion_matrix(model.body_iquat[link])
    tensor = rotation @ np.diag(model.body_inertia[link]) @ rotation.T
    np.testing.assert_allclose(tensor, manifest['link06_combined_inertia_kg_m2'], atol=1e-9)
    np.testing.assert_allclose(model.site('tool_tcp').pos, VD13_TCP_POSITION, atol=1e-10)
    np.testing.assert_allclose(model.geom('link06_collision_0').size[:2], [.028, .55], atol=1e-10)
    return dict(version='VD13', status='passed', gun_mass_kg=2.93,
                nominal_total_mass_kg=float(model.body_mass.sum()), tcp_position_m=list(VD13_TCP_POSITION),
                link06_com_m=model.body_ipos[link].tolist(), link06_inertia=tensor.tolist())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--curvature-max", type=float, default=0.0)
    parser.add_argument("--cache-dir", type=Path, default=Path("/dev/shm/vd1_mujoco_asset_check"))
    arguments = parser.parse_args()
    import mujoco
    output = build_scene(arguments.curvature_max, arguments.cache_dir)
    compiled = mujoco.MjModel.from_xml_path(str(output))
    print(output)
    print(validate_model(compiled))
