"""Exclude the known gun silhouette using capture-time encoders, never scene truth."""
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array(((0., -z, y), (z, 0., -x), (-y, x, 0.)))
    return np.eye(3) + np.sin(angle)*skew + (1.-np.cos(angle))*(skew @ skew)


def origin(element):
    item = element.find('origin')
    attrs = {} if item is None else item.attrib
    position = np.fromstring(attrs.get('xyz', '0 0 0'), sep=' ')
    roll, pitch, yaw = np.fromstring(attrs.get('rpy', '0 0 0'), sep=' ')
    matrix = rotation((0., 0., 1.), yaw) @ rotation((0., 1., 0.), pitch) @ rotation((1., 0., 0.), roll)
    return position, matrix


class EncoderGunMask:
    """Conservative convex silhouette of the installed gun mesh plus an 8 px rim.

    This is a self-exclusion mask, not a visual TCP measurement. All poses come
    from the six encoders and the fixed URDF mounting geometry. Occluded seam
    pixels are removed, never generated or assigned a substitute depth.
    """

    joint_names = ['joint%d' % i for i in range(1, 7)]

    def __init__(self, urdf_path, margin_px=8):
        self.margin_px = int(margin_px)
        path = Path(urdf_path).resolve()
        robot = ET.parse(str(path)).getroot()
        parents = {joint.find('child').get('link'): joint for joint in robot.findall('joint')}
        chain, child = [], 'link06'
        while child != 'base_link':
            if child not in parents or len(chain) > len(parents):
                raise ValueError('gun link has no URDF chain to base')
            joint = parents[child]
            name = joint.get('name')
            index = self.joint_names.index(name) if name in self.joint_names else None
            if index is None and joint.get('type') != 'fixed':
                raise ValueError('unexpected moving joint in gun chain: ' + str(name))
            axis = joint.find('axis')
            axis = np.fromstring('1 0 0' if axis is None else axis.get('xyz', '1 0 0'), sep=' ')
            chain.append((*origin(joint), axis, index))
            child = joint.find('parent').get('link')
        self.chain = chain[::-1]
        vertices = []
        for visual in robot.find("link[@name='link06']").findall('visual'):
            mesh = visual.find('geometry/mesh')
            if mesh is None or 'gun' not in mesh.get('filename', ''):
                continue
            raw = (path.parent / mesh.get('filename')).read_bytes()
            count = int.from_bytes(raw[80:84], 'little')
            if len(raw) != 84 + 50*count:
                raise ValueError('gun self-mask needs a binary STL mesh')
            dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
            points = np.frombuffer(raw, dtype=dtype, count=count, offset=84)['vertices'].reshape(-1, 3)
            scale = np.fromstring(mesh.get('scale', '1 1 1'), sep=' ')
            position, matrix = origin(visual)
            vertices.append((points*scale) @ matrix.T + position)
        if not vertices:
            raise ValueError('URDF contains no installed gun mesh on link06')
        self.vertices = np.unique(np.vstack(vertices), axis=0)

    def link_pose(self, q):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError('gun self-mask needs six finite measured arm joint angles')
        position, matrix = np.zeros(3), np.eye(3)
        for offset, fixed, axis, index in self.chain:
            position += matrix @ offset
            matrix = matrix @ fixed
            if index is not None:
                matrix = matrix @ rotation(axis, q[index])
        return position, matrix

    def _visible_vertices(self, points_c):
        """Legacy VD5 vertex selection; version-specific clipping can override."""
        return points_c[points_c[:, 2] > .08]

    def project(self, metadata, intrinsic, camera_to_base, shape):
        if metadata.get('arm_joint_names') != self.joint_names:
            raise ValueError('gun self-mask is missing the capture-time arm joint order')
        timestamp = float(metadata.get('arm_encoder_timestamp', float('nan')))
        capture = float(metadata.get('capture_timestamp', float('nan')))
        if not np.isfinite((timestamp, capture)).all() or abs(timestamp-capture) > .04:
            raise ValueError('gun self-mask encoder sample is missing or differs from image by over 40 ms')
        position, matrix = self.link_pose(metadata.get('arm_joint_positions_rad'))
        points_b = self.vertices @ matrix.T + position
        points_c = (points_b-camera_to_base[:3, 3]) @ camera_to_base[:3, :3]
        points_c = self._visible_vertices(points_c)
        mask = np.zeros(shape, np.uint8)
        contour = []
        if len(points_c) >= 3:
            rays = points_c @ intrinsic.T
            uv = rays[:, :2] / rays[:, 2:3]
            uv = np.rint(np.clip(uv, -100000, 100000)).astype(np.int32)
            hull = cv2.convexHull(uv)
            cv2.fillConvexPoly(mask, hull, 255)
            if self.margin_px > 0:
                diameter = 2*self.margin_px+1
                mask = cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (diameter, diameter)))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                contour = cv2.approxPolyDP(max(contours, key=cv2.contourArea), 1.5, True).reshape(-1, 2).tolist()
        return mask > 0, dict(gun_self_mask_source='capture_arm_encoders_and_fixed_urdf_gun_mesh',
                             gun_self_mask_margin_px=self.margin_px,
                             gun_self_mask_area_ratio=float(np.mean(mask > 0)),
                             gun_exclusion_contour_uv=contour)
