"""Measured board calibration shared by the VD5 renderer and image publisher."""
from pathlib import Path

import numpy as np
import yaml

from vd1_mujoco_scene import quaternion_from_matrix, quaternion_matrix


DEFAULT_CALIBRATION = Path(__file__).resolve().parents[1] / 'config/vd5_measured_cameras.yaml'


def measured_camera_calibration(path=DEFAULT_CALIBRATION):
    path = Path(path).resolve()
    source = yaml.safe_load(path.read_text(encoding='utf-8'))
    result = {}
    for name in ('front_rgb', 'rear_d435i'):
        item = source[name]
        extrinsics = item
        if 'extrinsics_file' in item:
            extrinsics = yaml.safe_load((path.parent / item['extrinsics_file']).read_text(encoding='utf-8'))
        transform = np.asarray(extrinsics['T_base_camera'], dtype=np.float64)
        intrinsic = np.asarray(item['intrinsic_matrix'], dtype=np.float64).copy()
        if (transform.shape != (4, 4) or not np.isfinite(transform).all()
                or not np.allclose(transform[3], (0., 0., 0., 1.))
                or not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-5)
                or np.linalg.det(transform[:3, :3]) < .999):
            raise ValueError('invalid measured optical-to-base transform: ' + name)
        native = np.asarray(item['native_size'], dtype=float)
        render = np.asarray(item['render_size'], dtype=float)
        if (intrinsic.shape != (3, 3) or not np.isfinite(intrinsic).all()
                or np.any(native <= 0) or np.any(render <= 0)):
            raise ValueError('invalid measured image calibration: ' + name)
        intrinsic[:2] *= (render / native)[:, None]
        quat = quaternion_from_matrix(transform[:3, :3])
        # Use the same normalized rotation in both the renderer and metadata.
        transform[:3, :3] = quaternion_matrix(quat)
        result[name] = dict(position_b=transform[:3, 3].tolist(),
                            quaternion_ros_wxyz=quat.tolist(),
                            intrinsic_matrix=intrinsic.tolist(),
                            fx=float(intrinsic[0, 0]), fy=float(intrinsic[1, 1]),
                            cx=float(intrinsic[0, 2]), cy=float(intrinsic[1, 2]),
                            width=int(render[0]), height=int(render[1]), near_m=.08, far_m=4.5,
                            source=item['source'], rectified=True,
                            calibration_quality=extrinsics.get('calibration_quality', 'recorded_robot_calibration'),
                            native_image_size=native.astype(int).tolist())
    return result
