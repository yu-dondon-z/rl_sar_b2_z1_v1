"""Front projection uses image calibration, prior IMU gravity and fixed height.

No pose/velocity integration, rear depth, or simulator state. The ring retains
the B2 host receipt time; it cannot establish an image exposure timestamp.
"""
import numpy as np
from vd1_front_path_estimator import nominal_front_plane

HISTORY_SIZE = 64


def publish_gravity(history, timestamp, gravity):
    row = np.r_[timestamp, gravity]
    if not np.isfinite(row).all() or row.shape != (4,) or timestamp <= 0:
        raise ValueError('invalid IMU host receipt sample')
    with history.get_lock():
        rows = np.asarray(history[:]).reshape(-1, 4)
        # Re-reading a B2 packet does not create a new IMU sample.
        if np.any(rows[:, 0] == timestamp):
            return
        rows[np.argmin(rows[:, 0])] = row
        history[:] = rows.ravel().tolist()


def front_metadata(meta, history, height_m, max_imu_age_s=.10):
    if history is None:
        raise ValueError('front projection waiting for B2 IMU gravity')
    with history.get_lock():
        rows = np.asarray(history[:]).reshape(-1, 4)
    capture = float(meta['capture_timestamp'])
    eligible = rows[(rows[:, 0] > 0) & (rows[:, 0] <= capture)]
    if not len(eligible):
        raise ValueError('front capture has no preceding B2 IMU sample')
    sample = eligible[np.argmax(eligible[:, 0])]
    age = capture - sample[0]
    if age > max_imu_age_s:
        raise ValueError('front capture IMU gravity is stale')
    plane = nominal_front_plane(sample[1:], height_m)
    return dict(meta, ground_plane_b=plane.tolist(),
                ground_plane_source='imu_gravity_and_configured_nominal_height',
                imu_gravity_b=sample[1:].tolist(), imu_host_receive_timestamp=float(sample[0]),
                imu_to_image_host_receipt_gap_s=float(age), nominal_base_height_m=float(height_m),
                geometry_source='front_rgb_imu_nominal_height_flat_floor', rear_depth_used=False,
                timestamp_limit='host receipt alignment only; exposure alignment is not established')


def process_front(perception, meta, image, rear_result, history, height_m=None):
    """Keep legacy projection available only when the estimator is disabled."""
    try:
        if height_m is None:
            if rear_result.get('ground_plane_b') is None:
                raise ValueError('rear measured ground plane unavailable')
            projected = dict(meta, ground_plane_b=rear_result['ground_plane_b'],
                             ground_plane_source='paired_rear_depth')
        else:
            projected = front_metadata(meta, history, height_m)
        result = perception.process_front(projected, image)
        if height_m is not None:
            result.update({key: projected[key] for key in (
                'geometry_source', 'ground_plane_source', 'imu_gravity_b',
                'imu_host_receive_timestamp', 'imu_to_image_host_receipt_gap_s',
                'nominal_base_height_m', 'rear_depth_used')})
        return result, projected
    except (KeyError, TypeError, ValueError) as error:
        return dict(valid=False, reason='front projection: '+str(error)), meta
