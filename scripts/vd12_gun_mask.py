"""VD12 encoder-only gun exclusion with proper camera near-plane clipping."""
import numpy as np

from vd5_gun_mask import EncoderGunMask


def clip_convex_vertices(points_c, near_m=.08):
    """Vertices sufficient for the near-clipped convex hull of a fixed mesh.

    Keeping only front-facing original vertices loses the tube sides when
    the long tube crosses the camera's near plane. Cross-plane intersections
    of all vertex pairs contain the clipped convex hull's edge intersections;
    the extra points remain inside that hull and cannot enlarge it. These
    are self-exclusion geometry, never new scene/seam measurements.
    """
    points = np.asarray(points_c, dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all()
            or not np.isfinite(near_m) or near_m <= 0.):
        raise ValueError('near clipping needs finite camera xyz vertices and positive depth')
    front = points[points[:, 2] >= near_m]
    back = points[points[:, 2] < near_m]
    if not len(front) or not len(back):
        return front.copy()
    delta = front[None, :, :] - back[:, None, :]
    fraction = (near_m - back[:, None, 2]) / delta[:, :, 2]
    intersections = back[:, None, :] + fraction[:, :, None]*delta
    intersections[:, :, 2] = near_m
    return np.vstack((front, intersections.reshape(-1, 3)))


class NearClippedEncoderGunMask(EncoderGunMask):
    """Same fixed gun and capture encoders; only convex near clipping differs."""

    def __init__(self, urdf_path, margin_px=8, near_m=.08):
        super().__init__(urdf_path, margin_px=margin_px)
        self.near_m = float(near_m)
        if not np.isfinite(self.near_m) or self.near_m <= 0.:
            raise ValueError('gun mask near plane must be finite and positive')

    def _visible_vertices(self, points_c):
        return clip_convex_vertices(points_c, self.near_m)

    def project(self, metadata, intrinsic, camera_to_base, shape):
        mask, diagnostics = super().project(metadata, intrinsic, camera_to_base, shape)
        diagnostics.update(gun_self_mask_near_plane_clipped=True, gun_self_mask_near_plane_m=self.near_m)
        return mask, diagnostics
