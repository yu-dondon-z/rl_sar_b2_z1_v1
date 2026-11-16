"""Background-only image registration, not odometry or robot motion authority."""
import cv2
import numpy as np


def predict_line(previous, current, previous_dark, coeff):
    """Warp the old near-line using independently tracked floor texture.

    Work at 640x360. Never estimate motion from the tape itself. Reject poor,
    spatially degenerate or excessive motion rather than opening a wide gate.
    """
    mask = np.zeros(previous.shape, np.uint8)
    mask[172:350, 55:585] = 255
    excluded = cv2.dilate(previous_dark.astype(np.uint8), np.ones((9, 9), np.uint8))
    mask[excluded > 0] = 0
    points = cv2.goodFeaturesToTrack(previous, 300, .01, 7, mask=mask)
    if points is None or len(points) < 24:
        return None
    following, status, _ = cv2.calcOpticalFlowPyrLK(previous, current, points, None,
        winSize=(21, 21), maxLevel=3)
    if following is None or status is None:
        return None
    back, reverse_status, _ = cv2.calcOpticalFlowPyrLK(current, previous, following, None,
        winSize=(21, 21), maxLevel=3)
    if back is None or reverse_status is None:
        return None
    valid = status.ravel().astype(bool) & reverse_status.ravel().astype(bool)
    valid &= np.linalg.norm(points.reshape(-1, 2)-back.reshape(-1, 2), axis=1) < 1.
    a, b = points.reshape(-1, 2)[valid], following.reshape(-1, 2)[valid]
    if len(a) < 24 or not np.isfinite(b).all():
        return None
    transform, inliers = cv2.findHomography(a, b, cv2.RANSAC, 1.5)
    if transform is None or inliers is None or not np.isfinite(transform).all():
        return None
    accepted = inliers.ravel().astype(bool)
    a, b = a[accepted], b[accepted]
    if len(a) < 20 or accepted.mean() < .65 or np.ptp(a[:, 0]) < 180. or np.ptp(a[:, 1]) < 45.:
        return None
    cells = set(zip((a[:, 0]//130).astype(int), (a[:, 1]//55).astype(int)))
    if len(cells) < 6:
        return None
    projected = cv2.perspectiveTransform(a[None].astype(np.float32), transform)[0]
    residual = float(np.median(np.linalg.norm(projected-b, axis=1)))
    determinant = float(np.linalg.det(transform[:2, :2]))
    if residual > .9 or not .80 < determinant < 1.25:
        return None
    corners = np.array([[[128., 274.], [512., 274.], [128., 324.], [512., 324.]]], np.float32)
    warped_corners = cv2.perspectiveTransform(corners, transform)
    displacement = float(np.max(np.linalg.norm(warped_corners-corners, axis=2))*3.)
    if not np.isfinite(displacement) or displacement > 75.:
        return None
    rows = np.linspace(885., 1050., 20)
    old_line = np.c_[np.polyval(coeff, rows), rows]/3.
    warped = cv2.perspectiveTransform(old_line[None].astype(np.float32), transform)[0]*3.
    if not np.isfinite(warped).all() or np.ptp(warped[:, 1]) < 90.:
        return None
    return dict(coeff=np.polyfit(warped[:, 1], warped[:, 0], 1), inliers=len(a),
                residual_native_px=residual*3., displacement_native_px=displacement)
