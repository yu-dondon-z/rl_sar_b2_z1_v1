"""Near-field tape association, not a global crack/metric trajectory extractor."""
import cv2
import numpy as np
import time
from camera_motion import predict_line


class NearTracker:
    # Native-pixel limits for the black tape experiment. Real archived tape
    # was 117--144 px wide; do not confuse a network fragment with its RGB
    # physical region. This does not change any contact/motor/stop threshold.
    MIN_WIDTH = 12.
    MAX_WIDTH = 180.

    def __init__(self):
        self.reference = None
        self.streak = 0
        self.debug = {}
        self.reference_image = None
        self.reference_dark = None
        self.reference_stamp = None

    def detect(self, image, mask, probability, timestamp_ns=None):
        if image.shape[:2] != (1080, 1920):
            raise ValueError("Calibrated 1920x1080 stream required")
        self.debug = dict(rgb_regions=0,width_rejections=0,signal_rejections=0,
                          continuity_rejections=0,accepted_rows=0,max_rgb_width_px=0.)
        size = (640, 360)
        stamp = time.monotonic_ns() if timestamp_ns is None else timestamp_ns
        rgb = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
        prob = cv2.resize(probability.astype(np.float32), size)
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY).astype(np.float32)
        # Recover RGB tape boundaries, then require BROAD neural support.
        # A glare stripe can split the neural mask into two competing edges;
        # following either fragment makes the estimated centre jump sideways.
        background = cv2.morphologyEx(gray, cv2.MORPH_CLOSE, np.ones((1,91),np.uint8))
        dark = (background-gray)/np.maximum(background,1.) >= .25
        # Join only short photometric gaps (<=18 native px). Low network
        # coverage or a thin grout line is still rejected below.
        dark = cv2.morphologyEx(dark.astype(np.uint8),cv2.MORPH_CLOSE,np.ones((1,7),np.uint8)) > 0
        self.frame_fields = gray,prob,dark
        expected = self.reference
        motion = None
        if (self.reference is not None and self.reference_image is not None and
                self.reference_stamp < stamp <= self.reference_stamp+500_000_000):
            motion = predict_line(self.reference_image, gray.astype(np.uint8), self.reference_dark, self.reference)
            if motion is not None:
                expected = motion['coeff']
        self.debug.update(motion_compensation_valid=motion is not None,
                          motion_inliers=motion['inliers'] if motion else 0,
                          motion_displacement_px=motion['displacement_native_px'] if motion else 0.)
        points, widths, contrasts, confidences = [], [], [], []
        previous = None
        missing = 0
        # ONLY this near strip contributes to steering. A far tile branch,
        # endpoint, reflection or pedestrian cannot become a far red target.
        for v in range(int(.97 * 360), int(.77 * 360) - 1, -2):
            row = dark[v].copy()
            row[:128] = False
            row[512:] = False
            changes = np.diff(np.r_[False, row, False].astype(np.int8))
            starts, ends = np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)
            choices = []
            for a, b in zip(starts, ends):
                width = (b - a) * 3.
                self.debug['rgb_regions'] += 1
                self.debug['max_rgb_width_px'] = max(self.debug['max_rgb_width_px'],width)
                if not self.MIN_WIDTH <= width <= self.MAX_WIDTH:
                    self.debug['width_rejections'] += 1
                    continue
                u = .5 * (a + b - 1)
                flank = np.r_[gray[v, max(0, a - 8):max(0, a - 2)], gray[v, min(640, b + 2):min(640, b + 8)]]
                if len(flank) < 4:
                    continue
                background = float(np.median(flank))
                contrast = (background - float(np.median(gray[v, a:b]))) / max(1., background)
                confidence = float(np.median(prob[v,a:b]))
                coverage = float(np.mean(prob[v,a:b] >= .90))
                if contrast < .25 or confidence < .95 or coverage < .60:
                    self.debug['signal_rejections'] += 1
                    continue
                if previous is not None and abs(u - previous) > 4. * (missing + 1):
                    self.debug['continuity_rejections'] += 1
                    continue
                predicted_u = float(np.polyval(expected, v*3.))/3. if expected is not None else None
                score = (-(abs(u-previous)) if previous is not None else
                         -abs(u-predicted_u) if predicted_u is not None else width, -abs(u-320))
                choices.append((score, u, width, contrast, confidence))
            if not choices:
                missing += 1
                if previous is not None and missing > 3:
                    break
                continue
            _, u, width, contrast, confidence = max(choices)
            previous, missing = u, 0
            points.append([u * 3., v * 3.])
            widths.append(width)
            contrasts.append(contrast)
            confidences.append(confidence)
            self.debug['accepted_rows'] = len(points)
            # Stay within the nearest 120 px supported prefix. The extended
            # search handles a visible tape tip without steering from a far
            # branch. Do not use the whole newly visible floor region.
            if points[0][1]-points[-1][1] >= 120.:
                break
        if len(points) < 15 or points[0][1] - points[-1][1] < 90.:
            self.streak = 0
            raise ValueError("No continuous, sufficiently wide dark near tape")
        points = np.asarray(points, dtype=np.float64)
        self.debug.update(supported_near_v_max=float(points[0,1]),supported_near_v_min=float(points[-1,1]))
        v = .90 * 1080
        if v < points[-1,1]-6. or v > points[0,1]+6.:
            self.streak = 0
            raise ValueError("Tape does not cover the calibrated near anchor row")
        coeff = np.polyfit(points[:, 1], points[:, 0], 1)
        predicted = np.polyval(coeff, points[:, 1])
        rms = float(np.sqrt(np.mean((predicted - points[:, 0]) ** 2)))
        if rms > 6.:
            self.streak = 0
            raise ValueError("Near association is too irregular; refusing steering")
        if self.reference is not None:
            rows = np.linspace(max(points[-1,1],v-30.),min(points[0,1],v+30.),12)
            delta = float(np.max(np.abs(np.polyval(coeff, rows) - np.polyval(expected, rows))))
            self.debug['association_delta_px'] = delta
            # Real walking changes perspective even after background warping.
            # Use the physical RGB tape width to scale identity tolerance;
            # still reject a jump to a distant line (not a confidence bypass).
            width = float(np.median(widths))
            limit = min(60., max(30., .75*width)) if motion is not None else min(48., max(24., .5*width))
            self.debug['association_limit_px'] = limit
            if delta > limit:
                self.streak = 0
                raise ValueError("Near tape changed; do not switch to another line")
        self.reference = coeff
        self.reference_image = gray.astype(np.uint8)
        self.reference_dark = dark.copy()
        self.reference_stamp = stamp
        self.streak += 1
        return dict(near_u=float(np.polyval(coeff, v)), near_v=v,
                    native_width_px=float(np.median(widths)), contrast=float(np.median(contrasts)),
                    confidence=float(np.mean(confidences)), native_fit_rms_px=rms,
                    neural_support_method='median_over_rgb_tape_region',
                    stable_frames=self.streak, points=points.tolist(),
                    motion_compensation_valid=motion is not None,
                    motion_inliers=motion['inliers'] if motion else 0,
                    association_delta_px=self.debug.get('association_delta_px', 0.))
