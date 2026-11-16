"""Near-field tape association, not a global crack/metric trajectory extractor."""
import cv2
import numpy as np


class NearTracker:
    def __init__(self):
        self.reference = None
        self.streak = 0

    def detect(self, image, mask, probability):
        if image.shape[:2] != (1080, 1920):
            raise ValueError("Calibrated 1920x1080 stream required")
        size = (640, 360)
        rgb = cv2.resize(image, size, interpolation=cv2.INTER_AREA)
        binary = cv2.resize(mask.astype(np.uint8), size, interpolation=cv2.INTER_NEAREST) > 0
        prob = cv2.resize(probability.astype(np.float32), size)
        gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY).astype(np.float32)
        points, widths, contrasts, confidences = [], [], [], []
        previous = None
        missing = 0
        # ONLY this near strip contributes to steering. A far tile branch,
        # endpoint, reflection or pedestrian cannot become a far red target.
        for v in range(int(.97 * 360), int(.83 * 360) - 1, -2):
            row = binary[v].copy()
            row[:128] = False
            row[512:] = False
            changes = np.diff(np.r_[False, row, False].astype(np.int8))
            starts, ends = np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)
            choices = []
            for a, b in zip(starts, ends):
                width = (b - a) * 3.
                if not 5. <= width <= 80.:
                    continue
                u = .5 * (a + b - 1)
                flank = np.r_[gray[v, max(0, a - 8):max(0, a - 2)], gray[v, min(640, b + 2):min(640, b + 8)]]
                if len(flank) < 4:
                    continue
                background = float(np.median(flank))
                contrast = (background - float(np.median(gray[v, a:b]))) / max(1., background)
                confidence = float(prob[v, int(round(u))])
                if contrast < .25 or confidence < .95:
                    continue
                if previous is not None and abs(u - previous) > 4. * (missing + 1):
                    continue
                score = (-(abs(u - previous)) if previous is not None else width, -abs(u - 320))
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
        if len(points) < 15 or points[0][1] - points[-1][1] < 90.:
            self.streak = 0
            raise ValueError("No continuous, sufficiently wide dark near tape")
        points = np.asarray(points, dtype=np.float64)
        coeff = np.polyfit(points[:, 1], points[:, 0], 1)
        predicted = np.polyval(coeff, points[:, 1])
        rms = float(np.sqrt(np.mean((predicted - points[:, 0]) ** 2)))
        if rms > 6.:
            self.streak = 0
            raise ValueError("Near association is too irregular; refusing steering")
        if self.reference is not None:
            rows = np.linspace(.84 * 1080, .96 * 1080, 12)
            delta = float(np.max(np.abs(np.polyval(coeff, rows) - np.polyval(self.reference, rows))))
            if delta > 12.:
                self.streak = 0
                raise ValueError("Near tape changed; do not switch to another line")
        self.reference = coeff
        self.streak += 1
        v = .90 * 1080
        return dict(near_u=float(np.polyval(coeff, v)), near_v=v,
                    native_width_px=float(np.median(widths)), contrast=float(np.median(contrasts)),
                    confidence=float(np.mean(confidences)), native_fit_rms_px=rms,
                    stable_frames=self.streak, points=points.tolist())
