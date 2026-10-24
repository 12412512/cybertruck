"""Extracted unchanged from the inspected ring_step1/ring_detector.py digit path.
Unknown below similarity .58 or margin .08; rotation templates cover 360 degrees.
"""
from pathlib import Path
import cv2
import numpy as np
UNKNOWN_ID = "UNKNOWN"

def extract_digit(image: np.ndarray, cx: int, cy: int, radius: int) -> np.ndarray | None:
    """Extract the central black digit and normalize it to an 80x80 mask."""
    h, w = image.shape[:2]
    R = max(15, int(radius * 0.53))
    if cx - R < 0 or cy - R < 0 or cx + R > w or cy + R > h:
        return None
    roi = cv2.cvtColor(image[cy - R : cy + R, cx - R : cx + R], cv2.COLOR_BGR2GRAY)
    roi = cv2.GaussianBlur(roi, (3, 3), 0)
    _, binary = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY_INV | cv2.THRESH_OTSU)
    count, labels, stats, centers = cv2.connectedComponentsWithStats(binary)

    options = []
    for i in range(1, count):
        x, y, bw, bh, area = [int(k) for k in stats[i]]
        dist = float(np.linalg.norm(centers[i] - np.array([R, R])))
        if area > R * R * 0.07 and dist < R * 0.32 and max(bw, bh) < R * 1.9:
            options.append((area, i))
    if not options:
        return None

    chosen = max(options)[1]
    digit = np.uint8(labels == chosen) * 255
    contours, _ = cv2.findContours(digit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    x, y, bw, bh = cv2.boundingRect(max(contours, key=cv2.contourArea))
    patch = digit[y : y + bh, x : x + bw]
    factor = 62 / max(bw, bh)
    nw, nh = max(1, round(bw * factor)), max(1, round(bh * factor))
    patch = cv2.resize(patch, (nw, nh), interpolation=cv2.INTER_NEAREST)
    result = np.zeros((80, 80), np.uint8)
    xx, yy = (80 - nw) // 2, (80 - nh) // 2
    result[yy : yy + nh, xx : xx + nw] = patch
    return result


class DigitClassifier:
    """Template classifier with an explicit UNKNOWN reject option.

    The class exposes scores for all three labels so a future HOG+KNN model can
    be dropped in without changing :class:`RingDetector` or the temporal voter.
    """

    def __init__(self, template_dir: Path):
        self.rotations: dict[int, list[np.ndarray]] = {}
        for number in (1, 2, 3):
            path = template_dir / f"templ_{number}.png"
            if not path.exists():
                raise FileNotFoundError(f"缺少数字模板：{path}")
            encoded = np.fromfile(str(path), dtype=np.uint8)
            template = cv2.imdecode(encoded, cv2.IMREAD_GRAYSCALE)
            if template is None:
                raise RuntimeError(f"无法加载数字模板：{path}")
            template = cv2.resize(template, (80, 80), interpolation=cv2.INTER_NEAREST)
            self.rotations[number] = [
                cv2.warpAffine(
                    template,
                    cv2.getRotationMatrix2D((40, 40), angle, 1.0),
                    (80, 80),
                    flags=cv2.INTER_NEAREST,
                )
                > 0
                for angle in range(0, 360, 10)
            ]

    def score(self, digit: np.ndarray | None) -> dict[int, float]:
        if digit is None:
            return {number: 0.0 for number in (1, 2, 3)}
        target = cv2.resize(digit, (80, 80), interpolation=cv2.INTER_NEAREST) > 0
        scores: dict[int, float] = {}
        for number, rotations in self.rotations.items():
            best = 0.0
            for rotation in rotations:
                for dx, dy in ((0, 0), (-2, 0), (2, 0), (0, -2), (0, 2)):
                    shifted = np.roll(rotation, shift=(dy, dx), axis=(0, 1))
                    overlap = np.count_nonzero(shifted & target)
                    union = np.count_nonzero(shifted | target)
                    best = max(best, overlap / max(1, union))
            scores[number] = float(best)
        return scores

    def predict_detail(self, digit: np.ndarray | None) -> tuple[str, float, float, dict[int, float]]:
        scores = self.score(digit)
        ranked = sorted(scores.items(), key=lambda pair: pair[1], reverse=True)
        (predicted, best), (_, second) = ranked[:2]
        margin = float(best - second)
        if best < 0.58 or margin < 0.08:
            return UNKNOWN_ID, float(best), margin, scores
        return str(predicted), float(best), margin, scores

    def predict(self, digit: np.ndarray | None) -> tuple[str, float]:
        """Backward-compatible two-value API with UNKNOWN instead of ``?``."""
        label, similarity, _, _ = self.predict_detail(digit)
        return label, similarity
