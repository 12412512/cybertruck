"""压缩模板必须与旧逐模板IoU完全等价，包括拒识和边缘环绕。"""

import unittest
from pathlib import Path

import cv2
import numpy as np

from robot.vision.legacy.ring_vision.digits import DigitClassifier


def legacy_score(classifier, digit):
    if digit is None:
        return dict.fromkeys((1, 2, 3), 0.0)
    target = cv2.resize(digit, (80, 80), interpolation=cv2.INTER_NEAREST) > 0
    scores = {}
    for number, rotations in classifier.rotations.items():
        best = 0.0
        for rotation in rotations:
            for dx, dy in ((0, 0), (-2, 0), (2, 0), (0, -2), (0, 2)):
                shifted = np.roll(rotation, (dy, dx), axis=(0, 1))
                overlap = np.count_nonzero(shifted & target)
                union = np.count_nonzero(shifted | target)
                best = max(best, overlap / max(1, union))
        scores[number] = float(best)
    return scores


class RingDigitPerformanceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        templates = Path(__file__).resolve().parents[1] / 'pi/robot/vision/legacy/ring_vision/templates'
        cls.classifier = DigitClassifier(templates)

    def test_rotated_shifted_and_noisy_masks_match_legacy_exactly(self):
        rng = np.random.default_rng(7)
        for number in (1, 2, 3):
            for rotation_index in (0, 5, 11, 19, 27, 35):
                mask = np.roll(self.classifier.rotations[number][rotation_index], (2, -2), axis=(0, 1))
                noisy = np.logical_xor(mask, rng.random((80, 80)) < .03)
                with self.subTest(number=number, rotation=rotation_index):
                    self.assertEqual(self.classifier.score(np.uint8(noisy)*255),
                                     legacy_score(self.classifier, np.uint8(noisy)*255))

    def test_empty_full_edge_and_resized_inputs_match_legacy(self):
        for mask in (None, np.zeros((80, 80), np.uint8), np.full((80, 80), 255, np.uint8),
                     np.eye(80, dtype=np.uint8)*255, np.eye(40, dtype=np.uint8)*255):
            self.assertEqual(self.classifier.score(mask), legacy_score(self.classifier, mask))

    def test_unknown_rejection_unchanged(self):
        for mask in (None, np.zeros((80, 80), np.uint8), np.full((80, 80), 255, np.uint8)):
            self.assertEqual(self.classifier.predict_detail(mask)[0], 'UNKNOWN')


if __name__ == '__main__':
    unittest.main()
