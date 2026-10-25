"""核对摘要必须保留独立验证及实扫证据边界，不成为动作许可。"""
import copy
import json
from pathlib import Path
import unittest

from test_lidar_calibration import report
from robot.lidar.calibration import sample, audit_axes
from robot.lidar.review import build_review


def records():
    rows = []
    for index, (label, source, target) in enumerate(zip(
            ('P1复测', 'P2', 'P3', 'P4独立点'),
            ((400, 1900), (400, 1600), (700, 1900), (700, 1600)),
            ((450, 450), (750, 450), (450, 750), (750, 750)))):
        capture = report(str(index))
        for frame in capture['frames']:
            frame['objects'][0].update(x_mm=source[0], y_mm=source[1])
        rows.append(dict(label=label, capture=capture, sample=sample(capture, *target, 5)))
    return rows


class ReviewTests(unittest.TestCase):
    def setUp(self):
        self.installation = json.loads((Path(__file__).resolve().parents[1]/
            'config/lidar_installation.json').read_text(encoding='utf-8'))

    def test_even_supported_axis_hypothesis_does_not_promote_archive(self):
        rows = records(); original = copy.deepcopy(rows)
        review = build_review(self.installation, [.15, 2.25, 0], rows)
        self.assertLess(review['independent_error_m'], 1e-10)
        self.assertEqual(review['diagnostic']['hypotheses'][1]['status'], 'HYPOTHESIS_SUPPORTED')
        self.assertFalse(review['real_map_registered'])
        self.assertFalse(review['auto_execution_ready'])
        self.assertFalse(review['live'])
        self.assertEqual(rows, original)

    def test_holdout_is_not_refitted_and_bad_error_stays_rejected(self):
        rows = records()
        for frame in rows[3]['capture']['frames']:
            frame['objects'][0]['y_mm'] -= 40
        rows[3]['sample'] = sample(rows[3]['capture'], 750, 750, 5)
        review = build_review(self.installation, [.15, 2.25, 0], rows)
        self.assertAlmostEqual(review['independent_error_m'], .04)
        self.assertEqual(review['diagnostic']['hypotheses'][1]['status'], 'REJECTED')
        self.assertFalse(review['applied'])

    def test_mismatched_capture_identity_stop_and_point_order_rejected(self):
        for variant in ('source', 'identity', 'stop', 'order'):
            rows = records()
            if variant == 'source': rows[0]['sample']['source_m'][0] += .01
            if variant == 'identity': rows[3]['capture']['board_identity'] = ['other', 'boot']
            if variant == 'stop': rows[3]['capture']['stop_confirmed'] = False
            if variant == 'order': rows.reverse()
            with self.subTest(variant=variant), self.assertRaises(ValueError):
                build_review(self.installation, [.15, 2.25, 0], rows)


if __name__ == '__main__':
    unittest.main()
