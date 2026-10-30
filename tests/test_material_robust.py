"""六色参考补偿与保守拒识：基于合成场景，不代表现场验收。"""

import copy
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT/'pi'))
from robot.vision.material_robust import RobustMaterialDetector, make_reference, add_labelled_color
from robot.vision.legacy.material_vision.core import PALETTE, MaterialDetector
from tools.benchmark_six_colors import calibration, fixture_context, scene, perturb


class MaterialRobustTests(unittest.TestCase):
    def setUp(self):
        self.profile, self.reference = calibration()
        self.detector = RobustMaterialDetector(self.profile, self.reference)

    def test_all_six_colors_under_neutral_global_light_and_color_cast(self):
        for color in range(1, 7):
            for light in ('normal', 'dim', 'bright', 'warm', 'cool', 'blur'):
                frame, _ = scene(color)
                result = self.detector.process(perturb(frame, light), fixture_context())
                with self.subTest(color=color, light=light):
                    self.assertEqual([i['color_id'] for i in result['items']], [color])
                    self.assertFalse(result['confirmation_allowed'])
                    self.assertFalse(result['physical_acceptance'])

    def test_extreme_color_cast_does_not_receive_unbounded_compensation(self):
        frame, _ = scene(1)
        frame = np.rint(frame.astype(np.float32) * [.5, 1., 1.4]).clip(0,255).astype(np.uint8)
        result = self.detector.process(frame, fixture_context())
        self.assertEqual(result['items'], [])
        self.assertTrue(result['reasons'])

    def test_invalid_input_and_reference_roi_fail_explicitly(self):
        with self.assertRaises(ValueError):
            make_reference(None, [0,0,16,16], fixture_context(), 'c'*64)
        reference = copy.deepcopy(self.reference)
        reference['roi'] = [0,0,1,256]
        with self.assertRaises(ValueError):
            RobustMaterialDetector(self.profile, reference)

    def test_rotated_scaled_and_perspective_material_preserves_class(self):
        for color in range(1, 7):
            for scale, angle, perspective in ((.65, 40, False), (1.35, -35, True)):
                frame, _ = scene(color, scale, angle, perspective)
                result = self.detector.process(frame, fixture_context())
                with self.subTest(color=color, scale=scale):
                    self.assertEqual([i['color_id'] for i in result['items']], [color])

    def test_missing_dark_clipped_or_covered_reference_is_unknown_not_empty(self):
        for patch in (0, 255, None):
            frame, _ = scene(1)
            if patch is None:
                frame[32:64, 32:96] = 30
            else:
                frame[32:96, 32:96] = patch
            result = self.detector.process(frame, fixture_context())
            self.assertEqual(result['items'], [])
            self.assertTrue(result['reasons'])
            self.assertTrue(result['uncertain_regions'])

    def test_camera_pose_resolution_or_plane_change_invalidates_reference(self):
        frame, _ = scene(1)
        for key, value in (('camera_id', 'other'), ('pose_revision', 2), ('target_plane', 'other'),
                           ('camera_config_sha256', 'd'*64), ('image_size', [640, 360])):
            context = fixture_context()
            context[key] = value
            result = self.detector.process(frame, context)
            self.assertEqual(result['items'], [])
            self.assertEqual(result['reasons'], ['OBSERVATION_CONTEXT_MISMATCH'])

    def test_blank_white_or_thin_black_line_is_not_black_material(self):
        frame, _ = scene(1)
        frame[140:660, 180:1120] = 230
        self.assertEqual(self.detector.process(frame, fixture_context())['items'], [])
        frame[400:403, 500:800] = PALETTE[5]
        self.assertEqual(self.detector.process(frame, fixture_context())['items'], [])

    def test_black_requires_shape_reference_even_when_color_matches(self):
        profile = copy.deepcopy(self.profile)
        profile.update(shape_reference=None, shape_references=[])
        frame, _ = scene(5)
        result = RobustMaterialDetector(profile, self.reference).process(frame, fixture_context())
        self.assertEqual(result['items'], [])
        self.assertTrue(result['uncertain_regions'])

    def test_connected_blue_cyan_ambiguity_is_not_forced_into_one_color(self):
        frame, _ = scene(3)
        region = frame[360:461, 650:696]
        foreground = np.any(region != 230, axis=2)
        region[foreground] = PALETTE[6]
        result = self.detector.process(frame, fixture_context())
        self.assertEqual(result['items'], [])
        self.assertTrue(result['uncertain_regions'])

    def test_inputs_and_baseline_profiles_are_not_modified(self):
        frame, _ = scene(2)
        before_frame, before_profile = frame.copy(), copy.deepcopy(self.profile)
        self.detector.process(frame, fixture_context())
        np.testing.assert_array_equal(frame, before_frame)
        self.assertEqual(self.profile, before_profile)

    def test_multiview_sample_preserves_provenance_and_original_profile(self):
        frame = np.full((720, 1280, 3), PALETTE[1], np.uint8)
        before = copy.deepcopy(self.profile)
        result = add_labelled_color(self.profile, frame, [100,100,140,140], 1, fixture_context(), 'c'*64)
        self.assertEqual(self.profile, before)
        self.assertEqual(result['colors']['1']['source_sha256'], 'c'*64)
        self.assertEqual(result['colors']['1']['observation_context'], fixture_context())
        with self.assertRaises(ValueError):
            add_labelled_color(self.profile, frame, [100,100,140,140], True, fixture_context(), 'c'*64)

    def test_reference_requires_neutral_uniform_labeled_patch(self):
        frame, _ = scene(1)
        frame[32:96,32:96] = PALETTE[3]
        with self.assertRaises(ValueError):
            make_reference(frame, [32,32,96,96], fixture_context(), 'c'*64)


if __name__ == '__main__':
    unittest.main()
