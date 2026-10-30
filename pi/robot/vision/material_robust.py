"""六色识别离线候选：同帧灰参考补偿，复用原LAB/轮廓/歧义保护；默认入口不变。"""

import copy
import hashlib
import json

import numpy as np

from .legacy.material_vision.core import MaterialDetector, sample_color, merge_color_sample, validate_profile
from .legacy.placement_vision.core import cleaned
from .task_evidence import observation_context, same_json, sha256_text


def image_patch(frame, roi):
    if not isinstance(frame, np.ndarray) or frame.dtype != np.uint8 or frame.ndim != 3 or frame.shape[2] != 3:
        raise ValueError('BGR_UINT8_IMAGE_REQUIRED')
    if not isinstance(roi, list) or len(roi) != 4 or any(type(v) is not int for v in roi):
        raise ValueError('INTEGER_REFERENCE_ROI_REQUIRED')
    x1, y1, x2, y2 = roi
    if not (0 <= x1 < x2 <= frame.shape[1] and 0 <= y1 < y2 <= frame.shape[0]) or min(x2-x1, y2-y1) < 16:
        raise ValueError('REFERENCE_ROI_INVALID')
    return frame[y1:y2, x1:x2]


def reference_median(patch):
    # 只使用灰块内区，避免模糊/插值把背景混入边缘；内区纹理仍需拒绝。
    margin = max(2, min(patch.shape[:2]) // 8)
    patch = patch[margin:-margin, margin:-margin]
    pixels = patch.reshape(-1, 3).astype(np.float32)
    valid = np.all((pixels >= 24) & (pixels <= 240), axis=1)
    if np.mean(valid) < .95:
        raise ValueError('REFERENCE_DARK_OR_CLIPPED')
    pixels = pixels[valid]
    median = np.median(pixels, axis=0)
    # 灰参考被物料/纹理/阴影边缘覆盖时，停止补偿而非猜测背景颜色。
    if np.max(np.quantile(np.abs(pixels - median), .9, axis=0)) > 8:
        raise ValueError('REFERENCE_NONUNIFORM')
    return median


def make_reference(frame, roi, context, source_sha256):
    """由人工标注的基准灰块建立参考，不能自动取全图均值当作灰色。"""
    context = observation_context(context)
    patch = image_patch(frame, roi)
    if list(frame.shape[:2][::-1]) != context['image_size'] or not sha256_text(source_sha256):
        raise ValueError('REFERENCE_CONTEXT_OR_SOURCE_INVALID')
    median = reference_median(patch)
    if float(np.ptp(median) / np.mean(median)) > .10:
        raise ValueError('BASELINE_NEUTRAL_PATCH_REQUIRED')
    return dict(schema_version=1, context=context, roi=list(roi), bgr=median.tolist(),
                source_sha256=source_sha256, source_kind='labelled_neutral_reference')


def add_labelled_color(profile, frame, roi, color, context, source_sha256):
    """独立增加带来源的多视角表面样本；不修改原配置或按增强图自动宣称已标定。"""
    context = observation_context(context)
    image_patch(frame, roi)
    if type(color) is not int or color not in range(1, 7) or not sha256_text(source_sha256):
        raise ValueError('LABEL_OR_SOURCE_INVALID')
    if list(frame.shape[:2][::-1]) != context['image_size'] or context['image_size'] != profile['resolution']:
        raise ValueError('SAMPLE_RESOLUTION_MISMATCH')
    result = copy.deepcopy(profile)
    sample = sample_color(frame, roi, color)
    sample.update(source_sha256=source_sha256, observation_context=context)
    result['colors'][str(color)] = merge_color_sample(result['colors'][str(color)], sample)
    return validate_profile(result)


class RobustMaterialDetector:
    """可选纯图像入口。色彩补偿不授予运动或成功确认，缺参考不降级成空槽。"""

    def __init__(self, profile, reference):
        self.profile = validate_profile(copy.deepcopy(profile))
        self.reference = copy.deepcopy(reference)
        if reference.get('schema_version') != 1 or reference.get('source_kind') != 'labelled_neutral_reference':
            raise ValueError('LABELLED_REFERENCE_REQUIRED')
        observation_context(reference['context'])
        if reference['context']['image_size'] != profile['resolution'] or not sha256_text(reference.get('source_sha256')):
            raise ValueError('REFERENCE_PROFILE_MISMATCH')
        bgr = np.asarray(reference.get('bgr'), dtype=float)
        if (bgr.shape != (3,) or not np.isfinite(bgr).all() or np.any((bgr < 24) | (bgr > 240)) or
                float(np.ptp(bgr) / np.mean(bgr)) > .10):
            raise ValueError('REFERENCE_BGR_INVALID')
        roi = reference.get('roi')
        width, height = profile['resolution']
        if (not isinstance(roi, list) or len(roi) != 4 or any(type(v) is not int for v in roi) or
                not (0 <= roi[0] < roi[2] <= width and 0 <= roi[1] < roi[3] <= height) or
                min(roi[2]-roi[0], roi[3]-roi[1]) < 16):
            raise ValueError('REFERENCE_ROI_INVALID')
        self.detector = MaterialDetector(profile)
        self.profile_sha256 = hashlib.sha256(json.dumps(dict(profile=profile, reference=reference),
                                                        sort_keys=True, allow_nan=False).encode()).hexdigest()

    def process(self, frame, context):
        result = dict(domain='offline_candidate', items=[], uncertain_regions=[], rejected_regions=[],
                      reasons=[], profile_sha256=self.profile_sha256, coordinate_frame='image_pixels',
                      bbox_format='xywh', metric_valid=False, motion_authority=False,
                      material_state='UNKNOWN', confirmation_allowed=False, physical_acceptance=False)
        try:
            if not same_json(context, self.reference['context']):
                raise ValueError('OBSERVATION_CONTEXT_MISMATCH')
            patch = image_patch(frame, self.reference['roi'])
            if list(frame.shape[:2][::-1]) != self.profile['resolution']:
                raise ValueError('RESOLUTION_MISMATCH')
            observed = reference_median(patch)
            gains = np.asarray(self.reference['bgr'], np.float32) / observed
            if np.any((gains < .5) | (gains > 2.0)) or float(np.max(gains) / np.min(gains)) > 1.65:
                raise ValueError('ILLUMINATION_OUTSIDE_REFERENCE_RANGE')
            corrected = np.clip(np.rint(frame.astype(np.float32) * gains), 0, 255).astype(np.uint8)
            candidates = self.detector.detect(corrected)
            accepted = []
            rejected = copy.deepcopy(self.detector.rejected_regions)
            uncertain = copy.deepcopy(self.detector.uncertain_regions)
            for item in cleaned(candidates):
                if (not item['color_calibrated'] or item['quality_score'] < self.profile['min_score'] or
                        item['clipped'] or (item['color_id'] == 5 and not item['shape_consistent'])):
                    uncertain.append(dict(item, rejection_reason='QUALITY_SHAPE_OR_BOUNDARY_UNVERIFIED'))
                else:
                    accepted.append(item)
            result.update(items=accepted, uncertain_regions=uncertain, rejected_regions=rejected,
                          correction=dict(method='labelled_neutral_diagonal_bgr', gains=gains.tolist(),
                                          reference_sha256=self.reference['source_sha256']))
        except ValueError as exc:
            result['reasons'] = [str(exc)]
            result['uncertain_regions'] = [dict(possible_colors=list(range(1, 7)),
                                                rejection_reason=str(exc))]
        return result
