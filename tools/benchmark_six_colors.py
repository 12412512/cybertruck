"""六色合成扰动对照；尺度仅模拟像素占比，不代表实物高度或相机精度。"""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))
from robot.vision.legacy.material_vision.core import (
    PALETTE, MaterialDetector, default_profile, sample_color, append_shape_reference)
from robot.vision.material_robust import RobustMaterialDetector, make_reference


def fixture_context():
    return dict(camera_id='synthetic-camera', camera_config_sha256='a'*64, image_size=[1280, 720],
                pose_id='synthetic-observation', pose_revision=1, pose_sha256='b'*64,
                target_plane='synthetic-plane', view='raw')


def scene(color, scale=1., angle=0., perspective=False):
    frame = np.full((720, 1280, 3), 230, np.uint8)
    frame[32:96, 32:96] = 160
    points = np.float32([[-45, 50], [45, 50], [25, -50], [-25, -50]])
    if perspective:
        points[:, 0] += np.where(points[:, 1] < 0, 18, -5)
        points[:, 1] *= .7
    theta = np.deg2rad(angle)
    rotation = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    points = np.rint(points @ rotation.T * scale + [650, 410]).astype(np.int32)
    cv2.fillPoly(frame, [points], PALETTE[color])
    return frame, points


def calibration():
    profile = default_profile()
    profile.update(plate_roi=[180, 140, 1120, 660], plate_roi_shape='rectangle',
                   grasp_roi=[180, 140, 1120, 660])
    for color in range(1, 7):
        patch = np.full((40, 40, 3), PALETTE[color], np.uint8)
        profile['colors'][str(color)] = sample_color(patch, [0, 0, 40, 40], color)
    frame, points = scene(1)
    append_shape_reference(profile, points)
    reference = make_reference(frame, [32, 32, 96, 96], fixture_context(),
                               hashlib.sha256(frame.tobytes()).hexdigest())
    return profile, reference


def perturb(frame, name):
    value = frame.astype(np.float32)
    if name == 'dim':
        value *= .65
    elif name == 'bright':
        value *= 1.25
    elif name == 'warm':
        value *= np.float32([.78, 1., 1.15])
    elif name == 'cool':
        value *= np.float32([1.15, 1., .78])
    elif name == 'gamma':
        value = 255 * (value / 255) ** 1.25
    elif name == 'local_shadow':
        value[:, 540:740] *= .55  # 灰参考未被相同阴影覆盖，不能假定补偿必然有效。
    elif name == 'blur':
        value = cv2.GaussianBlur(value, (7, 7), 1.6)
    elif name != 'normal':
        raise ValueError('未知扰动')
    return np.clip(np.rint(value), 0, 255).astype(np.uint8)


def scores(items, color, expected_box):
    accepted = []
    for item in items:
        if item['clipped'] or item['quality_score'] < .55 or (item['color_id'] == 5 and not item['shape_consistent']):
            continue
        accepted.append(item)
    def overlaps(item):
        x, y, w, h = item['bbox_px']
        ex, ey, ew, eh = expected_box
        intersection = max(0, min(x+w, ex+ew)-max(x, ex)) * max(0, min(y+h, ey+eh)-max(y, ey))
        return intersection / max(1., w*h+ew*eh-intersection) >= .5
    correct = sum(item['color_id'] == color and overlaps(item) for item in accepted)
    false_positive = sum(not (item['color_id'] == color and overlaps(item)) for item in accepted)
    return dict(correct=correct == 1, false_positive=false_positive)


def benchmark(output):
    profile, reference = calibration()
    old, new = MaterialDetector(profile), RobustMaterialDetector(profile, reference)
    records = []
    for color in range(1, 7):
        for light in ('normal', 'dim', 'bright', 'warm', 'cool', 'gamma', 'local_shadow', 'blur'):
            for scale in (.65, 1., 1.35):
                for angle, perspective in ((0., False), (40., False), (-35., True)):
                    base, points = scene(color, scale, angle, perspective)
                    frame = perturb(base, light)
                    box = cv2.boundingRect(points)
                    start = time.perf_counter()
                    original = old.detect(frame)
                    old_ms = (time.perf_counter() - start)*1000
                    start = time.perf_counter()
                    result = new.process(frame, fixture_context())
                    new_ms = (time.perf_counter() - start)*1000
                    records.append(dict(color=color, lighting=light, scale=scale, angle=angle,
                                        perspective=perspective, baseline=scores(original, color, box),
                                        candidate=scores(result['items'], color, box),
                                        baseline_ms=old_ms, candidate_ms=new_ms))
    grouped = {}
    for light in sorted({r['lighting'] for r in records}):
        rows = [r for r in records if r['lighting'] == light]
        grouped[light] = dict(cases=len(rows), **{kind: dict(
            correct=sum(r[kind]['correct'] for r in rows),
            false_positive=sum(r[kind]['false_positive'] for r in rows)) for kind in ('baseline', 'candidate')})
    report = dict(domain='synthetic_benchmark', physical_acceptance=False,
                  opencv=cv2.__version__, numpy=np.__version__, cases=len(records), groups=grouped,
                  median_ms={kind: float(np.median([r[kind+'_ms'] for r in records]))
                             for kind in ('baseline', 'candidate')}, records=records,
                  limitation='单物料合成色块；scale/透视不等于实物高度/角度标定；无现场准确率结论')
    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'output/six-color/benchmark.json')
    args = parser.parse_args()
    report = benchmark(args.output)
    print(json.dumps({k: report[k] for k in ('cases', 'groups', 'median_ms', 'physical_acceptance')}, ensure_ascii=False))
