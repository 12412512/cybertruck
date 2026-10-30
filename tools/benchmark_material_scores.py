"""只读对照六色得分内存优化；本机时间不代表Pi帧率或250ms整链验收。"""

import argparse
import copy
import gc
import hashlib
import json
from pathlib import Path
import statistics
import sys
import time
import tracemalloc
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'pi')]

import cv2
import numpy as np
from robot.vision.legacy.material_vision import core
from robot.vision.legacy.placement_vision.core import cleaned
from tools.replay_material_handoff import checked_path, verify_package


def original_scores(channels, hsv, colors):
    """保留改动前的全样本距离图与reduce，作为独立对照。"""
    scores = []
    for color in range(1, 7):
        model = colors[str(color)]
        distances = []
        for sample in model.get('examples', [model]):
            distance = core.color_distance(channels, sample)
            gate = (hsv[:, :, 2] <= min(145, max(70, sample['v95'] + 20))
                    if color == 5 else hsv[:, :, 1] >= 25)
            distances.append(np.where(gate, distance, 1e4))
        scores.append(np.minimum.reduce(distances))
    return np.stack(scores)


def observed(detector, frame):
    return (cleaned(detector.detect(frame)), copy.deepcopy(detector.rejected_regions),
            copy.deepcopy(detector.uncertain_regions))


def summarize(values):
    return {'median_ms': statistics.median(values), 'p95_ms': float(np.percentile(values, 95)),
            'observations': len(values)}


def run(package, output, rounds):
    package, output = Path(package).resolve(), Path(output).resolve()
    if output.exists() or output.is_relative_to(package):
        raise ValueError('输出必须为原包外的新文件')
    if not 3 <= rounds <= 20:
        raise ValueError('轮数必须为3至20')
    _, manifest = verify_package(package)
    frames = []
    for row in manifest['files']:
        if Path(row['path']).suffix.lower() != '.jpg':
            continue
        raw = checked_path(package, row['path']).read_bytes()
        if hashlib.sha256(raw).hexdigest() != row['sha256']:
            raise ValueError('回放图像摘要不匹配')
        frame = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError('图像解码失败')
        frames.append((row['path'], frame))
    if not frames:
        raise ValueError('交接包无图像')
    profile_path = ROOT / 'config/vision/material_profile.json'
    profile_raw = profile_path.read_bytes()
    profile = json.loads(profile_raw)
    optimized = core._color_scores
    previous_threads = cv2.getNumThreads()
    try:
        cv2.setNumThreads(1)
        detector = core.MaterialDetector(profile)
        for name, frame in frames:
            actual = observed(detector, frame)
            with patch.object(core, '_color_scores', original_scores):
                expected = observed(detector, frame)
            if actual != expected:
                raise AssertionError(f'候选/拒识/歧义不等价: {name}')

        # 原归档九图作为定时子集；排除I/O、解码、报告序列化与tracemalloc开销。
        timing_frames = [(name, frame) for name, frame in frames if '/samples/' in name
                         and '/workbench/' not in name]
        if not timing_frames:
            raise ValueError('交接包缺少历史样本定时子集')
        times = {'original': [], 'optimized': []}
        for _, frame in timing_frames:
            detector.detect(frame)
            with patch.object(core, '_color_scores', original_scores):
                detector.detect(frame)
        for round_id in range(rounds):
            order = ('original', 'optimized') if round_id % 2 == 0 else ('optimized', 'original')
            for name in order:
                with patch.object(core, '_color_scores', original_scores if name == 'original' else optimized):
                    for _, frame in timing_frames:
                        start = time.perf_counter()
                        detector.detect(frame)
                        times[name].append((time.perf_counter() - start) * 1000)

        small = cv2.resize(frames[0][1], (480, 270), interpolation=cv2.INTER_AREA)
        channels = np.ascontiguousarray(cv2.cvtColor(small, cv2.COLOR_BGR2LAB).astype(np.float32).transpose(2, 0, 1))
        hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
        peaks = {}
        for name, scorer in (('original', original_scores), ('optimized', optimized)):
            gc.collect()
            tracemalloc.start()
            scores = scorer(channels, hsv, profile['colors'])
            peaks[name] = tracemalloc.get_traced_memory()[1]
            tracemalloc.stop()
            del scores
        report = {
            'domain': 'local_offline_performance_only', 'opencv': cv2.__version__, 'numpy': np.__version__,
            'opencv_threads': 1, 'profile_file_sha256': hashlib.sha256(profile_raw).hexdigest(),
            'exact_candidate_rejection_uncertainty_matches': len(frames),
            'timing_frames': [name for name, _ in timing_frames], 'rounds': rounds,
            'detector_timing': {name: summarize(values) for name, values in times.items()},
            'score_kernel_tracemalloc_peak_bytes': peaks,
            'limits': ['本机单线程原参数；不含相机、传输、排队、二维码或环号',
                       '计时对照替换原得分聚合块；两组共用当前等价的最优/次优排序',
                       '得分内核tracemalloc峰值并非进程RSS；依赖NumPy分配跟踪',
                       '不是Pi帧率、250ms端到端或现场精度验收'],
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        with output.open('x', encoding='utf-8') as stream:
            json.dump(report, stream, ensure_ascii=False, indent=2)
        return report
    finally:
        cv2.setNumThreads(previous_threads)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--package', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--rounds', type=int, default=7)
    args = parser.parse_args()
    print(json.dumps(run(args.package, args.output, args.rounds), ensure_ascii=False, indent=2))
