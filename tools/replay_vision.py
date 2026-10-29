"""旧现场样本逐帧回放，对比原核心与迁入核心；不打开相机和串口。"""

import hashlib
import json
from pathlib import Path
import sys
import argparse
import shutil
import types
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'pi'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=Path('C:/Users/RISI/Desktop/智能搬运小车'))
    parser.add_argument('--prepare', action='store_true', help='复制指定旧样本至忽略的output目录')
    parser.add_argument('--legacy-tests', action='store_true', help='对迁入核心运行旧圆环/占用/对准测试')
    args = parser.parse_args()
    import cv2
    import numpy as np
    from robot.vision.observe import Detector
    from robot.vision.legacy.placement_vision.core import cleaned

    reference = args.source / 'Pi'
    sys.path.insert(0, str(reference))
    from material_vision.core import MaterialDetector
    from ring_vision.core import RingDetector

    detector = Detector()
    old = dict(material=MaterialDetector(detector.profiles['material_profile']),
               ring=RingDetector(detector.profiles['ring_profile']))
    if args.prepare:
        target = ROOT / 'output/vision/samples'
        target.mkdir(parents=True, exist_ok=True)
        docs = args.source / 'Docs'
        paths = sorted((docs / 'material_vision_20261020').glob('*_raw.jpg'))[:8]
        paths += sorted((docs / 'station_reference_completion_20261020/rough1_initial').glob('frame_*.jpg'))[:3]
        if not paths:
            raise RuntimeError('未找到旧现场样本')
        entries = []
        for index, path in enumerate(paths):
            name = f'{index:02d}_{path.name}'
            shutil.copyfile(path, target / name)
            entries.append(dict(id=str(index), file=name, source=str(path),
                                sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        (target.parent / 'samples.json').write_text(json.dumps(entries, ensure_ascii=False, indent=2), encoding='utf-8')
    samples = json.loads((ROOT / 'output/vision/samples.json').read_text(encoding='utf-8'))
    records = []
    for sample in samples:
        raw = (ROOT / 'output/vision/samples' / sample['file']).read_bytes()
        assert hashlib.sha256(raw).hexdigest() == sample['sha256']
        frame = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        assert frame is not None
        for mode in old:
            expected = cleaned(old[mode].detect(frame))
            actual = detector.process(frame, mode)['items']
            # 同一OpenCV运行时逐字段比较；不删减分数或候选。
            assert json.dumps(expected, sort_keys=True) == json.dumps(actual, sort_keys=True), (sample['id'], mode)
            records.append(dict(sample=sample, mode=mode, matched=True, items=actual))
    legacy_tests = None
    if args.legacy_tests:
        suite = unittest.TestSuite()
        for name in ('ring', 'placement', 'alignment'):
            path = args.source / f'Test/test_{name}_vision.py'
            source = path.read_text(encoding='utf-8')
            for module in ('qr_camera', 'material_vision', 'ring_vision', 'placement_vision', 'alignment_vision'):
                source = source.replace('from ' + module + '.', 'from robot.vision.legacy.' + module + '.')
            scope = types.ModuleType('migrated_' + name)
            scope.__file__ = str(path)
            exec(compile(source, str(path), 'exec'), scope.__dict__)
            suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(scope))
        result = unittest.TextTestRunner().run(suite)
        if not result.wasSuccessful():
            raise RuntimeError('旧核心回归失败')
        legacy_tests = result.testsRun
    report = dict(domain='historical_replay', opencv=cv2.__version__, numpy=np.__version__,
                  legacy_tests=legacy_tests,
                  compared=len(records), records=records, physical_acceptance=False,
                  limitation='原核心与迁入核心同运行时一致；不是当前实物精度或成功抓放验收')
    (ROOT / 'output/vision/replay.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(dict(compared=len(records), matched=True, candidates=sum(len(v['items']) for v in records))))


if __name__ == '__main__':
    main()
