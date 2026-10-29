"""离线工作区和版本文件；只保存参数/地图/轨迹，不保存设备凭据或控制令牌。"""

import json
import hashlib


def write_json(path, value):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2), encoding='utf-8')
    temporary.replace(path)


def payload(plan):
    value = dict(plan.view, specs=[raw.hex() for raw in plan.specs])
    value['artifact_sha256'] = artifact_digest(value)
    return value


def artifact_digest(value):
    data = {key: item for key, item in value.items() if key != 'artifact_sha256'}
    return hashlib.sha256(json.dumps(data, sort_keys=True, allow_nan=False).encode()).hexdigest()


def versions(runtime):
    folder = runtime / 'plans'
    if not folder.exists():
        return []
    result = []
    for path in sorted(folder.glob('*.json'), key=lambda item: item.stat().st_mtime, reverse=True)[:30]:
        try:
            value = json.loads(path.read_text(encoding='utf-8'))
            result.append(dict(id=path.stem, source=value['source'], seconds=value['seconds'],
                               speed_m_s=value['parameters']['speed_m_s'], schema=value.get('schema', 1)))
        except (OSError, ValueError, TypeError, KeyError):
            continue
    return result
