"""已核对的底盘协议能力与入口许可；协议存在不代表已准许实车运动。"""

from dataclasses import dataclass


@dataclass(frozen=True)
class FirmwareCapabilities:
    odom_fault: bool = False
    can_history: bool = False
    start_schema: int = 0
    physical_motion: frozenset = frozenset()
    simulated_motion: frozenset = frozenset()
    physical_key: bool = False
    simulated_key: bool = False

    def permits_motion(self, route, *, simulation=False):
        return route in (self.simulated_motion if simulation else self.physical_motion)

    def permits_key(self, *, simulation=False):
        return self.simulated_key if simulation else self.physical_key


_SHORT = frozenset(('short',))
_NORTH = _SHORT | {'north'}
_RAW = _NORTH | {'raw'}
_TOUR = _RAW | {'tour'}
_BATCH = _TOUR | {'batch'}
_START2 = _BATCH | {'start2'}
_SWITCH = _START2 | {'switch'}
_NONE = frozenset()

# 保留drive_core原实车白名单：.12～.16及.22以后不因只读能力而开放运动。
_PHYSICAL = {6: _SHORT, 7: _NORTH, 8: _RAW, 9: _RAW, 10: _TOUR, 11: _BATCH,
             17: _BATCH, 18: _BATCH, 19: _START2, 20: _SWITCH, 21: _SWITCH}
_PHYSICAL_KEY = frozenset((17, 18, 19, 20, 21))
# 此表同样保留原离线入口，不把未加入回归的.15/.16/.25～.34自动视为已验证。
_SIMULATED = {**_PHYSICAL, 12: _BATCH, 13: _BATCH, 14: _BATCH,
              **dict.fromkeys((22, 23, 24, 35, 36, 37, 38, 39, 40, 41, 42, 45, 46, 47, 48, 49, 50), _SWITCH)}
_SIMULATED_KEY = _PHYSICAL_KEY | {22, 23, 24, 35, 36, 37, 38, 39, 40, 41, 42, 45, 46, 47, 48, 49, 50}

# 37：.9起固定112字节ExecFault；38：.12起schema1首次CAN错误/历史分页。
# 60：.15/.16为schema1，.17～.19为schema2，.20～.43为schema3（增加PC1）。
# 对照Comm/execute.c、Board/base_port.c及docs/can_capture.md、speed_start.md、
# zone_switch.md。这里只列已核对发布段；未来版本必须核对协议后显式扩充。
_CAPABILITIES = {
    f'GC-BASE-0.3.{version}': FirmwareCapabilities(
        odom_fault=version >= 9,
        can_history=version >= 12,
        start_schema=3 if version >= 20 else 2 if version >= 17 else 1 if version >= 15 else 0,
        physical_motion=_PHYSICAL.get(version, _NONE),
        simulated_motion=_SIMULATED.get(version, _NONE),
        physical_key=version in _PHYSICAL_KEY,
        simulated_key=version in _SIMULATED_KEY)
    for version in range(6, 51)
}
_UNKNOWN = FirmwareCapabilities()


def firmware_capabilities(firmware):
    """严格匹配完整版本；未知/缺失/别名无扩展查询或运动许可。"""
    return _CAPABILITIES.get(firmware, _UNKNOWN) if isinstance(firmware, str) else _UNKNOWN
