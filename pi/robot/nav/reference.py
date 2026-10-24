"""直接绑定正式C冻结参考，规划器不另写运动曲线或弧线几何公式。"""

import ctypes as c
import math

from ..device.tracking import Spec, Limit, PathSpec, Reference
from .frames import Pose, relative

CRUISE_M_S = .14
DOCK_M_S = .025
SHORT_MOVE_M = .3
SETTLE_ESTIMATE_S = 1.0


def specification(kind, start, end, radius=.1, reverse=False, velocity=CRUISE_M_S):
    spec = Spec()
    spec.linear = Limit(velocity, .25, .8)
    spec.angular = Limit(.7, 8, 300)
    if kind in ('LINE', 'DIAGONAL'):
        local = relative(start, end)
        spec.kind, spec.x_m, spec.y_m = 1, local.x_m, local.y_m
    elif kind == 'TURN':
        spec.kind, spec.yaw_rad = 2, end.yaw_rad - start.yaw_rad
    elif kind == 'ARC':
        spec.kind = 3
        spec.path = PathSpec(0, 0, radius, radius * .35,
                             1 if end.yaw_rad > start.yaw_rad else -1, int(reverse))
        spec.tangent = 1
        spec.linear.velocity = min(velocity, radius * .6)
    else:
        raise ValueError('未知路线动作')
    return spec


class NativeReference:
    def __init__(self, library, start, spec):
        self.lib = library
        library.Ref_Bytes.restype = c.c_size_t
        library.Ref_Time.argtypes = [c.c_void_p]
        library.Ref_Time.restype = c.c_double
        library.Ref_Make.argtypes = [c.c_void_p, c.POINTER(Spec), c.c_double, c.c_double, c.c_double]
        library.Ref_At.argtypes = [c.c_void_p, c.c_double, c.POINTER(Reference)]
        library.Ref_Bounds.argtypes = [c.c_void_p, c.POINTER(c.c_double), c.POINTER(c.c_double)]
        self.storage = (c.c_double * ((library.Ref_Bytes() + 7) // 8))()
        self.start, self._spec = start, bytes(spec)
        if not library.Ref_Make(self.storage, c.byref(spec), *start.values()):
            raise ValueError('正式C参考拒绝目标或角速度/加速度/加加速度限制')
        self.duration_s = library.Ref_Time(self.storage)
        velocity, yaw = c.c_double(), c.c_double()
        if not library.Ref_Bounds(self.storage, c.byref(velocity), c.byref(yaw)):
            raise ValueError('不能证明参考变化率上界')
        self.velocity_bound = velocity.value
        self.yaw_bound = yaw.value
        self.end = self.at(self.duration_s)

    @property
    def spec(self):
        return Spec.from_buffer_copy(self._spec)

    def at(self, seconds):
        point = self.point(seconds)
        return Pose(point.x_m, point.y_m, point.yaw_rad)

    def point(self, seconds):
        point = Reference()
        if not self.lib.Ref_At(self.storage, seconds, c.byref(point)):
            raise ValueError('参考采样超限')
        return point

    def samples(self, step_s=.03):
        if not math.isfinite(step_s) or step_s <= 0:
            raise ValueError('参考采样周期无效')
        count = max(1, math.ceil(self.duration_s / step_s))
        if count > 100000:
            raise ValueError('参考检查超过有界采样预算')
        return [(self.duration_s * i / count, self.at(self.duration_s * i / count)) for i in range(count + 1)]
