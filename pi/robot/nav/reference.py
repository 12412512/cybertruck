"""直接绑定正式C冻结参考，规划器不另写运动曲线或弧线几何公式。"""

import ctypes as c
import math

from ..device.tracking import Spec, Limit, PathSpec, Reference
from .frames import Pose, relative, angle

CRUISE_M_S = .14
DOCK_M_S = .025
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


def flow_specification(library, kind, start, end, corner=None, radius=.1, terminal=False,
                       window_m=.08, velocity=CRUISE_M_S, lookahead=False, head_pad_m=0,
                       turn_rate_rad_s=.7):
    """位置曲线与车身朝向解耦；内部低速区不创建第二个动作或停车状态。"""
    local = relative(start, end)
    spec = specification('LINE', start, end, velocity=velocity)
    spec.yaw_rad = angle(end.yaw_rad-start.yaw_rad)
    spec.kind = 5
    spec.angular = Limit(.7, 2.0, 12.0)
    if kind == 'FLOW':
        if corner is None:
            raise ValueError('连续弯道缺少拐点')
        incoming = math.atan2(corner.y_m-start.y_m, corner.x_m-start.x_m)
        outgoing = math.atan2(end.y_m-corner.y_m, end.x_m-corner.x_m)
        turn = angle(outgoing-incoming)
        if abs(abs(turn)-math.pi/2) > 1e-5:
            raise ValueError('当前平滑几何仅支持90度连接')
        measure = specification('ARC',Pose(0,0,0),Pose(0,0,turn),radius)
        measure.tangent = 0
        geometry = NativeReference(library,Pose(0,0,0),measure)
        displacement = abs(geometry.end.x_m)
        lead = math.dist(start.values()[:2],corner.values()[:2])-displacement
        tail = math.dist(end.values()[:2],corner.values()[:2])-displacement
        if min(lead,tail) <= .02:
            raise ValueError('直线长度不足以容纳连续弯道')
        spec.kind, spec.x_m, spec.y_m = 4, 0, 0
        spec.path = PathSpec(lead,tail,radius,radius*.35,1 if turn>0 else -1,0)
        spec.path_heading_rad = angle(incoming-start.yaw_rad)
        # 巡航档由规划实例冻结，弯道仍受向心加速度与曲率jerk限制。
        spec.curve_velocity_m_s = min(velocity,math.sqrt(.125*radius),(.4*radius*radius/4.5)**(1/3))
        if lookahead:
            speed,core_length = geometry.curve_limits(spec.linear)
            spec.curve_velocity_m_s = speed
            spec.yaw_begin_m = max(0,lead-head_pad_m)
            spec.yaw_end_m = min(lead+core_length+tail,lead+core_length+head_pad_m)
        spec.tail_velocity_m_s = min(DOCK_M_S,velocity) if terminal else velocity
    elif kind == 'WINDOW':
        if corner is None:
            raise ValueError('转姿段缺少安全窗口')
        along = math.dist(start.values()[:2],corner.values()[:2])
        distance = math.hypot(local.x_m,local.y_m)
        if along <= window_m or along >= distance-window_m:
            raise ValueError('转姿窗口不在直线内部')
        spec.kind = 6
        spec.yaw_begin_m, spec.yaw_end_m = along-window_m, along+window_m
        spec.curve_velocity_m_s = velocity
        spec.tail_velocity_m_s = min(DOCK_M_S,velocity) if terminal else velocity
    elif terminal:
        distance = math.hypot(local.x_m,local.y_m)
        if distance <= .07:
            spec.linear.velocity = min(DOCK_M_S,velocity)
        else:
            spec.kind = 6
            spec.yaw_begin_m, spec.yaw_end_m = distance-.05, distance-.03
            spec.curve_velocity_m_s = min(DOCK_M_S,velocity)
            spec.tail_velocity_m_s = min(DOCK_M_S,velocity)
    if lookahead and kind in ('FLOW','WINDOW'):
        spec.angular.velocity = turn_rate_rad_s
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
        try:
            library.Ref_TurnSpeed.argtypes = [c.c_void_p,c.POINTER(c.c_double)]
        except AttributeError as exc:
            raise ValueError('导航模型库版本不匹配，请重新构建主机库') from exc
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
        turning = c.c_double()
        if not library.Ref_TurnSpeed(self.storage,c.byref(turning)):
            raise ValueError('不能证明航向变化区间的平移上界')
        self.turning_velocity_bound = turning.value
        self.end = self.at(self.duration_s)

    @property
    def spec(self):
        return Spec.from_buffer_copy(self._spec)

    def curve_limits(self,limit):
        try:
            self.lib.Ref_Curve.argtypes = [c.c_void_p,Limit,c.POINTER(c.c_double),c.POINTER(c.c_double)]
        except AttributeError as exc:
            raise ValueError('弯道模型库版本不匹配，请重新构建主机库') from exc
        speed,length = c.c_double(),c.c_double()
        if not self.lib.Ref_Curve(self.storage,limit,c.byref(speed),c.byref(length)):
            raise ValueError('无法证明弯道常速区的V/A/J限制')
        return speed.value,length.value

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
