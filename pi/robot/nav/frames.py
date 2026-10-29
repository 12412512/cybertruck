"""SE(2)坐标边界。内部米/弧度、X前/Y左/逆时针正，画布单独翻转Y。"""

from dataclasses import dataclass
import math


def angle(value):
    if not math.isfinite(value):
        raise ValueError('角度必须有限')
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class Pose:
    x_m: float
    y_m: float
    yaw_rad: float

    def __post_init__(self):
        if any(type(v) not in (int, float) or not math.isfinite(v)
               for v in (self.x_m, self.y_m, self.yaw_rad)):
            raise ValueError('位姿必须包含有限米/弧度数值')

    def values(self):
        return [self.x_m, self.y_m, self.yaw_rad]


def compose(parent, local):
    co, si = math.cos(parent.yaw_rad), math.sin(parent.yaw_rad)
    return Pose(parent.x_m + co * local.x_m - si * local.y_m,
                parent.y_m + si * local.x_m + co * local.y_m,
                parent.yaw_rad + local.yaw_rad)


def relative(parent, world):
    co, si = math.cos(parent.yaw_rad), math.sin(parent.yaw_rad)
    dx, dy = world.x_m - parent.x_m, world.y_m - parent.y_m
    return Pose(co * dx + si * dy, -si * dx + co * dy, world.yaw_rad - parent.yaw_rad)


def legacy(x_m, y_right_m, yaw_clockwise_rad):
    return Pose(x_m, -y_right_m, -yaw_clockwise_rad)


def canvas(pose, height_m, pixels_per_m):
    if not math.isfinite(height_m) or not math.isfinite(pixels_per_m) or height_m <= 0 or pixels_per_m <= 0:
        raise ValueError('画布尺寸无效')
    return pose.x_m * pixels_per_m, (height_m - pose.y_m) * pixels_per_m, -pose.yaw_rad
