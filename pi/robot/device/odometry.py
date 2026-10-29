"""正式C连续观察器的主机模型绑定。只用于离线验证，不连接硬件。"""

import ctypes as c
from pathlib import Path


class Geometry(c.Structure):
    _fields_ = [('radius_m', c.c_float), ('half_length_m', c.c_float),
                ('half_width_m', c.c_float), ('layout', c.c_int)]


class Config(c.Structure):
    _fields_ = [('geometry', Geometry), ('counts_per_rev', c.c_double * 4),
                ('address', c.c_uint8 * 4), ('sign', c.c_int8 * 4),
                ('yaw_scale_rad', c.c_float), ('yaw_sign', c.c_int),
                ('max_yaw_rad_s', c.c_float), ('max_rpm', c.c_float),
                ('margin_counts', c.c_double), ('fresh_ms', c.c_uint32),
                ('gap_ms', c.c_uint32), ('can_epoch', c.c_uint32),
                ('imu_epoch', c.c_uint32), ('imu_source_epoch', c.c_uint32),
                ('calibration', c.c_uint32)]


class Stamp(c.Structure):
    _fields_ = [(key, c.c_uint32) for key in ('first_ms', 'last_ms', 'handled_ms', 'sequence', 'epoch')]


class Motor(c.Structure):
    _fields_ = [('stamp', Stamp), ('value', c.c_int64), ('address', c.c_uint8), ('kind', c.c_uint8)]


class Imu(c.Structure):
    _fields_ = [('stamp', Stamp), ('source_epoch', c.c_uint32), ('axes', c.c_int16 * 3),
                ('raw', c.c_uint8 * 11), ('kind', c.c_uint8)]


class Feed(c.Structure):
    _fields_ = [('stamp', Stamp), ('source_epoch', c.c_uint32), ('values', c.c_int64 * 3),
                ('kind', c.c_uint8), ('address', c.c_uint8)]


class Pose(c.Structure):
    _fields_ = ([(key, c.c_double) for key in ('x_m', 'y_m', 'yaw_rad', 'wheel_yaw_rad')]
                + [(key, c.c_uint32) for key in ('tick_ms', 'origin_ms', 'steps', 'waits',
                                                'fault', 'valid', 'calibration')])


class Body(c.Structure):
    _fields_ = [(key, c.c_float) for key in ('vx_m_s', 'vy_m_s', 'wz_rad_s')]


class Odometry:
    def __init__(self, library, config, now_ms=0):
        self.lib = c.CDLL(str(Path(library).resolve(strict=True)))
        self.config = config
        self.lib.Odom_Bytes.restype = c.c_size_t
        self.lib.Odom_Init.argtypes = [c.c_void_p, c.POINTER(Config), c.c_uint32]
        self.lib.Odom_Pos.argtypes = [c.c_void_p, c.POINTER(Motor), c.c_uint32]
        self.lib.Odom_Yaw.argtypes = [c.c_void_p, c.POINTER(Imu), c.c_uint32]
        self.lib.Odom_Step.argtypes = [c.c_void_p, c.c_uint32]
        self.lib.Odom_Read.argtypes = [c.c_void_p, c.c_uint32, c.POINTER(Pose)]
        self.lib.Kin_Inverse.argtypes = [Geometry, Body, c.POINTER(c.c_float)]
        self.lib.Feed_Encode.argtypes = [c.POINTER(Feed), c.c_uint32, c.POINTER(c.c_uint8)]
        size = self.lib.Odom_Bytes()
        # double数组保证本C结构所需8字节对齐，大小由当前正式实现报告。
        self.storage = (c.c_double * ((size + 7) // 8))()
        if not self.lib.Odom_Init(self.storage, c.byref(config), now_ms):
            raise ValueError('里程计配置不完整或无效')

    def encode(self, kind, address, stamp, source_epoch, values, report_ms):
        sample = Feed(stamp, source_epoch, (c.c_int64 * 3)(*values), kind, address)
        raw = (c.c_uint8 * 56)()
        if not self.lib.Feed_Encode(c.byref(sample), report_ms, raw):
            raise ValueError('模拟反馈编码失败')
        return bytes(raw)

    def inverse(self, vx, vy, wz):
        raw = (c.c_float * 4)()
        if not self.lib.Kin_Inverse(self.config.geometry, Body(vx, vy, wz), raw):
            raise ValueError('模拟运动学输入无效')
        return tuple(raw)

    def push(self, sample, now_ms):
        stamp = Stamp(sample.first_ms, sample.last_ms, sample.handled_ms, sample.sequence, sample.epoch)
        if sample.kind == 2:
            raw = Motor(stamp, sample.values[0], sample.address, 0x36)
            return self.lib.Odom_Pos(self.storage, c.byref(raw), now_ms)
        if sample.kind == 4:
            raw = Imu()
            raw.stamp = stamp
            raw.source_epoch = sample.source_epoch
            raw.axes[:] = sample.values
            raw.kind = 0x53
            return self.lib.Odom_Yaw(self.storage, c.byref(raw), now_ms)
        raise ValueError('里程计仅消费位置和航向，其他诊断通道由设备管理者分派')

    def step(self, now_ms):
        return self.lib.Odom_Step(self.storage, now_ms)

    def snapshot(self, now_ms):
        pose = Pose()
        self.lib.Odom_Read(self.storage, now_ms, c.byref(pose))
        return {name: getattr(pose, name) for name, _ in Pose._fields_}
