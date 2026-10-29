"""雷达安装的平面几何；只接收明确的雷达局部点，不接收厂家场地坐标。"""
import math

from .replay import map_point


def finite_vector(value, size):
    if (not isinstance(value, (list, tuple)) or len(value) != size or
            any(type(number) not in (int, float) or not math.isfinite(number) for number in value)):
        raise ValueError('需要有限数值的平面坐标或位姿，单位米/弧度')
    return value


def radar_pose(installation, body_pose):
    """body_pose为O的map位姿；安装z未知，不输出三维TF。"""
    finite_vector(body_pose, 3)
    if installation.get('parent_frame') != 'base_link':
        raise ValueError('安装参数必须以四轮中心base_link为基准')
    offset = installation['translation_m']
    yaw = installation.get('mount_yaw_rad')
    finite_vector([offset['x'], offset['y'], yaw], 3)
    position = map_point(offset['x']*1000, offset['y']*1000, body_pose)
    return position + [math.atan2(math.sin(body_pose[2]+yaw), math.cos(body_pose[2]+yaw))]


def local_to_map(observation, installation, body_pose):
    """仅几何预览；调用者不能将厂家field点重命名成局部点来重复加外参。"""
    if observation.get('frame_id') != 'lidar_local':
        raise ValueError('仅接受lidar_local；厂家场地坐标须先明确其转换定义')
    point = finite_vector(observation['position_m'], 2)
    pose = radar_pose(installation, body_pose)
    position = map_point(point[0]*1000, point[1]*1000, pose)
    return dict(domain='lidar_geometry_preview', position_m=position, frame_id='map',
        real_map_registered=False, authorizes_motion=False,
        installation_version=installation['version'])
