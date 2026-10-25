"""厂家两点校准的结果解码与保存一致性核对；没有命令发送或设备访问。"""
import struct


def decode_result(payload):
    """复用厂家SDK的15字节EVT_CALIBRATION，角度Q6转为度。"""
    if len(payload) != 15:
        raise ValueError('厂家校准结果必须为15字节')
    zone, x, y, heading, dx, dy, trim, residual, pair_error = struct.unpack('<BhhHhhhBB', payload)
    if zone not in (1, 2) or heading >= 23040:
        raise ValueError('厂家校准区号或角度非法')
    return dict(zone=zone, x_mm=x, y_mm=y, heading_deg=heading/64,
        x_offset_mm=dx, y_offset_mm=dy, heading_trim_deg=trim/64,
        residual_mm=residual, pair_distance_error_mm=pair_error)


def verify_saved(previous, current, result, zone):
    """须先由调用者确认本轮ACK/事件归属；本函数只检查新旧保存内容。"""
    if type(zone) is not int or zone not in (1, 2) or result['zone'] != zone:
        raise ValueError('校准结果不是本次请求区')
    if previous['storage_version'] != 11 or current['storage_version'] != 11:
        raise ValueError('保存布局版本未核对，不能猜测兼容')
    if current['source_zone'] != zone:
        raise ValueError('保存记录来源区与本次校准不一致')
    advance = (current['record_sequence']-previous['record_sequence']) & 0xffffffff
    if not 0 < advance < 0x80000000:
        raise ValueError('没有本次新保存记录，旧缓存不能当作成功')
    pose = {key:result[key] for key in ('zone', 'x_mm', 'y_mm', 'heading_deg')}
    if current[f'zone{zone}_pose'] != pose:
        raise ValueError('厂家结果与已保存的当前区位姿不匹配')
    other = f'zone{3-zone}_pose'
    if current[other] != previous[other]:
        raise ValueError('另一区保存位姿意外改变，拒绝本次验收')
    return dict(domain='vendor_lidar_calibration_verification', saved_consistent=True,
        zone=zone, record_sequence=current['record_sequence'], pose=pose,
        residual_mm=result['residual_mm'], pair_distance_error_mm=result['pair_distance_error_mm'],
        real_map_registered=False, authorizes_motion=False,
        reason='仅厂家结果与保存内容一致；仍需本轮ACK、停止、地图轴向和独立点验证')
