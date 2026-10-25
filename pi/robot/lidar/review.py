"""从已结束实扫生成安装及配准核对摘要；无设备访问或动作权限。"""
import copy
import math

from .calibration import audit_axes, digest, sample, same_profile
from .geometry import radar_pose


def build_review(installation, body_pose, records):
    if [row['label'] for row in records] != ['P1复测', 'P2', 'P3', 'P4独立点']:
        raise ValueError('需要明确三点拟合与第四独立点的对应，不能交换或省略')
    samples = []
    for row in records:
        measured = row['sample']
        checked = sample(row['capture'], measured['map_m'][0]*1000,
            2400-measured['map_m'][1]*1000, measured['measurement_uncertainty_m']*1000)
        # 浮点单位换算只比较毫米级读取的必要误差，不改变任何实测点。
        if (checked['scan_id'] != measured['scan_id'] or
                checked['source_m'] != measured['source_m'] or
                any(abs(a-b) > 1e-12 for a, b in zip(checked['map_m'], measured['map_m'])) or
                checked['vendor_pose'] != measured['vendor_pose'] or
                checked['board_identity'] != measured['board_identity']):
            raise ValueError('尺量样本与原始采集报告不对应')
        samples.append(checked)
    same_profile(samples)
    diagnostic = audit_axes(samples[:3], samples[3])
    reflected = diagnostic['hypotheses'][1]
    pose = radar_pose(installation, body_pose)
    points = [dict(label=row['label'], scan_id=value['scan_id'], map_m=value['map_m'],
        vendor_m=value['source_m'], spread_m=value['spread_m'],
        measurement_uncertainty_m=value['measurement_uncertainty_m'],
        used_for_fit=index < 3) for index, (row, value) in enumerate(zip(records, samples))]
    review = dict(schema=1, domain='lidar_commissioning_review', live=False,
        historical=True, real_map_registered=False, authorizes_motion=False,
        auto_execution_ready=False, applied=False,
        installation=copy.deepcopy(installation), nominal_body_pose_m_rad=list(body_pose),
        nominal_radar_pose_m_rad=pose, body_pose_is_nominal=True,
        information=copy.deepcopy(records[-1]['capture']['information']),
        board_identity=samples[0]['board_identity'], points=points,
        independent_prediction_m=reflected.get('predicted_m'),
        independent_error_m=reflected.get('holdout_error_m'), independent_limit_m=.03,
        diagnostic=diagnostic,
        checks=[
            dict(name='安装参数', status='RECORDED', detail='右侧约145mm；正前方向−Y，盲区＋Y；误差未量化'),
            dict(name='静态扫描', status='HISTORY_PASS', detail='已结束样本完整、停止确认有效；不代表当前设备在线'),
            dict(name='地图坐标', status='PENDING', detail=reflected['reason']),
            dict(name='道路覆盖', status='PENDING', detail='已证明局部目标响应；未确认全场道路覆盖'),
            dict(name='树莓派自动执行', status='PENDING', detail='复用现有规划与执行链，坐标和覆盖验收后再接入')],
        next_run_contract=['PC1选择起点与匹配雷达区号', 'PC0启动后取得本轮停车扫描',
            '校验坐标版本、覆盖、反馈、场景与速度', '同一规划器生成本轮路线',
            '唯一执行器运行并保留故障停车'])
    if not all(math.isfinite(value) for value in pose):
        raise ValueError('安装几何不是有限值')
    review['version'] = digest(review)
    return review
