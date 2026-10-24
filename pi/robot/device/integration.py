"""实车接入准备清单：读取本工程映射，不打开设备，不提升运动权限。"""

import json
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[3]


def preparation():
    header = (ROOT / 'firmware/User/Config/mech_hw.h').read_text(encoding='utf-8')
    values = {name: int(value) for name, value in re.findall(r'^#define\s+(MECH_\w+)\s+(-?\d+)[UL]*\s*$', header, re.M)}
    install = json.loads((ROOT / 'config/mechanism_installation.json').read_text(encoding='utf-8'))
    ioc = (ROOT / 'firmware/Car.ioc').read_text(encoding='utf-8')
    pins = dict((label, pin) for pin, label in re.findall(r'^(P[A-Z]\d+)\.GPIO_Label=(\w+)\s*$', ioc, re.M))
    servos = []
    for name, key in [('料盘', 'TRAY'), ('转臂', 'ARM'), ('夹爪', 'GRIP')]:
        pin = pins[f'{key}_PWM']
        channel = values[f'MECH_{key}_CHANNEL']
        if f'{pin}.Signal=S_TIM1_CH{channel}' not in ioc:
            raise ValueError('机构逻辑通道与CubeMX引脚不一致')
        servos.append(dict(name=name, pin=pin, channel=channel))
    slots = install['tray_slot_deg']
    if (install['schema'] != 1 or len(slots) != 3 or len(set(slots)) != 3
            or any(type(angle) is not int or not 0 <= angle <= 270 for angle in slots)
            or install['tray_reference_deg'] in slots):
        raise ValueError('机构安装记录的料位/基准定义无效')
    return dict(domain='preparation', real_connected=False, real_motion=False,
                axis_executor=dict(stage='offline_validated', app_bound=False, device_enabled=False,
                                   states=['ACCEPTED','SENDING','RUNNING','SETTLING','STOPPING','COMPLETE','FAILED','CANCELLED'],
                                   completion='TXOK、ACK及新反馈共同确认；停止TXOK不代表物理停车'),
                wiring_confirmed=install['wiring_confirmed_by_user'], physical_revalidated=False,
                baseline=install['zero_baseline'], tray_reference_deg=install['tray_reference_deg'], tray_slot_deg=slots,
                axes=[dict(name='升降', address=values['MECH_LIFT_ID'], direction='向下为正', feedback_sign=values['MECH_LIFT_SIGN'],
                           max_mm=values['MECH_LIFT_MAX_UM'] / 1000),
                      dict(name='伸缩', address=values['MECH_REACH_ID'], direction='向外为正', feedback_sign=values['MECH_REACH_SIGN'],
                           max_mm=values['MECH_REACH_MAX_UM'] / 1000)],
                servos=servos,
                pending=['每次接入均需核对板端版本、故障与停车证据；历史验收记录不替代当前状态',
                         'ID5/6完整配置与35/36/3A/33反馈尚未接入唯一CAN调度器',
                         '单轴意图执行器、人工建零及互锁已离线准备；未绑定App或真实通信命令',
                         '舵机首次PWM输出与实际姿态须单独验收；启动保持不输出',
                         'Web实车命令尚未启用；模拟零点/示教不得提升为实物有效状态'],
                next_step='通知用户后连接树莓派与开发板，只读核对当前状态；不使能、不清故障、不动作')
