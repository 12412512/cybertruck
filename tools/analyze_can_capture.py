"""离线解释已有UART/RAM证据和新旧配置；不连接设备、不改固件。"""

import argparse
import hashlib
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'pi'))
from robot.device.can_diag import esr, unsigned
from robot.workbench.storage import write_json


def bit_timing(value, apb1_hz):
    value=unsigned(value)
    prescaler=(value&1023)+1
    segment1=((value>>16)&15)+1
    segment2=((value>>20)&7)+1
    return dict(btr=value,prescaler=prescaler,segment1=segment1,segment2=segment2,
                sjw=((value>>24)&3)+1,nominal_apb1_hz=apb1_hz,
                nominal_bitrate=apb1_hz/(prescaler*(1+segment1+segment2)),
                sample_point=(1+segment1)/(1+segment1+segment2))


def simultaneous_data_frames(first_id,first_payload,second_id,second_payload):
    """仅检查两帧同时起发这一假设；不仿真电气波形或证明现场重叠。"""
    if first_id!=second_id:
        return dict(arbitration_can_distinguish=True,divergence='identifier',physical_overlap_proven=False)
    divergence='dlc' if len(first_payload)!=len(second_payload) else 'data' if first_payload!=second_payload else 'none'
    return dict(arbitration_can_distinguish=False,divergence=divergence,
                differing_after_arbitration=divergence!='none',physical_overlap_proven=False)


def reply_barrier_model(reply_delay_ms, count=3, reply_timeout_ms=100.):
    """条件性反例：同一节点的三次不同查询，延迟从请求SOF算起；不仿真波形。"""
    if reply_delay_ms < .2 or reply_timeout_ms <= 0:
        raise ValueError('回复不能早于假设的0.2ms请求帧，期限必须为正')
    periodic=[5.*index for index in range(count)]
    periodic_replies=[start+reply_delay_ms for start in periodic]
    # 只统计完全相同起发时刻，不能把总线忙期间的普通重叠当作碰撞。
    same_sof=[start for start in periodic[1:] if start in periodic_replies]
    paced=[]; start=0.
    for _ in range(count):
        paced.append(start)
        if reply_delay_ms > reply_timeout_ms:
            return dict(periodic_same_sof_ms=same_sof,paced_requests_ms=paced,
                        paced_status='reply_timeout_stop',physical_proof=False)
        start+=reply_delay_ms+.25+2.  # 假设回复帧0.25ms；回复结束再保留原2ms周转。
    paced_replies=[value+reply_delay_ms for value in paced]
    return dict(periodic_same_sof_ms=same_sof,paced_requests_ms=paced,
                paced_same_sof_ms=[value for value in paced[1:] if value in paced_replies],
                paced_status='model_complete',physical_proof=False,
                assumptions='请求帧0.2ms、回复帧0.25ms、回复延迟为人工假设，均非本车实测')


def analyze(ram):
    snapshot=ram['snapshots'][-1]; can=snapshot['can_diag']; bus=snapshot['bus_diag']
    error_esr=esr(can['error_esr']); tsr=unsigned(can['error_tsr'])
    words=can['error_tx_data0'].to_bytes(4,'little')+can['error_tx_data1'].to_bytes(4,'little')
    payload=words[:can['error_tx_length']]
    confirmed=[f'冻结TSR=0x{tsr:08X}：mailbox0完成={bool(tsr&1)}、TXOK={bool(tsr&2)}、ALST={bool(tsr&4)}、TERR={bool(tsr&8)}。',
               f'冻结ESR：LEC={error_esr["lec"]}（{error_esr["lec_name"]}）、TEC低8位={error_esr["tec_low8"]}；无bus-off={not error_esr["bus_off"]}。',
               f'最近错误帧ID=0x{can["error_tx_id"]:03X}，数据{payload.hex(" ").upper()}；为已有只读查询，不是运动指令。',
               f'真实错误{can["errors"]}与纯仲裁丢失{can["arbitration_lost"]}分别累计；RX环溢出{can["rx_overflow"]}、发送超时{can["tx_timeouts"]}。',
               '当前启用TX邮箱/总错误/bus-off通知，未启用LEC细分通知；HAL只报TERR0与ESR保留LEC4可以同时成立。',
               '新旧CAR代码均保留2ms周转、50ms总期限、同一HAL句柄错误归零；真实发送错误不重发，停车锁存保持。']
    return dict(schema=1,domain='offline_can_analysis',hardware_accessed=False,firmware_changed=False,
                authorizes_motion=False,physical_cause_confirmed=False,
                observed_terr=bool(tsr&8) and not bool(tsr&2),bus_locked=bool(bus['locked']),
                nominal_timing=bit_timing(can['btr'],30000000),confirmed=confirmed,
                uncertain=['同ID请求/回包恰好同时起发可在DLC/数据阶段发生位冲突；当前没有故障瞬间RX时间线，尚未证实。',
                           '端接有效阻值、接插件、收发器供电/共地、波形和节点时钟容差仍缺独立测量，不能只认定是接线。',
                           '冻结记录是最近一次错误，不能用它代替首次0x800故障的全部寄存器现场。'],
                topology={'source':'用户确认，未测量','driver_count':6,'addresses':[1,2,3,4,5,6],
                          'termination':'CAN模块端与最远端','approx_length_m':.30},
                hypotheses=simultaneous_data_frames(can['error_tx_id'],payload,can['error_tx_id'],bytes([0x3A,3,0x6B])),
                query_schedule_model=reply_barrier_model(10.),
                next_steps=['回复屏障模型已做离线对照；候选按“发送完成→匹配地址/命令的回复完成→下一查询”，仍需正式调度夹具和现场时序证据。保留原超时与保护，不重发运动。',
                            '下次获准更新时优先补首次CAN错误和故障前TX/RX环形历史；本轮不烧录。',
                            '物理链路检查或对照采集需另行通知；不要现在复位、清故障或移动来重试。'])


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--ram',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--web-record',type=Path)
    args=parser.parse_args()
    report=analyze(json.loads(args.ram.read_text(encoding='utf-8')))
    names=['firmware/User/Device/can_tx.c','firmware/User/Device/bus.c',
           'firmware/User/Board/base_port.c','firmware/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c']
    report['source_sha256']={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in names}
    write_json(args.output,report)
    if args.web_record:
        value=json.loads(args.web_record.read_text(encoding='utf-8'))
        value['analysis']=report
        write_json(args.web_record,value)
    print(json.dumps(dict(observed_terr=report['observed_terr'],physical_cause_confirmed=False,
                          nominal_bitrate=report['nominal_timing']['nominal_bitrate'],hardware_accessed=False)))


if __name__=='__main__': main()
