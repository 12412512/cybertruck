# 0.3.33故障恢复入口烧录与只读验收

2026-10-26，用户明确确认CAN线已修复、机构静止空爪并有支撑，授权烧录0.3.33；烧录后再次确认开放Web由用户自行操作。恢复设计见mechanism_recovery.md。

99项相关Python/C回归、263项JS检查、1152工程检查和Keil零错零警告通过。实际烧录前归档旧.32运行现场，正常退出唯一Pi核心PID2577。探针8801490222005A504E413836、UID550040001951333136363733、ID413及1024KiB相符。完整1MiB备份后145680有效字节读回一致，已先通知用户再做只读检查。

HEX SHA256：40C048D365D45C6E82D2795871DBDBD7B89E5823F4864889F85BD6FB0C3C077A。归档output/flash/program_0333_20261026；该板RAM使用其中symbols，不用旧.32符号或当前可能变化的Objects。

启动30.437秒40组SWD通过：CAN0→0、故障0、四轮0RPM、停车有效、使能0、PWM0、零点无效。随后唯一只读核心/home/pi/gongchuang_mech_readonly_0333_20261026 PID3146，仅observe48附着。六节点60.016秒95组1900反馈全部相干有效，轮/轴最大龄均111ms，CAN/查询超时/首次错误0、停车/PWM0。HSE/PLL寄存器检查通过。原8000仅GET监测60.031秒216次同身份，最大状态龄169ms。

用户烧录后确认现场并要求开放本次Web，已通知后正常切为唯一/home/pi/gongchuang_mech_commission_0333_20261026 PID3200，site_ready/commissioning=true。新会话10.563秒14组SWD CAN0→0、故障0、停车/使能0/PWM0/零点无效；原8000显示实车在线、0.3.33、获取控制权入口。PC8091仍是原固定SSH转发PID2608。没有自动claim、使能、建零、记录参考、切模式、动作或点击recover。

用户现在可以在当前Web自行获取控制权、接入机构反馈、建零并核对当前舵机参考。三个舵机目标和参考默认留空。发生故障时先停止并处理原因，勾选现场确认后使用“清除故障并重新连接”；若线路仍有问题或反馈不能确认静止则拒绝。该入口会关闭PWM并重启STM32，正常STOP后的61准备入口继续单独保留。

本轮没有实际点击65恢复按钮。受控复位/完整故障归档/新身份检查已离线覆盖，但整套实板恢复事务仍待用户操作验收；不因本次烧录清空旧RAM而宣称65实板已通过。PE11物理方向及零位仍待小角度核对，不沿用此前发生扯线前的0°参考。

证据output/mechanism-live/recovery_0333_20261026：incident_before_flash、staged、old_core_stop、startup_readonly、deployment、six_readonly/six_summary、web_readonly、commission_deployment、commission_readonly、final_state及web_ready.png。旧断线故障现场仍归sequence_0332_20261026，不删除。小节点不单独Git，不修改用户docs/start2_reverse.md。
