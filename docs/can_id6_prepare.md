# ID6/0x33审查与正常停止重新准备

2026-10-26，候选GC-BASE-0.3.26。仅离线验证与Keil编译；未连接设备、重启服务、清故障、烧录或触发动作。板端最后已知版本仍为0.3.25，原始现场保存在`output/workbench/fault_after_stability_20261026/`，不得使用本候选AXF解释旧板RAM。

## CAN结论

现场首次错误为405037ms、ID0x600、`33 6B`，TERR0/HAL0x1000，查询超时0。前一ID6/3A匹配回复在405025ms；不能混同正常Web停止与18秒后出现的真实CAN错误，也不能认定只是接线或ID6损坏。

对照旧`智能搬运小车/User/Mech/axis.c`与`User/Config/mech_cfg.h`：沿用35/36/3A/33顺序、完成后10ms间隔，0x33请求2字节、回复7字节有符号位置。未发现确定的构帧或推进错误。本轮不盲目提高间隔，不更改500k或250ms反馈期限。

新增ID6/33合成场景：硬件TXOK早于HAL回调、无TXOK旧帧、错地址/错命令/畸形帧、重复回复、TERR。正确回复仍需TXOK及匹配；TERR立即锁存并停止，不能靠回复或重新准备清除。六节点正常/7ms延迟模型最大反馈龄119/239ms；8ms延迟反例259ms仍被250ms保护拒绝。注入TERR仅证明保护路径，未复现其实际产生原因；毫秒日志不能排除微秒级迟到/重复回包或其他总线因素。

发现并加固`Bus_Rearm()`的底层缺口：此前真实错误拒绝依赖调用者；现在CAN、RX、驱动或查询超时错误在底层也拒绝，保留原错误和历史。历史总线夹具相应改为验证故障后禁止重开，并用移除该检查的负对照验证能检出回归。该缺口不等于ID6发送错误根因。

## 正常停止后的明确入口

新增正式命令61：`uint32 操作令牌 + uint32 当前零点版本 + uint8 人工确认(1)`，沿用同boot/session、TTL、去重及结果协议。Web通过唯一设备核心的`prepare`请求使用它；浏览器不打开串口。

仅候选0.3.26在显式机械调试模式支持。要求新鲜四轮停车证据、两轴配置与8项反馈、正常人工停止原因6、STOP发送完成、两轴静止、无真实错误、PWM全部关闭且没有待发/运行机构动作。板端自行复核，任何CAN0x800都拒绝，不把真实故障返回成无限BUSY重试。

接受后仅解除正常STOP的软件准备锁，保留会话、操作去重及动作编号；零点仍无效，舵机参考失效，不使能、不写PWM、不移动。重新取得控制权、人工建零和记录参考均需独立操作。读到准备后的新状态之前，旧状态不能授权动作。新STOP可抢占准备请求；不自动续跑、重抓或张爪。

Web新增“正常停止后重新准备”，使用现有现场确认框。旧0.3.25、真实故障、失联、STOP未确认、PWM保持等情形按钮禁用。PWM保持期间暂不支持此入口，以保留夹持状态；不得为点亮按钮自动关闭PWM。页面资源版本19/机械7/设备5/规则3；本轮未重载现有页面或部署Pi核心，当前在线界面不视为已更新验收。

## 验证与产物

93项Python回归通过，覆盖正式板级26场景、六节点9场景、CAN回包顺序及39组原总线场景等；含完整正常STOP→61→持续只读链路以及真实错误拒绝。215项JS断言通过（远程99+准备8+键盘35+提示73），不访问网络/设备。两份入口JS语法检查通过。Keil编译前后1122项静态检查通过，0错误0警告。

执行：设置`PYTHONPATH=tests;pi`后运行`test_stamp_race test_mech_query_schedule test_mech_prepare test_mech_port test_can_reply_order test_can_turnaround test_can_query_capture test_can_irq_evidence test_can_evidence_analysis test_mech_stop_readonly test_physical_mechanism test_base_startup test_mechanism_ready test_mech_servo_pose`；运行`node tests/workbench_remote.mjs`、`node tests/workbench_input.mjs`、`node tests/mech_guide.mjs`；运行`tools/build.ps1 -Python .venv/Scripts/python.exe`。

回归记录：`output/tests/can_prepare_0326.txt`。候选AXF/HEX、构建日志与摘要：`output/candidates/GC-BASE-0.3.26/`，HEX SHA256 `CD13BE6F40CAD9E61B502C00EF2CC1CD371304744EAE72C4CA205B624AB9AC3D`。这些是离线证据，不是烧录或实车验收。本小节点不单独提交Git。
