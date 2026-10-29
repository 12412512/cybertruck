# 启停区2车头+X、首段倒车斜线

2026-10-25。按用户最新确认修正初始姿态；GC-BASE-0.3.19已烧录并只读验收，配置version8，旧version7停用。实测等待本次烧录后的放地确认。

## 统一约定

启停区1、2起始车头均沿地图+X；启停区2先倒车沿-X离开。起点O=(2.25,2.25)m、yaw=0；扫码点(1.20,2.05)m、yaw=0。第一段直接为一条固定朝向斜线：车体x=-1.05m、y=-0.20m，保持航向，无中途停车/转头。其余原料+Y、粗加工-Y、暂存-X及返航朝向自由的约定保持。

config/field_preview.json为唯一初始朝向来源，地图版本更新至nominal-map-4。Simulation.reset从该配置读取，不再额外硬编码启停区2为π；Simulation.run采用本次冻结Plan的origin，避免规划、显示与执行各自假定方向。网页选择器明确显示“车头+X，倒车离开”。

首段直连依赖start2_exit、north_east_road、qr_dock均开放；复用Planner.make和整车扫掠。封路不能被直连绕过，直线有障碍时不使用该候选。后续路径仍由正式规划器生成。没有对电机方向、IMU符号或其余路段简单取反。

## 保存和执行

工作台新保存版本b28a37e0288142afaa3de84a85ba507c，9段、0.30m/s、100RPM、工位间隙50mm、无障碍；旧保存版本保留。基于旧地图朝向的版本需重新规划，不静默当成新姿态恢复。

config/start2_route.json改为start2_reverse_v1、map_origin=(2.25,2.25,0)，配置version8。目标摘要`d32e127280cd9fdffb89a28b97e507095589ad17e203ca10690da51fea444076`，完整包摘要`fce040d2b91e0eed46bf4fb7f862a793b07025591a3bde741938ce36074f76bf`。编译生成同源9段白名单，乱序、改速、跨版本和篡改起点仍拒绝；Python和STM32都拒绝已停用version7。新启停区2入口只接受0.3.19，不允许旧0.3.18降级启动。

PC0稳定松开后按一次、PB0短鸣后一次ARM；同一会话、连续odom，段间停稳，末尾STOP。250ms反馈、500ms保活、CAN错误、偏离、稳定到位和超时保护保持，轮变化率180RPM/s、加速度前馈0。Web与默认服务不会自动触发实车。

## 离线验收

- 原8000工作台重载只涉及PC模型，草稿/示教保留；重新规划、保存并完整运行9段，页面显示85.82秒、模型停稳、故障0。已恢复起点预览(2.25,2.25,0)，释放控制权。
- 保存包参考及独立Plant实际扫掠通过。正式Start/Exec/Odom/Track与声明轮参数、IMU负号模型9段通过，一次配置/使能、同一会话、最终STOP确认；总模型89.028秒含约2秒按键等待，44514个2ms位置样本无碰撞，完整停入启停区2。不能用模型时间代替实测。
- 60项执行、20项场景、32项导航测试通过；含首段负X/负Y/零转角、封路/障碍、改速/乱序、错误起始朝向、旧固件/旧版本拒绝。Keil零错误零警告。
- 扫掠仍按已保存的前后145mm、左右135mm包络和名义场地检查，body.complete=false保留，不冒充载物完整外形或独立定位精度验收。

证据：output/tests/start2_reverse_execution_final.log、start2_reverse_scene_final.log、start2_reverse_navigation.log；output/start2/workbench_replay.json、exec_with_pc0.json、reverse_ui_complete.json、reverse_workbench_map.png、reverse_workbench_initial.png。完整UI模型结果为85.82秒；参考运动加预留显示86.48秒，两者口径不同。

候选HEX SHA256：`24261ad5a5a948afaff9fa32aaab39a36c06379a5c214c4208460b0966c3f807`。烧录前已通知，100kHz备份/写入/逐字节读回；完成后先通知再做只读验收，等待用户本次重新确认O距右/上各150mm、车头+X、机构收拢、路线无障碍及可立即断电，然后才准备PC0。旧失败记录见start2_link.md，不自动重放或把旧放地确认沿用到新固件。

100kHz完整备份1MiB后，135472字节写入和读回一致，复位运行；清单output/flash/program_0319_20261025/manifest.json。烧录后先通知，随后SWD不停核只读30.375秒、42样本：CAN错误0、RX增加5947、故障0、四轮0RPM、停车确认有效、使能0、PWM关闭、机构零点无效。CPU保持运行，没有配置、使能、建零或下发目标。证据output/start2/readonly_0319_swd.json；1074项工程检查通过。当前仅为烧录/只读通过，不作为实车跑图通过。

## 首次实测：主体正常，入区未通过

用户在本次烧录后确认起点/车头+X/净空，按PC0后9段全部板端完成，最终STOP确认。首段实际odom约(-1.0524,-0.2012)m，车头基本保持。首个GOAL至最终停车80.51095秒，同一boot/session、配置及使能各1次、动作ID1～9、全程已记录odom有效、故障0/CAN错误0。原始trial-072f41648cb749ea898723148aa9a694.jsonl归档于output/start2/physical_0319_trial.jsonl，摘要physical_0319_summary.json。

用户观察主体路线正常，但没有完整返回蓝区，随后描述纵向入区不足约70mm；真实O位置尚未量尺。本轮场地验收失败，不能称为完整跑图通过。名义目标(2.249,2.239)m，板端最终局部odom约(-0.0010,-0.0108)m；按模型计算蓝区左/下边仅余3.4/3.6mm。模型内部停入不能替代实物，窄余量对累计位置/摆放误差敏感。

用户允许按同一版本复测。先恢复同一起点，不修改末段或直接加70mm补偿；待比较误差方向及重复性后，再决定目标居中、精确标定或外部位置校正。每轮仍由新准备后的PC0触发，不自动续跑；小测试不单独提交Git。

## 复测中断与串口缺失

用户恢复同一起点并再次按PC0，第二次trial-7cf048f50c3144528b0bec765fc0972d仅完成首段，随后执行反馈丢失，STOP请求未收到板端确认。已立即通知现场停车，用户确认目视静止，补充“刚刚卡到线了”。本轮未完整执行，不纳入返航70mm的重复性比较。原始证据physical_0319_repeat_cable_interrupted.jsonl、physical_0319_repeat_cable_stderr.log、repeat_cable_summary.json均在output/start2。

用户明确要求重新开始并确认解除卡线/恢复摆放后，重新核对无串口所有者，显式启动新准备进程；本次在open_transport之前因by-id串口路径不存在退出，空trial-5b8ed0f5248e4fd2b000cd1c3e280659未发握手/配置/使能/目标。未继续自动尝试。内核同期有USB过流状态变化、CH340断开，电气原因未定；已请用户恢复车载树莓派↔STM32数据线并补供电方式，区分电脑↔ST-Link调试线。证据after_cable_prepare_failed.log、cable_interruption_diag.txt。恢复后先确认稳定通信，再重新准备PC0，不从旧中断动作续跑。
