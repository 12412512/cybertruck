# 软件测试

当前`test_readonly.py`覆盖协议互通、坏帧恢复、会话/保活/复位、STOP保留应答、核心独占、Web只读性、双浏览器推送及快照失效。`native.c`是主机测试桥，链接正式`frame.c`和`comm.c`，不链接HAL或设备。

`test_task.py`覆盖全部25,920种合法任务码/计划、两启停区与六色完整模拟、证据负例、模拟与实车计数隔离、事务回滚、进程突然退出和事件重放。任务离线节点合计47项主机回归通过。

后续`test_base.py`直接编译正式C计算库，增加8项运动学/连续航向测试（含500组正逆运算往返）；`test_keil_policy.py`增加4项缺失字段补齐、选项保护、用户设置保留及幂等验证。当前合计59项通过。

`tools/test.ps1`同时运行主机回归和固件静态检查；任务节点发现外部Keil本地配置变化时，固件检查如实失败，没有修改规则绕过。单独验证离线软件可执行上述`unittest discover`命令，不能据此声称Keil基线通过。

上述3项历史偏差现已按用户授权修复；当前静态检查包含本机选项时为678项通过。构建前后均核对保护策略，不以主机测试代替实物验收。

安装`tests/requirements.txt`后，在根目录执行`tools/test.ps1`，或`.venv/Scripts/python.exe -B -m unittest discover -s tests -v`。Zig构建原生C库到`output/tests/`；这项验证不是板端UART、电机停止或实物验收。

正式实现可以被测试引用，正式程序不得从本目录导入运行实现。实物测试摘要归 `docs/`，原始日志和录像归 Git 忽略的 `output/`。

`test_tracking.py`增加18项正式C跟踪/到位回归，闭环覆盖11类轨迹；同时校验独立车体真值、零速发送屏障、最慢新反馈稳定窗口、失败/取消、动作幂等、连续原点及Web/账本隔离。误差结果写`output/tests/tracking_summary.json`，仅为合成模型，非实车精度。运行入口仍为`tools/test.ps1`。

`test_navigation.py`新增20项导航回归，含500组SE(2)往返、北向起点、蓝区通行、未知/封路/无路、转动中间碰撞、斜线中间障碍、正式C速率界、冻结参考、有界检查、连续弧段、两启停区条件规划和同一odom的完整巡回。最终147项主机回归、818项静态检查通过；报告位于output/navigation与output/tests/navigation_summary.json。只证明合成模型，不证明实场可达或180秒目标。
