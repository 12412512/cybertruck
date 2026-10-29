# Board

CAN、UART、PWM 等硬件适配，封装 HAL。避免其他业务模块重复管理同一外设或中断。

`port.c/.h`唯一管理USART1协议收发和USART2 IMU接收，分别保留512槽字节/ISR时间环。RNG提供启动身份，PC0默认只采样；显式准备新会话目标后，通过Start消抖、PG11短鸣和单次事件才可使能。UART错误使会话失效并中止执行。IMU错误推进接收源代，不发送IMU清零指令。

`base_port.c/.h`将唯一CAN调度器接到Exec/Track/Odom。启动先请求停止，按新反馈和稳定窗口确认后才接受配置；显式使能与目标分开。当前零速批次四轮TXOK后才回调Track_Sent，入队不算完成。正常STOP仅清理本地停止锁，真实错误不自动清除。默认PWM关闭，机构动作另受MechPort所有权和互锁限制。最新板端及验收边界见docs/speed_start.md，历史记录不能替代本次状态。
