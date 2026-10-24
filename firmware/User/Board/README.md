# Board

CAN、UART、PWM 等硬件适配，封装 HAL。避免其他业务模块重复管理同一外设或中断。

`port.c/.h`唯一管理USART1协议收发和USART2 IMU接收，分别保留512槽字节/ISR时间环。RNG提供启动身份，PC0只读；UART错误使会话失效并中止执行。IMU错误推进接收源代，不发送IMU清零指令。

`base_port.c/.h`将唯一CAN调度器接到Exec/Track/Odom。启动先请求四轮停止，按新反馈和稳定窗口确认后才接受配置；显式使能与目标分开。当前零速批次四轮TXOK后才回调Track_Sent，入队不算完成。正常STOP仅清理本地停止锁，真实错误不自动清除。PWM不启动，未烧录或进行本次板端验收。
