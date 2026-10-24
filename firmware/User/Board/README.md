# Board

CAN、UART、PWM 等硬件适配，封装 HAL。避免其他业务模块重复管理同一外设或中断。

当前`port.c/.h`只管理USART1中断收发和带接收时间的512槽环形缓冲；内部RNG提供启动身份，PC0只读上报。UART错误使会话失效并从帧边界恢复。CAN与PWM仍未启动。
