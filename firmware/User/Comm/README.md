# Comm

`frame.c/.h`负责COBS、CRC与有界解析；`comm.c/.h`负责启动/会话隔离、500毫秒租约、只读状态与独立STOP应答槽。协议见`docs/protocol.md`。

模块不依赖HAL，不访问电机、PWM或业务计数；同一份正式C实现由主机原生编译进行Python互通测试。
