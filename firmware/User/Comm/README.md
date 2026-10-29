# Comm

`frame.c/.h`负责COBS、CRC与有界解析；`comm.c/.h`负责启动/会话隔离、500毫秒租约、只读状态与独立STOP应答槽。协议见`docs/protocol.md`。

模块不依赖HAL，不访问电机、PWM或业务计数；同一份正式C实现由主机原生编译进行Python互通测试。

`feedback.c/.h`只编码56字节的原始反馈/时间负载，依赖Device的纯时间类型；未接入`comm.c`或发送路径。未来0x10请求为预留，当前固件仍返回READONLY，HELLO能力未改变。规范和离线证据见`docs/feedback_time.md`。
