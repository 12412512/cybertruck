/* 复用CARCAN13已核对CAN链；本阶段仅改名、归类和增加接收序号。 */
#ifndef PROTOCOL_H
#define PROTOCOL_H
#include <stdint.h>
/* rx_sequence是本机有效RX中断序号，非设备事务号；TX构帧时为0，不上总线。 */
typedef struct
{
    uint32_t id, tick, rx_sequence;
    uint8_t len, data[8];
} BusFrame;
typedef struct
{
    uint8_t address, code, result, valid;
    int64_t value; /* Emm: 35=RPM, 36=65536单位/圈；保留完整符号幅值范围 */
    uint32_t tick, rx_sequence, handled_ms;
} MotorReply;
/* op: F3使能, F6速度, FD位置, FE停, 35/36/3A查询。无广播/同步接口。 */
uint8_t Drive_Build(uint8_t address, uint8_t op, uint8_t dir, uint16_t rpm, uint8_t acc, uint32_t pulses,
                    uint8_t mode, uint8_t enable, BusFrame frames[2]);
uint8_t Drive_Parse(const BusFrame *frame, uint8_t address, MotorReply *reply);
typedef struct
{
    uint8_t raw[32], address, used, packet;
    uint32_t tick;
} MotorConfigRx;
typedef struct
{
    uint8_t valid, address, motor_type, interpolation, check_type;
    uint16_t microsteps;
    uint32_t tick;
} MotorConfig;
void Drive_Reset(MotorConfigRx *rx);
/* 0=等分包,1=完整有效,2=错误。33字节串口返回去掉地址后32字节。 */
uint8_t Drive_Config(MotorConfigRx *rx, const BusFrame *frame, uint8_t address, MotorConfig *config);
#endif
