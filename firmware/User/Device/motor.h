#ifndef MOTOR_H
#define MOTOR_H

#include "stamp.h"

typedef struct
{
    uint32_t id;
    uint32_t received_ms;
    uint32_t sequence;
    uint32_t epoch;
    uint8_t extended;
    uint8_t remote;
    uint8_t length;
    uint8_t data[8];
} MotorFrame;

typedef struct
{
    RxStamp stamp;
    int64_t value;
    uint8_t address;
    uint8_t kind;
} MotorSample;

/* 只解析Emm35速度(RPM)、36位置(65536计数/圈)、3A状态；不提供构帧/查询/使能/运动。
 * 单帧CAN的first=last是MCU完整帧接收时间，不是电机采样时间。地址尚不映射车轮。 */
int Motor_Read(const MotorFrame *frame, uint32_t handled_ms, MotorSample *sample);

#endif
