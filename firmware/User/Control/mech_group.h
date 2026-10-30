#ifndef MECH_GROUP_H
#define MECH_GROUP_H
#include "mech_axis.h"
#include "mech_servo.h"

enum { GROUP_IDLE, GROUP_RUNNING, GROUP_DONE, GROUP_FAILED, GROUP_CANCELLED };
typedef struct
{
    uint32_t id, zero_version, timeout_ms, mask;
    int32_t target[5]; /* 升降um、伸缩um、料盘/转臂/夹爪mdeg。 */
    uint32_t speed[5]; /* 前两项RPM，后三项mdeg/s。 */
    uint8_t accel[2], accel_set; /* 只适用于ID5/6；旧报文沿用acc=1。 */
} GroupGoal;
typedef struct
{
    Axis *axis;
    Servo *servo;
    Axis axes[2];
    Servo servos[3];
    GroupGoal goal;
    uint32_t state, reason, started_ms;
    int64_t stationary_raw[2];
    uint8_t axis_mask, sending;
} Group;
/* 复用单轴及舵机执行器；无HAL/总线访问，所有输出仍由唯一板级所有者消费。 */
void Group_Init(Group *group, Axis *axis, Servo *servo);
uint32_t Group_Begin(Group *group, const GroupGoal *goal, uint32_t now);
void Group_Poll(Group *group, uint32_t now, uint8_t base_stopped);
void Group_Stop(Group *group, uint32_t reason, uint32_t now);
uint8_t Group_Take(Group *group, BusFrame frames[2], uint32_t now);
void Group_Sent(Group *group, uint8_t ok, uint32_t now);
void Group_Ack(Group *group, const MotorReply *reply, uint32_t now);
uint8_t Group_Pwm(Group *group, unsigned index, ServoOutput *output, uint32_t now);
void Group_Applied(Group *group, unsigned index, const ServoOutput *output, uint8_t ok, uint32_t now);
#endif
