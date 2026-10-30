#ifndef MECH_PORT_H
#define MECH_PORT_H
#include "drive.h"

/* 唯一CAN/PWM适配层；启动仅初始化，48显式接入后才查询机构反馈。 */
void MechPort_Init(void);
void MechPort_Feed(const BusFrame *frame);
void MechPort_Run(uint32_t now, uint8_t base_stopped, uint32_t fault);
void MechPort_Stop(uint32_t reason, uint32_t now);
uint8_t MechPort_Command(uint8_t type, const uint8_t *data, uint16_t size,
                         uint8_t *reply, uint16_t *reply_size, uint32_t now);
uint8_t MechPort_Query(BusFrame frames[2], uint32_t now);
void MechPort_QueryAccept(void);
uint8_t MechPort_Output(BusFrame frames[2], uint32_t now);
void MechPort_Sent(uint8_t success, uint32_t now);
uint8_t MechPort_Owned(void);
/* 固定观察姿态的小位移租约；不等于运输/整场路线许可。 */
uint8_t MechPort_Holding(void);
uint32_t MechPort_HoldTicket(void);
uint8_t MechPort_HoldCheck(uint32_t now);
uint8_t MechPort_Handoff(uint8_t type, const uint8_t *data, uint16_t size,
                         uint8_t *reply, uint16_t *reply_size, uint32_t now, uint8_t stopped);
/* 只读当前活动身份，供底盘停车条件失败时冻结取证。 */
uint8_t MechPort_Active(uint32_t *id, uint32_t *channel);
uint8_t MechPort_Manual(void);
/* 人工恢复：仅核对实时静止，关闭PWM；不清故障、不建零、不使能。 */
uint8_t MechPort_CanReset(uint32_t now, uint32_t version);
uint8_t MechPort_PwmOff(void);
#endif
