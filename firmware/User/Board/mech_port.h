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
uint8_t MechPort_Output(BusFrame frames[2], uint32_t now);
void MechPort_Sent(uint8_t success, uint32_t now);
uint8_t MechPort_Owned(void);
#endif
