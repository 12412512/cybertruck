#ifndef COMM_H
#define COMM_H

#include "frame.h"

enum
{
    COMM_HELLO = 1, COMM_OPEN = 2, COMM_HEART = 3,
    COMM_STATE = 4, COMM_DIAG = 5, COMM_STOP = 6
};
enum
{
    COMM_OK = 0, COMM_BOOT = 1, COMM_SESSION = 2, COMM_BUSY = 3,
    COMM_EXPIRED = 4, COMM_FORMAT = 5, COMM_READONLY = 6
};

/* 仅实现通信。无电机、PWM、建零、运动或成功计数接口。 */
void Comm_Init(uint64_t boot, uint32_t now_ms);
void Comm_Tick(uint32_t now_ms, uint8_t key_pressed);
void Comm_Feed(uint8_t byte, uint32_t received_ms, uint32_t now_ms);
uint16_t Comm_Read(uint8_t *wire);
void Comm_Fault(uint32_t reason);

#endif
