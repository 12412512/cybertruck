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

/* 默认仍为只读；App可绑定唯一执行器，测试和只读设备不绑定。 */
typedef struct
{
    void (*stop)(uint32_t reason, uint32_t now_ms);
    uint8_t (*command)(uint8_t type, const uint8_t *data, uint16_t size,
                       uint8_t *reply, uint16_t *reply_size, uint32_t now_ms);
} CommHooks;
void Comm_Bind(CommHooks hooks);
void Comm_Init(uint64_t boot, uint32_t now_ms);
void Comm_Tick(uint32_t now_ms, uint8_t key_pressed);
void Comm_Feed(uint8_t byte, uint32_t received_ms, uint32_t now_ms);
uint16_t Comm_Read(uint8_t *wire);
void Comm_Fault(uint32_t reason);
uint64_t Comm_Boot(void);
uint64_t Comm_Session(void);

#endif
