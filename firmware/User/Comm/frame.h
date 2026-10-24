#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>

#define FRAME_PAYLOAD_MAX 128U
#define FRAME_RAW_MAX     158U
#define FRAME_WIRE_MAX    160U

typedef struct
{
    uint8_t type;
    uint64_t boot;
    uint64_t session;
    uint32_t request;
    uint16_t ttl_ms;
    uint16_t size;
    uint8_t data[FRAME_PAYLOAD_MAX];
} Frame;

typedef struct
{
    uint8_t data[FRAME_WIRE_MAX];
    uint16_t used;
    uint8_t dropping;
    uint32_t started_ms;
    uint32_t good;
    uint32_t bad;
} FrameRx;

/* 字节序固定为小端；编码结果含00分隔，不依赖C结构体布局。 */
uint16_t Frame_Encode(const Frame *frame, uint8_t *wire);
int Frame_Push(FrameRx *rx, uint8_t byte, uint32_t received_ms, Frame *frame);
void Frame_Put(uint8_t *data, uint64_t value, unsigned size);
uint64_t Frame_Get(const uint8_t *data, unsigned size);

#endif
