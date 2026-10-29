#include "frame.h"
#include <string.h>

void Frame_Put(uint8_t *data, uint64_t value, unsigned size)
{
    unsigned index;
    for (index = 0; index < size; index++)
    {
        data[index] = (uint8_t)(value >> (index * 8U));
    }
}

uint64_t Frame_Get(const uint8_t *data, unsigned size)
{
    uint64_t value = 0;
    unsigned index;
    for (index = 0; index < size; index++)
    {
        value |= (uint64_t)data[index] << (index * 8U);
    }
    return value;
}

static uint16_t Crc(const uint8_t *data, uint16_t size)
{
    uint16_t value = 0xFFFFU;
    unsigned index;
    unsigned bit;
    for (index = 0; index < size; index++)
    {
        value ^= (uint16_t)data[index] << 8;
        for (bit = 0; bit < 8; bit++)
        {
            value = (uint16_t)((value << 1) ^ ((value & 0x8000U) ? 0x1021U : 0U));
        }
    }
    return value;
}

uint16_t Frame_Encode(const Frame *frame, uint8_t *wire)
{
    uint8_t raw[FRAME_RAW_MAX];
    uint16_t size;
    uint16_t index;
    uint16_t out = 1;
    uint16_t code_at = 0;
    uint8_t code = 1;
    if (frame->size > FRAME_PAYLOAD_MAX)
    {
        return 0;
    }
    raw[0] = 'G';
    raw[1] = 'C';
    raw[2] = 1;
    raw[3] = frame->type;
    Frame_Put(raw + 4, frame->boot, 8);
    Frame_Put(raw + 12, frame->session, 8);
    Frame_Put(raw + 20, frame->request, 4);
    Frame_Put(raw + 24, frame->ttl_ms, 2);
    Frame_Put(raw + 26, frame->size, 2);
    memcpy(raw + 28, frame->data, frame->size);
    size = (uint16_t)(28U + frame->size);
    Frame_Put(raw + size, Crc(raw, size), 2);
    size += 2U;
    /* 最大原始帧小于254字节，编码块无需处理255字节分段。 */
    for (index = 0; index < size; index++)
    {
        if (raw[index] == 0U)
        {
            wire[code_at] = code;
            code_at = out++;
            code = 1;
        }
        else
        {
            wire[out++] = raw[index];
            code++;
        }
    }
    wire[code_at] = code;
    wire[out++] = 0;
    return out;
}

int Frame_Push(FrameRx *rx, uint8_t byte, uint32_t received_ms, Frame *frame)
{
    uint8_t raw[FRAME_RAW_MAX];
    uint16_t input = 0;
    uint16_t output = 0;
    uint16_t count;
    uint8_t code;
    if (byte != 0U)
    {
        if (!rx->used && !rx->dropping)
        {
            rx->started_ms = received_ms;
        }
        if (!rx->dropping)
        {
            if (rx->used >= FRAME_WIRE_MAX - 1U)
            {
                rx->used = 0;
                rx->dropping = 1;
                rx->bad++;
            }
            else
            {
                rx->data[rx->used++] = byte;
            }
        }
        return 0;
    }
    if (rx->dropping)
    {
        rx->dropping = 0;
        rx->used = 0;
        return 0;
    }
    if (!rx->used)
    {
        return 0;
    }
    while (input < rx->used)
    {
        code = rx->data[input++];
        count = (uint16_t)(code - 1U);
        if (!code || input + count > rx->used || output + count > FRAME_RAW_MAX)
        {
            goto bad;
        }
        memcpy(raw + output, rx->data + input, count);
        input += count;
        output += count;
        if (code != 255U && input < rx->used)
        {
            if (output >= FRAME_RAW_MAX)
            {
                goto bad;
            }
            raw[output++] = 0;
        }
    }
    rx->used = 0;
    if (output < 30U || raw[0] != 'G' || raw[1] != 'C' || raw[2] != 1U)
    {
        goto bad;
    }
    count = (uint16_t)Frame_Get(raw + 26, 2);
    if (count > FRAME_PAYLOAD_MAX || output != 30U + count ||
        Crc(raw, (uint16_t)(output - 2U)) != Frame_Get(raw + output - 2U, 2))
    {
        goto bad;
    }
    frame->type = raw[3];
    frame->boot = Frame_Get(raw + 4, 8);
    frame->session = Frame_Get(raw + 12, 8);
    frame->request = (uint32_t)Frame_Get(raw + 20, 4);
    frame->ttl_ms = (uint16_t)Frame_Get(raw + 24, 2);
    frame->size = count;
    memcpy(frame->data, raw + 28, count);
    rx->good++;
    return 1;
bad:
    rx->used = 0;
    rx->bad++;
    return 0;
}
