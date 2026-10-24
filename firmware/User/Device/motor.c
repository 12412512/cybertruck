#include "motor.h"
#include <string.h>

int Motor_Read(const MotorFrame *frame, uint32_t handled_ms, MotorSample *sample)
{
    uint32_t magnitude;
    uint32_t address;
    MotorSample decoded;
    if (!sample)
    {
        return 0;
    }
    memset(sample, 0, sizeof(*sample));
    if (!frame || frame->extended != 1 || frame->remote || !frame->epoch ||
        frame->length < 3 || frame->length > 8 || frame->data[frame->length - 1] != 0x6B)
    {
        return 0;
    }
    address = frame->id >> 8;
    if (!address || address > 255 || (frame->id & 255))
    {
        return 0;
    }
    memset(&decoded, 0, sizeof(decoded));
    decoded.address = (uint8_t)address;
    decoded.kind = frame->data[0];
    decoded.stamp.first_ms = frame->received_ms;
    decoded.stamp.last_ms = frame->received_ms;
    decoded.stamp.handled_ms = handled_ms;
    decoded.stamp.sequence = frame->sequence;
    decoded.stamp.epoch = frame->epoch;
    if (!Stamp_Check(&decoded.stamp, handled_ms, 250))
    {
        return 0;
    }
    if (decoded.kind == 0x35 && frame->length == 5 && frame->data[1] <= 1)
    {
        magnitude = ((uint32_t)frame->data[2] << 8) | frame->data[3];
        /* 沿用旧协议回复数值保护；这不是允许发送的转速。 */
        if (magnitude > 5000)
        {
            return 0;
        }
        decoded.value = frame->data[1] ? -(int64_t)magnitude : magnitude;
    }
    else if (decoded.kind == 0x36 && frame->length == 7 && frame->data[1] <= 1)
    {
        magnitude = ((uint32_t)frame->data[2] << 24) | ((uint32_t)frame->data[3] << 16) |
            ((uint32_t)frame->data[4] << 8) | frame->data[5];
        decoded.value = frame->data[1] ? -(int64_t)magnitude : magnitude;
    }
    else if (decoded.kind == 0x3A && frame->length == 3)
    {
        decoded.value = frame->data[1];
    }
    else
    {
        return 0;
    }
    *sample = decoded;
    return 1;
}
