/* 复用CARCAN13已核对CAN链；本阶段仅改名、归类和增加接收序号。 */
#include "drive.h"
#include "bus_config.h"
#include <string.h>
uint8_t Drive_Build(uint8_t address, uint8_t op, uint8_t dir, uint16_t rpm, uint8_t acc, uint32_t pulses,
                    uint8_t mode, uint8_t enable, BusFrame frames[2])
{
    uint8_t cmd[12], len = 0, offset = 1, packet = 0, n;
    if (!address || !frames)
        return 0;
    cmd[0] = op;
    switch (op)
    {
    case 0xF3:
        if (enable > 1)
            return 0;
        cmd[1] = 0xAB;
        cmd[2] = enable;
        cmd[3] = 0;
        len = 5;
        break;
    case 0xF6:
    case 0xFD:
        if (dir > 1 || rpm > MOTOR_MAX_COMMAND_RPM || mode > 2)
            return 0;
        cmd[1] = dir;
        cmd[2] = (uint8_t)(rpm >> 8);
        cmd[3] = (uint8_t)rpm;
        /* Emm acc=0直接启动；非0每(256-acc)*50us改变1RPM。无毫米换算。 */
        cmd[4] = acc;
        if (op == 0xF6)
        {
            cmd[5] = 0;
            len = 7;
        }
        else
        {
            cmd[5] = (uint8_t)(pulses >> 24);
            cmd[6] = (uint8_t)(pulses >> 16);
            cmd[7] = (uint8_t)(pulses >> 8);
            cmd[8] = (uint8_t)pulses;
            /* 00相对上一输入目标；01绝对零点；02相对当前实时位置 */
            cmd[9] = mode;
            cmd[10] = 0;
            len = 12;
        }
        break;
    case 0xFE:
        cmd[1] = 0x98;
        cmd[2] = 0;
        len = 4;
        break;
    case 0x35:
    case 0x36:
    case 0x3A:
        len = 2;
        break;
    case 0x42:
        cmd[1] = 0x6C;
        len = 3;
        break;
    default:
        return 0;
    }
    cmd[len - 1] = MOTOR_CHECK_BYTE;
    memset(frames, 0, 2 * sizeof(*frames));
    while (offset < len)
    {
        n = (uint8_t)(len - offset);
        if (n > 7)
            n = 7;
        frames[packet].id = ((uint32_t)address << 8) | packet;
        frames[packet].len = (uint8_t)(n + 1);
        frames[packet].data[0] = op;
        memcpy(&frames[packet].data[1], &cmd[offset], n);
        offset = (uint8_t)(offset + n);
        ++packet;
    }
    return packet;
}
uint8_t Drive_Parse(const BusFrame *f, uint8_t address, MotorReply *r)
{
    uint32_t magnitude;
    uint8_t code;
    if (!f || !r || !address || f->id != ((uint32_t)address << 8) || f->len < 3 || f->len > 8 ||
        f->data[f->len - 1] != MOTOR_CHECK_BYTE)
        return 0;
    code = f->data[0];
    memset(r, 0, sizeof(*r));
    r->address = address;
    r->code = code;
    r->tick = f->tick;
    r->rx_sequence = f->rx_sequence;
    if (code == 0x35 && f->len == 5 && f->data[1] <= 1)
    {
        magnitude = ((uint32_t)f->data[2] << 8) | f->data[3];
        if (magnitude > MOTOR_MAX_REPLY_RPM)
            return 0;
        r->value = f->data[1] ? -(int64_t)magnitude : (int64_t)magnitude;
    }
    else if (code == 0x36 && f->len == 7 && f->data[1] <= 1)
    {
        magnitude = ((uint32_t)f->data[2] << 24) | ((uint32_t)f->data[3] << 16) |
                    ((uint32_t)f->data[4] << 8) | f->data[5];
        r->value = f->data[1] ? -(int64_t)magnitude : (int64_t)magnitude;
    }
    else if (code == 0x3A && f->len == 3)
        r->value = f->data[1];
    else if ((code == 0xF3 || code == 0xF6 || code == 0xFD || code == 0xFE) && f->len == 3)
    {
        r->result = f->data[1];
        if (r->result != 0x02 && r->result != 0x12 && r->result != 0x22 && r->result != 0x9F &&
            r->result != 0xE2 && r->result != 0xEE)
            return 0;
    }
    else
        return 0;
    r->valid = 1;
    return 1;
}
void Drive_Reset(MotorConfigRx *rx)
{
    if (rx)
        memset(rx, 0, sizeof(*rx));
}
uint8_t Drive_Config(MotorConfigRx *rx, const BusFrame *f, uint8_t address, MotorConfig *config)
{
    uint8_t packet, count;
    if (!rx || !f || !config || !address || (f->id >> 8) != address || f->len < 2 || f->len > 8 ||
        f->data[0] != 0x42)
        return 2;
    packet = (uint8_t)f->id;
    if (!packet)
    {
        Drive_Reset(rx);
        rx->address = address;
        rx->raw[0] = 0x42;
        rx->used = 1;
    }
    if (rx->address != address || packet != rx->packet || packet > 4 ||
        (packet && (uint32_t)(f->tick - rx->tick) > 50))
    {
        Drive_Reset(rx);
        return 2;
    }
    count = (uint8_t)(f->len - 1);
    if ((packet < 4 && count != 7) || (packet == 4 && count != 3) || rx->used + count > 32)
    {
        Drive_Reset(rx);
        return 2;
    }
    memcpy(&rx->raw[rx->used], &f->data[1], count);
    rx->used = (uint8_t)(rx->used + count);
    rx->tick = f->tick;
    ++rx->packet;
    if (rx->used < 32)
        return 0;
    /* 手册p103字节序号含地址：5电机类型/10细分/11插补/21地址/22校验。 */
    if (rx->raw[1] != 0x21 || rx->raw[2] != 0x15 || rx->raw[19] != address ||
        rx->raw[31] != MOTOR_CHECK_BYTE || (rx->raw[3] != 0x19 && rx->raw[3] != 0x32) || rx->raw[9] > 1 ||
        rx->raw[20] != 0)
    {
        Drive_Reset(rx);
        return 2;
    }
    memset(config, 0, sizeof(*config));
    config->valid = 1;
    config->address = address;
    config->motor_type = rx->raw[3];
    config->microsteps = rx->raw[8] ? rx->raw[8] : 256;
    config->interpolation = rx->raw[9];
    config->check_type = rx->raw[20];
    config->tick = f->tick;
    Drive_Reset(rx);
    return 1;
}
