#ifndef IMU_H
#define IMU_H

#include "stamp.h"

#define IMU_SLOTS 16U
#define IMU_PARTIAL_MS 20U

typedef struct
{
    RxStamp stamp;
    uint32_t source_epoch;
    int16_t axes[3];
    uint8_t raw[11];
    uint8_t kind;
} ImuSample;

typedef struct
{
    ImuSample queue[IMU_SLOTS];
    uint32_t ticks[11];
    uint32_t source_epoch;
    uint32_t epoch;
    uint32_t sequence;
    uint32_t previous_ms;
    uint32_t good;
    uint32_t bad;
    uint32_t gaps;
    uint32_t overflow;
    unsigned head;
    unsigned count;
    uint8_t bytes[11];
    uint8_t used;
    uint8_t seen;
} ImuParser;

/* 沿用旧11字节55/52/53校验和协议，只保存三轴原始值，不假设量程或安装方向。
 * 唯一UART接收管理者提交逐字节ISR时间，主循环调用；本模块不打开UART、不向IMU写命令。 */
void Imu_Init(ImuParser *parser, uint32_t source_epoch);
void Imu_Loss(ImuParser *parser, uint32_t source_epoch);
void Imu_Feed(ImuParser *parser, uint8_t byte, uint32_t received_ms,
              uint32_t handled_ms, uint32_t source_epoch);
/* 仅在已排空接收积压后调用，避免把处理延期当成线上字节间隙。 */
void Imu_Tick(ImuParser *parser, uint32_t now_ms);
int Imu_Read(ImuParser *parser, ImuSample *sample);
/* 旧队列记录保留真实标签，使用前必须验证同源/同代和时效。 */
int Imu_Valid(const ImuParser *parser, const ImuSample *sample, uint32_t now_ms);

#endif
