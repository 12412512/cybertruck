#ifndef STAMP_H
#define STAMP_H

#include <stdint.h>

/* 全部为同一MCU启动中的毫秒，不是传感器内部采样时间。
 * first/last由唯一接收者在收到字节/完整CAN帧时记录；handled在主循环解析时记录。
 * sequence是本机接收序号，epoch是接收连续性代号；0代号表示未知。 */
typedef struct
{
    uint32_t first_ms;
    uint32_t last_ms;
    uint32_t handled_ms;
    uint32_t sequence;
    uint32_t epoch;
} RxStamp;

int Stamp_Check(const RxStamp *stamp, uint32_t now_ms, uint32_t max_age_ms);
/* 同代内序号必须前进；允许同毫秒多帧，拒绝旧帧/逆序。
 * 调用前仍须Stamp_Check；换代须由所有者显式重建历史。 */
int Stamp_Next(const RxStamp *previous, const RxStamp *next);

#endif
