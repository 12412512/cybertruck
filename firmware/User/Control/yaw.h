#ifndef YAW_H
#define YAW_H

#include <stdint.h>

#define YAW_SLOTS 64U

typedef enum
{
    YAW_OK = 0,
    YAW_CONFIG = 1,
    YAW_INPUT = 2,
    YAW_TIME = 3,
    YAW_GAP = 4,
    YAW_JUMP = 5
} YawFault;

typedef struct
{
    uint32_t tick_ms;
    float yaw_rad;
} YawSample;

typedef struct
{
    YawSample samples[YAW_SLOTS];
    unsigned first;
    unsigned count;
    float last_raw_rad;
    float max_rate_rad_s;
    uint32_t max_gap_ms;
    YawFault fault;
} YawHistory;

/* 输入是已在设备边界转换好的CCW正弧度，范围[-pi,pi]。
 * 仅建立本地历史，不给IMU写零；正式任务不能每段调用Init清累计角。 */
int Yaw_Init(YawHistory *history, uint32_t tick_ms, float yaw_rad,
             float max_rate_rad_s, uint32_t max_gap_ms);
int Yaw_Push(YawHistory *history, uint32_t tick_ms, float yaw_rad);
/* 仅插值：1有效，0等待未来样本，-1错误/早于保留区间。失败时输出0。 */
int Yaw_At(const YawHistory *history, uint32_t tick_ms, float *yaw_rad);
int Yaw_Fresh(const YawHistory *history, uint32_t now_ms);

#endif
