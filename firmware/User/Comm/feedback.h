#ifndef FEEDBACK_H
#define FEEDBACK_H

#include "stamp.h"

#define FEED_WIRE_SIZE 56U

#if defined(ODOM_HOST) && defined(_WIN32)
#define FEED_API __declspec(dllexport)
#else
#define FEED_API
#endif

enum
{
    FEED_SPEED = 1, FEED_POSITION = 2, FEED_STATUS = 3,
    FEED_ANGLE = 4, FEED_GYRO = 5
};

typedef struct
{
    RxStamp stamp;
    uint32_t source_epoch;
    int64_t values[3];
    uint8_t kind;
    uint8_t address;
} FeedSample;

/* 只编码纯数据，不发送；boot/session/request由外层GC帧管理。
 * 保留原始计数/IMU原始三轴，不携带未知的物理采样时刻。
 * 当前Comm尚未接此接口；0x10反馈请求仅为预留，不改变只读固件能力。 */
FEED_API int Feed_Encode(const FeedSample *sample, uint32_t report_ms, uint8_t output[FEED_WIRE_SIZE]);

#endif
