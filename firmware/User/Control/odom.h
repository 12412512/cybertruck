#ifndef ODOM_H
#define ODOM_H

#include "kin.h"
#include "yaw.h"
#include "motor.h"
#include "imu.h"
#include <stddef.h>

#if defined(ODOM_HOST) && defined(_WIN32)
#define ODOM_API __declspec(dllexport)
#else
#define ODOM_API
#endif

#define ODOM_SLOTS 16U

enum
{
    ODOM_OK = 0, ODOM_CONFIG = 1, ODOM_TIME = 2, ODOM_SOURCE = 3,
    ODOM_STALE = 4, ODOM_JUMP = 5, ODOM_HISTORY = 6, ODOM_YAW = 7
};

typedef struct
{
    KinGeometry geometry;
    double counts_per_rev[4];
    uint8_t address[4];
    int8_t sign[4];
    float yaw_scale_rad;
    int yaw_sign;
    float max_yaw_rad_s;
    float max_rpm;
    double margin_counts;
    uint32_t fresh_ms;
    uint32_t gap_ms;
    uint32_t can_epoch;
    uint32_t imu_epoch;
    uint32_t imu_source_epoch;
    uint32_t calibration;
} OdomConfig;

typedef struct
{
    RxStamp stamp;
    int64_t counts;
} OdomWheel;

typedef struct
{
    OdomWheel samples[ODOM_SLOTS];
    unsigned first;
    unsigned count;
} OdomHistory;

typedef struct
{
    double x_m;
    double y_m;
    double yaw_rad;
    double wheel_yaw_rad;
    uint32_t tick_ms;
    uint32_t origin_ms;
    uint32_t steps;
    uint32_t waits;
    uint32_t fault;
    uint32_t valid;
    uint32_t calibration;
} OdomPose;

typedef struct
{
    OdomConfig config;
    OdomHistory wheels[4];
    YawHistory yaw;
    RxStamp yaw_stamp;
    OdomPose pose;
    uint32_t created_ms;
    float origin_yaw;
    float raw_yaw;
    unsigned yaw_ready;
    unsigned started;
} Odom;

/* 唯一连续观察器；只接显式标定和带时间/代号的数据，不读设备、不发送命令。
 * 首次共同覆盖区间建立局部odom原点，后续动作不清零；不改变IMU设备零点。
 * 四轮先插值到相同时间边界，再用IMU分段旋转积分，整段成功才提交。
 * 输入时间是MCU软件接收时刻，不能据此声称消除了传感器内部延迟/打滑。 */
ODOM_API size_t Odom_Bytes(void);
ODOM_API int Odom_Init(Odom *odom, const OdomConfig *config, uint32_t now_ms);
ODOM_API int Odom_Pos(Odom *odom, const MotorSample *sample, uint32_t now_ms);
ODOM_API int Odom_Yaw(Odom *odom, const ImuSample *sample, uint32_t now_ms);
/* 1本次提交，0等待新共同区间，-1锁存异常；等待不消费半批、不外推。 */
ODOM_API int Odom_Step(Odom *odom, uint32_t now_ms);
ODOM_API int Odom_Read(const Odom *odom, uint32_t now_ms, OdomPose *pose);

#endif
