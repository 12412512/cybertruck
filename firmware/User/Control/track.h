#ifndef TRACK_H
#define TRACK_H

#include "ref.h"
#include "odom.h"

enum
{
    TRACK_IDLE = 0, TRACK_ACCEPTED = 1, TRACK_RUNNING = 2, TRACK_SETTLING = 3,
    TRACK_COMPLETE = 4, TRACK_FAILED = 5, TRACK_CANCELLED = 6
};
enum
{
    TRACK_OK = 0, TRACK_CONFIG = 1, TRACK_STALE = 2, TRACK_SOURCE = 3,
    TRACK_TIME = 4, TRACK_REFERENCE = 5, TRACK_LIMIT = 6, TRACK_TIMEOUT = 7,
    TRACK_FEEDBACK = 8
};

typedef struct
{
    KinGeometry geometry;
    double kp_xy;
    double kp_yaw;
    double kd_gyro;
    double correction_m_s;
    double correction_rad_s;
    double max_rpm;
    double slew_rpm_s;
    double position_tol_m;
    double yaw_tol_rad;
    double still_rpm;
    double still_m_s;
    double still_rad_s;
    double count_drift[4];
    uint32_t fresh_ms;
    uint32_t feedback_gap_ms;
    uint32_t control_gap_ms;
    uint32_t stable_ms;
    uint32_t can_epoch;
    uint32_t imu_epoch;
    uint32_t imu_source_epoch;
    uint32_t calibration;
} TrackConfig;

typedef struct
{
    OdomPose pose;
    RxStamp position[4];
    RxStamp speed[4];
    RxStamp yaw_stamp;
    RxStamp gyro_stamp;
    double counts[4];
    double rpm[4]; /* 已转换为理论轮正向，轮序FL/FR/RL/RR */
    double yaw_rad; /* 与pose同一连续odom航向参考，时间为yaw_stamp */
    double gyro_rad_s;
    uint32_t imu_source_epoch;
} TrackObservation;

typedef struct
{
    uint32_t id;
    uint32_t timeout_ms;
    RefSpec reference;
} TrackGoal;

typedef struct
{
    double rpm[4]; /* 理论轮整数RPM，仅为输出数据，未连接任何发送者 */
    RefPoint reference;
    double position_error_m;
    double yaw_error_rad;
    double scale;
    uint32_t state;
    uint32_t fault;
    uint32_t id;
    uint32_t output_id;
    uint32_t zero_id;
    uint32_t zero_ack_ms;
    uint32_t stable_elapsed_ms;
    uint32_t blockers;
    uint32_t done_ms;
} TrackOutput;

typedef struct
{
    TrackConfig config;
    TrackGoal goal;
    RefPlan plan;
    RefPlan candidate;
    TrackOutput output;
    TrackObservation previous;
    double slew[4];
    double stable_counts[4];
    uint32_t start_ms;
    uint32_t last_ms;
    uint32_t zero_requested_ms;
    uint32_t stable_tick;
    uint32_t have_previous;
    uint32_t have_zero_ack;
    uint32_t stabilizing;
    uint32_t position_active;
    uint32_t yaw_active;
    uint32_t next_output;
} Track;

/* 主循环唯一所有者；候选/冻结计划均静态持有，不把几何表放小栈。
 * 不调用HAL/CAN/PWM，不使能设备；本模块完成只证明输入模型满足门槛。
 * 真实所有者还需在前后检查接收源代、总线故障和执行预算。 */
ODOM_API size_t Track_Bytes(void);
ODOM_API int Track_Init(Track *track, const TrackConfig *config);
/* 1接受，0同ID同内容重复，-2同ID冲突，-3忙，-1非法/锁存异常。 */
ODOM_API int Track_Start(Track *track, const TrackGoal *goal,
                         const TrackObservation *observation, uint32_t now_ms);
ODOM_API int Track_Step(Track *track, const TrackObservation *observation, uint32_t now_ms);
/* 仅由唯一发送管理者在四轮当前零速批次TXOK后调用；接收/入队成功不等于Sent。
 * 本轮只由模拟传输回调，不写CAN；过期/错动作/错序号/重复ACK不推进。 */
ODOM_API int Track_Sent(Track *track, uint32_t id, uint32_t output_id, uint32_t sent_ms);
ODOM_API void Track_Cancel(Track *track);
ODOM_API void Track_Read(const Track *track, TrackOutput *output);
ODOM_API int Track_Reference(const Track *track, uint32_t tick_ms, RefPoint *point);

#endif
