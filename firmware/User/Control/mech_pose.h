#ifndef MECH_POSE_H
#define MECH_POSE_H

#include "mech_axis.h"
#include "mech_servo.h"

enum { POSE_IDLE, POSE_ACCEPTED, POSE_RUNNING, POSE_WAIT_CONFIRM, POSE_STEP_READY, POSE_STOPPING,
       POSE_COMPLETE, POSE_FAILED, POSE_CANCELLED };
enum { POSE_ID = 300, POSE_BUSY, POSE_SEQUENCE, POSE_TIMEOUT, POSE_CANCEL };
enum { POSE_AXIS = 1, POSE_SERVO = 2 };
typedef struct
{
    uint32_t id, zero_version, revision, timeout_ms;
    /* 升降/伸缩微米，转臂/料盘/夹爪毫度；驱动边界内使用整数。 */
    int32_t target[5];
    uint8_t release_confirmed, single_step;
} PoseGoal;
typedef struct
{
    int32_t target;
    uint8_t kind, actuator;
} PoseStep;
typedef struct
{
    Axis *axis;
    Servo *servo;
    PoseGoal goal;
    PoseStep steps[8];
    uint64_t boot, session;
    uint32_t state, reason, started_ms, child_id;
    uint8_t count, index, child_started, stop_confirmed, step_authorized;
} Pose;

/* 只编排已审核的整数姿态，无文件存储、HAL、CAN或PWM入口。
 * Web保存版本不调用Begin；适配层必须显式锁定版本后才能执行。 */
void Pose_Init(Pose *pose, Axis *axis, Servo *servo);
uint32_t Pose_Preview(const Pose *pose, const PoseGoal *goal, PoseStep steps[8], uint8_t *count,
                      uint32_t now_ms);
uint32_t Pose_Begin(Pose *pose, const PoseGoal *goal, uint32_t now_ms);
void Pose_Poll(Pose *pose, uint32_t now_ms, uint8_t base_stopped);
void Pose_Cancel(Pose *pose, uint32_t now_ms);
uint32_t Pose_Next(Pose *pose, uint32_t id, uint32_t revision, uint8_t index, uint32_t now_ms);

#endif
