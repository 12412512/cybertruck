#include "exec.h"
#include "addresses.h"
#include "raw_route.h"
#include "tour_route.h"
#include "batch_route.h"
#include "calibration_route.h"
#include <math.h>
#include <string.h>

#define EXEC_PI 3.14159265358979323846

static void Capture(Exec *exec, uint32_t source, uint32_t kind, uint32_t address,
                    const RxStamp *frame, const RxStamp *previous, int64_t value,
                    int64_t previous_value, uint32_t source_epoch, uint32_t now_ms)
{
    ExecFault *fault = &exec->first_fault;
    unsigned index;
    if (fault->valid || !exec->odom.pose.fault)
    {
        return;
    }
    memset(fault, 0, sizeof(*fault));
    fault->fault = exec->odom.pose.fault;
    fault->source = source;
    fault->now_ms = now_ms;
    fault->action_id = exec->last_goal.id;
    fault->kind = kind;
    fault->address = address;
    if (frame)
    {
        fault->frame = *frame;
    }
    if (previous)
    {
        fault->previous = *previous;
    }
    fault->value = value;
    fault->previous_value = previous_value;
    fault->source_epoch = source_epoch;
    for (index = 0; index < 4; index++)
    {
        fault->latest_ms[index] = exec->position[index].stamp.last_ms;
    }
    fault->latest_ms[4] = exec->angle.stamp.last_ms;
    fault->yaw_fault = exec->odom.yaw.fault;
    fault->valid = 1;
}

void Exec_Init(Exec *exec, ExecIo io)
{
    memset(exec, 0, sizeof(*exec));
    exec->io = io;
}

int Exec_Config(Exec *exec, const ExecConfig *config, uint32_t now_ms)
{
    OdomConfig odom = {0};
    TrackConfig track = {0};
    unsigned index;
    const uint8_t addresses[4] = BASE_SAFE_ADDRESSES;
    if (config && exec->mode == EXEC_OBSERVING && memcmp(config, &exec->config, sizeof(*config)) == 0)
    {
        return 0;
    }
    if (!config || exec->mode != EXEC_DISABLED || !config->version || !isfinite(config->max_rpm) ||
        config->max_rpm < 1 || (config->version == 6 ? config->max_rpm != 150 :
        (config->version == 3 || config->version == 4 || config->version == 5) ? config->max_rpm != 100 : config->max_rpm > 10) ||
        floor(config->max_rpm) != config->max_rpm ||
        !isfinite(config->gyro_scale_rad_s) || config->gyro_scale_rad_s <= 0 ||
        config->gyro_scale_rad_s > 0.01f)
    {
        return -1;
    }
    /* 启动中可能已发生重同步；配置显式绑定当前有效代，不能假定永远为1。
     * 配置后再换代则中止，不把旧历史补贴到新代。 */
    if (!Stamp_Check(&exec->angle.stamp, now_ms, 250) || !Stamp_Check(&exec->gyro.stamp, now_ms, 250) ||
        exec->angle.stamp.epoch != exec->gyro.stamp.epoch ||
        exec->angle.source_epoch != exec->gyro.source_epoch || !exec->angle.source_epoch)
    {
        return -1;
    }
    odom.geometry = track.geometry = config->geometry;
    odom.yaw_scale_rad = config->angle_scale_rad;
    odom.yaw_sign = config->yaw_sign;
    odom.max_yaw_rad_s = 1.0f;
    odom.max_rpm = 20;
    odom.margin_counts = 64;
    odom.fresh_ms = odom.gap_ms = 250;
    odom.can_epoch = 1;
    odom.imu_epoch = exec->angle.stamp.epoch;
    odom.imu_source_epoch = exec->angle.source_epoch;
    odom.calibration = config->version;
    track.kp_xy = 1;
    track.kp_yaw = 1;
    track.kd_gyro = 0.1;
    track.correction_m_s = 0.015;
    track.correction_rad_s = 0.10;
    track.max_rpm = config->max_rpm;
    track.slew_rpm_s = 30;
    track.position_tol_m = 0.005;
    track.yaw_tol_rad = 0.035;
    track.still_rpm = 1;
    track.still_m_s = 0.005;
    track.still_rad_s = 0.035;
    track.fresh_ms = track.feedback_gap_ms = 250;
    track.control_gap_ms = 50;
    track.stable_ms = 150;
    track.can_epoch = 1;
    track.imu_epoch = odom.imu_epoch;
    track.imu_source_epoch = odom.imu_source_epoch;
    track.calibration = config->version;
    if (config->version == 3 || config->version == 4 || config->version == 5 || config->version == 6)
    {
        /* 与离线闭环同一档；高轮速仅供下文冻结的两段目标使用。 */
        odom.max_rpm = config->version == 6 ? 180 : 120;
        odom.max_yaw_rad_s = 5;
        track.kp_xy = 2;
        track.kp_yaw = 3;
        track.kd_gyro = 0.15;
        track.correction_m_s = 0.12;
        track.correction_rad_s = 0.7;
        track.slew_rpm_s = 180;
        track.position_tol_m = 0.003;
        track.yaw_tol_rad = 0.012;
    }
    for (index = 0; index < 4; index++)
    {
        if (config->address[index] != addresses[index] ||
            (config->command_sign[index] != 1 && config->command_sign[index] != -1))
        {
            return -1;
        }
        odom.address[index] = config->address[index];
        odom.sign[index] = config->feedback_sign[index];
        odom.counts_per_rev[index] = config->counts_per_rev;
        track.count_drift[index] = 32;
    }
    if (!Odom_Init(&exec->odom, &odom, now_ms) || !Track_Init(&exec->track, &track))
    {
        return -1;
    }
    exec->config = *config;
    exec->mode = EXEC_OBSERVING;
    exec->control_ms = now_ms;
    return 1;
}

static int Observe(Exec *exec, uint32_t now_ms)
{
    unsigned index;
    TrackObservation *sample = &exec->observation;
    memset(sample, 0, sizeof(*sample));
    if (!Odom_Read(&exec->odom, now_ms, &sample->pose) ||
        !Odom_Heading(&exec->odom, now_ms, &sample->yaw_rad, &sample->yaw_stamp))
    {
        return 0;
    }
    for (index = 0; index < 4; index++)
    {
        sample->position[index] = exec->position[index].stamp;
        sample->speed[index] = exec->speed[index].stamp;
        sample->counts[index] = (double)exec->position[index].value;
        sample->rpm[index] = (double)exec->speed[index].value * exec->config.feedback_sign[index];
        if (!Stamp_Check(&sample->position[index], now_ms, 250) ||
            !Stamp_Check(&sample->speed[index], now_ms, 250))
        {
            return 0;
        }
    }
    sample->gyro_stamp = exec->gyro.stamp;
    sample->gyro_rad_s = exec->gyro.axes[2] * exec->config.gyro_scale_rad_s * exec->config.yaw_sign;
    sample->imu_source_epoch = exec->gyro.source_epoch;
    return Stamp_Check(&sample->gyro_stamp, now_ms, 250);
}

int Exec_Ready(Exec *exec, uint32_t now_ms)
{
    unsigned index;
    ExecGate *gate;
    /* 只更新实时观测快照，不使能、不发送CAN；供按键准备与正式使能共用。 */
    if (!exec)
    {
        return 0;
    }
    gate = &exec->gate;
    memset(gate, 0, sizeof(*gate));
    gate->now_ms = now_ms;
    gate->pose_ms = exec->odom.pose.tick_ms;
    gate->yaw_ms = exec->odom.yaw_stamp.last_ms;
    gate->gyro_ms = exec->gyro.stamp.last_ms;
    for (index = 0; index < 4; index++)
    {
        gate->position_ms[index] = exec->position[index].stamp.last_ms;
        gate->speed_ms[index] = exec->speed[index].stamp.last_ms;
    }
    if (exec->mode != EXEC_OBSERVING)
        gate->mask |= 1U;
    if (exec->fault)
        gate->mask |= 2U;
    if (!Observe(exec, now_ms))
    {
        OdomPose pose;
        RxStamp stamp;
        double heading;
        gate->mask |= 4U;
        if (!Odom_Read(&exec->odom, now_ms, &pose))
            gate->mask |= 32U;
        if (!Odom_Heading(&exec->odom, now_ms, &heading, &stamp))
            gate->mask |= 64U;
        for (index = 0; index < 4; index++)
        {
            if (!Stamp_Check(&exec->position[index].stamp, now_ms, 250))
                gate->mask |= 128U;
            if (!Stamp_Check(&exec->speed[index].stamp, now_ms, 250))
                gate->mask |= 256U;
        }
        if (!Stamp_Check(&exec->gyro.stamp, now_ms, 250))
            gate->mask |= 512U;
    }
    if (!exec->io.ready(now_ms, 0, 0))
        gate->mask |= 8U;
    if (exec->io.fault())
        gate->mask |= 16U;
    return !gate->mask;
}

int Exec_Arm(Exec *exec, uint32_t now_ms)
{
    if (!Exec_Ready(exec, now_ms))
    {
        return -1;
    }
    exec->mode = EXEC_ARMING;
    exec->enabled_wheels = 0;
    exec->stop_confirmed = 0;
    exec->arm_ms = now_ms;
    return 1;
}

static int QuarterTurn(const RefSpec *ref)
{
    return ref->kind == 2 && ref->x_m == 0 && ref->y_m == 0 &&
        fabs(ref->yaw_rad - EXEC_PI / 2) <= 1e-6;
}

static unsigned RawIndex(const RefSpec *ref)
{
    unsigned index;
    for (index = 0; index < 2; index++)
    {
        if (memcmp(ref, RawRefs[index], sizeof(*ref)) == 0)
        {
            return index + 1;
        }
    }
    return 0;
}

static unsigned TourIndex(const RefSpec *ref)
{
    unsigned index;
    RefSpec nominal, candidate;
    for (index = 0; index < TOUR_COUNT; index++)
    {
        memcpy(&nominal, TourRefs[index], sizeof(nominal));
        candidate = *ref;
        /* 仅坐标边界的小幅修正；速度、曲率和其余字段逐字节冻结。 */
        if (!isfinite(ref->x_m) || !isfinite(ref->y_m) || !isfinite(ref->yaw_rad) ||
            !isfinite(ref->path_heading_rad) || fabs(ref->x_m - nominal.x_m) > 0.03 ||
            fabs(ref->y_m - nominal.y_m) > 0.03 || fabs(ref->yaw_rad - nominal.yaw_rad) > 0.04 ||
            fabs(atan2(sin(ref->path_heading_rad - nominal.path_heading_rad),
                       cos(ref->path_heading_rad - nominal.path_heading_rad))) > 0.04)
        {
            continue;
        }
        if (nominal.kind == 1 || nominal.kind == 5 || nominal.kind == 6)
        {
            candidate.x_m = nominal.x_m;
            candidate.y_m = nominal.y_m;
        }
        if (nominal.kind == 2 || nominal.kind == 4 || nominal.kind == 5 || nominal.kind == 6)
        {
            candidate.yaw_rad = nominal.yaw_rad;
        }
        if (nominal.kind == 4)
        {
            candidate.path_heading_rad = nominal.path_heading_rad;
        }
        if (memcmp(&candidate, &nominal, sizeof(nominal)) == 0)
        {
            return index + 1;
        }
    }
    return 0;
}

static unsigned BatchIndex(const RefSpec *ref, uint32_t action)
{
    unsigned index;
    RefSpec nominal, candidate;
    if (action == 0 || action > BATCH_COUNT)
    {
        return 0;
    }
    for (index = action - 1; index < action; index++)
    {
        memcpy(&nominal, BatchRefs[index], sizeof(nominal));
        candidate = *ref;
        /* 仅坐标边界的小幅修正；速度、曲率和其余字段逐字节冻结。 */
        if (!isfinite(ref->x_m) || !isfinite(ref->y_m) || !isfinite(ref->yaw_rad) ||
            !isfinite(ref->path_heading_rad) || fabs(ref->x_m - nominal.x_m) > 0.03 ||
            fabs(ref->y_m - nominal.y_m) > 0.03 || fabs(ref->yaw_rad - nominal.yaw_rad) > 0.04 ||
            fabs(atan2(sin(ref->path_heading_rad - nominal.path_heading_rad),
                       cos(ref->path_heading_rad - nominal.path_heading_rad))) > 0.04)
        {
            continue;
        }
        if (nominal.kind == 1 || nominal.kind == 5 || nominal.kind == 6)
        {
            candidate.x_m = nominal.x_m;
            candidate.y_m = nominal.y_m;
        }
        if (nominal.kind == 2 || nominal.kind == 4 || nominal.kind == 5 || nominal.kind == 6)
        {
            candidate.yaw_rad = nominal.yaw_rad;
        }
        if (nominal.kind == 4)
        {
            candidate.path_heading_rad = nominal.path_heading_rad;
        }
        if (memcmp(&candidate, &nominal, sizeof(nominal)) == 0)
        {
            return index + 1;
        }
    }
    return 0;
}

static unsigned FrozenIndex(const RefSpec *ref, uint32_t version, uint32_t action)
{
    return version == 5 ? BatchIndex(ref, action) : version == 4 ? TourIndex(ref) : version == 3 ? RawIndex(ref) : 0;
}

static int CalibrationMatch(const RefSpec *ref)
{
    unsigned index;
    for (index = 0; index < 3; index++)
        if (memcmp(ref, CalibrationRefs[index], sizeof(*ref)) == 0)
            return 1;
    return 0;
}

static int GoalValid(const TrackGoal *goal)
{
    const RefSpec *ref = &goal->reference;
    if (CalibrationMatch(ref))
        return goal->id == 1 && goal->timeout_ms == 10000;
    if (RawIndex(ref) || TourIndex(ref) || BatchIndex(ref, goal->id))
    {
        return goal->id != 0 && goal->timeout_ms == 30000;
    }
    /* 只开放标记起点到north这一条固定朝向低速斜直线，非任意米级目标。 */
    if (ref->kind == 1 && ref->x_m == 1.05f && ref->y_m == -0.13f && ref->yaw_rad == 0 &&
        ref->linear.velocity == 0.018f && ref->linear.acceleration == 0.05f &&
        ref->linear.jerk == 0.2f && ref->angular.velocity == 0.10f &&
        ref->angular.acceleration == 0.3f && ref->angular.jerk == 1.0f &&
        ref->tangent == 0 && ref->path.lead_m == 0 && ref->path.tail_m == 0 &&
        ref->path.radius_m == 0 && ref->path.blend_m == 0 && ref->path.turn == 0 &&
        ref->path.reverse == 0 && ref->path_heading_rad == 0 && ref->curve_velocity_m_s == 0 &&
        ref->yaw_begin_m == 0 && ref->yaw_end_m == 0 && ref->tail_velocity_m_s == 0)
    {
        return goal->id != 0 && goal->timeout_ms == 90000;
    }
    if (!goal->id || goal->timeout_ms < 1000 || goal->timeout_ms > 20000 || !isfinite(ref->x_m) ||
        !isfinite(ref->y_m) || !isfinite(ref->yaw_rad) || hypot(ref->x_m, ref->y_m) > 0.15001 ||
        (fabs(ref->yaw_rad) > EXEC_PI / 18.0 + 1e-6 && !QuarterTurn(ref)) || ref->linear.velocity > 0.05f ||
        ref->linear.acceleration > 0.10f || ref->linear.jerk > 0.4f || ref->angular.velocity > 0.20f ||
        ref->angular.acceleration > 0.4f || ref->angular.jerk > 2.0f)
    {
        return 0;
    }
    /* 只增加独立的逆时针90度低速原地目标，不扩大平移或弧线的转姿权限。 */
    if (QuarterTurn(ref))
    {
        return ref->angular.velocity <= 0.10f && ref->angular.acceleration <= 0.3f &&
            ref->angular.jerk <= 1.0f;
    }
    /* 平移、小弧线和同步转姿分别限定参数；不开放切线转头或整图。 */
    if (ref->kind == 1)
    {
        return ref->yaw_rad == 0 && hypot(ref->x_m, ref->y_m) >= 0.005;
    }
    if (ref->kind == 4)
    {
        return ref->x_m == 0 && ref->y_m == 0 && ref->yaw_rad == 0 && ref->tangent == 0 &&
            ref->path.lead_m == 0.01f && ref->path.tail_m == 0.01f &&
            ref->path.radius_m == 0.06f && ref->path.blend_m == 0.02f &&
            ref->path.turn == 1 && ref->path.reverse == 0 && ref->path_heading_rad == 0 &&
            ref->curve_velocity_m_s > 0 && ref->curve_velocity_m_s <= 0.012f &&
            ref->linear.velocity <= 0.018f && ref->yaw_begin_m == 0 && ref->yaw_end_m == 0 &&
            ref->tail_velocity_m_s == 0;
    }
    if (ref->kind == 5)
    {
        /* 沿起点前方60mm，同时逆时针10度；复用Ref的双S曲线同步采样。 */
        return ref->x_m == 0.06f && ref->y_m == 0 && fabs(ref->yaw_rad - EXEC_PI / 18) <= 1e-6 &&
            ref->linear.velocity <= 0.012f && ref->linear.acceleration <= 0.05f &&
            ref->linear.jerk <= 0.2f && ref->angular.velocity <= 0.06f &&
            ref->angular.acceleration <= 0.2f && ref->angular.jerk <= 0.6f && ref->tangent == 0 &&
            ref->path.lead_m == 0 && ref->path.tail_m == 0 && ref->path.radius_m == 0 &&
            ref->path.blend_m == 0 && ref->path.turn == 0 && ref->path.reverse == 0 &&
            ref->path_heading_rad == 0 && ref->curve_velocity_m_s == 0 &&
            ref->yaw_begin_m == 0 && ref->yaw_end_m == 0 && ref->tail_velocity_m_s == 0;
    }
    return ref->kind == 2 && ref->x_m == 0 && ref->y_m == 0 && fabs(ref->yaw_rad) >= 0.01;
}

int Exec_Goal(Exec *exec, const TrackGoal *goal, uint32_t now_ms)
{
    int result;
    if (!goal || !GoalValid(goal))
    {
        return -1;
    }
    if ((exec->config.version == 6 && !CalibrationMatch(&goal->reference)) ||
        (exec->config.version != 6 && CalibrationMatch(&goal->reference)))
        return -1;
    if (((exec->config.version == 3 || exec->config.version == 4 || exec->config.version == 5) && FrozenIndex(&goal->reference, exec->config.version, goal->id) == 0) ||
        ((exec->config.version != 3 && exec->config.version != 4 && exec->config.version != 5) && (RawIndex(&goal->reference) || TourIndex(&goal->reference) || BatchIndex(&goal->reference, goal->id))) ||
        (goal->reference.kind == 1 && goal->reference.x_m == 1.05f && exec->config.max_rpm > 6))
    {
        return -1;
    }
    if (exec->have_goal && goal->id == exec->last_goal.id)
    {
        return memcmp(goal, &exec->last_goal, sizeof(*goal)) == 0 ? 0 : -2;
    }
    if ((exec->config.version == 3 || exec->config.version == 4 || exec->config.version == 5) &&
        (goal->id != FrozenIndex(&goal->reference, exec->config.version, goal->id) ||
         (goal->id == 1 && exec->have_goal) ||
         (goal->id > 1 && (!exec->have_goal || exec->last_goal.id != goal->id - 1 ||
          FrozenIndex(&exec->last_goal.reference, exec->config.version, exec->last_goal.id) != goal->id - 1))))
    {
        return -2;
    }
    if (exec->have_goal && (uint32_t)(goal->id - exec->last_goal.id) >= 0x80000000UL)
    {
        return -2;
    }
    if (exec->mode != EXEC_READY || exec->fault || !Observe(exec, now_ms))
    {
        return -3;
    }
    result = Track_Start(&exec->track, goal, &exec->observation, now_ms);
    if (result == 1)
    {
        exec->last_goal = *goal;
        exec->have_goal = 1;
        exec->mode = EXEC_ACTIVE;
        exec->control_ms = now_ms;
    }
    return result;
}

void Exec_Motor(Exec *exec, const MotorSample *sample, uint32_t now_ms)
{
    unsigned index;
    const uint8_t addresses[4] = BASE_SAFE_ADDRESSES;
    if (!sample)
    {
        return;
    }
    for (index = 0; index < 4; index++)
    {
        if (sample->address != addresses[index])
        {
            continue;
        }
        if (sample->kind == 0x36 && sample->stamp.sequence != exec->position[index].stamp.sequence)
        {
            MotorSample previous = exec->position[index];
            exec->position[index] = *sample;
            if (exec->config.version)
            {
                (void)Odom_Pos(&exec->odom, sample, now_ms);
                Capture(exec, 1, sample->kind, sample->address, &sample->stamp, &previous.stamp,
                        sample->value, previous.value, 1, now_ms);
            }
        }
        if (sample->kind == 0x35)
        {
            exec->speed[index] = *sample;
        }
        if (sample->kind == 0x3A)
        {
            exec->status[index] = *sample;
        }
    }
}

void Exec_Imu(Exec *exec, const ImuSample *sample, uint32_t now_ms)
{
    if (!sample)
    {
        return;
    }
    if (sample->kind == 0x53)
    {
        ImuSample previous = exec->angle;
        exec->angle = *sample;
        if (exec->config.version)
        {
            (void)Odom_Yaw(&exec->odom, sample, now_ms);
            Capture(exec, 2, sample->kind, 0, &sample->stamp, &previous.stamp,
                    sample->axes[2], previous.axes[2], sample->source_epoch, now_ms);
        }
    }
    if (sample->kind == 0x52)
    {
        exec->gyro = *sample;
    }
}

void Exec_Stop(Exec *exec, uint32_t reason, uint32_t now_ms)
{
    if (exec->mode >= EXEC_STOPPING)
    {
        if (reason && !exec->fault)
        {
            exec->fault = reason;
        }
        return;
    }
    if (exec->mode <= EXEC_OBSERVING)
    {
        /* 未使能时会话结束无需向电机发指令；真实通信/硬件故障仍锁存。 */
        if (reason && reason != 0x200U)
        {
            exec->fault = reason;
            exec->mode = EXEC_LOCKED;
        }
        return;
    }
    exec->fault = reason;
    Track_Cancel(&exec->track);
    exec->mode = EXEC_STOPPING;
    exec->stop_ms = now_ms;
    exec->stop_confirmed = 0;
    exec->io.stop();
}

void Exec_Run(Exec *exec, uint32_t now_ms)
{
    TrackOutput output;
    int16_t rpm[4] = {0};
    unsigned index;
    exec->tick_ms = now_ms;
    if (exec->config.version)
    {
        (void)Odom_Step(&exec->odom, now_ms);
        Capture(exec, 3, 0, 0, 0, 0, 0, 0, 0, now_ms);
    }
    if (exec->mode == EXEC_DISABLED || exec->mode == EXEC_LOCKED)
    {
        return;
    }
    if (exec->mode == EXEC_STOPPING)
    {
        exec->stop_confirmed = (uint32_t)exec->io.stopped(exec->stop_ms);
        if (exec->stop_confirmed && !exec->fault && !exec->io.fault() && exec->io.rearm())
        {
            exec->mode = EXEC_OBSERVING;
            return;
        }
        if (exec->stop_confirmed || (uint32_t)(now_ms - exec->stop_ms) > 3000)
        {
            exec->mode = EXEC_LOCKED;
        }
        return;
    }
    if (exec->io.fault() || exec->odom.pose.fault)
    {
        Exec_Stop(exec, 1, now_ms);
        return;
    }
    if (exec->mode == EXEC_ARMING)
    {
        if ((uint32_t)(now_ms - exec->arm_ms) > 2000)
        {
            Exec_Stop(exec, 2, now_ms);
        }
        else if (exec->enabled_wheels < 4)
        {
            if (exec->io.enable(exec->enabled_wheels))
            {
                exec->enabled_wheels++;
            }
        }
        else if (exec->io.ready(now_ms, 1, exec->arm_ms) && exec->io.start())
        {
            exec->mode = EXEC_READY;
            exec->control_ms = now_ms;
        }
        return;
    }
    if ((uint32_t)(now_ms - exec->control_ms) < 10 || exec->mode == EXEC_OBSERVING)
    {
        return;
    }
    if ((uint32_t)(now_ms - exec->control_ms) > 50 || !Observe(exec, now_ms))
    {
        Exec_Stop(exec, 3, now_ms);
        return;
    }
    exec->control_ms = now_ms;
    if (exec->mode == EXEC_ACTIVE)
    {
        int north = exec->last_goal.reference.kind == 1 &&
            exec->last_goal.reference.x_m == 1.05f && exec->last_goal.reference.y_m == -0.13f;
        unsigned raw = FrozenIndex(&exec->last_goal.reference, exec->config.version, exec->last_goal.id);
        int calibration = exec->config.version == 6 && CalibrationMatch(&exec->last_goal.reference);
        double distance_limit = raw ? 2.0 : north ? 1.10 : calibration ? 1.02 : 0.20;
        double yaw_limit = raw ? fabs(exec->last_goal.reference.yaw_rad) + 0.10 : (north || calibration) ? 0.05 : QuarterTurn(&exec->last_goal.reference) ? EXEC_PI / 2 + 0.10 : 0.22;
        double dx = exec->observation.pose.x_m - exec->track.plan.origin[0];
        double dy = exec->observation.pose.y_m - exec->track.plan.origin[1];
        double heading = exec->track.plan.origin[2];
        double local_x = cos(heading) * dx + sin(heading) * dy;
        double local_y = -sin(heading) * dx + cos(heading) * dy;
        if (raw || calibration)
        {
            RefPoint point;
            double seconds = (uint32_t)(now_ms - exec->track.start_ms) / 1000.0;
            if (!Ref_At(&exec->track.plan, seconds, &point) ||
                hypot(point.x_m - exec->observation.pose.x_m, point.y_m - exec->observation.pose.y_m) > 0.06 ||
                fabs(point.yaw_rad - exec->observation.yaw_rad) > 0.15)
            {
                Exec_Stop(exec, 6, now_ms);
                return;
            }
        }
        /* 独立偏离走廊保护；不因延长目标距离而删除横向和航向限制。 */
        if (calibration && (fabs(local_y) > .02 || local_x < -.01 || local_x > 1.02))
        {
            Exec_Stop(exec, 6, now_ms);
            return;
        }
        if (north && (fabs(0.13 * local_x + 1.05 * local_y) / 1.058017 > 0.02 ||
            local_x < -0.01 || local_x > 1.07))
        {
            Exec_Stop(exec, 6, now_ms);
            return;
        }
        if (hypot(exec->observation.pose.x_m - exec->track.plan.origin[0],
                  exec->observation.pose.y_m - exec->track.plan.origin[1]) > distance_limit ||
            fabs(exec->observation.yaw_rad - exec->track.plan.origin[2]) > yaw_limit)
        {
            Exec_Stop(exec, 6, now_ms);
            return;
        }
        if (Track_Step(&exec->track, &exec->observation, now_ms) < 0)
        {
            Exec_Stop(exec, 4, now_ms);
            return;
        }
        Track_Read(&exec->track, &output);
        for (index = 0; index < 4; index++)
        {
            rpm[index] = (int16_t)(output.rpm[index] * exec->config.command_sign[index]);
        }
        if (!exec->io.send(rpm, output.id, output.zero_id))
        {
            Exec_Stop(exec, 5, now_ms);
        }
        else if (output.state == TRACK_COMPLETE)
        {
            exec->mode = EXEC_READY;
        }
    }
    else if (!exec->io.send(rpm, 0, 0))
    {
        Exec_Stop(exec, 5, now_ms);
    }
}

void Exec_Sent(Exec *exec, uint32_t id, uint32_t zero_id, uint32_t now_ms)
{
    if (exec->mode == EXEC_ACTIVE)
    {
        (void)Track_Sent(&exec->track, id, zero_id, now_ms);
    }
}
