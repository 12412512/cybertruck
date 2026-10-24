#include "exec.h"
#include "addresses.h"
#include <math.h>
#include <string.h>

#define EXEC_PI 3.14159265358979323846

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
        config->max_rpm < 1 || config->max_rpm > 10 || floor(config->max_rpm) != config->max_rpm ||
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

int Exec_Arm(Exec *exec, uint32_t now_ms)
{
    if (exec->mode != EXEC_OBSERVING || exec->fault || !Observe(exec, now_ms) ||
        !exec->io.ready(now_ms, 0, 0) || exec->io.fault())
    {
        return -1;
    }
    exec->mode = EXEC_ARMING;
    exec->enabled_wheels = 0;
    exec->stop_confirmed = 0;
    exec->arm_ms = now_ms;
    return 1;
}

static int GoalValid(const TrackGoal *goal)
{
    const RefSpec *ref = &goal->reference;
    if (!goal->id || goal->timeout_ms < 1000 || goal->timeout_ms > 20000 || !isfinite(ref->x_m) ||
        !isfinite(ref->y_m) || !isfinite(ref->yaw_rad) || hypot(ref->x_m, ref->y_m) > 0.15001 ||
        fabs(ref->yaw_rad) > EXEC_PI / 18.0 + 1e-6 || ref->linear.velocity > 0.05f ||
        ref->linear.acceleration > 0.10f || ref->linear.jerk > 0.4f || ref->angular.velocity > 0.20f ||
        ref->angular.acceleration > 0.4f || ref->angular.jerk > 2.0f)
    {
        return 0;
    }
    /* 短斜线复用平移；首个弧线仅开放固定朝向的已验证小轮廓。
     * 不开放切线转头、任意弯道或整图，独立位移/航向保护不变。 */
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
    return ref->kind == 2 && ref->x_m == 0 && ref->y_m == 0 && fabs(ref->yaw_rad) >= 0.01;
}

int Exec_Goal(Exec *exec, const TrackGoal *goal, uint32_t now_ms)
{
    int result;
    if (!goal || !GoalValid(goal))
    {
        return -1;
    }
    if (exec->have_goal && goal->id == exec->last_goal.id)
    {
        return memcmp(goal, &exec->last_goal, sizeof(*goal)) == 0 ? 0 : -2;
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
            exec->position[index] = *sample;
            if (exec->config.version)
            {
                (void)Odom_Pos(&exec->odom, sample, now_ms);
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
        exec->angle = *sample;
        if (exec->config.version)
        {
            (void)Odom_Yaw(&exec->odom, sample, now_ms);
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
        if (hypot(exec->observation.pose.x_m - exec->track.plan.origin[0],
                  exec->observation.pose.y_m - exec->track.plan.origin[1]) > 0.20 ||
            fabs(exec->observation.yaw_rad - exec->track.plan.origin[2]) > 0.22)
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
