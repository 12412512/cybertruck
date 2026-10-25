#include "track.h"
#include <math.h>
#include <string.h>

#define TRACK_HALF 0x80000000UL

uint32_t Track_ConfigBytes(void)
{
    return (uint32_t)sizeof(TrackConfig);
}

static void Publish(Track *track)
{
    track->output.output_id = ++track->next_output;
    if (!track->output.output_id)
    {
        track->output.output_id = ++track->next_output;
    }
}

static void Zero(Track *track)
{
    unsigned index;
    for (index = 0; index < 4; index++)
    {
        track->output.rpm[index] = 0;
        track->slew[index] = 0;
    }
}

static int Fail(Track *track, unsigned reason)
{
    Zero(track);
    /* 失败也发布新零速版本，消费者不能因旧序号而漏掉停止输出。 */
    Publish(track);
    track->output.state = TRACK_FAILED;
    track->output.fault = reason;
    track->output.done_ms = 0;
    track->stabilizing = 0;
    return -1;
}

static int Config(const TrackConfig *config)
{
    float rpm[4];
    KinBody zero = {0};
    double values[16];
    unsigned index;
    if (!config || !Kin_Inverse(config->geometry, zero, rpm))
    {
        return 0;
    }
    if (!isfinite(config->lag_ff_s) || config->lag_ff_s < 0 || config->lag_ff_s > .15)
        return 0;
    values[0] = config->kp_xy;
    values[1] = config->kp_yaw;
    values[2] = config->kd_gyro;
    values[3] = config->correction_m_s;
    values[4] = config->correction_rad_s;
    values[5] = config->max_rpm;
    values[6] = config->slew_rpm_s;
    values[7] = config->position_tol_m;
    values[8] = config->yaw_tol_rad;
    values[9] = config->still_rpm;
    values[10] = config->still_m_s;
    values[11] = config->still_rad_s;
    for (index = 0; index < 4; index++)
    {
        values[12 + index] = config->count_drift[index];
    }
    for (index = 0; index < 16; index++)
    {
        if (!isfinite(values[index]) || values[index] < 0)
        {
            return 0;
        }
    }
    if (config->correction_m_s <= 0 || config->correction_rad_s <= 0 ||
        config->max_rpm < 1 || config->max_rpm > 3000 || floor(config->max_rpm) != config->max_rpm ||
        config->slew_rpm_s <= 0 || config->position_tol_m <= 0 || config->yaw_tol_rad <= 0 ||
        config->still_rpm <= 0 || config->still_m_s <= 0 || config->still_rad_s <= 0 ||
        !config->fresh_ms || config->fresh_ms > 250 || !config->feedback_gap_ms ||
        config->feedback_gap_ms > config->fresh_ms || !config->control_gap_ms ||
        config->control_gap_ms > config->fresh_ms || !config->stable_ms ||
        config->stable_ms >= TRACK_HALF || !config->can_epoch || !config->imu_epoch ||
        !config->imu_source_epoch || !config->calibration)
    {
        return 0;
    }
    for (index = 0; index < 4; index++)
    {
        if (config->count_drift[index] <= 0)
        {
            return 0;
        }
    }
    return 1;
}

static int SameLimit(CurveLimit first, CurveLimit last)
{
    return first.velocity == last.velocity && first.acceleration == last.acceleration &&
        first.jerk == last.jerk;
}

static int SameGoal(const TrackGoal *first, const TrackGoal *last)
{
    const RefSpec *a = &first->reference;
    const RefSpec *b = &last->reference;
    if (first->id != last->id || first->timeout_ms != last->timeout_ms || a->kind != b->kind ||
        a->x_m != b->x_m || a->y_m != b->y_m || a->yaw_rad != b->yaw_rad || a->tangent != b->tangent ||
        a->path_heading_rad != b->path_heading_rad || a->curve_velocity_m_s != b->curve_velocity_m_s ||
        a->yaw_begin_m != b->yaw_begin_m || a->yaw_end_m != b->yaw_end_m ||
        a->tail_velocity_m_s != b->tail_velocity_m_s ||
        !SameLimit(a->angular, b->angular) || (a->kind != 2 && !SameLimit(a->linear, b->linear)))
    {
        return 0;
    }
    return (a->kind != 3 && a->kind != 4) || (a->path.lead_m == b->path.lead_m && a->path.tail_m == b->path.tail_m &&
        a->path.radius_m == b->path.radius_m && a->path.blend_m == b->path.blend_m &&
        a->path.turn == b->path.turn && a->path.reverse == b->path.reverse);
}

static int Stamp(const Track *track, const RxStamp *sample, const RxStamp *previous,
                 double value, double old_value, uint32_t epoch, uint32_t now)
{
    uint32_t elapsed;
    if (sample->epoch != epoch)
    {
        return TRACK_SOURCE;
    }
    if (!isfinite(value) || !Stamp_Check(sample, now, track->config.fresh_ms))
    {
        return TRACK_STALE;
    }
    if (!previous)
    {
        return TRACK_OK;
    }
    if (sample->sequence == previous->sequence)
    {
        return sample->first_ms == previous->first_ms && sample->last_ms == previous->last_ms &&
            sample->handled_ms == previous->handled_ms && value == old_value ? TRACK_OK : TRACK_TIME;
    }
    if (!Stamp_Next(previous, sample))
    {
        return TRACK_TIME;
    }
    elapsed = sample->last_ms - previous->last_ms;
    if (!elapsed && value != old_value)
    {
        return TRACK_TIME;
    }
    return elapsed <= track->config.feedback_gap_ms ? TRACK_OK : TRACK_STALE;
}

static int Observe(const Track *track, const TrackObservation *sample, uint32_t now, unsigned compare)
{
    const TrackObservation *old = compare ? &track->previous : 0;
    unsigned index;
    int result;
    if (!sample || sample->pose.valid != 1 || sample->pose.fault ||
        sample->pose.calibration != track->config.calibration ||
        sample->imu_source_epoch != track->config.imu_source_epoch)
    {
        return TRACK_SOURCE;
    }
    if (!isfinite(sample->pose.x_m) || !isfinite(sample->pose.y_m) ||
        !isfinite(sample->pose.yaw_rad) || (uint32_t)(now - sample->pose.tick_ms) >= track->config.fresh_ms)
    {
        return TRACK_STALE;
    }
    if (old && (sample->pose.origin_ms != old->pose.origin_ms ||
        (uint32_t)(sample->pose.tick_ms - old->pose.tick_ms) >= TRACK_HALF ||
        (sample->pose.tick_ms == old->pose.tick_ms &&
         (sample->pose.x_m != old->pose.x_m || sample->pose.y_m != old->pose.y_m ||
          sample->pose.yaw_rad != old->pose.yaw_rad || sample->pose.steps != old->pose.steps))))
    {
        return TRACK_TIME;
    }
    for (index = 0; index < 4; index++)
    {
        if (fabs(sample->rpm[index]) > 5000 || fabs(sample->counts[index]) > 4294967295.0)
        {
            return TRACK_FEEDBACK;
        }
        result = Stamp(track, &sample->position[index], old ? &old->position[index] : 0,
                       sample->counts[index], old ? old->counts[index] : 0, track->config.can_epoch, now);
        if (result)
        {
            return result;
        }
        result = Stamp(track, &sample->speed[index], old ? &old->speed[index] : 0,
                       sample->rpm[index], old ? old->rpm[index] : 0, track->config.can_epoch, now);
        if (result)
        {
            return result;
        }
    }
    result = Stamp(track, &sample->yaw_stamp, old ? &old->yaw_stamp : 0,
                   sample->yaw_rad, old ? old->yaw_rad : 0, track->config.imu_epoch, now);
    if (result)
    {
        return result;
    }
    return Stamp(track, &sample->gyro_stamp, old ? &old->gyro_stamp : 0,
                 sample->gyro_rad_s, old ? old->gyro_rad_s : 0, track->config.imu_epoch, now);
}

static int Still(const Track *track, const TrackObservation *sample)
{
    float rpm[4];
    KinBody body;
    unsigned index;
    for (index = 0; index < 4; index++)
    {
        if (fabs(sample->rpm[index]) > track->config.still_rpm)
        {
            return 0;
        }
        rpm[index] = (float)sample->rpm[index];
    }
    return Kin_Forward(track->config.geometry, rpm, &body) &&
        hypot(body.vx_m_s, body.vy_m_s) <= track->config.still_m_s &&
        fabs(body.wz_rad_s) <= track->config.still_rad_s &&
        fabs(sample->gyro_rad_s) <= track->config.still_rad_s;
}

static double Seconds(const Track *track, uint32_t tick)
{
    uint32_t elapsed = tick - track->start_ms;
    return elapsed < TRACK_HALF ? elapsed / 1000.0 : 0;
}

int Track_Reference(const Track *track, uint32_t tick_ms, RefPoint *point)
{
    return track && track->output.id && Ref_At(&track->plan, Seconds(track, tick_ms), point);
}

size_t Track_Bytes(void)
{
    return sizeof(Track);
}

int Track_Init(Track *track, const TrackConfig *config)
{
    TrackConfig input;
    if (!track)
    {
        return 0;
    }
    if (config)
    {
        input = *config;
    }
    memset(track, 0, sizeof(*track));
    track->output.state = TRACK_FAILED;
    track->output.fault = TRACK_CONFIG;
    if (!config || !Config(&input))
    {
        return 0;
    }
    track->config = input;
    track->output.state = TRACK_IDLE;
    track->output.fault = TRACK_OK;
    return 1;
}

int Track_Start(Track *track, const TrackGoal *goal,
                const TrackObservation *observation, uint32_t now_ms)
{
    if (!track || !goal || !goal->id || track->output.fault ||
        track->output.state == TRACK_CANCELLED)
    {
        return -1;
    }
    if (track->output.id == goal->id)
    {
        return SameGoal(goal, &track->goal) ? 0 : -2;
    }
    if (track->output.state == TRACK_ACCEPTED || track->output.state == TRACK_RUNNING ||
        track->output.state == TRACK_SETTLING)
    {
        return -3;
    }
    if ((track->output.id && (uint32_t)(goal->id - track->output.id) >= TRACK_HALF) ||
        !goal->timeout_ms || goal->timeout_ms >= TRACK_HALF ||
        Observe(track, observation, now_ms, 0) || !Still(track, observation) ||
        !Ref_Make(&track->candidate, &goal->reference, observation->pose.x_m,
                  observation->pose.y_m, observation->pose.yaw_rad) ||
        track->candidate.duration_s * 1000 + track->config.stable_ms >= goal->timeout_ms)
    {
        return -1;
    }
    track->plan = track->candidate;
    track->goal = *goal;
    memset(&track->output, 0, sizeof(track->output));
    track->output.id = goal->id;
    track->output.state = TRACK_ACCEPTED;
    track->start_ms = track->last_ms = now_ms;
    track->previous = *observation;
    track->have_previous = 1;
    track->have_zero_ack = track->stabilizing = 0;
    track->position_active = track->yaw_active = 0;
    Zero(track);
    return 1;
}

static double Clamp(double value, double limit)
{
    return value < -limit ? -limit : value > limit ? limit : value;
}

static int Command(Track *track, const TrackObservation *sample, uint32_t now, double dt)
{
    RefPoint position;
    RefPoint heading;
    RefPoint gyro;
    KinBody body;
    float ff[4];
    float rpm[4];
    double x;
    double y;
    double norm;
    double angle;
    double correction;
    double cosine = cos(sample->yaw_rad);
    double sine = sin(sample->yaw_rad);
    double change = 0;
    double scale;
    unsigned index;
    if (!Track_Reference(track, now, &track->output.reference) ||
        !Track_Reference(track, sample->pose.tick_ms, &position) ||
        !Track_Reference(track, sample->yaw_stamp.last_ms, &heading) ||
        !Track_Reference(track, sample->gyro_stamp.last_ms, &gyro))
    {
        return TRACK_REFERENCE;
    }
    /* 前馈用当前时刻；位置、航向和角速度误差分别回到各自反馈时刻。 */
    x = position.x_m - sample->pose.x_m;
    y = position.y_m - sample->pose.y_m;
    norm = hypot(x, y);
    angle = heading.yaw_rad - sample->yaw_rad;
    track->output.position_error_m = hypot(track->plan.target[0] - sample->pose.x_m,
                                           track->plan.target[1] - sample->pose.y_m);
    track->output.yaw_error_rad = fabs(track->plan.target[2] - sample->yaw_rad);
    if (!isfinite(norm) || !isfinite(angle) || !isfinite(track->output.position_error_m))
    {
        return TRACK_FEEDBACK;
    }
    if (norm > track->config.position_tol_m)
    {
        track->position_active = 1;
    }
    else if (norm <= track->config.position_tol_m / 2)
    {
        track->position_active = 0;
    }
    if (fabs(angle) > track->config.yaw_tol_rad)
    {
        track->yaw_active = 1;
    }
    else if (fabs(angle) <= track->config.yaw_tol_rad / 2)
    {
        track->yaw_active = 0;
    }
    x = track->position_active ? x * track->config.kp_xy : 0;
    y = track->position_active ? y * track->config.kp_xy : 0;
    norm = hypot(x, y);
    if (norm > track->config.correction_m_s)
    {
        x *= track->config.correction_m_s / norm;
        y *= track->config.correction_m_s / norm;
    }
    correction = (track->yaw_active ? track->config.kp_yaw * angle : 0) +
        track->config.kd_gyro * (gyro.wz_rad_s - sample->gyro_rad_s);
    if (!isfinite(correction))
    {
        return TRACK_FEEDBACK;
    }
    body.vx_m_s = (float)(cosine * track->output.reference.vx_m_s + sine * track->output.reference.vy_m_s);
    body.vy_m_s = (float)(-sine * track->output.reference.vx_m_s + cosine * track->output.reference.vy_m_s);
    body.wz_rad_s = (float)track->output.reference.wz_rad_s;
    if (track->config.lag_ff_s > 0)
    {
        /* 对车体系目标速度求导，包含旋转坐标的交叉项；不积分加速度定位。 */
        double ax = cosine * track->output.reference.ax_m_s2 + sine * track->output.reference.ay_m_s2 +
                    track->output.reference.wz_rad_s * body.vy_m_s;
        double ay = -sine * track->output.reference.ax_m_s2 + cosine * track->output.reference.ay_m_s2 -
                    track->output.reference.wz_rad_s * body.vx_m_s;
        body.vx_m_s += (float)(track->config.lag_ff_s * ax);
        body.vy_m_s += (float)(track->config.lag_ff_s * ay);
        body.wz_rad_s += (float)(track->config.lag_ff_s * track->output.reference.angular_accel_rad_s2);
    }
    if (!Kin_Inverse(track->config.geometry, body, ff) || Kin_Limit(ff, (float)track->config.max_rpm) < 1)
    {
        return TRACK_LIMIT;
    }
    body.vx_m_s += (float)(cosine * x + sine * y);
    body.vy_m_s += (float)(-sine * x + cosine * y);
    body.wz_rad_s += (float)Clamp(correction, track->config.correction_rad_s);
    if (!Kin_Inverse(track->config.geometry, body, rpm))
    {
        return TRACK_LIMIT;
    }
    scale = Kin_Limit(rpm, (float)track->config.max_rpm);
    if (scale <= 0)
    {
        return TRACK_LIMIT;
    }
    track->output.scale = scale;
    if (track->output.reference.done && track->output.position_error_m <= track->config.position_tol_m &&
        track->output.yaw_error_rad <= track->config.yaw_tol_rad &&
        !track->position_active && !track->yaw_active)
    {
        for (index = 0; index < 4; index++)
        {
            rpm[index] = 0;
        }
    }
    for (index = 0; index < 4; index++)
    {
        if (fabs(rpm[index] - track->slew[index]) > change)
        {
            change = fabs(rpm[index] - track->slew[index]);
        }
    }
    /* 四轮共用变化比例，最后取整最多引入0.5RPM量化误差。 */
    scale = change > track->config.slew_rpm_s * dt ? track->config.slew_rpm_s * dt / change : 1;
    for (index = 0; index < 4; index++)
    {
        track->slew[index] += scale * (rpm[index] - track->slew[index]);
        track->output.rpm[index] = floor(fabs(track->slew[index]) + .5) * (track->slew[index] < 0 ? -1 : 1);
    }
    return TRACK_OK;
}

static uint32_t Oldest(const TrackObservation *sample, uint32_t now)
{
    uint32_t age = now - sample->pose.tick_ms;
    uint32_t next;
    unsigned index;
    for (index = 0; index < 4; index++)
    {
        next = now - sample->position[index].last_ms;
        if (next > age)
        {
            age = next;
        }
        next = now - sample->speed[index].last_ms;
        if (next > age)
        {
            age = next;
        }
    }
    next = now - sample->yaw_stamp.last_ms;
    if (next > age)
    {
        age = next;
    }
    next = now - sample->gyro_stamp.last_ms;
    if (next > age)
    {
        age = next;
    }
    return now - age;
}

static void Arrival(Track *track, const TrackObservation *sample, uint32_t now)
{
    /* 用最慢反馈的时间推进稳定窗口，反复读取缓存不累计停车时间。 */
    uint32_t tick = Oldest(sample, now);
    unsigned index;
    unsigned zero = 1;
    unsigned capture;
    for (index = 0; index < 4; index++)
    {
        if (track->output.rpm[index] != 0)
        {
            zero = 0;
        }
    }
    capture = track->output.reference.done && track->output.position_error_m <= track->config.position_tol_m &&
        track->output.yaw_error_rad <= track->config.yaw_tol_rad &&
        !track->position_active && !track->yaw_active && zero;
    track->output.blockers = track->output.reference.done ? 0 : 1;
    if (!capture)
    {
        track->output.blockers |= 2;
        track->output.zero_id = 0;
        track->output.zero_ack_ms = 0;
        track->have_zero_ack = track->stabilizing = 0;
        track->output.stable_elapsed_ms = 0;
        track->output.state = TRACK_RUNNING;
        return;
    }
    if (!track->output.zero_id)
    {
        track->output.zero_id = track->output.output_id;
        track->zero_requested_ms = now;
        track->have_zero_ack = 0;
    }
    track->output.state = TRACK_SETTLING;
    if (!track->have_zero_ack)
    {
        track->output.blockers |= 4;
    }
    if (!track->have_zero_ack || (uint32_t)(tick - track->output.zero_ack_ms) == 0 ||
        (uint32_t)(tick - track->output.zero_ack_ms) >= TRACK_HALF)
    {
        track->output.blockers |= 8;
        track->stabilizing = 0;
        return;
    }
    if (!Still(track, sample))
    {
        track->output.blockers |= 16;
        track->stabilizing = 0;
        track->output.stable_elapsed_ms = 0;
        return;
    }
    for (index = 0; index < 4; index++)
    {
        if (track->stabilizing && fabs(sample->counts[index] - track->stable_counts[index]) >
            track->config.count_drift[index])
        {
            track->output.blockers |= 32;
            track->stabilizing = 0;
        }
    }
    if (!track->stabilizing)
    {
        track->stable_tick = tick;
        for (index = 0; index < 4; index++)
        {
            track->stable_counts[index] = sample->counts[index];
        }
        track->stabilizing = 1;
    }
    track->output.stable_elapsed_ms = tick - track->stable_tick;
    if (track->output.stable_elapsed_ms < track->config.stable_ms)
    {
        track->output.blockers |= 64;
        return;
    }
    track->output.state = TRACK_COMPLETE;
    track->output.done_ms = now;
}

int Track_Step(Track *track, const TrackObservation *observation, uint32_t now_ms)
{
    uint32_t elapsed;
    int result;
    uint32_t previous_zero;
    if (!track)
    {
        return -1;
    }
    if (track->output.state == TRACK_COMPLETE)
    {
        return 1;
    }
    if (track->output.state != TRACK_ACCEPTED && track->output.state != TRACK_RUNNING &&
        track->output.state != TRACK_SETTLING)
    {
        return -1;
    }
    elapsed = now_ms - track->last_ms;
    if (elapsed >= TRACK_HALF || elapsed > track->config.control_gap_ms)
    {
        return Fail(track, TRACK_TIME);
    }
    result = Observe(track, observation, now_ms, track->have_previous);
    if (result)
    {
        return Fail(track, (unsigned)result);
    }
    if ((uint32_t)(now_ms - track->start_ms) > track->goal.timeout_ms)
    {
        return Fail(track, TRACK_TIMEOUT);
    }
    if (!elapsed)
    {
        return 0;
    }
    previous_zero = track->output.zero_id;
    result = Command(track, observation, now_ms, elapsed / 1000.0);
    if (result)
    {
        return Fail(track, (unsigned)result);
    }
    if (!previous_zero)
    {
        Publish(track);
    }
    Arrival(track, observation, now_ms);
    if (previous_zero && !track->output.zero_id)
    {
        Publish(track);
    }
    track->previous = *observation;
    track->last_ms = now_ms;
    return track->output.state == TRACK_COMPLETE ? 1 : 0;
}

int Track_Sent(Track *track, uint32_t id, uint32_t output_id, uint32_t sent_ms)
{
    if (!track || track->output.state != TRACK_SETTLING || track->output.id != id ||
        !output_id || track->output.zero_id != output_id || track->have_zero_ack ||
        (uint32_t)(sent_ms - track->zero_requested_ms) >= TRACK_HALF ||
        (uint32_t)(sent_ms - track->start_ms) > track->goal.timeout_ms)
    {
        return 0;
    }
    track->have_zero_ack = 1;
    track->output.zero_ack_ms = sent_ms;
    return 1;
}

void Track_Cancel(Track *track)
{
    if (track && (track->output.state == TRACK_ACCEPTED || track->output.state == TRACK_RUNNING ||
                  track->output.state == TRACK_SETTLING))
    {
        Zero(track);
        Publish(track);
        track->output.state = TRACK_CANCELLED;
        track->output.done_ms = 0;
        track->stabilizing = 0;
    }
}

void Track_Read(const Track *track, TrackOutput *output)
{
    if (output)
    {
        memset(output, 0, sizeof(*output));
        if (track)
        {
            *output = track->output;
        }
    }
}
