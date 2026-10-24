#include "odom.h"
#include <float.h>
#include <math.h>
#include <string.h>

#define ODOM_HALF 0x80000000UL
#define ODOM_PI 3.14159265358979323846

static int Fail(Odom *odom, unsigned fault)
{
    odom->pose.fault = fault;
    odom->pose.valid = 0;
    return -1;
}

static const OdomWheel *Wheel(const OdomHistory *history, unsigned index)
{
    return &history->samples[(history->first + index) % ODOM_SLOTS];
}

static int Config(const OdomConfig *config)
{
    float rpm[4];
    KinBody zero = {0};
    unsigned index;
    unsigned other;
    if (!config || !Kin_Inverse(config->geometry, zero, rpm) ||
        !isfinite(config->yaw_scale_rad) || config->yaw_scale_rad <= 0 ||
        config->yaw_scale_rad * 32768.0 > ODOM_PI + 1e-6 ||
        (config->yaw_sign != 1 && config->yaw_sign != -1) ||
        !isfinite(config->max_yaw_rad_s) || config->max_yaw_rad_s <= 0 ||
        !isfinite(config->max_rpm) || config->max_rpm <= 0 ||
        !isfinite(config->margin_counts) || config->margin_counts < 0 ||
        !config->fresh_ms || config->fresh_ms > 250 || !config->gap_ms ||
        config->gap_ms > config->fresh_ms || !config->can_epoch || !config->imu_epoch ||
        !config->imu_source_epoch || !config->calibration)
    {
        return 0;
    }
    for (index = 0; index < 4; index++)
    {
        if (!config->address[index] || !isfinite(config->counts_per_rev[index]) ||
            config->counts_per_rev[index] <= 0 ||
            (config->sign[index] != 1 && config->sign[index] != -1))
        {
            return 0;
        }
        for (other = 0; other < index; other++)
        {
            if (config->address[index] == config->address[other])
            {
                return 0;
            }
        }
    }
    return 1;
}

size_t Odom_Bytes(void)
{
    return sizeof(Odom);
}

int Odom_Init(Odom *odom, const OdomConfig *config, uint32_t now_ms)
{
    OdomConfig input;
    if (!odom)
    {
        return 0;
    }
    if (config)
    {
        input = *config;
    }
    memset(odom, 0, sizeof(*odom));
    odom->pose.fault = ODOM_CONFIG;
    if (!config || !Config(&input))
    {
        return 0;
    }
    odom->config = input;
    odom->created_ms = now_ms;
    odom->pose.calibration = input.calibration;
    odom->pose.fault = ODOM_OK;
    return 1;
}

int Odom_Pos(Odom *odom, const MotorSample *sample, uint32_t now_ms)
{
    OdomHistory *history;
    const OdomWheel *previous;
    double bound;
    double delta;
    uint32_t elapsed;
    unsigned index;
    unsigned slot;
    if (!odom || odom->pose.fault)
    {
        return -1;
    }
    if (!sample || sample->kind != 0x36)
    {
        return Fail(odom, ODOM_SOURCE);
    }
    if (sample->stamp.first_ms != sample->stamp.last_ms)
    {
        return Fail(odom, ODOM_TIME);
    }
    for (index = 0; index < 4 && odom->config.address[index] != sample->address; index++)
    {
    }
    if (index == 4 || sample->stamp.epoch != odom->config.can_epoch)
    {
        return Fail(odom, ODOM_SOURCE);
    }
    if (!Stamp_Check(&sample->stamp, now_ms, odom->config.fresh_ms))
    {
        return Fail(odom, ODOM_STALE);
    }
    if (sample->value < -4294967295LL || sample->value > 4294967295LL)
    {
        return Fail(odom, ODOM_JUMP);
    }
    history = &odom->wheels[index];
    if (history->count)
    {
        previous = Wheel(history, history->count - 1);
        if (previous->stamp.sequence == sample->stamp.sequence)
        {
            if (previous->counts == sample->value && previous->stamp.last_ms == sample->stamp.last_ms &&
                previous->stamp.first_ms == sample->stamp.first_ms &&
                previous->stamp.handled_ms == sample->stamp.handled_ms)
            {
                return 0;
            }
            return Fail(odom, ODOM_TIME);
        }
        if (!Stamp_Next(&previous->stamp, &sample->stamp))
        {
            return Fail(odom, ODOM_TIME);
        }
        elapsed = sample->stamp.last_ms - previous->stamp.last_ms;
        delta = (double)sample->value - previous->counts;
        if (elapsed > odom->config.gap_ms)
        {
            return Fail(odom, ODOM_STALE);
        }
        if (!elapsed)
        {
            if (delta != 0)
            {
                return Fail(odom, ODOM_TIME);
            }
            history->samples[(history->first + history->count - 1) % ODOM_SLOTS].stamp = sample->stamp;
            return 0;
        }
        bound = odom->config.max_rpm * odom->config.counts_per_rev[index] * elapsed / 60000.0 +
            odom->config.margin_counts;
        if (!isfinite(bound) || fabs(delta) > bound)
        {
            return Fail(odom, ODOM_JUMP);
        }
    }
    if (history->count == ODOM_SLOTS)
    {
        history->first = (history->first + 1) % ODOM_SLOTS;
        history->count--;
    }
    slot = (history->first + history->count) % ODOM_SLOTS;
    history->samples[slot].stamp = sample->stamp;
    history->samples[slot].counts = sample->value;
    history->count++;
    return 1;
}

int Odom_Yaw(Odom *odom, const ImuSample *sample, uint32_t now_ms)
{
    float angle;
    if (!odom || odom->pose.fault)
    {
        return -1;
    }
    if (!sample || sample->kind != 0x53 || sample->stamp.epoch != odom->config.imu_epoch ||
        sample->source_epoch != odom->config.imu_source_epoch)
    {
        return Fail(odom, ODOM_SOURCE);
    }
    if (!Stamp_Check(&sample->stamp, now_ms, odom->config.fresh_ms))
    {
        return Fail(odom, ODOM_STALE);
    }
    angle = sample->axes[2] * odom->config.yaw_scale_rad * odom->config.yaw_sign;
    if (odom->yaw_ready)
    {
        if (sample->stamp.sequence == odom->yaw_stamp.sequence)
        {
            if (angle == odom->raw_yaw && sample->stamp.first_ms == odom->yaw_stamp.first_ms &&
                sample->stamp.last_ms == odom->yaw_stamp.last_ms &&
                sample->stamp.handled_ms == odom->yaw_stamp.handled_ms)
            {
                return 0;
            }
            return Fail(odom, ODOM_TIME);
        }
        if (!Stamp_Next(&odom->yaw_stamp, &sample->stamp))
        {
            return Fail(odom, ODOM_TIME);
        }
        if (sample->stamp.last_ms == odom->yaw_stamp.last_ms)
        {
            if (angle != odom->raw_yaw)
            {
                return Fail(odom, ODOM_TIME);
            }
            odom->yaw_stamp = sample->stamp;
            return 0;
        }
        if (!Yaw_Push(&odom->yaw, sample->stamp.last_ms, angle))
        {
            return Fail(odom, ODOM_YAW);
        }
    }
    else if (!Yaw_Init(&odom->yaw, sample->stamp.last_ms, angle,
                      odom->config.max_yaw_rad_s, odom->config.gap_ms))
    {
        return Fail(odom, ODOM_YAW);
    }
    odom->yaw_ready = 1;
    odom->yaw_stamp = sample->stamp;
    odom->raw_yaw = angle;
    return 1;
}

static int Counts(const OdomHistory *history, uint32_t tick, double *counts)
{
    const OdomWheel *first = Wheel(history, 0);
    const OdomWheel *last = Wheel(history, history->count - 1);
    uint32_t offset = tick - first->stamp.last_ms;
    unsigned index;
    if (offset > (uint32_t)(last->stamp.last_ms - first->stamp.last_ms))
    {
        return 0;
    }
    if (!offset)
    {
        *counts = (double)first->counts;
        return 1;
    }
    for (index = 1; index < history->count; index++)
    {
        last = Wheel(history, index);
        if ((uint32_t)(tick - first->stamp.last_ms) <=
            (uint32_t)(last->stamp.last_ms - first->stamp.last_ms))
        {
            *counts = (double)first->counts + ((double)last->counts - first->counts) *
                (uint32_t)(tick - first->stamp.last_ms) /
                (uint32_t)(last->stamp.last_ms - first->stamp.last_ms);
            return isfinite(*counts);
        }
        first = last;
    }
    return 0;
}

static uint32_t Next(const Odom *odom, uint32_t current, uint32_t target)
{
    uint32_t distance = target - current;
    uint32_t delta;
    unsigned wheel;
    unsigned index;
    for (wheel = 0; wheel < 4; wheel++)
    {
        for (index = 0; index < odom->wheels[wheel].count; index++)
        {
            delta = Wheel(&odom->wheels[wheel], index)->stamp.last_ms - current;
            if (delta > 0 && delta < distance)
            {
                distance = delta;
            }
        }
    }
    for (index = 0; index < odom->yaw.count; index++)
    {
        delta = odom->yaw.samples[(odom->yaw.first + index) % YAW_SLOTS].tick_ms - current;
        if (delta > 0 && delta < distance)
        {
            distance = delta;
        }
    }
    return current + distance;
}

static int Latest(const Odom *odom, uint32_t now_ms)
{
    unsigned index;
    if (!odom->yaw_ready || !Stamp_Check(&odom->yaw_stamp, now_ms, odom->config.fresh_ms))
    {
        return 0;
    }
    for (index = 0; index < 4; index++)
    {
        if (!odom->wheels[index].count || !Stamp_Check(
                &Wheel(&odom->wheels[index], odom->wheels[index].count - 1)->stamp,
                now_ms, odom->config.fresh_ms))
        {
            return 0;
        }
    }
    return 1;
}

int Odom_Step(Odom *odom, uint32_t now_ms)
{
    OdomPose result;
    uint32_t newest_age = 0;
    uint32_t oldest_age = UINT32_MAX;
    uint32_t age;
    uint32_t target;
    uint32_t current;
    uint32_t next;
    unsigned wheel;
    unsigned steps = 0;
    double before;
    double after;
    double theta;
    double half;
    double factor;
    float start_yaw;
    float end_yaw;
    float rpm[4];
    KinBody body;
    if (!odom || odom->pose.fault)
    {
        return -1;
    }
    if (!Latest(odom, now_ms))
    {
        if (odom->started || (uint32_t)(now_ms - odom->created_ms) >= odom->config.fresh_ms)
        {
            return Fail(odom, ODOM_STALE);
        }
        odom->pose.waits++;
        return 0;
    }
    for (wheel = 0; wheel < 4; wheel++)
    {
        age = now_ms - Wheel(&odom->wheels[wheel], odom->wheels[wheel].count - 1)->stamp.last_ms;
        if (age > newest_age)
        {
            newest_age = age;
        }
        age = now_ms - Wheel(&odom->wheels[wheel], 0)->stamp.last_ms;
        if (age < oldest_age)
        {
            oldest_age = age;
        }
    }
    age = now_ms - odom->yaw_stamp.last_ms;
    if (age > newest_age)
    {
        newest_age = age;
    }
    age = now_ms - odom->yaw.samples[odom->yaw.first].tick_ms;
    if (age < oldest_age)
    {
        oldest_age = age;
    }
    target = now_ms - newest_age;
    result = odom->pose;
    current = odom->started ? result.tick_ms : now_ms - oldest_age;
    if ((uint32_t)(target - current) >= ODOM_HALF)
    {
        odom->pose.waits++;
        return 0;
    }
    if (!odom->started)
    {
        if (Yaw_At(&odom->yaw, current, &start_yaw) != 1)
        {
            return Fail(odom, ODOM_HISTORY);
        }
        odom->origin_yaw = start_yaw;
        result.origin_ms = current;
    }
    while (current != target)
    {
        if (++steps > 4 * ODOM_SLOTS + YAW_SLOTS + 1)
        {
            return Fail(odom, ODOM_HISTORY);
        }
        next = Next(odom, current, target);
        if (Yaw_At(&odom->yaw, current, &start_yaw) != 1 ||
            Yaw_At(&odom->yaw, next, &end_yaw) != 1)
        {
            return Fail(odom, ODOM_HISTORY);
        }
        for (wheel = 0; wheel < 4; wheel++)
        {
            if (!Counts(&odom->wheels[wheel], current, &before) ||
                !Counts(&odom->wheels[wheel], next, &after))
            {
                return Fail(odom, ODOM_HISTORY);
            }
            rpm[wheel] = (float)((after - before) * odom->config.sign[wheel] * 60.0 /
                                 odom->config.counts_per_rev[wheel]);
        }
        /* Kin_Forward的线性关系用于单位时间的轮增量，数值输出即本段位移。 */
        if (!Kin_Forward(odom->config.geometry, rpm, &body))
        {
            return Fail(odom, ODOM_JUMP);
        }
        half = ((double)end_yaw - start_yaw) / 2;
        theta = ((double)start_yaw + end_yaw) / 2 - odom->origin_yaw;
        factor = fabs(half) < 1e-8 ? 1 : sin(half) / half;
        result.x_m += factor * (cos(theta) * body.vx_m_s - sin(theta) * body.vy_m_s);
        result.y_m += factor * (sin(theta) * body.vx_m_s + cos(theta) * body.vy_m_s);
        result.wheel_yaw_rad += body.wz_rad_s;
        result.yaw_rad = (double)end_yaw - odom->origin_yaw;
        current = next;
    }
    if (!isfinite(result.x_m) || !isfinite(result.y_m) || !isfinite(result.yaw_rad))
    {
        return Fail(odom, ODOM_JUMP);
    }
    if (odom->started && current == odom->pose.tick_ms)
    {
        odom->pose.waits++;
        return 0;
    }
    result.tick_ms = target;
    result.steps++;
    result.valid = 1;
    odom->pose = result;
    odom->started = 1;
    return 1;
}

int Odom_Read(const Odom *odom, uint32_t now_ms, OdomPose *pose)
{
    if (!pose)
    {
        return 0;
    }
    memset(pose, 0, sizeof(*pose));
    if (!odom)
    {
        return 0;
    }
    *pose = odom->pose;
    pose->valid = !pose->fault && odom->started && Latest(odom, now_ms) &&
        (uint32_t)(now_ms - pose->tick_ms) < odom->config.fresh_ms;
    return (int)pose->valid;
}

int Odom_Heading(const Odom *odom, uint32_t now_ms, double *yaw_rad, RxStamp *stamp)
{
    unsigned latest;
    if (!yaw_rad || !stamp)
    {
        return 0;
    }
    *yaw_rad = 0;
    memset(stamp, 0, sizeof(*stamp));
    if (!odom || odom->pose.fault || !odom->started || !odom->yaw_ready ||
        !Stamp_Check(&odom->yaw_stamp, now_ms, odom->config.fresh_ms))
    {
        return 0;
    }
    latest = (odom->yaw.first + odom->yaw.count - 1) % YAW_SLOTS;
    *yaw_rad = (double)odom->yaw.samples[latest].yaw_rad - odom->origin_yaw;
    *stamp = odom->yaw_stamp;
    return 1;
}
