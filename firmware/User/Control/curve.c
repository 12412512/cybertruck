#include "curve.h"
#include <float.h>
#include <math.h>
#include <string.h>

/* 数值域来自旧工程的SI换算，仅防止不可分辨相位，不代表实车允许值。 */
#define CURVE_MAX_DISTANCE 1000.0f
#define CURVE_MIN_LIMIT 1.0e-7f
#define CURVE_MAX_VELOCITY 100.0f
#define CURVE_MAX_ACCELERATION 100.0f
#define CURVE_MAX_JERK 10000.0f

static const int jerk_sign[7] = {1, 0, -1, 0, -1, 0, 1};

static int Finite(double x)
{
    return x == x && x <= DBL_MAX && x >= -DBL_MAX;
}

static int Fits(double x)
{
    return Finite(x) && x <= FLT_MAX && x >= -FLT_MAX;
}

int Curve_Make(CurvePlan *p, float distance, CurveLimit l)
{
    double d, tj, ta, tc, v, a, j, threshold, dt, x, next;
    unsigned i;

    if (!p)
    {
        return 0;
    }
    memset(p, 0, sizeof(*p));
    if (!Finite(distance) || !Finite(l.velocity) || !Finite(l.acceleration) || !Finite(l.jerk) ||
        fabs((double)distance) > CURVE_MAX_DISTANCE || l.velocity < CURVE_MIN_LIMIT ||
        l.velocity > CURVE_MAX_VELOCITY || l.acceleration < CURVE_MIN_LIMIT ||
        l.acceleration > CURVE_MAX_ACCELERATION || l.jerk < CURVE_MIN_LIMIT || l.jerk > CURVE_MAX_JERK)
    {
        return 0;
    }

    d = fabs((double)distance);
    p->distance = d;
    p->sign = distance < 0 ? -1.0 : 1.0;
    p->jerk = l.jerk;
    if (d == 0)
    {
        p->valid = 1;
        return 1;
    }

    /* 先求达到速度上限需要的加速相位，再判断是否存在巡航。 */
    tj = (double)l.acceleration / l.jerk;
    if ((double)l.velocity < (double)l.acceleration * tj)
    {
        tj = sqrt((double)l.velocity / l.jerk);
        ta = 0;
    }
    else
    {
        ta = (double)l.velocity / l.acceleration - tj;
    }
    v = (double)l.jerk * tj * (tj + ta);
    threshold = v * (2 * tj + ta);
    tc = 0;
    if (d >= threshold)
    {
        tc = (d - threshold) / v;
    }
    else
    {
        tj = (double)l.acceleration / l.jerk;
        threshold = 2 * (double)l.acceleration * tj * tj;
        if (d >= threshold)
        {
            /* 解D=A*(Ta+Tj)*(Ta+2Tj)，有理化避免阈值附近两大数相减。 */
            ta = 2 * (d / l.acceleration - 2 * tj * tj) / (sqrt(tj * tj + 4 * d / l.acceleration) + 3 * tj);
            if (ta < 0)
            {
                ta = 0;
            }
        }
        else
        {
            /* 先转double再除法，避免极小float距离下溢成零。 */
            tj = pow(d / (2 * (double)l.jerk), 1.0 / 3.0);
            ta = 0;
        }
        v = (double)l.jerk * tj * (tj + ta);
    }

    p->duration[0] = tj;
    p->duration[1] = ta;
    p->duration[2] = tj;
    p->duration[3] = tc;
    p->duration[4] = tj;
    p->duration[5] = ta;
    p->duration[6] = tj;
    p->peak_velocity = v;
    p->peak_acceleration = (double)l.jerk * tj;
    for (i = 0; i < 7; ++i)
    {
        dt = p->duration[i];
        j = (double)jerk_sign[i] * l.jerk;
        x = p->position[i];
        v = p->velocity[i];
        a = p->acceleration[i];
        next = p->time[i] + dt;
        /* 极端长巡航可能吞掉短相位，不能把这样的曲线报为有效。 */
        if (!Finite(dt) || dt < 0 || !Finite(next) ||
            (dt > 0 && (next <= p->time[i] || next * DBL_EPSILON > dt * 1.0e-6 ||
                        fabs((next - p->time[i]) - dt) > dt * 1.0e-6)))
        {
            memset(p, 0, sizeof(*p));
            return 0;
        }
        p->position[i + 1] = x + dt * (v + dt * (a / 2 + dt * j / 6));
        p->velocity[i + 1] = v + dt * (a + dt * j / 2);
        p->acceleration[i + 1] = a + j * dt;
        p->time[i + 1] = next;
    }
    p->total = p->time[7];
    if (!Finite(p->total) || p->total <= 0 || !Finite(p->position[7]) ||
        fabs(p->position[7] - d) > d * 1.0e-9 || p->peak_velocity > (double)l.velocity * (1 + 1.0e-12) ||
        p->peak_acceleration > (double)l.acceleration * (1 + 1.0e-12))
    {
        memset(p, 0, sizeof(*p));
        return 0;
    }
    /* 固定末端为解析静止状态；采样减速时镜像加速段以减少抵消误差。 */
    p->position[7] = d;
    p->velocity[7] = 0;
    p->acceleration[7] = 0;
    p->valid = 1;
    return 1;
}

int Curve_At(const CurvePlan *p, double t, CurvePoint *point)
{
    unsigned i, reverse;
    double u, dt, j, x, v, a;

    if (!point)
    {
        return 0;
    }
    memset(point, 0, sizeof(*point));
    if (!p || p->valid != 1 || !Finite(t) || t < 0)
    {
        return 0;
    }
    if (t >= p->total)
    {
        point->position = (float)(p->sign * p->distance);
        point->done = 1;
        return 1;
    }

    reverse = t > p->total / 2;
    u = reverse ? p->total - t : t;
    /* 对称曲线只需求前半程；循环次数恒定且不随距离增长。 */
    for (i = 0; i < 3; ++i)
    {
        if (u < p->time[i + 1])
        {
            break;
        }
    }
    if (i == 3)
    {
        x = p->distance / 2 + p->peak_velocity * (u - p->total / 2);
        v = p->peak_velocity;
        a = 0;
        j = p->duration[3] > 0 ? 0 : -p->jerk;
    }
    else
    {
        dt = u - p->time[i];
        j = (double)jerk_sign[i] * p->jerk;
        x = p->position[i] + dt * (p->velocity[i] + dt * (p->acceleration[i] / 2 + dt * j / 6));
        v = p->velocity[i] + dt * (p->acceleration[i] + dt * j / 2);
        a = p->acceleration[i] + j * dt;
    }
    if (reverse)
    {
        x = p->distance - x;
        a = -a;
    }
    if (!Fits(x) || !Fits(v) || !Fits(a) || !Fits(j))
    {
        return 0;
    }
    point->position = (float)(p->sign * x);
    point->velocity = (float)(p->sign * v);
    point->acceleration = (float)(p->sign * a);
    point->jerk = (float)(p->sign * j);
    return 1;
}

/* 每段保存同一积分状态。边界只换 jerk，不重置位置/速度/加速度。 */
static int TimeAppend(CurveTiming *p, double dt, double j)
{
    unsigned n = p->count;
    double x, v, a, next;
    if (!Finite(dt) || dt < 0 || !Finite(j))
    {
        return 0;
    }
    if (dt == 0)
    {
        return 1;
    }
    if (n >= CURVE_TIME_PHASES)
    {
        return 0;
    }
    next = p->time[n] + dt;
    if (!Finite(next) || next <= p->time[n] || next * DBL_EPSILON > dt * 1.0e-6)
    {
        return 0;
    }
    x = p->position[n];
    v = p->velocity[n];
    a = p->acceleration[n];
    p->time[n + 1U] = next;
    p->position[n + 1U] = x + dt * (v + dt * (a / 2 + dt * j / 6));
    p->velocity[n + 1U] = v + dt * (a + dt * j / 2);
    p->acceleration[n + 1U] = a + dt * j;
    p->jerk[n] = j;
    p->count = n + 1U;
    return Finite(p->position[n + 1U]) && Finite(p->velocity[n + 1U]) &&
        Finite(p->acceleration[n + 1U]);
}

static void TimeRamp(double from, double to, CurveLimit l, double *tj, double *ta, double *d)
{
    double dv = fabs(to - from);
    *tj = fmin((double)l.acceleration / l.jerk, sqrt(dv / l.jerk));
    *ta = fmax(0, dv / l.acceleration - *tj);
    *d = (from + to) * (2 * (*tj) + *ta) / 2;
}

static int TimeChange(CurveTiming *p, double from, double to, CurveLimit l)
{
    double tj, ta, d, j = to >= from ? l.jerk : -(double)l.jerk;
    TimeRamp(from, to, l, &tj, &ta, &d);
    return TimeAppend(p, tj, j) && TimeAppend(p, ta, 0) && TimeAppend(p, tj, -j);
}

static int LowFits(const CurvePlan *p, double lead_m, double tail_m)
{
    return p->position[3] <= lead_m && p->distance - p->position[4] <= tail_m;
}
/* 一侧直线必须同时容纳“0→峰值”和“峰值→曲线速度”。
 * 调换时间方向即为出弯侧，距离相同；不把短侧的峰值强加给长侧。
 * 只在动作开始前固定48次搜索，在线采样不搜索、不推进子动作。 */
static double Wing(double room, double curve, double requested, CurveLimit limit)
{
    double lo = curve, hi = requested, tj, ta, d0, dc, mid;
    unsigned i;
    TimeRamp(0, requested, limit, &tj, &ta, &d0);
    TimeRamp(requested, curve, limit, &tj, &ta, &dc);
    if (d0 + dc <= room)
    {
        return requested;
    }
    for (i = 0; i < 48U; i++)
    {
        mid = (lo + hi) / 2;
        TimeRamp(0, mid, limit, &tj, &ta, &d0);
        TimeRamp(mid, curve, limit, &tj, &ta, &dc);
        if (d0 + dc <= room)
        {
            lo = mid;
        }
        else
        {
            hi = mid;
        }
    }
    return lo - curve < curve * 1.0e-10 ? curve : lo;
}

int Curve_TimeMake(CurveTiming *p, float distance, float lead_m, float tail_m,                    CurveLimit slow, float fast_velocity, float buffer_m, float tail_velocity)
{
    CurvePlan low;
    double tj, ta, d0, dc, lead, tail, available_in, available_out, lo, hi, mid, curve, b0, b1, v0, v1;
    unsigned i;
    if (!p)
    {
        return 0;
    }
    memset(p, 0, sizeof(*p));
    if (!Finite(distance) || !Finite(lead_m) || !Finite(tail_m) || !Finite(buffer_m) ||
    !Finite(fast_velocity) || !Finite(tail_velocity) || tail_velocity <= 0 || distance <= 0 || lead_m <= 0 || tail_m <= 0 ||
    (double)lead_m + tail_m >= distance || buffer_m < 0 || fast_velocity < slow.velocity ||
    !Curve_Make(&low, distance, slow))
    {
        return 0;
    }
    /* 同时校验高速数值范围，A/J 继续使用原线性限制。 */
    {
        CurveLimit high = slow;
        high.velocity = fast_velocity;
        if (!Curve_Make(&low, distance, high))
        {
            return 0;
        }
    }
    p->distance = distance;
    {
        /* 先确定可行曲线档。短直线缓冲缩为不超过直线一半，给起步/末停
        * 留下正空间；缓冲随短段缩小，避免只剩极小加速距离而退到异常慢档。
        * 缓冲值由调用者按米传入，最低数值速度也放不下则拒绝。 */
        available_in = lead_m - fmin(buffer_m, (double)lead_m / 2);
        available_out = tail_m - fmin(buffer_m, (double)tail_m / 2);
        if (!Curve_Make(&low, distance, slow))
        {
            goto invalid;
        }
        if (!LowFits(&low, available_in, available_out))
        {
            CurveLimit candidate_limit = slow;
            /* 不先试绝对最小速度：长巡航可能使极短末坡失去数值时间分辨率。
            * 先有界折半找到可行档，再用剩余次数逼近；总计不超过48次。 */
            lo = hi = slow.velocity;
            for (i = 0; i < 48U; i++)
            {
                lo = fmax(CURVE_MIN_LIMIT, lo / 2);
                candidate_limit.velocity = (float)lo;
                if (Curve_Make(&low, distance, candidate_limit) && LowFits(&low, available_in, available_out))
                {
                    break;
                }
                hi = lo;
                if (lo <= CURVE_MIN_LIMIT)
                {
                    goto invalid;
                }
            }
            if (i == 48U)
            {
                goto invalid;
            }
            for (i++; i < 48U; i++)
            {
                mid = (lo + hi) / 2;
                candidate_limit.velocity = (float)mid;
                if (Curve_Make(&low, distance, candidate_limit) && LowFits(&low, available_in, available_out))
                {
                    lo = candidate_limit.velocity;
                }
                else
                {
                    hi = mid;
                }
            }
            candidate_limit.velocity = (float)lo;
            if (!Curve_Make(&low, distance, candidate_limit) ||
            !LowFits(&low, available_in, available_out))
            {
                goto invalid;
            }
        }
    }
    curve = fmin(low.peak_velocity, (double)slow.velocity);
    b0 = (double)lead_m - available_in;
    b1 = (double)tail_m - available_out;
    v0 = Wing(available_in, curve, fast_velocity, slow);
    v1 = tail_velocity < curve ? tail_velocity : Wing(available_out, curve, fmin(fast_velocity, tail_velocity), slow);
    TimeRamp(0, v0, slow, &tj, &ta, &d0);
    TimeRamp(v0, curve, slow, &tj, &ta, &dc);
    lead = available_in - d0 - dc;
    p->start_end = d0;
    /* 二分保留可行下界；仅消除积分末位舍入，不移动几何边界。 */
    if (fabs(lead) < (double)distance * 1.0e-10)
    {
        lead = 0;
    }
    if (!TimeChange(p, 0, v0, slow) || !TimeAppend(p, lead / v0, 0) ||
    !TimeChange(p, v0, curve, slow) ||
    !TimeAppend(p, ((double)distance - lead_m - tail_m + b0 + b1) / curve, 0))
    {
        goto invalid;
    }
    TimeRamp(0, v1, slow, &tj, &ta, &d0);
    TimeRamp(v1, curve, slow, &tj, &ta, &dc);
    tail = available_out - d0 - dc;
    p->stop_begin = distance - d0;
    if (fabs(tail) < (double)distance * 1.0e-10)
    {
        tail = 0;
    }
    if (!TimeChange(p, curve, v1, slow) || !TimeAppend(p, tail / v1, 0) ||
    !TimeChange(p, v1, 0, slow))
    {
        goto invalid;
    }
    p->lead_peak = v0;
    p->tail_peak = v1;
    p->peak_velocity = fmax(v0, v1);
    p->curve_velocity = curve;
    p->split = v0 > curve || v1 > curve;
    p->total = p->time[p->count];
    if (!Finite(p->total) || p->total <= 0 ||
    fabs(p->position[p->count]-(double)distance) > (double)distance * 1.0e-9 ||
    fabs(p->velocity[p->count]) > 1.0e-7 || fabs(p->acceleration[p->count]) > 1.0e-7)
    {
        goto invalid;
    }
    p->position[p->count] = distance;
    p->velocity[p->count] = 0;
    p->acceleration[p->count] = 0;
    p->valid = 1;
    return 1;
invalid:
    memset(p, 0, sizeof(*p));
    return 0;
}

int Curve_TimeAt(const CurveTiming *p, double t, CurvePoint *point)
{
    unsigned i;
    double dt, x, v, a, j;
    if (!point)
    {
        return 0;
    }
    memset(point, 0, sizeof(*point));
    if (!p || p->valid != 1 || !p->count || p->count > CURVE_TIME_PHASES || !Finite(t) || t < 0)
    {
        return 0;
    }
    if (t >= p->total)
    {
        point->position = (float)p->distance;
        point->done = 1;
        return 1;
    }
    for (i = 0; i < p->count; i++)
    {
        if (t < p->time[i + 1U])
        {
            break;
        }
    }
    if (i == p->count)
    {
        return 0;
    }
    dt = t - p->time[i];
    j = p->jerk[i];
    x = p->position[i]+dt * (p->velocity[i]+dt * (p->acceleration[i]/2 + dt * j / 6));
    v = p->velocity[i]+dt * (p->acceleration[i]+dt * j / 2);
    a = p->acceleration[i]+dt * j;
    if (!Fits(x) || !Fits(v) || !Fits(a) || !Fits(j))
    {
        return 0;
    }
    point->position = (float)x;
    point->velocity = (float)v;
    point->acceleration = (float)a;
    point->jerk = (float)j;
    return 1;
}

double Curve_TimeFind(const CurveTiming *plan, double position_m)
{
    double low = 0;
    double high;
    double middle;
    CurvePoint point;
    unsigned index;
    if (!plan || !plan->valid || !isfinite(position_m) || position_m < 0 || position_m > plan->distance)
    {
        return -1;
    }
    high = plan->total;
    for (index = 0; index < 48; index++)
    {
        middle = (low + high) / 2;
        if (!Curve_TimeAt(plan, middle, &point))
        {
            return -1;
        }
        if (point.position < position_m)
        {
            low = middle;
        }
        else
        {
            high = middle;
        }
    }
    return (low + high) / 2;
}
