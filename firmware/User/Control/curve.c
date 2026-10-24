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
