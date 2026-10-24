#include "kin.h"
#include <float.h>
#include <math.h>

#define KIN_PI 3.14159265358979323846

static int Finite(double value)
{
    return value == value && value <= DBL_MAX && value >= -DBL_MAX;
}

static int Fits(double value)
{
    return Finite(value) && value <= FLT_MAX && value >= -FLT_MAX;
}

static int Geometry(KinGeometry geometry)
{
    return geometry.layout == KIN_MECANUM_X &&
        Finite(geometry.radius_m) && geometry.radius_m > 0 &&
        Finite(geometry.half_length_m) && geometry.half_length_m > 0 &&
        Finite(geometry.half_width_m) && geometry.half_width_m > 0;
}

int Kin_Inverse(KinGeometry geometry, KinBody body, float rpm[4])
{
    double scale;
    double rotation;
    double wheel[4];
    unsigned index;
    if (!rpm)
    {
        return 0;
    }
    for (index = 0; index < 4; index++)
    {
        rpm[index] = 0;
    }
    if (!Geometry(geometry) || !Finite(body.vx_m_s) ||
        !Finite(body.vy_m_s) || !Finite(body.wz_rad_s))
    {
        return 0;
    }
    scale = 60.0 / (2.0 * KIN_PI * geometry.radius_m);
    rotation = ((double)geometry.half_length_m + geometry.half_width_m) * body.wz_rad_s;
    /* 旧公式Y右/顺时针正在迁移时转换一次，当前公式直接使用新约定。 */
    wheel[0] = ((double)body.vx_m_s - body.vy_m_s - rotation) * scale;
    wheel[1] = ((double)body.vx_m_s + body.vy_m_s + rotation) * scale;
    wheel[2] = ((double)body.vx_m_s + body.vy_m_s - rotation) * scale;
    wheel[3] = ((double)body.vx_m_s - body.vy_m_s + rotation) * scale;
    for (index = 0; index < 4; index++)
    {
        if (!Fits(wheel[index]))
        {
            return 0;
        }
    }
    for (index = 0; index < 4; index++)
    {
        rpm[index] = (float)wheel[index];
    }
    return 1;
}

int Kin_Forward(KinGeometry geometry, const float rpm[4], KinBody *body)
{
    double scale;
    double radius;
    double vx;
    double vy;
    double wz;
    unsigned index;
    if (!body)
    {
        return 0;
    }
    body->vx_m_s = body->vy_m_s = body->wz_rad_s = 0;
    if (!rpm || !Geometry(geometry))
    {
        return 0;
    }
    for (index = 0; index < 4; index++)
    {
        if (!Finite(rpm[index]))
        {
            return 0;
        }
    }
    scale = 2.0 * KIN_PI * geometry.radius_m / 240.0;
    radius = (double)geometry.half_length_m + geometry.half_width_m;
    vx = ((double)rpm[0] + rpm[1] + rpm[2] + rpm[3]) * scale;
    vy = (-(double)rpm[0] + rpm[1] + rpm[2] - rpm[3]) * scale;
    wz = (-(double)rpm[0] + rpm[1] - rpm[2] + rpm[3]) * scale / radius;
    if (!Fits(vx) || !Fits(vy) || !Fits(wz))
    {
        return 0;
    }
    body->vx_m_s = (float)vx;
    body->vy_m_s = (float)vy;
    body->wz_rad_s = (float)wz;
    return 1;
}

float Kin_Limit(float rpm[4], float max_rpm)
{
    double peak = 0;
    double scale;
    unsigned index;
    if (!rpm || !Finite(max_rpm) || max_rpm <= 0)
    {
        return -1;
    }
    for (index = 0; index < 4; index++)
    {
        if (!Finite(rpm[index]))
        {
            return -1;
        }
        if (fabs((double)rpm[index]) > peak)
        {
            peak = fabs((double)rpm[index]);
        }
    }
    scale = peak > max_rpm ? (double)max_rpm / peak : 1.0;
    if ((float)scale == 0)
    {
        return -1;
    }
    for (index = 0; index < 4; index++)
    {
        rpm[index] = (float)(rpm[index] * scale);
    }
    return (float)scale;
}
