/* 正式轨迹库主机桥；不链接驱动，不模拟实物到位。 */
#include "curve.h"
#include "line.h"
#include "path.h"
#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif
static CurvePlan curve;
static PathPlan path;
static LinePlan line;
static CurveTiming timing;

EXPORT int Test_Timing(float distance, float lead, float tail, float slow, float fast, float end)
{
    CurveLimit limit = {slow, 0.25f, 0.8f};
    return Curve_TimeMake(&timing, distance, lead, tail, limit, fast, 0, end);
}

EXPORT double Test_TimingTime(void)
{
    return timing.total;
}

EXPORT unsigned Test_TimingCount(void)
{
    return timing.count;
}

EXPORT double Test_TimingBoundary(unsigned index)
{
    return index <= timing.count ? timing.time[index] : -1;
}

EXPORT int Test_TimingAt(double seconds, float *out)
{
    CurvePoint point;
    int ok = Curve_TimeAt(&timing, seconds, &point);
    out[0] = point.position;
    out[1] = point.velocity;
    out[2] = point.acceleration;
    out[3] = point.jerk;
    out[4] = (float)point.done;
    return ok;
}
EXPORT int Test_Curve(float distance, float velocity, float accel, float jerk)
{
    CurveLimit limit = {velocity, accel, jerk};
    return Curve_Make(&curve, distance, limit);
}
EXPORT double Test_Time(void)
{
    return curve.total;
}
EXPORT int Test_Scalar(double time, float *out)
{
    CurvePoint point;
    int ok = Curve_At(&curve, time, &point);
    out[0] = point.position;
    out[1] = point.velocity;
    out[2] = point.acceleration;
    out[3] = point.jerk;
    out[4] = (float)point.done;
    return ok;
}
EXPORT double Test_Boundary(unsigned index)
{
    return curve.time[index];
}
EXPORT int Test_Path(float lead, float tail, float radius, float blend, int turn, int reverse)
{
    PathSpec spec = {lead, tail, radius, blend, turn, reverse};
    return Path_Make(&path, &spec);
}
EXPORT int Test_PathAt(double progress, float *out)
{
    PathPoint point;
    int ok = Path_At(&path, progress, &point);
    out[0] = point.x_m;
    out[1] = point.y_m;
    out[2] = point.tx;
    out[3] = point.ty;
    out[4] = point.dtx;
    out[5] = point.dty;
    out[6] = point.ddtx;
    out[7] = point.ddty;
    out[8] = point.yaw_rad;
    out[9] = point.yaw_s;
    out[10] = point.yaw_ss;
    out[11] = point.yaw_sss;
    out[12] = point.curvature;
    out[13] = (float)point.part;
    return ok;
}
EXPORT float Test_Length(void)
{
    return path.length_m;
}
EXPORT float Test_Join(unsigned index)
{
    return path.boundary_m[index];
}
EXPORT int Test_Valid(void)
{
    return Path_Valid(&path);
}
EXPORT void Test_Corrupt(void)
{
    path.cell[4].cx[2] += .001f;
}
EXPORT int Test_Reached(unsigned index, float x, float y, float *out)
{
    return Path_Reached(&path, index, x, y, &out[0], &out[1]);
}
EXPORT int Test_Line(float x, float y)
{
    CurveLimit limit = {.3f, .5f, 2};
    return Line_Make(&line, x, y, limit);
}
EXPORT double Test_LineTime(void)
{
    return line.curve.total;
}
EXPORT int Test_LineAt(double time, float *out)
{
    LinePoint point;
    int ok = Line_At(&line, time, &point);
    out[0] = point.x_m;
    out[1] = point.y_m;
    out[2] = point.vx_m_s;
    out[3] = point.vy_m_s;
    out[4] = point.ax_m_s2;
    out[5] = point.ay_m_s2;
    out[6] = (float)point.reference_done;
    return ok;
}
