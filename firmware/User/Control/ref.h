#ifndef REF_H
#define REF_H

#include "line.h"
#include "path.h"
#include <stddef.h>

#if defined(ODOM_HOST) && defined(_WIN32)
#define REF_API __declspec(dllexport)
#else
#define REF_API
#endif

typedef struct
{
    uint32_t kind; /* 1平移，2转向，3切线弧，4连续弯道，5移姿同步，6直线转姿窗口 */
    float x_m;
    float y_m;
    float yaw_rad;
    PathSpec path;
    CurveLimit linear;
    CurveLimit angular;
    uint32_t tangent;
    float path_heading_rad;
    float curve_velocity_m_s;
    float yaw_begin_m;
    float yaw_end_m;
    float tail_velocity_m_s;
} RefSpec;

typedef struct
{
    double x_m;
    double y_m;
    double yaw_rad;
    double vx_m_s;
    double vy_m_s;
    double wz_rad_s;
    double angular_accel_rad_s2;
    double angular_jerk_rad_s3;
    double ax_m_s2;
    double ay_m_s2;
    double jx_m_s3;
    double jy_m_s3;
    uint32_t done;
} RefPoint;

typedef struct
{
    LinePlan line;
    CurvePlan scalar;
    PathPlan path;
    RefSpec spec;
    double origin[3];
    double target[3];
    double duration_s;
    unsigned valid;
    CurvePlan heading;
    CurveTiming timing;
    double position_time_s;
    double heading_begin_s;
    double heading_end_s;
} RefPlan;

/* 目标固定于动作开始的车体，输出在连续odom坐标；构建后由所有者冻结。
 * 大计划由所有者静态保存。曲线只构建一次，采样不推进游标，可回查反馈时刻。
 * 弧线先有界预验，且逐次检查角V/A/J；这不是整车扫掠/可通行证明。 */
REF_API size_t Ref_Bytes(void);
REF_API double Ref_Time(const RefPlan *plan);
REF_API int Ref_Make(RefPlan *plan, const RefSpec *spec, double x_m, double y_m, double yaw_rad);
REF_API int Ref_At(const RefPlan *plan, double seconds, RefPoint *point);
/* 全区间位置/航向变化率保守界，供上位机扫掠包络膨胀，不含反馈修正。 */
REF_API int Ref_Bounds(const RefPlan *plan, double *velocity_m_s, double *yaw_rad_s);
/* 航向变化期间的平移速率界，避免叠加不同时间的线速/角速峰值。 */
REF_API int Ref_TurnSpeed(const RefPlan *plan, double *velocity_m_s);
/* 已构建曲线的几何常速上限及长度，仅用于几何弯段，不替代完整参考检查。 */
REF_API int Ref_Curve(const RefPlan *plan, CurveLimit limit, double *velocity_m_s, double *length_m);

#endif
