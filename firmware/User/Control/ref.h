#ifndef REF_H
#define REF_H

#include "line.h"
#include "path.h"

typedef struct
{
    uint32_t kind; /* 1直线/斜线，2转向，3平滑90度弧线 */
    float x_m;
    float y_m;
    float yaw_rad;
    PathSpec path;
    CurveLimit linear;
    CurveLimit angular;
    uint32_t tangent;
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
} RefPlan;

/* 目标固定于动作开始的车体，输出在连续odom坐标；构建后由所有者冻结。
 * 大计划由所有者静态保存。曲线只构建一次，采样不推进游标，可回查反馈时刻。
 * 弧线先有界预验，且逐次检查角V/A/J；这不是整车扫掠/可通行证明。 */
int Ref_Make(RefPlan *plan, const RefSpec *spec, double x_m, double y_m, double yaw_rad);
int Ref_At(const RefPlan *plan, double seconds, RefPoint *point);

#endif
