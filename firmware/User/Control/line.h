#ifndef LINE_H
#define LINE_H

#include "curve.h"

typedef struct
{
    CurvePlan curve;
    float dx_m;
    float dy_m;
    double ux;
    double uy;
} LinePlan;

typedef struct
{
    float x_m;
    float y_m;
    float vx_m_s;
    float vy_m_s;
    float ax_m_s2;
    float ay_m_s2;
    unsigned reference_done;
} LinePoint;

/* X/Y/任意斜线共用一条标量时间轴，保证两轴同时起停。
 * 坐标固定于动作起点，不是运动中旋转的车体坐标；无电机输出。 */
int Line_Make(LinePlan *plan, float dx_m, float dy_m, CurveLimit limit);
int Line_At(const LinePlan *plan, double seconds, LinePoint *point);

#endif
