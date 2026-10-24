#ifndef PATH_H
#define PATH_H

#include <stdint.h>

#define PATH_RAMP_CELLS 8U
#define PATH_ARC_CELLS 16U
#define PATH_CELLS (2U * PATH_RAMP_CELLS + PATH_ARC_CELLS)
#define PATH_BOUNDARIES 4U

/* 动作起点固定坐标，X前/Y左/逆时针正。正s从0单调增加。
 * 入直线→余弦升曲率→恒曲率圆弧→余弦降曲率→出直线，总转角90°。
 * turn=+1左/-1右；reverse=1同时反转位置/切向，车头角不额外加pi。
 * 这是离线几何接口，不检查车体扫掠/地图，也不授权实车切角。 */
typedef struct
{
    float lead_m;
    float tail_m;
    float radius_m;
    float blend_m;
    int turn;
    int reverse;
} PathSpec;

typedef struct
{
    double x;
    double y;
    float cx[7];
    float cy[7];
} PathCell;

typedef struct
{
    PathSpec spec;
    float length_m;
    float x_m;
    float y_m;
    float boundary_m[PATH_BOUNDARIES];
    double arc_length;
    double curve_length;
    double displacement;
    double theta_join;
    double boundary_x[PATH_BOUNDARIES];
    double boundary_y[PATH_BOUNDARIES];
    double boundary_tx[PATH_BOUNDARIES];
    double boundary_ty[PATH_BOUNDARIES];
    PathCell cell[PATH_CELLS];
    uint32_t magic;
    uint32_t checksum;
} PathPlan;

typedef struct
{
    float x_m;
    float y_m;
    /* 同一多项式的位置一至三阶s导数；单位1、1/m、1/m²，不另行归一化。 */
    float tx;
    float ty;
    float dtx;
    float dty;
    float ddtx;
    float ddty;
    float curvature;
    float curvature_rate;
    /* 车头切向角及s导数，rad、rad/m、rad/m²、rad/m³。 */
    float yaw_rad;
    float yaw_s;
    float yaw_ss;
    float yaw_sss;
    unsigned part;
} PathPoint;

int Path_Make(PathPlan *plan, const PathSpec *spec);
int Path_Measure(const PathSpec *spec, double *length_m, double *displacement_m);
/* 冻结时完整检查缓存校验和。接受后所有者保持不可变；采样只做O(1)边界检查。
 * Plan约2.6KB，由所有者静态保存，不在中断或小栈创建。 */
int Path_Valid(const PathPlan *plan);
int Path_At(const PathPlan *plan, double s_m, PathPoint *point);
/* 按实际位置检查边界切线法平面；沿程/cross单位米，cross左正。
 * 投影越界不等于动作完成，调用者仍需检查横向误差、序号和新鲜反馈。 */
int Path_Reached(const PathPlan *plan, unsigned index, float x_m, float y_m, float *along_m, float *cross_m);

#endif
