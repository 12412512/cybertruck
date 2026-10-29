#ifndef CURVE_H
#define CURVE_H

/* 单一标量的限速度/加速度/加加速度曲线。
 * 平移使用m/s/s²/s³，转角使用rad/s/s²/s³；调用者不得混用单位。
 * 无本车默认值；以下数值域不代表硬件能力：距离<=1000，V/A<=100，J<=10000，限值>=1e-7。 */
typedef struct
{
    float velocity;
    float acceleration;
    float jerk;
} CurveLimit;

typedef struct
{
    double duration[7];
    double time[8];
    double position[8];
    double velocity[8];
    double acceleration[8];
    double distance;
    double total;
    double peak_velocity;
    double peak_acceleration;
    double jerk;
    double sign;
    unsigned valid;
} CurvePlan;

typedef struct
{
    float position;
    float velocity;
    float acceleration;
    float jerk;
    unsigned done;
} CurvePoint;

/* 静止起点到静止终点，可退化七段。计划由所有者冻结，不得在采样中改写。
 * done仅表示参考结束，不证明实际停车或动作完成。double运算只在非中断任务使用。 */
int Curve_Make(CurvePlan *plan, float distance, CurveLimit limit);
int Curve_At(const CurvePlan *plan, double seconds, CurvePoint *point);

#define CURVE_TIME_PHASES 15U
typedef struct
{
    double time[CURVE_TIME_PHASES + 1U];
    double position[CURVE_TIME_PHASES + 1U];
    double velocity[CURVE_TIME_PHASES + 1U];
    double acceleration[CURVE_TIME_PHASES + 1U];
    double jerk[CURVE_TIME_PHASES];
    double distance;
    double total;
    double peak_velocity;
    double curve_velocity;
    double start_end;
    double stop_begin;
    double lead_peak;
    double tail_peak;
    unsigned count;
    unsigned valid;
    unsigned split;
} CurveTiming;

/* 复用旧PathTimePlan：首末直线高速、内部曲线低速，边界V/A连续且不停车。
 * 全部距离米；tail_velocity单独约束末端接近档，不改变全局速度上限。 */
int Curve_TimeMake(CurveTiming *plan, float distance_m, float lead_m, float tail_m,
                   CurveLimit slow, float fast_velocity_m_s, float buffer_m, float tail_velocity_m_s);
int Curve_TimeAt(const CurveTiming *plan, double seconds, CurvePoint *point);
double Curve_TimeFind(const CurveTiming *plan, double position_m);

#endif
