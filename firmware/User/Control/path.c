#include "path.h"
#include <float.h>
#include <math.h>
#include <string.h>

/* 数值边界，非本车轮廓、通行或速度参数。 */
#define PATH_MAX_DISTANCE_M 1000.0f
#define PATH_MIN_RADIUS_M 0.001f
#define PATH_MIN_BLEND_M 0.001f

#define PATH_PI 3.14159265358979323846
#define PATH_MAGIC 0x50415432UL

/* 构建期小工作区，约144B；不在栈上创建含整张表的PathPlan。 */
typedef struct
{
    double x[PATH_RAMP_CELLS + 1U], y[PATH_RAMP_CELLS + 1U];
} RampTable;
typedef struct
{
    double x, y, tx, ty, ax, ay, jx, jy;
} BuildPoint;

static int Finite(double value)
{
    return value == value && value >= -DBL_MAX && value <= DBL_MAX;
}

static int SpecValid(const PathSpec *spec)
{
    return spec && Finite(spec->lead_m) && Finite(spec->tail_m) && Finite(spec->radius_m) &&
           Finite(spec->blend_m) && spec->lead_m >= 0 && spec->tail_m >= 0 &&
           spec->lead_m <= PATH_MAX_DISTANCE_M && spec->tail_m <= PATH_MAX_DISTANCE_M &&
           spec->radius_m >= PATH_MIN_RADIUS_M && spec->radius_m <= PATH_MAX_DISTANCE_M &&
           spec->blend_m >= PATH_MIN_BLEND_M && spec->blend_m < PATH_PI * (double)spec->radius_m / 2 &&
           (spec->turn == 1 || spec->turn == -1) && (spec->reverse == 0 || spec->reverse == 1);
}

/* 仅构建期调用。升段k=(1-cos(pi*u/L))/(2R)，
 * theta=(u-L*sin(pi*u/L)/pi)/(2R)。连接处k及dk/ds连续。 */
static double Theta(const PathSpec *spec, double u)
{
    double length = spec->blend_m;
    return (u - length * sin(PATH_PI * u / length) / PATH_PI) / (2 * (double)spec->radius_m);
}

static void Integral(const PathSpec *spec, double lower, double upper, double *x, double *y)
{
    static const double node[4] = {-0.86113631159405257522, -0.33998104358485626480, 0.33998104358485626480,
                                   0.86113631159405257522};
    static const double weight[4] = {0.34785484513745385737, 0.65214515486254614263, 0.65214515486254614263,
                                     0.34785484513745385737};
    double middle = (lower + upper) / 2, half = (upper - lower) / 2;
    unsigned i;
    *x = 0;
    *y = 0;
    for (i = 0; i < 4U; ++i)
    {
        double theta = Theta(spec, middle + half * node[i]);
        *x += weight[i] * cos(theta);
        *y += weight[i] * sin(theta);
    }
    *x *= half;
    *y *= half;
}

static void Ramp(const PathPlan *plan, const RampTable *table, double u, double *x, double *y)
{
    double length = plan->spec.blend_m;
    double step = length / PATH_RAMP_CELLS, dx, dy;
    unsigned index;
    if (u <= 0)
    {
        *x = 0;
        *y = 0;
        return;
    }
    if (u >= length)
    {
        *x = table->x[PATH_RAMP_CELLS];
        *y = table->y[PATH_RAMP_CELLS];
        return;
    }
    index = (unsigned)(u / step);
    if (index >= PATH_RAMP_CELLS)
    {
        index = PATH_RAMP_CELLS - 1U;
    }
    Integral(&plan->spec, index * step, u, &dx, &dy);
    *x = table->x[index] + dx;
    *y = table->y[index] + dy;
}

/* 只在配置/接收动作时积分；控制周期不重建几何。测量与建表同源。 */
static double Geometry(const PathSpec *spec, RampTable *table)
{
    double step = (double)spec->blend_m / PATH_RAMP_CELLS, dx, dy;
    double theta = spec->blend_m / (2 * (double)spec->radius_m);
    unsigned i;
    memset(table, 0, sizeof(*table));
    for (i = 0; i < PATH_RAMP_CELLS; i++)
    {
        Integral(spec, i * step, (i + 1U) * step, &dx, &dy);
        table->x[i + 1U] = table->x[i] + dx;
        table->y[i + 1U] = table->y[i] + dy;
    }
    return table->x[PATH_RAMP_CELLS] + table->y[PATH_RAMP_CELLS] + spec->radius_m * (cos(theta) - sin(theta));
}

int Path_Measure(const PathSpec *spec, double *length, double *displacement)
{
    RampTable table;
    if (!length || !displacement || !SpecValid(spec))
    {
        return 0;
    }
    *length = PATH_PI * (double)spec->radius_m / 2 + spec->blend_m;
    *displacement = Geometry(spec, &table);
    return 1;
}

/* 仅构建期求真几何节点及位置的前三阶弧长导数：
 * r'=T，r''=kN，r'''=k'N-k^2T。转向符号只作用于Y。 */
static void BuildSample(const PathPlan *plan, const RampTable *table, double u, BuildPoint *p)
{
    double radius = plan->spec.radius_m, length = plan->spec.blend_m;
    double x, y, theta, k, dk, a, sign = plan->spec.turn;
    if (u <= 0)
    {
        x = 0;
        y = 0;
        theta = 0;
        k = 0;
        dk = 0;
    }
    else if (u < length)
    {
        Ramp(plan, table, u, &x, &y);
        theta = Theta(&plan->spec, u);
        k = (1 - cos(PATH_PI * u / length)) / (2 * radius);
        dk = PATH_PI * sin(PATH_PI * u / length) / (2 * radius * length);
    }
    else if (u < length + plan->arc_length)
    {
        theta = plan->theta_join + (u - length) / radius;
        x = table->x[PATH_RAMP_CELLS] + radius * (sin(theta) - sin(plan->theta_join));
        y = table->y[PATH_RAMP_CELLS] + radius * (cos(plan->theta_join) - cos(theta));
        k = 1 / radius;
        dk = 0;
    }
    else if (u < plan->curve_length)
    {
        double remaining = plan->curve_length - u;
        Ramp(plan, table, remaining, &x, &y);
        a = x;
        x = plan->displacement - y;
        y = plan->displacement - a;
        theta = PATH_PI / 2 - Theta(&plan->spec, remaining);
        a = u - length - plan->arc_length;
        k = (1 + cos(PATH_PI * a / length)) / (2 * radius);
        dk = -PATH_PI * sin(PATH_PI * a / length) / (2 * radius * length);
    }
    else
    {
        x = plan->displacement;
        y = x;
        theta = PATH_PI / 2;
        k = 0;
        dk = 0;
    }
    p->x = plan->spec.lead_m + x;
    p->y = sign * y;
    p->tx = cos(theta);
    p->ty = sign * sin(theta);
    p->ax = -k * sin(theta);
    p->ay = sign * k * cos(theta);
    p->jx = -dk * sin(theta) - k * k * cos(theta);
    p->jy = sign * (dk * cos(theta) - k * k * sin(theta));
}

/* 七次Hermite同时约束两端位置、一/二/三阶导数。
 * 先用double计算局部残差，再存float系数；不将大坐标float相减。 */
static void Coefficients(double p0, double d0, double a0, double j0, double p1, double d1, double a1, double j1,
                         double h, float c[7])
{
    double c1 = h * d0, c2 = h * h * a0 / 2, c3 = h * h * h * j0 / 6;
    double r0 = p1 - p0 - c1 - c2 - c3;
    double r1 = h * d1 - c1 - 2 * c2 - 3 * c3;
    double r2 = h * h * a1 - 2 * c2 - 6 * c3;
    double r3 = h * h * h * j1 - 6 * c3;
    c[0] = (float)c1;
    c[1] = (float)c2;
    c[2] = (float)c3;
    c[3] = (float)(35 * r0 - 15 * r1 + 2.5 * r2 - r3 / 6);
    c[4] = (float)(-84 * r0 + 39 * r1 - 7 * r2 + r3 / 2);
    c[5] = (float)(70 * r0 - 34 * r1 + 6.5 * r2 - r3 / 2);
    c[6] = (float)(-20 * r0 + 10 * r1 - 2 * r2 + r3 / 6);
}

static uint32_t HashBytes(uint32_t hash, const void *data, unsigned size)
{
    const unsigned char *bytes = (const unsigned char *)data;
    unsigned i;
    for (i = 0; i < size; ++i)
    {
        hash ^= bytes[i];
        hash *= 16777619UL;
    }
    return hash;
}

static uint32_t Checksum(const PathPlan *plan)
{
    uint32_t hash = 2166136261UL;
    unsigned i;
#define HASH(field) hash = HashBytes(hash, &plan->field, (unsigned)sizeof(plan->field))
    HASH(spec.lead_m);
    HASH(spec.tail_m);
    HASH(spec.radius_m);
    HASH(spec.blend_m);
    HASH(spec.turn);
    HASH(spec.reverse);
    HASH(length_m);
    HASH(x_m);
    HASH(y_m);
    HASH(boundary_m);
    HASH(arc_length);
    HASH(curve_length);
    HASH(displacement);
    HASH(theta_join);
    HASH(boundary_x);
    HASH(boundary_y);
    HASH(boundary_tx);
    HASH(boundary_ty);
    for (i = 0; i < PATH_CELLS; ++i)
    {
        HASH(cell[i].x);
        HASH(cell[i].y);
        HASH(cell[i].cx);
        HASH(cell[i].cy);
    }
#undef HASH
    return hash ? hash : 1U;
}

/* 周期O(1)检查；完整校验在接受/冻结时，所有者保证整个动作中不可改写。 */
static int Ready(const PathPlan *plan)
{
    return plan && plan->magic == PATH_MAGIC && plan->checksum && Finite(plan->length_m) &&
           plan->length_m > 0 && plan->length_m <= PATH_MAX_DISTANCE_M;
}

int Path_Valid(const PathPlan *plan)
{
    unsigned i, j;
    if (!Ready(plan) || !SpecValid(&plan->spec) || !Finite(plan->x_m) || !Finite(plan->y_m) ||
        !Finite(plan->arc_length) || plan->arc_length <= 0 || !Finite(plan->curve_length) ||
        plan->curve_length <= 0 || !Finite(plan->displacement) || plan->displacement <= 0 ||
        !Finite(plan->theta_join) ||
        plan->arc_length != PATH_PI * (double)plan->spec.radius_m / 2 - plan->spec.blend_m ||
        plan->curve_length != plan->arc_length + 2 * (double)plan->spec.blend_m ||
        plan->length_m != (float)(plan->spec.lead_m + plan->curve_length + plan->spec.tail_m) ||
        plan->x_m != (float)((plan->spec.reverse ? -1 : 1) * (plan->spec.lead_m + plan->displacement)) ||
        plan->y_m !=
            (float)((plan->spec.reverse ? -1 : 1) * plan->spec.turn * (plan->displacement + plan->spec.tail_m)))
    {
        return 0;
    }
    for (i = 0; i < PATH_BOUNDARIES; ++i)
    {
        if (!Finite(plan->boundary_m[i]) || !Finite(plan->boundary_x[i]) || !Finite(plan->boundary_y[i]) ||
            !Finite(plan->boundary_tx[i]) || !Finite(plan->boundary_ty[i]) || plan->boundary_m[i] < 0 ||
            plan->boundary_m[i] > plan->length_m || (i && plan->boundary_m[i] <= plan->boundary_m[i - 1U]))
        {
            return 0;
        }
    }
    for (i = 0; i < PATH_CELLS; ++i)
    {
        if (!Finite(plan->cell[i].x) || !Finite(plan->cell[i].y))
        {
            return 0;
        }
        for (j = 0; j < 7U; ++j)
        {
            if (!Finite(plan->cell[i].cx[j]) || !Finite(plan->cell[i].cy[j]))
            {
                return 0;
            }
        }
    }
    return Checksum(plan) == plan->checksum;
}

int Path_Make(PathPlan *plan, const PathSpec *spec)
{
    PathSpec input;
    RampTable table;
    BuildPoint first, last;
    double lower, width, h;
    unsigned i, phase, count, offset;
    if (!plan)
    {
        return 0;
    }
    if (spec)
    {
        input = *spec;
    }
    memset(plan, 0, sizeof(*plan));
    if (!spec || !SpecValid(&input))
    {
        goto invalid;
    }
    plan->spec = input;
    plan->theta_join = input.blend_m / (2 * (double)input.radius_m);
    plan->arc_length = PATH_PI * (double)input.radius_m / 2 - input.blend_m;
    plan->curve_length = plan->arc_length + 2 * (double)input.blend_m;
    h = input.lead_m + plan->curve_length + input.tail_m;
    if (!Finite(h) || h > PATH_MAX_DISTANCE_M)
    {
        goto invalid;
    }
    plan->length_m = (float)h;
    plan->displacement = Geometry(&input, &table);
    plan->x_m = (float)(input.lead_m + plan->displacement);
    plan->y_m = (float)(input.turn * (plan->displacement + input.tail_m));
    plan->boundary_m[0] = input.lead_m;
    plan->boundary_m[1] = (float)(input.lead_m + (double)input.blend_m);
    plan->boundary_m[2] = (float)(input.lead_m + (double)input.blend_m + plan->arc_length);
    plan->boundary_m[3] = (float)(input.lead_m + plan->curve_length);
    plan->boundary_x[0] = input.lead_m;
    plan->boundary_y[0] = 0;
    plan->boundary_tx[0] = 1;
    plan->boundary_ty[0] = 0;
    BuildSample(plan, &table, input.blend_m, &first);
    plan->boundary_x[1] = first.x;
    plan->boundary_y[1] = first.y;
    plan->boundary_tx[1] = first.tx;
    plan->boundary_ty[1] = first.ty;
    BuildSample(plan, &table, input.blend_m + plan->arc_length, &first);
    plan->boundary_x[2] = first.x;
    plan->boundary_y[2] = first.y;
    plan->boundary_tx[2] = first.tx;
    plan->boundary_ty[2] = first.ty;
    plan->boundary_x[3] = input.lead_m + plan->displacement;
    plan->boundary_y[3] = input.turn * plan->displacement;
    plan->boundary_tx[3] = 0;
    plan->boundary_ty[3] = input.turn;
    offset = 0;
    lower = 0;
    for (phase = 0; phase < 3U; ++phase)
    {
        count = phase == 1U ? PATH_ARC_CELLS : PATH_RAMP_CELLS;
        width = phase == 1U ? plan->arc_length : (double)input.blend_m;
        h = width / count;
        BuildSample(plan, &table, lower, &first);
        for (i = 0; i < count; ++i)
        {
            PathCell *cell = &plan->cell[offset + i];
            BuildSample(plan, &table, lower + (i + 1U) * h, &last);
            cell->x = first.x;
            cell->y = first.y;
            Coefficients(first.x, first.tx, first.ax, first.jx, last.x, last.tx, last.ax, last.jx, h, cell->cx);
            Coefficients(first.y, first.ty, first.ay, first.jy, last.y, last.ty, last.ay, last.jy, h, cell->cy);
            first = last;
        }
        offset += count;
        lower += width;
    }
    /* 倒车仅改变空间曲线的方向；正弧长S计划、各边界s和时间轴保持不变。
     * 边界切向也反号，实际反馈的投影进度才能继续沿正s增长。 */
    if (input.reverse)
    {
        unsigned j;
        plan->x_m = -plan->x_m;
        plan->y_m = -plan->y_m;
        for (i = 0; i < PATH_BOUNDARIES; i++)
        {
            plan->boundary_x[i] = -plan->boundary_x[i];
            plan->boundary_y[i] = -plan->boundary_y[i];
            plan->boundary_tx[i] = -plan->boundary_tx[i];
            plan->boundary_ty[i] = -plan->boundary_ty[i];
        }
        for (i = 0; i < PATH_CELLS; i++)
        {
            plan->cell[i].x = -plan->cell[i].x;
            plan->cell[i].y = -plan->cell[i].y;
            for (j = 0; j < 7U; j++)
            {
                plan->cell[i].cx[j] = -plan->cell[i].cx[j];
                plan->cell[i].cy[j] = -plan->cell[i].cy[j];
            }
        }
    }
    plan->magic = PATH_MAGIC;
    plan->checksum = Checksum(plan);
    if (!Path_Valid(plan))
    {
        goto invalid;
    }
    return 1;
invalid:
    memset(plan, 0, sizeof(*plan));
    return 0;
}

/* 全部用float Horner；位置及一至四阶导来自同一七次多项式。 */
static void Evaluate(const float c[7], float t, float inv_h, float *x, float *d, float *a, float *j,
                     float *fourth)
{
    *x = ((((((c[6] * t + c[5]) * t + c[4]) * t + c[3]) * t + c[2]) * t + c[1]) * t + c[0]) * t;
    *d = ((((((7 * c[6] * t + 6 * c[5]) * t + 5 * c[4]) * t + 4 * c[3]) * t + 3 * c[2]) * t + 2 * c[1]) * t +
          c[0]) *
         inv_h;
    *a = (((((42 * c[6] * t + 30 * c[5]) * t + 20 * c[4]) * t + 12 * c[3]) * t + 6 * c[2]) * t + 2 * c[1]) *
         inv_h * inv_h;
    *j = ((((210 * c[6] * t + 120 * c[5]) * t + 60 * c[4]) * t + 24 * c[3]) * t + 6 * c[2]) * inv_h * inv_h *
         inv_h;
    *fourth =
        (((840 * c[6] * t + 360 * c[5]) * t + 120 * c[4]) * t + 24 * c[3]) * inv_h * inv_h * inv_h * inv_h;
}

int Path_At(const PathPlan *plan, double s, PathPoint *point)
{
    const PathCell *cell;
    double u, width, fraction;
    float t, inv_h, dx, dy, q, root, cross, fourth_x, fourth_y;
    float q_rate, q_accel, cross_rate, cross_accel;
    unsigned count, offset, index, part;
    float direction;
    if (!point)
    {
        return 0;
    }
    memset(point, 0, sizeof(*point));
    if (!Ready(plan) || !Finite(s) || s < 0)
    {
        return 0;
    }
    direction = plan->spec.reverse ? -1.0f : 1.0f;
    if (s >= plan->length_m)
    {
        point->x_m = plan->x_m;
        point->y_m = plan->y_m;
        point->ty = direction * plan->spec.turn;
        point->part = 4;
        point->yaw_rad = (float)(plan->spec.turn * (PATH_PI / 2));
        return 1;
    }
    if (s < plan->spec.lead_m)
    {
        point->x_m = direction * (float)s;
        point->tx = direction;
        return 1;
    }
    u = s - plan->spec.lead_m;
    if (u >= plan->curve_length)
    {
        point->x_m = plan->x_m;
        point->y_m = (float)(direction * plan->spec.turn * (plan->displacement + u - plan->curve_length));
        point->ty = direction * plan->spec.turn;
        point->part = 4;
        point->yaw_rad = (float)(plan->spec.turn * (PATH_PI / 2));
        return 1;
    }
    if (u < plan->spec.blend_m)
    {
        offset = 0;
        count = PATH_RAMP_CELLS;
        width = plan->spec.blend_m;
        part = 1;
    }
    else if (u < plan->spec.blend_m + plan->arc_length)
    {
        u -= plan->spec.blend_m;
        offset = PATH_RAMP_CELLS;
        count = PATH_ARC_CELLS;
        width = plan->arc_length;
        part = 2;
    }
    else
    {
        u -= plan->spec.blend_m + plan->arc_length;
        offset = PATH_RAMP_CELLS + PATH_ARC_CELLS;
        count = PATH_RAMP_CELLS;
        width = plan->spec.blend_m;
        part = 3;
    }
    if (!Finite(width) || width <= 0)
    {
        return 0;
    }
    fraction = u * count / width;
    /* 即使调用者破坏不可变契约，也不能把NaN/越界值转为数组索引。 */
    if (!Finite(fraction) || fraction < 0 || fraction > count)
    {
        return 0;
    }
    index = (unsigned)fraction;
    if (index >= count)
    {
        index = count - 1U;
    }
    t = (float)(fraction - index);
    inv_h = (float)(count / width);
    cell = &plan->cell[offset + index];
    Evaluate(cell->cx, t, inv_h, &dx, &point->tx, &point->dtx, &point->ddtx, &fourth_x);
    Evaluate(cell->cy, t, inv_h, &dy, &point->ty, &point->dty, &point->ddty, &fourth_y);
    point->x_m = (float)(cell->x + dx);
    point->y_m = (float)(cell->y + dy);
    q = point->tx * point->tx + point->ty * point->ty;
    if (!Finite(q) || q <= 0)
    {
        return 0;
    }
    root = sqrtf(q);
    cross = point->tx * point->dty - point->ty * point->dtx;
    point->curvature = cross / (q * root);
    point->curvature_rate = (point->tx * point->ddty - point->ty * point->ddtx) / (q * root) -
                            3 * point->curvature * (point->tx * point->dtx + point->ty * point->dty) / q;
    /* theta=atan2(r'_y,r'_x)，令N=r'叉r''、q=r'点r'。
     * theta_s=N/q，后两阶直接微分；所有量来自同一缓存而非独立yaw轨迹。
     * 这里s是缓存的弧长参数；几何曲率N/q^(3/2)另作诊断保留。 */
    q_rate = 2 * (point->tx * point->dtx + point->ty * point->dty);
    q_accel = 2 * (point->dtx * point->dtx + point->dty * point->dty + point->tx * point->ddtx +
                   point->ty * point->ddty);
    cross_rate = point->tx * point->ddty - point->ty * point->ddtx;
    cross_accel =
        point->dtx * point->ddty - point->dty * point->ddtx + point->tx * fourth_y - point->ty * fourth_x;
    /* 反馈和P控制仍在动作起始坐标；车头参考沿行驶切线的反方向。
     * 同时反r'与r''后yaw导数N/q不变，reverse右弯得到v<0、CCW omega<0。 */
    point->yaw_rad = atan2f(direction * point->ty, direction * point->tx);
    point->yaw_s = cross / q;
    point->yaw_ss = (cross_rate - point->yaw_s * q_rate) / q;
    point->yaw_sss = (cross_accel - 2 * point->yaw_ss * q_rate - point->yaw_s * q_accel) / q;
    point->part = part;
    return Finite(point->x_m) && Finite(point->y_m) && Finite(point->tx) && Finite(point->ty) &&
           Finite(point->dtx) && Finite(point->dty) && Finite(point->ddtx) && Finite(point->ddty) &&
           Finite(point->curvature) && Finite(point->curvature_rate) && Finite(point->yaw_rad) &&
           Finite(point->yaw_s) && Finite(point->yaw_ss) && Finite(point->yaw_sss);
}

int Path_Reached(const PathPlan *plan, unsigned index, float x, float y, float *along, float *cross)
{
    double dx, dy, a, c;
    if (along)
    {
        *along = 0;
    }
    if (cross)
    {
        *cross = 0;
    }
    if (!Ready(plan) || index >= PATH_BOUNDARIES || !Finite(x) || !Finite(y))
    {
        return 0;
    }
    dx = (double)x - plan->boundary_x[index];
    dy = (double)y - plan->boundary_y[index];
    a = plan->boundary_m[index] + dx * plan->boundary_tx[index] + dy * plan->boundary_ty[index];
    c = -dx * plan->boundary_ty[index] + dy * plan->boundary_tx[index];
    if (!Finite(a) || !Finite(c) || fabs(a) > FLT_MAX || fabs(c) > FLT_MAX)
    {
        return 0;
    }
    if (along)
    {
        *along = (float)a;
    }
    if (cross)
    {
        *cross = (float)c;
    }
    return (float)a >= plan->boundary_m[index];
}
