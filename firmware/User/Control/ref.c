#include "ref.h"
#include <math.h>
#include <string.h>

#define REF_PI 3.14159265358979323846

size_t Ref_Bytes(void)
{
    return sizeof(RefPlan);
}

double Ref_Time(const RefPlan *plan)
{
    return plan && plan->valid ? plan->duration_s : -1;
}

int Ref_Bounds(const RefPlan *plan, double *velocity_m_s, double *yaw_rad_s)
{
    unsigned cell;
    unsigned power;
    double width;
    double dx;
    double dy;
    double ddx;
    double ddy;
    double lower;
    double speed;
    double yaw;
    if (!plan || !plan->valid || !velocity_m_s || !yaw_rad_s)
    {
        return 0;
    }
    *velocity_m_s = plan->spec.kind == 2 ? 0 : plan->spec.linear.velocity;
    *yaw_rad_s = plan->spec.kind == 2 ? plan->spec.angular.velocity : 0;
    if (plan->spec.kind >= 4 && plan->heading_end_s > plan->heading_begin_s)
    {
        *yaw_rad_s = plan->heading.peak_velocity * plan->heading.total /
            (plan->heading_end_s - plan->heading_begin_s);
    }
    if (plan->spec.kind != 3 && plan->spec.kind != 4)
    {
        return 1;
    }
    /* 每格t在[0,1]，系数绝对值和给导数上界；常数项减余项给下界。
     * |yaw_s| <= |r' x r''| / |r'|²，分母不能证明大于零则拒绝。 */
    for (cell = 0; cell < PATH_CELLS; cell++)
    {
        width = cell < PATH_RAMP_CELLS || cell >= PATH_RAMP_CELLS + PATH_ARC_CELLS ?
            plan->spec.path.blend_m / PATH_RAMP_CELLS : plan->path.arc_length / PATH_ARC_CELLS;
        dx = dy = ddx = ddy = 0;
        for (power = 1; power <= 7; power++)
        {
            dx += power * fabs(plan->path.cell[cell].cx[power - 1]) / width;
            dy += power * fabs(plan->path.cell[cell].cy[power - 1]) / width;
            ddx += power * (power - 1) * fabs(plan->path.cell[cell].cx[power - 1]) / (width * width);
            ddy += power * (power - 1) * fabs(plan->path.cell[cell].cy[power - 1]) / (width * width);
        }
        lower = fmax(2 * fabs(plan->path.cell[cell].cx[0]) / width - dx,
                     2 * fabs(plan->path.cell[cell].cy[0]) / width - dy);
        if (lower <= 0)
        {
            return 0;
        }
        speed = hypot(dx, dy) * plan->spec.linear.velocity;
        yaw = (dx * ddy + dy * ddx) / (lower * lower) * plan->spec.linear.velocity;
        *velocity_m_s = fmax(*velocity_m_s, speed);
        if (plan->spec.kind == 3 && plan->spec.tangent)
        {
            *yaw_rad_s = fmax(*yaw_rad_s, yaw);
        }
    }
    *velocity_m_s *= 1.0001;
    *yaw_rad_s *= 1.0001;
    return isfinite(*velocity_m_s) && isfinite(*yaw_rad_s);
}

int Ref_TurnSpeed(const RefPlan *plan, double *velocity_m_s)
{
    double overall;
    double yaw;
    double peak = 0;
    double begin;
    double end;
    double at[3];
    double delta;
    double velocity;
    unsigned phase;
    unsigned sample;
    const CurveTiming *timing;
    if (!velocity_m_s || !Ref_Bounds(plan, &overall, &yaw))
    {
        return 0;
    }
    *velocity_m_s = overall;
    if (plan->spec.kind != 4 && plan->spec.kind != 6)
    {
        return 1;
    }
    if (plan->heading.total == 0)
    {
        *velocity_m_s = 0;
        return 1;
    }
    timing = &plan->timing;
    if (!timing->valid || !timing->count || timing->count > CURVE_TIME_PHASES)
    {
        return 0;
    }
    /* 直接覆盖查时后的真实区间，不假定二分得到的边界恰好位于匀速段。
     * 每个速度二次多项式只在两端或加速度零点取得极值，最多15×3次。 */
    for (phase = 0; phase < timing->count; phase++)
    {
        begin = fmax(plan->heading_begin_s, timing->time[phase]);
        end = fmin(plan->heading_end_s, timing->time[phase + 1U]);
        if (end < begin)
        {
            continue;
        }
        at[0] = begin;
        at[1] = end;
        at[2] = timing->jerk[phase] != 0 ? timing->time[phase] -
            timing->acceleration[phase] / timing->jerk[phase] : begin;
        for (sample = 0; sample < 3; sample++)
        {
            if (at[sample] < begin || at[sample] > end)
            {
                continue;
            }
            delta = at[sample] - timing->time[phase];
            velocity = timing->velocity[phase] + delta * (timing->acceleration[phase] +
                delta * timing->jerk[phase] / 2);
            peak = fmax(peak, fabs(velocity));
        }
    }
    /* overall/V给几何导数界；相位积分为double，额外覆盖采样转float的舍入。 */
    *velocity_m_s = overall * peak / plan->spec.linear.velocity * 1.0001;
    return isfinite(*velocity_m_s);
}

int Ref_Curve(const RefPlan *plan, CurveLimit limit, double *velocity_m_s, double *length_m)
{
    unsigned cell;
    unsigned order;
    unsigned power;
    unsigned factor;
    double width;
    double dx;
    double dy;
    double rate;
    double bound[3] = {0, 0, 0};
    double speed;
    if (!plan || !plan->valid || !velocity_m_s || !length_m ||
        (plan->spec.kind != 3 && plan->spec.kind != 4) ||
        !isfinite(limit.velocity) || limit.velocity <= 0 ||
        !isfinite(limit.acceleration) || limit.acceleration <= 0 ||
        !isfinite(limit.jerk) || limit.jerk <= 0)
    {
        return 0;
    }
    /* 每格局部参数在[0,1]，导数系数绝对值和覆盖整个区间。
     * 弯段为常标量速度：a=r''v²，j=r'''v³；首末加减速仍在直线段。 */
    for (cell = 0; cell < PATH_CELLS; cell++)
    {
        width = cell < PATH_RAMP_CELLS || cell >= PATH_RAMP_CELLS + PATH_ARC_CELLS ?
            plan->spec.path.blend_m / PATH_RAMP_CELLS : plan->path.arc_length / PATH_ARC_CELLS;
        for (order = 1; order <= 3; order++)
        {
            dx = dy = 0;
            for (power = order; power <= 7; power++)
            {
                factor = power;
                if (order >= 2)
                {
                    factor *= power - 1;
                }
                if (order == 3)
                {
                    factor *= power - 2;
                }
                rate = factor / pow(width, (double)order);
                dx += rate * fabs(plan->path.cell[cell].cx[power - 1]);
                dy += rate * fabs(plan->path.cell[cell].cy[power - 1]);
            }
            bound[order - 1] = fmax(bound[order - 1], hypot(dx, dy));
        }
    }
    speed = limit.velocity / fmax(1, bound[0]);
    if (bound[1] > 0)
    {
        speed = fmin(speed, sqrt(limit.acceleration / bound[1]));
    }
    if (bound[2] > 0)
    {
        speed = fmin(speed, pow(limit.jerk / bound[2], 1.0 / 3.0));
    }
    *velocity_m_s = speed * 0.995;
    *length_m = plan->path.length_m;
    return isfinite(*velocity_m_s) && *velocity_m_s > 0;
}

static void Transform(const RefPlan *plan, RefPoint *point)
{
    double cosine = cos(plan->origin[2]);
    double sine = sin(plan->origin[2]);
    double x = point->x_m;
    double y = point->y_m;
    double vx = point->vx_m_s;
    double vy = point->vy_m_s;
    double ax = point->ax_m_s2;
    double ay = point->ay_m_s2;
    double jx = point->jx_m_s3;
    double jy = point->jy_m_s3;
    point->x_m = plan->origin[0] + cosine * x - sine * y;
    point->y_m = plan->origin[1] + sine * x + cosine * y;
    point->yaw_rad += plan->origin[2];
    point->vx_m_s = cosine * vx - sine * vy;
    point->vy_m_s = sine * vx + cosine * vy;
    point->ax_m_s2 = cosine * ax - sine * ay;
    point->ay_m_s2 = sine * ax + cosine * ay;
    point->jx_m_s3 = cosine * jx - sine * jy;
    point->jy_m_s3 = sine * jx + cosine * jy;
}

static int ExtendedAt(const RefPlan *plan, double seconds, RefPoint *point)
{
    CurvePoint scalar;
    CurvePoint heading;
    LinePoint line;
    PathPoint path;
    double scale;
    double local;
    double co = cos(plan->spec.path_heading_rad);
    double si = sin(plan->spec.path_heading_rad);
    if (plan->spec.kind == 5)
    {
        scale = plan->duration_s > 0 ? plan->position_time_s / plan->duration_s : 0;
        if (!Line_At(&plan->line, seconds * scale, &line))
        {
            return 0;
        }
        point->x_m = line.x_m;
        point->y_m = line.y_m;
        point->vx_m_s = line.vx_m_s * scale;
        point->vy_m_s = line.vy_m_s * scale;
        point->ax_m_s2 = line.ax_m_s2 * scale * scale;
        point->ay_m_s2 = line.ay_m_s2 * scale * scale;
        if (!Curve_At(&plan->line.curve, seconds * scale, &scalar))
        {
            return 0;
        }
        point->jx_m_s3 = plan->line.ux * scalar.jerk * scale * scale * scale;
        point->jy_m_s3 = plan->line.uy * scalar.jerk * scale * scale * scale;
    }
    else
    {
        if (!Curve_TimeAt(&plan->timing, seconds, &scalar))
        {
            return 0;
        }
        if (plan->spec.kind == 4)
        {
            if (!Path_At(&plan->path, scalar.position, &path))
            {
                return 0;
            }
            point->x_m = co * path.x_m - si * path.y_m;
            point->y_m = si * path.x_m + co * path.y_m;
            point->vx_m_s = (co * path.tx - si * path.ty) * scalar.velocity;
            point->vy_m_s = (si * path.tx + co * path.ty) * scalar.velocity;
            {
                double ax = path.tx * scalar.acceleration + path.dtx * scalar.velocity * scalar.velocity;
                double ay = path.ty * scalar.acceleration + path.dty * scalar.velocity * scalar.velocity;
                double jx = path.tx * scalar.jerk + 3 * path.dtx * scalar.velocity * scalar.acceleration +
                    path.ddtx * scalar.velocity * scalar.velocity * scalar.velocity;
                double jy = path.ty * scalar.jerk + 3 * path.dty * scalar.velocity * scalar.acceleration +
                    path.ddty * scalar.velocity * scalar.velocity * scalar.velocity;
                point->ax_m_s2 = co * ax - si * ay;
                point->ay_m_s2 = si * ax + co * ay;
                point->jx_m_s3 = co * jx - si * jy;
                point->jy_m_s3 = si * jx + co * jy;
            }
        }
        else
        {
            point->x_m = plan->line.ux * scalar.position;
            point->y_m = plan->line.uy * scalar.position;
            point->vx_m_s = plan->line.ux * scalar.velocity;
            point->vy_m_s = plan->line.uy * scalar.velocity;
            point->ax_m_s2 = plan->line.ux * scalar.acceleration;
            point->ay_m_s2 = plan->line.uy * scalar.acceleration;
            point->jx_m_s3 = plan->line.ux * scalar.jerk;
            point->jy_m_s3 = plan->line.uy * scalar.jerk;
        }
    }
    scale = plan->heading_end_s > plan->heading_begin_s ?
        plan->heading.total / (plan->heading_end_s - plan->heading_begin_s) : 0;
    local = fmax(0, fmin(plan->heading.total, (seconds - plan->heading_begin_s) * scale));
    if (!Curve_At(&plan->heading, local, &heading))
    {
        return 0;
    }
    point->yaw_rad = heading.position;
    if (seconds >= plan->heading_begin_s && seconds < plan->heading_end_s)
    {
        point->wz_rad_s = heading.velocity * scale;
        point->angular_accel_rad_s2 = heading.acceleration * scale * scale;
        point->angular_jerk_rad_s3 = heading.jerk * scale * scale * scale;
    }
    point->done = seconds >= plan->duration_s;
    return hypot(point->vx_m_s, point->vy_m_s) <= plan->spec.linear.velocity * 1.0003 &&
        hypot(point->ax_m_s2, point->ay_m_s2) <= plan->spec.linear.acceleration * 1.0003 &&
        hypot(point->jx_m_s3, point->jy_m_s3) <= plan->spec.linear.jerk * 1.0003;
}

static int TimeMake(RefPlan *plan, CurveLimit slow, double distance, double lead, double tail,
                    double heading_begin, double heading_end)
{
    if (!Curve_TimeMake(&plan->timing, (float)distance, (float)lead, (float)tail, slow,
        plan->spec.linear.velocity, 0.0f, plan->spec.tail_velocity_m_s > 0 ?
        plan->spec.tail_velocity_m_s : plan->spec.linear.velocity))
    {
        return 0;
    }
    plan->position_time_s = plan->duration_s = plan->timing.total;
    plan->heading_begin_s = Curve_TimeFind(&plan->timing, fmin(heading_begin, plan->timing.distance));
    plan->heading_end_s = Curve_TimeFind(&plan->timing, fmin(heading_end, plan->timing.distance));
    return plan->heading_begin_s >= 0 && plan->heading_end_s > plan->heading_begin_s &&
        plan->heading.total <= (plan->heading_end_s - plan->heading_begin_s) * 1.000001;
}

static int ExtendedMake(RefPlan *plan)
{
    const RefSpec *input = &plan->spec;
    CurveLimit slow = input->linear;
    double distance;
    double lead;
    double tail;
    double core;
    double heading_begin;
    double heading_end;
    double low;
    double high;
    double middle;
    unsigned index;
    if (input->tangent || !Curve_Make(&plan->heading, input->yaw_rad, input->angular))
    {
        return 0;
    }
    if (input->kind == 5)
    {
        if (!Line_Make(&plan->line, input->x_m, input->y_m, input->linear))
        {
            return 0;
        }
        plan->position_time_s = plan->line.curve.total;
        plan->duration_s = fmax(plan->position_time_s, plan->heading.total);
        plan->heading_end_s = plan->duration_s;
        return 1;
    }
    if (input->kind == 4)
    {
        if (input->x_m || input->y_m || !Path_Make(&plan->path, &input->path))
        {
            return 0;
        }
        distance = plan->path.length_m;
        lead = input->path.lead_m;
        tail = input->path.tail_m;
    }
    else
    {
        if (!Line_Make(&plan->line, input->x_m, input->y_m, input->linear))
        {
            return 0;
        }
        distance = plan->line.curve.distance;
        lead = input->yaw_begin_m;
        tail = distance - input->yaw_end_m;
    }
    core = distance - lead - tail;
    if (!isfinite(input->curve_velocity_m_s) || input->curve_velocity_m_s <= 0 ||
        input->curve_velocity_m_s > input->linear.velocity || core <= 0)
    {
        return 0;
    }
    slow.velocity = input->curve_velocity_m_s;
    heading_begin = lead;
    heading_end = distance - tail;
    if (input->kind == 4 && (input->yaw_begin_m != 0 || input->yaw_end_m != 0))
    {
        heading_begin = input->yaw_begin_m;
        heading_end = input->yaw_end_m;
        if (heading_begin < 0 || heading_begin > lead + 1e-6 ||
            heading_end < distance - tail - 1e-6 || heading_end > distance + 1e-6)
        {
            return 0;
        }
        /* 先按几何限速构建；转姿可利用入弯/出弯直线，不再一律压慢整弯。
         * 若时长不足，有界搜索并且只保留实际满足转姿时间的速度。 */
        if (TimeMake(plan, slow, distance, lead, tail, heading_begin, heading_end))
        {
            return 1;
        }
        if (plan->heading.total == 0)
        {
            return 0;
        }
        high = slow.velocity;
        low = fmin(high, core / (plan->heading.total * 1.001));
        slow.velocity = (float)low;
        if (!TimeMake(plan, slow, distance, lead, tail, heading_begin, heading_end))
        {
            return 0;
        }
        for (index = 0; index < 12; index++)
        {
            middle = (low + high) / 2;
            slow.velocity = (float)middle;
            if (TimeMake(plan, slow, distance, lead, tail, heading_begin, heading_end))
            {
                low = middle;
            }
            else
            {
                high = middle;
            }
        }
        slow.velocity = (float)low;
        return TimeMake(plan, slow, distance, lead, tail, heading_begin, heading_end);
    }
    if (plan->heading.total > 0)
    {
        slow.velocity = (float)fmin(slow.velocity, core / (plan->heading.total * 1.001));
    }
    return TimeMake(plan, slow, distance, lead, tail, heading_begin, heading_end);
}

int Ref_At(const RefPlan *plan, double seconds, RefPoint *point)
{
    CurvePoint scalar;
    LinePoint line;
    PathPoint path;
    double velocity;
    double accel;
    if (!point)
    {
        return 0;
    }
    memset(point, 0, sizeof(*point));
    if (!plan || !plan->valid || !isfinite(seconds) || seconds < 0)
    {
        return 0;
    }
    if (plan->spec.kind >= 4)
    {
        if (!ExtendedAt(plan, seconds, point))
        {
            return 0;
        }
    }
    else if (plan->spec.kind == 1)
    {
        if (!Line_At(&plan->line, seconds, &line))
        {
            return 0;
        }
        point->x_m = line.x_m;
        point->y_m = line.y_m;
        point->vx_m_s = line.vx_m_s;
        point->vy_m_s = line.vy_m_s;
        point->ax_m_s2 = line.ax_m_s2;
        point->ay_m_s2 = line.ay_m_s2;
        if (!Curve_At(&plan->line.curve, seconds, &scalar))
        {
            return 0;
        }
        point->jx_m_s3 = plan->line.ux * scalar.jerk;
        point->jy_m_s3 = plan->line.uy * scalar.jerk;
        point->done = line.reference_done;
    }
    else
    {
        if (!Curve_At(&plan->scalar, seconds, &scalar))
        {
            return 0;
        }
        point->done = scalar.done;
        if (plan->spec.kind == 2)
        {
            point->yaw_rad = scalar.position;
            point->wz_rad_s = scalar.velocity;
            point->angular_accel_rad_s2 = scalar.acceleration;
            point->angular_jerk_rad_s3 = scalar.jerk;
        }
        else
        {
            if (!Path_At(&plan->path, scalar.position, &path))
            {
                return 0;
            }
            velocity = scalar.velocity;
            accel = scalar.acceleration;
            point->x_m = path.x_m;
            point->y_m = path.y_m;
            point->vx_m_s = path.tx * velocity;
            point->vy_m_s = path.ty * velocity;
            point->ax_m_s2 = path.tx * accel + path.dtx * velocity * velocity;
            point->ay_m_s2 = path.ty * accel + path.dty * velocity * velocity;
            point->jx_m_s3 = path.tx * scalar.jerk + 3 * path.dtx * velocity * accel +
                path.ddtx * velocity * velocity * velocity;
            point->jy_m_s3 = path.ty * scalar.jerk + 3 * path.dty * velocity * accel +
                path.ddty * velocity * velocity * velocity;
            if (plan->spec.tangent)
            {
                point->yaw_rad = path.yaw_rad;
                point->wz_rad_s = path.yaw_s * velocity;
                point->angular_accel_rad_s2 = path.yaw_s * accel + path.yaw_ss * velocity * velocity;
                point->angular_jerk_rad_s3 = path.yaw_s * scalar.jerk +
                    3 * path.yaw_ss * velocity * accel + path.yaw_sss * velocity * velocity * velocity;
            }
        }
    }
    Transform(plan, point);
    return isfinite(point->x_m) && isfinite(point->y_m) && isfinite(point->yaw_rad) &&
        fabs(point->wz_rad_s) <= plan->spec.angular.velocity * 1.0001 &&
        fabs(point->angular_accel_rad_s2) <= plan->spec.angular.acceleration * 1.0001 &&
        fabs(point->angular_jerk_rad_s3) <= plan->spec.angular.jerk * 1.0001;
}

int Ref_Make(RefPlan *plan, const RefSpec *spec, double x_m, double y_m, double yaw_rad)
{
    RefSpec input;
    RefPoint point;
    unsigned index;
    if (!plan)
    {
        return 0;
    }
    if (spec)
    {
        input = *spec;
    }
    memset(plan, 0, sizeof(*plan));
    if (!spec || !isfinite(x_m) || !isfinite(y_m) || !isfinite(yaw_rad) ||
        fabs(x_m) > 1e6 || fabs(y_m) > 1e6 || fabs(yaw_rad) > 1e6 ||
        !isfinite(input.x_m) || !isfinite(input.y_m) || !isfinite(input.yaw_rad) ||
        input.kind < 1 || input.kind > 6 || input.tangent > 1 ||
        !isfinite(input.path_heading_rad) || !isfinite(input.curve_velocity_m_s) ||
        !isfinite(input.yaw_begin_m) || !isfinite(input.yaw_end_m) ||
        !isfinite(input.tail_velocity_m_s) || input.tail_velocity_m_s < 0 ||
        !isfinite(input.angular.velocity) || input.angular.velocity <= 0 ||
        !isfinite(input.angular.acceleration) || input.angular.acceleration <= 0 ||
        !isfinite(input.angular.jerk) || input.angular.jerk <= 0)
    {
        return 0;
    }
    plan->spec = input;
    plan->origin[0] = x_m;
    plan->origin[1] = y_m;
    plan->origin[2] = yaw_rad;
    if (input.kind >= 4)
    {
        if (!ExtendedMake(plan))
        {
            return 0;
        }
    }
    else if (input.kind == 1)
    {
        if (input.yaw_rad || input.tangent || !Line_Make(&plan->line, input.x_m, input.y_m, input.linear))
        {
            return 0;
        }
        plan->duration_s = plan->line.curve.total;
    }
    else if (input.kind == 2)
    {
        if (input.x_m || input.y_m || input.tangent ||
            !Curve_Make(&plan->scalar, input.yaw_rad, input.angular))
        {
            return 0;
        }
        plan->duration_s = plan->scalar.total;
    }
    else
    {
        if (input.x_m || input.y_m || input.yaw_rad || !Path_Make(&plan->path, &input.path) ||
            !Curve_Make(&plan->scalar, plan->path.length_m, input.linear))
        {
            return 0;
        }
        plan->duration_s = plan->scalar.total;
    }
    plan->valid = 1;
    for (index = 0; index <= 128; index++)
    {
        if (!Ref_At(plan, plan->duration_s * index / 128, &point))
        {
            plan->valid = 0;
            return 0;
        }
    }
    plan->target[0] = point.x_m;
    plan->target[1] = point.y_m;
    plan->target[2] = point.yaw_rad;
    return 1;
}
