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
    if (plan->spec.kind != 3)
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
        if (plan->spec.tangent)
        {
            *yaw_rad_s = fmax(*yaw_rad_s, yaw);
        }
    }
    *velocity_m_s *= 1.0001;
    *yaw_rad_s *= 1.0001;
    return isfinite(*velocity_m_s) && isfinite(*yaw_rad_s);
}

static void Transform(const RefPlan *plan, RefPoint *point)
{
    double cosine = cos(plan->origin[2]);
    double sine = sin(plan->origin[2]);
    double x = point->x_m;
    double y = point->y_m;
    double vx = point->vx_m_s;
    double vy = point->vy_m_s;
    point->x_m = plan->origin[0] + cosine * x - sine * y;
    point->y_m = plan->origin[1] + sine * x + cosine * y;
    point->yaw_rad += plan->origin[2];
    point->vx_m_s = cosine * vx - sine * vy;
    point->vy_m_s = sine * vx + cosine * vy;
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
    if (plan->spec.kind == 1)
    {
        if (!Line_At(&plan->line, seconds, &line))
        {
            return 0;
        }
        point->x_m = line.x_m;
        point->y_m = line.y_m;
        point->vx_m_s = line.vx_m_s;
        point->vy_m_s = line.vy_m_s;
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
        input.kind < 1 || input.kind > 3 || input.tangent > 1 ||
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
    if (input.kind == 1)
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
