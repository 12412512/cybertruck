#include "line.h"
#include <math.h>
#include <string.h>

int Line_Make(LinePlan *plan, float dx_m, float dy_m, CurveLimit limit)
{
    double distance;
    if (!plan)
    {
        return 0;
    }
    memset(plan, 0, sizeof(*plan));
    distance = sqrt((double)dx_m * dx_m + (double)dy_m * dy_m);
    if (!isfinite(distance) || !Curve_Make(&plan->curve, (float)distance, limit))
    {
        return 0;
    }
    plan->dx_m = dx_m;
    plan->dy_m = dy_m;
    plan->ux = distance > 0 ? dx_m / distance : 0;
    plan->uy = distance > 0 ? dy_m / distance : 0;
    return 1;
}

int Line_At(const LinePlan *plan, double seconds, LinePoint *point)
{
    CurvePoint scalar;
    if (!point)
    {
        return 0;
    }
    memset(point, 0, sizeof(*point));
    if (!plan || !Curve_At(&plan->curve, seconds, &scalar))
    {
        return 0;
    }
    point->x_m = scalar.done ? plan->dx_m : (float)(plan->ux * scalar.position);
    point->y_m = scalar.done ? plan->dy_m : (float)(plan->uy * scalar.position);
    point->vx_m_s = (float)(plan->ux * scalar.velocity);
    point->vy_m_s = (float)(plan->uy * scalar.velocity);
    point->ax_m_s2 = (float)(plan->ux * scalar.acceleration);
    point->ay_m_s2 = (float)(plan->uy * scalar.acceleration);
    point->reference_done = scalar.done;
    return 1;
}
