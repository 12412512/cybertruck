/* 正式底盘纯计算库的主机桥，不链接HAL或设备驱动。 */
#include "kin.h"
#include "yaw.h"

#ifdef _WIN32
#define EXPORT __declspec(dllexport)
#else
#define EXPORT
#endif

static YawHistory history;

EXPORT int Test_Inverse(const float *geometry, int layout, const float *body, float *rpm)
{
    KinGeometry config = {geometry[0], geometry[1], geometry[2], (KinLayout)layout};
    KinBody speed = {body[0], body[1], body[2]};
    return Kin_Inverse(config, speed, rpm);
}

EXPORT int Test_Forward(const float *geometry, int layout, const float *rpm, float *body)
{
    KinGeometry config = {geometry[0], geometry[1], geometry[2], (KinLayout)layout};
    KinBody speed;
    int result = Kin_Forward(config, rpm, &speed);
    body[0] = speed.vx_m_s;
    body[1] = speed.vy_m_s;
    body[2] = speed.wz_rad_s;
    return result;
}

EXPORT float Test_Limit(float *rpm, float limit)
{
    return Kin_Limit(rpm, limit);
}

EXPORT int Test_YawInit(uint32_t tick, float angle, float rate, uint32_t gap)
{
    return Yaw_Init(&history, tick, angle, rate, gap);
}

EXPORT int Test_YawPush(uint32_t tick, float angle)
{
    return Yaw_Push(&history, tick, angle);
}

EXPORT int Test_YawAt(uint32_t tick, float *angle)
{
    return Yaw_At(&history, tick, angle);
}

EXPORT int Test_YawFresh(uint32_t tick)
{
    return Yaw_Fresh(&history, tick);
}

EXPORT int Test_YawFault(void)
{
    return history.fault;
}
