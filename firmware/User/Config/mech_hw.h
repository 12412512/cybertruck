#ifndef MECH_HW_H
#define MECH_HW_H

/* 核对旧mech_cfg.h及本工程IOC。当前只供被动校验，不注册机构收发或PWM。 */
#define MECH_LIFT_ID                 5U
#define MECH_REACH_ID                6U
#define MECH_LIFT_SIGN               1
#define MECH_REACH_SIGN             -1
#define MECH_LIFT_UM_PER_320       4098L
#define MECH_REACH_UM_PER_320     12400L
#define MECH_RAW_PER_320          6554L
#define MECH_LIFT_MAX_UM         49000L
#define MECH_REACH_MAX_UM        50000L
#define MECH_FEEDBACK_AGE_MS       250U
#define MECH_ZERO_STABLE_MS        300U
#define MECH_STABLE_SAMPLES          5U
#define MECH_STILL_RAW              16L
#define MECH_STABLE_RAW              8L
#define MECH_FEEDBACK_TOL_UM       300L

/* 板级输出唯一来源仍为Car.ioc；这里只定义逻辑通道，不引入HAL。 */
#define MECH_TRAY_CHANNEL            1U
#define MECH_ARM_CHANNEL             2U
#define MECH_GRIP_CHANNEL            3U
#define MECH_ARM_RANGE_MDEG      359000L
#define MECH_TRAY_RANGE_MDEG     270000L
#define MECH_GRIP_MAX_MDEG        98000L
#define MECH_PWM_MIN_US            500U
#define MECH_PWM_SPAN_US          2000U
#define MECH_CLEARANCE_UM         1000L
#define MECH_EXTEND_MIN_LIFT_UM  45000L
#define MECH_ARM_CLEAR_MDEG      180000L

#endif
