#ifndef MECH_SERVO_H
#define MECH_SERVO_H

#include "mech_guard.h"

enum
{
    SERVO_IDLE, SERVO_ACCEPTED, SERVO_SLEWING, SERVO_OUTPUT,
    SERVO_SETTLING, SERVO_COMMAND_DONE, SERVO_CONFIRMED, SERVO_FAILED, SERVO_CANCELLED, SERVO_MANUAL_DONE
};
enum
{
    SERVO_ID = 200, SERVO_BUSY, SERVO_REFERENCE, SERVO_OUTPUT_ERROR,
    SERVO_TIMEOUT, SERVO_HOLD, SERVO_EVIDENCE, SERVO_CANCEL
};
enum { SERVO_MODEL = 1, SERVO_MANUAL = 2, SERVO_VISION = 3 };
typedef struct
{
    uint32_t id, zero_version, timeout_ms;
    int32_t target_mdeg;
    uint8_t channel, release_confirmed;
    uint32_t speed_mdeg_s; /* 0沿用45度/秒；新调试接口明确填写指令角速度。 */
} ServoGoal;
typedef struct
{
    uint32_t id, sequence, zero_version;
    int32_t command_mdeg;
    uint16_t pulse_us;
    uint8_t channel;
} ServoOutput;
typedef struct
{
    uint64_t boot, session;
    uint32_t id, zero_version, sequence, captured_ms;
    int32_t target_mdeg;
    uint8_t channel, kind;
} ServoEvidence;
typedef struct
{
    MechGuard *guard;
    uint64_t boot, session;
    ServoGoal goal;
    ServoOutput pending;
    int32_t command_mdeg[3];
    uint32_t zero_version, state, reason, started_ms, issued_ms, last_step_ms;
    uint32_t output_sequence, evidence_sequence;
    uint8_t reference_mask, output_mask, confirmed_mask, taken, simulation;
    uint8_t hold_possible, confirmation_kind, manual;
} Servo;

/* 全部为无IO状态机。初始化不输出；Seed仅记录已确认的初始参考。
 * 新实车必须人工核对参考，不可把网页模型锚点用作物理参考。 */
void Servo_Init(Servo *servo, MechGuard *guard, uint8_t simulation);
uint32_t Servo_Seed(Servo *servo, const int32_t initial_mdeg[3], uint32_t now_ms,
                     uint8_t manually_confirmed);
uint32_t Servo_Begin(Servo *servo, const ServoGoal *goal, uint32_t now_ms);
/* 手动调试不强制收拢/转臂顺序；仍保留零点、反馈、范围和释放确认。 */
uint32_t Servo_Manual(Servo *servo, const ServoGoal *goal, uint32_t now_ms);
uint32_t Servo_SeedManual(Servo *servo, const int32_t initial_mdeg[3], uint32_t now_ms,
                          uint8_t manually_confirmed);
uint8_t Servo_Take(Servo *servo, ServoOutput *output, uint32_t now_ms);
void Servo_Applied(Servo *servo, uint32_t id, uint32_t sequence, uint8_t ok, uint32_t now_ms);
void Servo_Poll(Servo *servo, uint32_t now_ms, uint8_t base_stopped);
uint32_t Servo_Confirm(Servo *servo, const ServoEvidence *evidence, uint32_t now_ms);
void Servo_Stop(Servo *servo, uint32_t reason, uint32_t now_ms);
/* 只返回参考是否已确认；不表示有真实角度传感器。 */
uint8_t Servo_Reference(const Servo *servo, uint8_t channel);

#endif
