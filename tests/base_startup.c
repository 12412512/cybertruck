/* 联调正式Base_Run/Comm/Exec/CAN/Bus，模拟四号驱动稍晚就绪；不访问设备。 */
#define main ExistingQueryMain
#define addresses fixture_addresses
#define Query FixtureQuery
#include "can_query_capture.c"
#undef Query
#undef addresses
#undef main
#include "../firmware/User/Board/base_port.c"

static unsigned physical_key;
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *can, const CAN_FilterTypeDef *filter)
{ (void)can; (void)filter; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *can)
{ (void)can; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *can, uint32_t flags)
{ (void)can; (void)flags; return HAL_OK; }
uint32_t HAL_GPIO_ReadPin(void *port, uint32_t pin)
{ (void)pin; return port == START_KEY_GPIO_Port && physical_key ? GPIO_PIN_RESET : GPIO_PIN_SET; }
void HAL_GPIO_WritePin(void *port, uint32_t pin, uint32_t state)
{ CHECK(port == BUZZER_GPIO_Port && pin == BUZZER_Pin && state == GPIO_PIN_SET); }
int Port_Imu(ImuSample *sample) { (void)sample; return 0; }
uint16_t Port_Diag(uint8_t *data) { (void)data; return 0; }
#ifndef BASE_REAL_MECH
void MechPort_Init(void) { }
static uint8_t mech_active;
uint8_t MechPort_Active(uint32_t *id,uint32_t *channel) { *id=mech_active?32:0;*channel=2;return mech_active; }
static uint8_t reset_ready;
uint8_t MechPort_CanReset(uint32_t now,uint32_t version) { (void)now;return reset_ready && version==7; }
static uint8_t reset_pwm_off=1;
uint8_t MechPort_PwmOff(void) { return reset_pwm_off; }
void MechPort_Feed(const BusFrame *frame) { (void)frame; }
void MechPort_Run(uint32_t now, uint8_t stopped, uint32_t fault)
{ (void)now; (void)stopped; (void)fault; }
void MechPort_Stop(uint32_t reason, uint32_t now) { (void)reason; (void)now; }
uint8_t MechPort_Command(uint8_t type, const uint8_t *data, uint16_t size,
                        uint8_t *reply, uint16_t *reply_size, uint32_t now)
{ (void)type; (void)data; (void)size; (void)reply; (void)reply_size; (void)now; return COMM_BUSY; }
uint8_t MechPort_Query(BusFrame frames[2], uint32_t now)
{ (void)frames; (void)now; CHECK(0); return 0; }
uint8_t MechPort_Output(BusFrame frames[2], uint32_t now)
{ (void)frames; (void)now; CHECK(0); return 0; }
void MechPort_Sent(uint8_t accepted, uint32_t now) { (void)accepted; (void)now; }
void MechPort_QueryAccept(void) { CHECK(0); }
uint8_t MechPort_Owned(void) { return 0; }
#endif

#ifndef BASE_STARTUP_NO_MAIN
int main(int argc, char **argv)
{
    unsigned which = argc > 1 ? (unsigned)atoi(argv[1]) : 0;
    uint32_t start, elapsed, first_query = UINT32_MAX;
    unsigned stop_mask = 0, injected = 0;
    uint8_t reply[128], data[52] = {0};
    uint16_t size = 0;
    if (which == 2)
        tick = UINT32_MAX - 500U;
    start = tick;
    physical_key = which == 3;
    Can_Refresh();
    Comm_Init(123, tick);
    Base_Init();
    CHECK(startup_pending && bus_diag.pending_stop_count == 6 && !sends);
    CHECK(Command(EXEC_CONFIG, data, sizeof(data), reply, &size, tick) == COMM_BUSY);
    for (elapsed = 0; elapsed < 3000; elapsed++)
    {
        tick = start + elapsed;
        if (occupied)
        {
            unsigned address = last.id >> 8, command = last.data[0];
            CHECK(command == 0xFE || command == 0x35 || command == 0x36 || command == 0x3A);
            if (which == 5 && !injected && elapsed >= 5)
            {
                injected = 1;
                Fail(HAL_CAN_ERROR_TX_TERR0);
            }
            else
            {
                Done();
                if (command == 0xFE)
                {
                    CHECK(address >= 1 && address <= 6);
                    stop_mask |= 1U << (address - 1U);
                    CHECK(elapsed < BUS_STARTUP_SETTLE_MS || which == 1 || which == 4);
                }
                else
                {
                    if (first_query == UINT32_MAX)
                        first_query = elapsed;
                    if (which != 6)
                        CHECK(elapsed >= BUS_STARTUP_SETTLE_MS);
                    if (address != 4 ||
                        (elapsed >= (which == 1 ? 2000U : 500U) && !(which == 4 && elapsed >= 1600U)))
                        Reply(address, command, 0);
                }
            }
        }
        Base_Run();
        CHECK(!executor.enabled_wheels && !executor.have_goal && !executor.arm_ms);
        CHECK(start_key.phase != START_FIRED && !start_key.beep);
    }
    CHECK(stop_mask == 63);
    if (which != 6)
        CHECK(first_query >= BUS_STARTUP_SETTLE_MS);
    if (which == 0 || which == 2 || which == 3)
    {
        CHECK(!bus_timeout.valid && !bus_diag.query_timeouts && !can_diag.errors);
        CHECK(!executor.fault && executor.mode == EXEC_DISABLED && executor.stop_confirmed);
        CHECK(!startup_pending);
    }
    else if (which == 1 || which == 4)
    {
        CHECK(bus_timeout.valid && bus_timeout.address == 4 && bus_timeout.code == 0x35);
        CHECK((uint32_t)(bus_timeout.tick - bus_timeout.start) == BUS_REPLY_TIMEOUT_MS);
        CHECK((uint32_t)(bus_timeout.start - start) >= (which == 4 ? 1600U : BUS_STARTUP_SETTLE_MS));
        CHECK(executor.fault && executor.mode == EXEC_LOCKED && bus_diag.locked);
        CHECK(!can_diag.errors);
        /* 一号场景2000ms后恢复反馈，原故障仍不能自动清除或续跑。 */
    }
    else if (which == 5)
    {
        CHECK(injected && can_diag.errors == 1 && can_capture.valid);
        CHECK(executor.fault && executor.mode == EXEC_LOCKED && bus_diag.locked);
    }
    else if (which == 6)
    {
        /* 仅旧逻辑负对照使用：四号500ms才就绪，早查询导致100ms缺回复锁存。 */
        CHECK(first_query < BUS_STARTUP_SETTLE_MS);
        CHECK(bus_timeout.valid && bus_timeout.address == 4 && bus_timeout.code == 0x35);
        CHECK((uint32_t)(bus_timeout.start - start) < BUS_STARTUP_SETTLE_MS);
        CHECK((uint32_t)(bus_timeout.tick - bus_timeout.start) == BUS_REPLY_TIMEOUT_MS);
        CHECK(bus_timeout.rx_sequence == bus_timeout.barrier);
        CHECK(executor.fault && executor.mode == EXEC_LOCKED);
        printf("old startup reproduced ID%u query0x%02x start%u timeout%u\n",
               bus_timeout.address, bus_timeout.code, bus_timeout.start, bus_timeout.tick);
    }
    else
        CHECK(0);
    printf("base startup case %u PASS: offline only\n", which);
    return 0;
}
#endif

#ifdef BASE_RESET_COUNTER
static unsigned reset_calls;
void NVIC_SystemReset(void) { reset_calls++; }
#else
void NVIC_SystemReset(void) { abort(); }
#endif
