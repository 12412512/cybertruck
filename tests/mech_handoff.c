#define PORT_MAIN PreviousPortMain
#include "mech_port.c"

static uint8_t Request(uint8_t *data, unsigned op, unsigned stopped)
{
    uint8_t reply[128];
    uint16_t size = 0;
    Frame_Put(data, op, 4);
    CHECK(MechPort_Handoff(71, data, 76, reply, &size, tick, (uint8_t)stopped) == COMM_OK);
    CHECK(size == 4);
    return (uint8_t)Frame_Get(reply, 4);
}

int main(int argc, char **argv)
{
    unsigned which = argc > 1 ? (unsigned)atoi(argv[1]) : 0;
    uint8_t data[96] = {0};
    MechPort_Init();
    Prepare();
    manual_mode = guard.manual_range = 1;
    pwm_mask = 7; /* 模拟已有明确保持输出，不调用PWM。 */
    Frame_Put(data + 4, 10, 4);
    Frame_Put(data + 12, 1, 4);
    Frame_Put(data + 16, 1, 4);
    for (unsigned i = 0; i < 3; i++)
        Frame_Put(data + 28 + i * 4, (uint32_t)servo.command_mdeg[i], 4);
    data[40] = 1;
    if(which>=9){
        /* 已接纳的0.6mm终点残差不应被交接重新收紧；漂移仍独立0.5mm。 */
        int64_t raw=which==10?Mech_RawTolerance(0,1200):which==11?-Mech_RawTolerance(0,400):Mech_RawTolerance(0,600);
        Feed(0,0x36,raw);Feed(0,0x33,0);
        if(which==10){CHECK(Request(data,1,1));return 0;}
        CHECK(!Request(data,1,1));CHECK(!Request(data,2,1));
        Feed(0,0x36,which==11?Mech_RawTolerance(0,200):raw+Mech_RawTolerance(0,100));
        CHECK(MechPort_HoldCheck(tick)==(which!=11));
        CHECK(!writes&&!starts&&!stops);return 0;
    }
    if (which == 1) data[12] = 2;
    if (which == 2) data[40] = 0;
    if (which == 3) Frame_Put(data + 20, (uint32_t)-1000, 4);
    if (which == 4) Frame_Put(data + 20, 10000, 4);
    if (which == 5) data[72] = 1;
    if (which == 5)
    {
        uint8_t reply[128]; uint16_t size = 0;
        Frame_Put(data, 1, 4);
        CHECK(MechPort_Handoff(71,data,76,reply,&size,tick,1)==COMM_FORMAT);
        return 0;
    }
    if (which >= 1 && which <= 4)
    {
        CHECK(Request(data, 1, 1) && !MechPort_Holding());
        CHECK(!writes && !starts && guard.zero_valid);
        return 0;
    }
    CHECK(!Request(data, 1, 1));
    CHECK(!Request(data, 1, 1) && hold_phase == 1);
    if (which == 6)
    {
        CHECK(!Request(data, 3, 1) && !hold_phase && hold_epoch == 1);
        CHECK(Request(data, 2, 1));
        return 0;
    }
    CHECK(!Request(data, 2, 1) && MechPort_Holding() && !MechPort_Owned());
    CHECK(hold_epoch == 1 && !Request(data, 2, 1) && hold_epoch == 1);
    CHECK(Request(data, 4, 0) && MechPort_Holding());
    if (which == 7)
    {
        data[40]++;
        CHECK(Request(data, 2, 1) == MECH_VERSION);
        return 0;
    }
    if (which == 8)
    {
        tick += 251;
        MechPort_Run(tick, 0, 0);
        CHECK(!guard.zero_valid && guard.fault && !MechPort_Holding() && stops);
        return 0;
    }
    for (unsigned n = 0; n < 5; n++)
    {
        tick += 40;
        for (unsigned a = 0; a < 2; a++)
        {
            Feed(a,0x35,0); Feed(a,0x36,0); Feed(a,0x33,0); Feed(a,0x3A,3);
        }
        MechPort_Run(tick, 0, 0);
        CHECK(guard.zero_valid && !guard.fault && !guard.base_stopped && MechPort_Holding());
    }
    CHECK(!Request(data, 4, 1) && !MechPort_Holding() && MechPort_Owned());
    CHECK(hold_epoch == 2 && guard.zero_version == 1 && pwm_mask == 7);
    CHECK(!writes && !starts && !stops);
    return 0;
}
