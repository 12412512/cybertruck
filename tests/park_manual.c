/* 正式板级停车门槛：只移除手动机械调试轮速项，不修改底盘Ready。 */
#define BASE_STARTUP_NO_MAIN
#include "base_startup.c"

int main(int argc,char **argv)
{
    unsigned which=argc>1?(unsigned)atoi(argv[1]):0;
    BusWheel *w=(BusWheel *)Bus_Wheels();
    uint8_t reply[128];uint16_t size=0;
    tick=12345;mech_active=mech_manual=1;executor.stop_confirmed=1;
    memset(w,0,4*sizeof(*w));
    for(unsigned i=0;i<4;i++)
    {
        w[i].speed.valid=w[i].status.valid=w[i].position.valid=1;
        w[i].speed.tick=w[i].status.tick=w[i].position.tick=tick-10;
        w[i].status.value=3;
    }
    w[2].speed.value=-3;
    if(which==1)mech_manual=0;
    if(which==2)executor.mode=EXEC_ACTIVE;
    if(which==3)bus_diag.active=1;
    if(which==4)bus_diag.pending_stop_count=1;
    if(which==5)bus_diag.stop_sequence=1;
    if(which==6)executor.stop_confirmed=0;
    if(which==7)executor.fault=1;
    if(which==8)can_diag.errors=1;
    if(which==9)w[0].speed.valid=0;
    if(which==10)w[0].status.tick=tick-251;
    if(which==11)w[0].status.value=7;
    if(which==12)w[0].position.valid=0;
    if(which==13){executor.mode=EXEC_OBSERVING;executor.enabled_wheels=4;}
    if(which==14)startup_pending=1;
    if(which==15)bus_diag.error=BUS_ERROR_CAN;
    if(which==16)executor.mode=EXEC_STOPPING;
    if(which==17)w[2].speed.value=-100;
    CHECK(!Ready(tick,0,0)); /* 跑图/自动停车仍拒绝非零轮速。 */
    uint8_t expected=which==0||which==13||which==17;
    CHECK(Park_Ready(tick)==expected);
    Park_Capture(tick);
    CHECK(park_failure.captured==!expected);
    CHECK(Command(69,0,0,reply,&size,tick)==COMM_OK && size==124);
    CHECK(Frame_Get(reply,4)==2);
    if(expected)
    {
        CHECK(Frame_Get(reply+120,4)==3);
        mech_manual=0;
        CHECK(!Park_Ready(tick)); /* 退出手动立即恢复严格判定，不复活任何旧动作。 */
    }
    printf("manual parking case %u PASS: no IO\n",which);
    return 0;
}
