/* 复现.40首次104快照：目标33偏差8raw、实际83.950mm、速度0。 */
#define main AxisFixtureMain
#include "mech_axis.c"
#undef main

static void Sample(int64_t actual,int64_t target)
{
    now+=80;
    Feed(0,0x35,0);Feed(0,0x36,lift);Feed(0,0x33,lift);Feed(0,0x3A,3);
    Feed(1,0x35,0);Feed(1,0x36,actual);Feed(1,0x33,target);Feed(1,0x3A,3);
    Mech_Poll(&guard,now,1);
}

int main(int argc,char **argv)
{
    unsigned which=argc>1?(unsigned)atoi(argv[1]):0;
    Init();guard.manual_range=1;guard.zero_actual[1]-=16;
    goal.axis=1;goal.rpm=100;goal.target_um=84000;goal.timeout_ms=6477;
    BusFrame frames[2];
    CHECK(!Axis_Manual(&axis,&goal,now));CHECK(Axis_Take(&axis,frames,now)==2);
    Axis_Sent(&axis,goal.id,1,++now);Ack(2);
    int64_t expected=axis.before_target+axis.delta_raw;
    int64_t target=expected-8,actual=target;
    if(which==1)target=expected-Mech_RawTolerance(1,1200);
    if(which==2)actual=expected-Mech_RawTolerance(1,1200);
    for(unsigned i=0;i<4;i++){Sample(actual,target);Axis_Poll(&axis,now,1);}
    if(which==1){CHECK(axis.state!=AXIS_COMPLETE);return 0;}
    if(which==2){CHECK(axis.reason==AXIS_REVERSE);return 0;}
    if(axis.state!=AXIS_COMPLETE){now=axis.started_ms+goal.timeout_ms;Sample(actual,target);Axis_Poll(&axis,now,1);fprintf(stderr,"reason=%lu\n",(unsigned long)axis.reason);}
    CHECK(axis.state==AXIS_COMPLETE&&!guard.fault&&guard.zero_valid);
    /* 下一段仍按固定零点核对，不把8raw变成新的坐标基准。 */
    goal.id++;goal.target_um=0;CHECK(!Axis_Manual(&axis,&goal,now));
    CHECK(Axis_Take(&axis,frames,now)==2);Axis_Sent(&axis,goal.id,1,++now);Ack(2);
    target=axis.before_target+axis.delta_raw;
    for(unsigned i=0;i<4;i++){Sample(target,target);Axis_Poll(&axis,now,1);}
    CHECK(axis.state==AXIS_COMPLETE&&!guard.fault);
    goal.id++;goal.target_um=40000;
    /* 改变外部目标超过容差，仍禁止接管，不能靠重新锚定隐去变化。 */
    Sample(target-Mech_RawTolerance(1,1200),target-Mech_RawTolerance(1,1200));
    CHECK(Axis_Manual(&axis,&goal,now)!=MECH_OK);
    return 0;
}
