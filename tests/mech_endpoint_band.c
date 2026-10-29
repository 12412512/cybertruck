/* Frozen ID6 evidence replay; synthetic feedback only, no hardware access. */
#define main AxisFixtureMain
#include "mech_axis.c"
#undef main
int main(int argc,char **argv)
{
    int good=argc>1?atoi(argv[1]):0;
    lift=3;reach=-11;Init();guard.manual_range=1;
    guard.zero_target[0]=1;
    goal.id=13;goal.axis=1;goal.rpm=200;goal.accel=200;goal.accel_set=1;
    goal.target_um=50000;goal.timeout_ms=3781;
    BusFrame frames[2];
    CHECK(!Axis_Manual(&axis,&goal,now));
    CHECK(Axis_Take(&axis,frames,now)==2);
    CHECK(axis.goal_pulses==1290 && axis.delta_raw==-26419);
    CHECK(frames[0].data[4]==200);
    Axis_Sent(&axis,goal.id,1,++now);Ack(2);
    for(unsigned i=0;i<4;i++) {
        now+=80;
        Feed(0,0x35,0);Feed(0,0x36,1);Feed(0,0x33,1);Feed(0,0x3A,3);
        Feed(1,0x35,0);Feed(1,0x36,good==2?-27065:good?-26673:-26767);
        Feed(1,0x33,-26429);Feed(1,0x3A,3);
        Mech_Poll(&guard,now,1);Axis_Poll(&axis,now,1);
        if(guard.fault)break;
    }
    if(good!=2) { CHECK(axis.state==AXIS_COMPLETE && !guard.fault); CHECK(!Mech_Ready(&guard,now,1)); }
    else CHECK(guard.fault==106 && guard.failure.actual[1]==-27065 && guard.failure.rpm[1]==0);
    printf("case=%s reason=%lu\n",good?"within_0.5mm":"captured_overshoot",(unsigned long)guard.fault);
    return 0;
}
