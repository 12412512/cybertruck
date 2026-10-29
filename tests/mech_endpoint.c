/* 上一目标有跟随残差时，下一段必须围绕固定目标判定，而非移动实测起点。 */
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
    Init();guard.manual_range=1;
    goal.axis=1;goal.rpm=100;goal.target_um=84000;goal.timeout_ms=6477;
    BusFrame frames[2];
    for(unsigned stage=0;stage<2;stage++)
    {
        if(stage){goal.id++;goal.target_um=which==2?40000:0;}
        CHECK(!Axis_Manual(&axis,&goal,now));
        CHECK(Axis_Take(&axis,frames,now)==2);
        Axis_Sent(&axis,goal.id,1,++now);Ack(2);
        int64_t target=axis.before_target+axis.delta_raw;
        int64_t residue=Mech_RawTolerance(1,460);
        /* 末端+0.46mm到下一段-0.46mm，绝对误差合格，旧位移比较却多算0.92mm。 */
        int64_t actual=target+(stage?-residue:residue);
        if(which==4)actual=target+(stage?residue:-residue);
        if(stage&&which==1)actual=target; /* 完全到位也不应因上一段残差超时。 */
        if(stage&&which==2)actual=target+Mech_RawTolerance(1,1200); /* 真正过冲仍停。 */
        if(stage&&which==3)actual=axis.before_actual-Mech_RawTolerance(1,1200); /* 真反向仍停。 */
        for(unsigned i=0;i<4;i++){Sample(actual,target);Axis_Poll(&axis,now,1);}
        if(stage&&(which==2||which==3)){CHECK(axis.reason==AXIS_REVERSE);return 0;}
        if(axis.state!=AXIS_COMPLETE&&!axis.reason){now=axis.started_ms+goal.timeout_ms;Sample(actual,target);Axis_Poll(&axis,now,1);}
        if(axis.state!=AXIS_COMPLETE)fprintf(stderr,"stage=%u reason=%lu actual=%lld target=%lld before=%lld delta=%lld\n",stage,(unsigned long)axis.reason,(long long)actual,(long long)target,(long long)axis.before_actual,(long long)axis.delta_raw);
        CHECK(axis.state==AXIS_COMPLETE&&!guard.fault&&guard.zero_valid);
    }
    return 0;
}
