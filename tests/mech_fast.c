/* 合成反馈复现0.46mm残差，验证新手动容差及快速交接，不访问设备。 */
#define main AxisFixtureMain
#include "mech_axis.c"
#undef main
#include "mech_servo.h"
#include "mech_group.h"

static void Feedback(int64_t actual_reach,int64_t target_reach,unsigned elapsed)
{
    now+=elapsed;
    Feed(0,0x35,0);Feed(0,0x36,lift);Feed(0,0x33,lift);Feed(0,0x3A,3);
    Feed(1,0x35,0);Feed(1,0x36,actual_reach);Feed(1,0x33,target_reach);Feed(1,0x3A,3);
    Mech_Poll(&guard,now,1);
}

int main(int argc,char **argv)
{
    unsigned which=argc>1?(unsigned)atoi(argv[1]):0;
    Init();
    if(which==0 || which==1 || which==2)
    {
        guard.manual_range=which!=2;
        int64_t delta=Mech_RawTolerance(1,which==1?MECH_MANUAL_ENDPOINT_UM+200:460);
        Feedback(reach+delta,reach,80);
        if(which==0){CHECK(!guard.fault&&guard.zero_valid);return 0;}
        CHECK(guard.fault==MECH_RANGE&&!guard.zero_valid);
        CHECK(guard.failure.captured&&guard.failure.axis==2&&guard.failure.reason==MECH_RANGE);
        CHECK(guard.failure.position_um[1]<(which==2?-300:-MECH_MANUAL_ENDPOINT_UM));
        MechFailure saved=guard.failure;
        Mech_Freeze(&guard,104,1,now+1);
        CHECK(!memcmp(&saved,&guard.failure,sizeof(saved)));return 0;
    }
    if(which==3 || which==4 || which==5)
    {
        goal.axis=1;goal.target_um=84000;goal.rpm=100;goal.timeout_ms=6477;
        CHECK(!Axis_Manual(&axis,&goal,now));
        BusFrame frames[2];CHECK(Axis_Take(&axis,frames,now)==2);
        Axis_Sent(&axis,goal.id,1,++now);Ack(2);
        int64_t expected=axis.before_target+axis.delta_raw;
        int64_t residual=Mech_RawTolerance(1,which==4?1200:460);
        for(unsigned i=0;i<4;i++)
        {
            Feedback(expected+residual,expected,80);Axis_Poll(&axis,now,1);
        }
        if(which==3)
        {
            CHECK(axis.state==AXIS_COMPLETE&&guard.zero_valid&&!guard.fault);
            CHECK(!Mech_Ready(&guard,now,1));return 0;
        }
        if(which==4)
        {
            CHECK(axis.state!=AXIS_COMPLETE);
            now=axis.started_ms+goal.timeout_ms;
            Feedback(expected+residual,expected,1);Axis_Poll(&axis,now,1);
            CHECK(axis.reason==AXIS_TIMEOUT&&guard.failure.captured);
            CHECK(guard.failure.axis==2&&guard.failure.reason==AXIS_TIMEOUT);return 0;
        }
        /* 另一轴真实非零速度仍停车，不能靠容差跳过。 */
        goal.id++;goal.target_um=40000;CHECK(!Axis_Manual(&axis,&goal,now));
        CHECK(Axis_Take(&axis,frames,now)==2);Axis_Sent(&axis,goal.id,1,++now);Ack(2);
        Feed(0,0x35,1);Axis_Poll(&axis,now,1);
        CHECK(axis.reason==AXIS_OTHER);return 0;
    }
    if(which==6 || which==7)
    {
        guard.manual_range=1;
        Feedback(reach,reach,100);Feedback(reach,reach,100);
        CHECK(!Mech_Ready(&guard,now,1));
        if(which==6)now+=251;else Feed(1,0x3A,13);
        Mech_Poll(&guard,now,1);
        CHECK(guard.fault==(which==6?MECH_STALE:MECH_DRIVER)&&guard.failure.captured);return 0;
    }
    if(which==8)
    {
        Servo servo;int32_t initial[3]={0,180000,98000};ServoOutput output;
        Servo_Init(&servo,&guard,0);CHECK(!Servo_SeedManual(&servo,initial,now,1));
        servo.output_mask=2;
        ServoGoal request={1,1,3000,180000,2,1,150000};
        CHECK(!Servo_Manual(&servo,&request,now));
        CHECK(servo.state==SERVO_MANUAL_DONE&&!Servo_Take(&servo,&output,now));
        request.id++;request.channel=1;request.target_mdeg=0;
        CHECK(!Servo_Manual(&servo,&request,now));
        CHECK(servo.state==SERVO_ACCEPTED&&Servo_Take(&servo,&output,now));return 0;
    }
    if(which==10 || which==11)
    {
        Servo servo;Group group;int32_t initial[3]={0,0,0};BusFrame frames[2];
        Servo_Init(&servo,&guard,0);CHECK(!Servo_SeedManual(&servo,initial,now,1));
        servo.output_mask=7;guard.manual_range=1;Group_Init(&group,&axis,&servo);
        GroupGoal request={1,1,6477,31,{0,84000,0,0,0},{30,100,250000,150000,250000},{0,0},0};
        for(unsigned stage=0;stage<(which==11?2U:1U);stage++)
        {
            if(stage){request.id++;request.target[1]=0;request.speed[1]=30;}
            CHECK(!Group_Begin(&group,&request,now));CHECK(Group_Take(&group,frames,now)==2);
            Group_Sent(&group,1,now);
            MotorReply reply={0};reply.valid=1;reply.address=6;reply.code=0xFD;reply.result=2;
            reply.tick=now;reply.rx_sequence=sequence++;Group_Ack(&group,&reply,now);
            int64_t expected=group.axes[1].before_target+group.axes[1].delta_raw;
            for(unsigned i=0;i<4;i++)
            {
                Feedback(expected+(stage?-1:1)*Mech_RawTolerance(1,which==11?638:460),expected,80);
                Group_Poll(&group,now,1);
            }
            CHECK(group.state==GROUP_DONE&&!guard.fault&&guard.zero_valid);
        }
        return 0;
    }
    /* 自动到位依旧按原始严格档，不继承手动0.5mm。 */
    goal.target_um=2000;Start();Ack(2);
    lift+=axis.delta_raw+Mech_RawTolerance(0,460);
    for(unsigned i=0;i<4;i++){Round();Axis_Poll(&axis,now,1);}
    CHECK(axis.state!=AXIS_COMPLETE&&guard.manual_range==0);
    return 0;
}
