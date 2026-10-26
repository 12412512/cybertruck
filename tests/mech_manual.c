/* 手动策略复用正式执行器与原单轴合成反馈；无设备IO。 */
#define main AxisFixtureMain
#include "mech_axis.c"
#undef main
#include "mech_pose.h"

static Servo servo;
static uint32_t output_count;
static void ServoTick(void)
{
    ServoOutput output;
    Round();
    Servo_Poll(&servo, now, 1);
    if (Servo_Take(&servo, &output, now))
    {
        output_count++;
        CHECK(output.pulse_us == Mech_Pulse(output.channel, output.command_mdeg));
        Servo_Applied(&servo, output.id, output.sequence, 1, now);
    }
}
int main(int argc, char **argv)
{
    unsigned which=argc>1?(unsigned)atoi(argv[1]):0,n;
    int32_t initial[3]={30000,90000,98000};
    ServoGoal request={1,1,10000,60000,1,0,0};
    Init();
    Servo_Init(&servo,&guard,0);
    if(which<=3)
    {
        goal.axis=1;
        CHECK(Axis_Begin(&axis,&goal,now,0,90000)==MECH_COLLISION);
        if(which==1)goal.target_um=50001;
        if(which==2)now+=251;
        if(which==3)guard.zero_valid=0;
        uint32_t result=Axis_Manual(&axis,&goal,now);
        CHECK(result==(which<=1?MECH_OK:which==2?MECH_STALE:MECH_NO_ZERO));
        CHECK(!axis.emitted&&!output_count);
        return 0;
    }
    if(which==9)
    {
        lift+=3200;for(n=0;n<6;n++)Round();
        CHECK(Servo_Seed(&servo,initial,now,1)==MECH_COLLISION);
        CHECK(Servo_SeedManual(&servo,initial,now,1)==MECH_OK&&!output_count);
        return 0;
    }
    CHECK(Servo_Seed(&servo,initial,now,1)==MECH_OK);
    if(which==4||which==5) {lift+=3200;for(n=0;n<6;n++)Round();}
    if(which==6)
    {
        request.channel=2;request.target_mdeg=82000;
        CHECK(Servo_Begin(&servo,&request,now)==MECH_RANGE);
        request.target_mdeg=359001;
        CHECK(Servo_Manual(&servo,&request,now)==MECH_RANGE);
        request.channel=3;request.target_mdeg=60000;
        CHECK(Servo_Manual(&servo,&request,now)==SERVO_HOLD&&!output_count);
        return 0;
    }
    if(which==4||which==5)CHECK(Servo_Begin(&servo,&request,now)==MECH_COLLISION);
    CHECK(Servo_Manual(&servo,&request,now)==MECH_OK&&!output_count);
    if(which==7)
    {
        now+=251;Servo_Poll(&servo,now,1);
        CHECK(servo.state==SERVO_FAILED&&guard.fault&&!output_count);
        return 0;
    }
    if(which==11||which==12||which==13)
    {
        ServoTick();uint32_t sent=output_count;
        if(which==11)now+=251;
        if(which==12)guard.zero_version++;
        if(which==13)Feed(1,0x35,1);
        Servo_Poll(&servo,now,1);
        ServoOutput output;
        CHECK(servo.state==SERVO_FAILED&&!Servo_Take(&servo,&output,now)&&output_count==sent);
        return 0;
    }
    if(which==8)
    {
        Servo_Stop(&servo,SERVO_CANCEL,now);
        for(n=0;n<8;n++)ServoTick();
        CHECK(servo.state==SERVO_CANCELLED&&!output_count);
        return 0;
    }
    for(n=0;n<300 && servo.state!=SERVO_MANUAL_DONE;n++)ServoTick();
    CHECK(servo.state==SERVO_MANUAL_DONE&&output_count&&!Servo_Reference(&servo,1));
    if(which==10)
    {
        ServoEvidence evidence={123,456,1,1,1,now,60000,1,SERVO_MANUAL};
        CHECK(Servo_Confirm(&servo,&evidence,now)==MECH_OK&&Servo_Reference(&servo,1));
        return 0;
    }
    /* 命令完成不是实物确认，等待超过原期限也不将手动空闲误作超时。 */
    for(n=0;n<400;n++)ServoTick();
    CHECK(servo.state==SERVO_MANUAL_DONE&&!guard.fault&&!Servo_Reference(&servo,1));
    CHECK(Servo_Begin(&servo,&request,now)==SERVO_ID);
    request.id=2;request.target_mdeg=30000;
    if(which==4||which==5)CHECK(Servo_Begin(&servo,&request,now)==MECH_COLLISION);
    CHECK(Servo_Manual(&servo,&request,now)==MECH_OK);
    return 0;
}
