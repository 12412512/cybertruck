/* 合成电机响应及PWM消费者验证并行，完全不访问设备。 */
#define main ExistingAxisMain
#include "mech_axis.c"
#undef main
#include "mech_group.h"
static Servo servo;
static Group group;
static GroupGoal request={1,1,12000,31,{10000,8000,30000,359000,60000},{5,5,120000,120000,120000}};
static unsigned overlap,pwm_while_axis,frames_sent,pwms;
static int64_t plant[2],target[2];
static int64_t Magnitude(int64_t value){return value<0?-value:value;}
static void Setup(void)
{
    int32_t initial[3]={0,180000,98000};
    Init();Servo_Init(&servo,&guard,0);CHECK(!Servo_Seed(&servo,initial,now,1));
    Group_Init(&group,&axis,&servo);plant[0]=target[0]=lift;plant[1]=target[1]=reach;
}
static void Tick(void)
{
    BusFrame frames[2];ServoOutput output;unsigned i;int moving[2]={0};
    now+=20;
    for(i=0;i<2;i++)
    {
        Axis *a=group.axes+i;
        if(a->state==AXIS_RUNNING||a->state==AXIS_SETTLING)
        {
            int64_t step=(int64_t)a->goal.rpm*65536/3000;
            target[i]=a->before_target+a->delta_raw;
            if(plant[i]!=target[i])
            {
                int sign=target[i]>plant[i]?1:-1;
                plant[i]+=sign*(Magnitude(target[i]-plant[i])>step?step:Magnitude(target[i]-plant[i]));
                moving[i]=plant[i]!=target[i]?a->goal.rpm*sign:0;
            }
        }
        Feed(i,0x35,moving[i]);Feed(i,0x36,plant[i]);Feed(i,0x33,target[i]);Feed(i,0x3A,3);
    }
    if(moving[0]&&moving[1])overlap++;
    Mech_Poll(&guard,now,1);
    Group_Poll(&group,now,1);
    if(Group_Take(&group,frames,now))
    {
        MotorReply reply={0};unsigned a=(frames[0].id>>8)-5;
        frames_sent++;Group_Sent(&group,1,now);
        reply.valid=1;reply.address=(uint8_t)(a+5);reply.code=0xFD;reply.result=2;
        reply.tick=now;reply.rx_sequence=sequence++;Group_Ack(&group,&reply,now);
    }
    for(i=0;i<3;i++)if(Group_Pwm(&group,i,&output,now))
    {
        CHECK(output.channel==i+1&&output.pulse_us==Mech_Pulse(output.channel,output.command_mdeg));
        if(moving[0]||moving[1])pwm_while_axis++;
        pwms++;Group_Applied(&group,i,&output,1,now);
    }
}
int main(int argc,char **argv)
{
    unsigned which=argc>1?(unsigned)atoi(argv[1]):0;
    Setup();
    if(which==1){request.target[4]=99000;CHECK(Group_Begin(&group,&request,now)==MECH_RANGE);CHECK(!Group_Take(&group,(BusFrame[2]){{0}},now)&&!pwms&&!frames_sent);return 0;}
    if(which==6){request.mask=4;request.target[2]=0;}
    CHECK(!Group_Begin(&group,&request,now));
    CHECK(!frames_sent&&!pwms);
    if(which==2){now+=251;Group_Poll(&group,now,1);CHECK(group.state==GROUP_FAILED&&guard.stop_required);return 0;}
    if(which==3){Tick();Group_Stop(&group,MECH_MANUAL,now);Tick();CHECK(group.state==GROUP_CANCELLED&&guard.stop_required&&!guard.zero_valid);return 0;}
    if(which==4){guard.fault=MECH_DRIVER;Group_Poll(&group,now,1);CHECK(group.state==GROUP_FAILED);return 0;}
    if(which==5){GroupGoal changed=request;changed.target[0]++;CHECK(Group_Begin(&group,&changed,now)==AXIS_ID);CHECK(!Group_Begin(&group,&request,now));}
    if(which==7){guard.zero_version++;Group_Poll(&group,now,1);CHECK(group.state==GROUP_FAILED);return 0;}
    for(unsigned n=0;n<600&&group.state==GROUP_RUNNING;n++)Tick();
    if(group.state!=GROUP_DONE)fprintf(stderr,"state %u reason %u guard %u a %u/%u s %u/%u/%u\n",group.state,group.reason,guard.fault,group.axes[0].state,group.axes[1].state,group.servos[0].state,group.servos[1].state,group.servos[2].state);
    CHECK(group.state==GROUP_DONE&&guard.zero_valid&&!guard.concurrent_axes);
    if(which==6){CHECK(!frames_sent&&pwms);return 0;}
    CHECK(frames_sent==2&&pwms&&overlap&&pwm_while_axis);
    CHECK(servo.command_mdeg[0]==30000&&servo.command_mdeg[1]==359000&&servo.command_mdeg[2]==60000);
    CHECK(!Servo_Reference(&servo,2));
    for(unsigned n=0;n<6;n++)Tick();
    request.id=2;request.target[0]=request.target[1]=0;request.target[3]=0;
    CHECK(!Group_Begin(&group,&request,now));
    for(unsigned n=0;n<600&&group.state==GROUP_RUNNING;n++)Tick();
    CHECK(group.state==GROUP_DONE&&servo.command_mdeg[1]==0&&frames_sent==4);
    return 0;
}
