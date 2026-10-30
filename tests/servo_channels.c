/* 正式执行器通道对照：匀速、S曲线、延迟及故障，不访问设备。 */
#define TUNING_MAIN ExistingTuningMain
#include "mech_tuning.c"

int main(int argc, char **argv)
{
    unsigned which = argc > 1 ? (unsigned)atoi(argv[1]) : 0, n;
    uint8_t channel = which & 1U ? MECH_GRIP_CHANNEL : MECH_TRAY_CHANNEL;
    int32_t initial[3] = {0,180000,98000};
    int32_t target = channel == MECH_TRAY_CHANNEL ? 30000 : 0;
    uint32_t rate = 250000U, started;
    ServoGoal request = {1,1,30000,0,0,1,0};
    ServoOutput output;
    if (which == 4) now = UINT32_MAX - 100;
    Setup();
    if (which == 5) target = 98000; /* 同角度不人为增加运动时间。 */
    if (which == 6) target = 1234; /* 不整除的末步不得越过目标。 */
    if (which == 7) rate = 0; /* 旧协议默认45度/秒。 */
    request.channel = channel; request.target_mdeg = target; request.speed_mdeg_s = rate;
    if (which == 8)
    {
        /* 先执行转臂，再复用对象执行料盘，不能残留前一条曲线。 */
        Move(1,MECH_ARM_CHANNEL,75000,100000);
        request.id = 2;
    }
    CHECK(!Servo_Manual(&servo,&request,now));
    CHECK(servo.profile.total == 0);
    CHECK(Servo_Take(&servo,&output,now));
    CHECK(output.command_mdeg == initial[channel-1]);
    Servo_Applied(&servo,output.id,output.sequence,1,now);
    started = now;
    if (which == 2)
    {
        now += 200; Round();
        CHECK(Servo_Take(&servo,&output,now));
        CHECK(output.command_mdeg == 5000); /* 延迟只走一个20ms步。 */
        Servo_Applied(&servo,output.id,output.sequence,1,now);
    }
    else if (which == 3)
    {
        now += 251; Servo_Poll(&servo,now,1);
        CHECK(servo.state == SERVO_FAILED && guard.fault);
        CHECK(!Servo_Take(&servo,&output,now));
        return 0;
    }
    else if (which == 9)
    {
        now += 20; Round(); CHECK(Servo_Take(&servo,&output,now));
        Servo_Applied(&servo,output.id,output.sequence,0,now);
        CHECK(servo.state == SERVO_FAILED && guard.fault);
        CHECK(servo.command_mdeg[channel-1] == initial[channel-1]);
        return 0;
    }
    for (n=0; n<1000 && servo.state!=SERVO_MANUAL_DONE; n++)
    {
        int32_t previous = servo.command_mdeg[channel-1];
        ServoTick();
        CHECK(servo.state!=SERVO_FAILED && servo.state!=SERVO_CANCELLED);
        if (previous!=target && servo.command_mdeg[channel-1]!=target)
            CHECK(llabs((int64_t)servo.command_mdeg[channel-1]-previous)==(rate?rate:45000)*20/1000);
    }
    CHECK(n<1000 && servo.command_mdeg[channel-1]==target && !guard.fault);
    CHECK(!Servo_Reference(&servo,channel));
    if (which==0) CHECK((uint32_t)(now-started)==720); /* 30°/250°s+600ms。 */
    if (which==1) CHECK((uint32_t)(now-started)==1000); /* 98°，末步夹紧。 */
    if (which==5) CHECK((uint32_t)(now-started)==600);
    return 0;
}
