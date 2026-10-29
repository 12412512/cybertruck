/* 使用正式舵机状态机与合成反馈；不访问串口、SWD或PWM硬件。 */
#define TUNING_MAIN ExistingTuningMain
#include "mech_tuning.c"
#include <math.h>
#define SERVO_RAD_PER_MDEG 0.000017453292519943295

int main(int argc, char **argv)
{
    unsigned which = argc > 1 ? (unsigned)atoi(argv[1]) : 0, n;
    int32_t target = which == 0 ? 180500 : which == 1 || which == 13 ? 359000 : which == 5 ? 180000 : 0;
    uint32_t rate = which >= 13 ? 100000 : which == 3 ? 0 : which == 4 ? 1000 : 360000;
    ServoGoal request = {1,1,420000,target,2,1,rate};
    ServoOutput output;
    CurvePoint point;
    if (which == 12) now = UINT32_MAX-1000;
    Setup();
    if (which >= 13)
    {
        int32_t initial[3]={0,which==13?0:359000,98000};
        Servo_Init(&servo,&guard,0);
        CHECK(!Servo_SeedManual(&servo,initial,now,1));
    }
    CHECK(!Servo_Manual(&servo,&request,now));
    CHECK(servo.profile.peak_velocity <= (rate ? rate : 45000)*SERVO_RAD_PER_MDEG*1.00001);
    CHECK(servo.profile.peak_acceleration <= 180000*SERVO_RAD_PER_MDEG*1.00001);
    CHECK(servo.profile.jerk <= 720000*SERVO_RAD_PER_MDEG*1.00001);
    CHECK(Servo_Take(&servo,&output,now) && output.command_mdeg==servo.origin_mdeg && !servo.profile_ms);
    Servo_Applied(&servo,output.id,output.sequence,1,now);
    if (which >= 6 && which <= 11)
    {
        if (which == 6) {now+=200;Round();}
        if (which == 7) {guard.stable_started=0;guard.stable_samples=0;}
        if (which == 8) now+=251;
        if (which == 9) guard.fault=MECH_FAULT;
        if (which == 10) Servo_Stop(&servo,SERVO_CANCEL,now);
        Servo_Poll(&servo,now,(uint8_t)(which!=11));
        if (which>=8)
        {
            CHECK(servo.state==SERVO_FAILED || servo.state==SERVO_CANCELLED);
            CHECK(!Servo_Take(&servo,&output,now));
            CHECK(servo.command_mdeg[1]==180000 && !servo.profile_ms);
            return 0;
        }
        if (which==6)
        {
            CHECK(Servo_Take(&servo,&output,now) && servo.pending_ms==20);
            CHECK(output.command_mdeg>179990); /* 延迟不能追赶成原来的7.2°跳角。 */
            Servo_Applied(&servo,output.id,output.sequence,1,now);
        }
        else
        {
            CHECK(!Servo_Take(&servo,&output,now) && !servo.profile_ms);
            for (n=0;n<4;n++) {ServoTick();CHECK(!servo.profile_ms);}
        }
    }
    for (n=0;n<20000 && servo.state!=SERVO_MANUAL_DONE;n++)
    {
        ServoTick();
        CHECK(servo.state!=SERVO_FAILED && servo.state!=SERVO_CANCELLED);
        CHECK(Curve_At(&servo.profile,servo.profile_ms/1000.0,&point));
        CHECK(fabs(point.velocity) <= (rate?rate:45000)*SERVO_RAD_PER_MDEG*1.00001);
        CHECK(fabs(point.acceleration) <= 180000*SERVO_RAD_PER_MDEG*1.00001);
        CHECK(fabs(point.jerk) <= 720000*SERVO_RAD_PER_MDEG*1.00001);
    }
    CHECK(n<20000 && servo.command_mdeg[1]==target && !guard.fault);
    CHECK(!Servo_Reference(&servo,2)); /* 命令完成不冒充真实角度。 */
    if(which==0) CHECK(servo.profile.peak_velocity < 10*3.141592653589793/180);
    printf("smooth case %u PASS: reference %.3fs, peak %.2f deg/s\n",which,
           servo.profile.total,servo.profile.peak_velocity*180/3.141592653589793);
    return 0;
}
