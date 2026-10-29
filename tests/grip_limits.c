/* 正式舵机/姿态状态机，核对夹持语义与最大角解耦；无设备IO。 */
#define TUNING_MAIN ExistingTuningMain
#include "mech_tuning.c"
#include "mech_pose.h"

int main(int argc, char **argv)
{
    unsigned which = argc > 1 ? (unsigned)atoi(argv[1]) : 0, n;
    int32_t target = which == 0 ? 98000 : which == 1 ? 99000 : 100000;
    ServoGoal request = {1,1,10000,0,3,0,45000};
    ServoOutput output;
    Setup();
    CHECK(MECH_GRIP_HOLD_MDEG == 98000 && MECH_GRIP_MAX_MDEG == 100000);
    CHECK(Mech_Pulse(3,98000)==1226 && Mech_Pulse(3,99000)==1233 && Mech_Pulse(3,100000)==1241);
    if (which == 3 || which == 4)
    {
        request.target_mdeg = which == 3 ? -1 : 100001;
        CHECK(Servo_Manual(&servo,&request,now)==MECH_RANGE && !servo.output_mask);
        return 0;
    }
    if (which == 5 || which == 6)
    {
        /* 模型初始不假定持料，明确98或100参考都建立保夹语义。 */
        Servo_Init(&servo,&guard,1);
        CHECK(!servo.hold_possible);
        CHECK(!Servo_SeedManual(&servo,(int32_t[]){0,180000,which==5?98000:100000},now,1));
        CHECK(servo.hold_possible && !servo.output_mask);
        return 0;
    }
    if (which == 7)
    {
        request.target_mdeg=97000;
        CHECK(Servo_Manual(&servo,&request,now)==SERVO_HOLD && !servo.output_mask);
        return 0;
    }
    if (which == 8)
    {
        Pose pose; PoseStep steps[8]; uint8_t count;
        PoseGoal goal = {0};
        goal.id=goal.zero_version=goal.revision=1;goal.timeout_ms=30000;
        goal.target[2]=180000;goal.target[4]=100000;
        servo.simulation=1;Pose_Init(&pose,&axis,&servo);
        CHECK(!Pose_Preview(&pose,&goal,steps,&count,now) && count==1);
        CHECK(steps[0].target==100000 && servo.hold_possible);
        goal.target[4]=97000;CHECK(Pose_Preview(&pose,&goal,steps,&count,now)==SERVO_HOLD);
        goal.target[4]=100001;CHECK(Pose_Preview(&pose,&goal,steps,&count,now)==MECH_RANGE);
        return 0;
    }
    request.target_mdeg = target;
    if (which == 9) request.release_confirmed=1;
    CHECK(!Servo_Manual(&servo,&request,now));
    if (which == 10 || which == 11)
    {
        ServoTick();ServoTick();
        int32_t before=servo.command_mdeg[2];
        if (which==10) Servo_Stop(&servo,SERVO_CANCEL,now);
        else {now+=251;Servo_Poll(&servo,now,1);}
        CHECK(!guard.zero_valid && servo.hold_possible);
        CHECK(!Servo_Take(&servo,&output,now) && servo.command_mdeg[2]==before);
        return 0;
    }
    for(n=0;n<100 && servo.state!=SERVO_MANUAL_DONE;n++)ServoTick();
    CHECK(n<100 && servo.command_mdeg[2]==target && servo.output_mask==4 && servo.hold_possible);
    unsigned before=outputs;
    for(n=0;n<50;n++)ServoTick();
    CHECK(outputs==before && servo.command_mdeg[2]==target && servo.output_mask==4);
    CHECK(!Servo_Reference(&servo,3)); /* 持续命令不冒充实际角度或夹紧力。 */
    if(which==9)
    {
        ServoEvidence evidence={123,456,1,1,1,now,100000,3,SERVO_MANUAL};
        CHECK(!Servo_Confirm(&servo,&evidence,now) && servo.hold_possible);
        request.id=2;request.target_mdeg=98000;request.release_confirmed=1;
        CHECK(!Servo_Manual(&servo,&request,now));
        for(n=0;n<100 && servo.state!=SERVO_MANUAL_DONE;n++)ServoTick();
        evidence.id=2;evidence.sequence=2;evidence.target_mdeg=98000;evidence.captured_ms=now;
        CHECK(!Servo_Confirm(&servo,&evidence,now) && servo.hold_possible);
    }
    return 0;
}
