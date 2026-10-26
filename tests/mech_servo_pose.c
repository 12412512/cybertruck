/* 复用单轴合成反馈夹具；正式舵机/姿态模块无硬件IO。 */
#define main AxisFixtureMain
#include "mech_axis.c"
#undef main
#include "mech_pose.h"

static Servo servo;
static Pose pose;
static uint32_t feed_ms, evidence_sequence = 1, outputs, axis_outputs;
static uint8_t auto_confirm = 1, pose_mode;

static void Fresh(void)
{
    Feed(0, 0x35, 0); Feed(0, 0x36, lift); Feed(0, 0x3A, 3); Feed(0, 0x33, lift);
    Feed(1, 0x35, 0); Feed(1, 0x36, reach); Feed(1, 0x3A, 3); Feed(1, 0x33, reach);
    Mech_Poll(&guard, now, 1);
    feed_ms = now;
}
static void Setup(uint8_t simulation)
{
    int32_t initial[3] = {30000, 180000, 98000};
    Init();
    Servo_Init(&servo, &guard, simulation);
    CHECK(servo.output_mask == 0);
    CHECK(Servo_Seed(&servo, initial, now, 1) == MECH_OK && !servo.output_mask);
    Pose_Init(&pose, &axis, &servo);
    feed_ms = now;
}
static ServoEvidence Evidence(uint8_t kind)
{
    ServoEvidence evidence = {0};
    evidence.boot = 123; evidence.session = 456;
    evidence.id = servo.goal.id; evidence.zero_version = 1;
    evidence.channel = servo.goal.channel; evidence.target_mdeg = servo.goal.target_mdeg;
    evidence.kind = kind; evidence.sequence = evidence_sequence++; evidence.captured_ms = now;
    return evidence;
}
static void Tick(void)
{
    BusFrame frames[2];
    ServoOutput output;
    now += 20;
    if ((uint32_t)(now - feed_ms) >= 80) Fresh();
    if (pose_mode) Pose_Poll(&pose, now, 1);
    else Servo_Poll(&servo, now, 1);
    if (Axis_Take(&axis, frames, now))
    {
        MotorReply ack = {0};
        axis_outputs++;
        CHECK(!Servo_Take(&servo, &output, now)); /* 机构按单个子动作执行。 */
        Axis_Sent(&axis, axis.goal.id, 1, now);
        ack.address = axis.goal.axis ? 6 : 5; ack.code = 0xFD; ack.valid = 1; ack.result = 2;
        ack.tick = now; ack.rx_sequence = sequence++;
        Axis_Ack(&axis, &ack, now);
        if (axis.goal.axis) reach += axis.delta_raw; else lift += axis.delta_raw;
    }
    if (Servo_Take(&servo, &output, now))
    {
        int32_t previous = servo.command_mdeg[output.channel - 1];
        outputs++;
        CHECK(llabs((int64_t)output.command_mdeg - previous) <= MECH_SERVO_STEP_MDEG);
        CHECK(output.pulse_us == Mech_Pulse(output.channel, output.command_mdeg));
        CHECK(!Servo_Take(&servo, &(ServoOutput){0}, now));
        Servo_Applied(&servo, output.id, output.sequence, 1, now);
    }
    if (auto_confirm && servo.state == SERVO_COMMAND_DONE)
    {
        ServoEvidence evidence = Evidence(SERVO_MODEL);
        CHECK(Servo_Confirm(&servo, &evidence, now) == MECH_OK);
    }
}
static void ServoGoalStart(uint8_t channel, int32_t target, uint8_t release)
{
    ServoGoal request = {1, 1, 10000, 0, 0, 0, 0};
    request.channel = channel; request.target_mdeg = target; request.release_confirmed = release;
    CHECK(Servo_Begin(&servo, &request, now) == MECH_OK && !outputs);
}
static void UntilCommand(void)
{
    unsigned n;
    auto_confirm = 0;
    for (n = 0; n < 1000 && servo.state != SERVO_COMMAND_DONE; n++) Tick();
    CHECK(servo.state == SERVO_COMMAND_DONE);
}
static PoseGoal RawPose(void)
{
    PoseGoal request = {0};
    request.id = request.zero_version = request.revision = 1;
    request.timeout_ms = 60000;
    request.target[0] = 49000; request.target[1] = 50000;
    request.target[2] = 180000; request.target[3] = 150000; request.target[4] = 98000;
    return request;
}
int main(int argc, char **argv)
{
    unsigned which = argc > 1 ? (unsigned)atoi(argv[1]) : 0, n;
    ServoOutput output;
    PoseGoal request;
    if (which == 11) now = UINT32_MAX - 1000;
    Setup((uint8_t)(which != 6 && which != 20));
    if (which < 16)
    {
        if (which == 0)
        {
            CHECK(!outputs && !Servo_Take(&servo, &output, now));
            CHECK(Servo_Seed(&servo, (int32_t[]){0,180000,0}, now, 1) == SERVO_REFERENCE);
        }
        else if (which == 1 || which == 11)
        {
            ServoGoalStart(2, 83000, 0); UntilCommand();
            CHECK(outputs > 100 && !Servo_Reference(&servo, 2));
            for (n = 0; n < 5; n++) Tick();
            CHECK(servo.state == SERVO_COMMAND_DONE); /* 600ms不变成实物到位。 */
            ServoEvidence evidence = Evidence(SERVO_MODEL);
            CHECK(Servo_Confirm(&servo, &evidence, now) == MECH_OK);
            CHECK(servo.state == SERVO_CONFIRMED && Servo_Reference(&servo, 2));
        }
        else if (which == 2)
        {
            ServoGoalStart(2, 83000, 0);
            CHECK(Servo_Take(&servo, &output, now) && output.command_mdeg == 180000);
            Servo_Applied(&servo, 1, output.sequence, 1, now);
            now += 200; Fresh();
            CHECK(Servo_Take(&servo, &output, now) && output.command_mdeg == 179100);
        }
        else if (which == 3)
        {
            ServoGoalStart(2, 83000, 0);
            CHECK(Servo_Take(&servo, &output, now));
            Servo_Applied(&servo, 99, output.sequence, 1, now);
            CHECK(servo.state == SERVO_OUTPUT && !servo.output_mask);
            now += 50; Servo_Poll(&servo, now, 1);
            CHECK(servo.state == SERVO_FAILED && servo.reason == SERVO_OUTPUT_ERROR && guard.stop_required);
        }
        else if (which == 4)
        {
            ServoGoalStart(3, 0, 1);
            CHECK(servo.hold_possible && Servo_Take(&servo, &output, now));
            Servo_Applied(&servo, output.id, output.sequence, 0, now);
            CHECK(servo.state == SERVO_FAILED && servo.hold_possible && !servo.output_mask);
        }
        else if (which == 5)
        {
            ServoGoalStart(2, 83000, 0); UntilCommand();
            ServoEvidence evidence = Evidence(SERVO_MODEL); evidence.id++;
            CHECK(Servo_Confirm(&servo, &evidence, now) == SERVO_EVIDENCE);
            evidence = Evidence(SERVO_MODEL); evidence.captured_ms -= 251;
            CHECK(Servo_Confirm(&servo, &evidence, now) == SERVO_EVIDENCE);
            evidence = Evidence(SERVO_MODEL); evidence.target_mdeg++;
            CHECK(Servo_Confirm(&servo, &evidence, now) == SERVO_EVIDENCE);
            CHECK(servo.state == SERVO_COMMAND_DONE);
        }
        else if (which == 6)
        {
            ServoGoalStart(2, 83000, 0); UntilCommand();
            ServoEvidence evidence = Evidence(SERVO_MODEL);
            CHECK(Servo_Confirm(&servo, &evidence, now) == SERVO_EVIDENCE);
            evidence = Evidence(SERVO_MANUAL);
            CHECK(Servo_Confirm(&servo, &evidence, now) == MECH_OK);
            CHECK(servo.confirmation_kind == SERVO_MANUAL);
        }
        else if (which == 7)
        {
            ServoGoal request2 = {1,1,10000,0,3,0,0};
            CHECK(servo.hold_possible && Servo_Begin(&servo, &request2, now) == SERVO_HOLD);
            CHECK(!outputs && !servo.goal.id);
        }
        else if (which == 8)
        {
            ServoGoalStart(3, 0, 1); UntilCommand();
            CHECK(servo.hold_possible);
            ServoEvidence evidence = Evidence(SERVO_MODEL);
            CHECK(Servo_Confirm(&servo, &evidence, now) == MECH_OK && !servo.hold_possible);
        }
        else if (which == 9)
        {
            ServoGoalStart(2, 83000, 0); Tick();
            int32_t previous = servo.command_mdeg[1];
            Servo_Stop(&servo, SERVO_CANCEL, now);
            for (n = 0; n < 10; n++) Tick();
            CHECK(servo.state == SERVO_CANCELLED && servo.command_mdeg[1] == previous && servo.hold_possible);
        }
        else if (which == 10)
        {
            ServoGoalStart(2, 83000, 0); now += 251; Servo_Poll(&servo, now, 1);
            CHECK(servo.state == SERVO_FAILED && guard.stop_required && !outputs);
        }
        else if (which == 12)
        {
            ServoGoalStart(2, 83000, 0);
            CHECK(Servo_Begin(&servo, &servo.goal, now) == MECH_OK);
            ServoGoal conflict = servo.goal; conflict.target_mdeg++;
            CHECK(Servo_Begin(&servo, &conflict, now) == SERVO_ID && !outputs);
        }
        else if (which == 13)
        {
            ServoGoalStart(2, 83000, 0); Servo_Poll(&servo, now, 0);
            CHECK(servo.state == SERVO_FAILED && !outputs && guard.stop_required);
        }
        else if (which == 14)
        {
            ServoGoalStart(2, 83000, 0); guard.session++; Servo_Poll(&servo, now, 1);
            CHECK(servo.state == SERVO_FAILED && !outputs);
        }
        else if (which == 15)
        {
            ServoGoalStart(2, 83000, 0); UntilCommand();
            ServoEvidence evidence = Evidence(SERVO_MODEL);
            now = servo.started_ms + servo.goal.timeout_ms; Fresh();
            evidence.captured_ms = now;
            CHECK(Servo_Confirm(&servo, &evidence, now) == SERVO_EVIDENCE && servo.state == SERVO_FAILED);
        }
    }
    else
    {
        request = RawPose(); pose_mode = 1;
        if (which == 17)
        {
            PoseStep steps[8]; uint8_t count;
            CHECK(Pose_Preview(&pose, &request, steps, &count, now) == MECH_OK && count == 3);
            CHECK(steps[0].kind == POSE_SERVO && steps[0].actuator == 1);
            CHECK(steps[1].kind == POSE_AXIS && steps[1].actuator == 0);
            CHECK(!outputs && !axis_outputs && !pose.goal.id && !servo.goal.id);
        }
        else if (which == 18 || which == 19 || which == 20)
        {
            if (which == 18) { request.target[2] = 83000; }
            if (which == 19) { request.target[4] = 0; }
            CHECK(Pose_Begin(&pose, &request, now) == (which == 18 ? MECH_COLLISION : which == 19 ? SERVO_HOLD : SERVO_REFERENCE));
            CHECK(!outputs && !axis_outputs && !pose.goal.id);
        }
        else
        {
            if (which == 21) auto_confirm = 0;
            if (which == 25) request.single_step = 1;
            CHECK(Pose_Begin(&pose, &request, now) == MECH_OK && !outputs);
            CHECK(Pose_Begin(&pose, &request, now) == MECH_OK);
            PoseGoal conflict = request; conflict.revision++;
            CHECK(Pose_Begin(&pose, &conflict, now) == POSE_ID);
            for (n = 0; n < 3000 && pose.state != POSE_COMPLETE; n++)
            {
                Tick();
                if (which == 21 && pose.state == POSE_WAIT_CONFIRM)
                {
                    CHECK(!axis_outputs && pose.index == 0);
                    for (unsigned k = 0; k < 4; k++) Tick();
                    CHECK(pose.state == POSE_WAIT_CONFIRM && !axis_outputs);
                    break;
                }
                if (which == 22 && servo.state == SERVO_SLEWING)
                {
                    unsigned before = outputs;
                    Pose_Cancel(&pose, now);
                    for (unsigned k = 0; k < 10; k++) Tick();
                    CHECK(pose.state == POSE_STOPPING && outputs == before && !pose.stop_confirmed);
                    Axis_StopSent(&axis, 3, now);
                    for (unsigned k = 0; k < 30; k++) Tick();
                    CHECK(pose.state == POSE_CANCELLED && pose.stop_confirmed && servo.hold_possible);
                    break;
                }
                if (which == 23 && pose.child_started)
                {
                    servo.goal.id++; Tick(); CHECK(pose.state == POSE_STOPPING && pose.reason == POSE_SEQUENCE);
                    break;
                }
                if (which == 24 && pose.child_started)
                {
                    guard.zero_version++; Tick(); CHECK(pose.state == POSE_STOPPING);
                    break;
                }
                if (which == 25 && pose.state == POSE_STEP_READY)
                {
                    unsigned before = outputs + axis_outputs;
                    CHECK(Pose_Next(&pose, request.id, request.revision + 1, pose.index, now) == POSE_SEQUENCE);
                    Tick(); CHECK(outputs + axis_outputs == before);
                    uint32_t result = Pose_Next(&pose, request.id, request.revision, pose.index, now);
                    CHECK(result == MECH_OK || result == MECH_UNSTABLE);
                }
                if (which == 26 && pose.child_started)
                {
                    now = pose.started_ms + pose.goal.timeout_ms; Fresh(); Tick();
                    CHECK(pose.state == POSE_STOPPING && pose.reason == POSE_TIMEOUT);
                    break;
                }
            }
            CHECK(n < 3000);
            if (which == 16 || which == 25)
            {
                CHECK(pose.state == POSE_COMPLETE && axis_outputs == 2 && outputs > 100);
                CHECK(servo.confirmation_kind == SERVO_MODEL);
                CHECK(Mech_Position(&guard, 0) > 48000 && Mech_Position(&guard, 1) > 49000);
            }
        }
    }
    printf("servo/pose case %u PASS: synthetic feedback, no IO\n", which);
    return 0;
}
