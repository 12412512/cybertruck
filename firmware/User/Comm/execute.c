#include "execute.h"
#include "comm.h"
#include "feedback.h"
#include <string.h>

typedef char ConfigSize[(sizeof(ExecConfig) == 52) ? 1 : -1];
typedef char GoalSize[(sizeof(TrackGoal) == 96) ? 1 : -1];
typedef char FaultSize[(sizeof(ExecFault) == 112) ? 1 : -1];

uint8_t Execute_Command(Exec *exec, uint8_t type, const uint8_t *data, uint16_t size, uint8_t *reply,
                        uint16_t *reply_size, uint32_t now, uint32_t can_errors, uint32_t can_rx_sequence)

{
    ExecConfig config;
    TrackGoal goal;
    TrackOutput output;
    OdomPose pose = {0};
    uint32_t fields[11];
    float values[9];
    unsigned index;
    int result = -1;
    if (type == 37 && size == 0)
    {
        memcpy(reply, &exec->first_fault, sizeof(exec->first_fault));
        *reply_size = sizeof(exec->first_fault);
        result = 1;
    }
    else if (type == 16 && size == 2)
    {
        FeedSample sample = {0};
        const ImuSample *imu;
        const MotorSample *motor;
        sample.kind = data[0];
        sample.address = data[1];
        if ((sample.kind == FEED_ANGLE || sample.kind == FEED_GYRO) && !sample.address)
        {
            imu = sample.kind == FEED_ANGLE ? &exec->angle : &exec->gyro;
            sample.stamp = imu->stamp;
            sample.source_epoch = imu->source_epoch;
            for (index = 0; index < 3; index++)
            {
                sample.values[index] = imu->axes[index];
            }
        }
        else if (sample.kind >= FEED_SPEED && sample.kind <= FEED_STATUS)
        {
            for (index = 0; index < 4; index++)
            {
                motor = sample.kind == FEED_SPEED      ? &exec->speed[index]
                        : sample.kind == FEED_POSITION ? &exec->position[index]
                                                       : &exec->status[index];
                if (motor->address == sample.address)
                {
                    sample.stamp = motor->stamp;
                    sample.source_epoch = 1;
                    sample.values[0] = motor->value;
                    break;
                }
            }
        }
        if (Feed_Encode(&sample, now, reply))
        {
            *reply_size = FEED_WIRE_SIZE;
            result = 1;
        }
    }
    else if (type == EXEC_CONFIG && size == sizeof(config) && Frame_Get(data + 16, 4) == KIN_MECANUM_X)
    {
        memcpy(&config, data, sizeof(config));
        result = Exec_Config(exec, &config, now);
    }
    else if (type == EXEC_ARM && size == 0)
    {
        result = Exec_Arm(exec, now);
    }
    else if (type == EXEC_GOAL && size == sizeof(goal))
    {
        memcpy(&goal, data, sizeof(goal));
        result = Exec_Goal(exec, &goal, now);
    }
    else if (type == EXEC_STATE && size == 0)
    {
        Track_Read(&exec->track, &output);
        (void)Odom_Read(&exec->odom, now, &pose);
        fields[0] = exec->mode;
        fields[1] = exec->fault;
        fields[2] = output.id;
        fields[3] = output.state;
        fields[4] = output.fault;
        fields[5] = exec->stop_confirmed;
        fields[6] = now;
        fields[7] = exec->config.version;
        fields[8] = can_errors;
        fields[9] = can_rx_sequence;
        fields[10] = pose.valid;
        values[0] = (float)pose.x_m;
        values[1] = (float)pose.y_m;
        values[2] = (float)pose.yaw_rad;
        values[3] = (float)output.position_error_m;
        values[4] = (float)output.yaw_error_rad;
        for (index = 0; index < 4; index++)
        {
            values[5 + index] = (float)output.rpm[index];
        }
        memcpy(reply, fields, sizeof(fields));
        memcpy(reply + sizeof(fields), values, sizeof(values));
        *reply_size = sizeof(fields) + sizeof(values);
        result = 1;
    }
    return result >= 0 ? COMM_OK : COMM_BUSY;
}
