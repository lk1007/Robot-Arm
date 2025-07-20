
#include "arm.h"
#include <stdbool.h>

void arm_set_angles(arm_t *arm, float* angles, int delay_ms)
{
    bool done = false;
    while(!done){
        done |= joint_step_angle(arm->base, angles[0]);
        done |= joint_step_angle(arm->shoulder, angles[1]);
        done |= joint_step_angle(arm->elbow, angles[2]);
        done |= joint_step_angle(arm->wrist, angles[3]);
        done |= joint_step_angle(arm->hand, angles[4]);
        HAL_Delay(delay_ms);
    }
}

void arm_init(arm_t *arm)
{
    joint_init(arm->hand);
    joint_init(arm->wrist);
    joint_init(arm->elbow);
    joint_init(arm->shoulder);
    joint_init(arm->base);
}