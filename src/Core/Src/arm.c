
#include "arm.h"

void arm_set_angles(arm_t *arm, float* angles)
{
    joint_set_angle(arm->base, angles[0]);
    joint_set_angle(arm->shoulder, angles[1]);
    joint_set_angle(arm->elbow, angles[2]);
    joint_set_angle(arm->wrist, angles[3]);
    joint_set_angle(arm->hand, angles[4]);
}

void arm_init(arm_t *arm)
{
    joint_init(arm->hand);
    joint_init(arm->wrist);
    joint_init(arm->elbow);
    joint_init(arm->shoulder);
    joint_init(arm->base);
}