
#include "arm.h"

void arm_set_angles(arm_t *arm, int16_t base_angle, int16_t shoulder_angle, int16_t elbow_angle, int16_t wrist_angle, int16_t hand_angle)
{
    joint_set_angle(arm->base, base_angle);
    joint_set_angle(arm->shoulder, shoulder_angle);
    joint_set_angle(arm->elbow, elbow_angle);
    joint_set_angle(arm->wrist, wrist_angle);
    joint_set_angle(arm->hand, hand_angle);
}

void arm_init(arm_t *arm)
{
    joint_init(arm->hand);
    joint_init(arm->wrist);
    joint_init(arm->elbow);
    joint_init(arm->shoulder);
    joint_init(arm->base);
}