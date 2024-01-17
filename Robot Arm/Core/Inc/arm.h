#ifndef ARM_H
#define ARM_H

#include "main.h"
#include "joint.h"

typedef struct{
    joint_t* base;
    joint_t* shoulder; 
    joint_t* elbow;
    joint_t* wrist;
    joint_t* hand;
}arm_t;

void arm_set_angles(arm_t* arm, int16_t base_angle, int16_t shoulder_angle,int16_t elbow_angle,int16_t wrist_angle,int16_t hand_angle);

void arm_init(arm_t* arm);

#endif