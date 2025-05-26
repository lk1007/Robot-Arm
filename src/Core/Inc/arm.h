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

void arm_set_angles(arm_t *arm, float* angles);

void arm_init(arm_t* arm);

#endif