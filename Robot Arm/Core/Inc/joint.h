#ifndef JOINT_H
#define JOINT_H

#include "main.h"

typedef struct{
    TIM_HandleTypeDef* htimer;
    uint32_t * CCRReg;
    
    uint32_t channel;
    uint16_t min_angle;
    uint16_t max_angle;
}joint_t;

void joint_init(joint_t* joint);

void joint_set_angle(joint_t* joint, int16_t angle);

#endif