#ifndef JOINT_H
#define JOINT_H

#include "main.h"
#include <stdbool.h>

typedef enum {
    CCW=-1,
    CW=1, 
} direction_t;
typedef struct{
    TIM_HandleTypeDef* htimer;
    volatile uint32_t * CCRReg;
    uint32_t channel;
    uint16_t min_angle;
    uint16_t max_angle;
    float current_angle;
    int8_t polarity;
}joint_t;

void joint_init(joint_t* joint);

void joint_set_angle(joint_t* joint, float angle);
bool joint_step_angle(joint_t* joint, float target_angle);

void write_arm_angles();

#endif