#include "joint.h"
#include <stdbool.h>

#define joint_delta .1
void joint_init(joint_t* joint){
  HAL_TIM_PWM_Start(joint->htimer, joint->channel);
  joint_set_angle(joint,0);
}

void joint_set_angle(joint_t* joint, float angle){
    if(angle < joint->min_angle)
        angle = joint->min_angle;
    if(angle > joint->max_angle)
        angle = joint->max_angle;
    uint32_t scaled_angle = angle*(2400-700)/180 + 700;
    *(joint->CCRReg) = scaled_angle;
    joint->current_angle = angle;
}

bool joint_step_angle(joint_t* joint, float target_angle){
    bool ret;
    if(target_angle < joint->min_angle)
        target_angle = joint->min_angle;
    if(target_angle > joint->max_angle)
        target_angle = joint->max_angle;
    float difference = target_angle - joint->current_angle;
    if((difference > 0 ? difference : -difference) < joint_delta){
        return true;
    }
    int direction = difference > 0 ? 1 : -1;
    float new_angle = joint->current_angle + direction;
    if(new_angle >= target_angle && direction > 0){
        new_angle = target_angle;
        ret = false;
    }
    else if(new_angle <= target_angle && direction < 0){
        new_angle = target_angle;
        ret = false;
    }
    else{
        ret = true;
    }
    joint_set_angle(joint, joint->current_angle + direction);
    return ret;
}