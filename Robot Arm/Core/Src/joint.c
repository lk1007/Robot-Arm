#include "joint.h"

void joint_init(joint_t* joint){
  HAL_TIM_PWM_Start(joint->htimer, joint->channel);
}

void joint_set_angle(joint_t* joint, int16_t angle){
    if(angle < joint->min_angle)
        angle = joint->min_angle;
    if(angle > joint->max_angle)
        angle = joint->max_angle;
    uint32_t scaled_angle = angle*(2000-1000)/360 + 1000;
    *(joint->CCRReg) = scaled_angle;
    

}