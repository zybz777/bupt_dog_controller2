//
// Created by zyb on 24-5-1.
//

#ifndef BUPT_DOG_CONTROLLER2_SAFETY_PARAM_HPP
#define BUPT_DOG_CONTROLLER2_SAFETY_PARAM_HPP

#define LEG_NUM 4
#define ONE_LEG_DOF_NUM 3
#define DOF_NUM 12

// 电机角度保护
#define MOTOR0_MAX_POS (30.0f/180.0f*M_PI)
#define MOTOR0_MIN_POS (-30.0f/180.0f*M_PI)
#define MOTOR1_MAX_POS (100.0f/180.0f*M_PI)
#define MOTOR1_MIN_POS (0.0f/180.0f*M_PI)
#define MOTOR2_MAX_POS (-35.0f/180.0f*M_PI)
#define MOTOR2_MIN_POS (-158.0f/180.0f*M_PI)
// 电机速度保护
#define MOTOR_MAX_VEL (20.0f * M_PI)
// 电机力矩保护
#define MOTOR_MAX_TAU (33.5f * M_PI)

#define BASE_MAX_ROLL (30.0f/180.0f*M_PI)
#define BASE_MAX_PITCH (45.0f/180.0f*M_PI)
#endif //BUPT_DOG_CONTROLLER2_SAFETY_PARAM_HPP
