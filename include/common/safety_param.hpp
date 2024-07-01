//
// Created by zyb on 24-5-1.
//

#ifndef BUPT_DOG_CONTROLLER2_SAFETY_PARAM_HPP
#define BUPT_DOG_CONTROLLER2_SAFETY_PARAM_HPP

#define LEG_NUM 4
#define ONE_LEG_DOF_NUM 3
#define DOF_NUM 12

// 电机角度保护
#define MOTOR0_MAX_POS (40.0f / 180.0f * M_PI)
#define MOTOR0_MIN_POS (-40.0f / 180.0f * M_PI)
#define MOTOR1_MAX_POS (120.0f / 180.0f * M_PI)
#define MOTOR1_MIN_POS (-15.0f / 180.0f * M_PI)
#define MOTOR2_MAX_POS (-30.0f / 180.0f * M_PI)
#define MOTOR2_MIN_POS (-160.0f / 180.0f * M_PI)
// 电机速度保护
#define MOTOR_MAX_VEL (20.0f * M_PI)
// 电机力矩保护
#define MOTOR_MAX_TAU (33.5f * M_PI)

#define BASE_MAX_ROLL (30.0f / 180.0f * M_PI)
#define BASE_MAX_PITCH (45.0f / 180.0f * M_PI)
// 关节标准位置 站立 左前腿三关节
#define JOINT0_STAND_POS (0.0)
#define JOINT1_STAND_POS (45.0 / 180.0 * M_PI)
#define JOINT2_STAND_POS (-90.0 / 180.0 * M_PI)
// 关节标准位置 趴下 左前腿三关节
#define JOINT0_DOWN_POS (19.68 / 180.0 * M_PI)
#define JOINT1_DOWN_POS (61.959 / 180.0 * M_PI)
#define JOINT2_DOWN_POS (-151.641 / 180.0 * M_PI)
// 质心最大期望速度
#define BASE_MAX_CMD_VX (0.8)
#define BASE_MIN_CMD_VX (-0.4)
#define BASE_MAX_CMD_VY (0.2)
#define BASE_MAX_CMD_DYAW (45.0 / 180.0 * M_PI)
#endif //BUPT_DOG_CONTROLLER2_SAFETY_PARAM_HPP
