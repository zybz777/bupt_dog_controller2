/*
 * @Author       : Zybz
 * @Date         : 2024-05-05 19:51:11
 * @LastEditors  : Zybz
 * @LastEditTime : 2024-05-09 17:06:09
 * @FilePath     : /bupt_dog_controller2/include/control/mpc/mpc_param.hpp
 * @Description  : 
 * 
 * Copyright (c) 2024 by BUPT RobotTeam, All Rights Reserved. 
 */
//
// Created by zyb on 24-4-23.
//

#ifndef BUPT_DOG_CONTROLLER2_MPC_PARAM_HPP
#define BUPT_DOG_CONTROLLER2_MPC_PARAM_HPP
#define MPC_FREQUENCY 50
#define HORIZON 50

const double inf = 1e6;
const double mu = 0.5;
const double I_diag[3] = {0.126958, 0.439839, 0.546138};
const double f_min = 10;
const double f_max = 120;
#endif //BUPT_DOG_CONTROLLER2_MPC_PARAM_HPP
