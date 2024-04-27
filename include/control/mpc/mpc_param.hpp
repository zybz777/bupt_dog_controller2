//
// Created by zyb on 24-4-23.
//

#ifndef BUPT_DOG_CONTROLLER2_MPC_PARAM_HPP
#define BUPT_DOG_CONTROLLER2_MPC_PARAM_HPP
#define MPC_FREQUENCY 100
#define HORIZON 80

const double inf = 1e6;
const double M = 15.851f; // 不含4条腿是13.895   15.851
//const double M = 13.895f;
//const double M_BODY = 7.863f;
//const double M_LEG = 1.937f;
// 0.037221 0.145629 0.16547 origin
// 0.0309164, 0.360289, 0.375206 13.895kg cal
const double I_diag[3] = {0.037221, 0.360289, 0.375206};
//const double Body_Com[3] = {-0.00580, 0.0, -0.0158041};
const double Body_Com[3] = {-0.00480, 0.0, -0.0158041};
#endif //BUPT_DOG_CONTROLLER2_MPC_PARAM_HPP
